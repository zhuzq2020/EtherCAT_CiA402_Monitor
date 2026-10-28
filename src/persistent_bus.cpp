#include "persistent_bus.hpp"
#include <chrono>
#include <cstring>
#include <thread>
#include <limits>
namespace robot_maint {
namespace {
std::uint32_t load(const std::uint8_t* p,unsigned n) { std::uint32_t value=0; for(unsigned i=0;i<n;++i)value|=std::uint32_t(p[i])<<(8*i); return value; }
void store(std::uint8_t* p,std::uint32_t value,unsigned n) { for(unsigned i=0;i<n;++i)p[i]=static_cast<std::uint8_t>(value>>(8*i)); }
}
bool PersistentPdoBus::mapping(int slave,std::uint16_t assignment,bool output,Layout& layout) {
    std::uint8_t count=0;
    if(!engine_.read_sdo(slave,assignment,0,count)||count==0||count>16)return false;
    unsigned bits=0;
    for(unsigned p=1;p<=count;++p) {
        std::uint16_t index=0;std::uint8_t entries=0;
        if(!engine_.read_sdo(slave,assignment,p,index)||!engine_.read_sdo(slave,index,0,entries)||entries>64)return false;
        for(unsigned e=1;e<=entries;++e) {
            std::uint32_t value=0;if(!engine_.read_sdo(slave,index,e,value))return false;
            const auto object=static_cast<std::uint16_t>(value>>16);const unsigned sub=(value>>8)&255,width=value&255;
            if(!width)return false;
            int* field=nullptr;unsigned expected=0;
            if(sub==0) {
                if(output&&object==0x6040){field=&layout.cw;expected=16;}
                if(output&&object==0x607a){field=&layout.target;expected=32;}
                if(output&&object==0x6060){field=&layout.mode;expected=8;}
                if(!output&&object==0x6041){field=&layout.sw;expected=16;}
                if(!output&&object==0x6064){field=&layout.actual;expected=32;}
                if(!output&&object==0x6061){field=&layout.display;expected=8;}
            }
            if(field){if(bits%8||width!=expected||*field>=0)return false;*field=static_cast<int>(bits/8);}
            bits+=width;
        }
    }
    if(output)layout.rx_bits=bits;else layout.tx_bits=bits;
    return true;
}
bool PersistentPdoBus::prepare(const std::vector<std::string>& initial_axes,std::string& error) {
    if(!engine_.init()){error="PDO 服务初始化或扫描失败";return false;}
    if(engine_.slave_count_>64){error="PDO 服务最多支持 64 个从站";return false;}
    const int count=engine_.slave_count_;
    std::vector<int> required;
    if(!resolve_motion_axes(initial_axes,count,required,error))return false;
    layouts_.resize(count);info_.resize(count);
    bool supported=false;
    for(int i=0;i<count;++i) {
        auto& metadata=info_[i];auto& layout=layouts_[i];const auto& slave=engine_.context_.slavelist[i+1];
        metadata.name=slave_to_axis_name(i+1,count);
        if((slave.eep_man!=kTongchuanVendorId&&slave.eep_man!=kEyouVendorId)||!(slave.mbx_proto&ECT_MBXPROT_COE)) {
            metadata.unavailable="不是支持的同川/意优 CoE 伺服";continue;
        }
        // Discover actual mapping; do not guess offsets or re-map live axes.
        if(!mapping(i+1,0x1c12,true,layout)||!mapping(i+1,0x1c13,false,layout)
           ||layout.cw<0||layout.target<0||layout.sw<0||layout.actual<0) {
            metadata.unavailable="无法核验 PDO 核心字段映射";continue;
        }
        const std::int8_t mode=8;std::int8_t display=0;
        if(!engine_.write_sdo(i+1,SDO_MODE_OF_OPERATION,0,mode)
           ||(layout.display<0&&(!engine_.read_sdo(i+1,SDO_MODE_DISPLAY,0,display)||display!=8))) {
            metadata.unavailable="无法确认 CSP 模式";continue;
        }
        if(!engine_.get_axis_ratio(i+1,metadata.counts_per_deg)||metadata.counts_per_deg<=0) {
            metadata.unavailable="无法取得位置换算比例";continue;
        }
        supported=true;
    }
    if(!supported){error="没有可核验 PDO 映射和 CSP 模式的伺服轴";return false;}
    for(int i:required)if(!info_[i].unavailable.empty()) {
        if(!error.empty())error+="；";
        error+=info_[i].name+": "+info_[i].unavailable;
    }
    if(!error.empty())return false;
    // Upper bound: at most 64 slaves, each side's byte count is uint16.
    image_.assign(64U*65536U*2U,0);
    const int bytes=ecx_config_map_group(&engine_.context_,image_.data(),0);
    if(bytes<=0||static_cast<std::size_t>(bytes)>image_.size()){error="PDO 映射失败或超出容量";return false;}
    mapped_=true;
    for(int i=0;i<count;++i) {
        if(!info_[i].unavailable.empty())continue;
        const auto& s=engine_.context_.slavelist[i+1];const auto& l=layouts_[i];
        if(!s.outputs||!s.inputs||s.Obytes*8U<l.rx_bits||s.Ibytes*8U<l.tx_bits){error=info_[i].name+" PDO 缓冲区长度不匹配";info_[i].unavailable=error;return false;}
    }
    expected_=engine_.context_.grouplist[0].outputsWKC*2+engine_.context_.grouplist[0].inputsWKC;
    if(expected_<=0){error="无有效 PDO 工作计数";return false;}
    if(ecx_statecheck(&engine_.context_,0,EC_STATE_SAFE_OP,EC_TIMEOUTSTATE*2)!=EC_STATE_SAFE_OP) {
        error="总线无法进入 SAFE-OP";return false;
    }
    std::vector<MotionOutput> outputs(count);std::vector<MotionFeedback> feedback;
    for(int i=0;i<count;++i)if(info_[i].unavailable.empty()) {
        if(!engine_.read_sdo(i+1,SDO_ACTUAL_POSITION,0,outputs[i].target)){error=info_[i].name+" 读取初始位置失败";return false;}
    }
    cycle(outputs,feedback);
    for(int i=0;i<count;++i)if(feedback[i].valid)outputs[i].target=feedback[i].position;
    engine_.context_.slavelist[0].state=EC_STATE_OPERATIONAL;ecx_writestate(&engine_.context_,0);
    for(int n=0;n<1000;++n) {
        const bool valid=cycle(outputs,feedback);
        bool modes_ready=valid;
        for(int i:required)modes_ready=modes_ready&&feedback[i].valid&&feedback[i].mode==8;
        if(ecx_statecheck(&engine_.context_,0,EC_STATE_OPERATIONAL,1000)==EC_STATE_OPERATIONAL&&modes_ready)return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    error="总线无法进入 OP 或 PDO 反馈无效";return false;
}
bool PersistentPdoBus::cycle(const std::vector<MotionOutput>& outputs,std::vector<MotionFeedback>& feedback) {
    feedback.assign(info_.size(),{});
    if(!mapped_||outputs.size()!=info_.size())return false;
    for(std::size_t i=0;i<info_.size();++i) {
        if(!info_[i].unavailable.empty())continue;
        auto* p=engine_.context_.slavelist[i+1].outputs;const auto& l=layouts_[i];
        store(p+l.cw,outputs[i].control,2);store(p+l.target,static_cast<std::uint32_t>(outputs[i].target),4);
        if(l.mode>=0)p[l.mode]=8;
    }
    ecx_send_processdata(&engine_.context_);
    const int wkc=ecx_receive_processdata(&engine_.context_,1000);
    const bool healthy=wkc>=expected_;
    for(std::size_t i=0;i<info_.size();++i) {
        if(!info_[i].unavailable.empty())continue;
        const auto* p=engine_.context_.slavelist[i+1].inputs;const auto& l=layouts_[i];
        feedback[i]={healthy,static_cast<std::uint16_t>(load(p+l.sw,2)),static_cast<std::int32_t>(load(p+l.actual,4)),
                     static_cast<std::int8_t>(l.display<0?8:p[l.display])};
    }
    return healthy;
}
PersistentPdoBus::~PersistentPdoBus() { shutdown(); }
void PersistentPdoBus::shutdown() {
    if(!mapped_)return;
    std::vector<MotionOutput> outputs(info_.size());std::vector<MotionFeedback> feedback;
    for(std::size_t i=0;i<info_.size();++i)if(info_[i].unavailable.empty()) {
        const auto* p=engine_.context_.slavelist[i+1].inputs;
        if(p)outputs[i].target=static_cast<std::int32_t>(load(p+layouts_[i].actual,4));
    }
    for(int n=0;n<50;++n){cycle(outputs,feedback);std::this_thread::sleep_for(std::chrono::milliseconds(2));}
    engine_.context_.slavelist[0].state=EC_STATE_PRE_OP;ecx_writestate(&engine_.context_,0);
    ecx_statecheck(&engine_.context_,0,EC_STATE_PRE_OP,EC_TIMEOUTSTATE);
    mapped_=false;
    engine_.close();
}
}
