#include "persistent_bus.hpp"
#include "csp_mode.hpp"
#include "motion_pdo_group.hpp"
#include <chrono>
#include <cstring>
#include <thread>
#include <limits>
#include <iostream>
#include <sstream>
namespace robot_maint {
namespace {
std::uint32_t load(const std::uint8_t* p,unsigned n) { std::uint32_t value=0; for(unsigned i=0;i<n;++i)value|=std::uint32_t(p[i])<<(8*i); return value; }
void store(std::uint8_t* p,std::uint32_t value,unsigned n) { for(unsigned i=0;i<n;++i)p[i]=static_cast<std::uint8_t>(value>>(8*i)); }
}
bool PersistentPdoBus::mapping(int slave,std::uint16_t assignment,bool output,Layout& layout) {
    auto read=[&](int id,std::uint16_t index,std::uint8_t sub,auto& value){
        if(engine_.read_sdo(id,index,sub,value))return true;
        std::cerr<<"[启动] S"<<id<<" 读取 PDO 映射失败: 0x"<<std::hex<<index
                 <<":"<<unsigned(sub)<<std::dec<<'\n';
        return false;
    };
    std::uint8_t count=0;
    if(!read(slave,assignment,0,count)||count==0||count>16)return false;
    unsigned bits=0;
    for(unsigned p=1;p<=count;++p) {
        std::uint16_t index=0;std::uint8_t entries=0;
        if(!read(slave,assignment,p,index)||!read(slave,index,0,entries)||entries>64)return false;
        for(unsigned e=1;e<=entries;++e) {
            std::uint32_t value=0;if(!read(slave,index,e,value))return false;
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
    const auto started=std::chrono::steady_clock::now();
    auto progress=[&](const std::string& message){
        const auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-started).count();
        std::cerr<<"[启动 +"<<ms<<"ms] "<<message<<'\n';
    };
    auto bus_error=[&](const std::string& message){
        ecx_readstate(&engine_.context_);
        std::ostringstream detail;detail<<message;
        for(int i=1;i<=engine_.slave_count_;++i) {
            const auto& slave=engine_.context_.slavelist[i];
            detail<<"；S"<<i<<" state=0x"<<std::hex<<slave.state
                  <<" AL=0x"<<slave.ALstatuscode<<std::dec;
            if(slave.ALstatuscode)detail<<" ("<<ec_ALstatuscode2string(slave.ALstatuscode)<<")";
        }
        error=detail.str();progress(error);
    };
    progress("扫描总线并进入 PRE-OP");
    if(!engine_.init()){error="PDO 服务初始化或扫描失败";return false;}
    if(engine_.slave_count_>64){error="PDO 服务最多支持 64 个从站";return false;}
    const int count=engine_.slave_count_;
    progress("扫描完成，从站数量="+std::to_string(count));
    std::vector<int> required;
    if(initial_axes.empty()) {
        for(int i=0;i<count;++i) {
            const auto& slave=engine_.context_.slavelist[i+1];
            if((slave.eep_man==kTongchuanVendorId||slave.eep_man==kEyouVendorId)
               &&(slave.mbx_proto&ECT_MBXPROT_COE))required.push_back(i);
        }
        if(required.empty()){error="未扫描到支持的同川/意优 CoE 伺服";return false;}
    } else if(!resolve_motion_axes(initial_axes,count,required,error))return false;
    layouts_.resize(count);info_.resize(count);sdo_modes_.assign(count,0);
    bool supported=false;
    for(int i=0;i<count;++i) {
        auto& metadata=info_[i];auto& layout=layouts_[i];const auto& slave=engine_.context_.slavelist[i+1];
        metadata.name=slave_to_axis_name(i+1,count);
        if((slave.eep_man!=kTongchuanVendorId&&slave.eep_man!=kEyouVendorId)||!(slave.mbx_proto&ECT_MBXPROT_COE)) {
            metadata.unavailable="不是支持的同川/意优 CoE 伺服";continue;
        }
        progress(metadata.name+" 核验 RxPDO/TxPDO 映射");
        // Discover actual mapping; do not guess offsets or re-map live axes.
        if(!mapping(i+1,0x1c12,true,layout)||!mapping(i+1,0x1c13,false,layout)
           ||layout.cw<0||layout.target<0||layout.sw<0||layout.actual<0) {
            metadata.unavailable="无法核验 PDO 核心字段映射";progress(metadata.name+" "+metadata.unavailable);continue;
        }
        progress(metadata.name+" 请求 CSP 模式，进入 OP 后确认实际模式");
        std::string mode_error;
        if(!request_csp_mode(
            [&](std::int8_t mode){return engine_.write_sdo(i+1,SDO_MODE_OF_OPERATION,0,mode);},mode_error)) {
            metadata.unavailable=mode_error;
            progress(metadata.name+" "+metadata.unavailable);
            // Preserve mailbox abort codes in the log for diagnosis.
            while(ecx_iserror(&engine_.context_))
                std::cerr<<"[SDO] "<<ecx_elist2string(&engine_.context_);
            continue;
        }
        progress(metadata.name+" 读取位置换算比例");
        if(!engine_.get_axis_ratio(i+1,metadata.counts_per_deg)||metadata.counts_per_deg<=0) {
            metadata.unavailable="无法取得位置换算比例";continue;
        }
        progress(metadata.name+" 核验完成");
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
    configure_motion_group(engine_.context_,info_);
    for(int i=0;i<count;++i)if(engine_.context_.slavelist[i+1].group!=kMotionPdoGroup)
        progress(info_[i].name+" 排除在运动 PDO 组外："+info_[i].unavailable);
    progress("仅映射受支持伺服的运动 PDO 组，使用 LRD/LWR 分离读写");
    const int bytes=ecx_config_map_group(&engine_.context_,image_.data(),kMotionPdoGroup);
    if(bytes<=0||static_cast<std::size_t>(bytes)>image_.size()){error="PDO 映射失败或超出容量";return false;}
    mapped_=true;
    for(int i=0;i<count;++i) {
        if(!info_[i].unavailable.empty())continue;
        const auto& s=engine_.context_.slavelist[i+1];const auto& l=layouts_[i];
        if(!s.outputs||!s.inputs||s.Obytes*8U<l.rx_bits||s.Ibytes*8U<l.tx_bits){error=info_[i].name+" PDO 缓冲区长度不匹配";info_[i].unavailable=error;return false;}
    }
    expected_=engine_.context_.grouplist[kMotionPdoGroup].outputsWKC*2+engine_.context_.grouplist[kMotionPdoGroup].inputsWKC;
    progress("运动组期望 WKC="+std::to_string(expected_)+" outputsWKC="
             +std::to_string(engine_.context_.grouplist[kMotionPdoGroup].outputsWKC)
             +" inputsWKC="+std::to_string(engine_.context_.grouplist[kMotionPdoGroup].inputsWKC));
    if(expected_<=0){error="无有效 PDO 工作计数";return false;}
    progress("等待运动 PDO 组进入 SAFE-OP");
    if(!motion_group_matches(engine_.context_,EC_STATE_SAFE_OP,[&](int slave,std::uint16_t state){
        return ecx_statecheck(&engine_.context_,slave,state,EC_TIMEOUTSTATE*2);
    })) {
        bus_error("运动 PDO 组无法进入 SAFE-OP");return false;
    }
    progress("读取各轴初始位置");
    std::vector<MotionOutput> outputs(count);std::vector<MotionFeedback> feedback;
    for(int i=0;i<count;++i)if(info_[i].unavailable.empty()) {
        if(!engine_.read_sdo(i+1,SDO_ACTUAL_POSITION,0,outputs[i].target)){error=info_[i].name+" 读取初始位置失败";return false;}
    }
    cycle(outputs,feedback);
    for(int i=0;i<count;++i)if(feedback[i].valid)outputs[i].target=feedback[i].position;
    progress("等待运动 PDO 组进入 OP 并建立 PDO 通信（保持 Shutdown）");
    request_motion_group_state(engine_.context_,EC_STATE_OPERATIONAL,[&](int slave){
        ecx_writestate(&engine_.context_,slave);
    });
    bool operational=false;
    for(int n=0;n<1000;++n) {
        const bool valid=cycle(outputs,feedback);
        if(motion_group_matches(engine_.context_,EC_STATE_OPERATIONAL,[&](int slave,std::uint16_t state){
            return ecx_statecheck(&engine_.context_,slave,state,1000);
        })&&valid) {
            operational=true;break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if(!operational){bus_error("运动 PDO 组无法进入 OP 或 PDO 反馈无效");return false;}
    // Confirm SDO-only mode displays AFTER OP. Do not invent mode=8 in cycle().
    // This runs before the listener exists, while every controlword remains 6.
    for(int i=0;i<count;++i) {
        if(!info_[i].unavailable.empty()||layouts_[i].display>=0)continue;
        progress(info_[i].name+" 在 OP 下通过 SDO 确认 0x6061:00");
        std::string mode_error;bool communication_ok=true;
        const bool confirmed=confirm_csp_mode(
            [&](std::int8_t& display){return engine_.read_sdo(i+1,SDO_MODE_DISPLAY,0,display);},
            [&]{
                for(int n=0;n<50;++n) {
                    if(!cycle(outputs,feedback)){communication_ok=false;return false;}
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
                return true;
            },mode_error);
        if(!communication_ok){error=info_[i].name+": "+mode_error;return false;}
        if(confirmed)sdo_modes_[i]=8;
        else {
            info_[i].unavailable=mode_error;
            progress(info_[i].name+" "+mode_error);
            while(ecx_iserror(&engine_.context_))
                std::cerr<<"[SDO] "<<ecx_elist2string(&engine_.context_);
        }
    }
    for(int i:required)if(!info_[i].unavailable.empty()) {
        if(!error.empty())error+="；";
        error+=info_[i].name+": "+info_[i].unavailable;
    }
    if(!error.empty())return false;
    // PDO mode displays may also take several cycles to settle after OP.
    for(int n=0;n<1000;++n) {
        bool ready=cycle(outputs,feedback);
        for(int i:required)ready=ready&&feedback[i].valid&&feedback[i].mode==8;
        if(ready){progress("PDO 总线和目标轴 CSP 模式确认完成");return true;}
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    error="OP 下目标轴 CSP 模式或 PDO 反馈未就绪";
    for(int i:required)error+="；"+info_[i].name+" mode="+std::to_string(int(feedback[i].mode))
                               +" valid="+std::to_string(feedback[i].valid);
    return false;
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
    const auto started=std::chrono::steady_clock::now();
    auto clock=[] {return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();};
    last_exchange_=bounded_pdo_exchange(expected_,clock,[&](int timeout_us){
        const int sent=ecx_send_processdata_group(&engine_.context_,kMotionPdoGroup);
        auto result=receive_pdo_frames(engine_.context_,timeout_us);
        result.sent=sent>0;return result;
    });
    last_wkc_=last_exchange_.last.total;
    last_cycle_us_=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-started).count();
    const bool healthy=last_exchange_.healthy;
    if(healthy&&last_exchange_.attempts>1)++recovered_cycles_;
    for(std::size_t i=0;i<info_.size();++i) {
        if(!info_[i].unavailable.empty())continue;
        const auto* p=engine_.context_.slavelist[i+1].inputs;const auto& l=layouts_[i];
        feedback[i]={healthy,static_cast<std::uint16_t>(load(p+l.sw,2)),static_cast<std::int32_t>(load(p+l.actual,4)),
                     (l.display<0?sdo_modes_[i]:static_cast<std::int8_t>(p[l.display]))};
    }
    return healthy;
}
std::string PersistentPdoBus::diagnostics() const {
    std::ostringstream out;
    out<<"WKC="<<last_wkc_<<" expected="<<expected_<<" read="<<last_exchange_.last.input
       <<" write="<<last_exchange_.last.output<<" missing="<<last_exchange_.last.missing
       <<" attempts="<<last_exchange_.attempts<<" first_WKC="<<last_exchange_.first.total
       <<" first_read="<<last_exchange_.first.input<<" first_write="<<last_exchange_.first.output
       <<" first_missing="<<last_exchange_.first.missing<<" recovered_cycles="<<recovered_cycles_
       <<" first_no_frame="<<last_exchange_.first.no_frame<<" first_cmd_mismatch="<<last_exchange_.first.command_mismatch
       <<" first_expected_cmd="<<last_exchange_.first.expected_command<<" first_received_cmd="<<last_exchange_.first.received_command
       <<" no_frame="<<last_exchange_.last.no_frame<<" cmd_mismatch="<<last_exchange_.last.command_mismatch
       <<" expected_cmd="<<last_exchange_.last.expected_command<<" received_cmd="<<last_exchange_.last.received_command;
    return out.str();
}
void PersistentPdoBus::log_states() {
    if(!mapped_)return;
    ecx_readstate(&engine_.context_);
    for(std::size_t i=0;i<info_.size();++i) {
        const auto& slave=engine_.context_.slavelist[i+1];
        if(slave.group!=kMotionPdoGroup)continue;
        std::cerr<<"[退出状态] "<<info_[i].name<<" state=0x"<<std::hex<<slave.state
                 <<" AL=0x"<<slave.ALstatuscode<<std::dec;
        if(slave.ALstatuscode)std::cerr<<" "<<ec_ALstatuscode2string(slave.ALstatuscode);
        std::cerr<<'\n';
    }
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
    log_states();
    request_motion_group_state(engine_.context_,EC_STATE_PRE_OP,[&](int slave){
        ecx_writestate(&engine_.context_,slave);
    });
    motion_group_matches(engine_.context_,EC_STATE_PRE_OP,[&](int slave,std::uint16_t state){
        return ecx_statecheck(&engine_.context_,slave,state,EC_TIMEOUTSTATE);
    });
    mapped_=false;
    engine_.close();
}
}
