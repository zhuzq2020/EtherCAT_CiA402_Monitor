extern "C" {
#include <soem/soem.h>
}
#include "pdo_exchange.hpp"
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
namespace {
std::vector<int> commands;
std::vector<std::vector<uint8>> writes;
int input_wkc=14,output_wkc=14,combined_wkc=42;
void check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
}
// Exercise SOEM's real packet construction and receive accounting without NIC I/O.
extern "C" uint8 __wrap_ecx_getindex(ecx_portt* port){return port->lastidx++%EC_MAXBUF;}
extern "C" void __wrap_ecx_setbufstat(ecx_portt*,uint8,int){}
extern "C" int __wrap_ecx_outframe_red(ecx_portt* port,uint8 idx){
    const auto* datagram=reinterpret_cast<const ec_comt*>(port->txbuf[idx]+ETH_HEADERSIZE);
    commands.push_back(datagram->command);
    if(datagram->command==EC_CMD_LWR) {
        const auto* payload=port->txbuf[idx]+ETH_HEADERSIZE+EC_HEADERSIZE;
        writes.emplace_back(payload,payload+etohs(datagram->dlength));
    }
    std::memcpy(port->rxbuf[idx],port->txbuf[idx]+ETH_HEADERSIZE,port->txbuflength[idx]-ETH_HEADERSIZE);
    if(datagram->command==EC_CMD_LRD)
        std::memset(port->rxbuf[idx]+EC_HEADERSIZE,0x5a,etohs(datagram->dlength));
    return port->txbuflength[idx];
}
extern "C" int __wrap_ecx_waitinframe(ecx_portt* port,uint8 idx,int){
    switch(port->rxbuf[idx][EC_CMDOFFSET]) {
        case EC_CMD_LRD:return input_wkc;
        case EC_CMD_LWR:return output_wkc;
        case EC_CMD_LRW:return combined_wkc;
        default:throw std::runtime_error("unexpected EtherCAT command");
    }
}
int main(){try{
    auto c=std::make_unique<ecx_contextt>();
    std::vector<uint8> image(1024,0x33);
    auto& group=c->grouplist[1];
    group.outputs=image.data();group.Obytes=140;
    group.inputs=image.data()+140;group.Ibytes=280;
    group.mbxstatus=image.data()+420;group.mbxstatuslength=14;
    group.nsegments=1;group.IOsegment[0]=434;group.Isegment=0;group.Ioffset=140;
    group.outputsWKC=14;group.inputsWKC=14;
    const int expected=2*group.outputsWKC+group.inputsWKC;
    group.blockLRW=14;
    robot_maint::PdoFrameResult frames;
    auto cycle=[&]{commands.clear();check(ecx_send_processdata_group(c.get(),1)>0,"send succeeds");
        frames=robot_maint::receive_pdo_frames(*c,1000);frames.sent=true;return frames.total;};
    check(cycle()==expected,"separate datagrams retain expected WKC=42");
    check(frames.input==14&&frames.output==14&&frames.missing==0,"read and write counters retained independently");
    check(commands==std::vector<int>({EC_CMD_LRD,EC_CMD_LWR}),"actual SOEM sends LRD and LWR, no LRW");
    check(image[0]==0x33&&image[139]==0x33,"receiving inputs preserves held output bytes");
    check(image[140]==0x5a&&image[419]==0x5a&&image[433]==0x5a,"inputs and mailbox status reach process image");
    output_wkc=0;
    check(cycle()==14&&14<expected,"missing all output acknowledgements remains a failure");
    check(frames.input==14&&frames.output==0&&frames.missing==0,"zero write WKC differs from a lost frame");
    output_wkc=EC_NOFRAME;
    check(cycle()==14&&frames.missing==1,"lost output frame is reported separately");
    output_wkc=14;input_wkc=0;
    check(cycle()==28&&28<expected,"missing input acknowledgements remains a failure");
    input_wkc=14;output_wkc=13;
    check(cycle()==40&&40<expected,"one missing output acknowledgement remains a failure");
    group.blockLRW=0;combined_wkc=14;
    check(cycle()==14&&commands==std::vector<int>({EC_CMD_LRW}),"observed partial LRW WKC is never normalized to healthy");
    using robot_maint::PdoFrameResult;
    using robot_maint::bounded_pdo_exchange;
    std::int64_t now=0;int attempts=0;
    auto clock=[&]{return now;};
    auto full=PdoFrameResult{42,14,14,0,true};
    auto partial=PdoFrameResult{14,14,0,0,true};
    auto result=bounded_pdo_exchange(42,clock,[&](int timeout){
        check(timeout<=1000,"each attempt has bounded wait");++attempts;now+=500;
        return attempts==1?partial:full;
    });
    check(result.healthy&&result.attempts==2&&result.first.total==14&&result.last.total==42,"retry requires complete new exchange");
    now=0;attempts=0;
    result=bounded_pdo_exchange(42,clock,[&](int){++attempts;now+=500;return attempts==1?partial:PdoFrameResult{28,0,14,0,true};});
    check(!result.healthy&&attempts==2,"partial read and partial write from separate attempts never combine");
    now=0;attempts=0;
    result=bounded_pdo_exchange(42,clock,[&](int){++attempts;now+=2100;return full;});
    check(!result.healthy&&attempts==1,"full response after cycle deadline cannot advance motion");
    now=0;attempts=0;
    result=bounded_pdo_exchange(42,clock,[&](int timeout){++attempts;
        if(attempts==1){now+=1500;return partial;}
        check(timeout==500,"retry wait constrained by remaining cycle budget");now+=400;return full;
    });
    check(result.healthy&&attempts==2,"recovery within remaining cycle budget");
    now=0;attempts=0;
    result=bounded_pdo_exchange(42,clock,[&](int){++attempts;now+=500;return full;});
    check(result.healthy&&attempts==1,"healthy cycle never retransmits");
    now=0;attempts=0;
    result=bounded_pdo_exchange(42,clock,[&](int){++attempts;now+=500;auto missing=full;missing.missing=1;return missing;});
    check(!result.healthy&&attempts==2,"missing frame fails even if aggregate WKC is high");
    group.blockLRW=14;input_wkc=14;now=0;attempts=0;writes.clear();
    result=bounded_pdo_exchange(42,clock,[&](int){
        output_wkc=++attempts==1?0:14;cycle();now+=500;return frames;
    });
    check(result.healthy&&writes.size()==2&&writes[0]==writes[1],"actual retry repeats identical output payload");
    check(result.first.input==14&&result.first.output==0&&result.last.input==14&&result.last.output==14,"retry retains separate attempt diagnostics");
    std::cout<<"SOEM LRD/LWR packets, input delivery and strict partial-WKC checks passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
