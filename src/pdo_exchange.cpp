#include "pdo_exchange.hpp"
#include <chrono>
#include <cstring>

namespace robot_maint {
PdoFrameResult receive_pdo_frames(ecx_contextt& context,int timeout_us) {
    PdoFrameResult result;int total=0;bool received=false;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::microseconds(timeout_us);
    // Consume the same pending-frame stack as SOEM's receive function, while
    // preserving per-direction counters and a shared deadline for all frames.
    while(context.idxstack.pulled<context.idxstack.pushed) {
        const int pos=context.idxstack.pulled++;
        const auto idx=context.idxstack.idx[pos];
        const auto command=context.port.txbuf[idx][ETH_HEADERSIZE+EC_CMDOFFSET];
        const auto remaining=std::chrono::duration_cast<std::chrono::microseconds>(deadline-std::chrono::steady_clock::now()).count();
        int wkc=ecx_waitinframe(&context.port,idx,static_cast<int>(std::max<std::int64_t>(0,remaining)));
        const auto* data=context.port.rxbuf[idx];
        if(wkc>EC_NOFRAME&&data[EC_CMDOFFSET]==command) {
            received=true;
            if(context.idxstack.dcoffset[pos]>0) {
                std::uint16_t wire_wkc=0;std::int64_t dc=0;
                std::memcpy(&wire_wkc,data+EC_HEADERSIZE+context.idxstack.length[pos],sizeof(wire_wkc));
                wkc=etohs(wire_wkc);
                std::memcpy(&dc,data+context.idxstack.dcoffset[pos],sizeof(dc));
                context.DCtime=etohll(dc);
            }
            if(command==EC_CMD_LRD||command==EC_CMD_LRW) {
                std::memcpy(context.idxstack.data[pos],data+EC_HEADERSIZE,context.idxstack.length[pos]);
                total+=wkc;result.input+=wkc;
            } else if(command==EC_CMD_LWR) {total+=2*wkc;result.output+=wkc;}
            else ++result.missing;
        } else {
            ++result.missing;
            if(wkc<=EC_NOFRAME)++result.no_frame;
            else {++result.command_mismatch;result.expected_command=command;
                  result.received_command=data[EC_CMDOFFSET];result.received_wkc=wkc;}
        }
        ecx_setbufstat(&context.port,idx,EC_BUF_EMPTY);
    }
    context.idxstack.pushed=0;context.idxstack.pulled=0;
    result.total=received?total:EC_NOFRAME;
    return result;
}
}
