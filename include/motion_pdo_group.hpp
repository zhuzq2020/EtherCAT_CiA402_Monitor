#pragma once
#include "motion_control.hpp"
extern "C" {
#include <soem/soem.h>
}

namespace robot_maint {
// SOEM group 0 maps ALL slaves. Use a nonzero group to exclude unrelated I/O.
constexpr std::uint8_t kMotionPdoGroup=1;
inline void configure_motion_group(ecx_contextt& context,const std::vector<MotionAxisInfo>& info) {
    for(int i=1;i<=context.slavecount;++i) {
        const bool included=info.at(i-1).unavailable.empty();
        context.slavelist[i].group=included?kMotionPdoGroup:0;
        // Use explicit LRD/LWR for servo compatibility. SOEM still accounts for
        // each output acknowledgement twice in the combined WKC (2*O + I).
        if(included)context.slavelist[i].blockLRW=1;
    }
    // This freshly initialized context maps only one group. Start at logical 0
    // so SOEM mailbox-status pointers and the process image share the same base.
    context.grouplist[kMotionPdoGroup].logstartaddr=0;
}
template<class Check>
bool motion_group_matches(ecx_contextt& context,std::uint16_t state,Check check) {
    bool found=false,ready=true;
    for(int i=1;i<=context.slavecount;++i)if(context.slavelist[i].group==kMotionPdoGroup) {
        found=true;
        if(check(i,state)!=state)ready=false;
    }
    return found&&ready;
}
template<class Write>
void request_motion_group_state(ecx_contextt& context,std::uint16_t state,Write write) {
    for(int i=1;i<=context.slavecount;++i)if(context.slavelist[i].group==kMotionPdoGroup) {
        context.slavelist[i].state=state;
        write(i);
    }
}
}
