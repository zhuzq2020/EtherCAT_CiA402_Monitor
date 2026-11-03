#include "motion_pdo_group.hpp"
#include <iostream>
#include <memory>
#include <stdexcept>
using namespace robot_maint;
void check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(){try{
    auto context=std::make_unique<ecx_contextt>();context->slavecount=17;
    std::vector<MotionAxisInfo> info(17);
    for(int i=0;i<17;++i) {
        info[i].name="S"+std::to_string(i+1);
        context->slavelist[i+1].state=EC_STATE_PRE_OP;
    }
    for(int i:{0,8,16})info[i].unavailable="unsupported device";
    context->grouplist[1].logstartaddr=65536;
    configure_motion_group(*context,info);
    check(context->grouplist[1].logstartaddr==0,"single mapped group shares image and mailbox base");
    for(int i:{2,3,4,5,6,7,8,10,11,12,13,14,15,16})
        check(context->slavelist[i].blockLRW==1,"motion group uses separate read/write datagrams");
    for(int i:{1,9,17})check(context->slavelist[i].blockLRW==0,"excluded devices' transfer settings untouched");
    check(context->slavelist[9].group!=kMotionPdoGroup,"S9 excluded from servo mappings");
    std::vector<int> requested;
    request_motion_group_state(*context,EC_STATE_SAFE_OP,[&](int slave){requested.push_back(slave);});
    check(requested==std::vector<int>({2,3,4,5,6,7,8,10,11,12,13,14,15,16}),"only eligible servos receive state requests");
    for(int i:{1,9,17})check(context->slavelist[i].state==EC_STATE_PRE_OP,"unrelated devices remain PRE-OP");
    context->slavelist[9].state=0x12;context->slavelist[9].ALstatuscode=0x1e;
    int checked=0;
    auto read=[&](int slave,std::uint16_t){++checked;check(slave!=0&&slave!=9,"never check broadcast or failed S9 for servo readiness");return context->slavelist[slave].state;};
    check(motion_group_matches(*context,EC_STATE_SAFE_OP,read)&&checked==14,"S9 invalid input configuration does not block servo group");
    context->slavelist[3].state=0x14;checked=0;
    check(!motion_group_matches(*context,EC_STATE_SAFE_OP,read)&&checked==14,"error in any participating servo blocks startup");
    request_motion_group_state(*context,EC_STATE_OPERATIONAL,[](int){});
    check(motion_group_matches(*context,EC_STATE_OPERATIONAL,read),"servo group reaches OP independently");
    request_motion_group_state(*context,EC_STATE_PRE_OP,[](int){});
    check(context->slavelist[9].state==0x12,"shutdown does not broadcast to excluded devices");
    for(auto& axis:info)axis.unavailable="unsupported";
    configure_motion_group(*context,info);
    check(!motion_group_matches(*context,EC_STATE_OPERATIONAL,read),"empty group cannot report ready");
    std::cout<<"17-slave servo group isolation, S9 AL=0x1e, readiness and shutdown checks passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
