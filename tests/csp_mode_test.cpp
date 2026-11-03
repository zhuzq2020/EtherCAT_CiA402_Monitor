#include "csp_mode.hpp"
#include "motion_control.hpp"
#include <iostream>
#include <stdexcept>
using namespace robot_maint;
void check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(){try{
    std::string error;int writes=0,reads=0,exchanges=0;bool operational=false;
    std::int8_t requested=0;
    auto write=[&](std::int8_t mode){++writes;requested=mode;return true;};
    // Model the observed drive: 6060=8, but 6061=0 until OP/PDO exchange.
    auto read=[&](std::int8_t& mode){++reads;mode=operational?requested:0;return true;};
    auto exchange=[&]{++exchanges;return true;};
    check(!request_csp_mode([](std::int8_t){return false;},error),"rejected mode write fails");
    check(error.find("0x6060")!=std::string::npos,"write failure identified");
    check(request_csp_mode(write,error)&&requested==8&&reads==0,"PRE-OP requests without premature mode confirmation");
    operational=true;
    check(confirm_csp_mode(read,exchange,error)&&reads==1&&exchanges==1,"mode display confirms after OP and exchange");
    reads=exchanges=0;
    check(confirm_csp_mode([&](std::int8_t& mode){mode=++reads<3?0:8;return true;},exchange,error),"delayed OP display accepted");
    check(reads==3&&exchanges==3,"PDO exchange continues between mailbox reads");
    reads=exchanges=0;
    check(!confirm_csp_mode([&](std::int8_t& mode){++reads;mode=0;return true;},exchange,error),"OP display remaining zero fails");
    check(reads==10&&exchanges==10&&error.find("最后有效模式=0")!=std::string::npos,"bounded mismatch reports actual mode");
    reads=exchanges=0;
    check(!confirm_csp_mode([&](std::int8_t&){++reads;return false;},exchange,error),"unreadable display never assumed CSP");
    check(reads==10&&exchanges==10&&error.find("读取 0x6061:00 失败")!=std::string::npos,"read failure identified");
    reads=0;
    check(!confirm_csp_mode(read,[]{return false;},error)&&reads==0,"communication loss aborts confirmation");
    // Confirming OP does not itself permit enable: actual mode feedback is required.
    MotionControl control({{"S6","",1000},{"S7","",1000}});
    std::vector<MotionFeedback> feedback{{true,0x40,100,8},{true,0x40,200,0}};
    control.update(feedback,true);MotionTicket ticket;
    check(!control.enable({0,1},ticket,error),"one unconfirmed mode blocks entire enable request");
    check(control.outputs()[0].control==6&&control.outputs()[1].control==6,"no enable controlword before all modes confirmed");
    feedback[1].mode=8;control.update(feedback,true);
    check(control.enable({0,1},ticket,error),"enable accepted only after both actual modes confirm");
    check(writes==1,"mode request is separate from confirmation");
    std::cout<<"PRE-OP request, OP mode activation, continued PDO exchange and enable gating passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
