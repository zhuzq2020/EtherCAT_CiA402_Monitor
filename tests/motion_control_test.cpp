#include "motion_control.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace robot_maint;
void check(bool ok,const char* text){if(!ok)throw std::runtime_error(text);}
std::vector<MotionAxisInfo> axes(){return {{"D18","",1000},{"D19","",2000},{"D20","",1000}};}
std::vector<MotionFeedback> feedback(std::uint16_t state=0x40){return {{true,state,100,8},{true,state,200,8},{true,state,-100,8}};}
void make_enabled(MotionControl& c,std::vector<MotionFeedback>& f,const std::vector<int>& ids){
    MotionTicket ticket;std::string error;
    check(c.enable(ids,ticket,error),"enable accepted");
    for(int i:ids)f[i].status=0x21;c.update(f,true);
    for(int i:ids)f[i].status=0x23;c.update(f,true);
    for(int i:ids)f[i].status=0x27;c.update(f,true);
    check(c.result(ticket,error)==1,"all requested axes enabled");
}
int main(){try{
    std::string error;MotionTicket ticket;std::vector<int> ids;
    check(resolve_motion_axes({"D18","D19"},7,ids,error)&&ids==std::vector<int>({0,1}),"axis labels resolve");
    check(!resolve_motion_axes({"D18","1"},7,ids,error),"duplicate hardware target rejected");
    check(!resolve_motion_axes({"D18oops"},7,ids,error),"malformed axis rejected");
    check(!resolve_motion_axes({"D24"},3,ids,error),"missing axis rejected");
    {
        MotionControl c(axes());auto f=feedback();c.update(f,true);
        const auto before=c.outputs();
        check(!c.jog({0,1},true,1,.3,ticket,error),"disabled group cannot jog");
        check(error.find("D18 未使能")!=std::string::npos&&error.find("D19 未使能")!=std::string::npos,"all disabled axes named");
        const auto after=c.outputs();
        check(before[0].target==after[0].target&&before[1].target==after[1].target,"rejection changes no targets");
    }
    {
        MotionControl c(axes());auto f=feedback();c.update(f,true);make_enabled(c,f,{0});
        const auto before=c.outputs();
        check(!c.jog({0,1},true,1,.3,ticket,error),"mixed enabled/disabled batch rejected");
        check(error.find("D19 未使能")!=std::string::npos&&error.find("D18 未使能")==std::string::npos,"specific failing axis named");
        for(int n=0;n<1000;++n)c.update(f,true);
        check(c.outputs()[0].target==before[0].target&&c.outputs()[1].target==before[1].target,"enabled member never moves on failed batch");
        check(c.outputs()[0].control==15,"rejecting jog does not drop existing enable");
    }
    {
        MotionControl c(axes());auto f=feedback();c.update(f,true);make_enabled(c,f,{0,1});
        check(c.jog({0,1},true,1,.3,ticket,error),"multi-axis jog accepted");
        int n=0;for(;n<60000&&c.result(ticket,error)==0;++n)c.update(f,true);
        check(c.result(ticket,error)==1,"multi-axis jog completes");
        check(c.outputs()[0].target==1100&&c.outputs()[1].target==2200,"per-axis conversion and synchronized endpoints");
        check(c.outputs()[0].control==15&&c.outputs()[1].control==15,"axes remain enabled after jog");
        check(c.jog({0},false,-500,.5,ticket,error),"second independent jog accepted");
        for(int j=0;j<60000&&c.result(ticket,error)==0;++j)c.update(f,true);
        check(c.outputs()[0].target==600&&c.outputs()[1].target==2200,"relative motion and unaffected held axis");
    }
    {
        MotionControl c(axes());auto f=feedback();c.update(f,true);make_enabled(c,f,{0,1});
        MotionTicket a,b;
        check(c.jog({0},true,1,.3,a,error)&&c.jog({1},true,-1,.3,b,error),"disjoint commands may move concurrently");
        MotionTicket rejected;
        check(!c.jog({0},true,1,.3,rejected,error)&&error.find("D18 正在运动")!=std::string::npos,"overlapping motion rejected");
        for(int n=0;n<60000&&(c.result(a,error)==0||c.result(b,error)==0);++n)c.update(f,true);
        check(c.outputs()[0].target==1100&&c.outputs()[1].target==-1800,"independent trajectories complete");
    }
    {
        MotionControl c(axes());auto f=feedback();c.update(f,true);make_enabled(c,f,{0,1});
        const auto before=c.outputs();f[1].valid=false;c.update(f,true);
        check(!c.jog({0,1},true,1,.3,ticket,error)&&error.find("D19")!=std::string::npos,"invalid feedback blocks entire jog");
        check(c.outputs()[0].target==before[0].target&&c.outputs()[0].control==6,"feedback loss stops all controlled axes");
    }
    {
        MotionControl c(axes());auto f=feedback();c.update(f,true);make_enabled(c,f,{0,1});
        auto invalid=std::numeric_limits<double>::quiet_NaN();
        check(!c.jog({0,1},true,invalid,.3,ticket,error),"nonfinite displacement rejected");
        check(!c.jog({0,1},false,2147483647,.3,ticket,error),"overflow/long motion rejected");
        check(c.outputs()[0].target==100&&c.outputs()[1].target==200,"invalid planning is atomic");
    }
    {
        MotionControl c(axes());auto f=feedback();c.update(f,true);make_enabled(c,f,{0,1});
        MotionTicket motion,disable;
        check(c.jog({0,1},true,1,.3,motion,error),"start batch before disable");
        for(int n=0;n<10;++n)c.update(f,true);
        check(c.disable({1},disable,error),"disable selected member");
        const auto stopped=c.outputs();
        for(int n=0;n<10;++n){f[1].status=0x21;c.update(f,true);}
        check(c.result(motion,error)==-1,"disabled member cancels whole batch");
        check(c.result(disable,error)==1,"disable waits for feedback confirmation");
        check(c.outputs()[0].target==stopped[0].target&&c.outputs()[0].control==15,"other member holds without new movement");
    }
    {
        MotionControl c(axes());auto f=feedback();c.update(f,true);make_enabled(c,f,{0});
        check(c.jog({0},true,1,.3,ticket,error),"start before disconnect");
        c.abort(ticket,"客户端断开");
        check(c.result(ticket,error)==-1&&c.outputs()[0].control==6,"client disconnect cancels and disables request axes");
    }
    {
        MotionControl c(axes());auto f=feedback();c.update(f,true);
        check(c.enable({0,1},ticket,error),"enable batch before fault");
        f[1].status=8;c.update(f,true);
        check(c.result(ticket,error)==-1&&error.find("D19")!=std::string::npos,"enable fault names target and rejects group");
        check(c.outputs()[0].control==6,"failed enable rolls back requested state");
    }
    {
        MotionControl c(axes());auto f=feedback();c.update(f,true);make_enabled(c,f,{0,1});
        check(c.jog({0,1},true,1,.3,ticket,error),"start before bad WKC");
        c.update(f,false);
        check(c.result(ticket,error)==-1&&c.outputs()[0].control==6&&c.outputs()[1].control==6,"bad WKC stops both axes");
    }
    {
        MotionControl c(axes());auto f=feedback();c.update(f,true);make_enabled(c,f,{0,1});
        // List order follows the user's axis order, not numeric/sorted order.
        check(c.jog({1,0},true,std::vector<double>{-.5,1},.3,ticket,error),"per-axis opposite angles accepted");
        bool started0=false,started1=false;
        for(int n=0;n<60000&&c.result(ticket,error)==0;++n) {
            c.update(f,true);
            started0|=c.outputs()[0].target!=100;started1|=c.outputs()[1].target!=200;
        }
        check(c.result(ticket,error)==1&&started0&&started1,"both axes finish one coordinated request");
        check(c.outputs()[0].target==1100&&c.outputs()[1].target==-800,"axis ordering and different encoder ratios preserved");
        check(c.outputs()[0].control==15&&c.outputs()[1].control==15,"different-angle jog stays enabled");
    }
    {
        MotionControl c(axes());auto f=feedback();c.update(f,true);make_enabled(c,f,{0,1});
        check(c.jog({0,1},false,std::vector<double>{500,-2000},.3,ticket,error),"per-axis pulse list accepted");
        for(int n=0;n<60000&&c.result(ticket,error)==0;++n)c.update(f,true);
        check(c.outputs()[0].target==600&&c.outputs()[1].target==-1800,"different pulse endpoints");
    }
    {
        MotionControl c(axes());auto f=feedback();c.update(f,true);make_enabled(c,f,{0,1});
        const auto before=c.outputs();
        check(!c.jog({0,1},true,std::vector<double>{1,-.5,2},.3,ticket,error),"list length mismatch rejected");
        check(!c.jog({0,1},true,std::vector<double>{1,2147483647},.3,ticket,error),"second axis overflow rejects first axis too");
        check(error.find("D19")!=std::string::npos,"overflow names the failing axis");
        check(c.outputs()[0].target==before[0].target&&c.outputs()[1].target==before[1].target,"failed list never changes any target");
    }
    {
        MotionControl c(axes());auto f=feedback();c.update(f,true);make_enabled(c,f,{0,1});
        check(c.jog({0,1},true,std::vector<double>{0,-.5},.3,ticket,error),"zero displacement member holds");
        for(int n=0;n<60000&&c.result(ticket,error)==0;++n) {
            c.update(f,true);check(c.outputs()[0].target==100,"zero member never moves");
        }
        check(c.outputs()[1].target==-800,"other member moves requested distance");
        check(!c.jog({0,1},true,std::vector<double>{0,0},.3,ticket,error),"all-zero request rejected");
    }
    {
        MotionControl c(axes());auto f=feedback();c.update(f,true);make_enabled(c,f,{0});
        check(!c.jog({0,1},true,std::vector<double>{1,-.5},.3,ticket,error),"per-axis list retains all-enabled precondition");
        check(error.find("D19 未使能")!=std::string::npos&&c.outputs()[0].target==100,"disabled member named and enabled member stays put");
    }
    std::cout<<"Multi-axis enable/jog atomicity, per-axis displacement, named errors, cancellation and PDO-failure checks passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
