// Test-only bus implementation. This executable never opens an EtherCAT socket.
#include "control_service.hpp"
#include "persistent_bus.hpp"
#include <cstdlib>
#include <csignal>
#include <unistd.h>
#include <fstream>
#include <stdexcept>
#include <utility>
namespace robot_maint {
namespace {
void audit(const std::string& event){std::ofstream(std::getenv("ARM_MAINT_TEST_AUDIT"),std::ios::app)<<event<<'\n';}
}
MaintEngine::MaintEngine(std::string iface):iface_(std::move(iface)){}
MaintEngine::~MaintEngine()=default;
void MaintEngine::close(){}
bool MaintEngine::init(){throw std::runtime_error("unexpected legacy bus init in lifecycle test");}
bool MaintEngine::disable_axis(int,std::string&){throw std::runtime_error("unexpected legacy disable");}
bool PersistentPdoBus::prepare(const std::vector<std::string>& initial_axes,std::string& error){
    if(const char* failure=std::getenv("ARM_MAINT_TEST_FAIL")) {
        if(std::string(failure)=="signal"){std::signal(SIGSEGV,SIG_DFL);::raise(SIGSEGV);}
        if(std::string(failure)=="exit")::_exit(23);
        if(std::string(failure)=="error"){error="mock PDO mapping rejected";return false;}
    }
    if(!initial_axes.empty()){error="start-pdo must discover the entire bus";return false;}
    info_.resize(17);
    for(int i=0;i<17;++i)info_[i]={"S"+std::to_string(i+1),"",1000};
    for(int i:{0,8,16})info_[i].unavailable="unsupported";
    expected_=42;mapped_=true;audit("PREPARE 14");return true;
}
bool PersistentPdoBus::cycle(const std::vector<MotionOutput>& outputs,std::vector<MotionFeedback>& feedback){
    static std::vector<bool> enabled(17,false);
    feedback.assign(17,{});last_wkc_=42;last_cycle_us_=10;
    for(int i=0;i<17;++i)if(info_[i].unavailable.empty()){
        const auto control=outputs.at(i).control;
        const bool active=control==15;
        if(active!=enabled[i])audit(std::string(active?"ENABLED ":"DISABLED ")+info_[i].name);
        enabled[i]=active;
        feedback[i]={true,static_cast<std::uint16_t>(active?0x27:control==7?0x23:0x21),1000+i,8};
    }
    return true;
}
void PersistentPdoBus::shutdown(){if(mapped_){audit("SHUTDOWN");mapped_=false;}}
void PersistentPdoBus::log_states(){}
std::string PersistentPdoBus::diagnostics() const{return "WKC=42 expected=42";}
PersistentPdoBus::~PersistentPdoBus(){shutdown();}
}
int main(int argc,char** argv){int result=1;if(!std::getenv("ARM_MAINT_TEST_AUDIT"))return 2;
    return robot_maint::handle_control_cli(argc,argv,result)?result:2;}
