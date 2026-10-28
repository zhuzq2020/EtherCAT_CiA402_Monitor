#pragma once
#include <string>
#include <vector>
namespace robot_maint {
class InterfaceLock {
public:
    ~InterfaceLock();
    bool acquire(const std::string& iface,std::string& error);
    int fd() const { return fd_; }
    InterfaceLock()=default;
    InterfaceLock(const InterfaceLock&)=delete;
    InterfaceLock& operator=(const InterfaceLock&)=delete;
private:int fd_{-1};
};
// Called before the legacy parser and before any EtherCAT initialization.
bool handle_control_cli(int argc,char** argv,int& exit_code);
// 0=no service, 1=success response, -1=failure response/connection error.
int control_request(const std::string& iface,const std::string& request,std::string& response,int timeout_ms=125000);
struct ControlOptions {
    std::string command,iface{"enp2s0"};
    std::vector<std::string> axes;
    bool all{false},degrees{true},displacement{false};
    std::vector<double> deltas;
    double rpm{.5};
};
bool parse_control_options(const std::vector<std::string>& args,ControlOptions& options,std::string& error);
}
