#pragma once
#include <cstdint>
#include <string>

namespace robot_maint {
// Request in PRE-OP; confirm the actual mode only after OP and PDO exchange.
template<class Write>
bool request_csp_mode(Write write, std::string& error) {
    error.clear();
    if(write(std::int8_t{8}))return true;
    error="写入 0x6060:00 = 8 失败（CSP 模式请求未获应答）";
    return false;
}
template<class Read, class Exchange>
bool confirm_csp_mode(Read read, Exchange exchange, std::string& error) {
    error.clear();bool received=false;std::int8_t last=0;
    for(int attempt=0;attempt<10;++attempt) {
        // Keep sending Shutdown and held positions while awaiting mode activation.
        if(!exchange()){error="确认 CSP 模式期间 PDO 通信异常";return false;}
        std::int8_t display=0;
        if(read(display)) {
            received=true;last=display;
            if(display==8)return true;
        }
    }
    error=received ? "OP 下 0x6061:00 未确认 CSP，最后有效模式="+std::to_string(int(last))
                   : "OP 下读取 0x6061:00 失败";
    return false;
}
}
