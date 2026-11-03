#pragma once
#include <algorithm>
#include <cstdint>
extern "C" {
#include <soem/soem.h>
}
namespace robot_maint {
struct PdoFrameResult {
    int total{-1},input{0},output{0},missing{0};
    bool sent{false};
    int no_frame{0},command_mismatch{0},expected_command{-1},received_command{-1},received_wkc{-1};
    bool complete(int expected) const { return sent&&missing==0&&total>=expected; }
};
PdoFrameResult receive_pdo_frames(ecx_contextt& context,int timeout_us);
struct PdoCycleResult {
    PdoFrameResult first,last;
    unsigned attempts{0};
    bool healthy{false};
};
// A retry sends the SAME controlwords and absolute targets. Never add counters
// from different attempts or advance motion on an incomplete exchange.
template<class Clock,class Exchange>
PdoCycleResult bounded_pdo_exchange(int expected,Clock clock,Exchange exchange) {
    PdoCycleResult result;
    const std::int64_t deadline=clock()+2000;
    for(unsigned n=0;n<2;++n) {
        const auto remaining=deadline-clock();
        if(remaining<=0)break;
        result.last=exchange(static_cast<int>(std::min<std::int64_t>(1000,remaining)));
        ++result.attempts;
        if(n==0)result.first=result.last;
        if(result.last.complete(expected)&&clock()<=deadline){result.healthy=true;break;}
    }
    return result;
}
}
