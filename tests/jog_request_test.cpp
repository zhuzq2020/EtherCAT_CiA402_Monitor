#include "jog_request.hpp"
#include <cassert>

int main() {
    robot_maint::JogRequest request;
    std::string error;
    const auto parse = [&](const std::string& text) {
        return robot_maint::parse_enabled_jog(text, 1000.0, request, error);
    };
    assert(parse("jog-enabled --delta_deg -0.5 --speed_rpm 0.3"));
    assert(request.delta_counts == -500 && request.speed_rpm == 0.3);
    assert(parse("jog-enabled --delta_count 2147483647"));
    assert(request.delta_counts == 2147483647 && request.speed_rpm == 0.5);
    assert(parse("jog-enabled --delta_count -2147483648"));
    assert(!parse("jog-enabled"));
    assert(!parse("jog-enabled --delta_count 0"));
    assert(!parse("jog-enabled --delta_count 2147483648"));
    assert(!parse("jog-enabled --delta_count 1.5"));
    assert(!parse("jog-enabled --delta_count 1 --delta_deg 1"));
    assert(!parse("jog-enabled --delta_deg nan"));
    assert(!parse("jog-enabled --delta_deg inf"));
    assert(!parse("jog-enabled --delta_count 1 --speed_rpm nan"));
    assert(!parse("jog-enabled --delta_count 1 --speed_rpm 0"));
    assert(!parse("jog-enabled --delta_count 1 --speed_rpm 3"));
    assert(!parse("jog-enabled --delta_count 1 --speed_rpm 1 --speed_rpm 1"));
    assert(!parse("jog-enabled --delta_count 1 --axis D18"));
    assert(!parse("jog-enabled --delta_deg"));
    assert(!parse("jog --delta_deg 1"));
}
