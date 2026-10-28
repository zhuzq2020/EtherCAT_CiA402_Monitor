#pragma once
#include "maint_engine.hpp"
#include "motion_control.hpp"
#include <vector>
namespace robot_maint {
class PersistentPdoBus {
public:
    explicit PersistentPdoBus(const std::string& iface) : engine_(iface) {}
    ~PersistentPdoBus();
    void shutdown();
    bool prepare(const std::vector<std::string>& initial_axes,std::string& error);
    bool cycle(const std::vector<MotionOutput>& output, std::vector<MotionFeedback>& feedback);
    const std::vector<MotionAxisInfo>& info() const { return info_; }
private:
    struct Layout { int cw{-1}, target{-1}, mode{-1}, sw{-1}, actual{-1}, display{-1}; unsigned rx_bits{0},tx_bits{0}; };
    bool mapping(int slave, std::uint16_t assignment, bool output, Layout& layout);
    MaintEngine engine_;
    std::vector<std::uint8_t> image_;
    std::vector<Layout> layouts_;
    std::vector<MotionAxisInfo> info_;
    bool mapped_{false};
    int expected_{0};
};
}
