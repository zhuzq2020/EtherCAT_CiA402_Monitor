#pragma once
#include "maint_engine.hpp"
#include "motion_control.hpp"
#include "pdo_exchange.hpp"
#include <vector>
namespace robot_maint {
class PersistentPdoBus {
public:
    explicit PersistentPdoBus(const std::string& iface) : engine_(iface) {}
    ~PersistentPdoBus();
    void shutdown();
    bool prepare(const std::vector<std::string>& initial_axes,std::string& error);
    bool cycle(const std::vector<MotionOutput>& output, std::vector<MotionFeedback>& feedback);
    int last_wkc() const { return last_wkc_; }
    int expected_wkc() const { return expected_; }
    std::int64_t last_cycle_us() const { return last_cycle_us_; }
    void log_states();
    std::string diagnostics() const;
    const std::vector<MotionAxisInfo>& info() const { return info_; }
private:
    struct Layout { int cw{-1}, target{-1}, mode{-1}, sw{-1}, actual{-1}, display{-1}; unsigned rx_bits{0},tx_bits{0}; };
    bool mapping(int slave, std::uint16_t assignment, bool output, Layout& layout);
    MaintEngine engine_;
    std::vector<std::uint8_t> image_;
    std::vector<Layout> layouts_;
    // Actual SDO mode confirmed in OP for axes without a PDO mode display.
    std::vector<std::int8_t> sdo_modes_;
    std::vector<MotionAxisInfo> info_;
    bool mapped_{false};
    int expected_{0},last_wkc_{0};
    std::int64_t last_cycle_us_{0};
    PdoCycleResult last_exchange_;
    std::uint64_t recovered_cycles_{0};
};
}
