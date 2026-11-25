#pragma once

#include "cia402_defs.hpp"
#include "drive_profile.hpp"
#include "soem_bus.hpp"

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace robot_maint {

struct AxisStatus {
    int slave_index{0};
    std::uint16_t station_alias{0};
    std::string axis_name;
    std::string model_name;
    std::string ethercat_state;
    std::uint16_t status_word{0};
    std::string cia402_state;
    bool is_fault{false};
    std::uint16_t error_code{0};
    std::string error_desc;
    std::int32_t actual_position{0};
    std::int32_t actual_velocity{0};
    std::int16_t actual_torque{0};
    std::uint32_t vendor_id{0};
    std::string vendor_name;
    std::uint32_t product_code{0};
    std::uint32_t revision_num{0};
    std::uint32_t serial_num{0};
    std::string dev_name;
    std::string sw_version;
    double counts_per_rev{0.0};
    double counts_per_deg{0.0};
    bool is_servo{false};
    bool fixed_csp{false};
};

struct AxisMotionFeedback {
    int slave_index{0};
    std::string axis_name;
    double position_deg{0.0};
    std::int32_t position_counts{0};
    std::int32_t velocity_counts{0};
    std::int16_t torque_raw{0};
    std::uint16_t status_word{0};
    std::uint16_t error_code{0};
};

// Unified High-Level Robotic Arm SDK supporting Fairino, Tongchuan, Eyou, and CiA-402 drives.
class ArmController {
public:
    explicit ArmController(std::string iface);
    ~ArmController();

    ArmController(const ArmController&) = delete;
    ArmController& operator=(const ArmController&) = delete;

    bool init_bus();
    void close_bus();

    SoemBus& bus() noexcept { return bus_; }
    const std::vector<AxisStatus>& axes() const noexcept { return axes_; }
    int slave_count() const noexcept { return bus_.slave_count(); }

    // Discovery & Calibration
    std::vector<AxisStatus> scan_status(bool verbose_params = false);
    void print_status_table(const std::vector<AxisStatus>& list, bool detailed = false) const;
    bool set_zero(int slave_index, std::string& detail);

    // Reusable Real-Time Motion Session API (OP + 2ms PDO + smooth enable + hold)
    bool open_motion(const std::vector<int>& enable_slaves, std::string& error);
    bool move_axes_to(const std::vector<int>& target_slaves,
                      const std::vector<double>& target_deg,
                      int move_ms,
                      std::string& error,
                      bool print_progress = false);
    bool hold_motion(int hold_ms, std::string& error);
    bool read_motion_feedback(const std::vector<int>& target_slaves,
                              std::vector<AxisMotionFeedback>& out,
                              BusDiagnostics& diag,
                              std::string& error);
    void close_motion();

    // Convenience Self-Contained Single-Axis Commands
    bool run_jog(int target_slave, double delta_deg, int duration_ms = 3000);
    bool run_jog_abs(int target_slave, double abs_deg, int duration_ms = 4000);
    bool run_zero(int target_slave, int duration_ms = 5000);

    std::vector<int> eligible_servo_slaves() const;

    static void request_stop() noexcept;
    static bool is_stop_requested() noexcept;

private:
    void populate_axes_metadata();
    std::vector<bool> build_fixed_csp_mask() const;
    bool enable_axes_smoothly(const std::vector<int>& enable_slaves, int timeout_ms, std::string& error);
    bool check_enabled_health(const std::vector<int>& slaves,
                              const std::vector<PdoAxisFeedback>& fb_snap,
                              const BusDiagnostics& diag,
                              const char* phase,
                              std::string& error);
    bool run_single_axis_jog(int target_slave, double target_deg, bool is_relative,
                             int duration_ms, const std::string& title);

    SoemBus bus_;
    std::vector<AxisStatus> axes_;
    std::vector<int> motion_enabled_slaves_;
    std::vector<std::int32_t> held_targets_; // 1-based last commanded target per slave
    static std::atomic<bool> stop_requested_;
};

} // namespace robot_maint
