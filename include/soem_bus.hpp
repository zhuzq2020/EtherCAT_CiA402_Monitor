#pragma once

#include "cia402_defs.hpp"
#include "pdo_exchange.hpp"

extern "C" {
#include <soem/soem.h>
}

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace robot_maint {

// Exclusive interface lock preventing concurrent SOEM processes on the same NIC.
class InterfaceLock {
public:
    InterfaceLock() = default;
    ~InterfaceLock();
    InterfaceLock(const InterfaceLock&) = delete;
    InterfaceLock& operator=(const InterfaceLock&) = delete;

    bool acquire(const std::string& iface, std::string& error);
    void release();
    int fd() const noexcept { return fd_; }

private:
    int fd_{-1};
};

// Byte offsets discovered dynamically from 0x1C12 / 0x1C13 PDO mappings.
struct PdoAxisLayout {
    int cw{-1};            // 0x6040:00 uint16 (required)
    int target_pos{-1};    // 0x607A:00 int32  (required)
    int mode_op{-1};       // 0x6060:00 int8   (optional)
    int sw{-1};            // 0x6041:00 uint16 (required)
    int actual_pos{-1};    // 0x6064:00 int32  (required)
    int mode_disp{-1};     // 0x6061:00 int8   (optional)
    int actual_vel{-1};    // 0x606C:00 int32  (optional)
    int actual_torque{-1}; // 0x6077:00 int16  (optional)
    int error_code{-1};    // 0x603F:00 uint16 (optional)
    unsigned rx_bits{0};
    unsigned tx_bits{0};
    bool valid() const noexcept {
        return cw >= 0 && target_pos >= 0 && sw >= 0 && actual_pos >= 0;
    }
};

struct PdoAxisCmd {
    std::uint16_t control_word{CW_SHUTDOWN};
    std::int32_t target_position{0};
    bool hold_actual{true};
};

struct PdoAxisFeedback {
    bool valid{false};
    std::uint16_t status_word{0};
    std::int32_t actual_position{0};
    std::int32_t actual_velocity{0};
    std::int16_t actual_torque{0};
    std::uint16_t error_code{0};
    std::int8_t mode_display{8};
};

struct BusDiagnostics {
    int expected_wkc{0};
    int last_wkc{0};
    std::uint64_t total_cycles{0};
    std::uint64_t retried_cycles{0};
    std::int64_t max_cycle_us{0};
    std::int64_t max_late_us{0};
    bool healthy{true};
    PdoCycleResult last_exchange;
};

// Core SOEM EtherCAT Bus SDK:
// Handles NIC initialization, state management, CoE SDO, ESC registers,
// dynamic PDO layout discovery, motion group isolation, and real-time 2ms PDO loop.
class SoemBus {
public:
    explicit SoemBus(std::string iface);
    ~SoemBus();

    SoemBus(const SoemBus&) = delete;
    SoemBus& operator=(const SoemBus&) = delete;

    bool init(bool acquire_lock = true);
    void close();

    bool is_initialized() const noexcept { return initialized_; }
    bool is_operational() const noexcept { return operational_; }
    int slave_count() const noexcept { return slave_count_; }
    const std::string& iface() const noexcept { return iface_; }

    ecx_contextt& context() noexcept { return context_; }
    const ec_slavet& raw_slave(int slave) const { return context_.slavelist[slave]; }
    ec_slavet& raw_slave_mut(int slave) { return context_.slavelist[slave]; }

    void refresh_states();
    std::uint32_t slave_vendor_id(int slave);
    std::uint32_t slave_product_code(int slave);
    static std::string al_state_str(std::uint16_t state);

    // CoE SDO helpers
    template <typename T>
    bool read_sdo(int slave, std::uint16_t index, std::uint8_t subindex, T& value,
                  int timeout_us = EC_TIMEOUTRXM) {
        if (!initialized_ || slave < 1 || slave > slave_count_) return false;
        int size = sizeof(T);
        const int rc = ecx_SDOread(&context_, slave, index, subindex, FALSE, &size, &value, timeout_us);
        return rc > 0 && size == sizeof(T);
    }

    template <typename T>
    bool write_sdo(int slave, std::uint16_t index, std::uint8_t subindex, const T& value,
                   int timeout_us = EC_TIMEOUTRXM) {
        if (!initialized_ || slave < 1 || slave > slave_count_) return false;
        int size = sizeof(T);
        auto copy = value;
        const int rc = ecx_SDOwrite(&context_, slave, index, subindex, FALSE, size, &copy, timeout_us);
        return rc > 0;
    }

    bool read_sdo_bytes(int slave, std::uint16_t index, std::uint8_t subindex,
                        std::vector<std::uint8_t>& out, int timeout_us = EC_TIMEOUTRXM);
    bool write_sdo_bytes(int slave, std::uint16_t index, std::uint8_t subindex,
                         const std::vector<std::uint8_t>& data,
                         bool complete_access = false, int timeout_us = EC_TIMEOUTRXM);
    bool read_sdo_string(int slave, std::uint16_t index, std::uint8_t subindex, std::string& out);
    std::string pop_coe_errors();

    // ESC register helpers (FPRD/FPWR)
    int read_esc_reg(int slave, std::uint16_t reg, void* data, std::size_t size);
    int write_esc_reg(int slave, std::uint16_t reg, const void* data, std::size_t size);

    // Discover PDO mapping offsets from 0x1C12 (Rx) and 0x1C13 (Tx)
    bool discover_pdo_layout(int slave, PdoAxisLayout& layout);

    // Enter cyclic OP mode for selected slaves (empty = all eligible servos).
    // Starts the 2ms real-time PDO worker thread, confirms CSP mode, and clears pre-existing faults.
    bool enter_operational(const std::vector<int>& active_slaves,
                           const std::vector<bool>& fixed_csp_flags,
                           std::string& error);
    void shutdown_operational();

    // Real-time cyclic command / feedback interface (1-based slave indices)
    void set_axis_cmd(int slave, std::uint16_t cw, std::int32_t target_pos, bool hold_actual = false);
    void set_all_cmds(const std::vector<PdoAxisCmd>& cmds);
    bool snapshot_feedback(std::vector<PdoAxisFeedback>& out_fb, BusDiagnostics& out_diag);
    BusDiagnostics diagnostics() const;
    void report_diagnostics(const std::string& context_msg);

    const std::vector<bool>& active_mask() const noexcept { return active_mask_; }
    const std::vector<PdoAxisLayout>& layouts() const noexcept { return layouts_; }

private:
    bool discover_side_mapping(int slave, std::uint16_t assignment, bool output, PdoAxisLayout& layout);
    void start_pdo_thread();
    void stop_pdo_thread();
    void pdo_worker_loop();
    PdoCycleResult execute_one_cycle(const std::vector<PdoAxisCmd>& cmds,
                                     std::vector<PdoAxisFeedback>& prev_fb);

    std::string iface_;
    InterfaceLock lock_;
    ecx_contextt context_{};
    std::vector<std::uint8_t> iomap_;
    int slave_count_{0};
    int expected_wkc_{0};
    bool initialized_{false};
    bool operational_{false};

    std::vector<bool> active_mask_;       // 1-based [1..slave_count_]
    std::vector<bool> fixed_csp_mask_;    // 1-based [1..slave_count_]
    std::vector<std::int8_t> sdo_modes_;  // 1-based [1..slave_count_]
    std::vector<PdoAxisLayout> layouts_;  // 1-based [1..slave_count_]

    // Cyclic PDO worker thread
    std::thread pdo_thread_;
    std::atomic<bool> pdo_running_{false};
    std::atomic<bool> pdo_healthy_{true};
    std::atomic<bool> has_valid_pdo_{false};
    std::atomic<int> last_wkc_{0};
    std::atomic<std::uint64_t> total_cycles_{0};
    std::atomic<std::uint64_t> retried_cycles_{0};
    std::atomic<std::int64_t> max_cycle_us_{0};
    std::atomic<std::int64_t> max_late_us_{0};
    std::mutex pdo_mutex_;
    std::vector<PdoAxisCmd> desired_cmds_;
    std::vector<PdoAxisFeedback> latest_fb_;
    PdoCycleResult last_exchange_;
};

} // namespace robot_maint
