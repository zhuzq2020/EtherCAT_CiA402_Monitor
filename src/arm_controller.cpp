#include "arm_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace robot_maint {

std::atomic<bool> ArmController::stop_requested_{false};

void ArmController::request_stop() noexcept {
    stop_requested_.store(true);
}

bool ArmController::is_stop_requested() noexcept {
    return stop_requested_.load();
}

ArmController::ArmController(std::string iface) : bus_(std::move(iface)) {
    stop_requested_.store(false);
}

ArmController::~ArmController() {
    close_bus();
}

bool ArmController::init_bus() {
    stop_requested_.store(false);
    if (!bus_.init(true)) return false;
    populate_axes_metadata();
    return true;
}

void ArmController::close_bus() {
    close_motion();
    bus_.close();
    axes_.clear();
}

void ArmController::populate_axes_metadata() {
    const int n = bus_.slave_count();
    axes_.assign(n, AxisStatus{});

    bool any_fairino = false;
    for (int i = 1; i <= n; ++i) {
        if (bus_.slave_vendor_id(i) == kFairinoVendorId) {
            any_fairino = true;
            break;
        }
    }

    for (int i = 1; i <= n; ++i) {
        AxisStatus& ax = axes_[i - 1];
        const auto& raw = bus_.raw_slave(i);
        ax.slave_index = i;
        ax.station_alias = raw.aliasadr;
        ax.ethercat_state = SoemBus::al_state_str(raw.state);
        ax.vendor_id = bus_.slave_vendor_id(i);
        ax.product_code = bus_.slave_product_code(i);
        ax.axis_name = slave_to_axis_name(i, n, (ax.vendor_id == kFairinoVendorId) || any_fairino);

        if ((raw.mbx_proto & ECT_MBXPROT_COE) != 0) {
            bus_.read_sdo(i, SDO_IDENTITY_OBJECT, 3, ax.revision_num);
            bus_.read_sdo(i, SDO_IDENTITY_OBJECT, 4, ax.serial_num);
            bus_.read_sdo_string(i, SDO_DEVICE_NAME, 0, ax.dev_name);
            bus_.read_sdo_string(i, SDO_SOFTWARE_VERSION, 0, ax.sw_version);
        }

        ax.is_servo = is_eligible_servo(bus_, i);
        const DriveCapabilities caps = inspect_drive_capabilities(bus_, i, ax.vendor_id, ax.product_code, ax.dev_name);
        ax.vendor_name = caps.vendor_name;
        ax.model_name = caps.model_name;
        ax.fixed_csp = caps.fixed_csp_mode;

        if (ax.is_servo) {
            ax.counts_per_rev = resolve_counts_per_rev(bus_, i, ax.vendor_id);
            ax.counts_per_deg = ax.counts_per_rev / 360.0;
        }
    }
}

std::vector<int> ArmController::eligible_servo_slaves() const {
    std::vector<int> result;
    for (const auto& ax : axes_) {
        if (ax.is_servo) result.push_back(ax.slave_index);
    }
    return result;
}

std::vector<bool> ArmController::build_fixed_csp_mask() const {
    std::vector<bool> mask(axes_.size() + 1, false);
    for (const auto& ax : axes_) {
        mask[ax.slave_index] = ax.fixed_csp;
    }
    return mask;
}

std::vector<AxisStatus> ArmController::scan_status(bool verbose_params) {
    if (!bus_.is_initialized() && !init_bus()) return {};
    bus_.refresh_states();

    for (int i = 1; i <= bus_.slave_count(); ++i) {
        AxisStatus& ax = axes_[i - 1];
        const auto& raw = bus_.raw_slave(i);
        ax.station_alias = raw.aliasadr;
        ax.ethercat_state = SoemBus::al_state_str(raw.state);

        if ((raw.mbx_proto & ECT_MBXPROT_COE) != 0) {
            bus_.read_sdo(i, SDO_STATUS_WORD, 0, ax.status_word);
            ax.is_fault = (ax.status_word & SW_FAULT_BIT) != 0;
            ax.cia402_state = decode_cia402_state(ax.status_word);
            bus_.read_sdo(i, SDO_ERROR_CODE, 0, ax.error_code);
            ax.error_desc = describe_error_code(ax.error_code);
            bus_.read_sdo(i, SDO_ACTUAL_POSITION, 0, ax.actual_position);
            bus_.read_sdo(i, SDO_ACTUAL_VELOCITY, 0, ax.actual_velocity);
            if (verbose_params) {
                bus_.read_sdo(i, SDO_ACTUAL_TORQUE, 0, ax.actual_torque);
            }
        } else {
            ax.cia402_state = "不支持CoE";
        }
    }
    return axes_;
}

void ArmController::print_status_table(const std::vector<AxisStatus>& list, bool detailed) const {
    std::cout << "\n========================================= EtherCAT 总线扫描结果 (共 "
              << list.size() << " 个从站) =========================================\n";
    std::cout << std::left
              << std::setw(6)  << "轴号"
              << std::setw(6)  << "从站"
              << std::setw(7)  << "Alias"
              << std::setw(18) << "厂商 (Vendor)"
              << std::setw(15) << "型号"
              << std::setw(9)  << "总线状态"
              << std::setw(22) << "CiA-402 状态"
              << std::setw(9)  << "故障码"
              << std::setw(13) << "实际脉冲"
              << std::setw(10) << "角度(°)"
              << "故障描述\n";
    std::cout << "------------------------------------------------------------------------------------------------------------------------\n";

    for (const auto& a : list) {
        std::ostringstream code_ss;
        code_ss << "0x" << std::hex << std::setw(4) << std::setfill('0') << a.error_code;

        std::ostringstream deg_ss;
        if (a.is_servo && a.counts_per_deg > 0.0) {
            deg_ss << std::fixed << std::setprecision(3)
                   << (static_cast<double>(a.actual_position) / a.counts_per_deg);
        } else {
            deg_ss << "-";
        }

        std::cout << std::left
                  << std::setw(6)  << a.axis_name
                  << std::setw(6)  << a.slave_index
                  << std::setw(7)  << a.station_alias
                  << std::setw(18) << a.vendor_name
                  << std::setw(15) << a.model_name
                  << std::setw(9)  << a.ethercat_state
                  << std::setw(22) << a.cia402_state
                  << std::setw(9)  << code_ss.str()
                  << std::setw(13) << a.actual_position
                  << std::setw(10) << deg_ss.str()
                  << a.error_desc << "\n";

        if (detailed) {
            std::cout << "      Vendor=0x" << std::hex << a.vendor_id
                      << " Product=0x" << a.product_code
                      << " Rev=0x" << a.revision_num
                      << std::dec << " Serial=" << a.serial_num
                      << " | SW_FW=" << (a.sw_version.empty() ? "-" : a.sw_version)
                      << " | counts/rev=" << std::fixed << std::setprecision(1) << a.counts_per_rev
                      << " | vel=" << a.actual_velocity
                      << " | torque=" << a.actual_torque << "\n";
        }
    }
    std::cout << "========================================================================================================================\n\n";
}

bool ArmController::set_zero(int slave_index, std::string& detail) {
    if (!bus_.is_initialized() && !init_bus()) {
        detail = "总线初始化失败";
        return false;
    }
    return drive_set_zero(bus_, slave_index, detail);
}

bool ArmController::enable_axes_smoothly(const std::vector<int>& enable_slaves,
                                         int timeout_ms,
                                         std::string& error) {
    std::vector<bool> target_mask(bus_.slave_count() + 1, false);
    std::ostringstream desc;
    for (std::size_t idx = 0; idx < enable_slaves.size(); ++idx) {
        const int s = enable_slaves[idx];
        if (s >= 1 && s <= bus_.slave_count()) {
            target_mask[s] = true;
            if (idx > 0) desc << ",";
            desc << axes_[s - 1].axis_name;
        }
    }
    const std::string target_desc = (enable_slaves.size() > 1)
        ? ("全部 " + std::to_string(enable_slaves.size()) + " 轴 (" + desc.str() + ")")
        : desc.str();
    std::cout << "[阶段 1] CiA 402 使能序列 (" << target_desc << ") ...\n";

    std::vector<int> fault_reset_ms(bus_.slave_count() + 1, 0);
    bool all_enabled = false;
    int stable_ms = 0;
    bool hold_notified = false;

    const auto start_tp = std::chrono::steady_clock::now();
    std::vector<PdoAxisFeedback> fb_snap;
    BusDiagnostics diag;
    std::vector<PdoAxisCmd> cmds(bus_.slave_count() + 1);

    while (!stop_requested_.load()) {
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_tp).count();
        if (elapsed_ms >= timeout_ms) break;

        bus_.snapshot_feedback(fb_snap, diag);
        if (!diag.healthy) {
            bus_.report_diagnostics("使能阶段 PDO 通信异常");
            error = "使能阶段 PDO 通信异常";
            return false;
        }

        bool target_ready = true;
        for (int i = 1; i <= bus_.slave_count(); ++i) {
            if (!bus_.active_mask()[i]) continue;
            const auto& fb = fb_snap[i];
            cmds[i].target_position = fb.actual_position;
            cmds[i].hold_actual = true;

            const bool is_target = target_mask[i];
            const std::uint16_t sw = fb.status_word;

            if ((sw & SW_FAULT_BIT) != 0) {
                if (fault_reset_ms[i] < 20) cmds[i].control_word = 0x0000U;
                else if (fault_reset_ms[i] < 120) cmds[i].control_word = CW_FAULT_RESET;
                else { cmds[i].control_word = 0x0000U; fault_reset_ms[i] = 0; }
                fault_reset_ms[i] += 2;
                if (is_target) target_ready = false;
            } else {
                fault_reset_ms[i] = 0;
                if (!is_target) {
                    cmds[i].control_word = CW_SHUTDOWN;
                    continue;
                }
                if (elapsed_ms < 80) {
                    cmds[i].control_word = CW_SHUTDOWN;
                    target_ready = false;
                } else if ((sw & SW_MASK_STATE) == SW_OPERATION_ENABLED) {
                    cmds[i].control_word = CW_ENABLE_OPERATION;
                } else if ((sw & 0x004FU) == SW_SWITCH_ON_DISABLED) {
                    cmds[i].control_word = CW_SHUTDOWN;
                    target_ready = false;
                } else if ((sw & SW_MASK_STATE) == SW_READY_TO_SWITCH_ON) {
                    cmds[i].control_word = (elapsed_ms > 250) ? CW_ENABLE_OPERATION : CW_SWITCH_ON;
                    target_ready = false;
                } else if ((sw & SW_MASK_STATE) == SW_SWITCHED_ON) {
                    cmds[i].control_word = CW_ENABLE_OPERATION;
                    target_ready = false;
                } else {
                    cmds[i].control_word = CW_SHUTDOWN;
                    target_ready = false;
                }
            }
        }

        bus_.set_all_cmds(cmds);

        if (target_ready) {
            stable_ms += 2;
            if (!hold_notified && stable_ms >= 20) {
                std::cout << "[阶段 1] " << target_desc << " 已进入 Operation Enabled，执行 400ms 抱闸释放稳态保位 ...\n";
                hold_notified = true;
            }
            if (stable_ms >= 400) {
                all_enabled = true;
                break;
            }
        } else {
            stable_ms = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    if (!all_enabled) {
        bus_.snapshot_feedback(fb_snap, diag);
        std::ostringstream oss;
        oss << "目标关节 " << target_desc << " 未能进入 Operation Enabled (WKC="
            << diag.last_wkc << "/" << diag.expected_wkc << ")";
        for (int s : enable_slaves) {
            const auto& fb = fb_snap[s];
            oss << "；" << axes_[s - 1].axis_name << " SW=0x" << std::hex << fb.status_word
                << " Err=0x" << fb.error_code << std::dec
                << " (" << describe_error_code(fb.error_code) << ")";
        }
        error = oss.str();
        return false;
    }

    bus_.snapshot_feedback(fb_snap, diag);
    held_targets_.assign(bus_.slave_count() + 1, 0);
    for (int s : enable_slaves) {
        held_targets_[s] = fb_snap[s].actual_position;
    }
    std::cout << "[阶段 1] " << target_desc << " 抱闸已释放且稳态闭环就绪。\n";
    return true;
}

bool ArmController::check_enabled_health(const std::vector<int>& slaves,
                                         const std::vector<PdoAxisFeedback>& fb_snap,
                                         const BusDiagnostics& diag,
                                         const char* phase,
                                         std::string& error) {
    if (!diag.healthy) {
        bus_.report_diagnostics(std::string(phase) + " PDO 通信异常");
        error = std::string(phase) + " PDO 通信异常";
        return false;
    }
    for (int s : slaves) {
        const auto& fb = fb_snap[s];
        if ((fb.status_word & SW_FAULT_BIT) != 0) {
            std::ostringstream oss;
            oss << "轴 " << axes_[s - 1].axis_name << " 在" << phase << "报警: SW=0x"
                << std::hex << fb.status_word << " Err=0x" << fb.error_code << std::dec
                << " (" << describe_error_code(fb.error_code) << ")";
            error = oss.str();
            return false;
        }
        if (!is_operation_enabled(fb.status_word)) {
            std::ostringstream oss;
            oss << "轴 " << axes_[s - 1].axis_name << " 在" << phase
                << "退出 Operation Enabled: SW=0x" << std::hex << fb.status_word << std::dec;
            error = oss.str();
            return false;
        }
    }
    return true;
}

bool ArmController::open_motion(const std::vector<int>& enable_slaves_in, std::string& error) {
    error.clear();
    if (!bus_.is_initialized() && !init_bus()) {
        error = "总线初始化失败";
        return false;
    }
    const std::vector<int> enable_slaves = enable_slaves_in.empty() ? eligible_servo_slaves() : enable_slaves_in;
    if (enable_slaves.empty()) {
        error = "总线上未发现受支持的 CiA-402 伺服轴";
        return false;
    }
    for (int s : enable_slaves) {
        if (s < 1 || s > bus_.slave_count() || !axes_[s - 1].is_servo) {
            error = "目标从站 " + std::to_string(s) + " 无效或不是伺服轴";
            return false;
        }
    }

    const auto active = eligible_servo_slaves();
    if (!bus_.enter_operational(active, build_fixed_csp_mask(), error)) {
        return false;
    }

    const int timeout_ms = enable_slaves.size() > 1 ? 6000 : 4000;
    if (!enable_axes_smoothly(enable_slaves, timeout_ms, error)) {
        bus_.shutdown_operational();
        return false;
    }

    motion_enabled_slaves_ = enable_slaves;
    std::vector<PdoAxisCmd> cmds(bus_.slave_count() + 1);
    for (int s : active) cmds[s] = {CW_SHUTDOWN, 0, true};
    for (int s : motion_enabled_slaves_) cmds[s] = {CW_ENABLE_OPERATION, held_targets_[s], false};
    bus_.set_all_cmds(cmds);
    return true;
}

bool ArmController::move_axes_to(const std::vector<int>& target_slaves,
                                 const std::vector<double>& target_deg,
                                 int move_ms,
                                 std::string& error,
                                 bool print_progress) {
    error.clear();
    if (!bus_.is_operational()) {
        error = "运动会话未打开";
        return false;
    }
    if (target_slaves.empty() || target_slaves.size() != target_deg.size()) {
        error = "目标轴数量与目标角度数量不匹配";
        return false;
    }

    std::vector<PdoAxisFeedback> fb_snap;
    BusDiagnostics diag;
    bus_.snapshot_feedback(fb_snap, diag);
    if (!check_enabled_health(target_slaves, fb_snap, diag, "运动前", error)) {
        return false;
    }

    std::vector<std::int32_t> start_pulses(bus_.slave_count() + 1, 0);
    std::vector<std::int32_t> target_pulses(bus_.slave_count() + 1, 0);
    for (std::size_t idx = 0; idx < target_slaves.size(); ++idx) {
        const int s = target_slaves[idx];
        start_pulses[s] = fb_snap[s].actual_position;
        target_pulses[s] = static_cast<std::int32_t>(std::llround(target_deg[idx] * axes_[s - 1].counts_per_deg));
    }

    if (move_ms < 50) move_ms = 50;

    std::vector<PdoAxisCmd> cmds(bus_.slave_count() + 1);
    for (int s : eligible_servo_slaves()) cmds[s] = {CW_SHUTDOWN, 0, true};
    for (int s : motion_enabled_slaves_) cmds[s] = {CW_ENABLE_OPERATION, held_targets_[s], false};

    const auto start_tp = std::chrono::steady_clock::now();
    auto next_tick = start_tp;
    std::int64_t next_print_ms = 200;

    while (!stop_requested_.load()) {
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_tp).count();
        if (elapsed_ms >= move_ms) break;

        const double t = static_cast<double>(elapsed_ms) / static_cast<double>(move_ms);
        const double smooth = smoothstep5(t);

        for (int s : target_slaves) {
            const std::int64_t delta = static_cast<std::int64_t>(target_pulses[s]) -
                                       static_cast<std::int64_t>(start_pulses[s]);
            const std::int32_t cmd_pos = start_pulses[s] +
                static_cast<std::int32_t>(std::llround(static_cast<double>(delta) * smooth));
            cmds[s] = {CW_ENABLE_OPERATION, cmd_pos, false};
        }
        bus_.set_all_cmds(cmds);

        bus_.snapshot_feedback(fb_snap, diag);
        if (!check_enabled_health(target_slaves, fb_snap, diag, "运动中", error)) {
            return false;
        }

        if (print_progress && elapsed_ms >= next_print_ms) {
            next_print_ms += 200;
            std::cout << "[JOG] T=" << std::setw(5) << elapsed_ms << "/" << move_ms << "ms"
                      << " | WKC=" << diag.last_wkc << " | ";
            for (int s : target_slaves) {
                const double cur_d = static_cast<double>(fb_snap[s].actual_position) / axes_[s - 1].counts_per_deg;
                std::cout << axes_[s - 1].axis_name << ":" << std::fixed << std::setprecision(3) << cur_d << "° ";
            }
            std::cout << "\n";
        }

        next_tick += std::chrono::milliseconds(2);
        std::this_thread::sleep_until(next_tick);
    }

    if (stop_requested_.load()) {
        error = "收到停止信号";
        return false;
    }

    for (int s : target_slaves) {
        held_targets_[s] = target_pulses[s];
        cmds[s] = {CW_ENABLE_OPERATION, target_pulses[s], false};
    }
    bus_.set_all_cmds(cmds);
    return true;
}

bool ArmController::hold_motion(int hold_ms, std::string& error) {
    error.clear();
    if (!bus_.is_operational()) return true;
    const auto start_tp = std::chrono::steady_clock::now();
    auto next_tick = start_tp;
    std::vector<PdoAxisFeedback> fb_snap;
    BusDiagnostics diag;
    while (!stop_requested_.load()) {
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_tp).count();
        if (elapsed_ms >= hold_ms) break;

        bus_.snapshot_feedback(fb_snap, diag);
        if (!check_enabled_health(motion_enabled_slaves_, fb_snap, diag, "保位期间", error)) {
            return false;
        }
        next_tick += std::chrono::milliseconds(2);
        std::this_thread::sleep_until(next_tick);
    }
    return !stop_requested_.load();
}

bool ArmController::read_motion_feedback(const std::vector<int>& target_slaves,
                                         std::vector<AxisMotionFeedback>& out,
                                         BusDiagnostics& diag,
                                         std::string& error) {
    out.clear();
    error.clear();
    if (!bus_.is_operational()) {
        error = "运动会话未打开";
        return false;
    }
    std::vector<PdoAxisFeedback> fb_snap;
    bus_.snapshot_feedback(fb_snap, diag);
    if (!diag.healthy) {
        error = "PDO 通信异常";
        return false;
    }
    for (int s : target_slaves) {
        if (s < 1 || s > bus_.slave_count()) {
            error = "从站越界";
            return false;
        }
        const auto& ax = axes_[s - 1];
        const auto& fb = fb_snap[s];
        AxisMotionFeedback item;
        item.slave_index = s;
        item.axis_name = ax.axis_name;
        item.position_counts = fb.actual_position;
        item.position_deg = (ax.counts_per_deg > 0.0)
            ? static_cast<double>(fb.actual_position) / ax.counts_per_deg : 0.0;
        item.velocity_counts = fb.actual_velocity;
        item.torque_raw = fb.actual_torque;
        item.status_word = fb.status_word;
        item.error_code = fb.error_code;
        out.push_back(std::move(item));
    }
    return true;
}

void ArmController::close_motion() {
    if (bus_.is_operational()) {
        bus_.shutdown_operational();
    }
    motion_enabled_slaves_.clear();
    held_targets_.clear();
}

bool ArmController::run_single_axis_jog(int target_slave, double angle_deg, bool is_relative,
                                        int duration_ms, const std::string& title) {
    if (!bus_.is_initialized() && !init_bus()) return false;
    if (target_slave < 1 || target_slave > bus_.slave_count() || !axes_[target_slave - 1].is_servo) {
        std::cerr << "[错误] 目标从站 " << target_slave << " 无效或不是受支持的 CiA-402 伺服轴。\n";
        return false;
    }

    const AxisStatus& target = axes_[target_slave - 1];
    std::cout << "\n========================================================================================\n"
              << "  " << title << "\n"
              << "========================================================================================\n"
              << "目标关节: " << target.axis_name << " (从站 " << target_slave << ", " << target.model_name << ")\n"
              << "脉冲换算: " << std::fixed << std::setprecision(4) << target.counts_per_deg << " counts/deg\n";

    std::string err;
    if (!open_motion({target_slave}, err)) {
        std::cerr << "[错误] " << err << '\n';
        return false;
    }

    std::vector<AxisMotionFeedback> fb_before;
    BusDiagnostics diag;
    if (!read_motion_feedback({target_slave}, fb_before, diag, err)) {
        std::cerr << "[错误] " << err << '\n';
        close_motion();
        return false;
    }
    const double start_deg = fb_before[0].position_deg;
    const std::int32_t start_pulse = fb_before[0].position_counts;
    const double target_deg = is_relative ? (start_deg + angle_deg) : angle_deg;
    const std::int32_t target_pulse = static_cast<std::int32_t>(std::llround(target_deg * target.counts_per_deg));

    const int move_ms = std::max(1000, duration_ms * 4 / 5);
    const int hold_ms = std::max(500, duration_ms - move_ms);

    bool ok = move_axes_to({target_slave}, {target_deg}, move_ms, err, true);
    if (ok) ok = hold_motion(hold_ms, err);
    if (!ok && !err.empty()) {
        std::cerr << "[错误] " << err << '\n';
    }

    std::vector<AxisMotionFeedback> fb_after;
    std::string unused;
    read_motion_feedback({target_slave}, fb_after, diag, unused);
    close_motion();

    const std::int32_t final_pulse = fb_after.empty() ? start_pulse : fb_after[0].position_counts;
    const double final_deg = static_cast<double>(final_pulse) / target.counts_per_deg;
    const std::int32_t err_pulse = target_pulse - final_pulse;
    std::cout << "\n----------------------------------------------------------------------------------------\n"
              << "  起始位置: " << start_pulse << " (" << std::fixed << std::setprecision(4) << start_deg << "°)\n"
              << "  设定目标: " << target_pulse << " (" << target_deg << "°)\n"
              << "  最终实测: " << final_pulse << " (" << final_deg << "°)\n"
              << "  稳态误差: " << err_pulse << " 脉冲 ("
              << (static_cast<double>(err_pulse) / target.counts_per_deg) << "°)\n"
              << "----------------------------------------------------------------------------------------\n\n";
    return ok;
}

bool ArmController::run_jog(int target_slave, double delta_deg, int duration_ms) {
    return run_single_axis_jog(target_slave, delta_deg, true, duration_ms,
                               "单轴相对平滑点动 (jog: 到达目标后保持)");
}

bool ArmController::run_jog_abs(int target_slave, double abs_deg, int duration_ms) {
    return run_single_axis_jog(target_slave, abs_deg, false, duration_ms,
                               "单轴绝对角度平滑点动 (jog-abs: 到达目标后保持)");
}

bool ArmController::run_zero(int target_slave, int duration_ms) {
    return run_single_axis_jog(target_slave, 0.0, false, duration_ms,
                               "单轴平滑归零 (zero: 回到 0.000°)");
}

} // namespace robot_maint
