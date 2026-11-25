#pragma once

#include <cctype>
#include <cstdint>
#include <iomanip>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace robot_maint {

// Standard CiA-402 / CoE SDO Indices
constexpr std::uint16_t SDO_DEVICE_TYPE             = 0x1000U; // uint32
constexpr std::uint16_t SDO_ERROR_REGISTER          = 0x1001U; // uint8
constexpr std::uint16_t SDO_DEVICE_NAME             = 0x1008U; // string
constexpr std::uint16_t SDO_HARDWARE_VERSION        = 0x1009U; // string
constexpr std::uint16_t SDO_SOFTWARE_VERSION        = 0x100AU; // string
constexpr std::uint16_t SDO_IDENTITY_OBJECT         = 0x1018U; // Sub 1: Vendor, 2: Product, 3: Rev, 4: Serial
constexpr std::uint16_t SDO_ERROR_CODE              = 0x603FU; // uint16
constexpr std::uint16_t SDO_CONTROL_WORD            = 0x6040U; // uint16
constexpr std::uint16_t SDO_STATUS_WORD             = 0x6041U; // uint16
constexpr std::uint16_t SDO_MODE_OF_OPERATION       = 0x6060U; // int8
constexpr std::uint16_t SDO_MODE_DISPLAY            = 0x6061U; // int8
constexpr std::uint16_t SDO_ACTUAL_POSITION         = 0x6064U; // int32
constexpr std::uint16_t SDO_FOLLOWING_ERROR_WINDOW  = 0x6065U; // uint32
constexpr std::uint16_t SDO_ACTUAL_VELOCITY         = 0x606CU; // int32
constexpr std::uint16_t SDO_TARGET_TORQUE           = 0x6071U; // int16
constexpr std::uint16_t SDO_ACTUAL_TORQUE           = 0x6077U; // int16
constexpr std::uint16_t SDO_TARGET_POSITION         = 0x607AU; // int32
constexpr std::uint16_t SDO_SOFTWARE_POSITION_LIMIT = 0x607DU; // Sub 1: min (int32), Sub 2: max (int32)
constexpr std::uint16_t SDO_POSITION_UNITS_PER_REV  = 0x608FU; // Sub 1: uint32 counts/rev
constexpr std::uint16_t SDO_GEAR_RATIO_COE          = 0x6091U; // Sub 1: num, Sub 2: den
constexpr std::uint16_t SDO_VELOCITY_OFFSET         = 0x60B1U; // int32

// CiA-402 Controlword Commands
constexpr std::uint16_t CW_SHUTDOWN                 = 0x0006U;
constexpr std::uint16_t CW_SWITCH_ON                = 0x0007U;
constexpr std::uint16_t CW_ENABLE_OPERATION         = 0x000FU;
constexpr std::uint16_t CW_QUICK_STOP               = 0x0002U;
constexpr std::uint16_t CW_FAULT_RESET              = 0x0080U;

// CiA-402 Statusword Masks & States
constexpr std::uint16_t SW_MASK_STATE               = 0x006FU;
constexpr std::uint16_t SW_NOT_READY                = 0x0000U;
constexpr std::uint16_t SW_SWITCH_ON_DISABLED       = 0x0040U;
constexpr std::uint16_t SW_READY_TO_SWITCH_ON       = 0x0021U;
constexpr std::uint16_t SW_SWITCHED_ON              = 0x0023U;
constexpr std::uint16_t SW_OPERATION_ENABLED        = 0x0027U;
constexpr std::uint16_t SW_QUICK_STOP_ACTIVE        = 0x0007U;
constexpr std::uint16_t SW_FAULT_REACTION_ACTIVE    = 0x000FU;
constexpr std::uint16_t SW_FAULT_BIT                = 0x0008U;

// Shared PDO group metadata struct
struct MotionAxisInfo {
    std::string name;
    std::string unavailable;
    double counts_per_deg{0.0};
};

// 5th-order smoothstep polynomial: s(t) in [0, 1] for t in [0, 1],
// with zero first and second derivatives at endpoints.
inline double smoothstep5(double t) noexcept {
    if (t <= 0.0) return 0.0;
    if (t >= 1.0) return 1.0;
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
}

inline bool is_operation_enabled(std::uint16_t sw) noexcept {
    return (sw & SW_MASK_STATE) == SW_OPERATION_ENABLED;
}

inline bool is_disabled_state(std::uint16_t sw) noexcept {
    return (sw & SW_MASK_STATE) == SW_READY_TO_SWITCH_ON ||
           (sw & 0x004FU) == SW_SWITCH_ON_DISABLED ||
           (sw & 0x004FU) == SW_FAULT_BIT ||
           (sw & 0x004FU) == SW_NOT_READY;
}

inline std::string decode_cia402_state(std::uint16_t sw) {
    if ((sw & SW_FAULT_BIT) != 0) {
        return "Fault (0x0008)";
    }
    switch (sw & SW_MASK_STATE) {
        case 0x0000U:
        case 0x0020U: return "Not Ready to Switch On";
        case 0x0040U: return "Switch On Disabled";
        case 0x0021U: return "Ready to Switch On";
        case 0x0023U: return "Switched On";
        case 0x0027U: return "Operation Enabled";
        case 0x0007U: return "Quick Stop Active";
        case 0x000FU: return "Fault Reaction Active";
        default: {
            std::ostringstream ss;
            ss << "State(0x" << std::hex << std::setw(4) << std::setfill('0') << sw << ")";
            return ss.str();
        }
    }
}

inline std::string describe_error_code(std::uint16_t err) {
    if (err == 0) return "无故障";
    switch (err) {
        case 0x001F: return "ESC看门狗通信中断 / 同步丢失 (0x001F)";
        case 0x2310: return "过流报警 (Over-current 0x2310)";
        case 0x2340: return "瞬间峰值过流 (Peak Current 0x2340)";
        case 0x3210: return "母线过压 (Over-voltage 0x3210)";
        case 0x3220: return "母线欠压 (Under-voltage 0x3220)";
        case 0x4310: return "驱动过温 (Over-temperature 0x4310)";
        case 0x7305: return "编码器故障 / 信号异常 (0x7305)";
        case 0x7306: return "编码器通信断线 (0x7306)";
        case 0x7500: return "EtherCAT 通信看门狗超时 (0x7500)";
        case 0x8611: return "跟随误差过大 (Following Error 0x8611)";
        case 0x8A00: return "抱闸控制故障 (Brake Error 0x8A00)";
        default: {
            std::ostringstream ss;
            ss << "驱动器报警 (0x" << std::hex << std::setw(4) << std::setfill('0') << err << ")";
            return ss.str();
        }
    }
}

// Slave index (1-based) to display label.
// Fairino uses J1..J7, 7-axis Tongchuan/Eyou uses D18..D24, generic uses S1..SN.
inline std::string slave_to_axis_name(int slave_index, int total_slaves = 7, bool prefer_j_prefix = false) {
    if (prefer_j_prefix) {
        return "J" + std::to_string(slave_index);
    }
    if (total_slaves == 7 && slave_index >= 1 && slave_index <= 7) {
        return "D" + std::to_string(17 + slave_index);
    }
    return "S" + std::to_string(slave_index);
}

// Parse user axis name (1..N, S1..SN, J1..JN, D18..D24) to 1-based slave index.
// Returns -1 on invalid input.
inline int axis_name_to_slave(const std::string& axis_name, int total_slaves = 7) {
    if (axis_name.empty()) return -1;

    // Pure digits: "1", "2", ...
    bool all_digits = true;
    for (unsigned char c : axis_name) {
        if (!std::isdigit(c)) {
            all_digits = false;
            break;
        }
    }
    if (all_digits) {
        try {
            const int n = std::stoi(axis_name);
            if (n >= 1 && n <= 64) return n;
        } catch (...) {}
        return -1;
    }

    const char prefix = axis_name[0];
    if (axis_name.size() < 2) return -1;
    for (std::size_t i = 1; i < axis_name.size(); ++i) {
        if (!std::isdigit(static_cast<unsigned char>(axis_name[i]))) return -1;
    }

    int num = -1;
    try {
        num = std::stoi(axis_name.substr(1));
    } catch (...) {
        return -1;
    }

    if (prefix == 'S' || prefix == 's' || prefix == 'J' || prefix == 'j') {
        if (num >= 1 && num <= 64) return num;
        return -1;
    }

    if (prefix == 'D' || prefix == 'd') {
        if (total_slaves <= 7 && num >= 18 && num <= 24) {
            return num - 17;
        }
        if (num >= 1 && num <= 64) return num;
    }

    return -1;
}

// Resolve a list of axis labels to unique 1-based slave indices.
inline bool resolve_slave_axes(const std::vector<std::string>& names, int total_slaves,
                               std::vector<int>& result, std::string& error) {
    result.clear();
    error.clear();
    if (names.empty()) {
        error = "未指定目标轴";
        return false;
    }
    std::set<int> unique;
    for (const auto& name : names) {
        const int slave = axis_name_to_slave(name, total_slaves);
        if (slave < 1 || slave > total_slaves) {
            if (!error.empty()) error += "；";
            error += name + " 不是本次扫描中的有效目标轴";
        } else if (!unique.insert(slave).second) {
            if (!error.empty()) error += "；";
            error += name + " 重复指定同一目标轴";
        } else {
            result.push_back(slave);
        }
    }
    return error.empty();
}

} // namespace robot_maint
