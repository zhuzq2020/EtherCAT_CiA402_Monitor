#pragma once

#include <cstdint>
#include <string>

namespace robot_maint {

// 设备与厂商参数
constexpr std::uint32_t kTongchuanVendorId = 0x00075500U;
constexpr std::uint32_t kTongchuanProductCode = 0x00000002U;
constexpr std::uint32_t kEyouVendorId = 0x00001097U;
constexpr std::uint32_t kEyouProductCode = 0x00010002U;
constexpr std::size_t kArmAxes = 7;

// CiA-402 标准与专用 CoE SDO 索引
constexpr std::uint16_t SDO_CONTROL_WORD           = 0x6040U; // 控制字 (uint16)
constexpr std::uint16_t SDO_STATUS_WORD            = 0x6041U; // 状态字 (uint16)
constexpr std::uint16_t SDO_MODE_OF_OPERATION      = 0x6060U; // 运行模式下发 (int8)
constexpr std::uint16_t SDO_MODE_DISPLAY           = 0x6061U; // 当前运行模式显示 (int8)
constexpr std::uint16_t SDO_ACTUAL_POSITION        = 0x6064U; // 编码器实际脉冲 (int32)
constexpr std::uint16_t SDO_ACTUAL_VELOCITY        = 0x606CU; // 实际速度 (int32)
constexpr std::uint16_t SDO_ERROR_CODE             = 0x603FU; // 故障报警码 (uint16)
constexpr std::uint16_t SDO_ERROR_REGISTER         = 0x1001U; // 错误寄存器 (uint8)
constexpr std::uint16_t SDO_POSITION_UNITS_PER_REV = 0x608FU; // 编码器单圈分辨率 (Sub 1: uint32)
constexpr std::uint16_t SDO_GEAR_RATIO_COE         = 0x6091U; // 标准电子齿轮比 (Sub 1:分子, Sub 2:分母)

// 同川专用标定参数: Pr8.01 (写入 5 表示将当前物理位置标定为多圈绝对零点)
constexpr std::uint16_t SDO_TONGCHUAN_SET_ZERO     = 0x3801U;

// 意优专用参数: 0x2130 (写入 1 表示保存标定参数到 EEPROM/Flash)
constexpr std::uint16_t SDO_EYOU_SAVE_FLASH        = 0x2130U;

// 同川专有减速比与编码器参数
constexpr std::uint16_t SDO_CLOSED_LOOP_TYPE  = 0x3D00U; // PrD.00 (0:电机编码器, 1:输出端编码器)
constexpr std::uint16_t SDO_OUTPUT_RESOLUTION = 0x3D04U; // PrD.04 (输出端编码器分辨率/4)
constexpr std::uint16_t SDO_GEAR_RATIO        = 0x3D06U; // PrD.06 (减速比，如 101/81/51)

// 厂商识别字符串
inline std::string vendor_id_to_name(std::uint32_t vendor_id) {
    switch (vendor_id) {
        case kTongchuanVendorId: return "同川 (Tongchuan)";
        case kEyouVendorId:      return "意优 (Eyou)";
        default: {
            char buf[16];
            std::snprintf(buf, sizeof(buf), "0x%08X", vendor_id);
            return buf;
        }
    }
}

// 五次多项式平滑插值曲线: 0 -> 1，两端一阶和二阶导数均为0 (无冲击)
inline double smoothstep5(double t) noexcept {
    if (t <= 0.0) return 0.0;
    if (t >= 1.0) return 1.0;
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
}

// 从站与轴映射 (支持右臂 7 轴 D18..D24 及通用 S1..SN)
inline std::string slave_to_axis_name(int slave_index, int total_slaves = 7) {
    if (total_slaves == 7 && slave_index >= 1 && slave_index <= 7) {
        return "D" + std::to_string(17 + slave_index);
    }
    return "S" + std::to_string(slave_index);
}

// 支持 D18..D24、S1..SN、或纯数字 1..N 灵活解析
inline int axis_name_to_slave(const std::string& axis_name, int total_slaves = 7) {
    if (axis_name.empty()) return -1;

    // 纯数字格式: "1", "2", ...
    try {
        bool all_digits = true;
        for (char c : axis_name) {
            if (!std::isdigit(static_cast<unsigned char>(c))) {
                all_digits = false;
                break;
            }
        }
        if (all_digits) {
            int n = std::stoi(axis_name);
            if (n >= 1 && n <= 64) return n;
        }
    } catch (...) {}

    // "S1", "s2", ... 从站索引格式
    if ((axis_name[0] == 'S' || axis_name[0] == 's') && axis_name.size() > 1) {
        try {
            int n = std::stoi(axis_name.substr(1));
            if (n >= 1 && n <= 64) return n;
        } catch (...) {}
    }

    // "D18".."D24" 或 "D1".."D24" 机器人关节格式
    if ((axis_name[0] == 'D' || axis_name[0] == 'd') && axis_name.size() > 1) {
        try {
            int num = std::stoi(axis_name.substr(1));
            if (total_slaves <= 7 && num >= 18 && num <= 24) {
                return num - 17; // 单独测试右臂 7 轴时，D18 对应物理从站 1
            }
            if (num >= 1 && num <= 64) {
                return num;
            }
        } catch (...) {}
    }
    return -1;
}

// CiA-402 状态解码
inline std::string decode_cia402_state(std::uint16_t sw) {
    if ((sw & 0x0008U) != 0) return "FAULT (故障报警)";
    switch (sw & 0x006fU) {
        case 0x0000U:
        case 0x0020U: return "NotReadyToSwitchOn";
        case 0x0040U: return "SwitchOnDisabled (下使能)";
        case 0x0021U: return "ReadyToSwitchOn (就绪)";
        case 0x0023U: return "SwitchedOn (已通电)";
        case 0x0027U: return "OperationEnabled (已使能)";
        case 0x0007U: return "QuickStopActive";
        case 0x000fU: return "FaultReactionActive";
        default: return "Unknown";
    }
}

// 常见故障码描述
inline std::string describe_error_code(std::uint16_t err) {
    if (err == 0) return "无故障";
    switch (err) {
        case 0x2310: return "过流报警 (Over-current)";
        case 0x3210: return "母线过压 (Over-voltage)";
        case 0x3220: return "母线欠压 (Under-voltage)";
        case 0x4310: return "驱动过温 (Over-temperature)";
        case 0x7305: return "编码器故障";
        case 0x7306: return "编码器通信断线";
        case 0x8611: return "跟随误差过大 (Position error)";
        default: return "驱动器报警 (见手册)";
    }
}

} // namespace robot_maint
