#include "drive_profile.hpp"
#include "cia402_defs.hpp"
#include "soem_bus.hpp"

#include <chrono>
#include <cstdio>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <thread>

namespace robot_maint {

namespace {

// In-memory registry of loaded profiles
std::map<std::uint32_t, ArmVendorProfile> g_profiles;
bool g_profiles_loaded = false;

std::string trim(const std::string& str) {
    const auto first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    const auto last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, (last - first + 1));
}

// Minimalistic robust JSON value extractor for clean key-value parsing without external heavy deps
bool extract_json_string(const std::string& line, const std::string& key, std::string& val) {
    const std::string pattern = "\"" + key + "\"";
    auto p = line.find(pattern);
    if (p == std::string::npos) return false;
    p = line.find(':', p + pattern.size());
    if (p == std::string::npos) return false;
    auto start_q = line.find('"', p + 1);
    if (start_q == std::string::npos) return false;
    auto end_q = line.find('"', start_q + 1);
    if (end_q == std::string::npos) return false;
    val = line.substr(start_q + 1, end_q - start_q - 1);
    return true;
}

bool extract_json_bool(const std::string& line, const std::string& key, bool& val) {
    const std::string pattern = "\"" + key + "\"";
    auto p = line.find(pattern);
    if (p == std::string::npos) return false;
    p = line.find(':', p + pattern.size());
    if (p == std::string::npos) return false;
    if (line.find("true", p) != std::string::npos) {
        val = true;
        return true;
    }
    if (line.find("false", p) != std::string::npos) {
        val = false;
        return true;
    }
    return false;
}

bool extract_json_double(const std::string& line, const std::string& key, double& val) {
    const std::string pattern = "\"" + key + "\"";
    auto p = line.find(pattern);
    if (p == std::string::npos) return false;
    p = line.find(':', p + pattern.size());
    if (p == std::string::npos) return false;
    auto first = line.find_first_of("0123456789.-+", p + 1);
    if (first == std::string::npos) return false;
    auto last = line.find_first_not_of("0123456789.-+", first);
    std::string s = (last == std::string::npos) ? line.substr(first) : line.substr(first, last - first);
    try {
        val = std::stod(s);
        return true;
    } catch (...) {
        return false;
    }
}

bool extract_json_int(const std::string& line, const std::string& key, int& val) {
    double d = 0.0;
    if (extract_json_double(line, key, d)) {
        val = static_cast<int>(d);
        return true;
    }
    return false;
}

void init_builtin_profiles_fallback() {
    if (!g_profiles.empty()) return;

    // Fairino builtin
    ArmVendorProfile faao;
    faao.vendor_id = kFairinoVendorId;
    faao.vendor_name = "法奥 (Fairino)";
    faao.driver_type = "fairino";
    faao.fixed_csp_mode = true;
    faao.supports_set_zero = true;
    faao.zeroing_method = "controlword_bit13";
    faao.default_counts_per_rev = 1654784.0;
    faao.axes = {
        {1, "ART-70-100-M", 1654784.0},
        {2, "ART-70-100-M", 1654784.0},
        {3, "ART-70-80-M",  1982464.0},
        {4, "ART-70-80-M",  1982464.0},
        {5, "ART-70-50-M",  1654784.0},
        {6, "ART-70-50-M",  1654784.0},
        {7, "ART-70-50-M",  1654784.0}
    };
    g_profiles[faao.vendor_id] = faao;

    // Tongchuan builtin
    ArmVendorProfile tc;
    tc.vendor_id = kTongchuanVendorId;
    tc.vendor_name = "同川 (Tongchuan)";
    tc.driver_type = "tongchuan";
    tc.fixed_csp_mode = false;
    tc.supports_set_zero = true;
    tc.zeroing_method = "sdo_3801_val5";
    tc.default_counts_per_rev = 131072.0 * 101.0;
    g_profiles[tc.vendor_id] = tc;

    // Eyou builtin
    ArmVendorProfile ey;
    ey.vendor_id = kEyouVendorId;
    ey.vendor_name = "意优 (Eyou)";
    ey.driver_type = "eyou";
    ey.fixed_csp_mode = false;
    ey.supports_set_zero = true;
    ey.zeroing_method = "homing_6060_flash";
    ey.default_counts_per_rev = 131072.0 * 121.0;
    g_profiles[ey.vendor_id] = ey;
}

} // namespace

bool load_profile_from_file(const std::string& filepath, ArmVendorProfile& out_profile) {
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) return false;

    ArmVendorProfile prof;
    std::string line;
    bool in_axes_array = false;

    while (std::getline(ifs, line)) {
        line = trim(line);
        if (line.empty()) continue;

        if (line.find("\"axis_defaults\"") != std::string::npos) {
            in_axes_array = true;
            continue;
        }

        if (in_axes_array) {
            if (line.find(']') != std::string::npos) {
                in_axes_array = false;
                continue;
            }
            if (line.find('{') != std::string::npos) {
                AxisProfileDef axis;
                extract_json_int(line, "joint", axis.joint);
                extract_json_string(line, "model", axis.model);
                extract_json_double(line, "counts_per_rev", axis.counts_per_rev);
                if (axis.joint > 0) {
                    prof.axes.push_back(axis);
                }
            }
            continue;
        }

        std::string vid_str;
        if (extract_json_string(line, "vendor_id", vid_str)) {
            try {
                prof.vendor_id = static_cast<std::uint32_t>(std::stoul(vid_str, nullptr, 0));
            } catch (...) {}
        }
        extract_json_string(line, "vendor_name", prof.vendor_name);
        extract_json_string(line, "driver_type", prof.driver_type);
        extract_json_bool(line, "fixed_csp_mode", prof.fixed_csp_mode);
        extract_json_bool(line, "supports_set_zero", prof.supports_set_zero);
        extract_json_string(line, "zeroing_method", prof.zeroing_method);
        extract_json_double(line, "counts_per_rev", prof.default_counts_per_rev);
    }

    if (prof.vendor_id == 0) return false;
    out_profile = prof;
    return true;
}

bool load_profiles_from_dir(const std::string& dirpath) {
    DIR* d = opendir(dirpath.c_str());
    if (!d) return false;

    struct dirent* dir = nullptr;
    int count = 0;
    while ((dir = readdir(d)) != nullptr) {
        std::string name = dir->d_name;
        if (name.size() > 5 && name.substr(name.size() - 5) == ".json") {
            std::string full_path = dirpath + "/" + name;
            ArmVendorProfile prof;
            if (load_profile_from_file(full_path, prof)) {
                g_profiles[prof.vendor_id] = prof;
                count++;
            }
        }
    }
    closedir(d);
    g_profiles_loaded = true;
    return count > 0;
}

const ArmVendorProfile* get_vendor_profile(std::uint32_t vendor_id) {
    if (!g_profiles_loaded) {
        // Try common profile locations
        load_profiles_from_dir("profiles");
        load_profiles_from_dir("../profiles");
        load_profiles_from_dir("/home/zhuzq/workspace/EtherCAT_CiA402_Monitor/profiles");
        init_builtin_profiles_fallback();
        g_profiles_loaded = true;
    }

    auto it = g_profiles.find(vendor_id);
    if (it != g_profiles.end()) {
        return &it->second;
    }
    return nullptr;
}

DriveVendor classify_vendor(std::uint32_t vendor_id) {
    switch (vendor_id) {
        case kTongchuanVendorId: return DriveVendor::Tongchuan;
        case kEyouVendorId:      return DriveVendor::Eyou;
        case kFairinoVendorId:   return DriveVendor::Fairino;
        default:                 return DriveVendor::GenericCia402;
    }
}

std::string vendor_id_to_name(std::uint32_t vendor_id) {
    const auto* prof = get_vendor_profile(vendor_id);
    if (prof && !prof->vendor_name.empty()) {
        return prof->vendor_name;
    }
    switch (vendor_id) {
        case kTongchuanVendorId: return "同川 (Tongchuan)";
        case kEyouVendorId:      return "意优 (Eyou)";
        case kFairinoVendorId:   return "法奥 (Fairino)";
        default: {
            char buf[24];
            std::snprintf(buf, sizeof(buf), "0x%08X", vendor_id);
            return buf;
        }
    }
}

bool is_eligible_servo(SoemBus& bus, int slave) {
    if (slave < 1 || slave > bus.slave_count()) return false;
    const auto& s = bus.raw_slave(slave);
    if ((s.mbx_proto & ECT_MBXPROT_COE) == 0) return false;

    const std::uint32_t vendor = bus.slave_vendor_id(slave);
    if (vendor == kTongchuanVendorId || vendor == kEyouVendorId || vendor == kFairinoVendorId) {
        return true;
    }
    std::uint32_t device_type = 0;
    if (bus.read_sdo(slave, SDO_DEVICE_TYPE, 0, device_type)) {
        return (device_type & 0xFFFFU) == 402U;
    }
    return false;
}

DriveCapabilities inspect_drive_capabilities(SoemBus&, int slave,
                                             std::uint32_t vendor_id,
                                             std::uint32_t product_code,
                                             const std::string& dev_name) {
    DriveCapabilities caps;
    caps.vendor = classify_vendor(vendor_id);
    caps.vendor_name = vendor_id_to_name(vendor_id);

    const auto* prof = get_vendor_profile(vendor_id);
    if (prof) {
        caps.fixed_csp_mode = prof->fixed_csp_mode;
        caps.supports_set_zero = prof->supports_set_zero;
        for (const auto& a : prof->axes) {
            if (a.joint == slave) {
                caps.model_name = a.model;
                break;
            }
        }
    }

    if (caps.model_name.empty()) {
        switch (caps.vendor) {
            case DriveVendor::Fairino:
                if (product_code == 1 || slave == 1 || slave == 2) caps.model_name = "ART-70-100-M";
                else if (product_code == 2 || slave == 3 || slave == 4) caps.model_name = "ART-70-80-M";
                else caps.model_name = "ART-70-50-M";
                caps.fixed_csp_mode = true;
                caps.supports_set_zero = true;
                break;
            case DriveVendor::Tongchuan:
                caps.model_name = dev_name.empty() ? "Tongchuan-Drive" : dev_name;
                caps.fixed_csp_mode = false;
                caps.supports_set_zero = true;
                break;
            case DriveVendor::Eyou:
                caps.model_name = dev_name.empty() ? "Eyou-Drive" : dev_name;
                caps.fixed_csp_mode = false;
                caps.supports_set_zero = true;
                break;
            default:
                caps.model_name = dev_name.empty() ? "CiA402-Drive" : dev_name;
                caps.fixed_csp_mode = false;
                caps.supports_set_zero = false;
                break;
        }
    }

    return caps;
}

double resolve_counts_per_rev(SoemBus& bus, int slave, std::uint32_t vendor_id) {
    const auto* prof = get_vendor_profile(vendor_id);
    if (prof) {
        for (const auto& a : prof->axes) {
            if (a.joint == slave && a.counts_per_rev > 0.0) {
                return a.counts_per_rev;
            }
        }
    }

    if (vendor_id == kFairinoVendorId) {
        std::uint32_t enc_res = 0, gear_num = 0, gear_den = 0;
        const bool has_res = bus.read_sdo(slave, SDO_POSITION_UNITS_PER_REV, 1, enc_res) && enc_res > 0;
        const bool has_gear = bus.read_sdo(slave, SDO_GEAR_RATIO_COE, 1, gear_num) &&
                              bus.read_sdo(slave, SDO_GEAR_RATIO_COE, 2, gear_den) &&
                              gear_num > 0 && gear_den > 0;
        if (has_res && has_gear) {
            return static_cast<double>(enc_res) * static_cast<double>(gear_num) / static_cast<double>(gear_den);
        }
        if (has_res) return static_cast<double>(enc_res);
        return (slave == 3 || slave == 4) ? 1982464.0 : 1654784.0;
    }

    if (vendor_id == kEyouVendorId) {
        std::uint32_t units = 0;
        if (!bus.read_sdo(slave, SDO_POSITION_UNITS_PER_REV, 1, units) || units == 0) {
            units = 131072U;
        }
        double gear = 121.0;
        std::uint32_t gear_num = 0, gear_den = 0;
        if (bus.read_sdo(slave, SDO_GEAR_RATIO_COE, 1, gear_num) &&
            bus.read_sdo(slave, SDO_GEAR_RATIO_COE, 2, gear_den) &&
            gear_num > 0 && gear_den > 0) {
            gear = static_cast<double>(gear_num) / static_cast<double>(gear_den);
        }
        return static_cast<double>(units) * gear;
    }

    if (vendor_id == kTongchuanVendorId) {
        std::uint16_t closed_loop = 0;
        bus.read_sdo(slave, SDO_TC_CLOSED_LOOP_TYPE, 0, closed_loop);

        std::uint16_t gear = 101;
        if (!bus.read_sdo(slave, SDO_TC_GEAR_RATIO, 0, gear) || gear == 0) {
            gear = 101;
        }
        if (closed_loop == 1) {
            std::uint16_t out_res_div4 = 0;
            if (bus.read_sdo(slave, SDO_TC_OUTPUT_RES_DIV4, 0, out_res_div4) && out_res_div4 > 0) {
                return static_cast<double>(out_res_div4) * 4.0;
            }
        }
        return 131072.0 * static_cast<double>(gear);
    }

    std::uint32_t units = 0, gear_num = 0, gear_den = 0;
    if (!bus.read_sdo(slave, SDO_POSITION_UNITS_PER_REV, 1, units) || units == 0) {
        units = 131072U;
    }
    double gear = 1.0;
    if (bus.read_sdo(slave, SDO_GEAR_RATIO_COE, 1, gear_num) &&
        bus.read_sdo(slave, SDO_GEAR_RATIO_COE, 2, gear_den) &&
        gear_num > 0 && gear_den > 0) {
        gear = static_cast<double>(gear_num) / static_cast<double>(gear_den);
    }
    return static_cast<double>(units) * gear;
}

bool drive_set_zero(SoemBus& bus, int slave, std::string& detail) {
    if (!bus.is_initialized() || slave < 1 || slave > bus.slave_count()) {
        detail = "从站索引超出范围";
        return false;
    }

    std::uint16_t status_word = 0;
    if (bus.read_sdo(slave, SDO_STATUS_WORD, 0, status_word) && is_operation_enabled(status_word)) {
        detail = "安全拦截：该轴处于 OperationEnabled (已使能) 状态，禁止在运动使能态标零！";
        return false;
    }

    const std::uint32_t vendor_id = bus.slave_vendor_id(slave);

    if (vendor_id == kFairinoVendorId) {
        // 法奥 (Fairino) 官方零点标定协议:
        // 1. 确认关节处于去使能 (bit2=0) 及非 OP 状态；将 0x6040 写入 0x0000 确保去使能
        const std::uint16_t cw_disable = 0x0000U;
        bus.write_sdo(slave, SDO_CONTROL_WORD, 0, cw_disable);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 2. 将控制字 0x6040 的 bit13 置 1 触发校零 (0x2000)
        const std::uint16_t cw_zero_trig = 0x2000U;
        if (!bus.write_sdo(slave, SDO_CONTROL_WORD, 0, cw_zero_trig)) {
            detail = "法奥驱动: 写入控制字 0x6040:00 = 0x2000 触发校零失败";
            return false;
        }

        // 3. 读取状态字 0x6041，等待 bit13 变为 1 (校零完成)
        bool zero_done = false;
        for (int retry = 0; retry < 30; ++retry) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::uint16_t sw = 0;
            if (bus.read_sdo(slave, SDO_STATUS_WORD, 0, sw)) {
                if ((sw & 0x2000U) != 0) { // bit13 == 1: 已校零
                    zero_done = true;
                    break;
                }
            }
        }

        // 4. 将 0x6040 的 bit13 写回 0
        const std::uint16_t cw_clear = 0x0000U;
        bus.write_sdo(slave, SDO_CONTROL_WORD, 0, cw_clear);

        if (!zero_done) {
            detail = "法奥驱动: 触发校零后状态字 0x6041 bit13 未能在超时内置 1";
            return false;
        }

        // 5. 将参数保存到 Flash (0x2020:01 写入 2，根据法奥用户手册基本设定参数命令)
        const std::int32_t save_flash_cmd = 2;
        bus.write_sdo(slave, 0x2020, 1, save_flash_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } else if (vendor_id == kEyouVendorId) {
        const std::int8_t homing_mode = 6;
        if (!bus.write_sdo(slave, SDO_MODE_OF_OPERATION, 0, homing_mode)) {
            detail = "意优驱动: 写入回零模式 0x6060:00 = 6 失败";
            return false;
        }
        const std::uint16_t trigger = 0x0010U;
        if (!bus.write_sdo(slave, SDO_CONTROL_WORD, 0, trigger)) {
            detail = "意优驱动: 写入控制字 0x6040:00 = 0x0010 失败";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        const std::uint8_t save = 1;
        if (!bus.write_sdo(slave, SDO_EYOU_SAVE_FLASH, 0, save)) {
            detail = "意优驱动: 写入 Flash 保存对象 0x2130:00 = 1 失败";
            return false;
        }
        const std::int8_t csp_mode = 8;
        bus.write_sdo(slave, SDO_MODE_OF_OPERATION, 0, csp_mode);
    } else {
        const std::uint16_t cmd_zero = 5U;
        if (!bus.write_sdo(slave, SDO_TC_SET_ZERO, 0, cmd_zero)) {
            detail = "向标零对象 0x3801:00 写入 5 失败 (厂商: " + vendor_id_to_name(vendor_id) + ")";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::uint16_t readback = 0xFFFFU;
        if (!bus.read_sdo(slave, SDO_TC_SET_ZERO, 0, readback)) {
            detail = "回读 0x3801:00 失败";
            return false;
        }
        if (readback != 0 && readback != 5) {
            detail = "标零回读校验异常 (预期 0 或 5，实际返回: " + std::to_string(readback) + ")";
            return false;
        }
    }

    std::int32_t act_pos = 999999;
    bus.read_sdo(slave, SDO_ACTUAL_POSITION, 0, act_pos);
    detail = "标零完成 [" + vendor_id_to_name(vendor_id) + "]！当前位置脉冲: " + std::to_string(act_pos);
    return true;
}

} // namespace robot_maint
