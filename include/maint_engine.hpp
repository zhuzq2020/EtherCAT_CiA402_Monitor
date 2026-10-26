#pragma once

#include "tongchuan_def.hpp"

#include <string>
#include <vector>
#include <cstdint>
#include <atomic>

extern "C" {
#include <soem/soem.h>
}

namespace robot_maint {

struct AxisStatus {
    int slave_index{0};
    std::uint16_t station_alias{0};
    std::string axis_name;
    std::string ethercat_state;
    std::uint16_t status_word{0};
    std::string cia402_state;
    bool is_fault{false};
    std::uint16_t error_code{0};
    std::string error_desc;
    std::int32_t actual_position{0};
    std::int32_t actual_velocity{0};
    std::uint32_t vendor_id{0};
    std::string vendor_name;
    std::uint32_t product_code{0};
};

class MaintEngine {
public:
    explicit MaintEngine(std::string iface);
    ~MaintEngine();

    MaintEngine(const MaintEngine&) = delete;
    MaintEngine& operator=(const MaintEngine&) = delete;

    // 初始化 Raw Socket 并扫描从站至 PRE-OP 状态
    bool init();
    void close();

    int slave_count() const noexcept { return slave_count_; }
    const std::string& interface_name() const noexcept { return iface_; }

    // 扫描获取所有轴的状态
    std::vector<AxisStatus> scan_all();

    // 单轴清错 (发送 0x6040 = 0x0080 脉冲复位)
    bool clear_fault(int slave_index, std::string& detail);

    // 单轴标零 (发送 0x3801 = 5，延时并回读校验)
    bool set_zero(int slave_index, std::string& detail);

    // 快速读取单轴实际位置
    bool read_position(int slave_index, std::int32_t& pos);

    // 读取任意 CoE 对象，保留原始字节，供调用方按对象类型解释。
    bool read_sdo_bytes(int slave_index, std::uint16_t index,
                        std::uint8_t subindex, std::vector<std::uint8_t>& value);

    // 将 Hot Connect 组首 Alias 写入任意 EtherCAT 从站 SII EEPROM。
    // 写入前完整备份 EEPROM，且仅允许从 0 初次配置，避免覆盖已有身份。
    bool write_station_alias(int slave_index, std::uint16_t alias,
                             const std::string& backup_path, std::string& detail);

    // PDO 使能并保持当前位置，直到收到停止请求后失能退出。
    bool enable_axis(int slave_index, std::string& detail);
    // SDO Shutdown，并回读确认退出 Operation Enabled。
    bool disable_axis(int slave_index, std::string& detail);

    // 点动微调 (JOG): 按角度微调 (delta_deg) 或按脉冲微调 (delta_counts)
    bool jog_axis_deg(int slave_index, double delta_deg, double speed_rpm, std::string& detail);
    bool jog_axis_counts(int slave_index, std::int32_t delta_counts, double speed_rpm, std::string& detail);

    // 全局停止请求 (SIGINT 拦截)
    static void request_stop() noexcept;

    // 底层通用 SDO 读写模板
    template <typename T>
    bool read_sdo(int slave, std::uint16_t index, std::uint8_t subindex, T& value) {
        int size = sizeof(T);
        const int rc = ecx_SDOread(&context_, slave, index, subindex, FALSE,
                                   &size, &value, EC_TIMEOUTRXM);
        return rc > 0 && size == sizeof(T);
    }

    template <typename T>
    bool write_sdo(int slave, std::uint16_t index, std::uint8_t subindex, const T& value) {
        int size = sizeof(T);
        auto val_copy = value;
        const int rc = ecx_SDOwrite(&context_, slave, index, subindex, FALSE,
                                    size, &val_copy, EC_TIMEOUTRXM);
        return rc > 0;
    }

private:
    bool run_position_control(int slave_index, std::int32_t delta_counts,
                              double speed_rpm, bool hold_enabled, std::string& detail);
    bool get_axis_ratio(int slave_index, double& counts_per_deg);
    bool configure_dynamic_pdo(int slave_index);

    std::string iface_;
    ecx_contextt context_{};
    int slave_count_{0};
    bool initialized_{false};
    uint8_t iomap_[4096]{};

    static std::atomic<bool> stop_requested_;
};

} // namespace robot_maint
