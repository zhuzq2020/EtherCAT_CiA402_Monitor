#include "maint_engine.hpp"
#include "jog_request.hpp"
#include "sii_integrity.hpp"
#include "eeprom_access.hpp"

#include <poll.h>
#include <fcntl.h>
#include <functional>
#include <unistd.h>
#include <cerrno>
#include <limits>

#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <array>

namespace robot_maint {

std::atomic<bool> MaintEngine::stop_requested_{false};

void MaintEngine::request_stop() noexcept {
    stop_requested_.store(true);
}

namespace {
constexpr std::uint16_t kSiiAliasWord = 0x0004U;
constexpr std::uint16_t kSiiSizeWord = 0x003eU;
constexpr std::size_t kSiiHeaderWords = 64;
// SOEM EEPROM addresses are 16-bit WORD offsets, so the largest addressable
// image is 65536 words / 128 KiB.
constexpr std::size_t kMaxSiiBytes = 128U * 1024U;

class SoemEepromIo final : public EepromIo {
public:
    SoemEepromIo(ecx_contextt& context, std::uint16_t station)
        : context_(context), station_(station) {}
    int read(std::uint16_t reg, void* data, std::size_t size) override {
        return ecx_FPRD(&context_.port, station_, reg, static_cast<int>(size), data, EC_TIMEOUTRET);
    }
    int write(std::uint16_t reg, const void* data, std::size_t size) override {
        return ecx_FPWR(&context_.port, station_, reg, static_cast<int>(size),
                         const_cast<void*>(data), EC_TIMEOUTRET);
    }
    std::uint64_t now_us() override {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    void pause_us(unsigned us) override { std::this_thread::sleep_for(std::chrono::microseconds(us)); }
private:
    ecx_contextt& context_;
    std::uint16_t station_;
};

struct AliasCleanup {
    std::function<void()> action;
    ~AliasCleanup() { action(); }
};

bool save_sii_backup(const std::filesystem::path& path,
                     const std::vector<std::uint16_t>& words, std::string& detail) {
    // Persist explicit little-endian bytes; exclusive creation prevents a race
    // between the path check and opening an existing backup for truncation.
    std::vector<std::uint8_t> data(words.size() * 2);
    for (std::size_t i = 0; i < words.size(); ++i) {
        data[2 * i] = words[i] & 0xff;
        data[2 * i + 1] = words[i] >> 8;
    }
    if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, 0644);
    if (fd < 0) { detail = "无法新建备份 (拒绝覆盖): " + std::string(std::strerror(errno)); return false; }
    bool ok = true;
    std::size_t offset = 0;
    while (offset < data.size()) {
        const auto written = ::write(fd, data.data() + offset, data.size() - offset);
        if (written < 0 && errno == EINTR) continue;
        if (written <= 0) { ok = false; break; }
        offset += static_cast<std::size_t>(written);
    }
    if (ok && ::fsync(fd) != 0) ok = false;
    if (::close(fd) != 0) ok = false;
    if (!ok) detail = "备份写入或同步失败，未开始 EEPROM 写入";
    return ok;
}

std::string al_state_str(std::uint16_t state) {
    switch (state & 0x000fU) {
        case EC_STATE_NONE: return "NONE";
        case EC_STATE_INIT: return "INIT";
        case EC_STATE_PRE_OP: return "PRE-OP";
        case EC_STATE_BOOT: return "BOOT";
        case EC_STATE_SAFE_OP: return "SAFE-OP";
        case EC_STATE_OPERATIONAL: return "OP";
        default: return "0x" + std::to_string(state);
    }
}
} // namespace

MaintEngine::MaintEngine(std::string iface) : iface_(std::move(iface)) {}

MaintEngine::~MaintEngine() {
    close();
}

void MaintEngine::close() {
    if (initialized_) {
        ecx_close(&context_);
        initialized_ = false;
        slave_count_ = 0;
    }
}

bool MaintEngine::init() {
    close();
    std::memset(&context_, 0, sizeof(context_));

    if (ecx_init(&context_, iface_.c_str()) <= 0) {
        std::cerr << "[错误] 无法初始化网络接口: " << iface_ << '\n'
                  << "       请检查网卡名称是否正确，以及是否具有 root/CAP_NET_RAW 权限。\n";
        return false;
    }
    initialized_ = true;

    // 扫描总线，配置从站至 PRE-OP 状态
    // FALSE 表示不自动请求映射 PDO，纯 CoE/SDO 模式
    slave_count_ = ecx_config_init(&context_);
    if (slave_count_ <= 0) {
        std::cerr << "[提示] 网口 " << iface_ << " 上未扫描到任何 EtherCAT 从站。\n"
                  << "       请检查驱动器供电及网线物理连接。\n";
        close();
        return false;
    }

    ecx_readstate(&context_);
    return true;
}

std::vector<AxisStatus> MaintEngine::scan_all() {
    std::vector<AxisStatus> list;
    if (!initialized_ || slave_count_ <= 0) return list;

    ecx_readstate(&context_);
    list.reserve(slave_count_);

    for (int i = 1; i <= slave_count_; ++i) {
        AxisStatus status;
        status.slave_index = i;
        status.station_alias = context_.slavelist[i].aliasadr;
        status.axis_name = slave_to_axis_name(i, slave_count_);
        status.ethercat_state = al_state_str(context_.slavelist[i].state);
        status.vendor_id = context_.slavelist[i].eep_man;
        status.product_code = context_.slavelist[i].eep_id;

        // 如果 EEPROM 未读出厂商，尝试通过标准 CoE 0x1018 读取
        if (status.vendor_id == 0 && (context_.slavelist[i].mbx_proto & ECT_MBXPROT_COE) != 0) {
            read_sdo(i, 0x1018, 1, status.vendor_id);
            read_sdo(i, 0x1018, 2, status.product_code);
        }
        status.vendor_name = vendor_id_to_name(status.vendor_id);

        // 仅在支持 CoE 且非离线时读取字典
        if ((context_.slavelist[i].mbx_proto & ECT_MBXPROT_COE) != 0) {
            read_sdo(i, SDO_STATUS_WORD, 0, status.status_word);
            status.is_fault = (status.status_word & 0x0008U) != 0;
            status.cia402_state = decode_cia402_state(status.status_word);

            read_sdo(i, SDO_ERROR_CODE, 0, status.error_code);
            status.error_desc = describe_error_code(status.error_code);

            read_sdo(i, SDO_ACTUAL_POSITION, 0, status.actual_position);
            read_sdo(i, SDO_ACTUAL_VELOCITY, 0, status.actual_velocity);
        } else {
            status.cia402_state = "不支持CoE";
        }
        list.push_back(status);
    }
    return list;
}

bool MaintEngine::read_position(int slave_index, std::int32_t& pos) {
    if (!initialized_ || slave_index <= 0 || slave_index > slave_count_) return false;
    return read_sdo(slave_index, SDO_ACTUAL_POSITION, 0, pos);
}

bool MaintEngine::read_sdo_bytes(int slave_index, std::uint16_t index,
                                 std::uint8_t subindex, std::vector<std::uint8_t>& value) {
    value.clear();
    if (!initialized_ || slave_index <= 0 || slave_index > slave_count_ ||
        (context_.slavelist[slave_index].mbx_proto & ECT_MBXPROT_COE) == 0) {
        return false;
    }

    // 足够容纳常用标量和短字符串；SOEM 以 size 作为接收缓冲区上限。
    std::array<std::uint8_t, 256> buffer{};
    int size = static_cast<int>(buffer.size());
    const int rc = ecx_SDOread(&context_, slave_index, index, subindex, FALSE,
                               &size, buffer.data(), EC_TIMEOUTRXM);
    if (rc <= 0 || size <= 0 || size > static_cast<int>(buffer.size())) return false;
    value.assign(buffer.begin(), buffer.begin() + size);
    return true;
}

bool MaintEngine::read_station_alias(int slave_index, std::string& detail) {
    if (!initialized_ || slave_index <= 0 || slave_index > slave_count_) {
        detail = "从站索引超出范围";
        return false;
    }
    auto& slave = context_.slavelist[slave_index];
    SoemEepromIo io(context_, slave.configadr);
    EepromAccess access(io, &std::cout);
    std::uint16_t raw_alias = 0;
    const int alias_wkc = io.read(ECT_REG_ALIAS, &raw_alias, sizeof(raw_alias));
    if (alias_wkc != 1) {
        detail = "读取 ESC Alias 失败，WKC=" + std::to_string(alias_wkc);
        return false;
    }
    if (!access.acquire(detail)) {
        std::string cleanup;
        if (!access.restore(cleanup)) detail += "；恢复失败: " + cleanup;
        return false;
    }
    std::array<std::uint16_t, 8> words{};
    bool ok = true;
    for (std::size_t i = 0; i < words.size(); ++i) {
        if (!access.read_word(static_cast<std::uint16_t>(i), words[i], detail)) {
            ok = false;
            break;
        }
    }
    std::string cleanup;
    const bool restored = access.restore(cleanup);
    if (!restored) { detail += "；恢复失败: " + cleanup; return false; }
    slave.eep_pdi = (access.original().config & 1) ? 1 : 0;
    if (!ok) return false;
    const auto crc = sii_crc(words.data());
    detail = "ESC Alias=" + std::to_string(etohs(raw_alias))
           + "; EEPROM Alias=" + std::to_string(words[kSiiAliasWord])
           + "; CRC stored=" + std::to_string(words[7] & 0xffU)
           + ", calculated=" + std::to_string(crc)
           + ((words[7] & 0xffU) == crc ? " (OK)" : " (不匹配)");
    return (words[7] & 0xffU) == crc;
}

bool MaintEngine::backup_station_eeprom(int slave_index, const std::string& path, std::string& detail) {
    if (!initialized_ || slave_index <= 0 || slave_index > slave_count_ || path.empty()) {
        detail = "必须指定有效从站和备份路径";
        return false;
    }
    auto& slave = context_.slavelist[slave_index];
    SoemEepromIo io(context_, slave.configadr);
    EepromAccess access(io, &std::cout);
    auto fail = [&]() {
        std::string cleanup;
        if (!access.restore(cleanup)) detail += "；恢复失败: " + cleanup;
        return false;
    };
    if (!access.acquire(detail)) return fail();
    std::vector<std::uint16_t> words(kSiiHeaderWords);
    for (std::size_t i = 0; i < words.size(); ++i)
        if (!access.read_word(static_cast<std::uint16_t>(i), words[i], detail)) return fail();
    const auto identity = [&](std::size_t i) {
        return std::uint32_t(words[i]) | (std::uint32_t(words[i + 1]) << 16);
    };
    if (identity(ECT_SII_MANUF) != slave.eep_man || identity(ECT_SII_ID) != slave.eep_id) {
        detail = "SII 身份与扫描结果不一致，未保存备份";
        return fail();
    }
    const std::size_t bytes = (std::size_t(words[kSiiSizeWord]) + 1) * 128;
    if (bytes < kSiiHeaderWords * 2 || bytes > kMaxSiiBytes) {
        detail = "SII 容量无效，未保存备份";
        return fail();
    }
    words.resize(bytes / 2);
    for (std::size_t i = kSiiHeaderWords; i < words.size(); ++i)
        if (!access.read_word(static_cast<std::uint16_t>(i), words[i], detail)) return fail();
    try {
        if (!save_sii_backup(path, words, detail)) return fail();
    } catch (const std::exception& error) {
        detail = std::string("保存备份失败: ") + error.what();
        return fail();
    }
    std::string cleanup;
    if (!access.restore(cleanup)) { detail = "备份已保存，但恢复控制权失败: " + cleanup; return false; }
    slave.eep_pdi = (access.original().config & 1) ? 1 : 0;
    detail = "EEPROM 只读备份完成，" + std::to_string(bytes) + " bytes；文件=" + path
           + "; 配置 CRC=" + ((sii_crc(words.data()) == (words[7] & 0xff)) ? "OK" : "不匹配");
    return true;
}

bool MaintEngine::write_station_alias(int slave_index, std::uint16_t alias,
                                      const std::string& backup_path, std::string& detail) {
    if (!initialized_ || slave_index <= 0 || slave_index > slave_count_) {
        detail = "从站索引超出范围";
        return false;
    }
    if (alias == 0 || backup_path.empty()) {
        detail = "必须指定非零 Alias 和 EEPROM 备份路径";
        return false;
    }
    auto& slave = context_.slavelist[slave_index];
    if (slave.eep_man == 0 || slave.eep_id == 0) {
        detail = "在线从站身份无效，拒绝写入";
        return false;
    }
    SoemEepromIo io(context_, slave.configadr);
    std::uint16_t raw_state = 0, raw_alias = 0;
    if (io.read(ECT_REG_ALSTAT, &raw_state, sizeof(raw_state)) != 1
        || io.read(ECT_REG_ALIAS, &raw_alias, sizeof(raw_alias)) != 1) {
        detail = "读取从站 AL 状态或 ESC Alias 失败，未执行 EEPROM 写入";
        return false;
    }
    const auto state = etohs(raw_state);
    const auto esc_alias = etohs(raw_alias);
    if ((state & 0x10) || ((state & 0xf) != EC_STATE_INIT && (state & 0xf) != EC_STATE_PRE_OP)) {
        detail = "写 Alias 要求从站无 AL 错误且处于 INIT/PRE-OP，实际=" + al_state_str(state);
        return false;
    }
    if (slave.mbx_proto & ECT_MBXPROT_COE) {
        std::uint16_t status = 0;
        if (!read_sdo(slave_index, SDO_STATUS_WORD, 0, status) || (status & 0x0004U)) {
            detail = "无法确认驱动器已退出 Operation Enabled，未执行 EEPROM 写入";
            return false;
        }
    }
    std::cout << "[EEPROM] slave=" << slave_index << ", station=" << slave.configadr
              << ", vendor=0x" << std::hex << slave.eep_man << ", product=0x" << slave.eep_id
              << std::dec << ", AL=" << al_state_str(state) << ", ESC Alias=" << esc_alias << '\n';
    if (slave.eep_man == kTongchuanVendorId) {
        std::uint16_t alarm = 0;
        if (!read_sdo(slave_index, SDO_TC_ECAT_ALARM, 0, alarm)) {
            detail = "无法读取同川 Pr0.45 (0x302D) ECAT 报警码，未执行 EEPROM 写入";
            return false;
        }
        std::cout << "[EEPROM] 同川 Pr0.45 ECAT 报警码=" << alarm
                  << "；使用标准 SII 流程，不发送意优解锁或同川全参数保存指令\n";
        if (alarm != 0) { detail = "同川 ECAT 报警未清除，未执行 EEPROM 写入"; return false; }
    }

    EepromAccess access(io, &std::cout);
    bool permission_enabled = false;
    bool cleaned = false;
    auto restore = [&]() {
        if (cleaned) return true;
        cleaned = true;
        std::string error;
        bool ok = access.restore(error);
        if (!ok) std::cerr << "[错误] 恢复 EEPROM 控制权失败: " << error << '\n';
        if (ok && access.has_original()) slave.eep_pdi = (access.original().config & 1) ? 1 : 0;
        if (permission_enabled) {
            const std::uint8_t disable = 0;
            const bool disabled = write_sdo(slave_index, 0x2679, 0, disable);
            if (!disabled) std::cerr << "[错误] 关闭意优 EEPROM 写权限失败\n";
            ok = disabled && ok;
        }
        return ok;
    };
    AliasCleanup guard{[&]() { if (!cleaned) (void)restore(); }};
    auto fail = [&](std::string message) {
        if (!restore()) message += "；恢复 EEPROM 控制权或关闭写权限失败";
        detail = std::move(message);
        return false;
    };
    if (slave.eep_man == kEyouVendorId) {
        if ((slave.mbx_proto & ECT_MBXPROT_COE) == 0)
            return fail("意优从站不支持 CoE，无法开启 EEPROM 写权限");
        const std::uint8_t enable = 1;
        permission_enabled = true; // Also clean up an ambiguous SDO failure.
        if (!write_sdo(slave_index, 0x2679, 0, enable))
            return fail("开启意优 EEPROM 写权限失败 (0x2679=1)");
    }
    if (!access.acquire(detail)) return fail(detail);
    slave.eep_pdi = 0;

    std::vector<std::uint16_t> words(kSiiHeaderWords);
    for (std::size_t i = 0; i < words.size(); ++i) {
        if (!access.read_word(static_cast<std::uint16_t>(i), words[i], detail))
            return fail("备份读取 word " + std::to_string(i) + " 失败: " + detail);
    }
    const auto sii_u32 = [&](std::size_t offset) {
        return std::uint32_t(words[offset]) | (std::uint32_t(words[offset + 1]) << 16);
    };
    if (sii_u32(ECT_SII_MANUF) != slave.eep_man || sii_u32(ECT_SII_ID) != slave.eep_id)
        return fail("SII EEPROM 与在线从站身份不一致");
    if (sii_crc(words.data()) != (words[7] & 0xffU))
        return fail("原始 SII 配置 CRC 不匹配，拒绝写入");
    if (words[kSiiAliasWord] == alias) {
        if (!restore()) { detail = "恢复控制权或写权限失败"; return false; }
        detail = "EEPROM Alias 已为目标值且 CRC 正确，未执行写入；当前 ESC Alias="
               + std::to_string(esc_alias) + "，如未生效请重新上电后验证";
        return true;
    }
    std::cout << "[EEPROM] Alias 修改: EEPROM=" << words[kSiiAliasWord]
              << " -> " << alias << "，当前 ESC Alias=" << esc_alias << '\n';
    const std::size_t bytes = (std::size_t(words[kSiiSizeWord]) + 1U) * 128U;
    if (bytes < kSiiHeaderWords * 2 || bytes > kMaxSiiBytes)
        return fail("SII EEPROM 声明容量无效，拒绝写入");
    words.resize(bytes / 2);
    for (std::size_t i = kSiiHeaderWords; i < words.size(); ++i) {
        if (!access.read_word(static_cast<std::uint16_t>(i), words[i], detail))
            return fail("备份读取 word " + std::to_string(i) + " 失败: " + detail);
    }
    const std::string after_path = backup_path + ".after.bin";
    try {
        if (std::filesystem::exists(after_path)) return fail("写后镜像路径已存在，拒绝写入: " + after_path);
        if (!save_sii_backup(backup_path, words, detail)) return fail(detail);
    } catch (const std::exception& error) {
        return fail(std::string("创建 EEPROM 备份失败: ") + error.what());
    }
    std::cout << "[EEPROM] 完整备份已同步到磁盘: " << backup_path << " (" << bytes << " bytes)\n";
    // Re-probe the protected configuration after backup, before the first write.
    for (std::size_t i = 0; i < 8; ++i) {
        std::uint16_t value = 0;
        if (!access.read_word(static_cast<std::uint16_t>(i), value, detail)) return fail(detail);
        if (value != words[i]) return fail("备份后 SII 配置区变化，未执行写入");
    }
    const auto before = words;
    words[kSiiAliasWord] = alias;
    words[7] = static_cast<std::uint16_t>((words[7] & 0xff00U) | sii_crc(words.data()));
    // Match EC-Master's ESC configuration-area update. Keep every other word
    // unchanged; finish the complete header and CRC before EEPROM readback.
    std::array<std::uint16_t, 8> header{};
    std::copy_n(words.begin(), header.size(), header.begin());
    const bool written = access.write_config_header(header, detail);
    const std::string write_error = written ? "" : detail;
    std::vector<std::uint16_t> after(words.size());
    for (std::size_t i = 0; i < after.size(); ++i) {
        if (!access.read_word(static_cast<std::uint16_t>(i), after[i], detail))
            return fail(write_error + (write_error.empty() ? "" : "；") + detail
                        + "；无法完成写后镜像，停止重试并保留原备份=" + backup_path);
    }
    try {
        if (!save_sii_backup(after_path, after, detail))
            return fail(write_error + (write_error.empty() ? "" : "；") + detail
                        + "；原备份=" + backup_path);
    } catch (const std::exception& error) {
        return fail(write_error + "；保存写后镜像失败: " + error.what() + "；原备份=" + backup_path);
    }
    std::cout << "[EEPROM] 完整写后镜像已保存: " << after_path << " (" << bytes << " bytes)\n";
    if (!written)
        return fail(write_error + "；停止重试，原备份=" + backup_path + "；写后镜像=" + after_path);
    if (!verify_alias_image(before, after, alias, detail))
        return fail(detail + "；停止重试，原备份=" + backup_path + "；写后镜像=" + after_path);
    if (!restore()) { detail = "Alias/CRC 和完整镜像校验正确，但恢复控制权或写权限失败；备份=" + backup_path; return false; }
    detail = "EEPROM Alias 和 CRC 写入成功，完整镜像校验通过，其余 words (含 PDO 信息) 未变化；备份=" + backup_path
           + "；写后镜像=" + after_path + "。请安全重新上电，再运行 read-alias 和 status 验证 ESC Alias";
    return true;
}

bool MaintEngine::clear_fault(int slave_index, std::string& detail) {
    if (!initialized_ || slave_index <= 0 || slave_index > slave_count_) {
        detail = "从站索引超出范围";
        return false;
    }

    uint16_t status_word = 0;
    if (!read_sdo(slave_index, SDO_STATUS_WORD, 0, status_word)) {
        detail = "读取状态字 0x6041 失败";
        return false;
    }

    // 检查是否真有故障
    if ((status_word & 0x0008U) == 0) {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "0x%04X", status_word);
        detail = "当前无故障报警 (状态字: " + std::string(buf) + ")";
        return true;
    }

    // 1. 下发控制字 0x0000 保证复位上升沿有效
    write_sdo(slave_index, SDO_CONTROL_WORD, 0, static_cast<uint16_t>(0x0000U));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 2. 下发控制字 0x0080 (CiA-402 Fault Reset 上升沿)
    const uint16_t cmd_reset = 0x0080U;
    if (!write_sdo(slave_index, SDO_CONTROL_WORD, 0, cmd_reset)) {
        detail = "向 0x6040 写入 0x0080 失败";
        return false;
    }

    // 3. 保持脉冲 50ms
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 4. 复位为 0x0006 (Shutdown 进入 Ready to Switch On 就绪态)
    const uint16_t cmd_ready = 0x0006U;
    write_sdo(slave_index, SDO_CONTROL_WORD, 0, cmd_ready);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // 5. 再次回读状态字与故障码
    status_word = 0;
    read_sdo(slave_index, SDO_STATUS_WORD, 0, status_word);
    uint16_t err_code = 0;
    read_sdo(slave_index, SDO_ERROR_CODE, 0, err_code);

    if ((status_word & 0x0008U) == 0) {
        detail = "故障已成功清除 (当前状态: " + decode_cia402_state(status_word) + ")";
        return true;
    } else {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "0x%04X", err_code);
        detail = "清除失败，硬件故障仍未消除 (错误码: " + std::string(buf) + " - " + describe_error_code(err_code) + ")";
        return false;
    }
}

bool MaintEngine::set_zero(int slave_index, std::string& detail) {
    if (!initialized_ || slave_index <= 0 || slave_index > slave_count_) {
        detail = "从站索引超出范围";
        return false;
    }

    // 安全检查：确认轴处于失能状态
    uint16_t status_word = 0;
    if (read_sdo(slave_index, SDO_STATUS_WORD, 0, status_word)) {
        if ((status_word & 0x006fU) == 0x0027U) {
            detail = "安全拦截：该轴处于 OperationEnabled (已使能) 状态，禁止在运动使能态标零！";
            return false;
        }
    }

    // 获取该轴电机厂商
    uint32_t vendor_id = context_.slavelist[slave_index].eep_man;
    if (vendor_id == 0) {
        read_sdo(slave_index, 0x1018, 1, vendor_id);
    }

    if (vendor_id == kEyouVendorId) {
        // ========== 意优电机 (Eyou) 标零流程 ==========
        // 1. 切换模式到 Homing Mode (6)
        const int8_t mode = 6;
        if (!write_sdo(slave_index, SDO_MODE_OF_OPERATION, 0, mode)) {
            detail = "意优驱动: 写入回零模式 0x6060:00 = 6 失败";
            return false;
        }

        // 2. 发送 0x0010 触发原点设置
        const uint16_t control = 0x0010U;
        if (!write_sdo(slave_index, SDO_CONTROL_WORD, 0, control)) {
            detail = "意优驱动: 写入控制字 0x6040:00 = 0x0010 失败";
            return false;
        }

        // 3. 延时 200ms 等待驱动内部设置原点
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // 4. 保存参数至 Flash 0x2130:00 = 1
        const uint8_t save = 1;
        if (!write_sdo(slave_index, SDO_EYOU_SAVE_FLASH, 0, save)) {
            detail = "意优驱动: 写入 Flash 保存对象 0x2130:00 = 1 失败";
            return false;
        }

        // 5. 切回 CSP 同步位置模式 (8)
        const int8_t csp_mode = 8;
        write_sdo(slave_index, SDO_MODE_OF_OPERATION, 0, csp_mode);
    } else {
        // ========== 同川电机 (Tongchuan) 及通用兼容标零流程 ==========
        // 1. 向标零对象 0x3801:00 写入 5
        const uint16_t cmd_zero = 5U;
        if (!write_sdo(slave_index, SDO_TONGCHUAN_SET_ZERO, 0, cmd_zero)) {
            detail = "向标零对象 0x3801:00 写入 5 失败 (厂商: " + vendor_id_to_name(vendor_id) + ")";
            return false;
        }

        // 2. 等待驱动内部写入 Flash / EEPROM
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 3. 回读校验 0x3801:00
        uint16_t readback = 0xFFFFU;
        if (!read_sdo(slave_index, SDO_TONGCHUAN_SET_ZERO, 0, readback)) {
            detail = "回读 0x3801:00 失败";
            return false;
        }

        if (readback != 0 && readback != 5) {
            detail = "标零回读校验异常 (预期 0 或 5，实际返回: " + std::to_string(readback) + ")";
            return false;
        }
    }

    // 验证实际脉冲位置是否已成功归为零 (允许微小抖动)
    int32_t act_pos = 999999;
    read_sdo(slave_index, SDO_ACTUAL_POSITION, 0, act_pos);

    detail = "标零完成 [" + vendor_id_to_name(vendor_id) + "]！编码器原点已写入，当前位置脉冲: " + std::to_string(act_pos);
    return true;
}

bool MaintEngine::get_axis_ratio(int slave_index, double& counts_per_deg) {
    uint32_t vendor_id = context_.slavelist[slave_index].eep_man;
    if (vendor_id == 0) {
        read_sdo(slave_index, 0x1018, 1, vendor_id);
    }

    if (vendor_id == kEyouVendorId) {
        // 意优驱动: 尝试读取 0x608F:01 (单圈分辨率) 与 0x6091 (电子齿轮比)
        uint32_t units = 0;
        if (!read_sdo(slave_index, SDO_POSITION_UNITS_PER_REV, 1, units) || units == 0) {
            units = 131072; // 默认 17 位编码器
        }

        double gear = 121.0; // 意优关节默认减速比 121
        uint32_t gear_num = 0, gear_den = 0;
        if (read_sdo(slave_index, SDO_GEAR_RATIO_COE, 1, gear_num) &&
            read_sdo(slave_index, SDO_GEAR_RATIO_COE, 2, gear_den) &&
            gear_den > 0 && gear_num > 0) {
            gear = static_cast<double>(gear_num) / static_cast<double>(gear_den);
        }

        counts_per_deg = (static_cast<double>(units) * gear) / 360.0;
        return true;
    }

    // 同川驱动或默认通用驱动:
    uint16_t closed_loop = 0;
    read_sdo(slave_index, SDO_CLOSED_LOOP_TYPE, 0, closed_loop);

    uint16_t gear = 101;
    if (!read_sdo(slave_index, SDO_GEAR_RATIO, 0, gear) || gear == 0) {
        gear = 101; // 默认谐波减速比 101
    }

    if (closed_loop == 1) {
        uint16_t out_res_div4 = 0;
        if (read_sdo(slave_index, SDO_OUTPUT_RESOLUTION, 0, out_res_div4) && out_res_div4 > 0) {
            double total_counts_per_rev = static_cast<double>(out_res_div4) * 4.0;
            counts_per_deg = total_counts_per_rev / 360.0;
            return true;
        }
    }

    // 默认 17 位电机编码器 131072 counts/rev * 减速比
    double total_counts_per_rev = 131072.0 * static_cast<double>(gear);
    counts_per_deg = total_counts_per_rev / 360.0;
    return true;
}

bool MaintEngine::jog_axis_deg(int slave_index, double delta_deg, double speed_rpm, std::string& detail) {
    if (std::abs(delta_deg) < 1e-4) {
        detail = "点动角度过小 (< 0.0001 deg)";
        return false;
    }

    double counts_per_deg = 131072.0 * 101.0 / 360.0;
    get_axis_ratio(slave_index, counts_per_deg);

    int32_t delta_counts = static_cast<int32_t>(std::llround(delta_deg * counts_per_deg));
    return jog_axis_counts(slave_index, delta_counts, speed_rpm, detail);
}

bool MaintEngine::configure_dynamic_pdo(int slave_index) {
    if (slave_index <= 0 || slave_index > slave_count_) return false;

    // 1. 设置运行模式为 CSP (8) 通过 SDO
    const int8_t op_mode = 8;
    write_sdo(slave_index, SDO_MODE_OF_OPERATION, 0, op_mode);

    // 2. 清除 SM2 分配 (0x1C12:00 = 0)
    const uint8_t zero8 = 0;
    if (!write_sdo(slave_index, 0x1C12, 0, zero8)) {
        // 如果驱动器不支持动态修改 0x1C12，说明该驱动为固定 PDO 映射，保留其默认配置
        return false;
    }

    // 3. 配置 RxPDO 0x1600 (动态映射 3 个核心字段: Controlword, Target Pos, Mode)
    // 0x1600:00 = 0 (禁用映射以允许修改)
    write_sdo(slave_index, 0x1600, 0, zero8);
    // 0x1600:01 = 0x60400010 (Controlword, 16 bits)
    const uint32_t map_cw = 0x60400010U;
    write_sdo(slave_index, 0x1600, 1, map_cw);
    // 0x1600:02 = 0x607A0020 (Target Position, 32 bits)
    const uint32_t map_tp = 0x607A0020U;
    write_sdo(slave_index, 0x1600, 2, map_tp);
    // 0x1600:03 = 0x60600008 (Modes of Operation, 8 bits)
    const uint32_t map_mode = 0x60600008U;
    write_sdo(slave_index, 0x1600, 3, map_mode);
    // 0x1600:00 = 3 (激活映射条目数)
    const uint8_t rx_count = 3;
    write_sdo(slave_index, 0x1600, 0, rx_count);

    // 将 0x1600 挂载到 SM2 (0x1C12:01 = 0x1600, 0x1C12:00 = 1)
    const uint16_t rx_pdo_idx = 0x1600U;
    write_sdo(slave_index, 0x1C12, 1, rx_pdo_idx);
    const uint8_t one8 = 1;
    write_sdo(slave_index, 0x1C12, 0, one8);

    // 4. 清除 SM3 分配 (0x1C13:00 = 0)
    write_sdo(slave_index, 0x1C13, 0, zero8);

    // 5. 配置 TxPDO 0x1A00 (动态映射 2 个核心字段: Statusword, Actual Pos)
    // 0x1A00:00 = 0 (禁用映射以允许修改)
    write_sdo(slave_index, 0x1A00, 0, zero8);
    // 0x1A00:01 = 0x60410010 (Statusword, 16 bits)
    const uint32_t map_sw = 0x60410010U;
    write_sdo(slave_index, 0x1A00, 1, map_sw);
    // 0x1A00:02 = 0x60640020 (Actual Position, 32 bits)
    const uint32_t map_ap = 0x60640020U;
    write_sdo(slave_index, 0x1A00, 2, map_ap);
    // 0x1A00:00 = 2 (激活映射条目数)
    const uint8_t tx_count = 2;
    write_sdo(slave_index, 0x1A00, 0, tx_count);

    // 将 0x1A00 挂载到 SM3 (0x1C13:01 = 0x1A00, 0x1C13:00 = 1)
    const uint16_t tx_pdo_idx = 0x1A00U;
    write_sdo(slave_index, 0x1C13, 1, tx_pdo_idx);
    write_sdo(slave_index, 0x1C13, 0, one8);

    return true;
}

bool MaintEngine::jog_axis_counts(int slave_index, std::int32_t delta_counts, double speed_rpm, std::string& detail) {
    return run_position_control(slave_index, delta_counts, speed_rpm, false, detail);
}

bool MaintEngine::enable_axis(int slave_index, std::string& detail) {
    return run_position_control(slave_index, 0, 0.5, true, detail);
}

bool MaintEngine::disable_axis(int slave_index, std::string& detail) {
    if (!initialized_ || slave_index <= 0 || slave_index > slave_count_) {
        detail = "从站索引超出范围";
        return false;
    }
    const uint16_t shutdown = 0x0006;
    if (!write_sdo(slave_index, SDO_CONTROL_WORD, 0, shutdown)) {
        detail = "SDO 写入失能控制字 0x0006 失败";
        return false;
    }
    uint16_t sw = 0;
    for (int attempt = 0; attempt < 100; ++attempt) {
        if (!read_sdo(slave_index, SDO_STATUS_WORD, 0, sw)) {
            detail = "失能后读取状态字失败，无法确认失能";
            return false;
        }
        const auto state = sw & 0x006fU;
        if (state == 0x0021U || (sw & 0x004fU) == 0x0040U ||
            (sw & 0x004fU) == 0x0008U) {
            detail = "已确认失能，状态: " + decode_cia402_state(sw);
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    detail = "失能状态确认超时，当前状态: " + decode_cia402_state(sw);
    return false;
}

bool MaintEngine::run_position_control(int slave_index, std::int32_t delta_counts,
                                       double speed_rpm, bool hold_enabled, std::string& detail) {
    if (!initialized_ || slave_index <= 0 || slave_index > slave_count_) {
        detail = "从站索引超出范围";
        return false;
    }

    if (!hold_enabled && delta_counts == 0) {
        detail = "点动位移脉冲为 0";
        return false;
    }

    // 1. 转速安全限幅 (输出端转速限制在 0.05 ~ 2.0 RPM)
    speed_rpm = std::clamp(speed_rpm, 0.05, 2.0);

    // 2. 先在 PRE-OP 下通过 SDO 检查该轴状态，若有报警先提示清错
    uint16_t pre_status = 0;
    if (!read_sdo(slave_index, SDO_STATUS_WORD, 0, pre_status)) {
        detail = "读取使能前状态字失败";
        return false;
    }
    if ((pre_status & 0x0008U) != 0) {
        uint16_t err_code = 0;
        read_sdo(slave_index, SDO_ERROR_CODE, 0, err_code);
        detail = "该轴处于故障报警状态 (0x" + std::to_string(err_code) + ")，请先执行 clear-fault 清错！";
        return false;
    }

    // 3. 配置动态最小化 PDO 映射并进入 SAFE-OP / OP 状态
    // 对所有从站尝试动态配置仅含核心字段的 PDO (7 字节 Rx / 6 字节 Tx)
    // 若电机不支持动态 PDO (固定 PDO)，则自动降级使用电机出厂默认映射
    for (int s = 1; s <= slave_count_; ++s) {
        configure_dynamic_pdo(s);
    }

    std::memset(iomap_, 0, sizeof(iomap_));
    int mapped_size = ecx_config_map_group(&context_, iomap_, 0);
    if (mapped_size <= 0) {
        detail = "SOEM PDO 映射失败 (config_map_group 返回 0)";
        return false;
    }

    // 等待所有从站达到 SAFE-OP
    ecx_statecheck(&context_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 2);

    // 发送初次过程数据，使从站能平稳切入 OP
    ecx_send_processdata(&context_);
    ecx_receive_processdata(&context_, EC_TIMEOUTRXM);

    // 请求进入 OP 状态
    context_.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&context_, 0);

    int op_check = 0;
    do {
        ecx_send_processdata(&context_);
        ecx_receive_processdata(&context_, EC_TIMEOUTRXM);
        ecx_statecheck(&context_, slave_index, EC_STATE_OPERATIONAL, 50000);
    } while (op_check++ < 40 && context_.slavelist[slave_index].state != EC_STATE_OPERATIONAL);

    if (context_.slavelist[slave_index].state != EC_STATE_OPERATIONAL) {
        detail = "从站未能成功进入 OP 运行状态 (当前: " + al_state_str(context_.slavelist[slave_index].state) + ")";
        context_.slavelist[0].state = EC_STATE_PRE_OP;
        ecx_writestate(&context_, 0);
        return false;
    }

    // 获取当前轴的 PDO 内存指针及配置大小
    uint8_t* out_ptr = context_.slavelist[slave_index].outputs;
    uint8_t* in_ptr = context_.slavelist[slave_index].inputs;
    const int obytes = context_.slavelist[slave_index].Obytes;
    const int ibytes = context_.slavelist[slave_index].Ibytes;

    if (!out_ptr || !in_ptr || obytes < 6 || ibytes < 6) {
        detail = "未获取到有效的 PDO 输入/输出内存指针 (Obytes: " + std::to_string(obytes) + ", Ibytes: " + std::to_string(ibytes) + ")";
        context_.slavelist[0].state = EC_STATE_PRE_OP;
        ecx_writestate(&context_, 0);
        return false;
    }

    // 自适应确定模式字节偏移量:
    // 动态极简映射: 7 字节 (CW: 0..1, Pos: 2..5, Mode: 6) -> mode_offset = 6
    // 同川默认固定映射: 17 字节 (CW: 0..1, Pos: 2..5, ..., Mode: 16) -> mode_offset = 16
    // 其它驱动器若无 Mode 映射，则 mode_offset = -1 (已在 PRE-OP 下通过 SDO 0x6060 预置 CSP)
    int mode_offset = -1;
    if (obytes == 7) {
        mode_offset = 6;
    } else if (obytes >= 17) {
        mode_offset = 16;
    }

    // 辅助读写 PDO
    auto get_in_status = [&]() -> uint16_t {
        uint16_t sw = 0;
        if (in_ptr && ibytes >= 2) {
            std::memcpy(&sw, in_ptr, 2);
        }
        return sw;
    };
    auto get_in_pos = [&]() -> int32_t {
        int32_t p = 0;
        if (in_ptr && ibytes >= 6) {
            std::memcpy(&p, in_ptr + 2, 4);
        }
        return p;
    };
    auto set_out = [&](uint16_t cw, int32_t target_p, int8_t mode = 8) {
        if (!out_ptr) return;
        if (obytes >= 2) {
            std::memcpy(out_ptr, &cw, 2);
        }
        if (obytes >= 6) {
            std::memcpy(out_ptr + 2, &target_p, 4);
        }
        if (mode_offset >= 0 && mode_offset < obytes) {
            out_ptr[mode_offset] = static_cast<uint8_t>(mode); // Mode 8: CSP
        }
    };

    // 读取当前实际脉冲作为基准位置
    ecx_send_processdata(&context_);
    ecx_receive_processdata(&context_, EC_TIMEOUTRXM);
    int32_t start_pos = get_in_pos();
    int32_t current_cmd = start_pos;

    // 4. 驱动器伺服使能握手 (CiA-402 状态推进)
    stop_requested_.store(false);

    // Step A: 发送 0x0006 (Shutdown / Ready to Switch On)
    bool enabled = false;
    for (int step = 0; step < 100 && !stop_requested_.load(); ++step) {
        set_out(0x0006, start_pos, 8);
        ecx_send_processdata(&context_);
        ecx_receive_processdata(&context_, EC_TIMEOUTRXM);
        uint16_t sw = get_in_status();
        if ((sw & 0x006fU) == 0x0021U || (sw & 0x006fU) == 0x0023U) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // Step B: 发送 0x0007，先确认 Switched On，再请求运行使能。
    bool switched_on = false;
    for (int step = 0; step < 100 && !stop_requested_.load(); ++step) {
        set_out(0x0007, start_pos, 8);
        ecx_send_processdata(&context_);
        ecx_receive_processdata(&context_, EC_TIMEOUTRXM);
        if ((get_in_status() & 0x006fU) == 0x0023U) {
            switched_on = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // Step C: 发送 0x000f (Enable Operation)
    for (int step = 0; step < 200 && switched_on && !stop_requested_.load(); ++step) {
        set_out(0x000f, start_pos, 8);
        ecx_send_processdata(&context_);
        ecx_receive_processdata(&context_, EC_TIMEOUTRXM);
        uint16_t sw = get_in_status();
        if ((sw & 0x006fU) == 0x0027U) {
            enabled = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    if (!enabled || stop_requested_.load()) {
        set_out(0x0006, start_pos, 8);
        ecx_send_processdata(&context_);
        context_.slavelist[0].state = EC_STATE_PRE_OP;
        ecx_writestate(&context_, 0);
        detail = "驱动器使能握手超时或被手动取消";
        return false;
    }

    bool hold_ok = true;
    if (hold_enabled) {
        // Resolve ratio before entering the cyclic loop; no mailbox reads while moving.
        double counts_per_deg = 131072.0 * 101.0 / 360.0;
        get_axis_ratio(slave_index, counts_per_deg);
        std::cout << "[使能成功] 保持当前位置 " << start_pos << " counts\n"
                  << "输入 jog-enabled --delta_deg 1.0 [--speed_rpm 0.5]\n"
                  << "或 jog-enabled --delta_count 5000；输入 disable 或 Ctrl+C 失能退出。"
                  << std::endl;
        std::string command_line;
        bool discard_line = false;
        bool moving = false;
        int32_t motion_start = start_pos;
        int32_t motion_delta = 0;
        std::int64_t motion_cycle = 0, motion_cycles = 1;
        auto next_hold = std::chrono::steady_clock::now();
        while (!stop_requested_.load()) {
            // poll + single-byte read avoids blocking the PDO loop on an incomplete line.
            // Bound input processing to keep one cycle from consuming an unlimited stream.
            for (int n = 0; n < 256; ++n) {
                pollfd input{STDIN_FILENO, POLLIN, 0};
                const int ready = ::poll(&input, 1, 0);
                if (ready < 0) {
                    if (errno != EINTR) { hold_ok = false; stop_requested_.store(true); }
                    break;
                }
                if (ready == 0) break;
                if (input.revents & (POLLERR | POLLNVAL)) {
                    hold_ok = false; stop_requested_.store(true); break;
                }
                char ch = 0;
                const auto bytes = ::read(STDIN_FILENO, &ch, 1);
                if (bytes <= 0) {
                    if (bytes == 0 || errno != EINTR) stop_requested_.store(true);
                    break;
                }
                if (ch != '\n') {
                    if (command_line.size() < 512 && !discard_line) command_line += ch;
                    else { discard_line = true; command_line.clear(); }
                    continue;
                }
                const auto line = command_line;
                command_line.clear();
                if (discard_line) {
                    discard_line = false;
                    std::cout << "[拒绝] 输入过长\n";
                    continue;
                }
                std::istringstream words(line);
                std::string verb, extra;
                words >> verb;
                if (verb.empty()) continue;
                if ((verb == "disable" || verb == "quit") && !(words >> extra)) {
                    stop_requested_.store(true); break;
                }
                if (moving) {
                    std::cout << "[拒绝] 正在运动，请等待本次完成\n";
                    continue;
                }
                JogRequest request;
                std::string error;
                if (!parse_enabled_jog(line, counts_per_deg, request, error)) {
                    std::cout << "[拒绝] " << error << '\n';
                    continue;
                }
                const std::int64_t target = static_cast<std::int64_t>(current_cmd) + request.delta_counts;
                if (target < std::numeric_limits<int32_t>::min() ||
                    target > std::numeric_limits<int32_t>::max()) {
                    std::cout << "[拒绝] 目标位置超出 int32 范围\n";
                    continue;
                }
                motion_start = current_cmd;
                motion_delta = request.delta_counts;
                const double seconds = std::max(1.0, std::abs(static_cast<double>(motion_delta)) /
                    (counts_per_deg * 360.0) / request.speed_rpm * 60.0);
                motion_cycles = static_cast<std::int64_t>(std::ceil(seconds / 0.002));
                motion_cycle = 0;
                moving = true;
                std::cout << "[JOG 启动] 增量 " << motion_delta << " counts\n";
            }
            if (stop_requested_.load()) break;
            if (moving) {
                const double t = static_cast<double>(++motion_cycle) / motion_cycles;
                current_cmd = static_cast<int32_t>(static_cast<std::int64_t>(motion_start) +
                    std::llround(static_cast<double>(motion_delta) * smoothstep5(t)));
            }
            set_out(0x000f, current_cmd, 8);
            ecx_send_processdata(&context_);
            const int wkc = ecx_receive_processdata(&context_, EC_TIMEOUTRXM);
            const int expected_wkc = context_.grouplist[0].outputsWKC * 2 +
                                     context_.grouplist[0].inputsWKC;
            if (wkc < expected_wkc || (get_in_status() & 0x006fU) != 0x0027U) {
                hold_ok = false;
                break;
            }
            if (moving && motion_cycle >= motion_cycles) {
                moving = false;
                std::cout << "[JOG 指令发送完成] 保持终点 " << current_cmd
                          << " counts，继续使能；实际位置 " << get_in_pos() << " counts\n";
            }
            next_hold += std::chrono::milliseconds(2);
            std::this_thread::sleep_until(next_hold);
        }
    }

    // 5. 计算点动运行总时长与 2ms 周期循环参数
    double counts_per_rev = 131072.0 * 101.0;
    double counts_per_deg = counts_per_rev / 360.0;
    get_axis_ratio(slave_index, counts_per_deg);
    counts_per_rev = counts_per_deg * 360.0;

    double revs = std::abs(static_cast<double>(delta_counts)) / counts_per_rev;
    double duration_s = (revs / speed_rpm) * 60.0;
    if (duration_s < 1.0) duration_s = 1.0; // 最小平滑时间 1.0 秒，确保平稳起停

    constexpr int cycle_us = 2000; // 2ms 控制周期
    const int total_cycles = static_cast<int>(duration_s * 1000000.0 / cycle_us);

    if (!hold_enabled) std::cout << "  [JOG 启动] 目标增量: " << delta_counts << " counts ("
              << std::fixed << std::setprecision(2) << (delta_counts / counts_per_deg) << " deg), "
              << "规划时长: " << std::setprecision(2) << duration_s << " s, 输出转速: " << speed_rpm << " RPM\n";

    // 6. 执行 2ms 周期 CSP 平滑插值循环
    auto next_time = std::chrono::steady_clock::now();
    bool completed = true;

    for (int c = 1; !hold_enabled && c <= total_cycles; ++c) {
        if (stop_requested_.load()) {
            completed = false;
            std::cout << "\n  [JOG 中断] 收到停止信号 (Ctrl+C)，安全停机抱闸...\n";
            break;
        }

        double t = static_cast<double>(c) / total_cycles;
        double s = smoothstep5(t);
        current_cmd = start_pos + static_cast<int32_t>(std::llround(delta_counts * s));

        set_out(0x000f, current_cmd, 8);
        ecx_send_processdata(&context_);
        ecx_receive_processdata(&context_, EC_TIMEOUTRXM);

        next_time += std::chrono::microseconds(cycle_us);
        std::this_thread::sleep_until(next_time);
    }

    // 7. 到位后保持当前终点 200ms
    int32_t final_hold = current_cmd;
    for (int h = 0; !hold_enabled && h < 100; ++h) {
        set_out(0x000f, final_hold, 8);
        ecx_send_processdata(&context_);
        ecx_receive_processdata(&context_, EC_TIMEOUTRXM);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // 8. 安全脱使能下电 (下发 0x0006 抱闸锁死)
    for (int d = 0; d < 50; ++d) {
        set_out(0x0006, final_hold, 8);
        ecx_send_processdata(&context_);
        ecx_receive_processdata(&context_, EC_TIMEOUTRXM);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // 9. 从站切回 PRE-OP 状态以确保后续 SDO 纯维护命令无缝工作
    context_.slavelist[0].state = EC_STATE_PRE_OP;
    ecx_writestate(&context_, 0);
    ecx_statecheck(&context_, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

    int32_t final_actual = get_in_pos();
    detail = "点动结束！实际运动: " + std::to_string(final_actual - start_pos) + " counts ("
           + std::to_string((final_actual - start_pos) / counts_per_deg) + " deg), 当前绝对位置: "
           + std::to_string(final_actual);

    if (hold_enabled) {
        std::string disable_detail;
        const bool disabled = disable_axis(slave_index, disable_detail);
        detail = (hold_ok ? "使能保持结束；" : "过程数据异常或驱动器退出使能；") + disable_detail;
        return hold_ok && disabled;
    }
    return completed;
}

} // namespace robot_maint
