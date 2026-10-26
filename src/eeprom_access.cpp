#include "eeprom_access.hpp"
#include "sii_integrity.hpp"
#include <array>
#include <iomanip>
#include <sstream>

namespace robot_maint {
namespace {
constexpr std::uint16_t kConfig = 0x0500, kStatus = 0x0502;
constexpr std::uint16_t kAddress = 0x0504, kData = 0x0508;
std::uint16_t u16(const std::uint8_t* data) {
    return static_cast<std::uint16_t>(data[0] | (std::uint16_t(data[1]) << 8));
}
std::string status_text(std::uint16_t status) {
    std::ostringstream out;
    out << "EEPROM[0x0502]=0x" << std::hex << std::setw(4) << std::setfill('0') << status;
    out << ((status & 0x0020) ? " (仿真模式)" : " (I2C模式)");
    if (status & 0x0800) out << " CRC错误";
    if (status & 0x1000) out << " 加载状态错误";
    if (status & 0x2000) out << " NACK";
    if (status & 0x4000) out << " 写使能错误";
    if (status & 0x8000) out << " BUSY";
    return out.str();
}
}
std::string EepromSnapshot::text() const {
    std::ostringstream out;
    out << "config[0x0500]=0x" << std::hex << std::setw(2) << std::setfill('0')
        << unsigned(config) << ", access[0x0501]=0x" << std::setw(2) << unsigned(access)
        << ", owner=" << ((access & 1) ? "PDI" : "ECAT") << "; " << status_text(status);
    return out.str();
}
EepromAccess::EepromAccess(EepromIo& io, std::ostream* log, unsigned timeout_ms)
    : io_(io), log_(log), timeout_us_(std::uint64_t(timeout_ms) * 1000) {}
EepromAccess::~EepromAccess() {
    if (restore_needed_) {
        std::string error;
        if (!restore(error) && log_) *log_ << "[错误] " << error << '\n';
    }
}
bool EepromAccess::transfer(bool write, std::uint16_t reg, void* data,
                            std::size_t size, std::string& detail) {
    const int wkc = write ? io_.write(reg, data, size) : io_.read(reg, data, size);
    if (wkc == 1) return true;
    std::ostringstream out;
    out << (write ? "写" : "读") << "寄存器 0x" << std::hex << reg
        << " 失败，WKC=" << std::dec << wkc;
    detail = out.str();
    return false;
}
bool EepromAccess::snapshot(EepromSnapshot& state, std::string& detail) {
    std::array<std::uint8_t, 4> data{};
    if (!transfer(false, kConfig, data.data(), data.size(), detail)) return false;
    state = {data[0], data[1], u16(data.data() + 2)};
    return true;
}
void EepromAccess::report(const char* phase, const EepromSnapshot& state) {
    if (log_) *log_ << "[EEPROM] " << phase << ": " << state.text() << "; WKC=1\n";
}
bool EepromAccess::wait_ready(std::uint16_t& status, std::string& detail) {
    const auto start = io_.now_us();
    do {
        std::array<std::uint8_t, 2> raw{};
        if (!transfer(false, kStatus, raw.data(), raw.size(), detail)) return false;
        status = u16(raw.data());
        // Preserve errors before any new command can clear them.
        if (status & 0x7800U) {
            detail = "EEPROM 错误，未清除: " + status_text(status);
            return false;
        }
        if ((status & 0x8000U) == 0) return true;
        io_.pause_us(200);
    } while (io_.now_us() - start < timeout_us_);
    detail = "等待 EEPROM 超时: " + status_text(status);
    return false;
}
bool EepromAccess::owner_is_master(std::string& detail) {
    std::array<std::uint8_t, 2> owner{};
    if (!transfer(false, kConfig, owner.data(), owner.size(), detail)) return false;
    if (acquired_ && (owner[0] & 3) == 2 && (owner[1] & 1) == 0) return true;
    detail = "EEPROM 强制主站控制权未确认或已丢失，config=" + std::to_string(owner[0])
           + ", access=" + std::to_string(owner[1]);
    return false;
}
bool EepromAccess::acquire(std::string& detail) {
    if (original_valid_) { detail = "EEPROM 会话已开始，不可重复切换"; return false; }
    if (!snapshot(original_, detail)) return false;
    original_valid_ = true;
    report("切换前", original_);
    std::uint16_t status = 0;
    if (!wait_ready(status, detail)) return false;
    // Match the verified EC-Master force-assign flow: retain override bit 1
    // throughout the session. Clearing it here differs from the successful
    // commercial-master session even though access still reports ECAT.
    restore_needed_ = true;
    std::uint8_t config = 2;
    if (!transfer(true, kConfig, &config, sizeof(config), detail)) return false;
    const auto start = io_.now_us();
    EepromSnapshot state;
    do {
        if (!snapshot(state, detail)) return false;
        if ((state.status & 0x7800U) != 0) {
            detail = "切换控制权后 EEPROM 错误: " + state.text();
            return false;
        }
        if ((state.config & 3) == 2 && (state.access & 1) == 0
            && (state.status & 0x8000) == 0) {
            acquired_ = true;
            report("切换后", state);
            return true;
        }
        io_.pause_us(200);
    } while (io_.now_us() - start < timeout_us_);
    detail = "切换 EEPROM 控制权超时: " + state.text();
    return false;
}
bool EepromAccess::restore(std::string& detail) {
    if (!restore_needed_) {
        if (restore_failed_) detail = restore_error_;
        return !restore_failed_;
    }
    auto failed = [&]() {
        restore_failed_ = true;
        restore_error_ = detail;
        return false;
    };
    // One explicit cleanup attempt; a failed cleanup is not silently retried
    // from the destructor. Errors remain visible in the snapshot.
    restore_needed_ = false;
    acquired_ = false;
    std::uint8_t config = original_.config & 3;
    if (!transfer(true, kConfig, &config, sizeof(config), detail)) return failed();
    const auto start = io_.now_us();
    EepromSnapshot state;
    do {
        if (!snapshot(state, detail)) return failed();
        // Config bit 0 grants PDI permission. Whether PDI actually takes it is
        // firmware-dependent; restoring permission does not require PDI active.
        const bool access_ok = (config & 1) || (state.access & 1) == 0;
        if ((state.config & 3) == config && access_ok) {
            report("恢复后", state);
            return true;
        }
        io_.pause_us(200);
    } while (io_.now_us() - start < timeout_us_);
    detail = "恢复 EEPROM 控制权超时: " + state.text();
    return failed();
}
bool EepromAccess::read_word(std::uint16_t offset, std::uint16_t& value,
                            std::string& detail) {
    std::uint16_t status = 0;
    if (!owner_is_master(detail) || !wait_ready(status, detail)) return false;
    std::array<std::uint8_t, 4> address{{std::uint8_t(offset), std::uint8_t(offset >> 8), 0, 0}};
    if (!transfer(true, kAddress, address.data(), address.size(), detail)) return false;
    std::array<std::uint8_t, 2> command{{0, 1}};
    if (!transfer(true, kStatus, command.data(), command.size(), detail)) return false;
    io_.pause_us(200);
    if (!wait_ready(status, detail)) return false;
    std::array<std::uint8_t, 8> data{};
    if (!transfer(false, kData, data.data(), (status & 0x0040) ? 8 : 4, detail)) return false;
    if (!owner_is_master(detail)) return false;
    value = u16(data.data());
    return true;
}
bool EepromAccess::issue_write(std::uint16_t offset, std::uint16_t value,
                             std::string& detail) {
    std::uint16_t status = 0;
    if (!owner_is_master(detail) || !wait_ready(status, detail)) return false;
    // Match the EC-Master ESC configuration-area register block: control (2),
    // address (4), and the complete data register (4). Only the low WORD is
    // written to EEPROM; the upper data-register WORD is explicitly zero.
    // The full-image readback verifies that the adjacent EEPROM WORD did not
    // change. In emulation mode a truncated data-register transfer may differ.
    std::array<std::uint8_t, 10> command{{1, 2,
        std::uint8_t(offset), std::uint8_t(offset >> 8), 0, 0,
        std::uint8_t(value), std::uint8_t(value >> 8), 0, 0}};
    if (log_) *log_ << "[EEPROM] 写 word=" << offset << ", value=" << value
                    << ", command=0x0201, FPWR[0x0502..0x050b], 10 bytes (不自动重试)\n";
    if (!transfer(true, kStatus, command.data(), command.size(), detail)) return false;
    io_.pause_us(400);
    if (!wait_ready(status, detail)) return false;
    if (log_) *log_ << "[EEPROM] 写命令完成: " << status_text(status) << "; WKC=1\n";
    return owner_is_master(detail);
}
bool EepromAccess::write_word(std::uint16_t offset, std::uint16_t value, std::string& detail) {
    if (!issue_write(offset, value, detail)) return false;
    std::uint16_t actual = 0;
    if (!read_word(offset, actual, detail)) return false;
    if (actual != value) {
        detail = "word " + std::to_string(offset) + " 写后回读不一致，目标="
               + std::to_string(value) + "，实际=" + std::to_string(actual)
               + "; 写命令完成但数据未验证";
        return false;
    }
    return true;
}
bool EepromAccess::write_config_header(const std::array<std::uint16_t, 8>& words,
                                      std::string& detail) {
    if (sii_crc(words.data()) != (words[7] & 0xffU)) {
        detail = "待写配置区 CRC 无效，未执行写入";
        return false;
    }
    if (log_) *log_ << "[EEPROM] 写完整配置区 word 0..7，CRC 最后写；完成前不发 EEPROM 读命令\n";
    for (std::uint16_t offset = 0; offset < words.size(); ++offset) {
        if (!issue_write(offset, words[offset], detail)) {
            detail = "配置区写入停止于 word " + std::to_string(offset) + ": " + detail;
            return false;
        }
    }
    for (std::uint16_t offset = 0; offset < words.size(); ++offset) {
        std::uint16_t actual = 0;
        if (!read_word(offset, actual, detail)) return false;
        if (actual != words[offset]) {
            detail = "配置区完整写入后回读不一致，word=" + std::to_string(offset)
                   + "，期望=" + std::to_string(words[offset]) + "，实际=" + std::to_string(actual);
            return false;
        }
    }
    return true;
}
}
