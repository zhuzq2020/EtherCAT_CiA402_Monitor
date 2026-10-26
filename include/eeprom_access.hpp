#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <ostream>
#include <string>

namespace robot_maint {
// Register access is injected so ownership and failure paths can be tested
// without an EtherCAT interface or a motor.
class EepromIo {
public:
    virtual ~EepromIo() = default;
    virtual int read(std::uint16_t reg, void* data, std::size_t size) = 0;
    virtual int write(std::uint16_t reg, const void* data, std::size_t size) = 0;
    virtual std::uint64_t now_us() = 0;
    virtual void pause_us(unsigned us) = 0;
};

struct EepromSnapshot {
    std::uint8_t config{0};
    std::uint8_t access{0};
    std::uint16_t status{0};
    std::string text() const;
};

class EepromAccess {
public:
    explicit EepromAccess(EepromIo& io, std::ostream* log = nullptr,
                          unsigned timeout_ms = 15000);
    ~EepromAccess();
    EepromAccess(const EepromAccess&) = delete;
    EepromAccess& operator=(const EepromAccess&) = delete;
    bool acquire(std::string& detail);
    bool restore(std::string& detail);
    bool read_word(std::uint16_t offset, std::uint16_t& value, std::string& detail);
    bool write_word(std::uint16_t offset, std::uint16_t value, std::string& detail);
    // Configuration-area writes are verified after all eight words, including
    // their CRC, have been issued. No interleaved EEPROM read commands.
    bool write_config_header(const std::array<std::uint16_t, 8>& words, std::string& detail);
    bool snapshot(EepromSnapshot& state, std::string& detail);
    bool has_original() const { return original_valid_; }
    const EepromSnapshot& original() const { return original_; }
private:
    bool transfer(bool write, std::uint16_t reg, void* data, std::size_t size,
                  std::string& detail);
    bool issue_write(std::uint16_t offset, std::uint16_t value, std::string& detail);
    bool owner_is_master(std::string& detail);
    bool wait_ready(std::uint16_t& status, std::string& detail);
    void report(const char* phase, const EepromSnapshot& state);
    EepromIo& io_;
    std::ostream* log_;
    std::uint64_t timeout_us_;
    EepromSnapshot original_{};
    bool restore_failed_{false};
    std::string restore_error_;
    bool original_valid_{false};
    bool restore_needed_{false};
    bool acquired_{false};
};
}
