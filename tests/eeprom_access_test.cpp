#include "eeprom_access.hpp"
#include "sii_integrity.hpp"
#include <algorithm>
#include <array>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace robot_maint;
void check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
struct FakeIo final : EepromIo {
    std::uint8_t config{1}, access{1};
    std::uint16_t status{0x20}, address{0}, pending{0};
    std::array<std::uint16_t, 128> memory{};
    std::vector<std::uint16_t> writes;
    std::vector<std::uint8_t> owner_writes;
    unsigned busy_reads{0}, write_commands{0}, read_commands{0};
    bool busy_forever{false}, refuse_owner{false}, ignore_write{false}, lose_owner_on_data{false};
    bool defer_header_until_crc{false}, header_active{false}, staged_valid{false};
    std::array<std::uint16_t, 8> staged{};
    std::uint16_t error_after_write{0};
    int fail_write_reg{-1}, fail_read_reg{-1};
    std::uint64_t clock{0};
    FakeIo() { memory[0] = 0x0c80; memory[1] = 0x6681; memory[7] = 0x62; }
    int read(std::uint16_t reg, void* data, std::size_t size) override {
        if (reg == fail_read_reg) return 0;
        auto* bytes = static_cast<std::uint8_t*>(data);
        std::fill(bytes, bytes + size, 0);
        if (reg == 0x500) {
            check(size == 2 || size == 4, "owner read size");
            bytes[0] = config; bytes[1] = access;
            if (size == 4) { bytes[2] = status & 0xff; bytes[3] = status >> 8; }
        } else if (reg == 0x502) {
            check(size == 2, "status read size");
            const std::uint16_t current = status | ((busy_forever || busy_reads) ? 0x8000 : 0);
            if (busy_reads) --busy_reads;
            bytes[0] = current & 0xff; bytes[1] = current >> 8;
        } else if (reg == 0x508) {
            check(size == ((status & 0x40) ? 8 : 4), "data read width");
            for (std::size_t i = 0; i < size / 2; ++i) {
                const auto word = memory.at(address + i);
                bytes[2*i] = word & 0xff; bytes[2*i+1] = word >> 8;
            }
        } else throw std::runtime_error("unexpected register read");
        return 1;
    }
    int write(std::uint16_t reg, const void* data, std::size_t size) override {
        writes.push_back(reg);
        if (reg == fail_write_reg) { fail_write_reg = -1; return 0; }
        const auto* bytes = static_cast<const std::uint8_t*>(data);
        if (reg == 0x500) {
            check(size == 1, "owner write size");
            owner_writes.push_back(bytes[0]);
            if (!refuse_owner) { config = bytes[0]; access = config & 1; }
        } else if (reg == 0x504) {
            check(size == 4 && bytes[2] == 0 && bytes[3] == 0, "32-bit little-endian address");
            address = bytes[0] | (std::uint16_t(bytes[1]) << 8);
        } else if (reg == 0x508) {
            check(size == 2, "write one word only");
            pending = bytes[0] | (std::uint16_t(bytes[1]) << 8);
            if (lose_owner_on_data) { config = 1; access = 1; }
        } else if (reg == 0x502) {
            check((size == 2 || size == 10) && bytes[1] != 0, "no implicit error clearing");
            check(config == 2 && access == 0, "command retains EC-Master-style force override");
            if (bytes[1] == 2) {
                check(size == 10 && bytes[0] == 1, "write enable/address/data accompany WRITE");
                check(bytes[4] == 0 && bytes[5] == 0, "32-bit word address upper bytes");
                check(bytes[8] == 0 && bytes[9] == 0, "complete 32-bit data register with zero upper word");
                address = bytes[2] | (std::uint16_t(bytes[3]) << 8);
                pending = bytes[6] | (std::uint16_t(bytes[7]) << 8);
                ++write_commands;
                check(std::count(writes.begin(), writes.end(), 0x508) == 0,
                      "no separate staged data write");
                if (!ignore_write && !error_after_write) {
                    if (defer_header_until_crc && address < 8) {
                        if (!staged_valid || address == 0) {
                            std::copy_n(memory.begin(), 8, staged.begin());
                            staged_valid = true;
                        }
                        if (address == 0) header_active = true;
                        staged[address] = pending;
                        if (address == 7 && sii_crc(staged.data()) == (staged[7] & 0xff)) {
                            std::copy(staged.begin(), staged.end(), memory.begin());
                            header_active = false;
                        }
                    } else memory.at(address) = pending;
                }
                status |= error_after_write;
            } else {
                check(!header_active, "no EEPROM reads before full header and CRC are sent");
                check(bytes[1] == 1, "read command"); ++read_commands;
            }
            busy_reads = 2;
        } else throw std::runtime_error("unexpected register write");
        return 1;
    }
    std::uint64_t now_us() override { return clock; }
    void pause_us(unsigned us) override { clock += us; }
};
int main() {
    try {
        std::string error;
        {
            FakeIo io;
            EepromAccess access(io, nullptr, 2);
            check(access.acquire(error), "acquire PDI -> ECAT");
            check(io.owner_writes == std::vector<std::uint8_t>({2}), "force handover even with stale cache");
            std::uint16_t value = 99;
            check(access.read_word(4, value, error) && value == 0, "read original alias");
            check(access.write_word(4, 18, error), "write alias");
            const auto crc = sii_crc(io.memory.data());
            check(access.write_word(7, crc, error), "write CRC");
            check(access.read_word(4, value, error) && value == 18, "read persisted alias");
            check(io.write_commands == 2, "exactly two content writes");
            check(access.restore(error) && io.config == 1 && io.access == 1, "restore original PDI permission");
        }
        {
            FakeIo io; io.config = io.access = 0;
            { EepromAccess access(io, nullptr, 2); check(access.acquire(error), "master initial owner"); }
            check(io.config == 0 && io.owner_writes.back() == 0, "destructor restores original master");
        }
        {
            FakeIo io; io.fail_write_reg = 0x500;
            { EepromAccess access(io, nullptr, 2); check(!access.acquire(error), "failed force write"); }
            check(io.config == 1 && io.owner_writes.back() == 1, "partial acquire restores");
        }
        {
            FakeIo io; io.refuse_owner = true;
            EepromAccess access(io, nullptr, 1);
            check(!access.acquire(error) && io.clock >= 1000, "ownership timeout");
            check(io.write_commands == 0 && io.read_commands == 0, "no command without ownership");
        }
        for (int reg : {0x502}) {
            FakeIo io;
            EepromAccess access(io, nullptr, 2);
            check(access.acquire(error), "acquire before transport failure");
            io.fail_write_reg = reg;
            check(!access.write_word(4, 18, error), "register WKC failure blocks write");
            check(io.write_commands == 0 && io.memory[4] == 0, "no speculative command/retry");
        }
        {
            FakeIo io;
            EepromAccess access(io, nullptr, 2);
            check(access.acquire(error), "acquire before stale readback");
            io.ignore_write = true;
            check(!access.write_word(4, 18, error), "ACK without persistence must fail");
            check(io.write_commands == 1 && io.memory[7] == 0x62, "failed alias leaves CRC untouched");
            check(error.find("实际=0") != std::string::npos, "actual readback is reported");
        }
        for (auto flag : {0x0800, 0x1000, 0x2000, 0x4000}) {
            FakeIo io;
            EepromAccess access(io, nullptr, 2);
            check(access.acquire(error), "acquire before hardware error");
            io.error_after_write = flag;
            check(!access.write_word(4, 18, error), "each hardware error stops");
            check(io.read_commands == 0 && (io.status & flag), "no read command clears write error");
            check(access.restore(error), "restore permission while retaining error");
        }
        {
            FakeIo io; io.status = 0x4020;
            EepromAccess access(io, nullptr, 2);
            check(!access.acquire(error) && io.writes.empty(), "preserve preexisting error without mutations");
        }
        {
            FakeIo io;
            EepromAccess access(io, nullptr, 2);
            check(access.acquire(error), "acquire before busy test");
            io.busy_forever = true;
            check(!access.write_word(4, 18, error) && io.write_commands == 0, "busy timeout blocks writes");
        }
        {
            FakeIo io;
            EepromAccess access(io, nullptr, 2);
            check(access.acquire(error), "acquire before status communication loss");
            io.fail_read_reg = 0x502;
            check(!access.write_word(4, 18, error) && io.write_commands == 0, "lost status must not appear idle");
        }
        {
            FakeIo io;
            EepromAccess access(io, nullptr, 2);
            check(access.acquire(error), "acquire before ownership loss");
            io.config = io.access = 1;
            check(!access.write_word(4, 18, error) && io.write_commands == 0, "check ownership before command");
        }
        {
            FakeIo io;
            EepromAccess access(io, nullptr, 2);
            check(access.acquire(error), "acquire before override loss");
            io.config = io.access = 0; // ECAT access alone must not mask lost override.
            check(!access.write_word(4, 18, error) && io.write_commands == 0, "lost override blocks write");
        }
        {
            FakeIo io; io.config = 2; io.access = 0;
            EepromAccess access(io, nullptr, 2);
            check(access.acquire(error), "already forced owner");
            check(access.restore(error) && io.config == 2, "restore original override exactly");
        }
        {
            FakeIo io;
            EepromAccess access(io, nullptr, 2);
            check(access.acquire(error), "acquire before cleanup failure");
            io.fail_write_reg = 0x500;
            check(!access.restore(error), "restore failure must surface");
            const auto count = io.writes.size();
            check(!access.restore(error) && io.writes.size() == count, "cleanup failure remains visible without retry");
        }
        {
            FakeIo io; io.status |= 0x40;
            EepromAccess access(io, nullptr, 2);
            std::uint16_t value = 0;
            check(access.acquire(error) && access.read_word(7, value, error)
                  && value == 0x62, "64-bit EEPROM read support");
        }
        {
            FakeIo io; io.defer_header_until_crc = true;
            io.memory[4] = 18; io.memory[7] = sii_crc(io.memory.data());
            EepromAccess access(io, nullptr, 2);
            check(access.acquire(error), "acquire deferred header device");
            check(!access.write_word(4, 11, error) && io.memory[4] == 18,
                  "reproduce old immediate-readback failure on deferred commit");
            std::array<std::uint16_t,8> header{};
            std::copy_n(io.memory.begin(),8,header.begin());
            header[4] = 11; header[7] = sii_crc(header.data());
            check(access.write_config_header(header,error), "complete header commits before readback");
            check(io.memory[4] == 11 && io.memory[7] == header[7], "new Alias and CRC persisted");
            check(io.write_commands == 9, "one rejected single-word flow plus eight header writes");
        }
        {
            FakeIo io;
            EepromAccess access(io,nullptr,2);
            check(access.acquire(error), "acquire before invalid header");
            std::array<std::uint16_t,8> header{};
            std::copy_n(io.memory.begin(),8,header.begin());
            header[4] = 11; // Deliberately retain old CRC.
            check(!access.write_config_header(header,error) && io.write_commands == 0,
                  "reject invalid CRC before any content write");
        }
        {
            FakeIo io;
            EepromAccess access(io,nullptr,2);
            check(access.acquire(error), "acquire before header hardware error");
            std::array<std::uint16_t,8> header{};
            std::copy_n(io.memory.begin(),8,header.begin());
            header[4] = 11; header[7] = sii_crc(header.data());
            io.error_after_write = 0x4000;
            check(!access.write_config_header(header,error) && io.write_commands == 1,
                  "stop at first header transport/controller error");
            check(io.read_commands == 0, "do not clear header error by reading EEPROM");
        }
        std::cout << "EEPROM ownership, register protocol, header transaction and failure-path checks passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; return 1;
    }
}
