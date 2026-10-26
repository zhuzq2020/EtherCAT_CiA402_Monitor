#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace robot_maint {
// SII configuration CRC: first seven words, low byte first; word 7 stores CRC.
inline std::uint8_t sii_crc(const std::uint16_t* words) {
    std::uint8_t crc = 0xff;
    for (std::size_t i = 0; i < 14; ++i) {
        crc ^= static_cast<std::uint8_t>(words[i / 2] >> ((i % 2) * 8));
        for (int bit = 0; bit < 8; ++bit)
            crc = static_cast<std::uint8_t>((crc << 1) ^ ((crc & 0x80) ? 0x07 : 0));
    }
    return crc;
}
// Validate the entire image: PDO descriptions and vendor areas must not change.
inline bool verify_alias_image(const std::vector<std::uint16_t>& before,
                               const std::vector<std::uint16_t>& after,
                               std::uint16_t alias, std::string& error) {
    if (before.size() < 64 || before.size() != after.size()) {
        error = "完整 SII 镜像长度不一致";
        return false;
    }
    if (after[4] != alias) {
        error = "Alias 回读不一致，目标=" + std::to_string(alias)
              + "，实际=" + std::to_string(after[4]);
        return false;
    }
    auto header = before;
    header[4] = alias;
    const auto expected_crc = static_cast<std::uint16_t>((before[7] & 0xff00U) | sii_crc(header.data()));
    for (std::size_t i = 0; i < before.size(); ++i) {
        const auto expected = i == 4 ? alias : (i == 7 ? expected_crc : before[i]);
        if (after[i] != expected) {
            error = "完整 SII 校验失败，word=" + std::to_string(i)
                  + "，期望=" + std::to_string(expected) + "，实际=" + std::to_string(after[i]);
            return false;
        }
    }
    return true;
}
}
