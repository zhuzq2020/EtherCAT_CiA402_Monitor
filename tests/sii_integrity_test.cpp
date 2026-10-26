#include "sii_integrity.hpp"
#include <array>
#include <cstdlib>
#include <iostream>

void check(bool ok) {
    if (!ok) { std::cerr << "SII integrity regression failed\n"; std::exit(1); }
}
int main() {
    // Known header from the existing D18 backup; stored CRC is 0x62.
    std::array<std::uint16_t, 8> words{{0x0c80, 0x6681, 0, 0, 0, 0, 0, 0x0062}};
    check(robot_maint::sii_crc(words.data()) == 0x62);
    words[4] = 18;
    check(robot_maint::sii_crc(words.data()) != 0x62);
    // Upper byte of checksum word is reserved and is outside CRC coverage.
    auto crc = robot_maint::sii_crc(words.data());
    words[7] = 0xab00 | crc;
    check(robot_maint::sii_crc(words.data()) == crc);
    std::vector<std::uint16_t> before(1024);
    before[0] = 0x0c80; before[1] = 0x6681; before[7] = 98;
    auto after = before;
    after[4] = 18; after[7] = robot_maint::sii_crc(after.data());
    check(after[7] == 174); // Known successful EC-Master write on the real device.
    std::string error;
    check(robot_maint::verify_alias_image(before, after, 18, error));
    check(!robot_maint::verify_alias_image(before, before, 18, error));
    after[0x172] = 1; // PDO description must not change.
    check(!robot_maint::verify_alias_image(before, after, 18, error));
    after[0x172] = 0; after[900] = 1; // Also cover beyond the configuration header.
    check(!robot_maint::verify_alias_image(before, after, 18, error));
    after[900] = 0; after[7] ^= 1;
    check(!robot_maint::verify_alias_image(before, after, 18, error));
    after.resize(64);
    check(!robot_maint::verify_alias_image(before, after, 18, error));
    // Rewrite the real successful Alias 18 image to Alias 11.
    before[4] = 18;
    before[7] = robot_maint::sii_crc(before.data());
    after = before;
    after[4] = 11;
    after[7] = robot_maint::sii_crc(after.data());
    check(robot_maint::verify_alias_image(before, after, 11, error));
    check(!robot_maint::verify_alias_image(before, before, 11, error));
    std::cout << "SII CRC and full-image regression checks passed\n";
}
