#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace robot_maint {

class SoemBus;

// Supported Drive Vendors
constexpr std::uint32_t kTongchuanVendorId = 0x00075500U;
constexpr std::uint32_t kEyouVendorId      = 0x00001097U;
constexpr std::uint32_t kFairinoVendorId   = 0x00000CA0U;

// Tongchuan Vendor SDOs
constexpr std::uint16_t SDO_TC_ECAT_ALARM       = 0x302DU; // Pr0.45 ECAT alarm
constexpr std::uint16_t SDO_TC_SET_ZERO         = 0x3801U; // Pr8.01: 5 = encoder zero
constexpr std::uint16_t SDO_TC_CLOSED_LOOP_TYPE = 0x3D00U; // PrD.00 (0: motor, 1: output)
constexpr std::uint16_t SDO_TC_OUTPUT_RES_DIV4  = 0x3D04U; // PrD.04
constexpr std::uint16_t SDO_TC_GEAR_RATIO       = 0x3D06U; // PrD.06 (e.g., 101/81/51)

// Eyou Vendor SDOs
constexpr std::uint16_t SDO_EYOU_SAVE_FLASH     = 0x2130U; // Write 1 to save calibration to Flash
constexpr std::uint16_t SDO_EYOU_EEPROM_UNLOCK  = 0x2679U; // Write 1 to unlock EEPROM, 0 to lock

enum class DriveVendor {
    Unknown,
    Tongchuan,
    Eyou,
    Fairino,
    GenericCia402
};

struct DriveCapabilities {
    DriveVendor vendor{DriveVendor::Unknown};
    std::string vendor_name;
    std::string model_name;
    bool fixed_csp_mode{false};    // Fairino uses fixed CSP(8); skip 0x6060/0x6061 handshake
    bool supports_set_zero{false}; // Hardware encoder zeroing
};

struct AxisProfileDef {
    int joint{0};
    std::string model;
    double counts_per_rev{0.0};
};

struct ArmVendorProfile {
    std::uint32_t vendor_id{0};
    std::string vendor_name;
    std::string driver_type;
    bool fixed_csp_mode{false};
    bool supports_set_zero{false};
    std::string zeroing_method;
    double default_counts_per_rev{0.0};
    std::vector<AxisProfileDef> axes;
};

// Profile configuration loader (from profiles/ directory or defaults)
bool load_profile_from_file(const std::string& filepath, ArmVendorProfile& out_profile);
bool load_profiles_from_dir(const std::string& dirpath);
const ArmVendorProfile* get_vendor_profile(std::uint32_t vendor_id);

DriveVendor classify_vendor(std::uint32_t vendor_id);
std::string vendor_id_to_name(std::uint32_t vendor_id);

bool is_eligible_servo(SoemBus& bus, int slave);

DriveCapabilities inspect_drive_capabilities(SoemBus& bus, int slave,
                                             std::uint32_t vendor_id,
                                             std::uint32_t product_code,
                                             const std::string& dev_name);

double resolve_counts_per_rev(SoemBus& bus, int slave, std::uint32_t vendor_id);

// Vendor-specific encoder zero calibration (slave must be disabled)
bool drive_set_zero(SoemBus& bus, int slave, std::string& detail);

} // namespace robot_maint
