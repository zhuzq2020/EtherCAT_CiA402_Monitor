#include "arm_controller.hpp"
#include "cia402_defs.hpp"
#include "drive_profile.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace robot_maint;

void check(bool ok, const char* msg) {
    if (!ok) throw std::runtime_error(msg);
}

int main() {
    try {
        // 1. Axis resolution across Fairino (J1..J7), Tongchuan/Eyou (D18..D24), Generic (S1..SN, 1..N)
        check(axis_name_to_slave("J1", 7) == 1, "J1 -> 1");
        check(axis_name_to_slave("j7", 7) == 7, "j7 -> 7");
        check(axis_name_to_slave("D18", 7) == 1, "D18 -> 1 (7-axis)");
        check(axis_name_to_slave("D24", 7) == 7, "D24 -> 7 (7-axis)");
        check(axis_name_to_slave("D18", 17) == 18, "D18 -> 18 (17-slave)");
        check(axis_name_to_slave("S2", 17) == 2, "S2 -> 2");
        check(axis_name_to_slave("7", 7) == 7, "7 -> 7");
        check(axis_name_to_slave("D18oops", 7) == -1, "malformed rejected");
        check(axis_name_to_slave("0", 7) == -1, "0 rejected");

        std::vector<int> resolved;
        std::string err;
        check(resolve_slave_axes({"J1", "J7"}, 7, resolved, err) && resolved == std::vector<int>({1, 7}),
              "resolve J1,J7");
        check(!resolve_slave_axes({"D18", "1"}, 7, resolved, err), "duplicate hardware target rejected");
        check(!resolve_slave_axes({"D24"}, 3, resolved, err), "out of range rejected");

        // 2. Display naming
        check(slave_to_axis_name(1, 7, true) == "J1", "Fairino prefers J1");
        check(slave_to_axis_name(1, 7, false) == "D18", "7-axis non-Fairino uses D18");
        check(slave_to_axis_name(2, 17, false) == "S2", "17-slave uses S2");

        // 3. Vendor classification & capabilities
        check(classify_vendor(kFairinoVendorId) == DriveVendor::Fairino, "Fairino vendor");
        check(classify_vendor(kTongchuanVendorId) == DriveVendor::Tongchuan, "Tongchuan vendor");
        check(classify_vendor(kEyouVendorId) == DriveVendor::Eyou, "Eyou vendor");
        check(vendor_id_to_name(kFairinoVendorId).find("Fairino") != std::string::npos, "Fairino name");

        SoemBus dummy_bus("lo");
        auto f_caps = inspect_drive_capabilities(dummy_bus, 3, kFairinoVendorId, 2, "");
        check(f_caps.fixed_csp_mode && f_caps.supports_set_zero && f_caps.model_name == "ART-70-80-M",
              "Fairino capabilities");
        auto tc_caps = inspect_drive_capabilities(dummy_bus, 1, kTongchuanVendorId, 2, "");
        check(!tc_caps.fixed_csp_mode && tc_caps.supports_set_zero, "Tongchuan capabilities");
        auto ey_caps = inspect_drive_capabilities(dummy_bus, 1, kEyouVendorId, 2, "");
        check(!ey_caps.fixed_csp_mode && ey_caps.supports_set_zero, "Eyou capabilities");

        // Profile loader checks
        const auto* faao_prof = get_vendor_profile(kFairinoVendorId);
        check(faao_prof != nullptr && faao_prof->supports_set_zero && faao_prof->fixed_csp_mode, "Fairino profile loaded");
        check(faao_prof->axes.size() >= 7 && faao_prof->axes[2].counts_per_rev == 1982464.0, "Fairino axis 3 counts");

        // 4. Smoothstep5 boundary properties
        check(smoothstep5(-0.5) == 0.0 && smoothstep5(0.0) == 0.0, "smoothstep5 start");
        check(std::abs(smoothstep5(0.5) - 0.5) < 1e-12, "smoothstep5 mid");
        check(smoothstep5(1.0) == 1.0 && smoothstep5(1.5) == 1.0, "smoothstep5 end");

        // 5. CiA-402 statusword decode
        check(is_operation_enabled(0x0027), "0x0027 enabled");
        check(!is_operation_enabled(0x0021), "0x0021 not enabled");
        check(is_disabled_state(0x0040) && is_disabled_state(0x0021), "disabled states");

        std::cout << "SDK core unit checks passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
