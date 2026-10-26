#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>

namespace robot_maint {
struct JogRequest {
    std::int32_t delta_counts{0};
    double speed_rpm{0.5};
};

// Strict parser for the already-enabled session; never changes servo state.
inline bool parse_enabled_jog(const std::string& line, double counts_per_deg,
                              JogRequest& request, std::string& error) {
    std::istringstream input(line);
    std::string command, option, value;
    input >> command;
    if (command != "jog-enabled") {
        error = "使用 jog-enabled --delta_deg <度> 或 --delta_count <脉冲> [--speed_rpm <转速>]；disable 失能退出";
        return false;
    }
    bool displacement = false, speed = false;
    JogRequest parsed;
    while (input >> option) {
        if (!(input >> value)) { error = "参数缺少数值"; return false; }
        try {
            std::size_t end = 0;
            if (option == "--delta_deg" || option == "--delta_count") {
                if (displacement) { error = "只能指定一种位移参数且不能重复"; return false; }
                double counts = 0;
                if (option == "--delta_count") {
                    counts = static_cast<double>(std::stoll(value, &end));
                } else {
                    counts = std::stod(value, &end) * counts_per_deg;
                }
                if (end != value.size() || !std::isfinite(counts) ||
                    counts < std::numeric_limits<std::int32_t>::min() ||
                    counts > std::numeric_limits<std::int32_t>::max()) {
                    error = "位移无效或超出 int32 范围"; return false;
                }
                parsed.delta_counts = static_cast<std::int32_t>(std::llround(counts));
                displacement = true;
            } else if (option == "--speed_rpm") {
                if (speed) { error = "转速参数不能重复"; return false; }
                parsed.speed_rpm = std::stod(value, &end);
                if (end != value.size() || !std::isfinite(parsed.speed_rpm) ||
                    parsed.speed_rpm < 0.05 || parsed.speed_rpm > 2.0) {
                    error = "转速必须在 0.05~2.0 RPM 范围内"; return false;
                }
                speed = true;
            } else { error = "未知参数: " + option; return false; }
        } catch (...) { error = "无效数值: " + value; return false; }
    }
    if (!displacement || parsed.delta_counts == 0) {
        error = "必须指定非零位移"; return false;
    }
    request = parsed;
    return true;
}
} // namespace robot_maint
