#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace robot_maint {
struct MotionFeedback {
    bool valid{false};
    std::uint16_t status{0};
    std::int32_t position{0};
    std::int8_t mode{0};
};
struct MotionOutput { std::uint16_t control{6}; std::int32_t target{0}; };
struct MotionAxisInfo { std::string name, unavailable; double counts_per_deg{0}; };
struct MotionTicket { std::vector<int> axes; std::vector<std::uint64_t> generations; std::string verb; };
class MotionControl {
public:
    enum class Phase { Disabled, Enabling, Enabled, Disabling };
    explicit MotionControl(std::vector<MotionAxisInfo> axes);
    void update(const std::vector<MotionFeedback>& feedback, bool healthy);
    bool enable(const std::vector<int>& axes, MotionTicket& ticket, std::string& error);
    bool jog(const std::vector<int>& axes, bool degrees, double delta, double rpm,
             MotionTicket& ticket, std::string& error);
    // One displacement broadcasts; otherwise values follow the axes' order.
    bool jog(const std::vector<int>& axes, bool degrees, const std::vector<double>& deltas,
             double rpm, MotionTicket& ticket, std::string& error);
    bool disable(const std::vector<int>& axes, MotionTicket& ticket, std::string& error);
    // 0=pending, 1=complete, -1=failed. No state transitions occur here.
    int result(const MotionTicket& ticket, std::string& detail) const;
    void abort(const MotionTicket& ticket, const std::string& reason);
    void stop_all(const std::string& reason);
    std::vector<MotionOutput> outputs() const;
    std::string status(const std::vector<int>& selected = {}) const;
    bool any_active() const;
    int size() const { return static_cast<int>(axes_.size()); }
    const std::vector<MotionAxisInfo>& info() const { return info_; }
private:
    struct Axis {
        Phase phase{Phase::Disabled};
        std::uint64_t generation{0}, group{0};
        std::int32_t command{0}, start{0}, delta{0};
        unsigned elapsed{0}, duration{0}, transition{0};
        std::uint16_t control{6};
        bool moving{false};
        std::string error;
    };
    bool check_axes(const std::vector<int>& axes, std::string& error) const;
    void ticket_for(const std::vector<int>& axes, const std::string& verb, MotionTicket& ticket);
    std::vector<MotionAxisInfo> info_;
    std::vector<Axis> axes_;
    std::vector<MotionFeedback> feedback_;
    bool healthy_{false};
    std::uint64_t next_group_{0};
};
// User labels resolve to 0-based physical indices. D labels retain the existing
// seven-axis right-arm convention and do not use Station Alias values.
bool resolve_motion_axes(const std::vector<std::string>& names, int count,
                         std::vector<int>& result, std::string& error);
}
