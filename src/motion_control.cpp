#include "motion_control.hpp"
#include "tongchuan_def.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>

namespace robot_maint {
namespace {
bool enabled(std::uint16_t sw) { return (sw & 0x006fU) == 0x0027U; }
bool disabled(std::uint16_t sw) {
    return (sw & 0x6f) == 0x21 || (sw & 0x4f) == 0x40
        || (sw & 0x4f) == 8 || (sw & 0x4f) == 0;
}
void append(std::string& text, const std::string& item) { if (!text.empty()) text += "；"; text += item; }
}
MotionControl::MotionControl(std::vector<MotionAxisInfo> axes)
    : info_(std::move(axes)), axes_(info_.size()), feedback_(info_.size()) {}
bool MotionControl::check_axes(const std::vector<int>& axes, std::string& error) const {
    error.clear(); std::set<int> unique;
    if (axes.empty()) { error = "未指定目标轴"; return false; }
    for (int i : axes) {
        if (i < 0 || i >= size()) append(error, "目标轴超出扫描范围");
        else if (!unique.insert(i).second) append(error, info_[i].name + " 重复指定");
        else if (!info_[i].unavailable.empty()) append(error, info_[i].name + " 不可控制：" + info_[i].unavailable);
        else if (!healthy_ || !feedback_[i].valid) append(error, info_[i].name + " 状态读取失败，未下发运动");
    }
    return error.empty();
}
void MotionControl::ticket_for(const std::vector<int>& ids, const std::string& verb, MotionTicket& ticket) {
    ticket = {}; ticket.axes = ids; ticket.verb = verb;
    for (int i : ids) ticket.generations.push_back(axes_[i].generation);
}
bool MotionControl::enable(const std::vector<int>& ids, MotionTicket& ticket, std::string& error) {
    if (!check_axes(ids,error)) return false;
    for (int i : ids) {
        const auto& a = axes_[i]; const auto& f = feedback_[i];
        if (f.status & 8) append(error, info_[i].name + " 存在故障，请先清错");
        else if (f.mode != 8) append(error, info_[i].name + " 不是 CSP 模式");
        else if (a.phase == Phase::Enabling || a.phase == Phase::Disabling || a.moving)
            append(error, info_[i].name + " 正在切换状态或运动");
    }
    if (!error.empty()) return false;
    for (int i : ids) {
        auto& a = axes_[i];
        if (a.phase == Phase::Enabled && enabled(feedback_[i].status)) continue;
        ++a.generation; a.command = feedback_[i].position; a.control = 6;
        a.phase = Phase::Enabling; a.transition = 0; a.error.clear();
    }
    ticket_for(ids,"enable",ticket); return true;
}
bool MotionControl::jog(const std::vector<int>& ids, bool degrees, double delta, double rpm,
                         MotionTicket& ticket, std::string& error) {
    return jog(ids,degrees,std::vector<double>{delta},rpm,ticket,error);
}
bool MotionControl::jog(const std::vector<int>& ids, bool degrees,
                         const std::vector<double>& values, double rpm,
                         MotionTicket& ticket, std::string& error) {
    if (!check_axes(ids,error)) return false;
    if (values.size()!=1 && values.size()!=ids.size()) {
        error="位移数量必须为 1（所有轴相同）或与目标轴数量一致"; return false;
    }
    // Validate every target before changing ANY output or planning state.
    for (int i : ids) {
        const auto& a = axes_[i]; const auto& f = feedback_[i];
        if (a.phase != Phase::Enabled || !enabled(f.status))
            append(error, info_[i].name + " 未使能（" + decode_cia402_state(f.status) + "）");
        else if (f.mode != 8) append(error, info_[i].name + " 不在 CSP 模式，拒绝运动");
        else if (a.moving) append(error, info_[i].name + " 正在运动");
    }
    if (!error.empty()) { error += "；整条点动命令已拒绝，所有目标轴均未开始本次运动"; return false; }
    bool any_displacement=false;
    for (double value:values) {
        if (!std::isfinite(value)) { error="位移必须为有限数值"; return false; }
        any_displacement |= value!=0;
    }
    if (!any_displacement || !std::isfinite(rpm) || rpm < .05 || rpm > 2) {
        error = "至少一个轴的位移必须非零，转速范围 0.05..2 RPM"; return false;
    }
    std::vector<std::int32_t> deltas; unsigned cycles = 500;
    for (std::size_t axis=0;axis<ids.size();++axis) {
        const int i=ids[axis];
        const double delta=values.size()==1?values.front():values[axis];
        const double ratio = info_[i].counts_per_deg;
        const double counts = degrees ? delta * ratio : delta;
        if (!std::isfinite(ratio) || ratio <= 0 || !std::isfinite(counts)
            || counts < std::numeric_limits<std::int32_t>::min()
            || counts > std::numeric_limits<std::int32_t>::max()) {
            append(error,info_[i].name + " 换算后位移超限"); continue;
        }
        const auto n = static_cast<std::int32_t>(std::llround(counts));
        const auto end = std::int64_t(axes_[i].command) + n;
        const double seconds = std::max(1.0, 1.875 * std::abs(double(n)) / (ratio*360) / rpm * 60);
        if ((!n && delta!=0) || end < std::numeric_limits<std::int32_t>::min() || end > std::numeric_limits<std::int32_t>::max()
            || seconds > 120) { append(error,info_[i].name + " 目标超限、位移过小或时长超过 120 秒"); continue; }
        deltas.push_back(n); cycles = std::max(cycles,static_cast<unsigned>(std::ceil(seconds/.002)));
    }
    if (!error.empty()) return false;
    const auto group = ++next_group_;
    for (std::size_t n = 0; n < ids.size(); ++n) {
        auto& a = axes_[ids[n]]; ++a.generation; a.group = group; a.start = a.command;
        a.delta = deltas[n]; a.elapsed = 0; a.duration = cycles; a.moving = true; a.error.clear();
    }
    ticket_for(ids,"jog-enabled",ticket); return true;
}
bool MotionControl::disable(const std::vector<int>& ids, MotionTicket& ticket, std::string& error) {
    // Disabling remains available with stale feedback; completion still needs
    // fresh feedback confirming a disabled CiA-402 state.
    error.clear();
    if (ids.empty()) { error="未指定失能目标轴"; return false; }
    for (int i : ids) if (i < 0 || i >= size()) { error = "目标轴越界"; return false; }
    std::set<std::uint64_t> groups;
    for (int i : ids) if (axes_[i].moving) groups.insert(axes_[i].group);
    for (auto& a : axes_) if (a.moving && groups.count(a.group)) {
        a.moving = false; ++a.generation; a.error = "多轴点动被失能命令取消";
    }
    for (int i : ids) {
        auto& a = axes_[i]; ++a.generation; a.moving = false; a.control = 6;
        a.phase = Phase::Disabling; a.transition = 0; a.error.clear();
    }
    ticket_for(ids,"disable",ticket); return true;
}
void MotionControl::stop_all(const std::string& reason) {
    for (auto& a : axes_) {
        if (a.phase != Phase::Disabled || a.moving) {
            ++a.generation; a.error = reason; a.phase = Phase::Disabling;
            a.control = 6; a.moving = false; a.transition = 0;
        }
    }
}
void MotionControl::abort(const MotionTicket& ticket, const std::string& reason) {
    std::vector<int> matching;
    for (std::size_t j = 0; j < ticket.axes.size(); ++j) {
        int i = ticket.axes[j];
        if (axes_[i].generation == ticket.generations[j]) matching.push_back(i);
    }
    MotionTicket unused; std::string error; disable(matching,unused,error);
    for (int i : matching) axes_[i].error = reason;
}
void MotionControl::update(const std::vector<MotionFeedback>& feedback, bool healthy) {
    if (feedback.size() != axes_.size()) { healthy_ = false; stop_all("PDO 反馈长度异常"); return; }
    feedback_ = feedback; healthy_ = healthy;
    if (!healthy) { stop_all("PDO 通信工作计数异常，全部目标停止"); return; }
    for (int i = 0; i < size(); ++i) {
        auto& a = axes_[i]; const auto& f = feedback_[i];
        if (a.phase == Phase::Disabled) { if (f.valid) a.command = f.position; continue; }
        if (!f.valid) { stop_all(info_[i].name + " 状态读取失败"); return; }
        if (a.phase == Phase::Enabling) {
            if ((f.status & 8) || f.mode != 8 || ++a.transition > 1000) { stop_all(info_[i].name + " 使能失败/超时"); return; }
            if (enabled(f.status)) { a.phase = Phase::Enabled; a.control = 15; }
            else if ((f.status & 0x6f) == 0x23) a.control = 15;
            else if ((f.status & 0x6f) == 0x21) a.control = 7;
            else a.control = 6;
        } else if (a.phase == Phase::Disabling) {
            if (disabled(f.status)) { a.phase = Phase::Disabled; a.command = f.position; }
            else if (++a.transition > 1000) a.error = info_[i].name + " 失能未确认";
        } else if (!enabled(f.status) || f.mode != 8) {
            stop_all(info_[i].name + " 退出使能或 CSP 模式，全部目标停止"); return;
        }
        if (a.phase == Phase::Enabled && a.moving) {
            const double t = double(++a.elapsed) / a.duration;
            a.command = static_cast<std::int32_t>(std::int64_t(a.start) + std::llround(double(a.delta)*smoothstep5(t)));
            if (a.elapsed >= a.duration) a.moving = false;
        }
    }
}
int MotionControl::result(const MotionTicket& ticket, std::string& detail) const {
    bool done = true; detail.clear();
    for (std::size_t j = 0; j < ticket.axes.size(); ++j) {
        int i = ticket.axes[j]; const auto& a = axes_[i];
        if (a.generation != ticket.generations[j] || !a.error.empty()) {
            append(detail,info_[i].name + ": " + (a.error.empty() ? "命令已取消" : a.error)); continue;
        }
        if (ticket.verb == "disable") done &= a.phase == Phase::Disabled && feedback_[i].valid;
        else done &= a.phase == Phase::Enabled && !a.moving && feedback_[i].valid && enabled(feedback_[i].status);
    }
    if (!detail.empty()) return -1;
    if (!done) return 0;
    for (int i : ticket.axes) append(detail, info_[i].name);
    detail += ticket.verb == "enable" ? " [使能成功] 后台持续保持当前位置"
            : ticket.verb == "disable" ? " [失能成功]" : " [点动完成] 保持终点并继续使能";
    return 1;
}
std::vector<MotionOutput> MotionControl::outputs() const {
    std::vector<MotionOutput> output;
    for (const auto& a : axes_) output.push_back({a.control,a.command});
    return output;
}
bool MotionControl::any_active() const {
    for (const auto& a : axes_) if (a.phase != Phase::Disabled) return true;
    return false;
}
std::string MotionControl::status(const std::vector<int>& selected) const {
    std::ostringstream out;
    for (int i = 0; i < size(); ++i) {
        if (!selected.empty() && std::find(selected.begin(),selected.end(),i)==selected.end()) continue;
        out << info_[i].name << " " << (feedback_[i].valid ? decode_cia402_state(feedback_[i].status) : "状态读取失败")
            << " position=" << feedback_[i].position << (axes_[i].moving ? " 运动中" : "") << '\n';
    }
    return out.str();
}
bool resolve_motion_axes(const std::vector<std::string>& names,int count,std::vector<int>& ids,std::string& error) {
    ids.clear(); error.clear(); std::set<int> unique;
    for (auto name : names) {
        const auto original = name; std::size_t start = 0;
        bool d = !name.empty() && (name[0]=='D'||name[0]=='d');
        if (!name.empty() && (d || name[0]=='S'||name[0]=='s')) start=1;
        bool digits = start < name.size();
        for (std::size_t j=start;j<name.size();++j) digits &= name[j]>='0'&&name[j]<='9';
        int n=0;
        if (digits) { try { n=std::stoi(name.substr(start)); } catch (...) { digits=false; } }
        if (d && count==7 && n>=18 && n<=24) n-=17;
        if (!digits||n<1||n>count) append(error,original+" 不是本次扫描中的有效目标轴");
        else if (!unique.insert(n-1).second) append(error,original+" 重复指定同一目标轴");
        else ids.push_back(n-1);
    }
    if (names.empty()) error="未指定目标轴";
    return error.empty();
}
}
