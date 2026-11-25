#include "rpc_session.hpp"
#include "eeprom_access.hpp"

#include <array>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

namespace robot_maint {

namespace {

// Keeps RPC replies strictly on original stdout while routing internal C++ std::cout logs to stderr.
struct StdoutRedirectGuard {
    std::streambuf* orig_cout;
    std::ostream rpc_out;
    StdoutRedirectGuard()
        : orig_cout(std::cout.rdbuf()), rpc_out(orig_cout) {
        std::cout.rdbuf(std::cerr.rdbuf());
    }
    ~StdoutRedirectGuard() {
        std::cout.rdbuf(orig_cout);
    }
};

class BusEepromIo final : public EepromIo {
public:
    BusEepromIo(SoemBus& bus, int slave) : bus_(bus), slave_(slave) {}
    int read(std::uint16_t reg, void* data, std::size_t size) override {
        return bus_.read_esc_reg(slave_, reg, data, size);
    }
    int write(std::uint16_t reg, const void* data, std::size_t size) override {
        return bus_.write_esc_reg(slave_, reg, data, size);
    }
    std::uint64_t now_us() override {
        return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    }
    void pause_us(unsigned us) override {
        std::this_thread::sleep_for(std::chrono::microseconds(us));
    }

private:
    SoemBus& bus_;
    int slave_;
};

struct EepromSession {
    std::unique_ptr<BusEepromIo> io;
    std::unique_ptr<EepromAccess> access;
};

std::string to_hex(const std::uint8_t* data, std::size_t len) {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < len; ++i) {
        oss << std::setw(2) << static_cast<unsigned>(data[i]);
    }
    return oss.str();
}

bool parse_hex_bytes(const std::string& hex, std::vector<std::uint8_t>& out) {
    out.clear();
    if (hex.empty() || (hex.size() % 2) != 0) return false;
    out.reserve(hex.size() / 2);
    for (std::size_t i = 0; i < hex.size(); i += 2) {
        try {
            std::size_t used = 0;
            const unsigned long v = std::stoul(hex.substr(i, 2), &used, 16);
            if (used != 2 || v > 0xFFUL) return false;
            out.push_back(static_cast<std::uint8_t>(v));
        } catch (...) {
            return false;
        }
    }
    return true;
}

std::string json_escape(const std::string& s) {
    std::ostringstream oss;
    for (char c : s) {
        switch (c) {
            case '"':  oss << "\\\""; break;
            case '\\': oss << "\\\\"; break;
            case '\n': oss << "\\n"; break;
            case '\r': oss << "\\r"; break;
            case '\t': oss << "\\t"; break;
            default:   oss << c; break;
        }
    }
    return oss.str();
}

std::string single_line(std::string s) {
    for (char& c : s) {
        if (c == '\n' || c == '\r') c = ' ';
    }
    return s;
}

bool parse_u32(const std::string& s, std::uint32_t& out) {
    if (s.empty()) return false;
    try {
        std::size_t used = 0;
        const int base = (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) ? 16 : 10;
        const unsigned long val = std::stoul(s, &used, base);
        if (used != s.size()) return false;
        out = static_cast<std::uint32_t>(val);
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_csv_axes(const std::string& csv, int total_slaves,
                    std::vector<int>& out, std::string& error) {
    std::vector<std::string> labels;
    std::istringstream ss(csv);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        if (!tok.empty()) labels.push_back(tok);
    }
    return resolve_slave_axes(labels, total_slaves, out, error);
}

bool parse_csv_doubles(const std::string& csv, std::vector<double>& out) {
    out.clear();
    std::istringstream ss(csv);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        if (tok.empty()) return false;
        try {
            std::size_t used = 0;
            const double val = std::stod(tok, &used);
            if (used != tok.size()) return false;
            out.push_back(val);
        } catch (...) {
            return false;
        }
    }
    return !out.empty();
}

} // namespace

int run_rpc_session(ArmController& controller) {
    StdoutRedirectGuard io_guard;
    auto& rpc_out = io_guard.rpc_out;

    if (!controller.init_bus()) {
        rpc_out << "ERR 总线初始化失败\n" << std::flush;
        return 1;
    }

    auto& bus = controller.bus();
    std::map<int, EepromSession> eeprom_sessions;
    std::vector<int> active_motion_slaves;

    auto release_all_eeprom = [&]() {
        for (auto& kv : eeprom_sessions) {
            std::string err;
            if (kv.second.access) {
                if (kv.second.access->restore(err) && kv.second.access->has_original()) {
                    bus.raw_slave_mut(kv.first).eep_pdi = (kv.second.access->original().config & 1) ? 1 : 0;
                }
            }
        }
        eeprom_sessions.clear();
    };

    rpc_out << "READY " << controller.slave_count() << '\n' << std::flush;

    std::string line;
    while (!ArmController::is_stop_requested() && std::getline(std::cin, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd;
        if (cmd.empty()) continue;

        if (cmd == "quit" || cmd == "exit") {
            controller.close_motion();
            release_all_eeprom();
            rpc_out << "OK\n" << std::flush;
            break;
        }

        if (cmd == "slaves") {
            bus.refresh_states();
            const auto& axes = controller.axes();
            std::ostringstream out;
            out << "OK [";
            for (std::size_t i = 0; i < axes.size(); ++i) {
                const auto& ax = axes[i];
                const auto& raw = bus.raw_slave(ax.slave_index);
                if (i > 0) out << ',';
                out << "{\"slave\":" << ax.slave_index
                    << ",\"vendor\":" << ax.vendor_id
                    << ",\"product\":" << ax.product_code
                    << ",\"alias\":" << raw.aliasadr
                    << ",\"configadr\":" << raw.configadr
                    << ",\"mbx_proto\":" << static_cast<unsigned>(raw.mbx_proto)
                    << ",\"state\":" << raw.state
                    << ",\"name\":\"" << json_escape(ax.axis_name) << "\""
                    << ",\"vendor_name\":\"" << json_escape(ax.vendor_name) << "\""
                    << ",\"model\":\"" << json_escape(ax.model_name) << "\""
                    << ",\"is_servo\":" << (ax.is_servo ? "true" : "false")
                    << ",\"counts_per_deg\":" << std::setprecision(12) << ax.counts_per_deg
                    << "}";
            }
            out << "]\n";
            rpc_out << out.str() << std::flush;
            continue;
        }

        if (cmd == "read-sdo") {
            int slave = 0;
            std::string idx_str, sub_str;
            int timeout_us = EC_TIMEOUTRXM;
            std::uint32_t idx = 0, sub = 0;
            if (!(iss >> slave >> idx_str >> sub_str) ||
                !parse_u32(idx_str, idx) || !parse_u32(sub_str, sub) ||
                idx > 0xFFFFU || sub > 0xFFU) {
                rpc_out << "ERR 参数无效: read-sdo <slave> <index> <subindex> [timeout_us]\n" << std::flush;
                continue;
            }
            if (iss >> timeout_us && timeout_us <= 0) timeout_us = EC_TIMEOUTRXM;
            (void)bus.pop_coe_errors();
            std::vector<std::uint8_t> bytes;
            if (bus.read_sdo_bytes(slave, static_cast<std::uint16_t>(idx),
                                   static_cast<std::uint8_t>(sub), bytes, timeout_us)) {
                rpc_out << "OK " << to_hex(bytes.data(), bytes.size()) << '\n' << std::flush;
            } else {
                const auto coe = single_line(bus.pop_coe_errors());
                rpc_out << "ERR SDO读取失败" << (coe.empty() ? "" : (": " + coe)) << '\n' << std::flush;
            }
            continue;
        }

        if (cmd == "write-sdo") {
            int slave = 0, ca = 0;
            std::string idx_str, sub_str, hex_data;
            int timeout_us = EC_TIMEOUTRXM;
            std::uint32_t idx = 0, sub = 0;
            std::vector<std::uint8_t> bytes;
            if (!(iss >> slave >> idx_str >> sub_str >> ca >> hex_data) ||
                !parse_u32(idx_str, idx) || !parse_u32(sub_str, sub) ||
                idx > 0xFFFFU || sub > 0xFFU || !parse_hex_bytes(hex_data, bytes)) {
                rpc_out << "ERR 参数无效: write-sdo <slave> <index> <subindex> <ca:0|1> <hex_bytes> [timeout_us]\n" << std::flush;
                continue;
            }
            if (iss >> timeout_us && timeout_us <= 0) timeout_us = EC_TIMEOUTRXM;
            (void)bus.pop_coe_errors();
            if (bus.write_sdo_bytes(slave, static_cast<std::uint16_t>(idx),
                                    static_cast<std::uint8_t>(sub), bytes, ca != 0, timeout_us)) {
                rpc_out << "OK\n" << std::flush;
            } else {
                const auto coe = single_line(bus.pop_coe_errors());
                rpc_out << "ERR SDO写入失败" << (coe.empty() ? "" : (": " + coe)) << '\n' << std::flush;
            }
            continue;
        }

        if (cmd == "read-esc") {
            int slave = 0;
            std::string reg_str;
            std::size_t size = 0;
            std::uint32_t reg = 0;
            if (!(iss >> slave >> reg_str >> size) || !parse_u32(reg_str, reg) ||
                reg > 0xFFFFU || size == 0 || size > 64) {
                rpc_out << "ERR 参数无效: read-esc <slave> <reg> <size>\n" << std::flush;
                continue;
            }
            std::vector<std::uint8_t> buf(size, 0);
            const int wkc = bus.read_esc_reg(slave, static_cast<std::uint16_t>(reg), buf.data(), size);
            if (wkc == 1) {
                rpc_out << "OK " << to_hex(buf.data(), buf.size()) << '\n' << std::flush;
            } else {
                rpc_out << "ERR 读ESC寄存器失败 WKC=" << wkc << '\n' << std::flush;
            }
            continue;
        }

        if (cmd == "write-esc") {
            int slave = 0;
            std::string reg_str, hex_data;
            std::uint32_t reg = 0;
            std::vector<std::uint8_t> bytes;
            if (!(iss >> slave >> reg_str >> hex_data) || !parse_u32(reg_str, reg) ||
                reg > 0xFFFFU || !parse_hex_bytes(hex_data, bytes) || bytes.size() > 64) {
                rpc_out << "ERR 参数无效: write-esc <slave> <reg> <hex_bytes>\n" << std::flush;
                continue;
            }
            const int wkc = bus.write_esc_reg(slave, static_cast<std::uint16_t>(reg), bytes.data(), bytes.size());
            if (wkc == 1) rpc_out << "OK\n" << std::flush;
            else rpc_out << "ERR 写ESC寄存器失败 WKC=" << wkc << '\n' << std::flush;
            continue;
        }

        if (cmd == "eeprom-acquire") {
            int slave = 0;
            if (!(iss >> slave) || slave < 1 || slave > bus.slave_count()) {
                rpc_out << "ERR 从站索引无效\n" << std::flush;
                continue;
            }
            if (eeprom_sessions.count(slave)) {
                rpc_out << "ERR EEPROM 会话已打开\n" << std::flush;
                continue;
            }
            EepromSession sess;
            sess.io = std::make_unique<BusEepromIo>(bus, slave);
            sess.access = std::make_unique<EepromAccess>(*sess.io, &std::cerr);
            std::string detail;
            if (!sess.access->acquire(detail)) {
                std::string cleanup;
                if (!sess.access->restore(cleanup)) detail += "；恢复失败: " + cleanup;
                rpc_out << "ERR " << single_line(detail) << '\n' << std::flush;
                continue;
            }
            bus.raw_slave_mut(slave).eep_pdi = 0;
            const auto& orig = sess.access->original();
            std::ostringstream out;
            out << "OK {\"config\":" << unsigned(orig.config)
                << ",\"access\":" << unsigned(orig.access)
                << ",\"status\":" << orig.status << "}\n";
            eeprom_sessions.emplace(slave, std::move(sess));
            rpc_out << out.str() << std::flush;
            continue;
        }

        if (cmd == "eeprom-restore") {
            int slave = 0;
            if (!(iss >> slave)) {
                rpc_out << "ERR 从站索引无效\n" << std::flush;
                continue;
            }
            auto it = eeprom_sessions.find(slave);
            if (it == eeprom_sessions.end()) {
                rpc_out << "OK\n" << std::flush;
                continue;
            }
            std::string detail;
            const bool ok = it->second.access->restore(detail);
            if (ok && it->second.access->has_original()) {
                bus.raw_slave_mut(slave).eep_pdi = (it->second.access->original().config & 1) ? 1 : 0;
            }
            eeprom_sessions.erase(it);
            if (ok) rpc_out << "OK\n" << std::flush;
            else rpc_out << "ERR " << single_line(detail) << '\n' << std::flush;
            continue;
        }

        if (cmd == "eeprom-read") {
            int slave = 0;
            std::uint32_t start_word = 0, count = 0;
            if (!(iss >> slave >> start_word >> count) || count == 0 || count > 65536 || start_word + count > 65536) {
                rpc_out << "ERR 参数无效: eeprom-read <slave> <start_word> <count>\n" << std::flush;
                continue;
            }
            auto it = eeprom_sessions.find(slave);
            if (it == eeprom_sessions.end()) {
                rpc_out << "ERR EEPROM 控制权尚未获取\n" << std::flush;
                continue;
            }
            std::vector<std::uint8_t> bytes(count * 2U);
            std::string detail;
            bool ok = true;
            for (std::uint32_t i = 0; i < count; ++i) {
                std::uint16_t word = 0;
                if (!it->second.access->read_word(static_cast<std::uint16_t>(start_word + i), word, detail)) {
                    ok = false;
                    break;
                }
                bytes[2 * i] = static_cast<std::uint8_t>(word & 0xFFU);
                bytes[2 * i + 1] = static_cast<std::uint8_t>(word >> 8);
            }
            if (ok) rpc_out << "OK " << to_hex(bytes.data(), bytes.size()) << '\n' << std::flush;
            else rpc_out << "ERR " << single_line(detail) << '\n' << std::flush;
            continue;
        }

        if (cmd == "eeprom-write-header") {
            int slave = 0;
            std::string hex_data;
            std::vector<std::uint8_t> bytes;
            if (!(iss >> slave >> hex_data) || !parse_hex_bytes(hex_data, bytes) || bytes.size() != 16) {
                rpc_out << "ERR 参数无效: eeprom-write-header <slave> <16byte_hex>\n" << std::flush;
                continue;
            }
            auto it = eeprom_sessions.find(slave);
            if (it == eeprom_sessions.end()) {
                rpc_out << "ERR EEPROM 控制权尚未获取\n" << std::flush;
                continue;
            }
            std::array<std::uint16_t, 8> words{};
            for (std::size_t i = 0; i < 8; ++i) {
                words[i] = static_cast<std::uint16_t>(bytes[2 * i] | (std::uint16_t(bytes[2 * i + 1]) << 8));
            }
            std::string detail;
            if (it->second.access->write_config_header(words, detail)) rpc_out << "OK\n" << std::flush;
            else rpc_out << "ERR " << single_line(detail) << '\n' << std::flush;
            continue;
        }

        if (cmd == "motion-open") {
            std::string axes_csv;
            std::vector<int> targets;
            std::string err;
            if (iss >> axes_csv) {
                if (!parse_csv_axes(axes_csv, controller.slave_count(), targets, err)) {
                    rpc_out << "ERR " << single_line(err) << '\n' << std::flush;
                    continue;
                }
            } else {
                targets = controller.eligible_servo_slaves();
            }
            if (!controller.open_motion(targets, err)) {
                rpc_out << "ERR " << single_line(err) << '\n' << std::flush;
                continue;
            }
            active_motion_slaves = targets;
            std::ostringstream out;
            out << "OK [";
            for (std::size_t i = 0; i < targets.size(); ++i) {
                if (i > 0) out << ',';
                out << targets[i];
            }
            out << "]\n";
            rpc_out << out.str() << std::flush;
            continue;
        }

        if (cmd == "motion-move") {
            int move_ms = 0;
            std::string deg_csv;
            std::vector<double> degs;
            if (!(iss >> move_ms >> deg_csv) || move_ms <= 0 || !parse_csv_doubles(deg_csv, degs)) {
                rpc_out << "ERR 参数无效: motion-move <move_ms> <deg1,deg2,...>\n" << std::flush;
                continue;
            }
            std::string err;
            if (controller.move_axes_to(active_motion_slaves, degs, move_ms, err, false)) {
                rpc_out << "OK\n" << std::flush;
            } else {
                controller.close_motion();
                active_motion_slaves.clear();
                rpc_out << "ERR " << single_line(err) << '\n' << std::flush;
            }
            continue;
        }

        if (cmd == "motion-hold") {
            int hold_ms = 0;
            if (!(iss >> hold_ms) || hold_ms < 0) {
                rpc_out << "ERR 参数无效: motion-hold <hold_ms>\n" << std::flush;
                continue;
            }
            std::string err;
            if (controller.hold_motion(hold_ms, err)) {
                rpc_out << "OK\n" << std::flush;
            } else {
                controller.close_motion();
                active_motion_slaves.clear();
                rpc_out << "ERR " << single_line(err) << '\n' << std::flush;
            }
            continue;
        }

        if (cmd == "motion-status") {
            std::vector<AxisMotionFeedback> fb;
            BusDiagnostics diag;
            std::string err;
            if (!controller.read_motion_feedback(active_motion_slaves, fb, diag, err)) {
                rpc_out << "ERR " << single_line(err) << '\n' << std::flush;
                continue;
            }
            std::ostringstream out;
            out << "OK {\"wkc\":" << diag.last_wkc
                << ",\"expected_wkc\":" << diag.expected_wkc
                << ",\"cycles\":" << diag.total_cycles
                << ",\"retries\":" << diag.retried_cycles
                << ",\"max_cycle_us\":" << diag.max_cycle_us
                << ",\"max_late_us\":" << diag.max_late_us
                << ",\"axes\":[";
            for (std::size_t i = 0; i < fb.size(); ++i) {
                if (i > 0) out << ',';
                out << "{\"slave\":" << fb[i].slave_index
                    << ",\"name\":\"" << json_escape(fb[i].axis_name) << "\""
                    << ",\"deg\":" << std::fixed << std::setprecision(4) << fb[i].position_deg
                    << ",\"counts\":" << fb[i].position_counts
                    << ",\"torque\":" << fb[i].torque_raw
                    << ",\"sw\":" << fb[i].status_word
                    << ",\"err\":" << fb[i].error_code
                    << "}";
            }
            out << "]}\n";
            rpc_out << out.str() << std::flush;
            continue;
        }

        if (cmd == "motion-close") {
            controller.close_motion();
            active_motion_slaves.clear();
            rpc_out << "OK\n" << std::flush;
            continue;
        }

        rpc_out << "ERR 未知 RPC 命令: " << cmd << '\n' << std::flush;
    }

    controller.close_motion();
    release_all_eeprom();
    controller.close_bus();
    return 0;
}

} // namespace robot_maint
