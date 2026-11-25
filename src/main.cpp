#include "arm_controller.hpp"
#include "cia402_defs.hpp"
#include "rpc_session.hpp"

#include <csignal>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

void sigint_handler(int) {
    robot_maint::ArmController::request_stop();
}

bool parse_unsigned(const std::string& text, unsigned max, unsigned& result) {
    if (text.empty() || text[0] == '-' || text[0] == '+') return false;
    try {
        std::size_t end = 0;
        const int base = (text.size() > 2 && text[0] == '0' &&
                          (text[1] == 'x' || text[1] == 'X')) ? 16 : 10;
        const auto parsed = std::stoul(text, &end, base);
        if (end != text.size() || parsed > max) return false;
        result = static_cast<unsigned>(parsed);
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_double(const std::string& text, double& out) {
    if (text.empty()) return false;
    try {
        std::size_t end = 0;
        out = std::stod(text, &end);
        return end == text.size();
    } catch (...) {
        return false;
    }
}

bool parse_positive_int(const std::string& text, int& out) {
    if (text.empty() || text[0] == '-') return false;
    try {
        std::size_t end = 0;
        const long val = std::stol(text, &end, 10);
        if (end != text.size() || val <= 0 || val > 3600000L) return false;
        out = static_cast<int>(val);
        return true;
    } catch (...) {
        return false;
    }
}

std::string format_sdo_value(const std::vector<std::uint8_t>& bytes) {
    std::ostringstream out;
    out << "长度 " << bytes.size() << " 字节, 原始字节 (低位在前):";
    for (const auto byte : bytes) {
        out << ' ' << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(byte);
    }
    if (bytes.size() == 1 || bytes.size() == 2 || bytes.size() == 4 || bytes.size() == 8) {
        std::uint64_t number = 0;
        for (std::size_t i = 0; i < bytes.size(); ++i) {
            number |= static_cast<std::uint64_t>(bytes[i]) << (8 * i);
        }
        const unsigned bits = static_cast<unsigned>(bytes.size() * 8);
        if ((number & (std::uint64_t{1} << (bits - 1))) != 0 && bits < 64) {
            number |= ~std::uint64_t{0} << bits;
        }
        std::int64_t signed_number = 0;
        std::memcpy(&signed_number, &number, sizeof(number));
        const std::uint64_t unsigned_number = (bits == 64)
            ? number : (number & ((std::uint64_t{1} << bits) - 1));
        out << std::dec << ", 无符号: " << unsigned_number << ", 有符号: " << signed_number;
    }
    return out.str();
}

bool encode_sdo_scalar(const std::string& type_name, const std::string& val_str,
                       std::vector<std::uint8_t>& out, std::string& error) {
    out.clear();
    try {
        std::size_t used = 0;
        const int base = (val_str.size() > 2 && val_str[0] == '0' &&
                          (val_str[1] == 'x' || val_str[1] == 'X')) ? 16 : 10;
        if (type_name == "u8" || type_name == "u16" || type_name == "u32") {
            const unsigned long v = std::stoul(val_str, &used, base);
            if (used != val_str.size()) { error = "无效无符号整数: " + val_str; return false; }
            const std::size_t bytes = (type_name == "u8") ? 1 : (type_name == "u16" ? 2 : 4);
            const unsigned long max_v = (bytes == 1) ? 0xFFUL : (bytes == 2 ? 0xFFFFUL : 0xFFFFFFFFUL);
            if (v > max_v) { error = "数值超出 " + type_name + " 范围"; return false; }
            for (std::size_t i = 0; i < bytes; ++i) out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFFU));
            return true;
        }
        if (type_name == "i8" || type_name == "i16" || type_name == "i32") {
            const long v = std::stol(val_str, &used, base);
            if (used != val_str.size()) { error = "无效有符号整数: " + val_str; return false; }
            const std::size_t bytes = (type_name == "i8") ? 1 : (type_name == "i16" ? 2 : 4);
            const long min_v = (bytes == 1) ? -128L : (bytes == 2 ? -32768L : -2147483648L);
            const long max_v = (bytes == 1) ? 127L : (bytes == 2 ? 32767L : 2147483647L);
            if (v < min_v || v > max_v) { error = "数值超出 " + type_name + " 范围"; return false; }
            const std::uint32_t raw = static_cast<std::uint32_t>(static_cast<std::int32_t>(v));
            for (std::size_t i = 0; i < bytes; ++i) out.push_back(static_cast<std::uint8_t>((raw >> (8 * i)) & 0xFFU));
            return true;
        }
        error = "不支持的数据类型: " + type_name + " (可选: u8, i8, u16, i16, u32, i32)";
        return false;
    } catch (...) {
        error = "解析数值失败: " + val_str;
        return false;
    }
}

void show_help(const char* prog) {
    std::cout << "\n================================================================================\n"
              << " 通用机械臂 EtherCAT 控制与维护 SDK 工具 (arm_maint)\n"
              << " 支持 法奥 (Fairino) / 同川 (Tongchuan) / 意优 (Eyou) / 标准 CiA-402 伺服\n"
              << "================================================================================\n\n"
              << "用法:\n"
              << "  " << prog << " [-i <iface>] <命令> [参数...]\n\n"
              << "全局选项:\n"
              << "  -i, --iface <NAME>    指定以太网接口名称 (默认: eno0)\n"
              << "  -h, --help            显示本帮助信息\n\n"
              << "通用底层命令:\n"
              << "  status [--detail]     扫描总线拓扑、厂商、型号、CiA-402 状态、故障码与当前位置\n"
              << "  read-sdo              读取 CoE 对象 (--index 0xHHHH [--subindex 0] [--axis A | --all])\n"
              << "  write-sdo             写入 CoE 对象 (--axis A --index 0xHHHH --type u8|i8|u16|i16|u32|i32 --value V --confirm)\n"
              << "  set-zero              标定当前位置为编码器绝对零点 (法奥/同川/意优，需 --axis A | --all --confirm)\n"
              << "  jog                   单轴相对平滑点动并保持终点: jog <axis> <deg> [ms] 或 --axis A --deg D [--ms M] [--abs]\n"
              << "  jog-abs               单轴绝对角度平滑点动并保持终点: jog-abs <axis> <abs_deg> [ms]\n"
              << "  zero                  单轴平滑运动归零 0.000°: zero <axis> [ms]\n\n"
              << "Python 流程脚本 (位于 scripts/):\n"
              << "  scripts/pose_control.py   多姿态同步平滑轨迹控制 (法奥/同川/意优)\n"
              << "  scripts/backup_sii.py     EEPROM 完整 SII 备份\n"
              << "  scripts/read_alias.py     读取 ESC / EEPROM Alias 与 CRC\n"
              << "  scripts/write_alias.py    安全写入 Station Alias 并校验全镜像\n"
              << "  scripts/configure_pdo.py  同川 PDO 映射核验与写入\n\n";
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        show_help(argv[0]);
        return 1;
    }

    std::signal(SIGINT, sigint_handler);
    std::signal(SIGTERM, sigint_handler);

    std::string iface = "eno0";
    std::string command;
    std::vector<std::string> positional;

    std::string target_axis;
    bool all_axes = false;
    bool confirmed = false;
    bool detailed = false;
    bool jog_abs_flag = false;

    double jog_deg = 1.0;
    bool has_jog_deg = false;
    int jog_ms = -1;

    unsigned sdo_index = 0;
    unsigned sdo_subindex = 0;
    bool has_sdo_index = false;
    std::string sdo_type;
    std::string sdo_value;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            show_help(argv[0]);
            return 0;
        } else if (arg == "-i" || arg == "--iface") {
            if (++i >= argc) { std::cerr << "[错误] 缺少网卡名称参数。\n"; return 1; }
            iface = argv[i];
        } else if (arg == "--axis") {
            if (++i >= argc) { std::cerr << "[错误] --axis 缺少参数。\n"; return 1; }
            target_axis = argv[i];
        } else if (arg == "--all") {
            all_axes = true;
        } else if (arg == "--confirm") {
            confirmed = true;
        } else if (arg == "--detail" || arg == "-v") {
            detailed = true;
        } else if (arg == "--abs") {
            jog_abs_flag = true;
        } else if (arg == "--deg") {
            if (++i >= argc || !parse_double(argv[i], jog_deg)) {
                std::cerr << "[错误] --deg 必须是有效的数值。\n";
                return 1;
            }
            has_jog_deg = true;
        } else if (arg == "--ms") {
            if (++i >= argc || !parse_positive_int(argv[i], jog_ms)) {
                std::cerr << "[错误] --ms 必须是正整数。\n";
                return 1;
            }
        } else if (arg == "--index" || arg == "--subindex") {
            const bool is_index = (arg == "--index");
            unsigned parsed = 0;
            if (++i >= argc || !parse_unsigned(argv[i], is_index ? 0xFFFFU : 0xFFU, parsed)) {
                std::cerr << "[错误] " << arg << " 必须是有效的十进制或 0x 十六进制数。\n";
                return 1;
            }
            if (is_index) { sdo_index = parsed; has_sdo_index = true; }
            else sdo_subindex = parsed;
        } else if (arg == "--type") {
            if (++i >= argc) return 1;
            sdo_type = argv[i];
        } else if (arg == "--value") {
            if (++i >= argc) return 1;
            sdo_value = argv[i];
        } else if (command.empty() && !arg.empty() && arg[0] != '-') {
            command = arg;
        } else {
            positional.push_back(arg);
        }
    }

    if (command.empty()) {
        show_help(argv[0]);
        return 1;
    }

    if (command != "status" && command != "read-sdo" && command != "write-sdo" &&
        command != "set-zero" && command != "jog" && command != "jog-abs" &&
        command != "zero" && command != "rpc") {
        std::cerr << "[错误] 未知命令: " << command << "，请运行 --help 查看支持命令。\n";
        return 1;
    }

    if (command == "read-sdo" && !has_sdo_index) {
        std::cerr << "[错误] read-sdo 必须指定 --index，例如 --index 0x6064。\n";
        return 1;
    }

    std::vector<std::uint8_t> write_sdo_payload;
    if (command == "write-sdo") {
        if (!confirmed) {
            std::cerr << "[安全拦截] write-sdo 会写入从站 SDO 对象，请追加 --confirm。\n";
            return 1;
        }
        if (target_axis.empty() || !has_sdo_index || sdo_type.empty() || sdo_value.empty()) {
            std::cerr << "[错误] write-sdo 必须指定 --axis, --index, --type, --value, --confirm。\n";
            return 1;
        }
        std::string err;
        if (!encode_sdo_scalar(sdo_type, sdo_value, write_sdo_payload, err)) {
            std::cerr << "[错误] " << err << '\n';
            return 1;
        }
    }

    if (command == "set-zero") {
        if (!confirmed) {
            std::cerr << "\n[安全拦截] 标定零点会将当前机械位置永久写入电机绝对值编码器 Flash！\n"
                      << "           若您确认机械臂已处于标定原点工装位置，请追加参数: --confirm\n\n";
            return 1;
        }
        if (!all_axes && target_axis.empty()) {
            std::cerr << "[错误] 请指定标零目标: --axis <AXIS> 或 --all\n";
            return 1;
        }
    }

    if (command == "jog" || command == "jog-abs" || command == "zero") {
        if (command == "jog-abs") jog_abs_flag = true;
        std::size_t pos_idx = 0;
        if (target_axis.empty()) {
            if (pos_idx >= positional.size()) {
                std::cerr << "[错误] 请指定目标轴 (例如: " << command << " J7 ... 或 --axis J7)\n";
                return 1;
            }
            target_axis = positional[pos_idx++];
        }
        if (command != "zero" && !has_jog_deg) {
            if (pos_idx < positional.size()) {
                if (!parse_double(positional[pos_idx++], jog_deg)) {
                    std::cerr << "[错误] 无效的旋转角度: " << positional[pos_idx - 1] << '\n';
                    return 1;
                }
            } else {
                jog_deg = jog_abs_flag ? 0.0 : 1.0;
            }
        }
        if (jog_ms < 0) {
            if (pos_idx < positional.size()) {
                if (!parse_positive_int(positional[pos_idx++], jog_ms)) {
                    std::cerr << "[错误] 无效的运动时长(ms): " << positional[pos_idx - 1] << '\n';
                    return 1;
                }
            } else {
                jog_ms = (command == "zero") ? 5000 : (jog_abs_flag ? 4000 : 3000);
            }
        }
    }

    robot_maint::ArmController controller(iface);

    if (command == "rpc") {
        return robot_maint::run_rpc_session(controller);
    }

    std::cout << "[INFO] 网口: " << iface << " | 指令: " << command << "\n";
    if (!controller.init_bus()) {
        return 1;
    }
    const int total_slaves = controller.slave_count();

    auto resolve_single_axis = [&](const std::string& label) -> int {
        const int s = robot_maint::axis_name_to_slave(label, total_slaves);
        if (s < 1 || s > total_slaves) {
            std::cerr << "[错误] 无法识别指定轴或从站超出总线范围: " << label
                      << " (当前总线从站总数: " << total_slaves << ")\n";
            return -1;
        }
        return s;
    };

    if (command == "status") {
        const auto list = controller.scan_status(detailed);
        controller.print_status_table(list, detailed);
        return 0;
    }

    if (command == "read-sdo") {
        int target_slave = -1;
        if (!all_axes && target_axis.empty()) all_axes = true;
        if (!all_axes) {
            target_slave = resolve_single_axis(target_axis);
            if (target_slave < 0) return 1;
        }
        bool all_ok = true;
        for (int i = 1; i <= total_slaves; ++i) {
            if (!all_axes && i != target_slave) continue;
            std::vector<std::uint8_t> bytes;
            const std::string& name = controller.axes()[i - 1].axis_name;
            std::cout << name << " (从站 " << i << ") 0x"
                      << std::hex << std::setw(4) << std::setfill('0') << sdo_index
                      << ":" << std::setw(2) << sdo_subindex << std::dec << ": ";
            if (controller.bus().read_sdo_bytes(i, static_cast<std::uint16_t>(sdo_index),
                                                static_cast<std::uint8_t>(sdo_subindex), bytes)) {
                std::cout << format_sdo_value(bytes) << '\n';
            } else {
                std::cout << "读取失败（对象不存在、不支持 CoE 或超时）\n";
                all_ok = false;
            }
        }
        return all_ok ? 0 : 1;
    }

    if (command == "write-sdo") {
        const int target_slave = resolve_single_axis(target_axis);
        if (target_slave < 0) return 1;
        if (controller.bus().write_sdo_bytes(target_slave, static_cast<std::uint16_t>(sdo_index),
                                             static_cast<std::uint8_t>(sdo_subindex), write_sdo_payload)) {
            std::cout << "[成功] 已写入从站 " << target_slave << " 0x"
                      << std::hex << sdo_index << ":" << sdo_subindex << std::dec << '\n';
            return 0;
        }
        std::cerr << "[失败] SDO 写入失败: " << controller.bus().pop_coe_errors() << '\n';
        return 1;
    }

    if (command == "set-zero") {
        int target_slave = -1;
        if (!all_axes) {
            target_slave = resolve_single_axis(target_axis);
            if (target_slave < 0) return 1;
        }
        bool all_ok = true;
        for (int i = 1; i <= total_slaves; ++i) {
            if (!all_axes && i != target_slave) continue;
            if (!controller.axes()[i - 1].is_servo) continue;
            std::string detail;
            const bool ok = controller.set_zero(i, detail);
            std::cout << "  - [" << controller.axes()[i - 1].axis_name << "] 从站 " << i << ": "
                      << (ok ? "[标定成功] " : "[标定失败] ") << detail << "\n";
            if (!ok) all_ok = false;
        }
        return all_ok ? 0 : 1;
    }

    const int slave = resolve_single_axis(target_axis);
    if (slave < 0) return 1;
    if (command == "zero") return controller.run_zero(slave, jog_ms) ? 0 : 1;
    if (jog_abs_flag) return controller.run_jog_abs(slave, jog_deg, jog_ms) ? 0 : 1;
    return controller.run_jog(slave, jog_deg, jog_ms) ? 0 : 1;
}
