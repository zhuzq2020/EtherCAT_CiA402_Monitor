#include "maint_engine.hpp"
#include "tongchuan_def.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <sstream>

namespace {

void sigint_handler(int) {
    robot_maint::MaintEngine::request_stop();
}

bool parse_object_address(const std::string& text, unsigned max, unsigned& result) {
    if (text.empty() || text[0] == '-' || text[0] == '+') return false;
    try {
        std::size_t end = 0;
        const int base = text.size() > 2 && text[0] == '0' &&
                         (text[1] == 'x' || text[1] == 'X') ? 16 : 10;
        const auto parsed = std::stoul(text, &end, base);
        if (end != text.size() || parsed > max) return false;
        result = static_cast<unsigned>(parsed);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

std::string format_sdo_value(const std::vector<std::uint8_t>& bytes) {
    std::ostringstream out;
    out << "长度 " << bytes.size() << " 字节, 原始字节 (低位在前):";
    for (const auto byte : bytes) {
        out << ' ' << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<unsigned>(byte);
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
        const std::uint64_t unsigned_number = bits == 64
            ? number : number & ((std::uint64_t{1} << bits) - 1);
        out << std::dec << ", 无符号: " << unsigned_number << ", 有符号: " << signed_number;
    }
    return out.str();
}

void show_help() {
    std::cout << "\n======================================================================\n"
              << " 机器人现场维护与标定工具 (arm_maint) - 纯用户态 SOEM 架构\n"
              << " 支持同川 (Tongchuan)、意优 (Eyou) 及 CiA-402 标准伺服电机\n"
              << "======================================================================\n\n"
              << "用法:\n"
              << "  arm_maint [全局选项] <命令> [参数]\n\n"
              << "全局选项:\n"
              << "  -i, --iface <NAME>    指定以太网接口名称 (默认: enp2s0)\n"
              << "  -h, --help            显示本帮助信息\n\n"
              << "命令列表:\n"
              << "  1. status             扫描总线，以表格查看从站拓扑、电机厂商、CiA402状态、故障码及位置\n"
              << "  2. clear-fault        清除伺服驱动器故障报警 (标准 CiA-402 复位)\n"
              << "       --axis <Dxx|1..N>  清除指定单轴故障 (如 D18、1、S1)\n"
              << "       --all              清除总线所有轴故障\n"
              << "  3. jog                [受控 PDO 点动] 安全慢速微调转动指定关节以对齐零位刻线\n"
              << "       --axis <Dxx|1..N>  指定点动目标轴 (必填，单轴操作以保安全)\n"
              << "       --delta_deg <DEG>  微调转动角度 (度，如 1.0 或 -0.5，自动换算减速比)\n"
              << "       --delta_count <N>  微调脉冲数 (绝对计数，如 5000 或 -5000)\n"
              << "       --speed_rpm <RPM>  输出端安全微调转速 (默认: 0.5 RPM，限幅 0.05~2.0)\n"
              << "  4. set-zero           将机械臂当前物理姿态标定为编码器绝对零点 (自动适配同川/意优)\n"
              << "       --axis <Dxx|1..N>  标定指定单轴 (更换单轴电机时使用)\n"
              << "       --all              标定整机/整臂所有轴\n"
              << "       --confirm          强制确认标志 (安全防护，防止误触覆盖原点)\n"
              << "  5. read-pos           快速读取单轴或全臂当前编码器实际脉冲位置\n"
              << "       --axis <Dxx|1..N>  读取指定轴\n"
              << "       --all              读取全部轴\n"
              << "  6. read-sdo           读取指定 CoE 对象，默认读取全部从站\n"
              << "       --index <0xHHHH>   对象索引（必填，如 0x6064）\n"
              << "       --subindex <0..255> 子索引（默认 0，也可用 0x 格式）\n"
              << "       --axis <Dxx|1..N>  读取指定轴；--all 读取全部轴\n"
              << "  7. write-alias        将 Hot Connect 组首 Alias 写入从站 SII EEPROM（支持同川/意优/其他型号）\n"
              << "       --slave <1..N>     按本次扫描序号选择物理从站（不接受 Dxx，防止混淆）\n"
              << "       --alias <1..65535> 要写入的 Alias（右臂组首 D18 为 18）\n"
              << "       --backup <FILE>    按 SII 声明容量保存完整 EEPROM 备份（不得已存在）\n"
              << "       --confirm WRITE-EEPROM-ALIAS  精确确认短语\n\n"
              << "示例:\n"
              << "  sudo ./arm_maint status\n"
              << "  sudo ./arm_maint clear-fault --axis D18\n"
              << "  # 微调 D18 关节转动 +1.0 度以对齐零位刻线:\n"
              << "  sudo ./arm_maint jog --axis D18 --delta_deg 1.0\n"
              << "  # 微调 D20 关节反向转动 -0.5 度:\n"
              << "  sudo ./arm_maint jog --axis D20 --delta_deg -0.5 --speed_rpm 0.3\n"
              << "  # 对齐刻线后执行绝对零点标定 (自动识别同川或意优驱动):\n"
              << "  sudo ./arm_maint set-zero --axis D18 --confirm\n"
              << "  sudo ./arm_maint read-sdo --axis D18 --index 0x6064\n"
              << "  sudo ./arm_maint read-sdo --axis 1 --index 0x1018 --subindex 1\n"
              << "  # 仅为右臂链头（扫描从站 1）写入 D18 Hot Connect Alias:\n"
              << "  sudo ./arm_maint write-alias --slave 1 --alias 18 --backup ./d18-sii.bin --confirm WRITE-EEPROM-ALIAS\n\n";
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        show_help();
        return 1;
    }

    std::signal(SIGINT, sigint_handler);

    std::string iface = "enp2s0";
    std::string command;
    std::string target_axis;
    bool all_axes = false;
    bool confirmed = false;
    std::string confirmation_phrase;
    int alias_slave = -1;
    int alias_value = -1;
    std::string backup_path;
    double delta_deg = 1.0;
    bool has_delta_deg = false;
    int32_t delta_count = 0;
    bool has_delta_count = false;
    double speed_rpm = 0.5;
    unsigned sdo_index = 0;
    unsigned sdo_subindex = 0;
    bool has_sdo_index = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            show_help();
            return 0;
        } else if (arg == "-i" || arg == "--iface") {
            if (++i < argc) iface = argv[i];
        } else if (arg == "--axis") {
            if (++i < argc) target_axis = argv[i];
        } else if (arg == "--index" || arg == "--subindex") {
            const bool is_index = arg == "--index";
            unsigned parsed = 0;
            if (++i >= argc || !parse_object_address(argv[i], is_index ? 0xffffU : 0xffU, parsed)) {
                std::cerr << "[错误] " << arg << " 必须是有效的十进制或 0x 十六进制数。\n";
                return 1;
            }
            if (is_index) { sdo_index = parsed; has_sdo_index = true; }
            else sdo_subindex = parsed;
        } else if (arg == "--all") {
            all_axes = true;
        } else if (arg == "--confirm") {
            confirmed = true;
            if (i + 1 < argc && std::string(argv[i + 1]) == "WRITE-EEPROM-ALIAS") {
                confirmation_phrase = argv[++i];
            }
        } else if (arg == "--slave") {
            if (++i < argc) alias_slave = std::stoi(argv[i]);
        } else if (arg == "--alias") {
            if (++i < argc) alias_value = std::stoi(argv[i]);
        } else if (arg == "--backup") {
            if (++i < argc) backup_path = argv[i];
        } else if (arg == "--delta_deg") {
            if (++i < argc) { delta_deg = std::stod(argv[i]); has_delta_deg = true; }
        } else if (arg == "--delta_count") {
            if (++i < argc) { delta_count = std::stol(argv[i]); has_delta_count = true; }
        } else if (arg == "--speed_rpm") {
            if (++i < argc) { speed_rpm = std::stod(argv[i]); }
        } else if (command.empty() && arg[0] != '-') {
            command = arg;
        }
    }

    if (command.empty()) {
        std::cerr << "[错误] 未指定有效命令，请参考 --help。\n";
        return 1;
    }

    if (command != "status" && command != "clear-fault" && command != "set-zero" && command != "read-pos" && command != "read-sdo" && command != "jog" && command != "write-alias") {
        std::cerr << "[错误] 未知命令: " << command << "，输入 --help 查看支持命令。\n";
        return 1;
    }

    // 参数校验预检
    if (command == "clear-fault" && !all_axes && target_axis.empty()) {
        std::cerr << "[错误] 请指定待清错目标: --axis <D18..D24> 或 --all\n";
        return 1;
    }

    if (command == "read-sdo" && !has_sdo_index) {
        std::cerr << "[错误] read-sdo 必须指定 --index，例如 --index 0x6064。\n";
        return 1;
    }

    if (command == "set-zero") {
        if (!confirmed) {
            std::cerr << "\n[安全拦截] 标定零点会将当前机械位置永久写入电机绝对值编码器 Flash！\n"
                      << "           若您确认机械臂已处于标定原点工装位置，请追加参数: --confirm\n\n";
            return 1;
        }
        if (!all_axes && target_axis.empty()) {
            std::cerr << "[错误] 请指定标零目标: --axis <D18..D24> 或 --all\n";
            return 1;
        }
    }

    if (command == "jog") {
        if (target_axis.empty()) {
            std::cerr << "[错误] 点动命令必须显式指定单个目标轴: --axis <D18..D24> (安全起见禁止全轴同时点动)\n";
            return 1;
        }
        if (!has_delta_deg && !has_delta_count) {
            has_delta_deg = true;
            delta_deg = 1.0; // 默认微调 1 度
        }
    }

    if (command == "write-alias") {
        if (alias_slave <= 0 || alias_value <= 0 || alias_value > 65535 || backup_path.empty()) {
            std::cerr << "[错误] write-alias 必须指定 --slave、--alias 和 --backup。\n";
            return 1;
        }
        if (confirmation_phrase != "WRITE-EEPROM-ALIAS") {
            std::cerr << "\n[安全拦截] 写 Alias 会永久修改电机 SII EEPROM。\n"
                      << "           请核对物理拓扑并追加: --confirm WRITE-EEPROM-ALIAS\n\n";
            return 1;
        }
    }

    robot_maint::MaintEngine engine(iface);
    std::cout << "[INFO] 正在初始化以太网口: " << iface << " ...\n";
    if (!engine.init()) {
        return 1;
    }

    const int total_slaves = engine.slave_count();

    // 1. STATUS
    if (command == "status") {
        const auto axes = engine.scan_all();
        std::cout << "\n============================== EtherCAT 总线扫描结果 (共 " << axes.size() << " 个从站) ==============================\n";
        std::cout << std::left
                  << std::setw(6)  << "轴号"
                  << std::setw(8)  << "从站ID"
                  << std::setw(8)  << "Alias"
                  << std::setw(20) << "电机厂商 (Vendor)"
                  << std::setw(10) << "总线状态"
                  << std::setw(24) << "CiA-402状态"
                  << std::setw(10) << "故障代码"
                  << std::setw(16) << "当前脉冲位置"
                  << "故障描述\n";
        std::cout << "------------------------------------------------------------------------------------------------------------\n";

        for (const auto& a : axes) {
            std::string code_str = "0x" + ([&]{
                std::ostringstream ss;
                ss << std::hex << std::setw(4) << std::setfill('0') << a.error_code;
                return ss.str();
            })();

            std::cout << std::left
                      << std::setw(6)  << a.axis_name
                      << std::setw(8)  << a.slave_index
                      << std::setw(8)  << a.station_alias
                      << std::setw(20) << a.vendor_name
                      << std::setw(10) << a.ethercat_state
                      << std::setw(24) << a.cia402_state
                      << std::setw(10) << code_str
                      << std::setw(16) << a.actual_position
                      << a.error_desc << "\n";
        }
        std::cout << "============================================================================================================\n\n";
        return 0;
    }

    // 2. WRITE-ALIAS（按扫描序号选设备，避免把 D 逻辑名误当成硬件身份）
    if (command == "write-alias") {
        if (alias_slave > total_slaves) {
            std::cerr << "[错误] --slave 超出本次扫描范围 (1.." << total_slaves << ")\n";
            return 1;
        }
        std::cout << "[WARN] 即将为物理从站 " << alias_slave << " 写入 EEPROM Alias "
                  << alias_value << "，写入期间请勿断电。\n";
        std::string detail;
        const bool ok = engine.write_station_alias(
            alias_slave, static_cast<std::uint16_t>(alias_value), backup_path, detail);
        std::cout << (ok ? "[成功] " : "[失败] ") << detail << '\n';
        return ok ? 0 : 1;
    }

    // 解析目标轴索引 (如用户未指定 --all)
    int target_slave = -1;
    if (!all_axes && !target_axis.empty()) {
        target_slave = robot_maint::axis_name_to_slave(target_axis, total_slaves);
        if (target_slave <= 0 || target_slave > total_slaves) {
            std::cerr << "[错误] 无法识别指定轴或从站超出总线范围: " << target_axis
                      << " (当前总线从站总数: " << total_slaves << ")\n";
            return 1;
        }
    }

    // 2. CLEAR-FAULT
    if (command == "clear-fault") {
        std::cout << "[INFO] 开始执行清错操作...\n";
        bool all_success = true;

        for (int i = 1; i <= total_slaves; ++i) {
            const std::string name = robot_maint::slave_to_axis_name(i, total_slaves);
            if (all_axes || i == target_slave) {
                std::string detail;
                const bool ok = engine.clear_fault(i, detail);
                std::cout << "  - [" << name << "] 从站 " << i << ": " << (ok ? "[成功] " : "[失败] ") << detail << "\n";
                if (!ok) all_success = false;
            }
        }
        return all_success ? 0 : 1;
    }

    // 3. SET-ZERO
    if (command == "set-zero") {
        std::cout << "[WARN] 正在执行零点标定写入...\n";
        bool all_success = true;

        for (int i = 1; i <= total_slaves; ++i) {
            const std::string name = robot_maint::slave_to_axis_name(i, total_slaves);
            if (all_axes || i == target_slave) {
                std::string detail;
                const bool ok = engine.set_zero(i, detail);
                std::cout << "  - [" << name << "] 从站 " << i << ": " << (ok ? "[标定成功] " : "[标定失败] ") << detail << "\n";
                if (!ok) all_success = false;
            }
        }
        return all_success ? 0 : 1;
    }

    // 4. READ-POS
    if (command == "read-pos") {
        if (!all_axes && target_axis.empty()) all_axes = true; // 默认读全轴

        std::cout << "\n--- 当前各轴绝对位置脉冲 ---\n";
        for (int i = 1; i <= total_slaves; ++i) {
            const std::string name = robot_maint::slave_to_axis_name(i, total_slaves);
            if (all_axes || i == target_slave) {
                int32_t pos = 0;
                if (engine.read_position(i, pos)) {
                    std::cout << "  " << name << " (从站 " << i << "): " << pos << " counts\n";
                } else {
                    std::cout << "  " << name << " (从站 " << i << "): 读取失败\n";
                }
            }
        }
        std::cout << "---------------------------\n\n";
        return 0;
    }

    if (command == "read-sdo") {
        if (!all_axes && target_axis.empty()) all_axes = true;
        bool all_success = true;
        for (int i = 1; i <= total_slaves; ++i) {
            if (!all_axes && i != target_slave) continue;
            std::vector<std::uint8_t> value;
            const std::string name = robot_maint::slave_to_axis_name(i, total_slaves);
            std::cout << name << " (从站 " << i << ") 0x"
                      << std::hex << std::setw(4) << std::setfill('0') << sdo_index
                      << ":" << std::setw(2) << sdo_subindex << std::dec << ": ";
            if (engine.read_sdo_bytes(i, static_cast<std::uint16_t>(sdo_index),
                                      static_cast<std::uint8_t>(sdo_subindex), value)) {
                std::cout << format_sdo_value(value) << '\n';
            } else {
                std::cout << "读取失败（对象不存在、从站不支持 CoE、响应超时或数据超过 256 字节）\n";
                all_success = false;
            }
        }
        return all_success ? 0 : 1;
    }

    // 5. JOG
    if (command == "jog") {
        if (target_slave <= 0 || target_slave > total_slaves) {
            std::cerr << "[错误] 无效的轴名称或从站超出范围: " << target_axis
                      << " (当前总线从站数: " << total_slaves << ")\n";
            return 1;
        }

        std::string name = robot_maint::slave_to_axis_name(target_slave, total_slaves);
        std::cout << "[INFO] 准备执行点动微调: " << name << " (从站 " << target_slave << ") ...\n";
        std::string detail;
        bool ok = false;
        if (has_delta_count) {
            ok = engine.jog_axis_counts(target_slave, delta_count, speed_rpm, detail);
        } else {
            ok = engine.jog_axis_deg(target_slave, delta_deg, speed_rpm, detail);
        }

        std::cout << (ok ? "  [JOG 成功] " : "  [JOG 失败] ") << detail << "\n\n";
        return ok ? 0 : 1;
    }

    std::cerr << "[错误] 未知命令: " << command << "，输入 --help 查看支持命令。\n";
    return 1;
}
