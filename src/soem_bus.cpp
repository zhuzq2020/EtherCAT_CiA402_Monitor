#include "soem_bus.hpp"
#include "csp_mode.hpp"
#include "drive_profile.hpp"
#include "motion_pdo_group.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <net/if.h>
#include <pthread.h>
#include <sched.h>
#include <sstream>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace robot_maint {

namespace {

bool valid_iface_name(const std::string& iface) {
    if (iface.empty() || iface.size() > 15) return false;
    for (unsigned char c : iface) {
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.' || c == ':')) {
            return false;
        }
    }
    return true;
}

std::uint32_t load_le(const std::uint8_t* p, unsigned bytes) {
    std::uint32_t val = 0;
    for (unsigned i = 0; i < bytes; ++i) {
        val |= static_cast<std::uint32_t>(p[i]) << (8 * i);
    }
    return val;
}

void store_le(std::uint8_t* p, std::uint32_t val, unsigned bytes) {
    for (unsigned i = 0; i < bytes; ++i) {
        p[i] = static_cast<std::uint8_t>(val >> (8 * i));
    }
}

} // namespace

InterfaceLock::~InterfaceLock() {
    release();
}

void InterfaceLock::release() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool InterfaceLock::acquire(const std::string& iface, std::string& error) {
    release();
    if (!valid_iface_name(iface)) {
        error = "无效网卡名称: " + iface;
        return false;
    }
    const unsigned index = ::if_nametoindex(iface.c_str());
    if (index == 0) {
        error = "网卡不存在: " + iface;
        return false;
    }
    const auto path = "/tmp/arm_maint-ifindex-" + std::to_string(index) + ".lock";
    fd_ = ::open(path.c_str(), O_CREAT | O_RDONLY | O_CLOEXEC | O_NOFOLLOW, 0644);
    struct stat st{};
    if (fd_ < 0 || ::fstat(fd_, &st) || !S_ISREG(st.st_mode) || ::flock(fd_, LOCK_EX | LOCK_NB)) {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        error = "网口 " + iface + " 正被其他进程占用";
        return false;
    }
    return true;
}

SoemBus::SoemBus(std::string iface) : iface_(std::move(iface)) {}

SoemBus::~SoemBus() {
    close();
}

std::string SoemBus::al_state_str(std::uint16_t state) {
    switch (state & 0x000FU) {
        case EC_STATE_NONE:        return "NONE";
        case EC_STATE_INIT:        return "INIT";
        case EC_STATE_PRE_OP:      return "PRE-OP";
        case EC_STATE_BOOT:        return "BOOT";
        case EC_STATE_SAFE_OP:     return "SAFE-OP";
        case EC_STATE_OPERATIONAL: return "OP";
        default: {
            std::ostringstream ss;
            ss << "0x" << std::hex << state;
            return ss.str();
        }
    }
}

bool SoemBus::init(bool acquire_lock) {
    close();
    std::memset(&context_, 0, sizeof(context_));

    if (acquire_lock) {
        std::string lock_err;
        if (!lock_.acquire(iface_, lock_err)) {
            std::cerr << "[错误] " << lock_err << '\n';
            return false;
        }
    }

    if (ecx_init(&context_, iface_.c_str()) <= 0) {
        std::cerr << "[错误] 无法初始化网络接口: " << iface_ << '\n'
                  << "       请检查网卡名称是否正确，以及是否具备 root 或 CAP_NET_RAW/CAP_NET_ADMIN 权限。\n";
        lock_.release();
        return false;
    }
    initialized_ = true;

    slave_count_ = ecx_config_init(&context_);
    if (slave_count_ <= 0) {
        std::cerr << "[提示] 网口 " << iface_ << " 上未扫描到任何 EtherCAT 从站。\n"
                  << "       请检查驱动器供电及网线物理连接。\n";
        close();
        return false;
    }

    if (slave_count_ > 64) {
        std::cerr << "[错误] 扫描到 " << slave_count_ << " 个从站，超过 SDK 最大支持数 (64)。\n";
        close();
        return false;
    }

    ecx_readstate(&context_);
    return true;
}

void SoemBus::close() {
    if (initialized_) {
        if (operational_ || pdo_running_.load()) {
            shutdown_operational();
        }
        context_.slavelist[0].state = EC_STATE_PRE_OP;
        ecx_writestate(&context_, 0);
        ecx_statecheck(&context_, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
        ecx_close(&context_);
        initialized_ = false;
        operational_ = false;
        slave_count_ = 0;
        expected_wkc_ = 0;
        active_mask_.clear();
        fixed_csp_mask_.clear();
        sdo_modes_.clear();
        layouts_.clear();
    }
    lock_.release();
}

void SoemBus::refresh_states() {
    if (initialized_) {
        ecx_readstate(&context_);
    }
}

std::uint32_t SoemBus::slave_vendor_id(int slave) {
    if (!initialized_ || slave < 1 || slave > slave_count_) return 0;
    std::uint32_t vendor = context_.slavelist[slave].eep_man;
    if (vendor == 0 && (context_.slavelist[slave].mbx_proto & ECT_MBXPROT_COE) != 0) {
        read_sdo(slave, SDO_IDENTITY_OBJECT, 1, vendor);
    }
    return vendor;
}

std::uint32_t SoemBus::slave_product_code(int slave) {
    if (!initialized_ || slave < 1 || slave > slave_count_) return 0;
    std::uint32_t product = context_.slavelist[slave].eep_id;
    if (product == 0 && (context_.slavelist[slave].mbx_proto & ECT_MBXPROT_COE) != 0) {
        read_sdo(slave, SDO_IDENTITY_OBJECT, 2, product);
    }
    return product;
}

bool SoemBus::read_sdo_bytes(int slave, std::uint16_t index, std::uint8_t subindex,
                             std::vector<std::uint8_t>& out, int timeout_us) {
    out.clear();
    if (!initialized_ || slave < 1 || slave > slave_count_ ||
        (context_.slavelist[slave].mbx_proto & ECT_MBXPROT_COE) == 0) {
        return false;
    }
    std::array<std::uint8_t, 256> buffer{};
    int size = static_cast<int>(buffer.size());
    const int rc = ecx_SDOread(&context_, slave, index, subindex, FALSE, &size, buffer.data(), timeout_us);
    if (rc <= 0 || size <= 0 || size > static_cast<int>(buffer.size())) {
        return false;
    }
    out.assign(buffer.begin(), buffer.begin() + size);
    return true;
}

bool SoemBus::write_sdo_bytes(int slave, std::uint16_t index, std::uint8_t subindex,
                              const std::vector<std::uint8_t>& data,
                              bool complete_access, int timeout_us) {
    if (!initialized_ || slave < 1 || slave > slave_count_ ||
        (context_.slavelist[slave].mbx_proto & ECT_MBXPROT_COE) == 0 || data.empty()) {
        return false;
    }
    std::vector<std::uint8_t> copy = data;
    const int rc = ecx_SDOwrite(&context_, slave, index, subindex,
                                complete_access ? TRUE : FALSE,
                                static_cast<int>(copy.size()), copy.data(), timeout_us);
    return rc > 0;
}

bool SoemBus::read_sdo_string(int slave, std::uint16_t index, std::uint8_t subindex, std::string& out) {
    out.clear();
    std::vector<std::uint8_t> bytes;
    if (!read_sdo_bytes(slave, index, subindex, bytes)) return false;
    std::size_t len = bytes.size();
    while (len > 0 && (bytes[len - 1] == '\0' || bytes[len - 1] == ' ' ||
                       bytes[len - 1] == '\r' || bytes[len - 1] == '\n')) {
        --len;
    }
    out.assign(reinterpret_cast<const char*>(bytes.data()), len);
    return true;
}

std::string SoemBus::pop_coe_errors() {
    std::ostringstream out;
    while (ecx_iserror(&context_)) {
        if (out.tellp() > 0) out << " | ";
        out << ecx_elist2string(&context_);
    }
    return out.str();
}

int SoemBus::read_esc_reg(int slave, std::uint16_t reg, void* data, std::size_t size) {
    if (!initialized_ || slave < 1 || slave > slave_count_) return 0;
    return ecx_FPRD(&context_.port, context_.slavelist[slave].configadr,
                    reg, static_cast<int>(size), data, EC_TIMEOUTRET);
}

int SoemBus::write_esc_reg(int slave, std::uint16_t reg, const void* data, std::size_t size) {
    if (!initialized_ || slave < 1 || slave > slave_count_) return 0;
    return ecx_FPWR(&context_.port, context_.slavelist[slave].configadr,
                    reg, static_cast<int>(size), const_cast<void*>(data), EC_TIMEOUTRET);
}

bool SoemBus::discover_side_mapping(int slave, std::uint16_t assignment, bool output, PdoAxisLayout& layout) {
    std::uint8_t count = 0;
    if (!read_sdo(slave, assignment, 0, count) || count == 0 || count > 16) {
        return false;
    }
    unsigned bits = 0;
    for (unsigned p = 1; p <= count; ++p) {
        std::uint16_t index = 0;
        std::uint8_t entries = 0;
        if (!read_sdo(slave, assignment, static_cast<std::uint8_t>(p), index) ||
            !read_sdo(slave, index, 0, entries) || entries > 64) {
            return false;
        }
        for (unsigned e = 1; e <= entries; ++e) {
            std::uint32_t value = 0;
            if (!read_sdo(slave, index, static_cast<std::uint8_t>(e), value)) {
                return false;
            }
            const auto obj = static_cast<std::uint16_t>(value >> 16);
            const unsigned sub = (value >> 8) & 0xFFU;
            const unsigned width = value & 0xFFU;
            if (width == 0) return false;

            int* field = nullptr;
            unsigned expected_bits = 0;
            if (sub == 0) {
                if (output) {
                    if (obj == SDO_CONTROL_WORD)       { field = &layout.cw;         expected_bits = 16; }
                    else if (obj == SDO_TARGET_POSITION)  { field = &layout.target_pos; expected_bits = 32; }
                    else if (obj == SDO_MODE_OF_OPERATION){ field = &layout.mode_op;    expected_bits = 8; }
                } else {
                    if (obj == SDO_STATUS_WORD)        { field = &layout.sw;            expected_bits = 16; }
                    else if (obj == SDO_ACTUAL_POSITION)  { field = &layout.actual_pos;    expected_bits = 32; }
                    else if (obj == SDO_MODE_DISPLAY)     { field = &layout.mode_disp;     expected_bits = 8; }
                    else if (obj == SDO_ACTUAL_VELOCITY)  { field = &layout.actual_vel;    expected_bits = 32; }
                    else if (obj == SDO_ACTUAL_TORQUE)    { field = &layout.actual_torque; expected_bits = 16; }
                    else if (obj == SDO_ERROR_CODE)       { field = &layout.error_code;    expected_bits = 16; }
                }
            }
            if (field) {
                if ((bits % 8) != 0 || width != expected_bits || *field >= 0) {
                    return false;
                }
                *field = static_cast<int>(bits / 8);
            }
            bits += width;
        }
    }
    if (output) layout.rx_bits = bits;
    else layout.tx_bits = bits;
    return true;
}

bool SoemBus::discover_pdo_layout(int slave, PdoAxisLayout& layout) {
    layout = PdoAxisLayout{};
    if (!discover_side_mapping(slave, 0x1C12, true, layout)) return false;
    if (!discover_side_mapping(slave, 0x1C13, false, layout)) return false;
    return layout.valid();
}

bool SoemBus::enter_operational(const std::vector<int>& active_slaves,
                                const std::vector<bool>& fixed_csp_flags,
                                std::string& error) {
    error.clear();
    if (!initialized_ || slave_count_ <= 0) {
        error = "总线尚未初始化";
        return false;
    }
    if (operational_) {
        shutdown_operational();
    }

    // 1. Ensure clean PRE-OP state and acknowledge any residual AL error
    ecx_readstate(&context_);
    bool need_preop_reset = false;
    for (int i = 1; i <= slave_count_; ++i) {
        if ((context_.slavelist[i].state & 0x1FU) != EC_STATE_PRE_OP) {
            need_preop_reset = true;
            break;
        }
    }
    if (need_preop_reset) {
        context_.slavelist[0].state = EC_STATE_PRE_OP | EC_STATE_ACK;
        ecx_writestate(&context_, 0);
        ecx_statecheck(&context_, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    }

    // 2. Build active mask & discover PDO layouts + request CSP in PRE-OP
    active_mask_.assign(slave_count_ + 1, false);
    fixed_csp_mask_.assign(slave_count_ + 1, false);
    sdo_modes_.assign(slave_count_ + 1, 0);
    layouts_.assign(slave_count_ + 1, PdoAxisLayout{});

    for (int s : active_slaves) {
        if (s < 1 || s > slave_count_) {
            error = "指定从站编号越界: " + std::to_string(s);
            return false;
        }
        active_mask_[s] = true;
    }
    if (fixed_csp_flags.size() == static_cast<std::size_t>(slave_count_ + 1)) {
        fixed_csp_mask_ = fixed_csp_flags;
    }

    std::vector<MotionAxisInfo> group_info(slave_count_);
    bool any_active = false;
    for (int i = 1; i <= slave_count_; ++i) {
        group_info[i - 1].name = slave_to_axis_name(i, slave_count_, fixed_csp_mask_[i]);
        if (!active_mask_[i]) {
            group_info[i - 1].unavailable = "未包含在本次运动组";
            continue;
        }
        any_active = true;
        if (!discover_pdo_layout(i, layouts_[i])) {
            error = "从站 " + std::to_string(i) + " (" + group_info[i - 1].name + ") 无法核验 PDO 核心字段映射 (0x6040/0x607A/0x6041/0x6064)";
            return false;
        }
        if (fixed_csp_mask_[i]) {
            sdo_modes_[i] = 8;
        } else {
            std::string mode_err;
            if (!request_csp_mode([&](std::int8_t mode) { return write_sdo(i, SDO_MODE_OF_OPERATION, 0, mode); }, mode_err)) {
                error = "从站 " + std::to_string(i) + " (" + group_info[i - 1].name + ") " + mode_err;
                const auto coe_err = pop_coe_errors();
                if (!coe_err.empty()) error += " [" + coe_err + "]";
                return false;
            }
        }
    }
    if (!any_active) {
        error = "没有可控制的伺服轴";
        return false;
    }

    // 3. Configure motion group 1, PO2SOconfig for non-fixed-CSP slaves, and map IO
    iomap_.assign(static_cast<std::size_t>(slave_count_) * 256U + 4096U, 0);
    configure_motion_group(context_, group_info);

    for (int i = 1; i <= slave_count_; ++i) {
        if (!active_mask_[i]) continue;
        context_.slavelist[i].PO2SOconfig = [](ecx_contextt* ctx, std::uint16_t slave) -> int {
            if (ctx->slavelist[slave].eep_man != kFairinoVendorId) {
                std::int8_t mode_probe = 0;
                int sz = sizeof(mode_probe);
                if (ecx_SDOread(ctx, slave, SDO_MODE_OF_OPERATION, 0, FALSE, &sz, &mode_probe, EC_TIMEOUTRXM) > 0) {
                    std::int8_t mode = 8;
                    ecx_SDOwrite(ctx, slave, SDO_MODE_OF_OPERATION, 0, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM);
                }
            }
            return 1;
        };
    }

    const int mapped_bytes = ecx_config_map_group(&context_, iomap_.data(), kMotionPdoGroup);
    if (mapped_bytes <= 0 || static_cast<std::size_t>(mapped_bytes) > iomap_.size()) {
        error = "SOEM ecx_config_map_group 映射失败 (返回值: " + std::to_string(mapped_bytes) + ")";
        return false;
    }

    for (int i = 1; i <= slave_count_; ++i) {
        if (!active_mask_[i]) continue;
        const auto& s = context_.slavelist[i];
        const auto& l = layouts_[i];
        if (!s.outputs || !s.inputs || s.Obytes * 8U < l.rx_bits || s.Ibytes * 8U < l.tx_bits) {
            error = "从站 " + std::to_string(i) + " PDO 缓冲区长度不匹配 (Obytes=" +
                    std::to_string(s.Obytes) + ", Ibytes=" + std::to_string(s.Ibytes) + ")";
            return false;
        }
    }

    expected_wkc_ = context_.grouplist[kMotionPdoGroup].outputsWKC * 2 +
                    context_.grouplist[kMotionPdoGroup].inputsWKC;
    if (expected_wkc_ <= 0) {
        error = "运动组期望 WKC 无效 (" + std::to_string(expected_wkc_) + ")";
        return false;
    }

    // 4. Wait for SAFE-OP on motion group
    if (!motion_group_matches(context_, EC_STATE_SAFE_OP, [&](int slave, std::uint16_t state) {
            return ecx_statecheck(&context_, slave, state, EC_TIMEOUTSTATE * 4);
        })) {
        ecx_readstate(&context_);
        std::ostringstream oss;
        oss << "运动 PDO 组无法进入 SAFE-OP";
        for (int i = 1; i <= slave_count_; ++i) {
            if (active_mask_[i] && context_.slavelist[i].state != EC_STATE_SAFE_OP) {
                oss << "；S" << i << " state=0x" << std::hex << context_.slavelist[i].state
                    << " AL=0x" << context_.slavelist[i].ALstatuscode << std::dec
                    << " (" << ec_ALstatuscode2string(context_.slavelist[i].ALstatuscode) << ")";
            }
        }
        error = oss.str();
        return false;
    }

    // 5. Prefill RxPDO with CW_SHUTDOWN + actual_position, start 2ms cyclic worker
    desired_cmds_.assign(slave_count_ + 1, PdoAxisCmd{});
    latest_fb_.assign(slave_count_ + 1, PdoAxisFeedback{});

    for (int i = 1; i <= slave_count_; ++i) {
        if (!active_mask_[i]) continue;
        auto* out_ptr = context_.slavelist[i].outputs;
        if (out_ptr && context_.slavelist[i].Obytes > 0) {
            std::memset(out_ptr, 0, context_.slavelist[i].Obytes);
        }
        std::int32_t cur_pos = 0;
        if (!read_sdo(i, SDO_ACTUAL_POSITION, 0, cur_pos)) {
            error = "从站 " + std::to_string(i) + " 读取初始实际位置 (0x6064) 失败";
            return false;
        }
        desired_cmds_[i] = {CW_SHUTDOWN, cur_pos, true};
        latest_fb_[i].actual_position = cur_pos;
    }

    start_pdo_thread();
    for (int w = 0; w < 100 && !has_valid_pdo_.load(); ++w) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // 6. Transition motion group to OP while 2ms PDO thread continuously feeds frames
    request_motion_group_state(context_, EC_STATE_OPERATIONAL, [&](int slave) {
        ecx_writestate(&context_, slave);
    });

    bool op_reached = false;
    for (int n = 0; n < 1000; ++n) {
        const bool all_op = motion_group_matches(context_, EC_STATE_OPERATIONAL, [&](int slave, std::uint16_t state) {
            return ecx_statecheck(&context_, slave, state, 1000);
        });
        if (all_op && has_valid_pdo_.load() && last_wkc_.load() >= expected_wkc_) {
            op_reached = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    if (!op_reached) {
        stop_pdo_thread();
        ecx_readstate(&context_);
        std::ostringstream oss;
        oss << "从站未能全部进入 OP 状态 (WKC=" << last_wkc_.load() << "/" << expected_wkc_ << ")";
        for (int i = 1; i <= slave_count_; ++i) {
            if (active_mask_[i]) {
                oss << "；S" << i << " state=0x" << std::hex << context_.slavelist[i].state
                    << " AL=0x" << context_.slavelist[i].ALstatuscode << std::dec
                    << " (" << ec_ALstatuscode2string(context_.slavelist[i].ALstatuscode) << ")";
            }
        }
        error = oss.str();
        return false;
    }

    operational_ = true;

    // 7. Confirm CSP mode for drives that require it (SDO 0x6061 if not in TxPDO, or via TxPDO)
    for (int i = 1; i <= slave_count_; ++i) {
        if (!active_mask_[i] || fixed_csp_mask_[i] || layouts_[i].mode_disp >= 0) continue;
        std::string mode_err;
        const bool confirmed = confirm_csp_mode(
            [&](std::int8_t& display) { return read_sdo(i, SDO_MODE_DISPLAY, 0, display); },
            [&]() {
                for (int n = 0; n < 25; ++n) {
                    if (!pdo_healthy_.load()) return false;
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
                return pdo_healthy_.load();
            },
            mode_err);
        if (!confirmed) {
            error = "从站 " + std::to_string(i) + " " + mode_err;
            const auto coe_err = pop_coe_errors();
            if (!coe_err.empty()) error += " [" + coe_err + "]";
            shutdown_operational();
            return false;
        }
        sdo_modes_[i] = 8;
    }

    // Wait briefly for PDO mode display to settle if mapped in TxPDO
    for (int n = 0; n < 250; ++n) {
        std::vector<PdoAxisFeedback> fb;
        BusDiagnostics diag;
        if (snapshot_feedback(fb, diag)) {
            if (!diag.healthy) {
                error = "确认 CSP 模式期间 PDO 通信异常";
                shutdown_operational();
                return false;
            }
            bool all_csp = true;
            for (int i = 1; i <= slave_count_; ++i) {
                if (active_mask_[i] && !fixed_csp_mask_[i] && fb[i].mode_display != 8) {
                    all_csp = false;
                    break;
                }
            }
            if (all_csp) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // 8. Auto-clear any pre-existing fault bit across active axes
    std::vector<PdoAxisFeedback> fb_snap;
    BusDiagnostics diag_snap;
    snapshot_feedback(fb_snap, diag_snap);

    bool any_fault = false;
    for (int i = 1; i <= slave_count_; ++i) {
        if (active_mask_[i] && (fb_snap[i].status_word & SW_FAULT_BIT)) {
            any_fault = true;
            std::cout << "[SOEM] 检测到从站 " << i << " 存在历史故障 (SW=0x"
                      << std::hex << fb_snap[i].status_word
                      << ", Err=0x" << fb_snap[i].error_code << std::dec << ")，正在自动复位 ...\n";
        }
    }
    if (any_fault) {
        std::vector<PdoAxisCmd> cmds(slave_count_ + 1);
        for (int i = 1; i <= slave_count_; ++i) cmds[i] = {0x0000U, 0, true};
        set_all_cmds(cmds);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));

        for (int i = 1; i <= slave_count_; ++i) cmds[i] = {CW_FAULT_RESET, 0, true};
        set_all_cmds(cmds);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        for (int i = 1; i <= slave_count_; ++i) cmds[i] = {CW_SHUTDOWN, 0, true};
        set_all_cmds(cmds);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return true;
}

void SoemBus::shutdown_operational() {
    if (!operational_ && !pdo_running_.load()) return;

    if (pdo_running_.load()) {
        std::vector<PdoAxisCmd> cmds(slave_count_ + 1);
        for (int i = 1; i <= slave_count_; ++i) {
            cmds[i] = {CW_SHUTDOWN, 0, true};
        }
        set_all_cmds(cmds);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        stop_pdo_thread();
    } else if (initialized_) {
        std::vector<PdoAxisCmd> cmds(slave_count_ + 1);
        std::vector<PdoAxisFeedback> fb(slave_count_ + 1);
        for (int i = 1; i <= slave_count_; ++i) {
            cmds[i] = {CW_SHUTDOWN, 0, true};
        }
        for (int n = 0; n < 50; ++n) {
            execute_one_cycle(cmds, fb);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    if (initialized_) {
        request_motion_group_state(context_, EC_STATE_PRE_OP, [&](int slave) {
            ecx_writestate(&context_, slave);
        });
        motion_group_matches(context_, EC_STATE_PRE_OP, [&](int slave, std::uint16_t state) {
            return ecx_statecheck(&context_, slave, state, EC_TIMEOUTSTATE);
        });
    }
    operational_ = false;
}

void SoemBus::start_pdo_thread() {
    if (pdo_running_.load()) return;
    pdo_healthy_.store(true);
    has_valid_pdo_.store(false);
    last_wkc_.store(0);
    total_cycles_.store(0);
    retried_cycles_.store(0);
    max_cycle_us_.store(0);
    max_late_us_.store(0);
    pdo_running_.store(true);
    pdo_thread_ = std::thread(&SoemBus::pdo_worker_loop, this);
}

void SoemBus::stop_pdo_thread() {
    pdo_running_.store(false);
    if (pdo_thread_.joinable()) {
        pdo_thread_.join();
    }
}

void SoemBus::set_axis_cmd(int slave, std::uint16_t cw, std::int32_t target_pos, bool hold_actual) {
    if (slave < 1 || slave > slave_count_) return;
    std::lock_guard<std::mutex> lock(pdo_mutex_);
    if (slave < static_cast<int>(desired_cmds_.size())) {
        desired_cmds_[slave].control_word = cw;
        desired_cmds_[slave].target_position = target_pos;
        desired_cmds_[slave].hold_actual = hold_actual;
    }
}

void SoemBus::set_all_cmds(const std::vector<PdoAxisCmd>& cmds) {
    std::lock_guard<std::mutex> lock(pdo_mutex_);
    for (int i = 1; i <= slave_count_ &&
                    i < static_cast<int>(cmds.size()) &&
                    i < static_cast<int>(desired_cmds_.size()); ++i) {
        desired_cmds_[i] = cmds[i];
    }
}

bool SoemBus::snapshot_feedback(std::vector<PdoAxisFeedback>& out_fb, BusDiagnostics& out_diag) {
    std::lock_guard<std::mutex> lock(pdo_mutex_);
    out_fb = latest_fb_;
    out_diag.expected_wkc = expected_wkc_;
    out_diag.last_wkc = last_wkc_.load();
    out_diag.total_cycles = total_cycles_.load();
    out_diag.retried_cycles = retried_cycles_.load();
    out_diag.max_cycle_us = max_cycle_us_.load();
    out_diag.max_late_us = max_late_us_.load();
    out_diag.healthy = pdo_healthy_.load();
    out_diag.last_exchange = last_exchange_;
    return has_valid_pdo_.load();
}

BusDiagnostics SoemBus::diagnostics() const {
    BusDiagnostics d;
    d.expected_wkc = expected_wkc_;
    d.last_wkc = last_wkc_.load();
    d.total_cycles = total_cycles_.load();
    d.retried_cycles = retried_cycles_.load();
    d.max_cycle_us = max_cycle_us_.load();
    d.max_late_us = max_late_us_.load();
    d.healthy = pdo_healthy_.load();
    return d;
}

void SoemBus::report_diagnostics(const std::string& context_msg) {
    stop_pdo_thread();
    ecx_readstate(&context_);
    std::cerr << "\n[总线异常] " << context_msg
              << " | WKC=" << last_wkc_.load() << "/" << expected_wkc_
              << " | 总周期=" << total_cycles_.load()
              << " | 重试=" << retried_cycles_.load()
              << " | 最大交换=" << max_cycle_us_.load() << "us"
              << " | 最大抖动=" << max_late_us_.load() << "us\n";
    for (int i = 1; i <= slave_count_; ++i) {
        if (!active_mask_.empty() && !active_mask_[i]) continue;
        std::cerr << "  从站 " << i
                  << " 状态=0x" << std::hex << context_.slavelist[i].state
                  << " AL=0x" << context_.slavelist[i].ALstatuscode << std::dec
                  << " (" << ec_ALstatuscode2string(context_.slavelist[i].ALstatuscode) << ")\n";
    }
}

PdoCycleResult SoemBus::execute_one_cycle(const std::vector<PdoAxisCmd>& cmds,
                                          std::vector<PdoAxisFeedback>& prev_fb) {
    for (int i = 1; i <= slave_count_; ++i) {
        if (!active_mask_[i]) continue;
        auto* out_ptr = context_.slavelist[i].outputs;
        if (!out_ptr) continue;
        const auto& l = layouts_[i];
        const auto& cmd = cmds[i];
        const std::int32_t target = cmd.hold_actual ? prev_fb[i].actual_position : cmd.target_position;
        store_le(out_ptr + l.cw, cmd.control_word, 2);
        store_le(out_ptr + l.target_pos, static_cast<std::uint32_t>(target), 4);
        if (l.mode_op >= 0) {
            out_ptr[l.mode_op] = 8; // CSP
        }
    }

    auto clock_us = []() -> std::int64_t {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    };

    const PdoCycleResult res = bounded_pdo_exchange(expected_wkc_, clock_us, [&](int timeout_us) {
        const int sent = ecx_send_processdata_group(&context_, kMotionPdoGroup);
        auto frame = receive_pdo_frames(context_, timeout_us);
        frame.sent = (sent > 0);
        return frame;
    });

    if (res.healthy) {
        for (int i = 1; i <= slave_count_; ++i) {
            if (!active_mask_[i]) continue;
            const auto* in_ptr = context_.slavelist[i].inputs;
            if (!in_ptr) continue;
            const auto& l = layouts_[i];
            PdoAxisFeedback fb;
            fb.valid = true;
            fb.status_word = static_cast<std::uint16_t>(load_le(in_ptr + l.sw, 2));
            fb.actual_position = static_cast<std::int32_t>(load_le(in_ptr + l.actual_pos, 4));
            if (l.actual_vel >= 0) {
                fb.actual_velocity = static_cast<std::int32_t>(load_le(in_ptr + l.actual_vel, 4));
            }
            if (l.actual_torque >= 0) {
                fb.actual_torque = static_cast<std::int16_t>(load_le(in_ptr + l.actual_torque, 2));
            }
            if (l.error_code >= 0) {
                fb.error_code = static_cast<std::uint16_t>(load_le(in_ptr + l.error_code, 2));
            }
            fb.mode_display = (l.mode_disp >= 0) ? static_cast<std::int8_t>(in_ptr[l.mode_disp]) : sdo_modes_[i];
            prev_fb[i] = fb;
        }
    }
    return res;
}

void SoemBus::pdo_worker_loop() {
    ::pthread_setname_np(::pthread_self(), "soem_pdo_loop");
    (void)::mlockall(MCL_CURRENT);
    sched_param sp{};
    sp.sched_priority = 10;
    (void)::sched_setscheduler(0, SCHED_FIFO, &sp);

    constexpr auto kCyclePeriod = std::chrono::microseconds(2000);
    auto next_cycle = std::chrono::steady_clock::now();
    int consecutive_bad_wkc = 0;

    std::vector<PdoAxisCmd> local_cmds(slave_count_ + 1);
    std::vector<PdoAxisFeedback> local_prev_fb(slave_count_ + 1);
    {
        std::lock_guard<std::mutex> lock(pdo_mutex_);
        local_prev_fb = latest_fb_;
    }

    while (pdo_running_.load()) {
        const auto cycle_start = std::chrono::steady_clock::now();
        const std::int64_t late_us = std::chrono::duration_cast<std::chrono::microseconds>(
            cycle_start - next_cycle).count();
        if (late_us > max_late_us_.load()) {
            max_late_us_.store(late_us);
        }

        {
            std::lock_guard<std::mutex> lock(pdo_mutex_);
            local_cmds = desired_cmds_;
        }

        const PdoCycleResult res = execute_one_cycle(local_cmds, local_prev_fb);

        const std::int64_t ex_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - cycle_start).count();
        if (ex_us > max_cycle_us_.load()) {
            max_cycle_us_.store(ex_us);
        }
        last_wkc_.store(res.last.total);
        total_cycles_.fetch_add(1);
        if (res.attempts > 1) {
            retried_cycles_.fetch_add(1);
        }

        if (res.healthy) {
            consecutive_bad_wkc = 0;
            {
                std::lock_guard<std::mutex> lock(pdo_mutex_);
                latest_fb_ = local_prev_fb;
                last_exchange_ = res;
            }
            has_valid_pdo_.store(true);
        } else {
            {
                std::lock_guard<std::mutex> lock(pdo_mutex_);
                last_exchange_ = res;
            }
            ++consecutive_bad_wkc;
            if (consecutive_bad_wkc >= 5) {
                pdo_healthy_.store(false);
                std::vector<PdoAxisCmd> safe_cmds(slave_count_ + 1);
                for (int i = 1; i <= slave_count_; ++i) {
                    safe_cmds[i] = {CW_SHUTDOWN, local_prev_fb[i].actual_position, true};
                }
                execute_one_cycle(safe_cmds, local_prev_fb);
                break;
            }
        }

        next_cycle += kCyclePeriod;
        if (std::chrono::steady_clock::now() > next_cycle + std::chrono::milliseconds(20)) {
            next_cycle = std::chrono::steady_clock::now();
        }
        std::this_thread::sleep_until(next_cycle);
    }
}

} // namespace robot_maint
