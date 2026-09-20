# 机械臂现场维护与零点标定工具 (arm_maint)

专为现场售后、维护、装配人员打造的轻量级 EtherCAT 机械臂维护工具。

## 特性亮点

- **零内核模块依赖**：基于纯用户态 SOEM 协议栈，通过 Linux 原生 Raw Socket 访问网卡，**无需加载 `ec_master.ko` 等内核驱动**，升级内核绝不崩溃。
- **零外部动态库依赖**：项目内置 SOEM 源码静态打包，生成单一小巧二进制（< 400KB），无需安装系统依赖。
- **纯 SDO 安全交互**：仅初始化至 PRE-OP 状态，杜绝任何电机误动飞车风险。
- **核心功能**：
  1. `status`：查看从站拓扑、CiA-402状态、故障码、实际脉冲；
  2. `clear-fault`：单轴 / 全轴清除故障报警；
  3. `set-zero`：单轴 / 全轴标定绝对值编码器零位（同川 0x3801 协议，强制 `--confirm` 防误触）；
  4. `read-pos`：快速读取当前编码器脉冲。
  5. `write-alias`：备份 SII EEPROM 后，为任意厂商的 Hot Connect 组首写入 Station Alias。

## 从站 ID、D18 与 Alias

- **从站 ID** 是主站每次扫描总线得到的临时顺序号，取决于当时的物理拓扑，不是电机铭牌身份。
- **D18** 是机器人配置里的逻辑关节名。本工具在单独连接右臂 7 轴时，按拓扑顺序显示 D18～D24；这不代表该字符串存储在每台电机中。
- **Station Alias** 位于 EtherCAT 从站 SII EEPROM。全新电机通常为 0，不会天然带 D18。整机 Hot Connect 方案只给每条链的组首写 Alias：右臂组首 D18 写 18，D19～D24 不需要依次写 19～24。

---

## 快速编译

在 Linux 工控机或笔记本上执行：

```bash
chmod +x scripts/*.sh
./scripts/build.sh
```

如需免 `sudo` 运行：
```bash
./scripts/grant_cap.sh
```

---

## 常用维护命令指南

### 1. 扫描总线与查看健康状态
```bash
sudo ./build/arm_maint status
# 或指定网卡名:
sudo ./build/arm_maint status -i enp2s0
```

### 2. 清除驱动器故障报警
```bash
# 单轴清错 (例如 D18 轴过流报警):
sudo ./build/arm_maint clear-fault --axis D18

# 整臂所有轴一键清错:
sudo ./build/arm_maint clear-fault --axis D18 --all
```

### 3. 点动微调 (JOG) 慢速转动对齐机械刻线
> 💡 **安全机制**：基于轻量受控 CSP 模式，带 5 次多项式平滑起停，输出转速严格限幅在安全低速（默认 0.5 RPM），运行中支持 `Ctrl+C` 立即减速抱闸。

```bash
# 单轴按度数微调 (例如 D18 轴微调转动 +1.0 度):
sudo ./build/arm_maint jog --axis D18 --delta_deg 1.0

# 单轴反向微调 (例如 D20 轴反转 -0.5 度，速度 0.3 RPM):
sudo ./build/arm_maint jog --axis D20 --delta_deg -0.5 --speed_rpm 0.3

# 单轴按脉冲数精确微调 (例如微调 +5000 counts):
sudo ./build/arm_maint jog --axis D18 --delta_count 5000
```

### 4. 标定零点 (绝对值编码器原点校准)
> ⚠️ **注意**：标零操作会将当前物理姿态永久保存进编码器存储芯片。必须在机械臂处于机械对准工装刻度位置时执行，并追加 `--confirm` 确认。

```bash
# 单轴标零 (更换了 D20 关节电机后的独立标定):
sudo ./build/arm_maint set-zero --axis D20 --confirm

# 整臂所有 7 轴一次性标零:
sudo ./build/arm_maint set-zero --all --confirm
```

### 5. 辅助读取实际位置
```bash
# 查看全臂各关节当前实际编码器计数:
sudo ./build/arm_maint read-pos
```

### 6. 写入右臂组首 D18 的 Station Alias

先运行 `status`，确认目标电机确实是物理从站 1 且当前 Alias 为 0。写入会按 SII 容量字段备份完整 EEPROM，并核对备份中的 Vendor ID/Product Code 与在线身份一致；备份路径必须不存在。意优型号会自动通过厂商对象 `0x2679` 临时打开写权限，同川和其他型号使用标准 EtherCAT EEPROM 写入流程。

```bash
sudo ./build/arm_maint status -i enp2s0
sudo ./build/arm_maint write-alias -i enp2s0 \
  --slave 1 --alias 18 \
  --backup ./backups/d18-sii-before-alias.bin \
  --confirm WRITE-EEPROM-ALIAS
```

成功后给该从站重新上电，再运行 `status`，确认 Alias 显示为 18。工具拒绝覆盖非零 Alias，也不会自动写 D19～D24。
