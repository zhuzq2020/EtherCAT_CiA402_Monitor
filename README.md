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
  4. `read-pos`：快速读取当前编码器脉冲；
  5. `read-sdo`：按索引、子索引读取任意 CoE 对象；
  6. `write-alias`：备份 SII EEPROM 后，为任意厂商的 Hot Connect 组首写入 Station Alias。

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

### 6. 读取指定 SDO 字段

`--index` 必填，`--subindex` 默认为 0。可用 `--axis` 指定逻辑轴名或本次扫描的从站序号；不指定轴时读取全部从站。输出原始字节；当数据长度为 1、2、4 或 8 字节时，同时给出有符号和无符号十进制解释。请根据对象字典选择正确的解释，例如 `0x6064:00` 是有符号 32 位实际位置。

```bash
# 读取 D18 的实际位置 0x6064:00
sudo ./build/arm_maint read-sdo --axis D18 --index 0x6064

# 读取物理从站 1 的 Vendor ID 0x1018:01
sudo ./build/arm_maint read-sdo --axis 1 --index 0x1018 --subindex 1

# 所有从站的状态字 0x6041:00
sudo ./build/arm_maint read-sdo --all --index 0x6041
```

单个对象最多读取 256 字节。失败时命令返回非零状态；请检查对象是否存在、从站是否支持 CoE、网络响应和数据长度。

### 7. 写入右臂组首 D18 的 Station Alias

先运行 `status` 和 `read-alias`，确认目标电机确实是物理从站 1、当前 EEPROM Alias 已核对且 CRC 正确。写入要求从站处于无 AL 错误的 INIT/PRE-OP；支持 CoE 的驱动还需确认未处于 Operation Enabled。同川型号读取 `Pr0.45 / 0x302D` 的 ECAT 报警码，读失败或非零时停止。

工具按 SII 声明容量读取完整 EEPROM，核对 Vendor ID/Product Code，独占新建备份并同步到磁盘后才开始写入。意优型号通过 `0x2679` 临时打开写权限；同川使用标准 SII 流程，不调用意优解锁或 `0x313D` 全参数保存。

```bash
sudo ./build/arm_maint status -i enp2s0
sudo ./build/arm_maint write-alias -i enp2s0 \
  --slave 1 --alias 18 \
  --backup ./backups/d18-sii-before-alias.bin \
  --confirm WRITE-EEPROM-ALIAS
```

成功后给该从站重新上电，再运行 `read-alias --slave 1` 和 `status`，确认 EEPROM/ESC Alias 均为 18 且 CRC 正确。工具允许将已有 Alias 修改为其他 1..65535 的值，日志显示旧值和目标值；EEPROM 已是目标值时只校验、不重写，也不会自动写 D19～D24。

### 8. 使能与失能

```bash
# PDO 使能单轴并持续保持启动时的实际位置；Ctrl+C 或 SIGTERM 失能退出
sudo ./build/arm_maint enable --axis D18

# 独立 SDO 失能，回读状态字确认结果
sudo ./build/arm_maint disable --axis D18
sudo ./build/arm_maint disable --all
```

`enable` 使用与 Jog 共用的 CSP/PDO 初始化和使能握手，不下发位移增量。
命令需要持续运行以维持 2 ms PDO 通信，不能退出后继续保证使能；不支持全轴同时使能。
保持期间检测过程数据工作计数和 Operation Enabled 状态，异常时尝试失能并报错。
请先停止 `enable`（Ctrl+C），再运行 Jog 或其他命令，勿让多个主站进程同时占用同一网口。
`disable` 在 PRE-OP 下通过 SDO 写入 `0x6040=0x0006`，确认驱动器已退出运行使能。
抱闸动作取决于驱动器配置，程序没有独立抱闸反馈检查；失能后负载需要机械支撑。
Jog 仍自动执行使能、运动、失能的完整流程，无需提前运行 `enable`。

### 9. 使能后点动，运动后保持使能

先启动单轴使能会话：

```bash
sudo ./build/arm_maint enable --axis D18
```

看到 `[使能成功]` 后，在**同一个终端**输入以下会话指令（不加 `sudo` 或程序名）：

```text
jog-enabled --delta_deg 1.0 --speed_rpm 0.3
jog-enabled --delta_count -5000
```

每次运动完成后继续发送 `0x000F` 并保持终点，可以反复输入点动指令。
位移相对于上一次指令终点；必须明确指定一种非零位移，转速默认 0.5 RPM，允许 0.05~2.0 RPM。
运动中拒绝新的点动指令，不排队；“指令发送完成”表示轨迹已发完，不代表实际位置已确认到位。
输入 `disable`、`quit`、Ctrl+C，或标准输入关闭，会结束会话并执行失能。
通信异常或驱动器退出运行使能时，也会终止并尝试失能。
`jog-enabled` 仅属于已经使能的会话，不是另一个进程的独立命令。
原有 `jog` 命令仍然会自动使能，运动结束自动失能。
