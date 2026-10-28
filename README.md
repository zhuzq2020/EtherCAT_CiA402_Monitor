# 机械臂现场维护与零点标定工具 (arm_maint)

专为现场售后、维护、装配人员打造的轻量级 EtherCAT 机械臂维护工具。

## 特性亮点

- **零内核模块依赖**：基于纯用户态 SOEM 协议栈，通过 Linux 原生 Raw Socket 访问网卡，**无需加载 `ec_master.ko` 等内核驱动**，升级内核绝不崩溃。
- **零外部动态库依赖**：项目内置 SOEM 源码静态打包，生成单一小巧二进制（< 400KB），无需安装系统依赖。
- **维护与运动分开**：常规 SDO/EEPROM 维护使用 PRE-OP；使能和点动由持有网口的 PDO 进程在 OP 下执行。
- **核心功能**：
  1. `status`：查看从站拓扑、CiA-402状态、故障码、实际脉冲；
  2. `clear-fault`：单轴 / 全轴清除故障报警；
  3. `set-zero`：单轴 / 全轴标定绝对值编码器零位（同川 0x3801 协议，强制 `--confirm` 防误触）；
  4. `read-pos`：快速读取当前编码器脉冲；
  5. `read-sdo`：按索引、子索引读取任意 CoE 对象；
  6. `write-alias`：备份 SII EEPROM 后，为 Hot Connect 组首写入 Station Alias；
  7. `enable` / `disable`：单轴或多轴使能/失能，后台持续保持；
  8. `jog-enabled`：独立命令，对已经使能的单轴或多轴点动，完成后继续使能。

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

### 8. 独立命令使能、点动和失能（支持多轴）

每条命令都从普通终端运行，可以在同一终端依次执行，也可以使用不同终端。`enable` 启动后台 PDO 服务并完成握手后返回，之后不需要在会话内输入指令。所有命令使用相同的用户身份和网卡参数；以下为本机 `eno0` 示例。

```bash
# 单轴使能，成功后返回终端；后台保持当前位置
sudo ./build/arm_maint enable -i eno0 --axis D18

# 独立点动命令：只接受已经使能的轴，不自动使能
sudo ./build/arm_maint jog-enabled -i eno0 --axis D18 --delta_deg 1.0 --speed_rpm 0.3
sudo ./build/arm_maint jog-enabled -i eno0 --axis D18 --delta_count -5000

# 同时使能多个轴，也可以重复 --axis D18 --axis D19
sudo ./build/arm_maint enable -i eno0 --axis D18,D19

# 两个目标在同一周期开始，按各自的角度换算比例同步规划
sudo ./build/arm_maint jog-enabled -i eno0 --axis D18,D19 --delta_deg 1.0 --speed_rpm 0.3

# 不同角度：D18 +1.0 度，D19 -0.5 度，按 --axis 顺序对应
sudo ./build/arm_maint jog-enabled -i eno0 --axis D18,D19 --delta_deg 1.0,-0.5 --speed_rpm 0.3

# 不同脉冲数：D18 +5000，D19 -2000
sudo ./build/arm_maint jog-enabled -i eno0 --axis D18,D19 --delta_count 5000,-2000

# 查询后台 PDO 反馈，不重新扫描或改变使能状态
sudo ./build/arm_maint status -i eno0
sudo ./build/arm_maint read-pos -i eno0

# 失能选定轴，其余轴继续保持
sudo ./build/arm_maint disable -i eno0 --axis D18,D19

# 失能全部受控轴、结束后台服务并释放网口
sudo ./build/arm_maint disable -i eno0 --all
```

`jog-enabled` 必须指定 `--delta_deg` 或 `--delta_count`，不能同时指定。位移可以是单个数值（所有目标轴使用相同增量），或逗号分隔的列表（依照 `--axis` 顺序逐一对应）；列表长度必须等于目标轴数量，重复 `--axis` 也按出现顺序对应。正负数表示不同方向，都是相对于上次指令终点的增量，不是绝对角度。列表中的 0 表示该轴保持不动，但该轴同样必须已使能；不能全部为 0。多轴使用同一个转速上限，按各轴位移和换算比例规划为同周期开始、同时结束；转速默认 0.5 RPM，允许 0.05～2.0 RPM。使用五次平滑插值，按峰值速度限制规划时长；单次命令最长 120 秒。独立终端对不同轴发送的点动也可并行；与正在运动的轴重叠的点动请求报错，不排队。

所有目标统一预检后才启动本次运动。任意目标未使能、PDO 状态无效、非 CSP、正在运动或目标位置超限，整条命令拒绝，所有目标都不开始本次点动。例如 D18 已使能、D19 未使能时：

```text
[失败] D19 未使能（SwitchOnDisabled (下使能)）；整条点动命令已拒绝，所有目标轴均未开始本次运动
```

若服务未启动，错误会逐一列出请求中的轴名，例如 `D18 未使能（无运行中的 PDO 使能服务）`。轴名按扫描序号解析；7 轴右臂显示 D18～D24，与 Station Alias 无关。其他拓扑建议使用 `S1`、`S2` 或数字序号。

运动结束后保持终点并继续使能，`jog-enabled` 进程返回终端。这里的完成表示轨迹发送完成，不是额外的到位误差检测。等待点动/使能命令时按 Ctrl+C 会关闭请求，服务取消本次目标并尝试失能；成功返回的 `enable` 不依赖该终端存活，需要用 `disable` 停止保持。

后台服务为每个网口持有独占锁，只有一个进程访问 SOEM/PDO。服务启动时读取真实 PDO 映射确认 Controlword、Target Position、Statusword、Actual Position 等字段，不按字节总长猜偏移，也不在运行期间重映射 PDO。每 2 ms 交换过程数据；工作计数、驱动状态或控制周期异常时停止全部受控运动并尝试失能。收到对多轴运动中一个成员的失能命令时，同一批次的其他成员停止增量并保持当前指令位置。

服务运行时 `status`、`read-pos` 从同一服务读取快照，不建立第二个主站。`jog`、清错、标零、SDO/EEPROM 读写等其他维护操作会拒绝执行；先 `disable --all` 结束服务再操作。没有后台服务时，`disable` 仍可使用原有 SDO Shutdown 方式失能。

后台日志位于 `/tmp/arm_maint-control-<运行用户UID>/<网卡>.log`；使用 sudo 时通常为 `/tmp/arm_maint-control-0/eno0.log`。不要直接删除活动服务的套接字或锁文件；用 `disable --all` 退出。

原有 `jog` 保留自动使能、点动、结束失能的行为；独立 `jog-enabled` 从不自动使能。抱闸动作依赖驱动器配置，程序没有独立抱闸反馈检查。

新增流程已通过模拟 PDO 的多轴原子校验、握手、运动规划、错误轴名、取消/失能和通信异常测试，以及 Unix 套接字 CLI 测试。尚未进行实机使能/运动验证；本次开发没有向电机发送使能或运动指令。

升级含不同位移列表的版本后，若旧后台服务仍在运行，先执行 `disable -i eno0 --all`，再用新版 `enable` 重新启动。仅替换可执行文件不会改变已经运行的旧进程；不要在使能保持期间直接删除套接字或杀进程来升级。
