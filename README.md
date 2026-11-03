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
sudo ./build/arm_maint status -i eno0
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
sudo ./build/arm_maint status -i eno0
sudo ./build/arm_maint write-alias -i eno0 \
  --slave 1 --alias 18 \
  --backup ./backups/d18-sii-before-alias.bin \
  --confirm WRITE-EEPROM-ALIAS
```

成功后给该从站重新上电，再运行 `read-alias --slave 1` 和 `status`，确认 EEPROM/ESC Alias 均为 18 且 CRC 正确。工具允许将已有 Alias 修改为其他 1..65535 的值，日志显示旧值和目标值；EEPROM 已是目标值时只校验、不重写，也不会自动写 D19～D24。

### 8. 独立命令使能、点动、移动到零点和失能（支持多轴）

每条命令都从普通终端运行，可以在同一终端依次执行，也可以使用不同终端。`start-pdo` 先建立后台 PDO 通信，随后 `enable` 仅完成指定轴的使能握手；之后不需要在会话内输入指令。没有运行中的 PDO 服务时，`enable` 会提示先执行 `start-pdo`。所有命令使用相同的用户身份和网卡参数；以下为本机 `eno0` 示例。

```bash
# 先扫描全部受支持伺服并建立 PDO 通信，保持失能（当前 17 从站拓扑为 14 个同川）
sudo ./build/arm_maint start-pdo -i eno0

# 查询 PDO 服务；确认持续运行后再使能
sudo ./build/arm_maint status -i eno0

# 当前 17 从站拓扑示例：S2、S3 使能并移动到零点
sudo ./build/arm_maint enable -i eno0 --axis S2,S3
sudo ./build/arm_maint move-zero -i eno0 --axis S2,S3 --speed_rpm 0.5

# 停止运动并失能全部受控轴，关闭 PDO 通信并释放网口
sudo ./build/arm_maint stop-pdo -i eno0

# 以下 D18～D24 示例适用于仅扫描到 7 轴右臂的拓扑；先重新启动通信
sudo ./build/arm_maint start-pdo -i eno0

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

# 将已使能轴平滑移动到编码器绝对位置 0，完成后保持使能
sudo ./build/arm_maint move-zero -i eno0 --axis D18 --speed_rpm 0.3
sudo ./build/arm_maint move-zero -i eno0 --axis D18,D19 --speed_rpm 0.3

# 查询后台 PDO 反馈，不重新扫描或改变使能状态
sudo ./build/arm_maint status -i eno0
sudo ./build/arm_maint read-pos -i eno0

# 失能选定轴，其余轴继续保持
sudo ./build/arm_maint disable -i eno0 --axis D18,D19

# 失能全部受控轴、结束后台服务并释放网口
sudo ./build/arm_maint disable -i eno0 --all
```

`jog-enabled` 必须指定 `--delta_deg` 或 `--delta_count`，不能同时指定。位移可以是单个数值（所有目标轴使用相同增量），或逗号分隔的列表（依照 `--axis` 顺序逐一对应）；列表长度必须等于目标轴数量，重复 `--axis` 也按出现顺序对应。正负数表示不同方向，都是相对于上次指令终点的增量，不是绝对角度。列表中的 0 表示该轴保持不动，但该轴同样必须已使能；不能全部为 0。多轴使用同一个转速上限，按各轴位移和换算比例规划为同周期开始、同时结束；转速默认 0.5 RPM，允许 0.05～2.0 RPM。使用五次平滑插值，按峰值速度限制规划时长；单次命令最长 120 秒。独立终端对不同轴发送的点动也可并行；与正在运动的轴重叠的点动请求报错，不排队。

`move-zero` 以后台当前保持的指令位置为起点，将目标位置设为编码器绝对位置 0（0x607A = 0）。它只接受已使能的 CSP 轴，必须通过 `--axis` 显式列出单轴或多轴，不支持 `--all`，不接受 `--delta_deg` / `--delta_count`。`--speed_rpm` 默认 0.5，范围 0.05～2.0 RPM；沿用点动的五次平滑插值、多轴同步、120 秒时长上限、取消和异常停机规则。已在零点的轴保持不动，全部已在零点时也可成功完成。零点含义取决于已有编码器标定；该命令不执行 `set-zero`，不改写编码器零位。完成表示轨迹发送完成，未额外检测实际到位误差，之后继续保持使能。升级后需先结束旧后台服务，再用新版 `start-pdo` 启动服务才能使用新指令。

所有目标统一预检后才启动本次运动。任意目标未使能、PDO 状态无效、非 CSP、正在运动或目标位置超限，整条命令拒绝，所有目标都不开始本次点动。例如 D18 已使能、D19 未使能时：

```text
[失败] D19 未使能（SwitchOnDisabled (下使能)）；整条点动命令已拒绝，所有目标轴均未开始本次运动
```

若服务未启动，错误会逐一列出请求中的轴名，例如 `D18 未使能（无运行中的 PDO 使能服务）`。轴名按扫描序号解析；7 轴右臂显示 D18～D24，与 Station Alias 无关。其他拓扑建议使用 `S1`、`S2` 或数字序号。

运动结束后保持终点并继续使能，`jog-enabled` 进程返回终端。这里的完成表示轨迹发送完成，不是额外的到位误差检测。等待点动/使能命令时按 Ctrl+C 会关闭请求，服务取消本次目标并尝试失能；成功返回的 `enable` 不依赖该终端存活，需要用 `disable` 停止保持。

后台服务仅将通过核验的同川/意优伺服加入独立运动 PDO 组（SOEM group 1），映射、工作计数、周期收发和 SAFE-OP/OP/PRE-OP 切换均使用该组。运动组使用 LRD/LWR 分离读写；SOEM 将 LWR 工作计数乘 2 后与 LRD 相加，仍按 `2 × outputsWKC + inputsWKC` 严格判断通信健康，不将低计数当作正常。每周期最多在 2 ms 总预算内进行两次完整收发，两次使用相同控制字和绝对目标位置，只有单次完整应答才推进状态机/轨迹；不累加不同尝试的部分工作计数。两次仍不完整或超预算则停止。`status` 与错误日志提供读/写 WKC、缺帧数、尝试次数和累计恢复周期数。其他设备不加入运动过程映像，保持扫描初始化后的 PRE-OP；例如 S9 输入配置错误不会再阻止伺服组启动。组内任一伺服的状态或通信异常仍会阻止启动或停止运动。

运行前须确认同一网卡没有其他 EtherCAT 主站。IgH 即使显示 `Phase: Idle`、`Active: no`，仍可能发送扫描帧，与 SOEM 的帧索引冲突，导致启动后出现命令不匹配和 WKC 不足。可用 `ethercat master` 检查；确认 IgH 无控制任务且不再使用后，执行 `sudo modprobe -r ec_generic ec_master` 释放网卡。该操作卸载内核主站模块，影响全部 IgH 主站；本工具的文件锁不能阻止 IgH 访问网卡。

后台服务为每个网口持有独占锁，只有一个进程访问 SOEM/PDO。服务启动时读取真实 PDO 映射确认 Controlword、Target Position、Statusword、Actual Position 等字段，不按字节总长猜偏移，也不在运行期间重映射 PDO。每 2 ms 交换过程数据；工作计数、驱动状态或控制周期异常时停止全部受控运动并尝试失能。收到对多轴运动中一个成员的失能命令时，同一批次的其他成员停止增量并保持当前指令位置。

服务运行时 `status`、`read-pos` 从同一服务读取快照，不建立第二个主站。`jog`、清错、标零、SDO/EEPROM 读写等其他维护操作会拒绝执行；先 `disable --all` 结束服务再操作。没有后台服务时，`disable` 仍可使用原有 SDO Shutdown 方式失能。

启动时在 PRE-OP 写入 CSP 请求（`0x6060 = 8`），先保持 Shutdown（控制字 6）建立 OP/PDO 通信，再确认实际模式 `0x6061 = 8`。部分驱动在 PRE-OP 下 `0x6061` 仍为 0，因此不在该阶段判定模式失败。未映射 `0x6061` 的轴在 OP 下通过 SDO 确认并缓存结果；实时循环使用该缓存，不能持续检测其后通过外部方式发生的模式变更。映射了 `0x6061` 的轴逐周期使用实际 PDO 模式反馈。任一目标轴未确认 CSP 时整条使能请求被拒绝。

后台启动最多等待 120 秒；日志记录扫描、逐轴 PDO/CSP/换算参数核验及 SAFE-OP/OP 阶段和耗时。启动失败会返回具体原因；总线状态切换失败时附带各从站状态和 AL 错误码。

后台循环尝试锁定当前内存并使用 `SCHED_FIFO` 优先级 10，以减少桌面负载引起的调度抖动；权限不足会在日志中说明。完整交换失败后的 WKC 保护和 20 ms 周期延迟保护保持生效。日志会记录异常周期的实际/期望 WKC、收发耗时、调度延迟、退出原因，以及失能退出前的运动组 AL 状态。

后台日志位于 `/tmp/arm_maint-control-<运行用户UID>/<网卡>.log`；使用 sudo 时通常为 `/tmp/arm_maint-control-0/eno0.log`。不要直接删除活动服务的套接字或锁文件；用 `disable --all` 退出。

原有 `jog` 保留自动使能、点动、结束失能的行为；独立 `jog-enabled` 从不自动使能。抱闸动作依赖驱动器配置，程序没有独立抱闸反馈检查。

新增流程已通过模拟 PDO 的多轴原子校验、握手、运动规划、错误轴名、取消/失能和通信异常测试，以及 Unix 套接字 CLI 测试。尚未进行实机使能/运动验证；本次开发没有向电机发送使能或运动指令。

升级含不同位移列表的版本后，若旧后台服务仍在运行，先执行 `disable -i eno0 --all`，再用新版 `start-pdo` 重新启动。仅替换可执行文件不会改变已经运行的旧进程；不要在使能保持期间直接删除套接字或杀进程来升级。

## 同川 PDO 配置（SOEM）

`configure-pdo` 对齐 lagbotwbc 的同川机械臂映射，仅处理指定物理从站，
不根据 D 编号或 Alias 猜测目标。默认只读比较并打印原始映射；
追加 `--confirm --backup` 后，先独占创建并 fsync 文本备份，再仅修改不一致的一侧。
命令复用维护程序的网口独占锁，要求 PREOP、同川 Vendor=0x75500（读取并记录 Product Code，不限制产品编号），
且驱动未处于 Operation Enabled。检查每次写入结果，激活映射数量后逐条回读（SI0=0 时固件可能隐藏条目），失败立即停止；可能留下部分映射，
应保留日志和备份，不自动重试。不会使能电机、发送运动 PDO 或自动保存 Flash。

`--slaves` 支持物理从站范围与列表（`2`、`2-8`、`2,3,5`、`2-8,10-16`），
拒绝重复和倒序范围。先对全部目标做只读身份、状态、映射预检，通过后依次写入。
任何一台失败立即停止，已完成的电机不回滚，后续电机不写入。
批次不是原子操作；仍要求电机未使能，并由本程序独占网口。

```bash
# 按当前拓扑检查左臂；右臂为 10-16，1、9 不是电机
sudo ./build/arm_maint configure-pdo -i eno0 --slaves 2-8
# 写入左臂；父目录 backups 须存在，批次目录必须不存在
sudo ./build/arm_maint configure-pdo -i eno0 --slaves 2-8 \
  --confirm --backup-dir ./backups/left-arm-pdo-batch1
```

目标 RxPDO 0x1601（17 字节），各对象子索引均为 0：
6040:16、607A:32、60FF:32、6071:16、60B1:32、6060:8。
目标 TxPDO 0x1A01（25 字节）：
6041:16、6061:8、6064:32、606C:32、6077:16、60F4:32、603F:16、6079:32、300F:16。
分别由 0x1C12、0x1C13 分配。Rx 已一致时只重写 Tx；两侧都一致时不写入。

依据《TCHL_ECat协议说明书V2.0.8》7.5.3，每组允许最多 10 个映射条目。
手册 Pr1.61/0x313D 列出 33（0x1000 系列保存）与 88（所有参数保存），
但未明确逐项保证 PDO 映射的断电保留；默认不调用这些保存操作；显式保存选项见下文。
手册映射数量子索引的 RO 标注与已有 EC-Master 逐条重映射实践存在差异，
需要在新臂固件上实测写入能力；命令失败不会绕过错误继续写。
`PDO WRITE PASS` 仅表示当前映射回读一致，不代表 SAFEOP/OP 周期通信或断电保留验证通过。
生产使用应由主站启动阶段配置并核验 PDO，或另行验证厂商保存机制。

### 保存 PDO 所在的 0x1000 系列参数

显式添加 `--save` 会在映射校验成功后向 Pr1.61 `0x313D:00` 写入
**UINT16 十进制 33**，请求保存 0x1000 系列参数。该选项要求 `--confirm`
与备份路径；默认仍只修改运行时映射。即使映射已经一致，`--save` 也会先备份
当前映射再发送保存命令，不能因“一致”而跳过保存。
注意 33 的保存范围是整个 0x1000 系列，不只本次修改的 PDO。

```bash
# 建议先单台验证断电保留，使用全新的备份文件
sudo ./build/arm_maint configure-pdo -i eno0 --slave 2 \
  --confirm --backup ./backups/slave2-before-save.txt --save
# 单台断电验证通过后，可批量保存，使用全新的批次目录
sudo ./build/arm_maint configure-pdo -i eno0 --slaves 2-8,10-16 \
  --confirm --backup-dir ./backups/arm-pdo-save-batch1 --save
```

### PDO 通信与使能分步执行

`start-pdo -i eno0` 自动选择扫描到的全部同川/意优 CoE 伺服，核验所有目标的 PDO/CSP 配置，建立 OP 通信并在失能状态连续验证约 1 秒后返回。当前 17 从站拓扑会选中 S2～S8、S10～S16，共 14 个同川电机；S1、S9、S17 不加入运动 PDO 组。全部候选伺服必须通过核验。此命令不调用使能流程，控制字保持 Shutdown（6）；重复执行仅返回现有服务状态，不重新扫描，也不改变已有使能状态。`enable` 必须在服务启动后调用。

`stop-pdo -i eno0` 取消受控运动并失能所有可控制伺服，确认失能后关闭 PDO 通信、释放网口，再返回成功；未运行时直接成功返回“无需停止”。若失能未确认则报告失败，不宣称停止成功。`disable --all` 保留相同的关闭服务行为。`start-pdo` / `stop-pdo` 只接受网卡选项，不接受轴列表、`--all` 或运动参数；不提供其他拼写别名。
