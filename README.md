# 通用机械臂 EtherCAT (SOEM) 控制与维护 SDK（arm_maint）

基于纯用户态 **SOEM (Simple Open EtherCAT Master)** 实现的通用机械臂 EtherCAT 控制 SDK，支持 **法奥 (Fairino)**、**同川 (Tongchuan)**、**意优 (Eyou)** 及标准 CiA-402 伺服驱动器，无需 EtherCAT 内核模块。

---

## 1. 架构概览

项目采用极简分层设计：

1. **C++ 通用底层 SDK (`arm_soem_sdk` + `arm_maint`)**
   - `SoemBus` (`include/soem_bus.hpp`)：统一封装网卡锁 (`InterfaceLock`)、总线初始化、CoE SDO 读写、ESC 寄存器 (`FPRD`/`FPWR`)、动态 PDO 映射发现 (`0x1C12`/`0x1C13`)、运动组隔离 (`kMotionPdoGroup=1`)、有界防丢包交换与 2ms `SCHED_FIFO` 实时周期线程。
   - `DriveProfile` (`include/drive_profile.hpp`)：抽象各厂商伺服差异（法奥固定 CSP、同川/意优编码器分辨率与减速比解析、`set-zero` 零点标定）。
   - `ArmController` (`include/arm_controller.hpp`)：统一高层运动控制 SDK（`open_motion` / `move_axes_to` / `hold_motion` / `close_motion`），内置自动故障复位、平滑 CiA-402 使能（含 400ms 抱闸释放稳态保位）、五次多项式插值 (`smoothstep5`) 与安全失能。
   - `rpc_session` (`include/rpc_session.hpp`)：单会话标准输入/输出 RPC 接口，供 Python 流程脚本高效复用同一网口会话执行多步操作。

2. **Python 流程脚本（全部位于 `scripts/`）**
   - `scripts/pose_control.py`：多姿态同步平滑轨迹控制（支持 **法奥 / 同川 / 意优**）。
   - `scripts/backup_sii.py`：完整 SII EEPROM 只读备份。
   - `scripts/read_alias.py`：读取 ESC / EEPROM Alias 与配置 CRC。
   - `scripts/write_alias.py`：安全备份后写入 Station Alias 并逐字校验完整镜像。
   - `scripts/configure_pdo.py`：同川 PDO 映射只读比较、备份与写入对齐。

---

## 2. 编译、授权与测试

```bash
./scripts/build.sh
# 可选：授予网卡 raw socket 权限，之后可免 sudo 运行
./scripts/grant_cap.sh
# 运行单元测试
ctest --test-dir build --output-on-failure
```

---

## 3. 底层指令速查 (`./build/arm_maint`)

统一前缀：`./build/arm_maint -i eno0 <命令> [参数]`（未配置 `grant_cap.sh` 时加 `sudo`）。  
轴号支持 `J1..J7`、`D18..D24`、`S1..SN` 或物理从站序号 `1..N`。

| 命令 | 功能 | 示例 |
| --- | --- | --- |
| `status` | 扫描总线拓扑、厂商、型号、CiA-402 状态、故障码与位置（`--detail` 显示详情） | `status` / `status --detail` |
| `read-sdo` | 读取指定 CoE SDO 对象（默认全部从站，subindex 默认 0） | `read-sdo --axis J1 --index 0x6064` |
| `write-sdo` | 写入指定 CoE SDO 对象（需 `--confirm`） | `write-sdo --axis S2 --index 0x6060 --type i8 --value 8 --confirm` |
| `set-zero` | 标定当前位置为编码器零点（支持同川/意优，需 `--confirm`） | `set-zero --axis D18 --confirm` / `--all --confirm` |
| `jog` | 单轴相对平滑点动并保持终点 | `jog J7 5.0 3000` 或 `jog --axis J7 --deg 5.0 --ms 3000` |
| `jog-abs` | 单轴平滑点动到绝对物理角度并保持终点 | `jog-abs J4 90.0 8000` 或 `jog --axis J4 --deg 90 --abs` |
| `zero` | 单轴从当前位置平滑运动归零 `0.000°` | `zero J7 5000` |

---

## 4. Python 流程脚本使用指南 (`scripts/`)

```bash
# 多姿态同步平滑轨迹控制（法奥 / 同川 / 意优，无需重编译 C++）
python3 scripts/pose_control.py -i eno0 --mode once --speed 20.0 --interval 2.0
python3 scripts/pose_control.py -i eno0 --mode repeat --speed 20.0 --interval 2.0
python3 scripts/pose_control.py -i eno0 --axes S2,S3,S4,S5,S6,S7,S8 --mode once
python3 scripts/pose_control.py --dry-run

# EEPROM SII 备份与 Alias 读写
python3 scripts/backup_sii.py -i eno0 --slave 1 --backup ./backups/sii.bin
python3 scripts/read_alias.py -i eno0 --slave 1
python3 scripts/write_alias.py -i eno0 --slave 1 --alias 18 --backup ./backups/sii.bin --confirm WRITE-EEPROM-ALIAS

# 同川 PDO 映射核验与写入
python3 scripts/configure_pdo.py -i eno0 --slave 2
python3 scripts/configure_pdo.py -i eno0 --slaves 2-8 --confirm --backup-dir ./backups/pdo-batch --save
```
