# EtherCAT 机械臂维护工具（arm_maint）

基于 SOEM 用户态协议栈，支持同川、意优及 CiA-402 伺服的状态查询、标定和运动控制，无需 EtherCAT 内核模块。

## 编译

```bash
./scripts/build.sh
# 可选：授予网卡访问权限，之后可免 sudo
./scripts/grant_cap.sh
```

## 指令速查

统一格式：`sudo ./build/arm_maint -i eno0 <命令> <参数>`。网卡默认 `enp2s0`，请按实际连接修改。

| 命令 | 功能 | 参数 / 示例（接在统一前缀后） |
| --- | --- | --- |
| `status` | 查看拓扑、厂商、状态、故障和位置 | `status` |
| `read-pos` | 读取编码器位置；默认全部轴 | `read-pos --axis S2` 或 `read-pos --all` |
| `read-sdo` | 读取 CoE 对象；默认全部从站，子索引默认 0 | `read-sdo --axis S2 --index 0x6064 [--subindex 0]` |
| `read-sdo`（扭矩） | 读取同川扭矩传感器原始值 | `read-sdo --axis S2 --index 0x303A` |
| `clear-fault` | 清除驱动器故障 | `clear-fault --axis S2` 或 `clear-fault --all` |
| `set-zero` | 将当前位置标定为编码器零点；同川写 `0x3801:00 = 5` | `set-zero --axis S2 --confirm` 或 `set-zero --all --confirm` |
| `zero-torque` | 同川扭矩传感器清零，写 `0x3801:00 = 10`（uint16） | `zero-torque --axis S2 --confirm` 或 `zero-torque --all --confirm` |
| `jog` | 单轴自动使能、点动，结束后失能 | `jog --axis S2 --delta_deg 1.0 [--speed_rpm 0.3]` |
| `start-pdo` | 扫描受支持伺服，启动后台 PDO 通信，初始保持失能 | `start-pdo` |
| `enable` | 使能指定轴，后台保持当前位置；需先启动 PDO 服务 | `enable --axis S2,S3` |
| `jog-enabled` | 已使能轴增量点动，结束后继续使能 | `jog-enabled --axis S2,S3 --delta_deg 1.0,-0.5 [--speed_rpm 0.3]` |
| `move-zero` | 已使能轴移动到编码器位置 0，结束后继续使能 | `move-zero --axis S2,S3 [--speed_rpm 0.3]` |
| `disable` | 指定轴失能；`--all` 同时结束后台服务 | `disable --axis S2,S3` 或 `disable --all` |
| `stop-pdo` | 停止运动、失能全部受控轴并释放网口 | `stop-pdo` |
| `read-alias` | 读取 ESC / EEPROM Alias、CRC 和错误状态 | `read-alias --slave 1` |
| `backup-sii` | 备份完整 EEPROM；文件必须不存在 | `backup-sii --slave 1 --backup ./backups/sii.bin` |
| `write-alias` | 备份后写入 Station Alias（1～65535） | `write-alias --slave 1 --alias 18 --backup ./backups/sii.bin --confirm WRITE-EEPROM-ALIAS` |
| `configure-pdo` | 只读比较同川 PDO 映射 | `configure-pdo --slave 2` 或 `configure-pdo --slaves 2-8,10-16` |
| `configure-pdo`（写入） | 备份后修改不一致的映射 | `configure-pdo --slave 2 --confirm --backup ./backups/pdo.txt`；批量用 `--slaves 2-8 --confirm --backup-dir ./backups/pdo-batch` |
| `configure-pdo --save` | 映射核验后写 `0x313D:00 = 33`，请求保存整个 0x1000 系列 | 在上述写入命令后追加 `--save` |
| `--help` | 查看完整帮助 | `--help` |

方括号表示可选参数，执行时不输入方括号。

## 轴选择与操作约定

- `--axis S2` 或 `--axis 2` 表示本次扫描的第 2 个从站。仅连接右臂 7 轴时可用 `D18`～`D24`，它们按拓扑顺序解析，与 Station Alias 无关。
- 多轴 `--axis S2,S3`（或重复 `--axis`）用于 `enable`、`disable`、`jog-enabled`、`move-zero`；其他轴操作使用单轴或表中支持的 `--all`。`zero-torque` 只能二选一指定单轴或全轴。
- `--slave` / `--slaves` 始终使用物理扫描序号。Alias 是 EEPROM 中的独立标识；Hot Connect 通常只设置组首，写入后需重新上电并核对 ESC / EEPROM Alias 和 CRC。
- 运动顺序：`start-pdo` → `enable` → `jog-enabled` / `move-zero` → `stop-pdo`。各命令使用相同用户身份和网卡；升级程序后先停止旧服务再启动新版。
- `jog` / `jog-enabled` 可用 `--delta_count` 替代 `--delta_deg`。多轴位移可填单值或按轴顺序填写等长列表。转速默认 0.5 RPM，范围 0.05～2.0 RPM，单次运动最长 120 秒。`move-zero` 不接受位移参数或 `--all`。

## 执行前须知

- 同一网卡只能由一个 EtherCAT 主站使用。后台 PDO 服务运行时，`status` / `read-pos` 读取服务快照；其他维护操作前先 `stop-pdo`。IgH 即使处于 Idle 也可能占用网卡。
- `set-zero` 需先失能并对齐机械零位；同川手册要求写入后断电重启。`zero-torque` 需先失能并处于所需零扭矩条件，仅支持同川；写入应答后仍需读取扭矩反馈验证效果。
- `jog-enabled` / `move-zero` 要求所有目标已使能且处于 CSP；任一目标预检失败则整条运动拒绝。完成表示轨迹发送完成，未额外判断实际到位误差；抱闸动作依赖驱动配置。
- EEPROM / PDO 写入的备份文件或批次目录必须不存在；批次目录的父目录须存在。批量 PDO 写入失败即停止，已完成目标不回滚。`--save` 的断电保留效果需实机验证。
- 扭矩清零的 `--all` 会逐轴处理，非同川或其他失败轴报错，任一失败则退出码为 1。SDO 输出为原始字节及整数解释，数据类型和单位以对应固件的对象字典为准。

后台日志：`/tmp/arm_maint-control-<UID>/<网卡>.log`，例如 sudo 下的 `/tmp/arm_maint-control-0/eno0.log`。停止服务请使用 `stop-pdo`，不要直接删除活动套接字或锁文件。
