# 同川 SOEM Alias 实机验证（2026-10-26）

## 结果

本机 eno0、物理从站 1（Vendor=0x75500、Product=2、Revision=6），成功将 EEPROM Alias 从 18 改为 11。独立运行 read-alias 得到 ESC Alias=11、EEPROM Alias=11、CRC stored/calculated=86。7 个从站扫描正常，CiA-402 故障码均为 0。未自动重启控制器、未发送运动命令。

## 修正

保持 EEPROM 强制接管 config=0x02 并不是充分条件。将单字写报文从 8 字节扩为 10 字节后，单独写 word 4 再立即回读仍失败。

对照 EC-Master 的 ESC 配置区更新流程，最终实现为：从完整备份构建 word 0..7 配置区，只修改 Alias 和 CRC；依次写入全部八个 words，每步检查通信、控制权、BUSY 和错误位；word 7 的 CRC 最后写入；在完整配置区发送完成之前不发 EEPROM READ 命令。之后校验配置区，并读取、保存、逐字比较完整 EEPROM。

这次完整配置区流程实机成功。不能从这些结果单独断言设备内部具体的缓存或持久化实现；不能把无错误 ACK 当作数据已保存。

## 备份与抓包

失败的 10 字节单字尝试：`backups/alias-10byte-20261026-161613/`。完整 2048 字节写前写后镜像相同，Alias 仍为 18。已停止该次尝试，未自动重发命令。

成功的完整配置区尝试：`backups/alias-header-20261026-162032/`。

- `sii-before.bin`：完整 2048 字节写前备份，SHA-256 `6ad11fa11a423ba0820b2ee2bd837ed47f8890d27378306bf0eabdc8c07f2da8`。
- `sii-before.bin.after.bin`：完整 2048 字节写后镜像，SHA-256 `64815d4a810b69224b67d92d836246059951e18c70de5210ce9daf21f174b44e`。
- 两份镜像只有 word 4 和 word 7 变化，PDO 描述与所有其他 EEPROM 内容一致。
- `write.log`：完整写入输出；`write.exitcode` 为 0。
- `read-after.log`、`status-after.log`：独立实机复查记录。
- `write.pcap`：实际发送及返回报文；抓包由本次进程管理，已经结束。
- `code/`：执行本次写入时的代码和二进制快照。

`docs/alias-write-datagrams-20261026.json` 摘录了两个尝试的写命令返回报文。成功流程中 word 0..7 顺序完整、每个 WKC=1，word 4=11、word 7=86。

## 校验

编译通过；命令行参数、CRC/完整镜像、控制权/寄存器协议/配置区事务等三项测试通过。事务测试覆盖在 CRC 完成前读取仍返回旧值的设备模型，确认不会在配置区写入中途插入 EEPROM 读命令；同时保留错误停止和恢复控制权检查。该模型仅用于回归测试，不是对厂家固件内部实现的断言。

实机已恢复原控制配置 config=0x01。没有清除错误或自动回滚；成功后未再次写入，也没有写其他从站的 Alias。
