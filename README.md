# EtherCAT CiA402 状态机在线监控脚本

基于 SOEM + Qt 6/C++ 的只读上位机。它从 EtherCAT 从站自带的 SII/CoE 信息扫描设备，不需要 ESI XML，并通过 SDO 周期读取标准 CiA402 对象。SOEM 共享库与所需头文件已放在项目内的 `third_party/soem`，运行和构建都不依赖外部 SOEM 目录。

## 功能

- 枚举本机网卡并扫描 EtherCAT 从站，显示 Vendor ID、Product Code、Revision、CoE/DC 能力。
- 自动读取状态字 `6041h`、控制字 `6040h`、运行模式 `6060h/6061h`、位置 `6064h`、速度 `606Ch`、转矩 `6077h`、错误码 `603Fh` 和错误寄存器 `1001h`。
- 按 IEC 61800-7 / CiA402 状态字遮码解析 8 种标准状态。
- 读取工作在 Qt 后台线程，界面可调刷新周期。
- 内置无硬件演示模式，可先验证 UI 与状态告警。
- 默认严格只读：不写 `6040h`，不切换模式，不使能或驱动电机。

## Ubuntu 22.04 依赖

```bash
sudo apt update
sudo apt install -y build-essential qt6-base-dev libgl1-mesa-dev libcap2-bin
```

项目已内置为当前 Linux x86_64 主机构建的 SOEM 2.0 共享库，不再需要 `/home/admin1/workspace/SOEM`。

## 构建

```bash
cd /home/admin1/workspace/EtherCAT_CiA402_Monitor
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_PREFIX_PATH="$PWD/.deps/qt6/usr"
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

Qt6 未安装时，仍可单独构建和测试 CiA402 解析核心：

```bash
cmake -S . -B build-core -DBUILD_GUI=OFF
cmake --build build-core -j"$(nproc)"
ctest --test-dir build-core --output-on-failure
```

如果当前账号没有 `sudo` 权限，也可将 Ubuntu 的 Qt6 包解压到项目内（不修改系统）：

```bash
chmod +x scripts/bootstrap_qt6_local.sh scripts/run.sh
./scripts/bootstrap_qt6_local.sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_PREFIX_PATH="$PWD/.deps/qt6/usr"
cmake --build build -j"$(nproc)"
./scripts/run.sh
```

## 运行权限

SOEM 需要打开 raw Ethernet socket。推荐只给编译产物授予所需 capability，不要长期以 root 运行 GUI：

```bash
cd /home/admin1/workspace/EtherCAT_CiA402_Monitor
chmod +x scripts/grant_raw_socket.sh
./scripts/grant_raw_socket.sh
./build/ethercat_cia402_monitor
```

每次重新链接/替换可执行文件后，需要重新执行 capability 脚本。

## 使用说明

1. EtherCAT 专用网卡应直连从站，并避免 NetworkManager/DHCP 同时使用该网卡。
2. 打开程序后选择网卡，点击“扫描并监控”。程序会保持从站在 SOEM 扫描后的 PRE-OP 状态，通过 CoE/SDO 只读标准对象。
3. 若某个值显示 `—` 或“不支持/读取失败”，表示该从站未暴露对应对象、不支持 CoE，或当前状态不允许读取。
4. 多从站时，SDO 读取是顺序执行的。若从站较多，建议将刷新周期设为 1000–2000 ms，避免邮箱通信负载过高。

## 设计边界

该工具是通用诊断监控器，不会猜测厂商私有 PDO 映射，也不会自动进入 OP 或改变电机状态。标准 CiA402 对象可通过 CoE 读取时无需 ESI；但如需确定性高频 PDO 监控，仍应根据驱动器手册配置 PDO 映射和实时周期。
