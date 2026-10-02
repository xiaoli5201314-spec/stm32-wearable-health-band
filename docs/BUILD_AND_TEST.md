# 构建与测试

本文所有命令均从**仓库根目录**执行，使用 Linux / WSL 的 shell，不是 PowerShell。构建入口以当前 [Makefile](../firmware/Makefile)为准；结果状态以 [VERIFICATION.md](VERIFICATION.md)为准。

## 1. 环境与依赖

已确认的主机测试环境为 Ubuntu 22.04、GCC 11.4。需要 C99 编译器、GNU Make 和标准 C / POSIX 库；主机程序链接 `libm`。本地 WebSocket 联调另外需要 Python 3 和 POSIX shell，mock 服务端只使用 Python 标准库。

Ubuntu / Debian 可按需安装：

```bash
sudo apt-get update
sudo apt-get install -y build-essential python3
```

默认主机构建使用 `BAND_HOST_SIM=1`和 `_POSIX_C_SOURCE=200809L`，不要求 ARM 工具链或真实器件。Makefile 有 Windows socket 链接分支，但本次未核验原生 MinGW 构建；Windows 用户优先使用 WSL。

## 2. 构建、测试、演示

```bash
make -C firmware all
make -C firmware -B test
./firmware/build/band_demo 9
```

| 命令 / 产物 | 含义 |
| --- | --- |
| `make -C firmware all` | 构建 `firmware/build/band_demo`，不运行演示 |
| `make -C firmware test` | 构建并执行 `firmware/build/run_tests` |
| `make -C firmware -B test` | 强制重建后运行测试，适合记录新一次核验 |
| `./firmware/build/band_demo 9` | 注入模拟器件及合成运动，运行约 9 秒并输出控制台日志 |

演示参数顺序为 `[seconds] [ws_host] [ws_port]`，默认时长 6 秒。参数改变链路配置，但应用没有调用初始 `uplink_connect()`，所以运行演示不能证明连接了服务端。SSD1306 绘制经过模拟总线，不弹出图形窗口；温度 / 电压 / 运动不是实测。

默认编译选项包括 C99、`-O2`、多项警告、`-g`；没有默认开启 ASan / UBSan。不能将普通 `make test`结果写成 sanitizer 通过。

## 3. 测试覆盖与解释

[run_tests.c](../firmware/test/run_tests.c)先运行六类模块测试，再运行应用冒烟；只要断言失败就返回非零。

| 测试 | 覆盖方向 | 不覆盖的结论 |
| --- | --- | --- |
| [帧协议](../firmware/test/test_frame_codec.c) | CRC、编码 / 解码、分段输入、TLV 与载荷 | 真实传输安全性 |
| [计步](../firmware/test/test_step_counter.c) | 合成运动、阈值、不应期及统计 | 真实佩戴者长期准确率 |
| [WebSocket](../firmware/test/test_ws_client.c) | 握手、掩码、控制帧、部分分片与传输桩 | 完整 RFC 兼容性、真实 TCP 端到端成功 |
| [电源管理](../firmware/test/test_power_mgr.c) | 状态、外设回调、顺序与日志 | STOP 电流、时钟恢复、物理唤醒 |
| [驱动](../firmware/test/test_drivers.c) | 模拟总线寄存器、换算、PEC 与部分故障 | 实际器件电气时序及精度 |
| [上行链路](../firmware/test/test_uplink.c) | 发送、缓存、ACK、参数和补传逻辑 | 无损持久交付或 exactly-once |
| 应用冒烟 | 五任务被调度、模拟总线访问、软件休眠及断网缓存 | 已完成真实手环整机验收 |

应用冒烟通过 `band_app_run_steps()`推进虚拟时间，不需要对应时长的现实等待。完整测试数会随代码变化，应从每次测试汇总读取；本次结果见核验记录。

## 4. 真实 TCP / WebSocket 本地联调

这套联调独立于普通 `make test`，使用 [ws_live_client.c](../firmware/test/ws_live_client.c)、[ws_mock_server.py](../tools/ws_mock_server.py)和 [run_ws_e2e.sh](../tools/run_ws_e2e.sh)。它不是 `band_demo`自动连接的证明。

### 一条命令

```bash
make -C firmware live-test
```

默认地址为 `127.0.0.1:9001`，WebSocket 路径为 `/band`。脚本启动服务端：第一条业务连接收到 2 帧后主动断开，第二条业务连接起发送 ACK；客户端尝试检测断线、缓存 8 条记录并重连补传。端口就绪探测连接不计入业务连接编号。

若端口已占用，指定另一个空闲端口：

```bash
make -C firmware live-test WS_PORT=19001
```

脚本默认把日志写到 `${TMPDIR:-/tmp}`，保留服务端日志并删除客户端临时日志。要把日志限制在仓库内，可先创建目录：

```bash
mkdir -p firmware/build/e2e-logs
TMPDIR="$(pwd)/firmware/build/e2e-logs" make -C firmware live-test
```

脚本会查看客户端的 `自检结论: PASS`，并检查服务端摘要中 `duplicates`、`order_violations`、`crc_errors`均为 0。**2026-10-02 实际执行 `make -C firmware live-test` 已通过**，8 / 8 条缓存记录完成补传、最终待补传为 0；完整执行结果和 ACK 统计解释见 [VERIFICATION.md](VERIFICATION.md)。自定义端口或 BUILD 目录的示例命令不属于本条已执行记录。

### 双终端手动运行

终端 A：

```bash
python3 tools/ws_mock_server.py --host 127.0.0.1 --port 9001 --drop-after 2 --ack-from-conn 2 --timeout 25
```

终端 B：

```bash
make -C firmware build/ws_live_client
./firmware/build/ws_live_client 127.0.0.1 9001 -v
```

客户端详细日志可用于检查发送序号、ACK 序号及 pending 数。若换端口，服务端与客户端必须一致。人工观察连接成功不等于所有自检成立，应保留双方结果及退出状态。

本次通过只证明这组 loopback TCP / mock 交互。它不覆盖 Wi-Fi AT 模组、手机 App、公网服务器、TLS、服务端数据库、所有 ACK 丢失 / 乱序情形或长期稳定性。

## 5. STM32 代码检查

```bash
make -C firmware target-syntax
```

该目标用主机 C 编译器的 `-fsyntax-only`检查：

- `platform/port_stm32.c`
- `platform/bus_stm32.c`
- `platform/net_wifi_at.c`
- `platform/rtc_iwdg_port.c`

**2026-10-02 实际执行结果为 PASS**，4 个文件均通过 GCC `-fsyntax-only`检查。不执行寄存器访问、不覆盖完整应用链接、不生成 ARM ELF / HEX，不烧录、不测试硬件，也不能发现所有引脚冲突或运行时问题。执行环境与结果见 [VERIFICATION.md](VERIFICATION.md)。

不要把 `arm-check`当作完整交叉构建：当前目标使用包含主机平台的 `ALL_SRC`列表；缺少工具链时，第一行的“跳过”不会阻止独立下一行继续调用编译器。[platformio.ini](../firmware/platformio.ini)也尚未形成经验证的完整目标入口。目标集成缺口见 [DESIGN.md](DESIGN.md)。

## 6. 独立产物目录与工具注意事项

`BUILD`相对于 `firmware/`，可以避免复用既有产物：

```bash
make -C firmware BUILD=build/docs-check -B all test
./firmware/build/docs-check/band_demo 9
make -C firmware BUILD=build/docs-check live-test WS_PORT=19001
```

更换编译选项时建议用独立 BUILD 目录或 `-B`，避免误用未重建的可执行文件。联调目标会按 BUILD 自动定位客户端。

字库已随源码提供，正常构建不需要重新生成。当前 `make -C firmware fonts`中的脚本相对路径指向错误位置；实际工具应从仓库根目录调用：

```bash
python3 tools/gen_oled_font.py
```

**这会改写 `firmware/drivers/oled_font.c`，不是只读检查；本次文档核验不执行它。** 文档指出现有工具限制，不在本次范围内修复 Makefile、脚本或固件代码。

## 7. CI 与核验记录

[ci.yml](../.github/workflows/ci.yml)在 `push`、`pull_request`和 `workflow_dispatch`触发 Ubuntu 22.04 主机构建、测试及指定平台文件语法检查；使用 `actions/checkout@v4`、只读 `contents`权限，单 job 超时 10 分钟。

CI 不执行真实 TCP 联调、sanitizer、ARM 链接、烧录或硬件测量。配置文件的存在不表示远程 Actions 已运行或通过。

每次对外展示结果，记录日期、工作区 / commit 状态、工具链、完整命令、退出码、用例汇总和未覆盖项。可使用 [VERIFICATION.md](VERIFICATION.md)记录，不沿用旧 README 中未经本次复核的测量或通过声明。
