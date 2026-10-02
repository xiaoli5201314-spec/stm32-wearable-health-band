# 核验记录

执行日期：**2026-10-02**。本文件分别记录已实际完成的主机测试、本地 TCP / WebSocket 联调和 STM32 平台文件语法检查；每项结论只覆盖对应验证层级。

## 已确认的主机测试

在 Ubuntu 22.04、GCC 11.4 环境重新构建并执行测试，结果如下。

| 项目 | 记录 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| 编译器 | GCC 11.4 |
| 编译入口 | 当前 [firmware/Makefile](../firmware/Makefile) |
| 工作目录 | 仓库根目录 |
| 命令 | `make -C firmware -B test` |
| 结果 | PASS，180 个用例、1019 条断言、0 失败 |
| 代码状态 | 当前未提交工作区，存在既有源码修改；未绑定已发布 commit |
| 验证层级 | C99 主机仿真、模块测试与应用冒烟；非真实 STM32 整机 |

```bash
make -C firmware -B test
```

测试入口为 [run_tests.c](../firmware/test/run_tests.c)。它执行帧协议、计步、WebSocket、功耗、驱动、上行链路及应用冒烟；断言失败时返回非零。本次普通主机测试 PASS 不代表 sanitizer 通过，也不提供执行耗时或覆盖率结论。

## 本地 TCP / WebSocket 联调

2026-10-02 实际执行：

```bash
make -C firmware live-test
```

结果：**PASS**。使用本地 Python mock 服务端和专用 C 客户端，在 localhost 上建立真实 TCP / WebSocket 连接，不是传输桩测试。

| 项目 | 执行结果 |
| --- | --- |
| 业务连接 | 2 次；端口就绪探测连接不计入业务连接数 |
| 唯一应用序号 | 10 个 |
| 数据分布 | 2 条在线发送、8 条断网期间进入缓存 |
| 补传 | 8 / 8 条已补传 |
| 最终待补传记录 | 0 条 |
| 服务端重复序号 | 0 |
| 服务端乱序 | 0 |
| 服务端 CRC 错误 | 0 |
| 客户端收到 ACK | 9 条 |
| 未命中缓存的在线 ACK | 1 条；在线发送成功的记录不保留在缓存中 |
| 远程 TLV 参数更新 | 成功应用 1 次 |

客户端将未命中缓存的 ACK 计入 `dup_acks`，因此本次该计数为 1；它对应未跟踪的在线 ACK，**不表示服务端收到了重复数据**，也不与服务端重复序号为 0 的结果冲突。

本次通过证明这组本地断线、缓存、重连、补传及参数更新场景成立，不保证任意 ACK 丢失 / 乱序下的数据完整性，也不证明所有参数已接入运行时。它不覆盖 Wi-Fi AT 模组、手机 App、公网、TLS 或业务数据库。

## STM32 平台文件语法检查

2026-10-02 在 Ubuntu 22.04、GCC 11.4 环境实际执行：

```bash
make -C firmware target-syntax
```

结果：**PASS**。GCC 使用 `-fsyntax-only`检查了以下 4 个平台 C 文件：

- [port_stm32.c](../firmware/platform/port_stm32.c)
- [bus_stm32.c](../firmware/platform/bus_stm32.c)
- [net_wifi_at.c](../firmware/platform/net_wifi_at.c)
- [rtc_iwdg_port.c](../firmware/platform/rtc_iwdg_port.c)

该检查不执行寄存器访问，不进行 ARM 固件链接、生成可烧录镜像或烧录，也不证明真实板卡功能。引脚冲突、目标入口、调度 tick、STOP 唤醒等集成缺口仍见 [DESIGN.md](DESIGN.md)。

## 待补充的核验

下表区分已经核对的远端构建和仍待补录的项目；远端记录见下节。
后续执行其他命令时，应补记日期、环境、退出码与实际输出，不仅修改状态标签。

| 项目 | 命令 | 当前记录状态 |
| --- | --- | --- |
| 主机演示构建 | `make -C firmware -B all` | 2026-10-03 远端 CI 构建通过 |
| 控制台演示 | `./firmware/build/band_demo 9` | 待补录；传感器仍是合成数据 |
| GitHub Actions | [.github/workflows/ci.yml](../.github/workflows/ci.yml) | 2026-10-03 已核对远端成功，见下节 |
| ASan / UBSan | 需单独构建与运行并记录命令 | 未在本条记录核验 |
| ARM 固件链接与烧录 | 需补齐目标工程及工具链 | 未核验 |
| 真实硬件测量 | 需板卡、仪器与可追溯记录 | 未核验 |

联调脚本和手动双终端复现命令见 [BUILD_AND_TEST.md](BUILD_AND_TEST.md)。上述已通过项目与本表待补充项目独立记录，不将本地测试结果扩大为整机或生产环境验收。

## 发布前复验

发布前于 **2026-10-03** 再次执行 `make -C firmware -B test`、`make -C firmware live-test` 和 `make -C firmware target-syntax`，三条命令均以退出码 0 完成。单元测试仍为 **180 个用例、1019 条断言、0 失败**；本地 TCP 补传场景仍为 **8 / 8 条补传、0 条待补传、0 重复 / 乱序 / CRC 错误**；四个目标平台文件的主机语法检查通过。

## 远端自动测试

2026-10-03 核对发布提交 `93740ab` 的 [GitHub Actions 记录](https://github.com/xiaoli5201314-spec/stm32-wearable-health-band/actions/runs/37053451067)，状态为 `completed / success`。
工作流在 Ubuntu 22.04 构建主机演示、运行单元测试并检查 STM32 平台文件语法。
`live-test` 仍为单独记录的本地 TCP 联调；后续提交的实时状态以首页徽章和对应 Actions 记录为准。

## 硬件与健康边界

- 未见可核对的原理图、PCB 项目、目标烧录记录或整机验收记录；本文件不认定自制板已完成。
- 主机模拟 I2C / ADC / GPIO 不证明真实电气时序、传感器误差、射频稳定性或长期运行可靠性。
- 软件功耗状态 / 回调顺序不证明已进入 STOP，不提供电流、续航、唤醒延迟、WCET 或栈水位测量。
- 当前应用使用 CPU 姿态融合；DMP 包接口 / 合成 FIFO 测试不证明已加载厂商 DMP 固件。
- 无 PPG 驱动；运动峰值推导的“心率”和红外对象温度没有医学有效性或健康诊断认证。
- WebSocket 实现没有 TLS / `wss`和设备认证，RAM 缓存与 ACK 行为不提供无损持久交付或 exactly-once 保证。

集成缺口见 [DESIGN.md](DESIGN.md)，协议边界见 [PROTOCOLS.md](PROTOCOLS.md)。代码变更后应重新执行相关命令并更新记录，不能把本次测试数字永久当作所有版本的结果。
