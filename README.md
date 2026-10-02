# STM32 可穿戴健康手环固件原型

[![Host Build and Tests](https://github.com/xiaoli5201314-spec/stm32-wearable-health-band/actions/workflows/ci.yml/badge.svg)](https://github.com/xiaoli5201314-spec/stm32-wearable-health-band/actions/workflows/ci.yml)

`stm32-wearable-health-band` 是面向 STM32F405 的 **C99 嵌入式软件原型**：围绕运动采集、触摸交互、单色显示、数据通信和电源管理组织固件，并通过 Linux 主机仿真验证驱动与业务逻辑。

**提供可复现的主机演示、模块测试和本地 WebSocket 联调。** 同时保留 STM32 寄存器级适配代码，便于沿接口逐步完成板级移植；目标固件和真实器件的验收范围见下方实现状态。

## 招聘阅读入口

适合查看的能力方向：嵌入式 C、驱动接口抽象、状态机设计、通信协议、异常路径处理和自动化测试。以下亮点均可沿链接核对源码。

| 项目亮点 | 可以核对的实现 |
| --- | --- |
| 分层组织固件 | [应用入口](firmware/src/app_main.c)组织采集、输入、界面、通信、电源五个任务；[传感器接口](firmware/include/sensor_iface.h)隔离总线实现 |
| 自定义协作式调度 | [调度器](firmware/src/task_sched.c)提供优先级、时间片记账、周期唤醒、信号量和定长邮箱；不是 uCOS-II / FreeRTOS |
| 多外设驱动与错误处理 | [驱动目录](firmware/drivers/)包含 MPU6050、FT6236、MLX90615、SSD1306、电池、RTC、IWDG；[测试](firmware/test/test_drivers.c)覆盖模拟总线、数据转换及部分故障路径 |
| 分层通信与断网缓存 | [帧编解码](firmware/src/frame_codec.c)、[WebSocket 客户端](firmware/src/ws_client.c)、[上行链路](firmware/src/uplink.c)和[离线缓存](firmware/src/ring_buffer.c)分别负责边界、传输、分发和补传 |
| 可观察的电源管理 | [功耗状态机](firmware/src/power_mgr.c)集中管理外设生命周期，记录事件日志；[测试](firmware/test/test_power_mgr.c)核对调用顺序 |
| 可复现的主机测试 | [测试入口](firmware/test/run_tests.c)汇总模块测试及应用冒烟测试；[CI 配置](.github/workflows/ci.yml)在 Ubuntu 上构建并运行测试 |

**2026-10-02 核验记录：** 在 Ubuntu 22.04、GCC 11.4 环境执行 `make -C firmware -B test`，得到 **180 个用例、1019 条断言、0 失败**。独立执行的 `live-test` 真实本地 TCP / WebSocket 场景通过：2 次业务连接、10 个唯一序号、8 / 8 条缓存记录补传、最终待补传 0 条，服务端重复 / 乱序 / CRC 错误均为 0；`target-syntax` 的 4 个平台 C 文件语法检查也通过。完整记录见 [VERIFICATION.md](docs/VERIFICATION.md)。这些结果不是 ARM 链接、烧录、真实硬件或医学有效性验证。

## 实现状态

“有实现”表示能在源码中找到相应逻辑；“主机验证”表示测试使用模拟器件或传输桩；两者都不等于真实 STM32 硬件验收。

| 能力 | 已实现的软件 | 主机仿真 / 测试 | STM32 集成边界 |
| --- | --- | --- | --- |
| 五任务调度与同步 | 协作式调度、信号量、16 字节邮箱 | 应用冒烟测试运行五个任务 | 需要目标入口、中断及 tick 接入；无抢占保证 |
| 运动与姿态 | MPU6050 原始采集、计步、CPU 四元数融合 | 模拟寄存器、合成运动及 FIFO 包测试 | 真实佩戴与误差未核验；未见 DMP 固件下载流程，不能宣称板上 DMP 已运行 |
| 触摸与显示 | FT6236 手势、SSD1306 128×64 显存、字库及页刷新 | 模拟 I2C 访问；演示只输出控制台日志 | 真实屏幕及触摸未核验；不是 STemWin / TFT 界面 |
| 温度与电量 | SMBus PEC、红外温度转换、ADC / VREFINT / OCV 与电量积分 | 注入温度、ADC 数值并断言 | 对象温度不是已校准核心体温；电流由应用估算，非实测库仑计 |
| 通信与远程参数 | 应用帧、WebSocket 子集、ACK、TLV、易失离线缓存 | 协议与链路桩测试；本地真实 TCP 补传场景及 1 次 TLV 更新通过 | Wi-Fi AT 适配仍需接入；应用未发起初始连接；无手机 / 云端闭环证据 |
| 电源与可靠性 | RUN / IDLE / SLEEP / WAKEUP、外设回调、RTC / IWDG 接口 | 状态、日志、回调顺序及监督喂狗测试 | SLEEP 未接入完整 STOP 流程；唤醒、时钟恢复、电流和续航未实测 |

## 系统架构

```mermaid
flowchart TB
    SCHED["自定义协作式调度器"] --> SENSOR["SENSOR 20 ms"]
    SCHED --> KEY["KEY 10 ms"]
    SCHED --> UI["UI 50 ms"]
    SCHED --> COMM["COMM 20 ms"]
    SCHED --> POWER["POWER 1000 ms"]

    SENSOR --> DRV["MPU6050 / MLX90615 驱动"]
    SENSOR --> ALGO["计步 / CPU 姿态融合"]
    SENSOR --> SNAP["共享快照 g_app.snap"]
    SENSOR --> DATA["data_mbox 通知"]
    KEY --> TOUCH["FT6236 / GPIO"]
    KEY --> INPUT["cmd_mbox / touch_sem"]
    INPUT --> UI
    DATA --> UI
    SNAP --> UI
    SNAP --> COMM
    UI --> OLED["SSD1306 显存与页刷新"]
    COMM --> UPLINK["uplink / 应用帧 / TLV"]
    UPLINK --> CACHE["RAM 离线缓存 / ACK"]
    UPLINK --> WS["WebSocket 客户端"]
    WS --> NET["net_transport"]
    POWER --> PM["功耗状态机 / 外设生命周期"]
    POWER --> AUX["电池 / RTC / IWDG"]

    DRV --> BUS["sensor_iface 总线接口"]
    TOUCH --> BUS
    OLED --> BUS
    BUS --> SIM["主机模拟 I2C / ADC / GPIO"]
    NET --> SOCKET["主机 TCP socket"]
    BUS -.待硬件核验.-> STM["STM32 寄存器级适配"]
    NET -.待应用接入.-> AT["USART / Wi-Fi AT 适配"]
```

`data_mbox` 不是完整快照广播通道：SENSOR 更新共享快照并投递通知，UI 清空通知队列，COMM 直接读取共享快照。图中的 STM32 / AT 分支表示适配代码存在，不表示整机已连通。

## 核心模块

### 调度与应用

[task_sched.c](firmware/src/task_sched.c)把任务写成反复调用的函数，任务通过 `return` 让出执行权；[app_main.c](firmware/src/app_main.c)保存任务上下文并编排五个业务任务。

时间片只影响调度决策和记账，**不会中断一个尚未返回的函数**。阻塞 I/O 会延迟其他任务；所有任务共享调用栈，也仍然存在栈溢出和中断并发风险。邮箱结构体带编译期长度断言，避免与 16 字节消息契约不一致。详见 [设计说明](docs/DESIGN.md)。

### 采集、算法与显示

[mpu6050.c](firmware/drivers/mpu6050.c)负责六轴采集及 CPU 姿态融合，[step_counter.c](firmware/src/step_counter.c)实现滤波、迟滞阈值、不应期和步频统计；[mlx90615.c](firmware/drivers/mlx90615.c)处理红外读数与 PEC。

界面使用 [oled_ssd1306.c](firmware/drivers/oled_ssd1306.c)的单色显存和[点阵字库](firmware/drivers/oled_font.c)，包含数据、温度、电池、网络及菜单页面。主机演示运行绘制和模拟写屏逻辑，**不提供可见 OLED 图形窗口**。

### 通信与缓存

[frame_codec.c](firmware/src/frame_codec.c)实现小端应用帧和 CRC16，[ws_client.c](firmware/src/ws_client.c)实现握手、掩码、控制帧和部分分片处理，[uplink.c](firmware/src/uplink.c)串联发送、参数分发和补传。

离线记录保留原始帧与序号，每轮最多补传 8 条，ACK 按载荷中的序号移除记录。缓存满时覆盖最旧记录、重启即丢失；在线成功发送不保留到 ACK，且没有同会话 ACK 超时重试，因此不能声称“不丢不重”或 exactly-once。字节布局和边界见 [协议说明](docs/PROTOCOLS.md)。

### 电源与任务监督

[power_mgr.c](firmware/src/power_mgr.c)规定休眠与唤醒的外设回调顺序，维护 128 条事件日志；[iwdg.c](firmware/drivers/iwdg.c)记录任务心跳并做超时监督。

这些测试证明的是软件状态转换及回调执行。当前 IDLE 不实际降频，部分外设关闭回调仅修改标志，SLEEP 未暂停其余业务任务或调用完整 STOP 流程。硬件电源管理还需独立完成和测量。

## 目录导航

| 路径 | 阅读内容 |
| --- | --- |
| [firmware/src/](firmware/src/) | 应用、调度、帧协议、WebSocket、缓存、计步、电源管理 |
| [firmware/drivers/](firmware/drivers/) | 器件驱动、显示字库和数据换算 |
| [firmware/include/](firmware/include/) | 接口契约、数据结构、协议常量和编译期配置 |
| [firmware/platform/](firmware/platform/) | 主机模拟总线 / socket 与 STM32 寄存器 / AT 适配 |
| [firmware/test/](firmware/test/) | 模块测试、应用冒烟测试、真实 TCP 联调客户端 |
| [firmware/Makefile](firmware/Makefile) | 主机构建、测试和目标代码语法检查入口 |
| [tools/](tools/) | Python mock 服务端、联调脚本、字库生成脚本 |
| [docs/DESIGN.md](docs/DESIGN.md) | 调度语义、同步对象、电源生命周期及集成缺口 |
| [docs/BUILD_AND_TEST.md](docs/BUILD_AND_TEST.md) | Linux / WSL 快速运行与分层测试方法 |
| [docs/PROTOCOLS.md](docs/PROTOCOLS.md) | 应用帧、ACK、TLV、WebSocket 的实际行为 |
| [docs/VERIFICATION.md](docs/VERIFICATION.md) | 本次核验日期、命令、工具链与未验证项 |
| [.github/workflows/ci.yml](.github/workflows/ci.yml) | Ubuntu 主机构建与测试自动化 |

## 快速运行

在 Linux 或 WSL 中，安装 C 编译器、GNU Make、Python 3 后，从仓库根目录执行：

```bash
make -C firmware all
make -C firmware -B test
./firmware/build/band_demo 9
```

测试入口包含帧协议、计步、WebSocket、功耗、驱动、上行链路和应用冒烟测试。演示注入合成传感器数据并输出运行统计；它未调用 `uplink_connect()`，不能作为在线 WebSocket 展示。

以下两项已于 **2026-10-02 独立执行并通过**，可用相同命令复现：

```bash
make -C firmware target-syntax
make -C firmware live-test
```

`target-syntax` 使用主机编译器检查指定 STM32 平台文件，不进行 ARM 链接或烧录。`live-test` 使用本地 mock 服务端和专用 C 客户端测试实际 TCP；需要空闲端口和 Python 3。结果只覆盖本次记录的场景，不保证所有网络故障下无损补传。构建产物、自定义端口、手动双终端联调及工具限制见 [BUILD_AND_TEST.md](docs/BUILD_AND_TEST.md)。

## 使用边界

- **健康数据不是诊断结果。** 无 PPG 驱动；界面中的“心率”来自运动峰值 / 步频逻辑复用，可能受步态影响。红外对象温度不能直接等同于核心体温。本项目没有健康诊断认证或临床有效性证据。
- **不主张已完成硬件设计。** 仓库中未见可核对的原理图、PCB 工程或真实硬件测量记录；配置中的 MCU、引脚、电池容量是软件目标参数，不是已制板验收的证明。
- **DMP 不等于已部署。** 存在 FIFO / 四元数包解析接口，但未见厂商 DMP 固件镜像及加载过程；当前应用采用原始数据加 CPU 融合。
- **目标固件仍需集成。** 启动文件、链接脚本、目标 `main`、中断 / tick 接入、网络绑定及 STOP 唤醒闭环尚需补齐；USART 引脚定义与实现还存在冲突，详见设计说明。
- **通信原型不具备生产安全性。** 未实现 TLS / `wss`、身份认证或持久业务去重；CRC 和 WebSocket 掩码不是加密或授权机制，参数下发只适合可信测试环境。
- **测试结论必须分层。** 主机测试和本地 TCP 联调通过不能推导真实 I2C 时序、屏幕效果、射频稳定性、功耗、续航或长期佩戴精度。未单独运行的 sanitizer、其他联调场景及硬件项目不记为通过。
