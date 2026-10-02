# =====================================================================
#  stm32-wearable-health-band 固件 Makefile
#
#  默认目标在 PC 上编译并运行（传感器走 sensor_iface 仿真桩）：
#     make            # 构建主机端演示程序 build/band_demo
#     make test       # 构建并运行全部单元测试（0 失败才算通过）
#     make live-test  # 启动 tools/ws_mock_server.py，跑真实 TCP 的 WebSocket 联调
#     make fonts      # 重新生成 SSD1306 5x7 点阵字库
#     make target-syntax  # 用 -fsyntax-only 校验 STM32 目标代码（寄存器级实现）
#     make clean
# =====================================================================

CC       ?= gcc
CSTD     ?= -std=c99
OPT      ?= -O2
WARN     ?= -Wall -Wextra -Wpedantic -Wshadow -Wstrict-prototypes -Wmissing-prototypes
BUILD    ?= build
PYTHON   ?= python3
WS_PORT  ?= 9001

CPPFLAGS += -DBAND_HOST_SIM=1 -D_POSIX_C_SOURCE=200809L -Iinclude
CFLAGS   += $(CSTD) $(OPT) $(WARN) -g
LDFLAGS  +=
LDLIBS   += -lm

# Windows(MinGW) 下 socket 需要额外链接 ws2_32
ifeq ($(OS),Windows_NT)
  LDLIBS += -lws2_32
endif

SRC_CORE    := src/ring_buffer.c src/frame_codec.c src/ws_client.c src/power_mgr.c \
               src/task_sched.c src/step_counter.c src/uplink.c src/app_main.c
SRC_DRIVERS := drivers/mpu6050.c drivers/ft6236.c drivers/mlx90615.c \
               drivers/oled_ssd1306.c drivers/oled_font.c drivers/battery.c \
               drivers/rtc.c drivers/iwdg.c
SRC_HOST    := platform/port_host.c platform/bus_sim.c platform/net_socket.c \
               platform/net_wifi_at.c platform/rtc_iwdg_port.c
SRC_TESTS   := test/test_util.c test/test_frame_codec.c test/test_step_counter.c \
               test/test_ws_client.c test/test_power_mgr.c test/test_drivers.c \
               test/test_uplink.c test/run_tests.c

ALL_SRC := $(SRC_CORE) $(SRC_DRIVERS) $(SRC_HOST)
HDRS    := $(wildcard include/*.h) $(wildcard platform/*.h)

DEMO_BIN := $(BUILD)/band_demo
TEST_BIN := $(BUILD)/run_tests
LIVE_BIN := $(BUILD)/ws_live_client

.PHONY: all test live-test fonts target-syntax clean help

all: $(DEMO_BIN)

$(BUILD):
	@mkdir -p $(BUILD)

$(DEMO_BIN): $(ALL_SRC) platform/main_host.c $(HDRS) | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(ALL_SRC) platform/main_host.c -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_BIN): $(ALL_SRC) $(SRC_TESTS) $(HDRS) | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(ALL_SRC) $(SRC_TESTS) -o $@ $(LDFLAGS) $(LDLIBS)

$(LIVE_BIN): $(ALL_SRC) test/ws_live_client.c $(HDRS) | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(ALL_SRC) test/ws_live_client.c -o $@ $(LDFLAGS) $(LDLIBS)

# ---------------- 单元测试 ----------------
test: $(TEST_BIN)
	@echo "--- 运行单元测试 ---"
	@./$(TEST_BIN)
	@echo "--- 单元测试返回码: $$? ---"

# ---------------- 真实 TCP 联调（WebSocket + 补传） ----------------
live-test: $(LIVE_BIN)
	@sh ../tools/run_ws_e2e.sh $(abspath $(LIVE_BIN)) $(WS_PORT)

# ---------------- 运行主机端演示 ----------------
demo: $(DEMO_BIN)
	@./$(DEMO_BIN)

# ---------------- 重新生成字库 ----------------
fonts:
	$(PYTHON) tools/gen_oled_font.py

# ---------------- 目标代码语法校验（无 ARM 工具链也能做基本检查） ----------------
target-syntax:
	@echo "--- 用 -fsyntax-only 校验 STM32 目标实现（寄存器级代码） ---"
	@for f in platform/port_stm32.c platform/bus_stm32.c platform/net_wifi_at.c platform/rtc_iwdg_port.c; do \
	    echo "  [check] $$f"; \
	    $(CC) -std=c99 -Wall -Wextra -Iinclude -Iplatform -fsyntax-only $$f || exit 1; \
	done
	@echo "--- 目标代码语法校验通过 ---"

# ---------------- 交叉编译（可选，需要 arm-none-eabi-gcc） ----------------
CROSS ?= arm-none-eabi-gcc
arm-check:
	@command -v $(CROSS) >/dev/null 2>&1 || { echo "未找到 $(CROSS)，跳过（属正常，PC 测试不依赖它）"; exit 0; }
	$(CROSS) -std=c99 -mcpu=cortex-m4 -mthumb -Iinclude -Iplatform \
	    -fsyntax-only $(ALL_SRC) && echo "ARM 交叉语法校验通过"

clean:
	rm -rf $(BUILD)

help:
	@echo "可用目标: all test live-test demo fonts target-syntax arm-check clean"
