/*
 * band_platform.h -- 平台层工厂函数
 *
 * STM32 目标与 PC 仿真各自提供一份实现，编译期通过 BAND_HOST_SIM 选择：
 *   -DBAND_HOST_SIM=1  -> platform/port_host.c, bus_sim.c, net_socket.c, rtc/iwdg host
 *   默认               -> platform/port_stm32.c, bus_stm32.c, net_wifi_at.c, rtc/iwdg stm32
 */
#ifndef BAND_PLATFORM_H
#define BAND_PLATFORM_H

#include <stdint.h>
#include "sensor_iface.h"
#include "net_transport.h"
#include "rtc.h"
#include "iwdg.h"
#include "band_port.h"

/* RTC / IWDG 寄存器访问接口（目标板走 MMIO，PC 走内存寄存器块） */
const rtc_iface_t  *rtc_platform_iface(void);
const iwdg_iface_t *iwdg_platform_iface(void);

/* USART 字节流接口：Wi-Fi 模组的 AT 指令通道 */
typedef struct {
    const char *name;
    int (*write)(const uint8_t *data, size_t len);
    int (*read)(uint8_t *buf, size_t cap, uint32_t timeout_ms);
    void (*flush)(void);
} uart_iface_t;

const uart_iface_t *uart_platform_iface(void);

#if defined(BAND_HOST_SIM)
/* PC 仿真辅助：模拟一次"看门狗超时复位"，用于验证 IWDG 任务监督的现场保留 */
void rtc_iwdg_sim_trigger_reset(uint32_t fault_task);
#endif

/* PC 仿真：把仿真 usart 的收发接到 TCP 中继（用于 USART+Wi-Fi 通路验证） */
void band_platform_init(void);

/* 帧协议在 USART 上的收包缓冲（目标板由 DMA+空闲中断填充） */
void     band_uart_feed(const uint8_t *data, size_t len);
size_t   band_uart_read(uint8_t *out, size_t cap);

/* 目标板低功耗：进入 STOP 模式并等待唤醒源；返回唤醒源编码 */
uint32_t band_platform_enter_stop(uint32_t max_sleep_ms);
void     band_platform_wakeup_stable(void);

#endif /* BAND_PLATFORM_H */
