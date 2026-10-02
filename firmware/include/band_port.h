/*
 * band_port.h -- 平台移植层（时间基准 / 延时 / 熵源 / 临界区）
 *
 * 固件核心与驱动层只依赖本头文件，PC 仿真与 STM32 目标各自提供实现：
 *   platform/port_host.c   -> PC (POSIX / MinGW) 仿真实现
 *   platform/port_stm32.c  -> STM32F405 目标实现（寄存器级）
 */
#ifndef BAND_PORT_H
#define BAND_PORT_H

#include <stdint.h>
#include <stddef.h>

/* 系统启动以来的毫秒计数，单调递增，32bit 回绕安全比较用 band_time_after() */
uint32_t band_millis(void);

/* 忙等延时 */
void band_delay_ms(uint32_t ms);

/* 熵源：MCU 上用 RTC 亚秒计数器 + ADC 噪声 + 未初始化 SRAM；
 * PC 上用 clock_gettime + getpid + 地址随机化。
 * 用于 WebSocket 握手随机 Key，无需密码学强度，但必须每次不同。 */
uint32_t band_entropy(void);

/* 临界区：进入后屏蔽调度器抢占（STM32 上同时屏蔽同优先级以下中断） */
void band_enter_critical(void);
void band_exit_critical(void);

/* 32bit 时间比较：a 是否在 b 之后（处理回绕） */
static inline uint32_t band_time_after_impl(uint32_t a, uint32_t b)
{
    return (uint32_t)(a - b) < 0x80000000u;
}
#define band_time_after(a, b) (band_time_after_impl((a), (b)) != 0u)

/* 板上 LED / 调试打印钩子，PC 仿真下直接走 stdout */
void band_trace(const char *fmt, ...);

#endif /* BAND_PORT_H */
