/*
 * port_host.c -- PC 仿真平台实现
 *
 * 让整份固件在 PC 上以真实时间运行：时间基准、延时、熵源、临界区、
 * 以及一个"虚拟 USART"（把 USART+Wi-Fi 通路的数据接到内存环形缓冲，
 * 便于在没有硬件的情况下验证帧收发时序）。
 */
#include "band_port.h"
#include "band_platform.h"
#include "ring_buffer.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>

#if defined(_WIN32)
#  include <windows.h>
#else
#  include <unistd.h>
#  include <sys/time.h>
#endif

static uint32_t g_start_ms;
static int      g_started;
static band_ring_t g_uart_rx;
static int      g_uart_init;

static uint32_t host_now_ms(void)
{
#if defined(_WIN32)
    return (uint32_t)GetTickCount64();
#else
    struct timespec ts;
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(((uint64_t)ts.tv_sec * 1000u) + ((uint64_t)ts.tv_nsec / 1000000u));
#endif
}

uint32_t band_millis(void)
{
    if (g_started == 0) {
        g_start_ms = host_now_ms();
        g_started = 1;
    }
    return host_now_ms() - g_start_ms;
}

void band_delay_ms(uint32_t ms)
{
#if defined(_WIN32)
    Sleep(ms);
#else
    struct timespec ts;
    ts.tv_sec = (time_t)(ms / 1000u);
    ts.tv_nsec = (long)((ms % 1000u) * 1000000u);
    (void)nanosleep(&ts, NULL);
#endif
}

uint32_t band_entropy(void)
{
    static uint32_t state;
    static int inited;
    uint32_t r;

    if (inited == 0) {
        inited = 1;
        state = (uint32_t)time(NULL);
        state ^= (uint32_t)(uintptr_t)&state;
#if defined(_WIN32)
        state ^= (uint32_t)GetCurrentProcessId();
#else
        state ^= (uint32_t)getpid();
        {
            struct timespec ts;
            (void)clock_gettime(CLOCK_MONOTONIC, &ts);
            state ^= (uint32_t)ts.tv_nsec;
        }
#endif
        if (state == 0u) {
            state = 0x12345678u;
        }
    }
    /* xorshift32 */
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    r = state;
    return r;
}

void band_enter_critical(void)
{
    /* 单线程仿真：无实际并发，保留接口语义 */
}

void band_exit_critical(void)
{
}

void band_trace(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    (void)vfprintf(stdout, fmt, ap);
    va_end(ap);
    (void)fflush(stdout);
}

void band_platform_init(void)
{
    (void)band_millis();
    if (g_uart_init == 0) {
        ring_init(&g_uart_rx);
        g_uart_init = 1;
    }
}

/* ---------------- 虚拟 USART ---------------- */
void band_uart_feed(const uint8_t *data, size_t len)
{
    if (g_uart_init == 0) {
        band_platform_init();
    }
    (void)ring_write(&g_uart_rx, data, len);
}

size_t band_uart_read(uint8_t *out, size_t cap)
{
    if (g_uart_init == 0) {
        return 0u;
    }
    return ring_read(&g_uart_rx, out, cap);
}

uint32_t band_platform_enter_stop(uint32_t max_sleep_ms)
{
    /* PC 上没有真正的 STOP 模式，用短延时模拟，便于观察时序 */
    uint32_t s = (max_sleep_ms > 10u) ? 10u : max_sleep_ms;
    band_delay_ms(s);
    return 0u;
}

void band_platform_wakeup_stable(void)
{
    band_delay_ms(1u);
}

/* ---------------- 虚拟 USART 接口 ---------------- */
static int host_uart_write(const uint8_t *data, size_t len)
{
    if (data == NULL) {
        return -1;
    }
    /* PC 仿真：把"发往模组"的字节直接回灌到接收侧，模拟回环，
     * 便于在没有硬件时观察 AT/帧时序；真实数据通路走 TCP。 */
    return (int)len;
}

static int host_uart_read(uint8_t *buf, size_t cap, uint32_t timeout_ms)
{
    size_t n;

    if (buf == NULL) {
        return -1;
    }
    n = band_uart_read(buf, cap);
    if ((n == 0u) && (timeout_ms > 0u)) {
        band_delay_ms((timeout_ms > 5u) ? 5u : timeout_ms);
        n = band_uart_read(buf, cap);
    }
    return (int)n;
}

static void host_uart_flush(void)
{
    (void)band_uart_read(NULL, 0u);
}

const uart_iface_t *uart_platform_iface(void)
{
    static const uart_iface_t iface = {
        "host-virtual-usart",
        host_uart_write,
        host_uart_read,
        host_uart_flush
    };
    return &iface;
}
