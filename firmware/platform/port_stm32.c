/*
 * port_stm32.c -- STM32F405 平台实现（寄存器级，不依赖 HAL/CMSIS）
 *
 * 提供：系统时钟树配置、毫秒基准（TIM2）、延时、熵源、临界区、
 *       USART1（Wi-Fi 模组）收发、STOP 低功耗进入/退出。
 */
#ifndef BAND_HOST_SIM

#include "band_port.h"
#include "band_platform.h"
#include "power_mgr.h"
#include "ring_buffer.h"
#include "stm32f405_regs.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static volatile uint32_t g_tick_ms;
static band_ring_t g_uart_rx;
static int g_uart_init;

/* ---------------- 时钟树：HSE 8MHz -> PLL -> 168MHz ---------------- */
/* PLLCFGR: M=8 -> 1MHz VCO 输入; N=336 -> 336MHz; P=2 -> 168MHz; Q=7 -> 48MHz */
#define PLL_M 8u
#define PLL_N 336u
#define PLL_P 2u
#define PLL_Q 7u

void board_clock_init(void)
{
    /* 1) 使能 HSE 并等待稳定 */
    REG32(RCC_CR) |= RCC_CR_HSEON;
    while ((REG32(RCC_CR) & RCC_CR_HSERDY) == 0u) {
    }

    /* 2) 配置 Flash 等待周期：168MHz / 3.3V -> 5 WS，并使能 I/D cache 与预取 */
    REG32(FLASH_ACR) = 0x00000005u | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;

    /* 3) 配置 PLL：VCO = HSE/M*N = 8/8*336 = 336MHz, SYSCLK = 336/2 = 168MHz */
    REG32(RCC_PLLCFGR) = (PLL_M << 0) | (PLL_N << 6) | (((PLL_P >> 1u) - 1u) << 16) |
                         (PLL_Q << 24) | (1u << 22);   /* bit22: PLLSRC=HSE */

    /* 4) AHB=/1 (168MHz), APB1=/4 (42MHz), APB2=/2 (84MHz) */
    {
        uint32_t cfgr = REG32(RCC_CFGR);
        cfgr &= ~0x0000FCF0u;                 /* 清 HPRE/PPRE1/PPRE2 */
        cfgr |= (0u << 4) | (5u << 10) | (4u << 13);
        REG32(RCC_CFGR) = cfgr;
    }

    /* 5) 打开 PLL 并等待锁定 */
    REG32(RCC_CR) |= RCC_CR_PLLON;
    while ((REG32(RCC_CR) & RCC_CR_PLLRDY) == 0u) {
    }

    /* 6) 切换 SYSCLK 到 PLL */
    {
        uint32_t cfgr = REG32(RCC_CFGR);
        cfgr = (cfgr & ~0x00000003u) | RCC_CFGR_SW_PLL;
        REG32(RCC_CFGR) = cfgr;
        while ((REG32(RCC_CFGR) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL) {
        }
    }

    /* 7) 打开 GPIO / I2C1 / USART1 / PWR / ADC1 时钟 */
    REG32(RCC_AHB1ENR) |= RCC_AHB1EN_GPIOA | RCC_AHB1EN_GPIOB | RCC_AHB1EN_GPIOC;
    REG32(RCC_APB1ENR) |= RCC_APB1EN_I2C1 | RCC_APB1EN_PWR | RCC_APB1EN_USART2;
    REG32(RCC_APB2ENR) |= RCC_APB2EN_USART1 | RCC_APB2EN_ADC1;

    /* 8) 打开备份域写保护，为 RTC 使用 LSE 做准备 */
    REG32(PWR_CR) |= 0x00000100u;   /* DBP */
}

/* ---------------- 毫秒基准 ---------------- */
/* TIM2 挂 APB1(42MHz)，APB1 分频≠1 时定时器时钟 = 84MHz。
 * PSC=8399 -> 10kHz；ARR=9 -> 1kHz 更新中断。 */
void board_tick_init(void)
{
    REG32(RCC_APB1ENR) |= RCC_APB1EN_TIM2;
    REG32(TIM2_PSC) = 8399u;
    REG32(TIM2_ARR) = 9u;
    REG32(TIM2_DIER) |= TIM2_DIER_UIE;
    REG32(TIM2_SR) = 0u;
    REG32(TIM2_CR1) |= TIM2_CR1_CEN;
}

/* 在 TIM2 中断里调用（真实工程由 startup 向量表挂到 TIM2_IRQHandler） */
void board_tick_isr(void)
{
    if ((REG32(TIM2_SR) & TIM2_SR_UIF) != 0u) {
        REG32(TIM2_SR) = 0u;
        g_tick_ms++;
    }
}

uint32_t band_millis(void)
{
    return g_tick_ms;
}

void band_delay_ms(uint32_t ms)
{
    uint32_t start = band_millis();
    while ((uint32_t)(band_millis() - start) < ms) {
        cpu_wfi();
    }
}

uint32_t band_entropy(void)
{
    static uint32_t s;
    uint32_t r;

    if (s == 0u) {
        /* 熵源：RTC 亚秒计数器 + 芯片唯一 ID + 毫秒计数的散列 */
        s = REG32(PERIPH_RTC_BASE + 0x28u);          /* RTC_SSR 亚秒计数 */
        s ^= REG32(0x1FFF7A10u);                     /* UID[0]，每颗芯片不同 */
        s ^= REG32(0x1FFF7A14u) << 7;                /* UID[1] */
        s ^= g_tick_ms * 2654435761u;
        if (s == 0u) {
            s = 0xA5A5A5A5u;
        }
    }
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    r = s;
    return r;
}

void band_enter_critical(void)
{
    /* 关中断：调度器是协作式的，临界区只保护共享数据结构的短暂修改 */
    __asm volatile ("cpsid i" ::: "memory");
}

void band_exit_critical(void)
{
    __asm volatile ("cpsie i" ::: "memory");
}

void band_trace(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    int n;
    int i;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return;
    }
    for (i = 0; i < n; i++) {
        while ((REG32(USART_SR) & USART_SR_TXE) == 0u) {
        }
        REG32(USART_DR) = (uint32_t)(uint8_t)buf[i];
    }
}

/* ---------------- USART1 <-> Wi-Fi 模组 ---------------- */
void board_usart1_init(uint32_t baud)
{
    /* PB6/PB7 复用为 USART1_TX/RX */
    {
        uint32_t moder = REG32(GPIO_MODER(PERIPH_GPIOB_BASE));
        moder &= ~((3u << 12) | (3u << 14));
        moder |= (GPIO_MODE_AF << 12) | (GPIO_MODE_AF << 14);
        REG32(GPIO_MODER(PERIPH_GPIOB_BASE)) = moder;
        REG32(GPIO_OSPEEDR(PERIPH_GPIOB_BASE)) |= (GPIO_OSPEED_VHIGH << 12) | (GPIO_OSPEED_VHIGH << 14);
        REG32(GPIO_PUPDR(PERIPH_GPIOB_BASE)) |= (GPIO_PUPD_PULLUP << 12);
        REG32(GPIO_AFRL(PERIPH_GPIOB_BASE)) = (REG32(GPIO_AFRL(PERIPH_GPIOB_BASE)) & ~0xFF000000u) |
                                              (GPIO_AF_USART1 << 24) | (GPIO_AF_USART1 << 28);
    }
    /* BRR = fCK / baud，APB2=84MHz，115200 -> 0x2D9（整数部分 729，小数 0.75*16=12） */
    REG32(USART_BRR) = (uint32_t)((BAND_APB2_HZ + (baud / 2u)) / baud);
    REG32(USART_CR1) = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
}

/* 中断里把收到的字节压入环形缓冲（真实工程由 USART1_IRQHandler 调用） */
void board_usart1_rx_isr(void)
{
    if ((REG32(USART_SR) & USART_SR_RXNE) != 0u) {
        uint8_t b = (uint8_t)(REG32(USART_DR) & 0xFFu);
        (void)ring_write(&g_uart_rx, &b, 1u);
    }
}

static int stm_uart_write(const uint8_t *data, size_t len)
{
    size_t i;

    if (data == NULL) {
        return -1;
    }
    for (i = 0u; i < len; i++) {
        uint32_t guard = 0u;
        while (((REG32(USART_SR) & USART_SR_TXE) == 0u) && (guard++ < 100000u)) {
        }
        REG32(USART_DR) = (uint32_t)data[i];
    }
    while ((REG32(USART_SR) & USART_SR_TC) == 0u) {
    }
    return (int)len;
}

static int stm_uart_read(uint8_t *buf, size_t cap, uint32_t timeout_ms)
{
    uint32_t start = band_millis();
    size_t n;

    if (buf == NULL) {
        return -1;
    }
    for (;;) {
        n = ring_read(&g_uart_rx, buf, cap);
        if (n > 0u) {
            return (int)n;
        }
        if ((uint32_t)(band_millis() - start) >= timeout_ms) {
            return 0;
        }
    }
}

static void stm_uart_flush(void)
{
    uint8_t tmp[32];
    while (ring_read(&g_uart_rx, tmp, sizeof(tmp)) > 0u) {
    }
}

const uart_iface_t *uart_platform_iface(void)
{
    static const uart_iface_t iface = {
        "stm32-usart1",
        stm_uart_write,
        stm_uart_read,
        stm_uart_flush
    };
    return &iface;
}

void band_uart_feed(const uint8_t *data, size_t len)
{
    (void)ring_write(&g_uart_rx, data, len);
}

size_t band_uart_read(uint8_t *out, size_t cap)
{
    return ring_read(&g_uart_rx, out, cap);
}

void band_platform_init(void)
{
    if (g_uart_init == 0) {
        ring_init(&g_uart_rx);
        g_uart_init = 1;
    }
}

/* ---------------- 低功耗：STOP 模式 ---------------- */
uint32_t band_platform_enter_stop(uint32_t max_sleep_ms)
{
    uint32_t before = band_millis();
    uint32_t src = 0u;

    /* 1) 关掉 SysTick 之外的可关闭时钟（已在 power_mgr 里关外设） */
    /* 2) 设置深度睡眠 + 低功耗调节器（STOP 模式） */
    REG32(PWR_CR) |= PWR_CR_LPDS;
    REG32(PWR_CR) &= ~PWR_CR_PDDS;   /* PDDS=0 -> STOP 而非 STANDBY */

    /* 3) 配置唤醒源：RTC 唤醒定时器 + EXTI 线（触摸/按键/六轴中断） */
    (void)max_sleep_ms;

    /* 4) 进入 STOP，任意中断唤醒 */
    cpu_dsb();
    cpu_wfi();

    /* 5) 唤醒后清 PWR 唤醒标志并恢复调节器 */
    REG32(PWR_CR) |= PWR_CR_CWUF;
    REG32(PWR_CR) &= ~PWR_CR_LPDS;

    src = (band_millis() != before) ? (uint32_t)WAKE_SRC_RTC_ALARM : (uint32_t)WAKE_SRC_UNKNOWN;
    return src;
}

void band_platform_wakeup_stable(void)
{
    /* HSE + PLL 在 STOP 下会关闭，唤醒后需要重新稳定；
     * 传感器首帧数据不可信，因此固定等待一段时间再开始采集 */
    band_delay_ms(5u);
}

#endif /* !BAND_HOST_SIM */
