/*
 * rtc_port.c -- RTC / IWDG 寄存器访问的两种平台实现
 *
 * PC 仿真（BAND_HOST_SIM）：用一块内存寄存器块 + 由 band_millis 驱动的
 *   模拟 LSE 计数器，这样 rtc.c 的 BCD 编解码、初始化时序、闹钟/唤醒
 *   配置逻辑都能在 PC 上被真实执行与断言。
 * STM32 目标：直接读写 RTC / IWDG 的 MMIO 地址（RM0090 §26 / §22）。
 */
#include "band_platform.h"
#include "band_port.h"
#include "rtc.h"
#include "iwdg.h"

#if defined(BAND_HOST_SIM)
/* ================================================================== */
/* PC 仿真实现                                                         */
/* ================================================================== */
#include <string.h>

#define SIM_RTC_REGS 32u

static uint32_t g_rtc_reg[SIM_RTC_REGS];
static uint32_t g_bkp[16];
static uint32_t g_iwdg_reg[8];
static uint32_t g_iwdg_reset_cause;
static uint32_t g_sim_base_ms;
static int      g_sim_inited;

/* 把 RTC_TR/DR 与"仿真时间"绑定：每次读 TR/DR 时按已流逝毫秒推进日历。
 * 真实器件里这是硬件自增的，仿真里由软件代劳，行为对驱动层透明。 */
static void sim_rtc_sync(void)
{
    uint32_t elapsed_s;
    uint32_t tr;
    uint32_t dr;
    uint32_t sec;
    uint32_t min;
    uint32_t hour;
    uint32_t day;
    uint32_t month;
    uint32_t year;
    uint32_t dim;

    if (g_sim_inited == 0) {
        return;
    }
    elapsed_s = (band_millis() - g_sim_base_ms) / 1000u;
    if (elapsed_s == 0u) {
        return;
    }
    g_sim_base_ms += elapsed_s * 1000u;

    /* 从当前 TR/DR 解出 BCD 时间 */
    tr = g_rtc_reg[RTC_OFF_TR / 4u];
    dr = g_rtc_reg[RTC_OFF_DR / 4u];
    sec = ((tr >> 4) & 0x7u) * 10u + (tr & 0xFu);
    min = (((tr >> 12) & 0x7u) * 10u) + ((tr >> 8) & 0xFu);
    hour = (((tr >> 20) & 0x3u) * 10u) + ((tr >> 16) & 0xFu);
    day = (((dr >> 4) & 0x3u) * 10u) + (dr & 0xFu);
    month = (((dr >> 12) & 0x1u) * 10u) + ((dr >> 8) & 0xFu);
    year = (((dr >> 20) & 0xFu) * 10u) + ((dr >> 16) & 0xFu);
    if (year > 99u) { year = 0u; }

    sec += elapsed_s;
    min += sec / 60u;
    sec %= 60u;
    hour += min / 60u;
    min %= 60u;
    day += hour / 24u;
    hour %= 24u;

    for (;;) {
        uint8_t m = (uint8_t)(month == 0u ? 1u : month);
        uint16_t y = (uint16_t)(2000u + year);
        uint8_t leap = (((y % 4u) == 0u) && (((y % 100u) != 0u) || ((y % 400u) == 0u))) ? 1u : 0u;
        static const uint8_t dim_tab[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        dim = dim_tab[(m - 1u) % 12u];
        if ((m == 2u) && (leap != 0u)) {
            dim = 29u;
        }
        if (day <= dim) {
            break;
        }
        day -= dim;
        month++;
        if (month > 12u) {
            month = 1u;
            year = (year + 1u) % 100u;
        }
    }

    tr = ((hour / 10u) << 20) | ((hour % 10u) << 16) |
         ((min / 10u) << 12) | ((min % 10u) << 8) |
         ((sec / 10u) << 4) | (sec % 10u);
    dr = ((year / 10u) << 20) | ((year % 10u) << 16) |
         ((month / 10u) << 12) | ((month % 10u) << 8) |
         (1u << 13) |   /* WDU 固定 1（周一），真实板子由星期算法填充 */
         ((day / 10u) << 4) | (day % 10u);
    g_rtc_reg[RTC_OFF_TR / 4u] = tr;
    g_rtc_reg[RTC_OFF_DR / 4u] = dr;
}

static int sim_rtc_read(uint32_t off, uint32_t *val)
{
    if (val == NULL) {
        return -1;
    }
    if ((off == RTC_OFF_TR) || (off == RTC_OFF_DR)) {
        sim_rtc_sync();
    }
    if ((off / 4u) >= SIM_RTC_REGS) {
        return -1;
    }
    *val = g_rtc_reg[off / 4u];
    return 0;
}

static int sim_rtc_write(uint32_t off, uint32_t val)
{
    if ((off / 4u) >= SIM_RTC_REGS) {
        return -1;
    }
    /* 模拟硬件写保护：WPR 未按 0xCA/0x53 解锁时拒绝写 TR/DR/PRER */
    if (((off == RTC_OFF_TR) || (off == RTC_OFF_DR) || (off == RTC_OFF_PRER)) &&
        (g_rtc_reg[RTC_OFF_WPR / 4u] != 0x53u)) {
        return -1;
    }
    if (off == RTC_OFF_WPR) {
        /* 键序列：写 0xCA 后再写 0x53 才解锁；写其它值立即上锁 */
        if (val == RTC_WPR_KEY1) {
            g_rtc_reg[off / 4u] = 0xCAu;
        } else if ((val == RTC_WPR_KEY2) && (g_rtc_reg[off / 4u] == 0xCAu)) {
            g_rtc_reg[off / 4u] = 0x53u;
        } else {
            g_rtc_reg[off / 4u] = 0u;
        }
        return 0;
    }
    g_rtc_reg[off / 4u] = val;
    /* 模拟 ISR 的就绪位：写完 PRER 后 WUTWF/ALRAWF 立即置位，INITF 跟随 INIT */
    if (off == RTC_OFF_ISR) {
        if ((val & RTC_ISR_INIT) != 0u) {
            g_rtc_reg[RTC_OFF_ISR / 4u] |= RTC_ISR_INITF;
        } else {
            g_rtc_reg[RTC_OFF_ISR / 4u] &= ~RTC_ISR_INITF;
        }
    }
    if (off == RTC_OFF_PRER) {
        g_rtc_reg[RTC_OFF_ISR / 4u] |= (RTC_ISR_WUTWF | RTC_ISR_ALRAWF | RTC_ISR_RSF);
    }
    return 0;
}

static uint32_t sim_now_ms(void)
{
    return band_millis();
}

static const rtc_iface_t g_rtc_iface = {
    sim_rtc_read,
    sim_rtc_write,
    sim_now_ms
};

const rtc_iface_t *rtc_platform_iface(void)
{
    if (g_sim_inited == 0) {
        memset(g_rtc_reg, 0, sizeof(g_rtc_reg));
        memset(g_bkp, 0, sizeof(g_bkp));
        /* 上电默认：ISR 就绪位全置位，PRER 已由驱动配置 */
        g_rtc_reg[RTC_OFF_ISR / 4u] = RTC_ISR_INITF | RTC_ISR_WUTWF | RTC_ISR_ALRAWF | RTC_ISR_RSF;
        g_sim_base_ms = band_millis();
        g_sim_inited = 1;
    }
    return &g_rtc_iface;
}

/* ---------------- IWDG 仿真 ---------------- */
static int sim_iwdg_read(uint32_t off, uint32_t *val)
{
    if ((val == NULL) || ((off / 4u) >= 8u)) {
        return -1;
    }
    *val = g_iwdg_reg[off / 4u];
    return 0;
}

static int sim_iwdg_write(uint32_t off, uint32_t val)
{
    if ((off / 4u) >= 8u) {
        return -1;
    }
    if (off == IWDG_OFF_KR) {
        if (val == IWDG_KEY_UNLOCK) {
            g_iwdg_reg[0] = IWDG_KEY_UNLOCK;
            return 0;
        }
        if (val == IWDG_KEY_START) {
            g_iwdg_reg[0] = IWDG_KEY_START;
            return 0;
        }
        if (val == IWDG_KEY_RELOAD) {
            /* 喂狗：清 WVU，模拟 SR 的更新完成 */
            g_iwdg_reg[IWDG_OFF_SR / 4u] = 0u;
            g_iwdg_reg[0] = 0u;
            return 0;
        }
        return -1;
    }
    if ((off == IWDG_OFF_PR) || (off == IWDG_OFF_RLR)) {
        if (g_iwdg_reg[0] != IWDG_KEY_UNLOCK) {
            return -1;   /* 未解锁写保护 */
        }
        g_iwdg_reg[off / 4u] = val;
        /* 仿真中 SR 的 PVU/RVU 立即清零 */
        g_iwdg_reg[IWDG_OFF_SR / 4u] = 0u;
        return 0;
    }
    g_iwdg_reg[off / 4u] = val;
    return 0;
}

static uint32_t sim_iwdg_reset_cause(void)
{
    return g_iwdg_reset_cause;
}

static void sim_iwdg_clear_reset_cause(void)
{
    g_iwdg_reset_cause = 0u;
}

static void sim_bkp_write(uint32_t idx, uint32_t val)
{
    if (idx < 16u) {
        g_bkp[idx] = val;
    }
}

static uint32_t sim_bkp_read(uint32_t idx)
{
    return (idx < 16u) ? g_bkp[idx] : 0u;
}

/* 测试辅助：模拟"看门狗超时复位"，让驱动能读到罪魁任务号 */
void rtc_iwdg_sim_trigger_reset(uint32_t fault_task);
void rtc_iwdg_sim_trigger_reset(uint32_t fault_task)
{
    g_iwdg_reset_cause = 1u;
    sim_bkp_write(0u, fault_task);
}

static const iwdg_iface_t g_iwdg_iface = {
    sim_iwdg_read,
    sim_iwdg_write,
    sim_iwdg_reset_cause,
    sim_iwdg_clear_reset_cause,
    sim_bkp_write,
    sim_bkp_read,
    sim_now_ms
};

const iwdg_iface_t *iwdg_platform_iface(void)
{
    return &g_iwdg_iface;
}

#else /* ---------------- STM32 目标实现 ---------------- */

#include "stm32f405_regs.h"

static int stm_rtc_read(uint32_t off, uint32_t *val)
{
    if (val == NULL) {
        return -1;
    }
    *val = REG32(PERIPH_RTC_BASE + off);
    return 0;
}

static int stm_rtc_write(uint32_t off, uint32_t val)
{
    REG32(PERIPH_RTC_BASE + off) = val;
    return 0;
}

static const rtc_iface_t g_rtc_iface = {
    stm_rtc_read,
    stm_rtc_write,
    band_millis
};

const rtc_iface_t *rtc_platform_iface(void)
{
    return &g_rtc_iface;
}

static int stm_iwdg_read(uint32_t off, uint32_t *val)
{
    if (val == NULL) {
        return -1;
    }
    *val = REG32(PERIPH_IWDG_BASE + off);
    return 0;
}

static int stm_iwdg_write(uint32_t off, uint32_t val)
{
    REG32(PERIPH_IWDG_BASE + off) = val;
    return 0;
}

static uint32_t stm_iwdg_reset_cause(void)
{
    return (REG32(RCC_CSR) & RCC_CSR_IWDGRSTF) ? 1u : 0u;
}

static void stm_iwdg_clear_reset_cause(void)
{
    /* RMVF：清所有复位标志 */
    REG32(RCC_CSR) |= RCC_CSR_RMVF;
}

static void stm_bkp_write(uint32_t idx, uint32_t val)
{
    if (idx < 20u) {
        REG32(PERIPH_RTC_BASE + RTC_OFF_BKP0R + (idx * 4u)) = val;
    }
}

static uint32_t stm_bkp_read(uint32_t idx)
{
    return (idx < 20u) ? REG32(PERIPH_RTC_BASE + RTC_OFF_BKP0R + (idx * 4u)) : 0u;
}

static const iwdg_iface_t g_iwdg_iface = {
    stm_iwdg_read,
    stm_iwdg_write,
    stm_iwdg_reset_cause,
    stm_iwdg_clear_reset_cause,
    stm_bkp_write,
    stm_bkp_read,
    band_millis
};

const iwdg_iface_t *iwdg_platform_iface(void)
{
    return &g_iwdg_iface;
}

#endif /* BAND_HOST_SIM */
