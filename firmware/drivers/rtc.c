/*
 * rtc.c -- STM32F4 片内 RTC 驱动（BCD 日历 / 唤醒定时器 / 闹钟）
 *
 * 初始化时序（RM0090 §26.3.5 "RTC 初始化序列"）：
 *   1) 解锁备份域：PWR_CR.DBP=1（平台层完成）
 *   2) 使能 LSE 并等待就绪；RCC_BDCR.RTCSEL=01(LSE)，RTCEN=1
 *   3) 解锁 RTC 写保护：RTC_WPR <- 0xCA, 0x53
 *   4) 进入初始化模式：RTC_ISR.INIT=1，等待 INITF=1
 *   5) 写 RTC_PRER（异步 127 / 同步 255 -> 1Hz）与 RTC_TR/DR（BCD）
 *   6) 退出初始化：INIT=0，等待影子寄存器同步 RSF=1
 *   7) 上锁 RTC_WPR
 *
 * 唤醒定时器：RTC_WUTR 以 ck_spre(1Hz) 计数，配合 CR.WUTE 与 ISR.WUTF，
 * 是手表休眠期间唯一的定时唤醒源，因此精度直接决定休眠功耗与时间准确性。
 */
#include "rtc.h"
#include "band_port.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/* 纯函数                                                              */
/* ------------------------------------------------------------------ */
uint8_t rtc_bin2bcd(uint8_t v)
{
    return (uint8_t)(((v / 10u) << 4) | (v % 10u));
}

uint8_t rtc_bcd2bin(uint8_t v)
{
    return (uint8_t)(((v >> 4) * 10u) + (v & 0x0Fu));
}

uint8_t rtc_is_leap(uint16_t year)
{
    if ((year % 4u) != 0u) {
        return 0u;
    }
    if ((year % 100u) != 0u) {
        return 1u;
    }
    return ((year % 400u) == 0u) ? 1u : 0u;
}

uint8_t rtc_days_in_month(uint16_t year, uint8_t month)
{
    static const uint8_t tab[12] = { 31u, 28u, 31u, 30u, 31u, 30u, 31u, 31u, 30u, 31u, 30u, 31u };

    if ((month < 1u) || (month > 12u)) {
        return 0u;
    }
    if ((month == 2u) && (rtc_is_leap(year) != 0u)) {
        return 29u;
    }
    return tab[month - 1u];
}

/* Sakamoto 算法：日期 -> 星期（1=周一 .. 7=周日），无查表无分支 */
uint8_t rtc_weekday_from_date(uint16_t year, uint8_t month, uint8_t day)
{
    static const uint8_t t[12] = { 0u, 3u, 2u, 5u, 0u, 3u, 5u, 1u, 4u, 6u, 2u, 4u };
    uint16_t y = year;

    if (month < 3u) {
        y = (uint16_t)(y - 1u);
    }
    {
        uint32_t w = ((uint32_t)y + (uint32_t)(y / 4u) - (uint32_t)(y / 100u) +
                      (uint32_t)(y / 400u) + (uint32_t)t[month - 1u] + (uint32_t)day);
        /* 结果 0=周日 .. 6=周六；转成 1=周一 .. 7=周日 */
        w = w % 7u;
        return (uint8_t)((w == 0u) ? 7u : w);
    }
}

uint32_t rtc_time_to_unix(const rtc_time_t *t)
{
    uint32_t days = 0u;
    uint16_t y;
    uint8_t m;

    if (t == NULL) {
        return 0u;
    }
    /* 基准：2000-01-01 00:00:00 */
    for (y = 2000u; y < (uint16_t)(2000u + t->year); y++) {
        days += (rtc_is_leap(y) != 0u) ? 366u : 365u;
    }
    for (m = 1u; m < t->month; m++) {
        days += rtc_days_in_month((uint16_t)(2000u + t->year), m);
    }
    days += (uint32_t)(t->day - 1u);
    return (days * 86400u) + ((uint32_t)t->hour * 3600u) + ((uint32_t)t->minute * 60u) + (uint32_t)t->second;
}

int rtc_unix_to_time(uint32_t unix_s, rtc_time_t *t)
{
    uint32_t days;
    uint32_t rem;
    uint16_t year = 2000u;

    if (t == NULL) {
        return -1;
    }
    days = unix_s / 86400u;
    rem = unix_s % 86400u;

    for (;;) {
        uint16_t len = (rtc_is_leap(year) != 0u) ? 366u : 365u;
        if (days < len) {
            break;
        }
        days -= len;
        year++;
        if (year > 2099u) {
            return -1;
        }
    }

    {
        uint8_t month = 1u;
        for (;;) {
            uint8_t dim = rtc_days_in_month(year, month);
            if (days < dim) {
                break;
            }
            days -= dim;
            month++;
        }
        t->year = (uint8_t)(year - 2000u);
        t->month = month;
        t->day = (uint8_t)(days + 1u);
    }
    t->hour = (uint8_t)(rem / 3600u);
    t->minute = (uint8_t)((rem % 3600u) / 60u);
    t->second = (uint8_t)(rem % 60u);
    t->weekday = rtc_weekday_from_date((uint16_t)(2000u + t->year), t->month, t->day);
    return 0;
}

int rtc_time_is_valid(const rtc_time_t *t)
{
    if (t == NULL) {
        return 0;
    }
    if (t->year > 99u) {
        return 0;
    }
    if ((t->month < 1u) || (t->month > 12u)) {
        return 0;
    }
    if ((t->day < 1u) || (t->day > rtc_days_in_month((uint16_t)(2000u + t->year), t->month))) {
        return 0;
    }
    if (t->hour > 23u) {
        return 0;
    }
    if ((t->minute > 59u) || (t->second > 59u)) {
        return 0;
    }
    return 1;
}

/* ------------------------------------------------------------------ */
/* 寄存器访问辅助                                                      */
/* ------------------------------------------------------------------ */
static int rtc_rd(rtc_dev_t *dev, uint32_t off, uint32_t *val)
{
    if ((dev == NULL) || (dev->iface == NULL) || (dev->iface->reg_read == NULL)) {
        return -1;
    }
    return dev->iface->reg_read(off, val);
}

static int rtc_wr(rtc_dev_t *dev, uint32_t off, uint32_t val)
{
    if ((dev == NULL) || (dev->iface == NULL) || (dev->iface->reg_write == NULL)) {
        return -1;
    }
    return dev->iface->reg_write(off, val);
}

int rtc_unlock(rtc_dev_t *dev)
{
    if (rtc_wr(dev, RTC_OFF_WPR, RTC_WPR_KEY1) != 0) {
        return -1;
    }
    return rtc_wr(dev, RTC_OFF_WPR, RTC_WPR_KEY2);
}

int rtc_lock(rtc_dev_t *dev)
{
    /* 写任意错误键即重新上锁 */
    return rtc_wr(dev, RTC_OFF_WPR, 0xFFu);
}

int rtc_enter_init(rtc_dev_t *dev)
{
    uint32_t isr = 0u;
    uint32_t guard;

    if (rtc_rd(dev, RTC_OFF_ISR, &isr) != 0) {
        return -1;
    }
    if (rtc_wr(dev, RTC_OFF_ISR, isr | RTC_ISR_INIT) != 0) {
        return -1;
    }
    /* 等待 INITF 置位，最长约 2 个 RTCCLK 周期 */
    for (guard = 0u; guard < 1000u; guard++) {
        if ((rtc_rd(dev, RTC_OFF_ISR, &isr) == 0) && ((isr & RTC_ISR_INITF) != 0u)) {
            return 0;
        }
        band_delay_ms(1u);
    }
    return -1;
}

int rtc_exit_init(rtc_dev_t *dev)
{
    uint32_t isr = 0u;
    uint32_t guard;

    if (rtc_rd(dev, RTC_OFF_ISR, &isr) != 0) {
        return -1;
    }
    if (rtc_wr(dev, RTC_OFF_ISR, isr & ~RTC_ISR_INIT) != 0) {
        return -1;
    }
    /* 退出后必须等影子寄存器同步完成（RSF），否则读回的是旧值 */
    for (guard = 0u; guard < 1000u; guard++) {
        if ((rtc_rd(dev, RTC_OFF_ISR, &isr) == 0) && ((isr & RTC_ISR_RSF) != 0u)) {
            return 0;
        }
        band_delay_ms(1u);
    }
    return -1;
}

int rtc_set_prescaler(rtc_dev_t *dev, uint32_t async_div, uint32_t sync_div)
{
    /* PRER: PREDIV_A[22:16] 异步，PREDIV_S[14:0] 同步
     * ck_spre = RTCCLK / ((PREDIV_A+1) * (PREDIV_S+1))，LSE 32768 -> 1Hz 用 127/255 */
    uint32_t v = ((async_div & 0x7Fu) << 16) | (sync_div & 0x7FFFu);

    if ((async_div == 0u) || (async_div > 128u) || (sync_div > 32768u)) {
        return -1;
    }
    return rtc_wr(dev, RTC_OFF_PRER, v);
}

int rtc_init(rtc_dev_t *dev, const rtc_iface_t *iface)
{
    uint32_t isr = 0u;

    if ((dev == NULL) || (iface == NULL)) {
        return -1;
    }
    memset(dev, 0, sizeof(*dev));
    dev->iface = iface;
    dev->clock_source = 0u;   /* LSE */

    if (rtc_unlock(dev) != 0) {
        return -1;
    }
    if (rtc_enter_init(dev) != 0) {
        (void)rtc_lock(dev);
        return -1;
    }
    /* LSE 32768Hz -> 1Hz：异步 127 + 同步 255 */
    if (rtc_set_prescaler(dev, 127u, 255u) != 0) {
        (void)rtc_exit_init(dev);
        (void)rtc_lock(dev);
        return -1;
    }
    if (rtc_exit_init(dev) != 0) {
        (void)rtc_lock(dev);
        return -1;
    }
    /* 清掉上电残留的唤醒/闹钟标志 */
    if (rtc_rd(dev, RTC_OFF_ISR, &isr) == 0) {
        (void)rtc_wr(dev, RTC_OFF_ISR, isr & ~(RTC_ISR_WUTF | RTC_ISR_ALRAF));
    }
    (void)rtc_lock(dev);
    dev->inited = 1u;
    dev->syncs++;
    return 0;
}

int rtc_set_time(rtc_dev_t *dev, const rtc_time_t *t)
{
    uint32_t tr;
    uint32_t dr;

    if ((dev == NULL) || (t == NULL) || (dev->inited == 0u)) {
        return -1;
    }
    if (rtc_time_is_valid(t) == 0) {
        return -1;
    }
    /* TR: HT[21:20] HU[19:16] MNT[14:12] MNU[11:8] ST[6:4] SU[3:0]
     * （bit23 PM 位不用，因为本工程用 24 小时制：CR.FMT=0） */
    tr = ((uint32_t)rtc_bin2bcd(t->hour) << 16) |
         ((uint32_t)rtc_bin2bcd(t->minute) << 8) |
         (uint32_t)rtc_bin2bcd(t->second);
    /* DR: YT[23:20] YU[19:16] WDU[15:13] MT[12:8] MU[7:4] DT[5:4] DU[3:0] */
    {
        uint8_t wd = (t->weekday >= 1u && t->weekday <= 7u)
                         ? t->weekday
                         : rtc_weekday_from_date((uint16_t)(2000u + t->year), t->month, t->day);
        dr = ((uint32_t)rtc_bin2bcd(t->year) << 16) |
             ((uint32_t)(wd & 0x07u) << 13) |
             ((uint32_t)rtc_bin2bcd(t->month) << 8) |
             (uint32_t)rtc_bin2bcd(t->day);
    }

    if (rtc_unlock(dev) != 0) {
        return -1;
    }
    if (rtc_enter_init(dev) != 0) {
        (void)rtc_lock(dev);
        return -1;
    }
    if (rtc_wr(dev, RTC_OFF_TR, tr) != 0) {
        (void)rtc_exit_init(dev);
        (void)rtc_lock(dev);
        return -1;
    }
    if (rtc_wr(dev, RTC_OFF_DR, dr) != 0) {
        (void)rtc_exit_init(dev);
        (void)rtc_lock(dev);
        return -1;
    }
    if (rtc_exit_init(dev) != 0) {
        (void)rtc_lock(dev);
        return -1;
    }
    (void)rtc_lock(dev);
    dev->set_count++;
    return 0;
}

int rtc_get_time(rtc_dev_t *dev, rtc_time_t *t)
{
    uint32_t tr = 0u;
    uint32_t dr = 0u;

    if ((dev == NULL) || (t == NULL)) {
        return -1;
    }
    /* 连读两次直到一致，避免在秒进位瞬间读到撕裂的时间 */
    {
        uint8_t attempt;
        for (attempt = 0u; attempt < 3u; attempt++) {
            uint32_t tr2 = 0u;
            uint32_t dr2 = 0u;
            if (rtc_rd(dev, RTC_OFF_TR, &tr) != 0) {
                return -1;
            }
            if (rtc_rd(dev, RTC_OFF_DR, &dr) != 0) {
                return -1;
            }
            if (rtc_rd(dev, RTC_OFF_TR, &tr2) != 0) {
                return -1;
            }
            if (rtc_rd(dev, RTC_OFF_DR, &dr2) != 0) {
                return -1;
            }
            if ((tr == tr2) && (dr == dr2)) {
                break;
            }
            tr = tr2;
            dr = dr2;
        }
    }

    t->second = rtc_bcd2bin((uint8_t)(tr & 0x7Fu));
    t->minute = rtc_bcd2bin((uint8_t)((tr >> 8) & 0x7Fu));
    /* bit22 = PM；本工程 24 小时制下该位为 0，bit21:20 直接是小时的十位 */
    t->hour = rtc_bcd2bin((uint8_t)((tr >> 16) & 0x3Fu));
    if (((tr >> 22) & 0x01u) != 0u) {
        /* 万一上层配成了 12 小时制，这里也要能正确还原 */
        t->hour = (uint8_t)(((t->hour % 12u) + 12u) % 24u);
    }
    t->day = rtc_bcd2bin((uint8_t)(dr & 0x3Fu));
    t->month = rtc_bcd2bin((uint8_t)((dr >> 8) & 0x1Fu));
    t->weekday = (uint8_t)((dr >> 13) & 0x07u);
    t->year = rtc_bcd2bin((uint8_t)((dr >> 16) & 0xFFu));
    if (t->weekday == 0u) {
        t->weekday = rtc_weekday_from_date((uint16_t)(2000u + t->year), t->month, t->day);
    }
    dev->syncs++;
    return 0;
}

int rtc_set_alarm(rtc_dev_t *dev, const rtc_time_t *t, uint8_t mask_date)
{
    uint32_t alrmar;
    uint32_t cr = 0u;

    if ((dev == NULL) || (t == NULL)) {
        return -1;
    }
    /* ALRMAR: MSK4(31) WDSEL(30) DT[29:28] DU[27:24] MSK3(23) PM(22) HT[21:20] HU[19:16]
     *         MSK2(15) MNT[14:12] MNU[11:8] MSK1(7) ST[6:4] SU[3:0]
     * 屏蔽位置 1 表示"该字段不参与比较"。本工程只做"每天固定时间"闹钟，
     * 因此屏蔽日期字段（MSK4=1）。 */
    alrmar = (1u << 31);
    if (mask_date == 0u) {
        alrmar &= ~(1u << 31);
        alrmar |= ((uint32_t)rtc_bin2bcd(t->day) << 24);
    }
    alrmar |= ((uint32_t)rtc_bin2bcd(t->hour) << 16);
    alrmar |= ((uint32_t)rtc_bin2bcd(t->minute) << 8);
    alrmar |= (uint32_t)rtc_bin2bcd(t->second);

    if (rtc_unlock(dev) != 0) {
        return -1;
    }
    /* 关闹钟再改寄存器，避免改到一半触发 */
    if (rtc_rd(dev, RTC_OFF_CR, &cr) == 0) {
        (void)rtc_wr(dev, RTC_OFF_CR, cr & ~(RTC_CR_ALRAE | RTC_CR_ALRAIE));
    }
    {
        uint32_t guard;
        uint32_t isr = 0u;
        for (guard = 0u; guard < 1000u; guard++) {
            if ((rtc_rd(dev, RTC_OFF_ISR, &isr) == 0) && ((isr & RTC_ISR_ALRAWF) != 0u)) {
                break;
            }
            band_delay_ms(1u);
        }
    }
    if (rtc_wr(dev, RTC_OFF_ALRMAR, alrmar) != 0) {
        (void)rtc_lock(dev);
        return -1;
    }
    if (rtc_rd(dev, RTC_OFF_CR, &cr) == 0) {
        (void)rtc_wr(dev, RTC_OFF_CR, cr | RTC_CR_ALRAE | RTC_CR_ALRAIE);
    }
    (void)rtc_lock(dev);
    dev->alarm_count++;
    return 0;
}

int rtc_enable_wakeup(rtc_dev_t *dev, uint16_t seconds)
{
    uint32_t cr = 0u;
    uint32_t guard;
    uint32_t isr = 0u;

    if ((dev == NULL) || (seconds == 0u)) {
        return -1;
    }
    if (rtc_unlock(dev) != 0) {
        return -1;
    }
    /* 关 WUTE 才能改 WUTR */
    if (rtc_rd(dev, RTC_OFF_CR, &cr) == 0) {
        (void)rtc_wr(dev, RTC_OFF_CR, cr & ~(RTC_CR_WUTE | RTC_CR_WUTIE));
    }
    for (guard = 0u; guard < 1000u; guard++) {
        if ((rtc_rd(dev, RTC_OFF_ISR, &isr) == 0) && ((isr & RTC_ISR_WUTWF) != 0u)) {
            break;
        }
        band_delay_ms(1u);
    }
    if (rtc_wr(dev, RTC_OFF_WUTR, (uint32_t)(seconds - 1u)) != 0) {
        (void)rtc_lock(dev);
        return -1;
    }
    /* WUTE=1 启动唤醒定时器，WUTIE=1 让中断能唤醒 STOP 模式 */
    if (rtc_rd(dev, RTC_OFF_CR, &cr) == 0) {
        (void)rtc_wr(dev, RTC_OFF_CR, cr | RTC_CR_WUTE | RTC_CR_WUTIE);
    }
    /* 清 WUTF，避免一进 STOP 就被旧标志唤醒 */
    if (rtc_rd(dev, RTC_OFF_ISR, &isr) == 0) {
        (void)rtc_wr(dev, RTC_OFF_ISR, isr & ~RTC_ISR_WUTF);
    }
    (void)rtc_lock(dev);
    dev->wakeup_count++;
    return 0;
}

int rtc_disable_wakeup(rtc_dev_t *dev)
{
    uint32_t cr = 0u;

    if (dev == NULL) {
        return -1;
    }
    if (rtc_unlock(dev) != 0) {
        return -1;
    }
    if (rtc_rd(dev, RTC_OFF_CR, &cr) == 0) {
        (void)rtc_wr(dev, RTC_OFF_CR, cr & ~(RTC_CR_WUTE | RTC_CR_WUTIE));
    }
    (void)rtc_lock(dev);
    return 0;
}

int rtc_clear_flags(rtc_dev_t *dev, uint32_t mask)
{
    uint32_t isr = 0u;

    if (rtc_rd(dev, RTC_OFF_ISR, &isr) != 0) {
        return -1;
    }
    /* WUTF/ALRAF 是"写 0 清除" */
    return rtc_wr(dev, RTC_OFF_ISR, isr & ~mask);
}

int rtc_get_flags(rtc_dev_t *dev, uint32_t *isr)
{
    return rtc_rd(dev, RTC_OFF_ISR, isr);
}
