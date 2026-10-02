/*
 * rtc.h -- STM32F4 片内 RTC 驱动（BCD 日历 + 唤醒定时器 + 闹钟）
 *
 * STM32F4 RTC 关键寄存器（寄存器级访问，不依赖 HAL）：
 *   RTC_TR    0x00 时间：BCD，bit22:20 = PM/HT, bit14:12=HU ...
 *   RTC_DR    0x04 日期：BCD，bit20:16 = 年(00-99), bit15:13=WDU, bit12:8=MT ...
 *   RTC_CR    0x08 控制：WUTE(10) WUTIE(14) ALRAE(8) ALRAIE(12) BYPSHAD(6) FMT(5)
 *   RTC_ISR   0x0C 状态：INIT(7) INITF(6) WUTWF(2) ALRAWF(0) RSF(5) WUTF(10)
 *   RTC_PRER  0x10 预分频：PREDIV_S[14:0] PREDIV_A[22:16]
 *   RTC_WUTR  0x14 唤醒自动重载（16bit，时钟 = ck_spre = 1Hz）
 *   RTC_ALRMAR 0x1C 闹钟 A：MSK4..MSK1 可屏蔽字段
 *   RTC_WPR   0x24 写保护：先 0xCA 再 0x53 解锁
 *   RTC_TAFCR 0x40 侵入/时间戳/输出配置
 * 时钟源：LSE 32768Hz，异步分频 127 + 同步分频 255 -> 1Hz
 */
#ifndef RTC_H
#define RTC_H

#include <stdint.h>
#include <stddef.h>

/* 寄存器偏移（相对 RTC 基址 0x40002800） */
#define RTC_OFF_TR        0x00u
#define RTC_OFF_DR        0x04u
#define RTC_OFF_CR        0x08u
#define RTC_OFF_ISR       0x0Cu
#define RTC_OFF_PRER      0x10u
#define RTC_OFF_WUTR      0x14u
#define RTC_OFF_CALIBR    0x18u
#define RTC_OFF_ALRMAR    0x1Cu
#define RTC_OFF_ALRMBR    0x20u
#define RTC_OFF_WPR       0x24u
#define RTC_OFF_TSTR      0x30u
#define RTC_OFF_TSDR      0x34u
#define RTC_OFF_TAFCR     0x40u
#define RTC_OFF_BKP0R     0x50u

/* ISR 位 */
#define RTC_ISR_INIT      0x00000080u
#define RTC_ISR_INITF     0x00000040u
#define RTC_ISR_RSF       0x00000020u
#define RTC_ISR_WUTF      0x00000400u
#define RTC_ISR_ALRAF     0x00000100u
#define RTC_ISR_WUTWF     0x00000004u
#define RTC_ISR_ALRAWF    0x00000001u

/* CR 位 */
#define RTC_CR_WUTE       0x00000400u
#define RTC_CR_WUTIE      0x00004000u
#define RTC_CR_ALRAE      0x00000100u
#define RTC_CR_ALRAIE     0x00001000u
#define RTC_CR_BYPSHAD    0x00000040u

#define RTC_WPR_KEY1      0xCAu
#define RTC_WPR_KEY2      0x53u

typedef struct {
    uint8_t  year;    /* 0..99，表示 2000+year */
    uint8_t  month;   /* 1..12 */
    uint8_t  day;     /* 1..31 */
    uint8_t  weekday; /* 1..7，1 = 周一 */
    uint8_t  hour;    /* 0..23 */
    uint8_t  minute;  /* 0..59 */
    uint8_t  second;  /* 0..59 */
} rtc_time_t;

/* 目标板上的 RTC 寄存器访问由平台层提供（PC 仿真下是一块内存/真实时钟） */
typedef struct {
    int (*reg_read)(uint32_t off, uint32_t *val);
    int (*reg_write)(uint32_t off, uint32_t val);
    uint32_t (*now_ms)(void);
} rtc_iface_t;

typedef struct {
    const rtc_iface_t *iface;
    uint8_t  inited;
    uint8_t  clock_source;   /* 0=LSE 1=LSI */
    uint32_t syncs;
    uint32_t set_count;
    uint32_t alarm_count;
    uint32_t wakeup_count;
} rtc_dev_t;

int  rtc_init(rtc_dev_t *dev, const rtc_iface_t *iface);
int  rtc_unlock(rtc_dev_t *dev);
int  rtc_lock(rtc_dev_t *dev);
int  rtc_enter_init(rtc_dev_t *dev);
int  rtc_exit_init(rtc_dev_t *dev);
int  rtc_set_prescaler(rtc_dev_t *dev, uint32_t async_div, uint32_t sync_div);
int  rtc_set_time(rtc_dev_t *dev, const rtc_time_t *t);
int  rtc_get_time(rtc_dev_t *dev, rtc_time_t *t);
int  rtc_set_alarm(rtc_dev_t *dev, const rtc_time_t *t, uint8_t mask_date);
int  rtc_enable_wakeup(rtc_dev_t *dev, uint16_t seconds);
int  rtc_disable_wakeup(rtc_dev_t *dev);
int  rtc_clear_flags(rtc_dev_t *dev, uint32_t mask);
int  rtc_get_flags(rtc_dev_t *dev, uint32_t *isr);

/* --- 纯函数 --- */
uint8_t  rtc_bin2bcd(uint8_t v);
uint8_t  rtc_bcd2bin(uint8_t v);
uint8_t  rtc_weekday_from_date(uint16_t year, uint8_t month, uint8_t day);
uint8_t  rtc_days_in_month(uint16_t year, uint8_t month);
uint8_t  rtc_is_leap(uint16_t year);
/* 2000-01-01 00:00:00 UTC 起的秒数 <-> 日历 */
uint32_t rtc_time_to_unix(const rtc_time_t *t);
int      rtc_unix_to_time(uint32_t unix_s, rtc_time_t *t);
int      rtc_time_is_valid(const rtc_time_t *t);

#endif /* RTC_H */
