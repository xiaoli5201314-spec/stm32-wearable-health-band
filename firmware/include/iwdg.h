/*
 * iwdg.h -- 独立看门狗（IWDG）+ 任务级喂狗监督
 *
 * STM32F4 IWDG 寄存器（基址 0x40003000）：
 *   0x00 IWDG_KR   键寄存器：0x5555 解除 PR/RLR 写保护，0xAAAA 喂狗，0xCCCC 启动
 *   0x04 IWDG_PR   预分频：0..7 -> 4,8,16,32,64,128,256
 *   0x08 IWDG_RLR  重装载值：12bit
 *   0x0C IWDG_SR   状态：PVU(0) RVU(1) WVU(2)，写 PR/RLR 后必须等对应位清零
 *
 * 超时 = (4 * 2^PR) * (RLR + 1) / f_LSI ,  f_LSI ≈ 32kHz
 *
 * 单纯"每 500ms 喂一次"只能防死循环，防不了"单个任务饿死"。
 * 因此这里做任务级监督：每个受监督任务必须在自己周期内上报心跳，
 * 有任何任务超期就停止喂狗，让 IWDG 复位整机并在 BKP 寄存器留下罪魁任务号。
 */
#ifndef IWDG_H
#define IWDG_H

#include <stdint.h>
#include <stddef.h>
#include "band_config.h"

#define IWDG_OFF_KR    0x00u
#define IWDG_OFF_PR    0x04u
#define IWDG_OFF_RLR   0x08u
#define IWDG_OFF_SR    0x0Cu
#define IWDG_OFF_WINR  0x10u

#define IWDG_KEY_UNLOCK   0x5555u
#define IWDG_KEY_RELOAD   0xAAAAu
#define IWDG_KEY_START    0xCCCCu

#define IWDG_SR_PVU       0x01u
#define IWDG_SR_RVU       0x02u
#define IWDG_SR_WVU       0x04u

#define IWDG_MAX_SUPERVISED  8u
#define IWDG_BKP_FAULT_IDX   0u    /* RTC_BKP0R：复位后读回罪魁任务下标 */

typedef struct {
    int (*reg_read)(uint32_t off, uint32_t *val);
    int (*reg_write)(uint32_t off, uint32_t val);
    /* 目标板上是 RCC_CSR.IWDGRSTF 与 BKP 寄存器；PC 仿真下由桩提供 */
    uint32_t (*reset_cause)(void);
    void     (*clear_reset_cause)(void);
    void     (*bkp_write)(uint32_t idx, uint32_t val);
    uint32_t (*bkp_read)(uint32_t idx);
    uint32_t (*now_ms)(void);
} iwdg_iface_t;

typedef struct {
    const char *name;
    uint32_t    timeout_ms;   /* 该任务允许的最大心跳间隔 */
    uint32_t    last_feed_ms;
    uint32_t    missed;       /* 累计超期次数 */
    uint8_t     registered;
} iwdg_task_t;

typedef struct {
    const iwdg_iface_t *iface;
    uint8_t   enabled;
    uint8_t   prescaler;      /* 0..7 */
    uint16_t  reload;
    uint32_t  timeout_ms;     /* 计算出的实际超时 */
    iwdg_task_t task[IWDG_MAX_SUPERVISED];
    uint8_t   task_count;
    uint32_t  feeds;          /* 成功喂狗次数 */
    uint32_t  blocked_feeds;  /* 因任务超期而拒绝喂狗的次数 */
    uint32_t  resets_detected;
    int8_t    last_fault_task;/* 触发复位的任务下标 */
    uint8_t   warned[IWDG_MAX_SUPERVISED];
} iwdg_t;

int  iwdg_init(iwdg_t *dev, const iwdg_iface_t *iface, uint32_t timeout_ms);
int  iwdg_start(iwdg_t *dev);
int  iwdg_set_timeout(iwdg_t *dev, uint32_t timeout_ms);
uint32_t iwdg_compute_timeout_ms(uint8_t prescaler, uint16_t reload, uint32_t lsi_hz);
/* 找出满足 timeout_ms 的最小 (prescaler, reload) 组合 */
int  iwdg_pick_config(uint32_t timeout_ms, uint32_t lsi_hz,
                      uint8_t *prescaler, uint16_t *reload, uint32_t *actual_ms);
int  iwdg_feed(iwdg_t *dev);            /* 直接喂狗（不检查任务） */
int  iwdg_register_task(iwdg_t *dev, const char *name, uint32_t timeout_ms);
/* 任务心跳上报（必须传调度器的虚拟时间，与 supervised_feed 用同一时间基准） */
int  iwdg_task_alive(iwdg_t *dev, const char *name, uint32_t now_ms);
/* 监督式喂狗：所有受监督任务都在期内才真正喂狗；返回 1 已喂 0 被拒绝 */
int  iwdg_supervised_feed(iwdg_t *dev, uint32_t now_ms);
int  iwdg_check_reset(iwdg_t *dev);      /* 检查上次复位是否由 IWDG 触发并取回罪魁 */
const char *iwdg_fault_task_name(const iwdg_t *dev);

#endif /* IWDG_H */
