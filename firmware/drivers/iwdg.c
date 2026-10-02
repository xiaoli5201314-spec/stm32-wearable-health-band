/*
 * iwdg.c -- 独立看门狗 + 任务级喂狗监督
 *
 * 单纯"定时喂狗"只能发现死循环，发现不了"某个任务被饿死"。
 * 这里的做法是：每个受监督任务在固定周期内上报一次心跳，
 * 只有所有任务都在期内才真正写 IWDG_KR = 0xAAAA；
 * 一旦有任务超期，就停止喂狗，让 IWDG 在超时后复位整机，
 * 并把罪魁任务号写进 RTC 备份寄存器，复位后由 iwdg_check_reset() 取回，
 * 使现场问题可定位（这是量产产品很实用的一招）。
 *
 * IWDG 超时时间：T = (4 * 2^PR) * (RLR + 1) / f_LSI
 *   f_LSI ≈ 32kHz（RM0090 给的范围 17~47kHz，因此要留足余量：
 *   实际超时可能比计算值短 40%，配置值要按最坏情况校验）
 */
#include "iwdg.h"
#include "band_port.h"
#include <string.h>

uint32_t iwdg_compute_timeout_ms(uint8_t prescaler, uint16_t reload, uint32_t lsi_hz)
{
    uint64_t div;
    uint64_t ticks;

    if ((prescaler > 7u) || (lsi_hz == 0u)) {
        return 0u;
    }
    div = (uint64_t)4u << prescaler;                 /* 4 * 2^PR */
    ticks = div * ((uint64_t)reload + 1u);           /* LSI 周期数 */
    /* ms = ticks * 1000 / f_LSI */
    return (uint32_t)((ticks * 1000u) / (uint64_t)lsi_hz);
}

int iwdg_pick_config(uint32_t timeout_ms, uint32_t lsi_hz,
                     uint8_t *prescaler, uint16_t *reload, uint32_t *actual_ms)
{
    uint8_t pr;
    uint32_t best_pr = 0u;
    uint32_t best_rlr = 0u;
    uint32_t best_ms = 0u;
    int found = 0;

    if (lsi_hz == 0u) {
        return -1;
    }
    /* 从最小预分频开始找：预分频越小，超时越精细 */
    for (pr = 0u; pr <= 7u; pr++) {
        uint64_t div = (uint64_t)4u << pr;
        /* 需要的 LSI 周期数 = timeout_ms * f_LSI / 1000 */
        uint64_t need = ((uint64_t)timeout_ms * (uint64_t)lsi_hz) / 1000u;
        uint64_t rlr = (need + div - 1u) / div;   /* 向上取整 */

        if (rlr == 0u) {
            rlr = 1u;
        }
        if (rlr > 4096u) {
            continue;   /* RLR 是 12bit，最大 4095 */
        }
        best_pr = pr;
        best_rlr = (uint32_t)(rlr - 1u);
        best_ms = iwdg_compute_timeout_ms((uint8_t)pr, (uint16_t)(rlr - 1u), lsi_hz);
        found = 1;
        break;
    }
    if (found == 0) {
        return -1;
    }
    if (prescaler != NULL) {
        *prescaler = (uint8_t)best_pr;
    }
    if (reload != NULL) {
        *reload = (uint16_t)best_rlr;
    }
    if (actual_ms != NULL) {
        *actual_ms = best_ms;
    }
    return 0;
}

static int iwdg_rd(iwdg_t *dev, uint32_t off, uint32_t *val)
{
    if ((dev == NULL) || (dev->iface == NULL) || (dev->iface->reg_read == NULL)) {
        return -1;
    }
    return dev->iface->reg_read(off, val);
}

static int iwdg_wr(iwdg_t *dev, uint32_t off, uint32_t val)
{
    if ((dev == NULL) || (dev->iface == NULL) || (dev->iface->reg_write == NULL)) {
        return -1;
    }
    return dev->iface->reg_write(off, val);
}

int iwdg_init(iwdg_t *dev, const iwdg_iface_t *iface, uint32_t timeout_ms)
{
    if ((dev == NULL) || (iface == NULL)) {
        return -1;
    }
    memset(dev, 0, sizeof(*dev));
    dev->iface = iface;
    dev->last_fault_task = -1;
    if (iwdg_set_timeout(dev, timeout_ms) != 0) {
        return -1;
    }
    return 0;
}

int iwdg_set_timeout(iwdg_t *dev, uint32_t timeout_ms)
{
    uint8_t pr = 0u;
    uint16_t rlr = 0u;
    uint32_t actual = 0u;
    uint32_t sr = 0u;
    uint32_t guard;

    if (dev == NULL) {
        return -1;
    }
    if (iwdg_pick_config(timeout_ms, BAND_LSI_HZ, &pr, &rlr, &actual) != 0) {
        return -1;
    }
    /* 1) 解锁 PR/RLR（0x5555） */
    if (iwdg_wr(dev, IWDG_OFF_KR, IWDG_KEY_UNLOCK) != 0) {
        return -1;
    }
    /* 2) 写预分频与重装载值 */
    if (iwdg_wr(dev, IWDG_OFF_PR, (uint32_t)pr) != 0) {
        return -1;
    }
    if (iwdg_wr(dev, IWDG_OFF_RLR, (uint32_t)rlr) != 0) {
        return -1;
    }
    /* 3) 等 SR 的 PVU/RVU 清零，表示新值已生效（必须等，否则下一次喂狗用的是旧值） */
    for (guard = 0u; guard < 1000u; guard++) {
        if (iwdg_rd(dev, IWDG_OFF_SR, &sr) != 0) {
            return -1;
        }
        if ((sr & (IWDG_SR_PVU | IWDG_SR_RVU)) == 0u) {
            break;
        }
        band_delay_ms(1u);
    }
    dev->prescaler = pr;
    dev->reload = rlr;
    dev->timeout_ms = actual;
    return 0;
}

int iwdg_start(iwdg_t *dev)
{
    if (dev == NULL) {
        return -1;
    }
    if (iwdg_wr(dev, IWDG_OFF_KR, IWDG_KEY_START) != 0) {
        return -1;
    }
    /* 启动后立刻喂一次，给初始化流程留足时间 */
    if (iwdg_feed(dev) != 0) {
        return -1;
    }
    dev->enabled = 1u;
    return 0;
}

int iwdg_feed(iwdg_t *dev)
{
    if (dev == NULL) {
        return -1;
    }
    if (iwdg_wr(dev, IWDG_OFF_KR, IWDG_KEY_RELOAD) != 0) {
        return -1;
    }
    dev->feeds++;
    return 0;
}

int iwdg_register_task(iwdg_t *dev, const char *name, uint32_t timeout_ms)
{
    iwdg_task_t *t;

    if ((dev == NULL) || (name == NULL) || (dev->task_count >= IWDG_MAX_SUPERVISED)) {
        return -1;
    }
    t = &dev->task[dev->task_count];
    t->name = name;
    t->timeout_ms = timeout_ms;
    t->last_feed_ms = (dev->iface->now_ms != NULL) ? dev->iface->now_ms() : 0u;
    t->missed = 0u;
    t->registered = 1u;
    dev->task_count++;
    dev->warned[dev->task_count - 1u] = 0u;
    return (int)(dev->task_count - 1u);
}

static iwdg_task_t *iwdg_find(iwdg_t *dev, const char *name)
{
    uint8_t i;

    for (i = 0u; i < dev->task_count; i++) {
        if ((dev->task[i].registered != 0u) && (dev->task[i].name != NULL) &&
            (strcmp(dev->task[i].name, name) == 0)) {
            return &dev->task[i];
        }
    }
    return NULL;
}

int iwdg_task_alive(iwdg_t *dev, const char *name, uint32_t now_ms)
{
    iwdg_task_t *t;

    if ((dev == NULL) || (name == NULL)) {
        return -1;
    }
    t = iwdg_find(dev, name);
    if (t == NULL) {
        return -1;
    }
    t->last_feed_ms = now_ms;
    dev->warned[(uint8_t)(t - dev->task)] = 0u;
    return 0;
}

int iwdg_supervised_feed(iwdg_t *dev, uint32_t now_ms)
{
    uint8_t i;
    int all_ok = 1;
    int8_t fault = -1;

    if (dev == NULL) {
        return 0;
    }
    for (i = 0u; i < dev->task_count; i++) {
        iwdg_task_t *t = &dev->task[i];
        if (t->registered == 0u) {
            continue;
        }
        if ((uint32_t)(now_ms - t->last_feed_ms) > t->timeout_ms) {
            all_ok = 0;
            fault = (int8_t)i;
            t->missed++;
            break;   /* 只记录第一个超期的任务，它就是"罪魁" */
        }
    }

    if (all_ok == 0) {
        /* 关键决策：不喂狗，让硬件复位，并在备份寄存器里留下线索 */
        dev->blocked_feeds++;
        dev->last_fault_task = fault;
        if (dev->iface->bkp_write != NULL) {
            dev->iface->bkp_write(IWDG_BKP_FAULT_IDX, (uint32_t)fault);
        }
        return 0;
    }

    if (iwdg_feed(dev) != 0) {
        return 0;
    }
    return 1;
}

int iwdg_check_reset(iwdg_t *dev)
{
    uint32_t cause;
    uint32_t fault;

    if ((dev == NULL) || (dev->iface == NULL) || (dev->iface->reset_cause == NULL)) {
        return 0;
    }
    cause = dev->iface->reset_cause();
    if (cause == 0u) {
        return 0;
    }
    dev->resets_detected++;
    fault = (dev->iface->bkp_read != NULL) ? dev->iface->bkp_read(IWDG_BKP_FAULT_IDX) : 0u;
    if (fault < IWDG_MAX_SUPERVISED) {
        dev->last_fault_task = (int8_t)fault;
        if (dev->task[fault].name != NULL) {
            dev->task[fault].missed++;
        }
    }
    if (dev->iface->clear_reset_cause != NULL) {
        dev->iface->clear_reset_cause();
    }
    return 1;
}

const char *iwdg_fault_task_name(const iwdg_t *dev)
{
    if ((dev == NULL) || (dev->last_fault_task < 0) ||
        ((uint8_t)dev->last_fault_task >= dev->task_count)) {
        return "(unknown)";
    }
    return (dev->task[(uint8_t)dev->last_fault_task].name != NULL)
               ? dev->task[(uint8_t)dev->last_fault_task].name
               : "(unnamed)";
}
