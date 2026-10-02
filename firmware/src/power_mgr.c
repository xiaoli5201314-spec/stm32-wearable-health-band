/*
 * power_mgr.c -- 低功耗状态机 + 外设生命周期集中管理
 *
 * 状态迁移：
 *   RUN ──(空闲 > sleep_after_idle)──> SLEEP
 *   RUN <──(任意唤醒源)────────────── WAKEUP ──(外设就绪+稳定)──> RUN
 *   RUN ──(屏幕灭但通信保持)────────> IDLE ──(空闲更久)────────> SLEEP
 *
 * 进休眠顺序（先无关紧要的，后关键的）：
 *   马达 -> 测温 -> 显示 -> 电池 ADC -> 六轴(降为运动唤醒) -> 触摸(monitor) -> Wi-Fi
 * 唤醒顺序（先关键外设，后非关键）：
 *   Wi-Fi -> 触摸 -> 六轴 -> 电池 ADC -> 显示 -> 测温 -> 马达
 * 两条序列都被记录进事件日志，测试直接断言"关过 / 重初始化过"。
 */
#include "power_mgr.h"
#include "band_config.h"
#include "band_port.h"
#include <string.h>
#include <stdio.h>

/* 进休眠的处理顺序（下标即顺序） */
static const periph_id_t k_sleep_order[PERIPH_COUNT] = {
    PERIPH_MOTOR,
    PERIPH_MLX90615,
    PERIPH_OLED,
    PERIPH_BATTERY_ADC,
    PERIPH_MPU6050,
    PERIPH_FT6236,
    PERIPH_WIFI,
    PERIPH_RTC,
    PERIPH_IWDG
};

/* 唤醒的处理顺序 */
static const periph_id_t k_wake_order[PERIPH_COUNT] = {
    PERIPH_WIFI,
    PERIPH_FT6236,
    PERIPH_MPU6050,
    PERIPH_BATTERY_ADC,
    PERIPH_OLED,
    PERIPH_MLX90615,
    PERIPH_MOTOR,
    PERIPH_RTC,
    PERIPH_IWDG
};

static void pwr_log(power_mgr_t *pm, pwr_evt_kind_t kind, int periph, int arg)
{
    pwr_evt_t *e;

    if (pm == NULL) {
        return;
    }
    e = &pm->log[pm->log_head];
    e->kind = kind;
    e->periph = periph;
    e->arg = arg;
    pm->log_head = (uint16_t)((pm->log_head + 1u) % PWR_EVT_LOG_MAX);
    if (pm->log_count < PWR_EVT_LOG_MAX) {
        pm->log_count++;
    }
}

void power_mgr_log_clear(power_mgr_t *pm)
{
    if (pm == NULL) {
        return;
    }
    pm->log_head = 0u;
    pm->log_count = 0u;
}

uint16_t power_mgr_log_count(const power_mgr_t *pm)
{
    return (pm == NULL) ? 0u : pm->log_count;
}

const pwr_evt_t *power_mgr_log_at(const power_mgr_t *pm, uint16_t i)
{
    uint16_t start;

    if ((pm == NULL) || (i >= pm->log_count)) {
        return NULL;
    }
    start = (uint16_t)((pm->log_head + PWR_EVT_LOG_MAX - pm->log_count) % PWR_EVT_LOG_MAX);
    return &pm->log[(uint16_t)((start + i) % PWR_EVT_LOG_MAX)];
}

static void pwr_set_state(power_mgr_t *pm, power_state_t st, uint32_t now_ms)
{
    if (pm->state == st) {
        return;
    }
    pwr_log(pm, PWR_EV_STATE, -1, (int)st);
    pm->state = st;
    pm->state_since_ms = now_ms;
}

void power_mgr_init(power_mgr_t *pm, uint32_t now_ms)
{
    if (pm == NULL) {
        return;
    }
    memset(pm, 0, sizeof(*pm));
    pm->state = POWER_ST_RUN;
    pm->now_ms = now_ms;
    pm->state_since_ms = now_ms;
    pm->last_activity_ms = now_ms;
    pm->idle_since_ms = now_ms;
    pm->sleep_after_idle_ms = POWER_IDLE_TO_SLEEP_MS;
    pm->auto_sleep_enabled = 1u;
    pm->last_wake_src = WAKE_SRC_NONE;
}

int power_mgr_register(power_mgr_t *pm, periph_id_t id, const char *name,
                       periph_action_fn init, periph_action_fn deinit,
                       periph_action_fn enter_lp, periph_action_fn exit_lp,
                       uint8_t always_on)
{
    if ((pm == NULL) || (id >= PERIPH_COUNT)) {
        return -1;
    }
    pm->ops[id].name = (name != NULL) ? name : "periph";
    pm->ops[id].init = init;
    pm->ops[id].deinit = deinit;
    pm->ops[id].enter_low_power = enter_lp;
    pm->ops[id].exit_low_power = exit_lp;
    pm->ops[id].always_on = (always_on != 0u) ? 1u : 0u;
    pm->ops[id].registered = 1u;
    pwr_log(pm, PWR_EV_REGISTER, (int)id, 0);
    return 0;
}

int power_mgr_periph_init(power_mgr_t *pm, periph_id_t id)
{
    if ((pm == NULL) || (id >= PERIPH_COUNT) || (pm->ops[id].registered == 0u)) {
        return -1;
    }
    if (pm->ops[id].init != NULL) {
        if (pm->ops[id].init() != 0) {
            return -1;
        }
    }
    pm->active[id] = 1u;
    pm->in_low_power[id] = 0u;
    pm->init_calls[id]++;
    pwr_log(pm, PWR_EV_INIT, (int)id, 0);
    return 0;
}

int power_mgr_periph_init_all(power_mgr_t *pm)
{
    uint8_t i;
    int rc = 0;

    if (pm == NULL) {
        return -1;
    }
    for (i = 0u; i < (uint8_t)PERIPH_COUNT; i++) {
        if (pm->ops[i].registered != 0u) {
            if (power_mgr_periph_init(pm, (periph_id_t)i) != 0) {
                rc = -1;
            }
        }
    }
    return rc;
}

void power_mgr_note_activity(power_mgr_t *pm, uint32_t now_ms)
{
    if (pm == NULL) {
        return;
    }
    pm->last_activity_ms = now_ms;
    pm->now_ms = now_ms;
    if (pm->state == POWER_ST_IDLE) {
        pwr_set_state(pm, POWER_ST_RUN, now_ms);
    }
}

void power_mgr_set_auto_sleep(power_mgr_t *pm, uint8_t enable, uint32_t idle_ms)
{
    if (pm == NULL) {
        return;
    }
    pm->auto_sleep_enabled = (enable != 0u) ? 1u : 0u;
    if (idle_ms > 0u) {
        pm->sleep_after_idle_ms = idle_ms;
    }
}

int power_mgr_can_sleep(const power_mgr_t *pm, uint32_t now_ms, uint32_t busy_tasks)
{
    if (pm == NULL) {
        return 0;
    }
    if (pm->auto_sleep_enabled == 0u) {
        return 0;
    }
    if (pm->state == POWER_ST_SLEEP) {
        return 0;
    }
    if (busy_tasks > 0u) {
        return 0;   /* 还有任务在跑，不能睡 */
    }
    if ((now_ms - pm->last_activity_ms) < pm->sleep_after_idle_ms) {
        return 0;
    }
    return 1;
}

int power_mgr_enter_sleep(power_mgr_t *pm, uint32_t now_ms, wake_src_t planned_src)
{
    uint8_t i;

    if ((pm == NULL) || (pm->state == POWER_ST_SLEEP)) {
        return -1;
    }
    pwr_set_state(pm, POWER_ST_SLEEP, now_ms);
    pwr_log(pm, PWR_EV_SLEEP_ENTER, -1, (int)planned_src);
    pm->now_ms = now_ms;

    for (i = 0u; i < (uint8_t)PERIPH_COUNT; i++) {
        periph_id_t id = k_sleep_order[i];
        periph_ops_t *op = &pm->ops[id];

        if (op->registered == 0u) {
            continue;
        }
        if (op->always_on != 0u) {
            /* RTC / IWDG 必须保持工作，否则无法定时唤醒或失去安全兜底 */
            continue;
        }
        if (pm->active[id] == 0u) {
            continue;
        }
        if (op->enter_low_power != NULL) {
            (void)op->enter_low_power();
            pm->enter_lp_calls[id]++;
            pwr_log(pm, PWR_EV_ENTER_LP, (int)id, 0);
        }
        if (op->deinit != NULL) {
            (void)op->deinit();
            pm->deinit_calls[id]++;
            pwr_log(pm, PWR_EV_DEINIT, (int)id, 0);
        }
        pm->active[id] = 0u;
        pm->in_low_power[id] = 1u;
    }
    pm->sleep_count++;
    pm->idle_since_ms = now_ms;
    return 0;
}

int power_mgr_wakeup(power_mgr_t *pm, uint32_t now_ms, wake_src_t src)
{
    uint8_t i;
    uint32_t slept;

    if (pm == NULL) {
        return -1;
    }
    if (pm->state != POWER_ST_SLEEP) {
        /* 允许从 IDLE/RUN 直接调用（例如触摸唤醒后的亮屏），但不算一次完整休眠 */
        power_mgr_note_activity(pm, now_ms);
        pm->last_wake_src = src;
        pwr_log(pm, PWR_EV_WAKE_SRC, -1, (int)src);
        return 0;
    }

    slept = now_ms - pm->idle_since_ms;
    pm->total_sleep_ms += slept;
    pwr_set_state(pm, POWER_ST_WAKEUP, now_ms);
    pm->last_wake_src = src;
    pwr_log(pm, PWR_EV_WAKE_SRC, -1, (int)src);

    for (i = 0u; i < (uint8_t)PERIPH_COUNT; i++) {
        periph_id_t id = k_wake_order[i];
        periph_ops_t *op = &pm->ops[id];

        if (op->registered == 0u) {
            continue;
        }
        if (op->always_on != 0u) {
            continue;   /* 从未关闭，无需重新初始化 */
        }
        if (pm->in_low_power[id] == 0u) {
            continue;   /* 本来就没进低功耗，跳过 */
        }
        if (op->exit_low_power != NULL) {
            (void)op->exit_low_power();
            pm->exit_lp_calls[id]++;
            pwr_log(pm, PWR_EV_EXIT_LP, (int)id, 0);
        }
        /* 唤醒后必须重新初始化：休眠期间外设掉电/时钟关闭，寄存器已丢失 */
        if (op->init != NULL) {
            (void)op->init();
            pm->init_calls[id]++;
            pwr_log(pm, PWR_EV_INIT, (int)id, 1);   /* arg=1 表示唤醒重初始化 */
        }
        pm->active[id] = 1u;
        pm->in_low_power[id] = 0u;
    }

    /* 唤醒后需要一段稳定时间（时钟切换 + 传感器首帧丢弃），由上层延时 */
    pwr_set_state(pm, POWER_ST_RUN, now_ms);
    pm->last_activity_ms = now_ms;
    pm->wake_count++;
    pwr_log(pm, PWR_EV_SLEEP_EXIT, -1, (int)src);
    return 0;
}

int power_mgr_tick(power_mgr_t *pm, uint32_t now_ms, uint32_t busy_tasks)
{
    if (pm == NULL) {
        return -1;
    }
    pm->now_ms = now_ms;

    if (pm->state == POWER_ST_RUN) {
        pm->total_run_ms += (now_ms - pm->state_since_ms);
        pm->state_since_ms = now_ms;
    }

    if (pm->state == POWER_ST_SLEEP) {
        return 0;
    }

    /* 空闲一段时间后先降级到 IDLE（关背光、降频），再考虑休眠 */
    if ((pm->state == POWER_ST_RUN) &&
        ((now_ms - pm->last_activity_ms) >= 3000u) &&
        (busy_tasks == 0u)) {
        pwr_set_state(pm, POWER_ST_IDLE, now_ms);
    }

    if (power_mgr_can_sleep(pm, now_ms, busy_tasks) != 0) {
        return power_mgr_enter_sleep(pm, now_ms, WAKE_SRC_RTC_ALARM);
    }
    return 0;
}

const char *power_state_name(power_state_t st)
{
    switch (st) {
    case POWER_ST_RUN:    return "RUN";
    case POWER_ST_IDLE:   return "IDLE";
    case POWER_ST_SLEEP:  return "SLEEP";
    case POWER_ST_WAKEUP: return "WAKEUP";
    default:              return "?";
    }
}

const char *periph_name(periph_id_t id)
{
    switch (id) {
    case PERIPH_MPU6050:     return "MPU6050";
    case PERIPH_FT6236:      return "FT6236";
    case PERIPH_MLX90615:    return "MLX90615";
    case PERIPH_OLED:        return "OLED";
    case PERIPH_BATTERY_ADC: return "BAT_ADC";
    case PERIPH_RTC:         return "RTC";
    case PERIPH_IWDG:        return "IWDG";
    case PERIPH_WIFI:        return "WIFI";
    case PERIPH_MOTOR:       return "MOTOR";
    default:                 return "?";
    }
}

const char *wake_src_name(wake_src_t src)
{
    switch (src) {
    case WAKE_SRC_NONE:      return "NONE";
    case WAKE_SRC_RTC_ALARM: return "RTC_ALARM";
    case WAKE_SRC_TOUCH:     return "TOUCH";
    case WAKE_SRC_KEY:       return "KEY";
    case WAKE_SRC_MOTION:    return "MOTION";
    case WAKE_SRC_UART:      return "UART";
    case WAKE_SRC_WIFI:      return "WIFI";
    case WAKE_SRC_USB:       return "USB";
    default:                 return "UNKNOWN";
    }
}
