/*
 * power_mgr.h -- 低功耗状态机 + 外设生命周期集中管理
 *
 * 手表最怕"忘了关外设"导致待机电流超标，所以这里把外设的
 * 上电/掉电/进入低功耗/退出低功耗四个动作集中登记，
 * 休眠时统一按序关闭、唤醒时统一按序重初始化，
 * 并把整条调用序列记录下来供测试断言（也可用于现场问题定位）。
 */
#ifndef POWER_MGR_H
#define POWER_MGR_H

#include <stdint.h>
#include <stddef.h>

typedef enum {
    POWER_ST_RUN = 0,     /* 全速运行：采集 + 显示 + 通信全开 */
    POWER_ST_IDLE,        /* 空闲：降频、关背光，通信保持 */
    POWER_ST_SLEEP,       /* 休眠：STOP 模式，只留 RTC / IWDG / 唤醒源 */
    POWER_ST_WAKEUP       /* 唤醒过渡：外设重初始化并稳定 */
} power_state_t;

typedef enum {
    PERIPH_MPU6050 = 0,   /* 六轴：休眠时降为 motion-detect 低功耗模式 */
    PERIPH_FT6236,        /* 电容触摸：休眠时进 monitor 模式，保留中断唤醒 */
    PERIPH_MLX90615,      /* 红外测温：休眠时进 sleep */
    PERIPH_OLED,          /* 显示：休眠关屏 + 断背光 */
    PERIPH_BATTERY_ADC,   /* 电池采样：休眠关 ADC */
    PERIPH_RTC,           /* RTC：休眠必须保持（唤醒源） */
    PERIPH_IWDG,          /* 独立看门狗：休眠必须保持（安全） */
    PERIPH_WIFI,          /* Wi-Fi 模组：休眠进 DTIM/掉电，保留唤醒能力 */
    PERIPH_MOTOR,         /* 振动马达：休眠必须关，否则漏电 */
    PERIPH_COUNT
} periph_id_t;

typedef enum {
    WAKE_SRC_NONE = 0,
    WAKE_SRC_RTC_ALARM,
    WAKE_SRC_TOUCH,
    WAKE_SRC_KEY,
    WAKE_SRC_MOTION,
    WAKE_SRC_UART,
    WAKE_SRC_WIFI,
    WAKE_SRC_USB,
    WAKE_SRC_UNKNOWN
} wake_src_t;

typedef int (*periph_action_fn)(void);

typedef struct {
    const char       *name;
    periph_action_fn  init;            /* 上电初始化 */
    periph_action_fn  deinit;          /* 完全关闭 */
    periph_action_fn  enter_low_power; /* 进入低功耗（保留唤醒能力） */
    periph_action_fn  exit_low_power;  /* 退出低功耗并恢复 */
    uint8_t           always_on;       /* 1 = 休眠期间也不关闭 */
    uint8_t           registered;
} periph_ops_t;

/* 事件日志：每条记录一次动作调用，测试断言"休眠后关、唤醒后重初始化" */
typedef enum {
    PWR_EV_REGISTER = 0,
    PWR_EV_STATE,
    PWR_EV_INIT,
    PWR_EV_DEINIT,
    PWR_EV_ENTER_LP,
    PWR_EV_EXIT_LP,
    PWR_EV_WAKE_SRC,
    PWR_EV_SLEEP_ENTER,
    PWR_EV_SLEEP_EXIT
} pwr_evt_kind_t;

typedef struct {
    pwr_evt_kind_t kind;
    int            periph;   /* periph_id_t 或 -1 */
    int            arg;      /* 状态 / 唤醒源 */
} pwr_evt_t;

#define PWR_EVT_LOG_MAX 128

typedef struct {
    power_state_t state;
    periph_ops_t  ops[PERIPH_COUNT];
    uint8_t       active[PERIPH_COUNT];   /* 当前是否已上电 */
    uint8_t       in_low_power[PERIPH_COUNT];
    uint32_t      now_ms;
    uint32_t      state_since_ms;
    uint32_t      last_activity_ms;
    wake_src_t    last_wake_src;
    uint32_t      sleep_count;
    uint32_t      wake_count;
    uint32_t      total_sleep_ms;
    uint32_t      total_run_ms;
    uint32_t      idle_since_ms;
    uint8_t       auto_sleep_enabled;
    uint32_t      sleep_after_idle_ms;

    /* 事件日志（环形） */
    pwr_evt_t log[PWR_EVT_LOG_MAX];
    uint16_t  log_head;
    uint16_t  log_count;
    /* 计数：断言用 */
    uint32_t  init_calls[PERIPH_COUNT];
    uint32_t  deinit_calls[PERIPH_COUNT];
    uint32_t  enter_lp_calls[PERIPH_COUNT];
    uint32_t  exit_lp_calls[PERIPH_COUNT];
} power_mgr_t;

void power_mgr_init(power_mgr_t *pm, uint32_t now_ms);
int  power_mgr_register(power_mgr_t *pm, periph_id_t id, const char *name,
                        periph_action_fn init, periph_action_fn deinit,
                        periph_action_fn enter_lp, periph_action_fn exit_lp,
                        uint8_t always_on);
/* 上电所有外设（按注册顺序） */
int  power_mgr_periph_init(power_mgr_t *pm, periph_id_t id);
int  power_mgr_periph_init_all(power_mgr_t *pm);

void power_mgr_note_activity(power_mgr_t *pm, uint32_t now_ms);
void power_mgr_set_auto_sleep(power_mgr_t *pm, uint8_t enable, uint32_t idle_ms);
/* 是否允许进入休眠：空闲够久 + 无任务在跑 + 无待确认补传 */
int  power_mgr_can_sleep(const power_mgr_t *pm, uint32_t now_ms, uint32_t busy_tasks);

/* 进入休眠：关外设 -> 进低功耗 -> 记录状态（不阻塞）*/
int  power_mgr_enter_sleep(power_mgr_t *pm, uint32_t now_ms, wake_src_t planned_src);
/* 唤醒：按序 exit_low_power -> deinit 的重新 init */
int  power_mgr_wakeup(power_mgr_t *pm, uint32_t now_ms, wake_src_t src);
/* 时间片推进：空闲超时自动休眠 */
int  power_mgr_tick(power_mgr_t *pm, uint32_t now_ms, uint32_t busy_tasks);

const char *power_state_name(power_state_t st);
const char *periph_name(periph_id_t id);
const char *wake_src_name(wake_src_t src);

/* 事件日志访问 */
uint16_t    power_mgr_log_count(const power_mgr_t *pm);
const pwr_evt_t *power_mgr_log_at(const power_mgr_t *pm, uint16_t i);  /* 0 = 最旧 */
void        power_mgr_log_clear(power_mgr_t *pm);

#endif /* POWER_MGR_H */
