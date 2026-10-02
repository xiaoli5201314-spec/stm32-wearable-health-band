/*
 * app_main.c -- 应用主程序：任务划分、同步原语、界面、低功耗
 *
 * 任务划分（详见 docs/DESIGN.md）：
 *   | 任务   | 优先级 | 周期  | 职责                                       |
 *   |--------|--------|-------|--------------------------------------------|
 *   | SENSOR | 5      | 20ms  | MPU6050/MLX90615 采集、计步、心率、姿态      |
 *   | KEY    | 4      | 10ms  | FT6236 触摸 + 物理按键 + 手势 -> 邮箱/信号量 |
 *   | UI     | 3      | 50ms  | 多级菜单、实时数据界面、SSD1306 刷新          |
 *   | COMM   | 2      | 20ms  | WebSocket 收发、补传、远程参数下发            |
 *   | POWER  | 1      | 1000ms| 低功耗状态机、电池采样、看门狗监督喂狗        |
 *
 * 同步原语：
 *   - touch_sem   : KEY 任务释放，UI 任务等待（触摸事件驱动重绘，不轮询刷屏）
 *   - data_mbox   : SENSOR 任务投递体征快照，UI/COMM 任务消费
 *   - cmd_mbox    : UI/COMM 投递命令（切换页面、下发参数生效等）
 *
 * 低功耗：POWER 任务判定空闲超时后调用 power_mgr_enter_sleep()，
 * 由 RTC 唤醒定时器 / 触摸中断 / 六轴运动中断唤醒，
 * 唤醒后按序重新初始化外设（power_mgr 保证顺序并记录事件日志）。
 */
#include "band_config.h"
#include "band_port.h"
#include "band_platform.h"
#include "band_app.h"
#include "task_sched.h"
#include "power_mgr.h"
#include "step_counter.h"
#include "sensor_iface.h"
#include "frame_codec.h"
#include "uplink.h"
#include "mpu6050.h"
#include "ft6236.h"
#include "mlx90615.h"
#include "oled_ssd1306.h"
#include "battery.h"
#include "rtc.h"
#include "iwdg.h"

#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/* 应用上下文                                                          */
/* ------------------------------------------------------------------ */
typedef enum {
    PAGE_MAIN = 0,     /* 心率 + 步数 */
    PAGE_TEMP,         /* 体温 */
    PAGE_BATTERY,      /* 电量 */
    PAGE_NET,          /* 网络状态 */
    PAGE_MENU,         /* 多级菜单第一级 */
    PAGE_COUNT
} ui_page_t;

/* 邮箱消息：定长 16 字节（必须与 MBOX_MSG_LEN 一致，否则 memcpy 会越界读栈） */
typedef struct {
    uint8_t  kind;      /* 0=体征快照 1=触摸事件 2=按键 3=命令 */
    uint8_t  arg0;
    uint8_t  arg1;
    uint8_t  arg2;
    uint32_t u32;
    uint16_t u16;
    uint32_t pad32;     /* 补齐到 16 字节 */
} band_msg_t;

/* 编译期断言：消息结构体必须正好是邮箱的消息长度 */
typedef char band_msg_size_must_match_mbox[(sizeof(band_msg_t) == MBOX_MSG_LEN) ? 1 : -1];

#define MSG_KIND_SNAPSHOT 0u
#define MSG_KIND_TOUCH    1u
#define MSG_KIND_KEY      2u
#define MSG_KIND_CMD      3u

struct band_app_s {
    os_sched_t     sched;
    power_mgr_t    power;
    band_uplink_t  uplink;
    /* 驱动实例 */
    mpu6050_t      imu;
    ft6236_dev_t   touch_dev;
    ft6236_t       touch;
    mlx90615_t     ir;
    ssd1306_t      oled;
    battery_dev_t  bat;
    rtc_dev_t      rtc;
    iwdg_t         iwdg;
    step_counter_t step;
    step_counter_t hr_det;   /* 无 PPG 硬件时的心率估计器（SCG 微动峰值间隔） */
    /* 同步原语 */
    os_sem_t       touch_sem;
    os_mbox_t      data_mbox;
    os_mbox_t      cmd_mbox;
    /* 状态 */
    band_snapshot_t snap;
    ui_page_t      page;
    uint8_t        menu_index;
    uint8_t        backlight_on;
    uint8_t        last_key_level;
    uint32_t       last_hr_ms;
    uint32_t       last_ui_ms;
    uint32_t       last_upload_ms;
    uint32_t       last_activity_ms;
    uint8_t        sleeping;
    wake_src_t     last_wake;
    uint8_t        motor_on;
    uint32_t       motor_off_ms;
    /* 统计 */
    uint32_t       sensor_runs;
    uint32_t       ui_runs;
    uint32_t       key_runs;
    uint32_t       comm_runs;
    uint32_t       power_runs;
    uint32_t       uploads;
    uint8_t        mpu_ok;
    uint8_t        touch_ok;
    uint8_t        ir_ok;
    uint8_t        oled_ok;
};

static band_app_t g_app;
/* ------------------------------------------------------------------ */
/* 外设生命周期（注册给 power_mgr，休眠关闭、唤醒按序重初始化）          */
/* ------------------------------------------------------------------ */
static int periph_mpu_init(void)
{
    if (mpu6050_init(&g_app.imu, sensor_bus_get(), BAND_I2C_ADDR_MPU6050) != SENSOR_OK) {
        g_app.mpu_ok = 0u;
        return -1;
    }
    g_app.mpu_ok = 1u;
    return 0;
}

static int periph_mpu_deinit(void)
{
    /* 完全关闭：进 SLEEP，电流从 3.9mA 降到 5uA */
    return (mpu6050_sleep(&g_app.imu, 1u) == SENSOR_OK) ? 0 : -1;
}

static int periph_mpu_enter_lp(void)
{
    /* 低功耗但保留唤醒能力：使能运动检测中断 */
    (void)mpu6050_enable_motion_wakeup(&g_app.imu, 80u, 20u);
    return (mpu6050_sleep(&g_app.imu, 0u) == SENSOR_OK) ? 0 : -1;
}

static int periph_mpu_exit_lp(void)
{
    uint8_t st = 0u;
    (void)mpu6050_read_int_status(&g_app.imu, &st);
    return 0;
}

static int periph_touch_init(void)
{
    if (ft6236_init(&g_app.touch_dev, sensor_bus_get(), BAND_I2C_ADDR_FT6236, 128u, 128u) != SENSOR_OK) {
        g_app.touch_ok = 0u;
        return -1;
    }
    g_app.touch_ok = 1u;
    return 0;
}

static int periph_touch_deinit(void)
{
    /* 深度休眠：关掉触摸供电（由 GPIO 控制的 LDO_EN 实现） */
    return (ft6236_set_power_mode(&g_app.touch_dev, 1u) == SENSOR_OK) ? 0 : -1;
}

static int periph_touch_enter_lp(void)
{
    /* monitor 模式：约 30ms 扫一次，触摸时拉中断脚唤醒 MCU */
    return (ft6236_set_power_mode(&g_app.touch_dev, 1u) == SENSOR_OK) ? 0 : -1;
}

static int periph_touch_exit_lp(void)
{
    return (ft6236_set_power_mode(&g_app.touch_dev, 0u) == SENSOR_OK) ? 0 : -1;
}

static int periph_ir_init(void)
{
    if (mlx90615_init(&g_app.ir, sensor_bus_get(), BAND_I2C_ADDR_MLX90615) != SENSOR_OK) {
        g_app.ir_ok = 0u;
        return -1;
    }
    g_app.ir_ok = 1u;
    return 0;
}

static int periph_ir_deinit(void)
{
    /* MLX90615 无软件关断脚，靠外部 MOS 断供电；这里只把状态置为不可用 */
    g_app.ir.online = 0u;
    return 0;
}

static int periph_ir_enter_lp(void)
{
    g_app.ir.online = 0u;
    return 0;
}

static int periph_ir_exit_lp(void)
{
    /* 红外器件上电后需要一次完整读取周期（约 100ms）才出有效值 */
    g_app.ir.online = 1u;
    return 0;
}

static int periph_oled_init(void)
{
    if (ssd1306_init(&g_app.oled, sensor_bus_get(), BAND_I2C_ADDR_SSD1306) != SENSOR_OK) {
        g_app.oled_ok = 0u;
        return -1;
    }
    g_app.oled_ok = 1u;
    return 0;
}

static int periph_oled_deinit(void)
{
    return (ssd1306_sleep(&g_app.oled, 1u) == SENSOR_OK) ? 0 : -1;
}

static int periph_oled_enter_lp(void)
{
    (void)ssd1306_sleep(&g_app.oled, 1u);
    g_app.backlight_on = 0u;
    return 0;
}

static int periph_oled_exit_lp(void)
{
    (void)ssd1306_sleep(&g_app.oled, 0u);
    g_app.backlight_on = 1u;
    return 0;
}

static int periph_bat_init(void)
{
    return (battery_init(&g_app.bat, sensor_bus_get()) == SENSOR_OK) ? 0 : -1;
}

static int periph_bat_deinit(void)
{
    return 0;   /* ADC 已在平台层关闭采样 */
}

static int periph_bat_enter_lp(void)
{
    return 0;
}

static int periph_bat_exit_lp(void)
{
    return 0;
}

static int periph_rtc_init(void)
{
    return (rtc_init(&g_app.rtc, rtc_platform_iface()) == 0) ? 0 : -1;
}

static int periph_rtc_deinit(void)
{
    return 0;   /* always_on，不会被调用 */
}

static int periph_iwdg_init(void)
{
    if (iwdg_init(&g_app.iwdg, iwdg_platform_iface(), 2000u) != 0) {
        return -1;
    }
    /* 任务级监督：每个任务的喂狗预算 = 周期 * 3，留 3 倍余量 */
    (void)iwdg_register_task(&g_app.iwdg, "SENSOR", 100u);
    (void)iwdg_register_task(&g_app.iwdg, "KEY", 60u);
    (void)iwdg_register_task(&g_app.iwdg, "UI", 300u);
    (void)iwdg_register_task(&g_app.iwdg, "COMM", 300u);
    (void)iwdg_register_task(&g_app.iwdg, "POWER", 3000u);
    return iwdg_start(&g_app.iwdg);
}

static int periph_iwdg_deinit(void)
{
    /* IWDG 一旦启动就无法软件关闭（这是它的设计目的），always_on=1 所以不会被调用 */
    return 0;
}

static int periph_wifi_init(void)
{
    net_platform_init();
    return 0;
}

static int periph_wifi_deinit(void)
{
    uplink_disconnect(&g_app.uplink);
    return 0;
}

static int periph_wifi_enter_lp(void)
{
    /* 保活连接但降频：不主动断开，靠模组 DTIM 省电 */
    return 0;
}

static int periph_wifi_exit_lp(void)
{
    return 0;
}

static int periph_motor_init(void)
{
    g_app.motor_on = 0u;
    return 0;
}

static int periph_motor_deinit(void)
{
    g_app.motor_on = 0u;
    return 0;
}

static int periph_motor_enter_lp(void)
{
    g_app.motor_on = 0u;
    return 0;
}

static int periph_motor_exit_lp(void)
{
    g_app.motor_on = 0u;
    return 0;
}

/* ------------------------------------------------------------------ */
/* 业务辅助                                                            */
/* ------------------------------------------------------------------ */
static void motor_vibrate(uint32_t now_ms, uint16_t ms)
{
    if (g_app.uplink.params.motor_enable == 0u) {
        return;
    }
    g_app.motor_on = 1u;
    g_app.motor_off_ms = now_ms + ms;
}

/* 简易心率估计：用加速度/陀螺的周期性抖动做峰值间隔统计。
 * 真实产品用 PPG 光电容积脉搏波；本板没有 PPG 硬件时，用同一套
 * 峰值检测（不应期 250ms 恰好对应 240bpm 上限）对腕部微动做 SCG
 * 心率估计，仅在静止时更新，避免步态谐波被误当成心搏。 */
static uint8_t hr_from_rr(uint16_t rr_ms)
{
    if (rr_ms < 250u) {
        return 0u;
    }
    return (uint8_t)(60000u / rr_ms);
}

static void ui_draw_main(void)
{
    char line[24];

    ssd1306_clear(&g_app.oled);
    ssd1306_draw_string(&g_app.oled, 0, 0, "HEART RATE", 1);
    (void)snprintf(line, sizeof(line), "%u", (unsigned)g_app.snap.heart_rate);
    ssd1306_draw_string_2x(&g_app.oled, 0, 12, line, 1);
    ssd1306_draw_string(&g_app.oled, 34, 18, "bpm", 1);
    /* 心形/脉搏示意条：随心率变化长度 */
    ssd1306_draw_hline(&g_app.oled, 0, 30, (int16_t)(g_app.snap.heart_rate / 4u), 1);
    (void)snprintf(line, sizeof(line), "STEP %u", (unsigned)g_app.snap.steps);
    ssd1306_draw_string(&g_app.oled, 0, 40, line, 1);
    (void)snprintf(line, sizeof(line), "%u%% %umV",
                   (unsigned)g_app.snap.battery_pct, (unsigned)g_app.snap.battery_mv);
    ssd1306_draw_string(&g_app.oled, 0, 54, line, 1);
}

static void ui_draw_temp(void)
{
    char line[24];
    int whole = (int)g_app.snap.body_temp_c;
    int frac = (int)((g_app.snap.body_temp_c - (float)whole) * 10.0f);

    if (frac < 0) {
        frac = -frac;
    }
    ssd1306_clear(&g_app.oled);
    ssd1306_draw_string(&g_app.oled, 0, 0, "BODY TEMP", 1);
    (void)snprintf(line, sizeof(line), "%d.%d C", whole, frac);
    ssd1306_draw_string_2x(&g_app.oled, 0, 16, line, 1);
    (void)snprintf(line, sizeof(line), "AMB %d C", (int)g_app.snap.ambient_temp_c);
    ssd1306_draw_string(&g_app.oled, 0, 42, line, 1);
    ssd1306_draw_string(&g_app.oled, 0, 54, g_app.ir_ok ? "IR OK" : "IR N/A", 1);
}

static void ui_draw_battery(void)
{
    char line[24];
    int w = (int)((g_app.snap.battery_pct * 100u) / 100u);

    ssd1306_clear(&g_app.oled);
    ssd1306_draw_string(&g_app.oled, 0, 0, "BATTERY", 1);
    ssd1306_draw_rect(&g_app.oled, 0, 16, 104, 20, 1);
    ssd1306_draw_hline(&g_app.oled, 104, 22, 4, 1);
    ssd1306_draw_hline(&g_app.oled, 104, 29, 4, 1);
    if (w > 0) {
        ssd1306_draw_hline(&g_app.oled, 2, 18, (int16_t)w, 1);
        /* 用竖线扫出实心效果 */
        {
            int16_t i;
            for (i = 19; i < 34; i++) {
                ssd1306_draw_hline(&g_app.oled, 2, i, (int16_t)w, 1);
            }
        }
    }
    (void)snprintf(line, sizeof(line), "%u%%  %umV",
                   (unsigned)g_app.snap.battery_pct, (unsigned)g_app.snap.battery_mv);
    ssd1306_draw_string(&g_app.oled, 0, 42, line, 1);
    ssd1306_draw_string(&g_app.oled, 0, 54, g_app.snap.charging ? "CHARGING" : "DISCHARGE", 1);
}

static void ui_draw_net(void)
{
    char line[32];

    ssd1306_clear(&g_app.oled);
    ssd1306_draw_string(&g_app.oled, 0, 0, "LINK", 1);
    ssd1306_draw_string(&g_app.oled, 0, 12, uplink_state_str(&g_app.uplink), 1);
    (void)snprintf(line, sizeof(line), "TX %u RX %u",
                   (unsigned)g_app.uplink.st.tx_frames, (unsigned)g_app.uplink.st.rx_frames);
    ssd1306_draw_string(&g_app.oled, 0, 26, line, 1);
    (void)snprintf(line, sizeof(line), "PEND %u", (unsigned)uplink_pending_count(&g_app.uplink));
    ssd1306_draw_string(&g_app.oled, 0, 38, line, 1);
    (void)snprintf(line, sizeof(line), "RECONN %u", (unsigned)g_app.uplink.st.reconnects);
    ssd1306_draw_string(&g_app.oled, 0, 50, line, 1);
}

static void ui_draw_menu(void)
{
    static const char *items[] = { "Heart Rate", "Body Temp", "Battery", "Network", "Back" };

    ssd1306_clear(&g_app.oled);
    ssd1306_draw_string(&g_app.oled, 0, 0, "== MENU ==", 1);
    {
        uint8_t i;
        for (i = 0u; i < 5u; i++) {
            char line[24];
            (void)snprintf(line, sizeof(line), "%c %s",
                           (i == g_app.menu_index) ? '>' : ' ', items[i]);
            ssd1306_draw_string(&g_app.oled, 0, (int16_t)(12 + (i * 10)), line, 1);
        }
    }
}

static void ui_render(void)
{
    switch (g_app.page) {
    case PAGE_MAIN:    ui_draw_main();    break;
    case PAGE_TEMP:    ui_draw_temp();    break;
    case PAGE_BATTERY: ui_draw_battery(); break;
    case PAGE_NET:     ui_draw_net();     break;
    case PAGE_MENU:    ui_draw_menu();    break;
    default:           ui_draw_main();    break;
    }
    (void)ssd1306_flush(&g_app.oled);
}

/* 触摸/按键事件 -> 页面切换 */
static void ui_handle_input(uint8_t kind, uint8_t arg0, uint8_t arg1)
{
    if (kind == MSG_KIND_TOUCH) {
        if (arg0 == 1u) {
            /* 单击 */
            if (g_app.page == PAGE_MENU) {
                if (g_app.menu_index == 0u) {
                    g_app.page = PAGE_MAIN;
                } else if (g_app.menu_index == 1u) {
                    g_app.page = PAGE_TEMP;
                } else if (g_app.menu_index == 2u) {
                    g_app.page = PAGE_BATTERY;
                } else if (g_app.menu_index == 3u) {
                    g_app.page = PAGE_NET;
                } else {
                    g_app.page = PAGE_MAIN;
                }
            } else {
                g_app.page = PAGE_MENU;
                g_app.menu_index = 0u;
            }
        } else if (arg0 == 2u) {
            /* 左右滑动切换页面 */
            if (arg1 == 1u) {
                g_app.page = (ui_page_t)((g_app.page + 1u) % PAGE_COUNT);
            } else {
                g_app.page = (ui_page_t)((g_app.page + PAGE_COUNT - 1u) % PAGE_COUNT);
            }
            g_app.menu_index = 0u;
        } else if (arg0 == 3u) {
            /* 上下滑动移动菜单光标 */
            if (g_app.page == PAGE_MENU) {
                if (arg1 == 1u) {
                    g_app.menu_index = (uint8_t)((g_app.menu_index + 1u) % 5u);
                } else {
                    g_app.menu_index = (uint8_t)((g_app.menu_index + 4u) % 5u);
                }
            }
        } else if (arg0 == 4u) {
            /* 长按：震动反馈 */
            motor_vibrate(sched_millis(&g_app.sched), 80u);
        }
    } else if (kind == MSG_KIND_KEY) {
        if (arg0 == 1u) {
            g_app.page = (ui_page_t)((g_app.page + 1u) % PAGE_COUNT);
        }
    }
}

/* ------------------------------------------------------------------ */
/* 任务实现                                                            */
/* ------------------------------------------------------------------ */
static void task_sensor(void *arg)
{
    uint32_t now;
    mpu6050_scaled_t s;
    (void)arg;
    now = sched_millis(&g_app.sched);
    g_app.sensor_runs++;

    /* 1) 六轴：读原始数据 -> 计步 -> 姿态融合 */
    if (g_app.mpu_ok != 0u) {
        if (mpu6050_read_scaled(&g_app.imu, &s) == SENSOR_OK) {
            int32_t ax = (int32_t)(s.ax_g * 1000.0f);
            int32_t ay = (int32_t)(s.ay_g * 1000.0f);
            int32_t az = (int32_t)(s.az_g * 1000.0f);
            uint32_t steps_before = (uint32_t)g_app.snap.steps;
            (void)step_counter_feed(&g_app.step, ax, ay, az, now);
            g_app.snap.steps = (uint16_t)step_counter_steps(&g_app.step);

            /* 只有"本采样没走出一步"时才更新心率：走动时步态谐波会污染估计 */
            if (g_app.snap.steps == steps_before) {
                if (step_counter_feed(&g_app.hr_det, ax, ay, az, now) != 0) {
                    uint32_t spm = g_app.hr_det.cadence_spm;
                    if ((spm >= 40u) && (spm <= 200u)) {
                        g_app.snap.rr_ms = (uint16_t)(60000u / spm);
                        g_app.snap.heart_rate = hr_from_rr(g_app.snap.rr_ms);
                        g_app.snap.hr_confidence = g_app.hr_det.confidence;
                    }
                }
            }

            {
                float g[3];
                float a[3];
                g[0] = s.gx_dps; g[1] = s.gy_dps; g[2] = s.gz_dps;
                a[0] = s.ax_g;   a[1] = s.ay_g;   a[2] = s.az_g;
                mpu6050_fuse_attitude(&g_app.imu, g, a, 0.02f);
            }
        }
    }
    g_app.snap.cadence_spm = g_app.step.cadence_spm;
    g_app.snap.step_confidence = g_app.step.confidence;
    (void)mpu6050_get_attitude(&g_app.imu, &g_app.snap.roll_deg,
                               &g_app.snap.pitch_deg, &g_app.snap.yaw_deg);

    /* 2) 红外测温：1Hz，避免自热影响读数 */
    if ((g_app.ir_ok != 0u) && ((now - g_app.last_hr_ms) >= 1000u)) {
        float body = 0.0f;
        float amb = 0.0f;
        if (mlx90615_read_both_c(&g_app.ir, &body, &amb) == SENSOR_OK) {
            g_app.snap.body_temp_c = body;
            g_app.snap.ambient_temp_c = amb;
        }
        g_app.last_hr_ms = now;
    }

    /* 3) 把快照投递给 UI / COMM 任务（消息邮箱，非阻塞投递） */
    {
        band_msg_t m;
        memset(&m, 0, sizeof(m));
        m.kind = MSG_KIND_SNAPSHOT;
        m.u16 = g_app.snap.steps;
        m.arg0 = g_app.snap.heart_rate;
        (void)os_mbox_post(&g_app.sched, &g_app.data_mbox, &m);
    }

    /* 4) 任务心跳，供看门狗监督 */
    (void)iwdg_task_alive(&g_app.iwdg, "SENSOR", now);
    /* 5) 周期任务无需手动 sleep，调度器按 period_ms 唤醒 */
}

static void task_key(void *arg)
{
    uint32_t now = sched_millis(&g_app.sched);
    uint8_t level = 0u;
    band_msg_t m;
    (void)arg;
    g_app.key_runs++;

    /* 1) 电容触摸：读坐标 -> 手势判定 */
    if (g_app.touch_ok != 0u) {
        int n = ft6236_read_touch(&g_app.touch_dev, &g_app.touch, now);
        if (n >= 0) {
            if (g_app.touch.tap != 0u) {
                g_app.touch.tap = 0u;
                memset(&m, 0, sizeof(m));
                m.kind = MSG_KIND_TOUCH;
                m.arg0 = 1u;
                m.arg1 = (uint8_t)g_app.touch.pt[0].x;
                m.u32 = g_app.touch.pt[0].y;
                (void)os_mbox_post(&g_app.sched, &g_app.cmd_mbox, &m);
                /* 信号量唤醒 UI：触摸事件驱动重绘，避免 UI 空转刷屏 */
                (void)os_sem_post(&g_app.sched, &g_app.touch_sem);
            }
            if ((g_app.touch.swipe_x != 0) || (g_app.touch.swipe_y != 0)) {
                memset(&m, 0, sizeof(m));
                m.kind = MSG_KIND_TOUCH;
                if (g_app.touch.swipe_x != 0) {
                    m.arg0 = 2u;   /* 左右滑 */
                    m.arg1 = (uint8_t)((g_app.touch.swipe_x > 0) ? 1u : 2u);
                } else {
                    m.arg0 = 3u;   /* 上下滑 */
                    m.arg1 = (uint8_t)((g_app.touch.swipe_y > 0) ? 1u : 2u);
                }
                g_app.touch.swipe_x = 0;
                g_app.touch.swipe_y = 0;
                (void)os_mbox_post(&g_app.sched, &g_app.cmd_mbox, &m);
                (void)os_sem_post(&g_app.sched, &g_app.touch_sem);
            }
            if (g_app.touch.long_press != 0u) {
                g_app.touch.long_press = 0u;
                memset(&m, 0, sizeof(m));
                m.kind = MSG_KIND_TOUCH;
                m.arg0 = 4u;
                (void)os_mbox_post(&g_app.sched, &g_app.cmd_mbox, &m);
            }
        }
    }

    /* 2) 物理按键 PC13：低电平有效，带 30ms 软件消抖 */
    if (sensor_bus_get()->gpio_read != NULL) {
        uint8_t raw = 0u;
        /* 仿真里 pin 0x20 = GPIOC0 复用为按键电平 */
        if (sensor_bus_get()->gpio_read(0x20u, &raw) == SENSOR_OK) {
            level = (uint8_t)((raw == 0u) ? 1u : 0u);
        }
    }
    if ((level != 0u) && (g_app.last_key_level == 0u)) {
        memset(&m, 0, sizeof(m));
        m.kind = MSG_KIND_KEY;
        m.arg0 = 1u;
        (void)os_mbox_post(&g_app.sched, &g_app.cmd_mbox, &m);
        (void)os_sem_post(&g_app.sched, &g_app.touch_sem);
        power_mgr_note_activity(&g_app.power, now);
    }
    g_app.last_key_level = level;

    (void)iwdg_task_alive(&g_app.iwdg, "KEY", now);
}

static void task_ui(void *arg)
{
    uint32_t now = sched_millis(&g_app.sched);
    band_msg_t m;
    int had_event = 0;
    (void)arg;
    g_app.ui_runs++;

    /* 等触摸信号量：拿到就重绘，拿不到就走低频刷新（1Hz 保底） */
    if (os_sem_try_pend(&g_app.sched, &g_app.touch_sem) == 0) {
        had_event = 1;
    }
    while (os_mbox_try_pend(&g_app.sched, &g_app.cmd_mbox, &m) == 0) {
        ui_handle_input(m.kind, m.arg0, m.arg1);
        had_event = 1;
    }
    /* 消费体征快照（只取最新一条，界面显示最新值即可） */
    while (os_mbox_try_pend(&g_app.sched, &g_app.data_mbox, &m) == 0) {
        /* 空转消费，数据已在 g_app.snap 中 */
    }

    if ((had_event != 0) || ((now - g_app.last_ui_ms) >= 1000u)) {
        ui_render();
        g_app.last_ui_ms = now;
    }

    /* 马达定时关闭 */
    if ((g_app.motor_on != 0u) && ((int32_t)(now - g_app.motor_off_ms) >= 0)) {
        g_app.motor_on = 0u;
    }

    (void)iwdg_task_alive(&g_app.iwdg, "UI", now);
}

static void task_comm(void *arg)
{
    uint32_t now = sched_millis(&g_app.sched);
    band_params_t *p = &g_app.uplink.params;
    uint8_t payload[FRAME_MAX_PAYLOAD];
    health_payload_t h;
    size_t n;
    (void)arg;
    g_app.comm_runs++;

    /* 1) 驱动链路（握手/收包/保活/重连/补传都在这一个调用里） */
    uplink_poll(&g_app.uplink, now);

    /* 2) 周期上传体征聚合包 */
    if ((now - g_app.last_upload_ms) >= p->upload_interval_ms) {
        g_app.last_upload_ms = now;
        memset(&h, 0, sizeof(h));
        h.heart_rate = g_app.snap.heart_rate;
        h.confidence = g_app.snap.hr_confidence;
        h.rr_interval_ms = g_app.snap.rr_ms;
        h.temperature_c100 = (int16_t)(g_app.snap.body_temp_c * 100.0f);
        h.steps = g_app.snap.steps;
        h.battery_mv = g_app.snap.battery_mv;
        h.battery_pct = g_app.snap.battery_pct;
        h.roll_c100 = (int16_t)(g_app.snap.roll_deg * 100.0f);
        h.pitch_c100 = (int16_t)(g_app.snap.pitch_deg * 100.0f);
        h.yaw_c100 = (int16_t)(g_app.snap.yaw_deg * 100.0f);

        n = frame_pack_health(&h, payload, sizeof(payload));
        if (n > 0u) {
            (void)uplink_send(&g_app.uplink, MSG_HEALTH_BATCH, payload, (uint16_t)n, NULL);
            g_app.uploads++;
        }
    }

    (void)iwdg_task_alive(&g_app.iwdg, "COMM", now);
}

static void task_power(void *arg)
{
    uint32_t now = sched_millis(&g_app.sched);
    uint32_t busy;
    (void)arg;
    g_app.power_runs++;

    /* 0) 休眠中：只等唤醒源。真实板子上这里是 STOP 模式 + WFI，
     *    由 RTC 唤醒定时器（每 20 秒兜底）或触摸/按键/运动中断唤醒。 */
    if (g_app.power.state == POWER_ST_SLEEP) {
        if ((now - g_app.power.idle_since_ms) >= 20000u) {
            (void)power_mgr_wakeup(&g_app.power, now, WAKE_SRC_RTC_ALARM);
            band_platform_wakeup_stable();
        }
        (void)iwdg_task_alive(&g_app.iwdg, "POWER", now);
        return;
    }

    /* 1) 电池采样 + 库仑计推进 */
    (void)battery_sample(&g_app.bat, now);
    g_app.snap.battery_mv = g_app.bat.data.vbat_mv;
    g_app.snap.battery_pct = g_app.bat.data.percent;
    g_app.snap.charging = g_app.bat.data.charging;
    g_app.snap.uptime_s = now / 1000u;

    /* 2) 静止判定：步频为 0 时按负载电流 -2mA 估算 */
    {
        int16_t current = (g_app.snap.cadence_spm == 0u) ? -2 : -12;
        (void)battery_coulomb_update(&g_app.bat, current, 1000u);
    }

    /* 3) 低功耗状态机推进。
     *    busy 只统计"除自己以外、当前处于 READY 的任务"：
     *    周期任务跑完就 BLOCKED，所以正常节拍下 busy 通常为 0；
     *    一旦某个任务因为处理积压始终 READY，就说明系统真的忙，不能睡。 */
    busy = 0u;
    {
        uint8_t i;
        for (i = 0u; i < g_app.sched.task_count; i++) {
            if (i == g_app.sched.current) {
                continue;   /* 自己正在运行，不算"忙" */
            }
            if (g_app.sched.task[i].state == TASK_READY) {
                busy++;
            }
        }
    }
    /* 只有"链路在线且还有待补传数据"时才阻止休眠：
     * 断网时缓存的数据可以安静地躺着，设备该睡就睡（省电优先）。 */
    if ((g_app.uplink.link_up != 0u) && (uplink_pending_count(&g_app.uplink) > 0u)) {
        busy++;
    }
    (void)power_mgr_tick(&g_app.power, now, busy);

    /* 4) 监督式喂狗：任一任务超期就不喂，交给 IWDG 复位 */
    (void)iwdg_supervised_feed(&g_app.iwdg, now);
    (void)iwdg_task_alive(&g_app.iwdg, "POWER", now);
}

/* ------------------------------------------------------------------ */
/* 对外 API                                                            */
/* ------------------------------------------------------------------ */
band_app_t *band_app_instance(void)
{
    return &g_app;
}

os_sched_t *band_app_sched(void)
{
    return &g_app.sched;
}

power_mgr_t *band_app_power(void)
{
    return &g_app.power;
}

band_uplink_t *band_app_uplink(void)
{
    return &g_app.uplink;
}

iwdg_t *band_app_iwdg(void)
{
    return &g_app.iwdg;
}

const band_snapshot_t *band_app_snapshot(void)
{
    return &g_app.snap;
}

int band_app_init(const char *ws_host, uint16_t ws_port, const char *ws_path)
{
    uint32_t now;

    memset(&g_app, 0, sizeof(g_app));
    band_platform_init();
    now = band_millis();

    sched_init(&g_app.sched, now);
    power_mgr_init(&g_app.power, now);
    step_counter_init(&g_app.step);
    step_counter_init(&g_app.hr_det);
    /* 心率检测比计步更灵敏（阈值更低），但不参与计步输出 */
    step_counter_set_sensitivity(&g_app.hr_det, 90u);

    /* 同步原语 */
    os_sem_init(&g_app.touch_sem, "touch", 0, 4);
    os_mbox_init(&g_app.data_mbox, "data", MBOX_DEPTH_SENSOR);
    os_mbox_init(&g_app.cmd_mbox, "cmd", MBOX_DEPTH_COMM);

    /* 上行链路 */
    uplink_init(&g_app.uplink, net_socket_get(),
                (ws_host != NULL) ? ws_host : "127.0.0.1",
                (ws_port != 0u) ? ws_port : (uint16_t)WS_DEFAULT_PORT,
                (ws_path != NULL) ? ws_path : WS_DEFAULT_PATH);

    /* 外设登记：休眠关哪些、唤醒重启哪些，全在这张表里 */
    (void)power_mgr_register(&g_app.power, PERIPH_RTC, "RTC",
                             periph_rtc_init, periph_rtc_deinit, NULL, NULL, 1u);
    (void)power_mgr_register(&g_app.power, PERIPH_IWDG, "IWDG",
                             periph_iwdg_init, periph_iwdg_deinit, NULL, NULL, 1u);
    (void)power_mgr_register(&g_app.power, PERIPH_MPU6050, "MPU6050",
                             periph_mpu_init, periph_mpu_deinit,
                             periph_mpu_enter_lp, periph_mpu_exit_lp, 0u);
    (void)power_mgr_register(&g_app.power, PERIPH_FT6236, "FT6236",
                             periph_touch_init, periph_touch_deinit,
                             periph_touch_enter_lp, periph_touch_exit_lp, 0u);
    (void)power_mgr_register(&g_app.power, PERIPH_MLX90615, "MLX90615",
                             periph_ir_init, periph_ir_deinit,
                             periph_ir_enter_lp, periph_ir_exit_lp, 0u);
    (void)power_mgr_register(&g_app.power, PERIPH_OLED, "OLED",
                             periph_oled_init, periph_oled_deinit,
                             periph_oled_enter_lp, periph_oled_exit_lp, 0u);
    (void)power_mgr_register(&g_app.power, PERIPH_BATTERY_ADC, "BAT_ADC",
                             periph_bat_init, periph_bat_deinit,
                             periph_bat_enter_lp, periph_bat_exit_lp, 0u);
    (void)power_mgr_register(&g_app.power, PERIPH_WIFI, "WIFI",
                             periph_wifi_init, periph_wifi_deinit,
                             periph_wifi_enter_lp, periph_wifi_exit_lp, 0u);
    (void)power_mgr_register(&g_app.power, PERIPH_MOTOR, "MOTOR",
                             periph_motor_init, periph_motor_deinit,
                             periph_motor_enter_lp, periph_motor_exit_lp, 0u);

    /* 上电初始化所有外设：某一个失败（例如器件没焊）不影响整机起来 */
    (void)power_mgr_periph_init_all(&g_app.power);

    /* 检查上次复位是否由看门狗触发，并把罪魁任务打出来 */
    if (iwdg_check_reset(&g_app.iwdg) != 0) {
        band_trace("[boot] IWDG reset detected, fault task = %s\n",
                   iwdg_fault_task_name(&g_app.iwdg));
    }

    /* 注册任务（优先级 / 时间片 / 周期） */
    (void)sched_add_task(&g_app.sched, "SENSOR", task_sensor, &g_app,
                         PRIO_SENSOR, 5u, 20u);
    (void)sched_add_task(&g_app.sched, "KEY", task_key, &g_app,
                         PRIO_KEY, 3u, 10u);
    (void)sched_add_task(&g_app.sched, "UI", task_ui, &g_app,
                         PRIO_UI, 8u, 50u);
    (void)sched_add_task(&g_app.sched, "COMM", task_comm, &g_app,
                         PRIO_COMM, 10u, 20u);
    (void)sched_add_task(&g_app.sched, "POWER", task_power, &g_app,
                         PRIO_POWER, 5u, 1000u);

    /* 自动休眠：15 秒无活动（可被远程参数下发修改） */
    power_mgr_set_auto_sleep(&g_app.power, 1u,
                             (uint32_t)g_app.uplink.params.sleep_timeout_s * 1000u);

    g_app.page = PAGE_MAIN;
    g_app.backlight_on = 1u;
    ui_render();
    return 0;
}

/* 跑 n 个调度周期（PC 自检用），每步按 1ms 推进虚拟时间 */
int band_app_run_steps(uint32_t steps)
{
    uint32_t i;

    for (i = 0u; i < steps; i++) {
        (void)sched_run_once(&g_app.sched);
        sched_tick(&g_app.sched, 1u);
    }
    return 0;
}

/* 实时运行测试给定毫秒（PC 上按真实时间推进） */
void band_app_run_ms(uint32_t ms)
{
    uint32_t start = band_millis();
    uint32_t last = start;

    while ((band_millis() - start) < ms) {
        uint32_t now = band_millis();
        uint32_t elapsed = now - last;

        if (elapsed >= 1u) {
            sched_tick(&g_app.sched, elapsed);
            last = now;
        }
        (void)sched_run_once(&g_app.sched);
    }
}

/* 打印运行统计，便于现场/自检观察任务占用 */
void band_app_dump_stats(void)
{
    band_trace("=== %s v%s ===\n", BAND_FW_NAME, BAND_FW_VERSION_STR);
    band_trace("sensor=%u ui=%u key=%u comm=%u power=%u uploads=%u\n",
               (unsigned)g_app.sensor_runs, (unsigned)g_app.ui_runs,
               (unsigned)g_app.key_runs, (unsigned)g_app.comm_runs,
               (unsigned)g_app.power_runs, (unsigned)g_app.uploads);
    band_trace("mpu=%u touch=%u ir=%u oled=%u steps=%u hr=%u batt=%umV/%u%%\n",
               (unsigned)g_app.mpu_ok, (unsigned)g_app.touch_ok,
               (unsigned)g_app.ir_ok, (unsigned)g_app.oled_ok,
               (unsigned)g_app.snap.steps, (unsigned)g_app.snap.heart_rate,
               (unsigned)g_app.snap.battery_mv, (unsigned)g_app.snap.battery_pct);
    band_trace("link=%s pending=%u reconn=%u\n",
               uplink_state_str(&g_app.uplink),
               (unsigned)uplink_pending_count(&g_app.uplink),
               (unsigned)g_app.uplink.st.reconnects);
    sched_dump_stats(&g_app.sched);
}
