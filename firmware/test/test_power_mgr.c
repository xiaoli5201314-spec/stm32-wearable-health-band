/*
 * test_power_mgr.c -- 低功耗状态机测试
 *
 * 覆盖验收标准第 6 条：休眠后外设关闭、唤醒后重初始化被调用（桩记录调用序列）。
 *
 * 用桩函数把每个外设的 init/deinit/enter_lp/exit_lp 调用写进共享日志，
 * 然后逐条断言顺序与次数：
 *   - 休眠：非 always_on 外设必须"先 enter_low_power 再 deinit"（顺序不能反，
 *     否则会在全速状态下掉电，可能造成器件状态异常）
 *   - RTC / IWDG 必须保持工作（不能被关）
 *   - 唤醒：必须"先 exit_low_power 再 init"，且 Wi-Fi 最先、马达最后
 *   - 重复休眠/唤醒不得漏调或重复调用
 */
#include "test_util.h"
#include "power_mgr.h"
#include <string.h>
#include <stdio.h>

/* ---------------- 调用序列记录 ---------------- */
#define SEQ_MAX 256

typedef struct {
    char  op[16];
    char  name[16];
} seq_evt_t;

static seq_evt_t g_seq[SEQ_MAX];
static int g_seq_len;

static void seq_add(const char *op, const char *name)
{
    if (g_seq_len < SEQ_MAX) {
        (void)snprintf(g_seq[g_seq_len].op, sizeof(g_seq[0].op), "%s", op);
        (void)snprintf(g_seq[g_seq_len].name, sizeof(g_seq[0].name), "%s", name);
        g_seq_len++;
    }
}

static void seq_reset(void)
{
    g_seq_len = 0;
}

static int seq_count(const char *op, const char *name)
{
    int i;
    int n = 0;
    for (i = 0; i < g_seq_len; i++) {
        if ((strcmp(g_seq[i].op, op) == 0) && (strcmp(g_seq[i].name, name) == 0)) {
            n++;
        }
    }
    return n;
}

/* 某操作首次出现的下标，-1 表示没有 */
static int seq_first(const char *op, const char *name)
{
    int i;
    for (i = 0; i < g_seq_len; i++) {
        if ((strcmp(g_seq[i].op, op) == 0) && (strcmp(g_seq[i].name, name) == 0)) {
            return i;
        }
    }
    return -1;
}

static void seq_dump(void)
{
    int i;
    printf("     [data] 调用序列(%d 条): ", g_seq_len);
    for (i = 0; i < g_seq_len; i++) {
        printf("%s(%s)%s", g_seq[i].op, g_seq[i].name,
               (i + 1 == g_seq_len) ? "" : " -> ");
    }
    printf("\n");
}

/* ---------------- 外设桩 ---------------- */
#define DEFINE_STUB(NAME, TOKEN)                                       \
    static int NAME##_init(void)    { seq_add("init", TOKEN);    return 0; } \
    static int NAME##_deinit(void)  { seq_add("deinit", TOKEN);  return 0; } \
    static int NAME##_enter(void)   { seq_add("enter_lp", TOKEN); return 0; } \
    static int NAME##_exit(void)    { seq_add("exit_lp", TOKEN); return 0; }

DEFINE_STUB(stub_mpu, "MPU6050")
DEFINE_STUB(stub_touch, "FT6236")
DEFINE_STUB(stub_ir, "MLX90615")
DEFINE_STUB(stub_oled, "OLED")
DEFINE_STUB(stub_bat, "BAT_ADC")
DEFINE_STUB(stub_wifi, "WIFI")
DEFINE_STUB(stub_motor, "MOTOR")
DEFINE_STUB(stub_rtc, "RTC")
DEFINE_STUB(stub_iwdg, "IWDG")

static int g_fail_init_flag;
static int failing_init(void)
{
    seq_add("init", "FAILDEV");
    return (g_fail_init_flag != 0) ? -1 : 0;
}
static int failing_deinit(void) { seq_add("deinit", "FAILDEV"); return 0; }

static void register_all(power_mgr_t *pm)
{
    (void)power_mgr_register(pm, PERIPH_MPU6050, "MPU6050",
                             stub_mpu_init, stub_mpu_deinit, stub_mpu_enter, stub_mpu_exit, 0u);
    (void)power_mgr_register(pm, PERIPH_FT6236, "FT6236",
                             stub_touch_init, stub_touch_deinit, stub_touch_enter, stub_touch_exit, 0u);
    (void)power_mgr_register(pm, PERIPH_MLX90615, "MLX90615",
                             stub_ir_init, stub_ir_deinit, stub_ir_enter, stub_ir_exit, 0u);
    (void)power_mgr_register(pm, PERIPH_OLED, "OLED",
                             stub_oled_init, stub_oled_deinit, stub_oled_enter, stub_oled_exit, 0u);
    (void)power_mgr_register(pm, PERIPH_BATTERY_ADC, "BAT_ADC",
                             stub_bat_init, stub_bat_deinit, stub_bat_enter, stub_bat_exit, 0u);
    (void)power_mgr_register(pm, PERIPH_WIFI, "WIFI",
                             stub_wifi_init, stub_wifi_deinit, stub_wifi_enter, stub_wifi_exit, 0u);
    (void)power_mgr_register(pm, PERIPH_MOTOR, "MOTOR",
                             stub_motor_init, stub_motor_deinit, stub_motor_enter, stub_motor_exit, 0u);
    /* RTC / IWDG：always_on = 1 */
    (void)power_mgr_register(pm, PERIPH_RTC, "RTC",
                             stub_rtc_init, stub_rtc_deinit, stub_rtc_enter, stub_rtc_exit, 1u);
    (void)power_mgr_register(pm, PERIPH_IWDG, "IWDG",
                             stub_iwdg_init, stub_iwdg_deinit, stub_iwdg_enter, stub_iwdg_exit, 1u);
}

static void test_sleep_closes_peripherals(void)
{
    power_mgr_t pm;
    static const char *devs[] = { "MPU6050", "FT6236", "MLX90615", "OLED",
                                  "BAT_ADC", "WIFI", "MOTOR" };
    size_t i;

    TEST_CASE("上电初始化：9 个外设的 init 都被调用一次");
    seq_reset();
    power_mgr_init(&pm, 0u);
    register_all(&pm);
    seq_reset();   /* 注册本身不产生外设调用 */
    TEST_ASSERT_EQ_INT(power_mgr_periph_init_all(&pm), 0, "初始化应全部成功");
    for (i = 0u; i < sizeof(devs) / sizeof(devs[0]); i++) {
        TEST_ASSERT_EQ_INT(seq_count("init", devs[i]), 1, "init 调用一次");
    }
    TEST_ASSERT_EQ_INT(seq_count("init", "RTC"), 1, "RTC init");
    TEST_ASSERT_EQ_INT(seq_count("init", "IWDG"), 1, "IWDG init");

    TEST_CASE("进入休眠：非 always_on 外设必须先 enter_lp 再 deinit");
    seq_reset();
    TEST_ASSERT_EQ_INT(power_mgr_enter_sleep(&pm, 20000u, WAKE_SRC_RTC_ALARM), 0, "进入休眠");
    TEST_ASSERT_EQ_INT((int)pm.state, (int)POWER_ST_SLEEP, "状态应为 SLEEP");
    seq_dump();
    for (i = 0u; i < sizeof(devs) / sizeof(devs[0]); i++) {
        int e = seq_first("enter_lp", devs[i]);
        int d = seq_first("deinit", devs[i]);
        TEST_ASSERT_EQ_INT(seq_count("enter_lp", devs[i]), 1, "enter_lp 调用一次");
        TEST_ASSERT_EQ_INT(seq_count("deinit", devs[i]), 1, "deinit 调用一次");
        TEST_ASSERT(e >= 0 && d >= 0 && e < d, "顺序必须是 enter_lp 在 deinit 之前");
    }
    TEST_ASSERT_EQ_INT(seq_count("deinit", "RTC"), 0, "RTC 不能关（唤醒源）");
    TEST_ASSERT_EQ_INT(seq_count("deinit", "IWDG"), 0, "IWDG 不能关（安全兜底）");
    TEST_ASSERT_EQ_INT(seq_count("enter_lp", "RTC"), 0, "RTC 不参与低功耗切换");
    TEST_ASSERT_EQ_INT(seq_count("deinit", "MOTOR"), 1, "马达必须关（防漏电）");
    TEST_ASSERT_EQ_UINT(pm.sleep_count, 1u, "休眠计数");

    TEST_CASE("唤醒：必须 exit_low_power 之后重新 init，且 Wi-Fi 最早、马达最后");
    seq_reset();
    TEST_ASSERT_EQ_INT(power_mgr_wakeup(&pm, 40000u, WAKE_SRC_TOUCH), 0, "唤醒");
    TEST_ASSERT_EQ_INT((int)pm.state, (int)POWER_ST_RUN, "状态应为 RUN");
    seq_dump();
    for (i = 0u; i < sizeof(devs) / sizeof(devs[0]); i++) {
        int x = seq_first("exit_lp", devs[i]);
        int n = seq_first("init", devs[i]);
        TEST_ASSERT_EQ_INT(seq_count("init", devs[i]), 1, "唤醒后必须重初始化");
        TEST_ASSERT(x >= 0 && n >= 0 && x < n, "顺序必须是 exit_lp 在 init 之前");
    }
    TEST_ASSERT_EQ_INT(seq_count("init", "RTC"), 0, "RTC 一直在跑，不需要重初始化");
    TEST_ASSERT_EQ_INT(seq_count("init", "IWDG"), 0, "IWDG 一直在跑，不需要重初始化");
    TEST_ASSERT(seq_first("init", "WIFI") < seq_first("init", "MOTOR"), "Wi-Fi 先于马达");
    TEST_ASSERT_EQ_UINT(pm.wake_count, 1u, "唤醒计数");
    TEST_ASSERT_EQ_UINT(pm.total_sleep_ms, 20000u, "休眠时长统计");
    TEST_ASSERT_EQ_INT((int)pm.last_wake_src, (int)WAKE_SRC_TOUCH, "唤醒源记录");
}

static void test_idle_auto_sleep(void)
{
    power_mgr_t pm;
    uint32_t t = 0u;

    TEST_CASE("空闲超时自动休眠：忙时不休、闲够时长才休");
    seq_reset();
    power_mgr_init(&pm, t);
    register_all(&pm);
    (void)power_mgr_periph_init_all(&pm);
    power_mgr_set_auto_sleep(&pm, 1u, 15000u);

    TEST_ASSERT_EQ_INT(power_mgr_can_sleep(&pm, 5000u, 0u), 0, "还没到空闲阈值不能睡");
    TEST_ASSERT_EQ_INT(power_mgr_can_sleep(&pm, 20000u, 3u), 0, "还有任务在跑不能睡");
    TEST_ASSERT_EQ_INT(power_mgr_can_sleep(&pm, 20000u, 0u), 1, "空闲够久且无任务 -> 可以睡");

    t = 20000u;
    seq_reset();
    TEST_ASSERT_EQ_INT(power_mgr_tick(&pm, t, 0u), 0, "tick 应触发休眠");
    TEST_ASSERT_EQ_INT((int)pm.state, (int)POWER_ST_SLEEP, "应已进入 SLEEP");
    TEST_ASSERT_EQ_INT(seq_count("deinit", "OLED"), 1, "OLED 应被关闭");

    TEST_CASE("休眠中继续 tick 不应重复关闭外设");
    (void)power_mgr_tick(&pm, t + 5000u, 0u);
    TEST_ASSERT_EQ_INT(seq_count("deinit", "OLED"), 1, "不应重复 deinit");
    TEST_ASSERT_EQ_UINT(pm.sleep_count, 1u, "仍是一次休眠");

    TEST_CASE("活动上报会阻止休眠");
    {
        power_mgr_t pm2;
        power_mgr_init(&pm2, 0u);
        register_all(&pm2);
        (void)power_mgr_periph_init_all(&pm2);
        power_mgr_set_auto_sleep(&pm2, 1u, 1000u);
        power_mgr_note_activity(&pm2, 9999u);
        TEST_ASSERT_EQ_INT(power_mgr_can_sleep(&pm2, 10000u, 0u), 0, "刚有活动不能睡");
        TEST_ASSERT_EQ_INT((int)pm2.state, (int)POWER_ST_RUN, "状态仍为 RUN");
    }

    TEST_CASE("IDLE 中间态：先降级到 IDLE 再考虑休眠");
    {
        power_mgr_t pm3;
        power_mgr_init(&pm3, 0u);
        register_all(&pm3);
        (void)power_mgr_periph_init_all(&pm3);
        power_mgr_set_auto_sleep(&pm3, 1u, 15000u);
        (void)power_mgr_tick(&pm3, 4000u, 0u);
        TEST_ASSERT_EQ_INT((int)pm3.state, (int)POWER_ST_IDLE, "3 秒无活动应进 IDLE");
        (void)power_mgr_tick(&pm3, 20000u, 0u);
        TEST_ASSERT_EQ_INT((int)pm3.state, (int)POWER_ST_SLEEP, "再久一些进 SLEEP");
    }
}

static void test_repeated_cycles(void)
{
    power_mgr_t pm;
    int cycle;
    uint32_t t = 1000u;

    TEST_CASE("连续 5 轮休眠/唤醒：调用次数精确匹配，无泄漏无遗漏");
    seq_reset();
    power_mgr_init(&pm, 0u);
    register_all(&pm);
    (void)power_mgr_periph_init_all(&pm);
    seq_reset();

    for (cycle = 0; cycle < 5; cycle++) {
        (void)power_mgr_enter_sleep(&pm, t, WAKE_SRC_RTC_ALARM);
        t += 10000u;
        (void)power_mgr_wakeup(&pm, t, WAKE_SRC_MOTION);
        t += 1000u;
    }
    TEST_ASSERT_EQ_INT(seq_count("deinit", "MPU6050"), 5, "5 轮 deinit");
    TEST_ASSERT_EQ_INT(seq_count("init", "MPU6050"), 5, "5 轮 init");
    TEST_ASSERT_EQ_INT(seq_count("enter_lp", "FT6236"), 5, "5 轮 enter_lp");
    TEST_ASSERT_EQ_INT(seq_count("exit_lp", "FT6236"), 5, "5 轮 exit_lp");
    TEST_ASSERT_EQ_UINT(pm.sleep_count, 5u, "5 次休眠");
    TEST_ASSERT_EQ_UINT(pm.wake_count, 5u, "5 次唤醒");
    TEST_ASSERT_EQ_UINT(pm.init_calls[PERIPH_MPU6050], 6u, "1 次上电 + 5 次唤醒重初始化");
    TEST_ASSERT_EQ_UINT(pm.deinit_calls[PERIPH_MPU6050], 5u, "5 次关闭");

    TEST_CASE("休眠期间非 always_on 外设都处于 inactive，always_on 保持 active");
    {
        power_mgr_t pm2;
        power_mgr_init(&pm2, 0u);
        register_all(&pm2);
        (void)power_mgr_periph_init_all(&pm2);
        (void)power_mgr_enter_sleep(&pm2, 5000u, WAKE_SRC_KEY);
        TEST_ASSERT_EQ_UINT(pm2.active[PERIPH_MPU6050], 0u, "MPU 应已掉电");
        TEST_ASSERT_EQ_UINT(pm2.active[PERIPH_OLED], 0u, "OLED 应已掉电");
        TEST_ASSERT_EQ_UINT(pm2.active[PERIPH_RTC], 1u, "RTC 保持工作");
        TEST_ASSERT_EQ_UINT(pm2.active[PERIPH_IWDG], 1u, "IWDG 保持工作");
        TEST_ASSERT_EQ_UINT(pm2.in_low_power[PERIPH_MPU6050], 1u, "MPU 处于低功耗态");
        (void)power_mgr_wakeup(&pm2, 9000u, WAKE_SRC_KEY);
        TEST_ASSERT_EQ_UINT(pm2.active[PERIPH_MPU6050], 1u, "唤醒后 MPU 恢复上电");
        TEST_ASSERT_EQ_UINT(pm2.in_low_power[PERIPH_MPU6050], 0u, "低功耗标志清掉");
    }
}

static void test_init_failure_and_log(void)
{
    power_mgr_t pm;

    TEST_CASE("某个外设初始化失败不影响其它外设（器件未焊也能开机）");
    seq_reset();
    power_mgr_init(&pm, 0u);
    g_fail_init_flag = 1;
    (void)power_mgr_register(&pm, PERIPH_MLX90615, "FAILDEV",
                             failing_init, failing_deinit, NULL, NULL, 0u);
    (void)power_mgr_register(&pm, PERIPH_OLED, "OLED",
                             stub_oled_init, stub_oled_deinit, stub_oled_enter, stub_oled_exit, 0u);
    {
        int rc = power_mgr_periph_init(&pm, PERIPH_MLX90615);
        TEST_ASSERT_EQ_INT(rc, -1, "失败的 init 应返回 -1");
        TEST_ASSERT_EQ_UINT(pm.active[PERIPH_MLX90615], 0u, "失败后不应标记为 active");
        TEST_ASSERT_EQ_INT(power_mgr_periph_init(&pm, PERIPH_OLED), 0, "后续外设仍能初始化");
        TEST_ASSERT_EQ_UINT(pm.active[PERIPH_OLED], 1u, "OLED 已上电");
    }

    TEST_CASE("事件日志记录了状态迁移、休眠进入与唤醒源");
    {
        uint16_t n = power_mgr_log_count(&pm);
        int saw_sleep_enter = 0;
        int saw_wake_src = 0;
        int saw_state = 0;
        uint16_t i;

        (void)power_mgr_enter_sleep(&pm, 30000u, WAKE_SRC_RTC_ALARM);
        (void)power_mgr_wakeup(&pm, 60000u, WAKE_SRC_WIFI);
        for (i = 0u; i < power_mgr_log_count(&pm); i++) {
            const pwr_evt_t *e = power_mgr_log_at(&pm, i);
            if (e == NULL) {
                break;
            }
            if (e->kind == PWR_EV_SLEEP_ENTER) {
                saw_sleep_enter++;
            }
            if ((e->kind == PWR_EV_WAKE_SRC) && (e->arg == (int)WAKE_SRC_WIFI)) {
                saw_wake_src++;
            }
            if (e->kind == PWR_EV_STATE) {
                saw_state++;
            }
        }
        TEST_ASSERT(power_mgr_log_count(&pm) > n, "日志应增长");
        TEST_ASSERT_EQ_INT(saw_sleep_enter, 1, "记录一次进入休眠");
        TEST_ASSERT_EQ_INT(saw_wake_src, 1, "记录唤醒源 WIFI");
        TEST_ASSERT(saw_state >= 3, "记录状态迁移");
        printf("     [data] 事件日志条数 = %u（环形上限 %d）\n",
               (unsigned)power_mgr_log_count(&pm), PWR_EVT_LOG_MAX);
    }

    TEST_CASE("名称映射");
    TEST_ASSERT_EQ_STR(power_state_name(POWER_ST_RUN), "RUN", "RUN");
    TEST_ASSERT_EQ_STR(power_state_name(POWER_ST_SLEEP), "SLEEP", "SLEEP");
    TEST_ASSERT_EQ_STR(periph_name(PERIPH_MLX90615), "MLX90615", "MLX90615");
    TEST_ASSERT_EQ_STR(wake_src_name(WAKE_SRC_TOUCH), "TOUCH", "TOUCH");
    TEST_ASSERT_EQ_STR(wake_src_name(WAKE_SRC_MOTION), "MOTION", "MOTION");

    TEST_CASE("参数校验：非法外设 id / 指针");
    TEST_ASSERT_EQ_INT(power_mgr_register(NULL, PERIPH_OLED, "x", NULL, NULL, NULL, NULL, 0u), -1,
                       "空指针应返回 -1");
    TEST_ASSERT_EQ_INT(power_mgr_periph_init(&pm, PERIPH_COUNT), -1, "越界 id 应返回 -1");
    TEST_ASSERT_EQ_INT(power_mgr_register(&pm, PERIPH_MOTOR, "MOTOR",
                                          stub_motor_init, stub_motor_deinit,
                                          stub_motor_enter, stub_motor_exit, 0u), 0,
                       "重复注册同一 id 允许（覆盖）");
    g_fail_init_flag = 0;
}

int test_power_mgr_run(void)
{
    test_suite_begin("power_mgr");
    test_sleep_closes_peripherals();
    test_idle_auto_sleep();
    test_repeated_cycles();
    test_init_failure_and_log();
    return test_suite_end();
}
