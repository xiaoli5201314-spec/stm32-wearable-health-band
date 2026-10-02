/*
 * test_step_counter.c -- 计步算法测试
 *
 * 覆盖验收标准第 5 条：合成 IMU 数据（静止段 + 步行段，已知步数）验证误差。
 *
 * 合成模型：
 *   静止：|a| = 1000mg + 带宽 8mg 的确定性伪随机噪声（xorshift，可复现）
 *   步行：|a| = 1000 + A*sin(2*pi*f*t) + 噪声，f = 步频，A = 300mg
 *   抖动干扰：3 个间隔 90ms 的 500mg 尖峰（模拟手腕抖动/打字）
 *
 * 采样率统一 50Hz（与固件里 20ms 周期的 SENSOR 任务一致）。
 */
#include "test_util.h"
#include "step_counter.h"
#include <string.h>
#include <math.h>

#define PI_F 3.14159265f

/* 确定性伪随机噪声，保证测试可复现 */
static uint32_t g_rnd = 0x12345678u;

static int32_t noise_mg(int32_t amplitude)
{
    g_rnd ^= g_rnd << 13;
    g_rnd ^= g_rnd >> 17;
    g_rnd ^= g_rnd << 5;
    return (int32_t)((g_rnd % (uint32_t)(2 * amplitude + 1))) - amplitude;
}

/* 生成步行段，返回"真实步数" */
static uint32_t synth_walk(step_counter_t *sc, uint32_t start_ms, uint32_t duration_ms,
                           float cadence_spm, int32_t amp_mg)
{
    const uint32_t dt = 20u;              /* 50Hz */
    uint32_t t;
    uint32_t steps_real = 0u;
    float phase = 0.0f;
    float omega = (2.0f * PI_F * cadence_spm) / 60.0f;   /* rad/s */

    for (t = 0u; t < duration_ms; t += dt) {
        float sec = (float)t / 1000.0f;
        int32_t mag = 1000 + (int32_t)((float)amp_mg * sinf(omega * sec)) + noise_mg(6);
        (void)step_counter_feed_mag(sc, mag, start_ms + t);
        /* 每完成一个正弦周期记一步 */
        phase += omega * ((float)dt / 1000.0f);
        if (phase >= (2.0f * PI_F)) {
            phase -= (2.0f * PI_F);
            steps_real++;
        }
    }
    return steps_real;
}

static uint32_t synth_still(step_counter_t *sc, uint32_t start_ms, uint32_t duration_ms,
                            int32_t gravity_mg)
{
    const uint32_t dt = 20u;
    uint32_t t;

    for (t = 0u; t < duration_ms; t += dt) {
        int32_t mag = gravity_mg + noise_mg(8);
        (void)step_counter_feed_mag(sc, mag, start_ms + t);
    }
    return duration_ms / dt;
}

static void test_static_no_false_steps(void)
{
    step_counter_t sc;
    uint32_t before;

    TEST_CASE("静止 30 秒（含传感器噪声）不应产生任何步数");
    step_counter_init(&sc);
    /* 前 2 秒作为基线建立期 */
    (void)synth_still(&sc, 0u, 2000u, 1000);
    before = step_counter_steps(&sc);
    (void)synth_still(&sc, 2000u, 30000u, 1000);
    TEST_ASSERT_EQ_UINT(step_counter_steps(&sc) - before, 0u, "静止段误检步数应为 0");
    TEST_ASSERT(sc.reject_refractory == 0u || sc.steps == 0u, "静止段不应有有效步");

    TEST_CASE("手表翻转/抬手（缓慢重力方向变化）不应计步");
    step_counter_init(&sc);
    (void)synth_still(&sc, 0u, 2000u, 1000);
    before = step_counter_steps(&sc);
    /* |a| 模值在抬手过程中基本不变，仍约 1000mg，只是方向变了 */
    (void)synth_still(&sc, 2000u, 5000u, 1000);
    TEST_ASSERT_EQ_UINT(step_counter_steps(&sc) - before, 0u, "抬手不应计步");
}

static void test_walking_accuracy(void)
{
    step_counter_t sc;
    uint32_t real;
    uint32_t got;
    int32_t err;
    int32_t err_pct;

    TEST_CASE("步行 30 秒 @100 步/分（50 步）误差应 <= 6%");
    step_counter_init(&sc);
    (void)synth_still(&sc, 0u, 2000u, 1000);              /* 先静置建立基线 */
    real = synth_walk(&sc, 2000u, 30000u, 100.0f, 300);   /* 30s @100spm -> 50 步 */
    got = step_counter_steps(&sc);
    err = (int32_t)got - (int32_t)real;
    err_pct = (int32_t)((err * 100) / (int32_t)((real == 0u) ? 1u : real));
    if (err < 0) {
        err_pct = -err_pct;
    }
    printf("     [data] 真实步数=%u 计步=%u 绝对误差=%d 相对误差=%d%%\n",
           (unsigned)real, (unsigned)got, (int)err, (int)err_pct);
    TEST_ASSERT(real >= 48u && real <= 52u, "合成信号真实步数应在 50 附近");
    TEST_ASSERT(err_pct <= 6, "计步相对误差应 <= 6%");
    TEST_ASSERT(sc.confidence > 0u, "持续步行后置信度应大于 0");
    printf("     [data] 步频=%u spm 置信度=%u 消除抖动=%u 幅值不足=%u\n",
           (unsigned)sc.cadence_spm, (unsigned)sc.confidence,
           (unsigned)sc.reject_refractory, (unsigned)sc.reject_low_amp);

    TEST_CASE("步行 30 秒 @140 步/分（70 步）误差应 <= 8%");
    step_counter_init(&sc);
    (void)synth_still(&sc, 0u, 2000u, 1000);
    real = synth_walk(&sc, 2000u, 30000u, 140.0f, 320);
    got = step_counter_steps(&sc);
    err = (int32_t)got - (int32_t)real;
    err_pct = (int32_t)((err * 100) / (int32_t)((real == 0u) ? 1u : real));
    if (err < 0) {
        err_pct = -err_pct;
    }
    printf("     [data] 真实步数=%u 计步=%u 绝对误差=%d 相对误差=%d%%\n",
           (unsigned)real, (unsigned)got, (int)err, (int)err_pct);
    TEST_ASSERT(err_pct <= 8, "快走相对误差应 <= 8%");

    TEST_CASE("慢走 30 秒 @70 步/分 误差应 <= 10%");
    step_counter_init(&sc);
    (void)synth_still(&sc, 0u, 2000u, 1000);
    real = synth_walk(&sc, 2000u, 30000u, 70.0f, 280);
    got = step_counter_steps(&sc);
    err = (int32_t)got - (int32_t)real;
    err_pct = (int32_t)((err * 100) / (int32_t)((real == 0u) ? 1u : real));
    if (err < 0) {
        err_pct = -err_pct;
    }
    printf("     [data] 真实步数=%u 计步=%u 绝对误差=%d 相对误差=%d%%\n",
           (unsigned)real, (unsigned)got, (int)err, (int)err_pct);
    TEST_ASSERT(err_pct <= 10, "慢走相对误差应 <= 10%");
}

/* 生成一个"波峰 + 回落"的完整事件：先爬升到 peak，再回落到静息值并保持，
 * 直到计数器确认这一步（或超过 max_ms 放弃）。返回是否确认成功。 */
static int feed_step_event(step_counter_t *sc, uint32_t *t, int32_t peak_mg, uint32_t max_ms)
{
    int i;
    uint32_t start = *t;
    uint32_t steps_before = step_counter_steps(sc);

    /* 上升沿：10 个采样（200ms）从 1000 爬到 peak */
    for (i = 1; i <= 10; i++) {
        (void)step_counter_feed_mag(sc, 1000 + ((peak_mg - 1000) * i) / 10, *t);
        *t += 20u;
    }
    /* 回落 + 静息，直到确认 */
    for (;;) {
        (void)step_counter_feed_mag(sc, 1000, *t);
        *t += 20u;
        if (step_counter_steps(sc) > steps_before) {
            return 1;
        }
        if ((*t - start) > max_ms) {
            return 0;
        }
    }
}

static void test_refractory(void)
{
    step_counter_t sc;
    uint32_t i;
    uint32_t t = 0u;
    uint32_t t1;

    TEST_CASE("不应期：距上一步 100ms 的上升沿必须被拒绝");
    step_counter_init(&sc);
    /* 静置 2 秒建立基线 */
    for (i = 0u; i < 100u; i++) {
        (void)step_counter_feed_mag(&sc, 1000, t);
        t += 20u;
    }
    TEST_ASSERT_EQ_INT(feed_step_event(&sc, &t, 1600, 2000u), 1, "第一个波峰应被确认");
    TEST_ASSERT_EQ_UINT(sc.steps, 1u, "第一个波峰应记为 1 步");
    t1 = sc.last_step_ms;

    /* 只推进 100ms 就注入一个尖峰 -> 上升沿落在 250ms 不应期内 */
    while (t < (t1 + 100u)) {
        (void)step_counter_feed_mag(&sc, 1000, t);
        t += 20u;
    }
    (void)step_counter_feed_mag(&sc, 1600, t);   /* 单采样尖峰 */
    t += 20u;
    TEST_ASSERT(sc.reject_refractory >= 1u, "应记录至少一次不应期拒绝");
    printf("     [data] 不应期拒绝次数=%u（尖峰出现在上一步之后 %u ms，不应期 %u ms）\n",
           (unsigned)sc.reject_refractory, (unsigned)(t - t1), (unsigned)sc.refractory_ms);

    TEST_CASE("不应期内注入的尖峰不得计步");
    while (t < (t1 + 600u)) {
        (void)step_counter_feed_mag(&sc, 1000, t);
        t += 20u;
    }
    TEST_ASSERT_EQ_UINT(step_counter_steps(&sc), 1u, "不应期内的尖峰不应产生新步");

    TEST_CASE("不应期外（>600ms）的波峰应正常计步");
    {
        uint32_t steps_before = step_counter_steps(&sc);
        TEST_ASSERT_EQ_INT(feed_step_event(&sc, &t, 1600, 2000u), 1, "不应期外的波峰应确认");
        TEST_ASSERT_EQ_UINT(step_counter_steps(&sc), steps_before + 1u, "步数 +1");
    }
}

static void test_motion_artifact_rejection(void)
{
    step_counter_t sc;
    uint32_t i;
    uint32_t j;

    TEST_CASE("手腕抖动（3 个 90ms 间隔的尖峰）最多只应计 1 步");
    step_counter_init(&sc);
    for (i = 0u; i < 100u; i++) {
        (void)step_counter_feed_mag(&sc, 1000, i * 20u);
    }
    /* 三次尖峰，间隔 90ms（远小于不应期） */
    for (j = 0u; j < 3u; j++) {
        uint32_t base = 2000u + (j * 90u);
        for (i = 0u; i < 3u; i++) {
            (void)step_counter_feed_mag(&sc, 1000 + (int32_t)((i + 1) * 170), base + (i * 20u));
        }
        for (i = 0u; i < 3u; i++) {
            (void)step_counter_feed_mag(&sc, 1510 - (int32_t)((i + 1) * 170), base + 60u + (i * 20u));
        }
    }
    TEST_ASSERT(step_counter_steps(&sc) <= 1u, "抖动尖峰最多算 1 步");
    printf("     [data] 抖动场景计步=%u 不应期拒绝=%u\n",
           (unsigned)sc.steps, (unsigned)sc.reject_refractory);

    TEST_CASE("幅值不足的扰动不计步（灵敏度可调）");
    step_counter_init(&sc);
    step_counter_set_sensitivity(&sc, 0u);     /* 灵敏度最低 -> 阈值最高 160mg */
    for (i = 0u; i < 100u; i++) {
        (void)step_counter_feed_mag(&sc, 1000, i * 2000u / 100u);
    }
    for (i = 0u; i < 200u; i++) {
        /* 幅度只有 45mg，低于阈值 */
        int32_t mag = 1000 + ((i % 20u < 10u) ? 45 : -45);
        (void)step_counter_feed_mag(&sc, mag, 2000u + (i * 20u));
    }
    TEST_ASSERT_EQ_UINT(step_counter_steps(&sc), 0u, "低于阈值的扰动不应计步");

    TEST_CASE("set_sensitivity 映射关系");
    step_counter_init(&sc);
    step_counter_set_sensitivity(&sc, 100u);
    TEST_ASSERT_EQ_INT(sc.threshold_mg, 60, "灵敏度 100 -> 阈值 60mg");
    step_counter_set_sensitivity(&sc, 0u);
    TEST_ASSERT_EQ_INT(sc.threshold_mg, 160, "灵敏度 0 -> 阈值 160mg");
    step_counter_set_sensitivity(&sc, 50u);
    TEST_ASSERT_EQ_INT(sc.threshold_mg, 110, "灵敏度 50 -> 阈值 110mg");
}

static void test_reset_and_stats(void)
{
    step_counter_t sc;
    uint32_t i;

    TEST_CASE("reset 清零步数与统计");
    step_counter_init(&sc);
    for (i = 0u; i < 200u; i++) {
        (void)step_counter_feed_mag(&sc, 1000 + (int32_t)((i % 40u < 20u) ? 300 : -300), i * 20u);
    }
    TEST_ASSERT(sc.steps > 0u, "应已计到步数");
    TEST_ASSERT(sc.samples == 200u, "采样计数");
    step_counter_reset(&sc);
    TEST_ASSERT_EQ_UINT(step_counter_steps(&sc), 0u, "reset 后步数为 0");
    TEST_ASSERT_EQ_UINT(sc.samples, 0u, "reset 后采样计数清零");
    TEST_ASSERT_EQ_UINT(sc.confidence, 0u, "reset 后置信度清零");
}

int test_step_counter_run(void)
{
    test_suite_begin("step_counter");
    test_static_no_false_steps();
    test_walking_accuracy();
    test_refractory();
    test_motion_artifact_rejection();
    test_reset_and_stats();
    return test_suite_end();
}
