/*
 * step_counter.c -- 基于加速度模值的计步算法
 *
 * 处理链（逐采样调用，采样率典型 50Hz）：
 *   1. 模值   mag = sqrt(ax^2+ay^2+az^2)                   (mg)
 *   2. 低通   lp += (mag - lp) >> 3                        去高频抖动
 *   3. 基线   base += (lp - base) >> 5                     跟踪重力分量（≈1000mg）
 *   4. 动态   dyn = lp - base                              走动时是准正弦
 *   5. 阈值   自适应：thr = max(min_thr, 0.55 * 最近波峰中位数)
 *   6. 迟滞   跨过 thr 记"上升"，回落到 thr-hyst 记"波峰确认"
 *   7. 不应期 距上一步 < 250ms 的波峰直接丢弃（抑制手臂抖动造成的重复计数）
 *
 * 之所以用"动态分量 + 迟滞 + 不应期"而不是简单过零计数：
 * 手表贴在手腕上，日常动作(打字/拿杯子)的加速度峰值经常达到 300mg，
 * 只靠幅度阈值必然误计；加上不应期与自适应阈值后才能把误检压到可接受范围。
 */
#include "step_counter.h"
#include <string.h>

#define STEP_HIST_LEN 8u

static int32_t isqrt_i32(int32_t v)
{
    int32_t x;
    int32_t y;
    if (v <= 0) {
        return 0;
    }
    x = v;
    y = (x + 1) / 2;
    /* 牛顿迭代，加速度数值范围小，10 次足够收敛 */
    for (int i = 0; i < 12; i++) {
        if (y >= x) {
            break;
        }
        x = y;
        y = (x + v / x) / 2;
    }
    return x;
}

void step_counter_init(step_counter_t *sc)
{
    if (sc == NULL) {
        return;
    }
    memset(sc, 0, sizeof(*sc));
    sc->threshold_mg = STEP_MIN_THRESHOLD_MG;
    sc->hysteresis_mg = STEP_HYSTERESIS_MG;
    sc->refractory_ms = STEP_REFRACTORY_MS;
    sc->adaptive = 1u;
    sc->confidence = 0u;
}

void step_counter_set_sensitivity(step_counter_t *sc, uint8_t sensitivity_0_100)
{
    int32_t thr;

    if (sc == NULL) {
        return;
    }
    if (sensitivity_0_100 > 100u) {
        sensitivity_0_100 = 100u;
    }
    /* 灵敏度越高 -> 阈值越低（灵敏度 0..100 映射阈值 160..60 mg），供 App 远程下发调节 */
    thr = 160 - (int32_t)sensitivity_0_100;
    if (thr < 60) {
        thr = 60;
    }
    sc->threshold_mg = thr;
}

/* 取最近有效波峰幅值的中位数：中位数比均值更抗单次异常冲击 */
static int32_t hist_median(const step_counter_t *sc)
{
    int32_t tmp[STEP_HIST_LEN];
    uint8_t n = sc->hist_count;
    uint8_t i;
    uint8_t j;

    if (n == 0u) {
        return 0;
    }
    memcpy(tmp, sc->hist, sizeof(tmp[0]) * (size_t)STEP_HIST_LEN);
    for (i = 0u; i < n; i++) {
        for (j = (uint8_t)(i + 1u); j < n; j++) {
            if (tmp[j] < tmp[i]) {
                int32_t t = tmp[i];
                tmp[i] = tmp[j];
                tmp[j] = t;
            }
        }
    }
    return tmp[n / 2u];
}

static int32_t effective_threshold(const step_counter_t *sc)
{
    int32_t thr = sc->threshold_mg;

    if ((sc->adaptive != 0u) && (sc->hist_count >= 3u)) {
        int32_t med = hist_median(sc);
        int32_t adaptive_thr = (med * 55) / 100;   /* 中位数的 55% */
        if (adaptive_thr > thr) {
            thr = adaptive_thr;
        }
    }
    if (thr < 40) {
        thr = 40;
    }
    return thr;
}

static void hist_push(step_counter_t *sc, int32_t peak)
{
    sc->hist[sc->hist_idx] = peak;
    sc->hist_idx = (uint8_t)((sc->hist_idx + 1u) % STEP_HIST_LEN);
    if (sc->hist_count < STEP_HIST_LEN) {
        sc->hist_count++;
    }
}

int step_counter_feed(step_counter_t *sc, int32_t ax_mg, int32_t ay_mg, int32_t az_mg,
                      uint32_t now_ms)
{
    int32_t mag;

    if (sc == NULL) {
        return 0;
    }
    mag = isqrt_i32(ax_mg * ax_mg + ay_mg * ay_mg + az_mg * az_mg);
    return step_counter_feed_mag(sc, mag, now_ms);
}

int step_counter_feed_mag(step_counter_t *sc, int32_t mag_mg, uint32_t now_ms)
{
    int32_t thr;
    int32_t hyst;
    int step = 0;

    if (sc == NULL) {
        return 0;
    }
    sc->samples++;
    if (sc->first_sample_set == 0u) {
        sc->first_sample_set = 1u;
        sc->start_ms = now_ms;
        sc->lp_mg = mag_mg;
        sc->baseline_mg = mag_mg;
        return 0;
    }

    /* 2. 一阶低通 */
    sc->lp_mg += (mag_mg - sc->lp_mg) / (int32_t)(1u << STEP_FILTER_SHIFT);
    /* 3. 重力基线 */
    sc->baseline_mg += (sc->lp_mg - sc->baseline_mg) / (int32_t)(1u << STEP_BASELINE_SHIFT);
    /* 4. 动态分量 */
    sc->prev_dyn_mg = sc->dyn_mg;
    sc->dyn_mg = sc->lp_mg - sc->baseline_mg;

    thr = effective_threshold(sc);
    hyst = sc->hysteresis_mg;

    if (sc->above == 0u) {
        /* 等待上升沿 */
        if (sc->dyn_mg >= thr) {
            uint32_t dt = now_ms - sc->last_step_ms;

            /* 不应期：距上一步太近的上升沿直接判为抖动/重复检波。
             * 这一判断放在"上升沿"而不是"波峰确认"上，因为真实步态信号
             * 从峰值回落到阈值以下还要几百毫秒，若在确认时才判断，
             * 相邻两步的确认时刻会被滤波延迟"拉近"，反而挡不住抖动。 */
            if ((sc->steps > 0u) && (dt < sc->refractory_ms)) {
                if (sc->suppress_latch == 0u) {
                    sc->reject_refractory++;
                    sc->suppress_latch = 1u;   /* 同一次超阈只计一次 */
                }
            } else {
                sc->above = 1u;
                sc->suppress_latch = 0u;
                sc->peak_mg = sc->dyn_mg;
                sc->last_peak_ms = now_ms;
            }
        } else {
            sc->suppress_latch = 0u;   /* 已回落到阈值以下，解锁 */
        }
    } else {
        /* 处于阈值以上：跟踪波峰幅值 */
        if (sc->dyn_mg > sc->peak_mg) {
            sc->peak_mg = sc->dyn_mg;
            sc->last_peak_ms = now_ms;
        }
        /* 回落到 (阈值 - 迟滞) 以下 -> 确认一个波峰 */
        if (sc->dyn_mg <= (thr - hyst)) {
            uint32_t dt = now_ms - sc->last_step_ms;
            sc->above = 0u;

            if (sc->peak_mg < thr) {
                sc->reject_low_amp++;
            } else {
                sc->steps++;
                step = 1;
                sc->last_step_ms = now_ms;
                hist_push(sc, sc->peak_mg);

                /* 步频 = 60 / 步间隔；单步无法算步频 */
                if (dt > 0u) {
                    uint32_t spm = 60000u / dt;
                    if ((spm >= STEP_MIN_CADENCE_SPM) && (spm <= STEP_MAX_CADENCE_SPM)) {
                        /* 一阶平滑，避免步频跳变 */
                        sc->cadence_spm = (uint16_t)(((uint32_t)sc->cadence_spm * 3u + spm) / 4u);
                    }
                }
                /* 置信度：连续稳定步频则升高，长时间无步则衰减 */
                if (sc->cadence_spm >= STEP_MIN_CADENCE_SPM) {
                    uint32_t c = (uint32_t)sc->confidence + 25u;
                    sc->confidence = (uint8_t)((c > 100u) ? 100u : c);
                }
            }
            sc->peak_mg = 0;
        }
    }

    step_counter_decay(sc, now_ms);
    return step;
}

void step_counter_decay(step_counter_t *sc, uint32_t now_ms)
{
    uint32_t idle;

    if ((sc == NULL) || (sc->last_step_ms == 0u)) {
        return;
    }
    idle = now_ms - sc->last_step_ms;
    /* 3 秒没有新步：认为停止走动，步频与置信度衰减到 0 */
    if (idle > 3000u) {
        sc->cadence_spm = 0u;
    }
    if (idle > 5000u) {
        sc->confidence = 0u;
    }
}

uint32_t step_counter_steps(const step_counter_t *sc)
{
    return (sc == NULL) ? 0u : sc->steps;
}

void step_counter_reset(step_counter_t *sc)
{
    uint32_t keep_steps;
    if (sc == NULL) {
        return;
    }
    keep_steps = 0u;   /* reset 语义：清零步数 */
    step_counter_init(sc);
    sc->steps = keep_steps;
}
