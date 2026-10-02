/*
 * step_counter.h -- 基于加速度模值的计步算法
 *
 * 算法链路：
 *   |a| -> 一阶低通去抖 -> 重力基线(慢速滑动平均)相减得到动态分量
 *       -> 双阈值 + 迟滞比较器提取波峰 -> 不应期(250ms)抑制抖动重复计数
 *       -> 自适应阈值(最近若干次有效波峰幅值的中位数比例)适应快走/慢走
 * 输出不仅给出步数，还给出实时步频与置信度，便于上层判断数据质量。
 */
#ifndef STEP_COUNTER_H
#define STEP_COUNTER_H

#include <stdint.h>
#include <stddef.h>
#include "band_config.h"

typedef struct {
    /* 配置 */
    int32_t  threshold_mg;      /* 基础动态阈值 */
    int32_t  hysteresis_mg;     /* 迟滞带宽 */
    uint16_t refractory_ms;     /* 不应期 */
    uint8_t  adaptive;          /* 是否启用自适应阈值 */

    /* 滤波状态 */
    int32_t  lp_mg;             /* 低通后的 |a| */
    int32_t  baseline_mg;       /* 重力基线（≈1000mg） */
    int32_t  dyn_mg;            /* 动态分量 = lp - baseline */
    int32_t  prev_dyn_mg;
    int32_t  peak_mg;           /* 本次波峰的最大幅值 */
    uint8_t  above;             /* 是否处于阈值以上（迟滞状态） */
    uint8_t  suppress_latch;    /* 不应期抑制锁存：一段连续超阈只记一次拒绝 */
    uint32_t last_step_ms;
    uint32_t last_peak_ms;

    /* 自适应阈值历史（最近 8 次有效波峰幅值） */
    int32_t  hist[8];
    uint8_t  hist_idx;
    uint8_t  hist_count;

    /* 输出 */
    uint32_t steps;
    uint32_t reject_refractory;  /* 被不应期拒绝的波峰数（误检抑制） */
    uint32_t reject_low_amp;     /* 幅值不足被拒绝的波峰数 */
    uint16_t cadence_spm;        /* 实时步频 步/分钟 */
    uint8_t  confidence;         /* 0..100 */

    /* 统计 */
    uint32_t samples;
    uint32_t first_sample_set;
    uint32_t start_ms;
} step_counter_t;

void step_counter_init(step_counter_t *sc);
void step_counter_set_sensitivity(step_counter_t *sc, uint8_t sensitivity_0_100);
/* 输入一次三轴加速度（单位 mg，1g = 1000mg）；返回本采样是否记为一步（1/0） */
int  step_counter_feed(step_counter_t *sc, int32_t ax_mg, int32_t ay_mg, int32_t az_mg,
                       uint32_t now_ms);
/* 便捷：直接给模值 */
int  step_counter_feed_mag(step_counter_t *sc, int32_t mag_mg, uint32_t now_ms);
/* 由最近两次步间隔估算步频；长时间无步则置信度衰减 */
void step_counter_decay(step_counter_t *sc, uint32_t now_ms);
uint32_t step_counter_steps(const step_counter_t *sc);
void step_counter_reset(step_counter_t *sc);

#endif /* STEP_COUNTER_H */
