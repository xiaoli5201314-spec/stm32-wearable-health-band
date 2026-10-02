/*
 * battery.h -- 锂电池电压采样 / 电量计（库仑计 + 开路电压查表）
 *
 * 采样链路（对应原理图）：
 *   VBAT --[R1 100k]--+--[R2 100k]-- GND      分压比 1/2
 *                     |
 *                  PC4 / ADC1_IN14
 *   ADC1_IN17 (VREFINT, 1.21V) 用于运行时校准 VDDA，消除 LDO 偏差
 *
 * 电量估计：
 *   1) 静置时用开路电压(OCV)查表 -> 精度高
 *   2) 有负载时用库仑计积分补偿 -> 动态响应快
 *   3) 两者按负载电流加权融合
 */
#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stddef.h>
#include "sensor_iface.h"
#include "band_config.h"

#define BAT_ADC_CH_VBAT     14u   /* ADC1_IN14 -> PC4 */
#define BAT_ADC_CH_VREFINT  17u   /* ADC1_IN17 内部基准 */

typedef struct {
    uint16_t adc_vbat_raw;     /* 过采样后的原始码 */
    uint16_t vrefint_raw;      /* VREFINT 原始码 */
    uint16_t vdda_mv;          /* 由 VREFINT 反推的实际 VDDA */
    uint16_t vbat_mv;          /* 电池电压 */
    uint8_t  percent;          /* 0..100 */
    int16_t  current_ma;       /* 正=充电 负=放电，来自库仑计/负载模型 */
    float    remaining_mah;    /* 库仑计剩余容量 */
    uint8_t  charging;
    uint8_t  low_battery;      /* <10% 置位 */
    uint8_t  critical;         /* <3% 置位，需强制进低功耗 */
} battery_t;

typedef struct {
    const sensor_bus_t *bus;
    battery_t data;
    /* 库仑计状态 */
    float    remaining_mah;
    uint32_t last_update_ms;
    uint32_t samples;
    uint32_t ocv_lookup_count;
    uint32_t coulomb_updates;
    /* 平滑 */
    uint16_t vbat_filtered_mv;
    uint8_t  filter_init;
} battery_dev_t;

int  battery_init(battery_dev_t *dev, const sensor_bus_t *bus);
/* 采样一次（含 VREFINT 校准）；返回 0 成功 */
int  battery_sample(battery_dev_t *dev, uint32_t now_ms);
/* 库仑计推进：给定负载/充电电流与时间增量 */
int  battery_coulomb_update(battery_dev_t *dev, int16_t current_ma, uint32_t dt_ms);
/* 静置判定：无充放电电流时可用 OCV 校正库仑计 */
int  battery_ocv_calibrate(battery_dev_t *dev, uint16_t vbat_mv);
int  battery_is_charging(battery_dev_t *dev);

/* --- 纯函数，便于单元测试 --- */
/* 由 ADC 码值算电池电压（mV）：先由 VREFINT 校准 VDDA，再乘分压比 */
uint16_t battery_compute_vbat_mv(uint16_t adc_code, uint16_t vrefint_code);
/* 由 VREFINT 码值算 VDDA（mV）：VDDA = 1.21V * 4095 / vrefint_code */
uint16_t battery_compute_vdda_mv(uint16_t vrefint_code);
/* 电压 -> 电量百分比（Li-ion 放电曲线线性插值） */
uint8_t  battery_percent_from_mv(uint16_t mv);
/* 满电容量标定：剩余 mAh -> 百分比 */
uint8_t  battery_percent_from_mah(float remaining_mah, float capacity_mah);

#endif /* BATTERY_H */
