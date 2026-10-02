/*
 * battery.c -- 锂电池电压采样与电量估计
 *
 * 采样算术（对应原理图的 100k/100k 分压）：
 *   1) 先读 VREFINT(ADC1_IN17) 得到码值，反推真实 VDDA：
 *        VDDA_mV = 1210 * 4095 / VREFINT_code
 *      LDO 输出随负载/温度漂移，直接按 3300mV 算会有 10% 以上误差，
 *      所以每次采样都用内部基准校准一次，这是最省成本也最有效的做法。
 *   2) 电池电压：
 *        VBAT_mV = adc_code * VDDA_mV / 4095 * (R1+R2)/R2
 *      分压比 2:1，所以乘 2。
 *   3) 过采样：连续 16 次求平均，等效提高约 2bit 有效位，抑制 PWM/背光噪声。
 *
 * 电量估计两条腿：
 *   - 静置（|I| < 5mA）且电压稳定：用开路电压查表，长期准确；
 *   - 有负载：用库仑计积分，动态响应快。
 *   两者按"是否静置"切换，静置时用 OCV 校正库仑计，消除累积误差。
 */
#include "battery.h"
#include "band_port.h"
#include <string.h>

/* Li-ion 放电曲线（室温 25℃，0.2C 放电），电压 mV -> 剩余容量百分比 */
typedef struct {
    uint16_t mv;
    uint8_t  pct;
} ocv_point_t;

static const ocv_point_t k_ocv[] = {
    { 4200u, 100u },
    { 4150u,  95u },
    { 4100u,  90u },
    { 4050u,  85u },
    { 4000u,  78u },
    { 3950u,  71u },
    { 3900u,  63u },
    { 3850u,  55u },
    { 3800u,  47u },
    { 3750u,  38u },
    { 3700u,  30u },
    { 3650u,  22u },
    { 3600u,  15u },
    { 3550u,  10u },
    { 3500u,   6u },
    { 3450u,   4u },
    { 3400u,   3u },
    { 3350u,   2u },
    { 3300u,   0u }
};

#define OCV_POINTS (sizeof(k_ocv) / sizeof(k_ocv[0]))

uint16_t battery_compute_vdda_mv(uint16_t vrefint_code)
{
    if (vrefint_code == 0u) {
        return (uint16_t)BAT_ADC_VREF_MV;
    }
    /* VDDA = VREFINT_典型值 * 满量程 / 码值（定点：先乘后除，避免精度损失） */
    return (uint16_t)(((uint32_t)BAT_VREFINT_MV * (uint32_t)BAT_ADC_MAX_CODE) / (uint32_t)vrefint_code);
}

uint16_t battery_compute_vbat_mv(uint16_t adc_code, uint16_t vrefint_code)
{
    uint32_t vdda = battery_compute_vdda_mv(vrefint_code);
    uint32_t mv = ((uint32_t)adc_code * vdda) / (uint32_t)BAT_ADC_MAX_CODE;

    /* 分压比 R1=R2=100k -> 采样点电压是电池的一半 */
    mv = (mv * (uint32_t)BAT_DIVIDER_NUM);
    if (mv > 65535u) {
        mv = 65535u;
    }
    return (uint16_t)mv;
}

uint8_t battery_percent_from_mv(uint16_t mv)
{
    size_t i;

    if (mv >= k_ocv[0].mv) {
        return 100u;
    }
    if (mv <= k_ocv[OCV_POINTS - 1u].mv) {
        return 0u;
    }
    for (i = 0u; i + 1u < OCV_POINTS; i++) {
        uint16_t hi = k_ocv[i].mv;
        uint16_t lo = k_ocv[i + 1u].mv;
        if ((mv <= hi) && (mv >= lo)) {
            /* 线性插值 */
            uint32_t span = (uint32_t)(hi - lo);
            uint32_t part = (uint32_t)(mv - lo);
            uint32_t span_pct = (uint32_t)(k_ocv[i].pct - k_ocv[i + 1u].pct);
            return (uint8_t)((uint32_t)k_ocv[i + 1u].pct + ((part * span_pct) / span));
        }
    }
    return 0u;
}

uint8_t battery_percent_from_mah(float remaining_mah, float capacity_mah)
{
    float pct;

    if (capacity_mah <= 0.0f) {
        return 0u;
    }
    pct = (remaining_mah / capacity_mah) * 100.0f;
    if (pct <= 0.0f) {
        return 0u;
    }
    if (pct >= 100.0f) {
        return 100u;
    }
    return (uint8_t)pct;
}

/* ------------------------------------------------------------------ */
int battery_init(battery_dev_t *dev, const sensor_bus_t *bus)
{
    if ((dev == NULL) || (bus == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    memset(dev, 0, sizeof(*dev));
    dev->bus = bus;
    dev->remaining_mah = (float)BAT_CELL_CAPACITY_MAH;
    dev->data.remaining_mah = dev->remaining_mah;
    dev->data.percent = 100u;
    return SENSOR_OK;
}

int battery_sample(battery_dev_t *dev, uint32_t now_ms)
{
    uint32_t acc_v = 0u;
    uint32_t acc_ref = 0u;
    uint16_t code = 0u;
    uint8_t i;
    battery_t *d;

    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    d = &dev->data;
    if ((dev->bus == NULL) || (dev->bus->adc_read_raw == NULL)) {
        return SENSOR_ERR_PARAM;   /* 平台没提供 ADC 回调：直接报错，绝不调用空指针 */
    }

    /* 过采样：16 次累加后右移 4 位 */
    for (i = 0u; i < BAT_ADC_OVERSAMPLE; i++) {
        if (dev->bus->adc_read_raw(BAT_ADC_CH_VBAT, &code) != SENSOR_OK) {
            return SENSOR_ERR_BUS;
        }
        acc_v += code;
        if (dev->bus->adc_read_raw(BAT_ADC_CH_VREFINT, &code) != SENSOR_OK) {
            return SENSOR_ERR_BUS;
        }
        acc_ref += code;
    }
    d->adc_vbat_raw = (uint16_t)(acc_v / BAT_ADC_OVERSAMPLE);
    d->vrefint_raw = (uint16_t)(acc_ref / BAT_ADC_OVERSAMPLE);
    d->vdda_mv = battery_compute_vdda_mv(d->vrefint_raw);
    d->vbat_mv = battery_compute_vbat_mv(d->adc_vbat_raw, d->vrefint_raw);

    /* 一阶低通，抑制马达/背光带来的瞬时跌落 */
    if (dev->filter_init == 0u) {
        dev->vbat_filtered_mv = d->vbat_mv;
        dev->filter_init = 1u;
    } else {
        dev->vbat_filtered_mv = (uint16_t)(((uint32_t)dev->vbat_filtered_mv * 7u + d->vbat_mv) / 8u);
    }

    /* 静置判定：滤波后电压波动小于 8mV 且电流很小 -> 可用 OCV 校准 */
    {
        uint16_t diff = (uint16_t)((dev->vbat_filtered_mv > d->vbat_mv)
                                       ? (dev->vbat_filtered_mv - d->vbat_mv)
                                       : (d->vbat_mv - dev->vbat_filtered_mv));
        if ((diff <= 8u) && ((d->current_ma > -5) && (d->current_ma < 5))) {
            (void)battery_ocv_calibrate(dev, dev->vbat_filtered_mv);
        }
    }

    d->percent = battery_percent_from_mv(dev->vbat_filtered_mv);
    {
        uint8_t mah_pct = battery_percent_from_mah(dev->remaining_mah, (float)BAT_CELL_CAPACITY_MAH);
        if ((d->current_ma < -5) || (d->current_ma > 5)) {
            /* 有明显充放电电流时 OCV 不可信，更信任库仑计 */
            d->percent = mah_pct;
        } else {
            /* 静置：两者取平均，兼顾长期准确与短期平滑 */
            d->percent = (uint8_t)(((uint16_t)d->percent + (uint16_t)mah_pct) / 2u);
        }
    }
    d->remaining_mah = dev->remaining_mah;
    d->low_battery = (d->percent < 10u) ? 1u : 0u;
    d->critical = (d->percent < 3u) ? 1u : 0u;
    d->charging = battery_is_charging(dev);
    dev->samples++;
    dev->last_update_ms = now_ms;
    return SENSOR_OK;
}

int battery_coulomb_update(battery_dev_t *dev, int16_t current_ma, uint32_t dt_ms)
{
    float delta_mah;

    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    /* mAh = mA * h = mA * ms / 3600000 */
    delta_mah = ((float)current_ma * (float)dt_ms) / 3600000.0f;
    dev->remaining_mah += delta_mah;
    if (dev->remaining_mah > (float)BAT_CELL_CAPACITY_MAH) {
        dev->remaining_mah = (float)BAT_CELL_CAPACITY_MAH;
    }
    if (dev->remaining_mah < 0.0f) {
        dev->remaining_mah = 0.0f;
    }
    dev->data.current_ma = current_ma;
    dev->data.remaining_mah = dev->remaining_mah;
    dev->coulomb_updates++;
    return SENSOR_OK;
}

int battery_ocv_calibrate(battery_dev_t *dev, uint16_t vbat_mv)
{
    uint8_t pct;

    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    pct = battery_percent_from_mv(vbat_mv);
    dev->remaining_mah = ((float)BAT_CELL_CAPACITY_MAH * (float)pct) / 100.0f;
    dev->data.remaining_mah = dev->remaining_mah;
    dev->ocv_lookup_count++;
    return SENSOR_OK;
}

int battery_is_charging(battery_dev_t *dev)
{
    /* 本板充电状态由充电管理 IC 的 CHRG 开漏脚给出，经 GPIO 读回；
     * 无该引脚信息时用电压趋势判断：电压高于 4.15V 且电流为正 */
    if (dev == NULL) {
        return 0;
    }
    if (dev->data.current_ma > 20) {
        return 1;
    }
    if (dev->data.vbat_mv >= 4150u) {
        return 1;
    }
    return 0;
}
