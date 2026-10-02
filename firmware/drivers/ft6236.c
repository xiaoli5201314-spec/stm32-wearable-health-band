/*
 * ft6236.c -- FocalTech FT6236 电容触摸驱动实现
 *
 * 触摸数据寄存器布局（点 1）：
 *   0x04 P1_XH: bit7:6 = 事件标志(0=按下 1=抬起 2=持续接触 3=保留)，bit3:0 = X 高 4 位
 *   0x05 P1_XL: X 低 8 位
 *   0x06 P1_YH: bit7:6 = 事件标志，bit3:0 = Y 高 4 位
 *   0x07 P1_YL: Y 低 8 位
 *   0x08 WEIGHT: 触摸压力/权重
 *   0x09 MISC:   触摸面积
 * 点 2 从 0x0A 开始，布局相同。
 *
 * 上电初始化会读 0x8C(FOCALTECH_ID)，FT6236 应返回 0x11；
 * 并配置 monitor 模式参数（0x83 TIMEENTERMONITOR / 0x84 PERIODACTIVE /
 * 0x85 PERIODMONITOR）以便休眠期间只保留唤醒能力。
 */
#include "ft6236.h"
#include "band_port.h"
#include <string.h>

int ft6236_init(ft6236_dev_t *dev, const sensor_bus_t *bus, uint8_t addr7,
                uint16_t width, uint16_t height)
{
    uint8_t id = 0u;

    if ((dev == NULL) || (bus == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    memset(dev, 0, sizeof(*dev));
    dev->bus = bus;
    dev->addr7 = addr7;
    dev->width = (width == 0u) ? 240u : width;
    dev->height = (height == 0u) ? 240u : height;

    if (ft6236_read_chip_id(dev, &id) != SENSOR_OK) {
        return SENSOR_ERR_NACK;
    }
    dev->chip_id = id;
    if (id != FT6236_CHIP_ID) {
        /* 不是 FT6236：可能焊了 FT6336/GT911，需要换驱动 */
        dev->online = 0u;
        return SENSOR_ERR_PARAM;
    }

    /* 读库版本，便于现场确认固件版本与产线一致 */
    (void)sensor_reg_read8(bus, addr7, FT6236_REG_LIB_VERSION_H, &dev->lib_version_h);
    (void)sensor_reg_read8(bus, addr7, FT6236_REG_LIB_VERSION_L, &dev->lib_version_l);
    /* 活动模式周期 12ms -> 约 83Hz 上报率，兼顾流畅与功耗 */
    (void)sensor_reg_write8(bus, addr7, FT6236_REG_PERIODACTIVE, 12u);
    /* monitor 模式周期 30ms，进入 monitor 前等待 1 秒 */
    (void)sensor_reg_write8(bus, addr7, FT6236_REG_PERIODMONITOR, 30u);
    (void)sensor_reg_write8(bus, addr7, FT6236_REG_TIMEENTERMONITOR, 1u);
    /* 阈值组：默认 0x80(128) 偏保守，手表小屏用 40 更灵敏 */
    (void)ft6236_set_threshold(dev, 40u, 0u);
    /* 中断模式：0 = 中断触发（低电平有效，接到 PC1/EXTI1 唤醒） */
    (void)ft6236_config_interrupt(dev, 0u);

    dev->online = 1u;
    dev->power_mode = 0u;
    return SENSOR_OK;
}

int ft6236_read_chip_id(ft6236_dev_t *dev, uint8_t *id)
{
    if ((dev == NULL) || (id == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    return sensor_reg_read8(dev->bus, dev->addr7, FT6236_REG_FOCALTECH_ID, id);
}

int ft6236_set_power_mode(ft6236_dev_t *dev, uint8_t mode)
{
    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    if (sensor_reg_write8(dev->bus, dev->addr7, FT6236_REG_POWER_MODE, mode) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    /* 0x82 CTRL bit0 = 1 时按 TIMEENTERMONITOR 自动进入 monitor */
    if (sensor_reg_update_bits(dev->bus, dev->addr7, FT6236_REG_CTRL, 0x01u,
                               (mode == 0u) ? 0x00u : 0x01u) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    dev->power_mode = mode;
    return SENSOR_OK;
}

int ft6236_config_interrupt(ft6236_dev_t *dev, uint8_t int_mode)
{
    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    return sensor_reg_write8(dev->bus, dev->addr7, FT6236_REG_INT_MODE, int_mode);
}

int ft6236_set_threshold(ft6236_dev_t *dev, uint8_t th_group, uint8_t th_diff)
{
    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    if (sensor_reg_write8(dev->bus, dev->addr7, FT6236_REG_TH_GROUP, th_group) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    if (th_diff != 0u) {
        (void)sensor_reg_write8(dev->bus, dev->addr7, FT6236_REG_TH_DIFF, th_diff);
    }
    return SENSOR_OK;
}

int ft6236_in_monitor(const ft6236_dev_t *dev)
{
    return ((dev != NULL) && (dev->power_mode != 0u)) ? 1 : 0;
}

/* ------------------------------------------------------------------ */
/* 纯解析（可脱离总线单测）                                            */
/* ------------------------------------------------------------------ */
int ft6236_parse_regs(const uint8_t *regs, size_t len, uint16_t width, uint16_t height,
                      ft6236_t *out)
{
    uint8_t n;
    uint8_t i;

    if ((regs == NULL) || (out == NULL) || (len < 14u)) {
        return SENSOR_ERR_PARAM;   /* 需要 2 字节头部 + 2 个点 x 6 字节 */
    }
    /* regs[0] = GEST_ID(0x02), regs[1] = TD_STATUS(0x03), regs[2..7] = 点1, regs[8..13] = 点2 */
    out->gesture = regs[0];
    n = (uint8_t)(regs[1] & 0x0Fu);   /* TD_STATUS[3:0] = 有效点数 */
    if (n > FT6236_MAX_POINTS) {
        n = FT6236_MAX_POINTS;        /* 本驱动只跟踪两点，多余点忽略 */
    }
    out->points = n;
    for (i = 0u; i < FT6236_MAX_POINTS; i++) {
        const uint8_t *p = &regs[2u + ((size_t)i * 6u)];
        uint16_t x = (uint16_t)((((uint16_t)p[0] & 0x0Fu) << 8) | (uint16_t)p[1]);
        uint16_t y = (uint16_t)((((uint16_t)p[2] & 0x0Fu) << 8) | (uint16_t)p[3]);
        out->pt[i].id = i;
        out->pt[i].event = (uint8_t)((p[0] >> 6) & 0x03u);
        out->pt[i].x = x;
        out->pt[i].y = y;
        out->pt[i].weight = p[4];
        out->pt[i].area = p[5];
    }
    (void)width;
    (void)height;
    return (int)n;
}

void ft6236_gesture_update(ft6236_t *t, uint32_t now_ms)
{
    int pressed_now;

    if (t == NULL) {
        return;
    }
    pressed_now = ((t->points > 0u) &&
                   ((t->pt[0].event == FT6236_EVENT_DOWN) ||
                    (t->pt[0].event == FT6236_EVENT_CONTACT))) ? 1 : 0;

    if (pressed_now != 0) {
        /* 有触摸：刷新最后坐标 */
        t->last_x = t->pt[0].x;
        t->last_y = t->pt[0].y;
    }

    if ((pressed_now != 0) && (t->pressed == 0u)) {
        /* 按下沿：记录起点，清掉上一次手势结果 */
        t->pressed = 1u;
        t->down_ms = now_ms;
        t->down_x = t->pt[0].x;
        t->down_y = t->pt[0].y;
        t->swipe_x = 0;
        t->swipe_y = 0;
        t->long_press = 0u;
        return;
    }

    if ((pressed_now == 0) && (t->pressed != 0u)) {
        /* 抬起沿：判定 滑动 / 单击 */
        uint32_t dur = now_ms - t->down_ms;
        int16_t dx = (int16_t)((int32_t)t->last_x - (int32_t)t->down_x);
        int16_t dy = (int16_t)((int32_t)t->last_y - (int32_t)t->down_y);
        int16_t adx = (int16_t)((dx < 0) ? -dx : dx);
        int16_t ady = (int16_t)((dy < 0) ? -dy : dy);

        t->pressed = 0u;
        if ((adx >= (int16_t)FT6236_SWIPE_MIN_PX) && (adx > ady)) {
            t->swipe_x = (int8_t)((dx > 0) ? 1 : -1);
        } else if ((ady >= (int16_t)FT6236_SWIPE_MIN_PX) && (ady >= adx)) {
            t->swipe_y = (int8_t)((dy > 0) ? 1 : -1);
        } else if ((dur <= FT6236_TAP_MAX_MS) &&
                   (adx <= FT6236_TAP_MAX_MOVE_PX) && (ady <= FT6236_TAP_MAX_MOVE_PX)) {
            t->tap = 1u;   /* 单击：上层消费后清零 */
        }
        return;
    }

    if (pressed_now != 0) {
        /* 持续接触：超过长按时间则置长按标志 */
        if ((now_ms - t->down_ms) >= FT6236_LONG_PRESS_MS) {
            t->long_press = 1u;
        }
    }
}

int ft6236_read_touch(ft6236_dev_t *dev, ft6236_t *touch, uint32_t now_ms)
{
    uint8_t regs[14];
    int n;

    if ((dev == NULL) || (touch == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    /* 从 0x02(GEST_ID) 连续读 14 字节，覆盖到 0x0F（点 2 的 MISC） */
    if (sensor_reg_read_buf(dev->bus, dev->addr7, FT6236_REG_GEST_ID, regs, sizeof(regs)) != SENSOR_OK) {
        dev->errors++;
        return SENSOR_ERR_BUS;
    }
    dev->polls++;
    n = ft6236_parse_regs(regs, sizeof(regs), dev->width, dev->height, touch);
    if (n < 0) {
        return n;
    }
    ft6236_gesture_update(touch, now_ms);
    return n;
}
