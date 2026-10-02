/*
 * ft6236.h -- FocalTech FT6236 电容触摸驱动
 *
 * 寄存器映射来自 FocalTech 通用触摸 IC 手册（FT5x06/FT6236 系列共用）：
 *   0x00 DEV_MODE        设备模式（bit6: 0=正常 1=测试）
 *   0x02 GEST_ID         手势 ID
 *   0x03 TD_STATUS       有效触摸点数 [3:0]
 *   0x04..0x09 点1：XH XL YH YL WEIGHT MISC（XH/YH 高 2bit 为事件标志）
 *   0x0A..0x0F 点2
 *   0x80 TH_GROUP        阈值组
 *   0x81 TH_DIFF         差分阈值
 *   0x82 CTRL            控制（bit0 进入 monitor）
 *   0x83 TIMEENTERMONITOR 进入 monitor 的延时（秒）
 *   0x84 PERIODACTIVE    活动模式周期 (ms)
 *   0x85 PERIODMONITOR   monitor 模式周期 (ms)
 *   0x86/0x87 LIB_VERSION_H/L
 *   0x8A POWER_MODE      电源模式
 *   0x8B FIRMWARE_ID
 *   0x8C FOCALTECH_ID    芯片 ID，FT6236 读回 0x11
 *   0xA4 INT_MODE        中断触发模式
 */
#ifndef FT6236_H
#define FT6236_H

#include <stdint.h>
#include <stddef.h>
#include "sensor_iface.h"

#define FT6236_REG_DEV_MODE          0x00
#define FT6236_REG_GEST_ID           0x02
#define FT6236_REG_TD_STATUS         0x03
#define FT6236_REG_P1_XH             0x04
#define FT6236_REG_P1_XL             0x05
#define FT6236_REG_P1_YH             0x06
#define FT6236_REG_P1_YL             0x07
#define FT6236_REG_P1_WEIGHT         0x08
#define FT6236_REG_P1_MISC           0x09
#define FT6236_REG_P2_XH             0x0A
#define FT6236_REG_TH_GROUP          0x80
#define FT6236_REG_TH_DIFF           0x81
#define FT6236_REG_CTRL              0x82
#define FT6236_REG_TIMEENTERMONITOR  0x83
#define FT6236_REG_PERIODACTIVE      0x84
#define FT6236_REG_PERIODMONITOR     0x85
#define FT6236_REG_LIB_VERSION_H     0x86
#define FT6236_REG_LIB_VERSION_L     0x87
#define FT6236_REG_POWER_MODE        0x8A
#define FT6236_REG_FIRMWARE_ID       0x8B
#define FT6236_REG_FOCALTECH_ID      0x8C
#define FT6236_REG_INT_MODE          0xA4

#define FT6236_CHIP_ID               0x11
#define FT6236_MAX_POINTS            2

/* 触摸事件类型（P1_XH / P1_YH 的 bit7:6） */
#define FT6236_EVENT_DOWN            0x00
#define FT6236_EVENT_UP              0x01
#define FT6236_EVENT_CONTACT         0x02
#define FT6236_EVENT_RESERVED        0x03

typedef struct {
    uint16_t x;
    uint16_t y;
    uint8_t  event;      /* FT6236_EVENT_xxx */
    uint8_t  weight;
    uint8_t  area;
    uint8_t  id;
} ft6236_point_t;

typedef struct {
    uint8_t gesture;     /* GEST_ID 原始值 */
    uint8_t points;      /* 有效点数 */
    ft6236_point_t pt[FT6236_MAX_POINTS];

    /* 手势判定状态（上层可直接用） */
    uint16_t down_x, down_y;
    uint16_t last_x, last_y;
    uint32_t down_ms;
    uint8_t  pressed;
    uint8_t  tap;          /* 松开时判定的单击 */
    uint8_t  long_press;   /* 按住 > LONG_PRESS_MS */
    int8_t   swipe_x;      /* -1 左滑 / +1 右滑 / 0 无 */
    int8_t   swipe_y;
} ft6236_t;

typedef struct {
    const sensor_bus_t *bus;
    uint8_t addr7;
    uint8_t online;
    uint8_t chip_id;
    uint8_t lib_version_h, lib_version_l;
    uint8_t power_mode;
    uint16_t width;
    uint16_t height;
    uint32_t polls;
    uint32_t errors;
} ft6236_dev_t;

#define FT6236_LONG_PRESS_MS   800u
#define FT6236_TAP_MAX_MS      300u
#define FT6236_TAP_MAX_MOVE_PX 12
#define FT6236_SWIPE_MIN_PX    40

int  ft6236_init(ft6236_dev_t *dev, const sensor_bus_t *bus, uint8_t addr7,
                 uint16_t width, uint16_t height);
int  ft6236_read_chip_id(ft6236_dev_t *dev, uint8_t *id);
int  ft6236_set_power_mode(ft6236_dev_t *dev, uint8_t mode);   /* 0=活动 1=monitor */
int  ft6236_config_interrupt(ft6236_dev_t *dev, uint8_t int_mode);
int  ft6236_set_threshold(ft6236_dev_t *dev, uint8_t th_group, uint8_t th_diff);
/* 读一次触摸状态并解析坐标；返回有效点数 */
int  ft6236_read_touch(ft6236_dev_t *dev, ft6236_t *touch, uint32_t now_ms);
/* 纯解析函数（可直接对寄存器块做单元测试） */
int  ft6236_parse_regs(const uint8_t *regs, size_t len, uint16_t width, uint16_t height,
                       ft6236_t *out);
/* 手势判定推进（内部由 read_touch 调用，单独暴露便于测试） */
void ft6236_gesture_update(ft6236_t *t, uint32_t now_ms);
int  ft6236_in_monitor(const ft6236_dev_t *dev);

#endif /* FT6236_H */
