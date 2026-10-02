/*
 * oled_ssd1306.c -- SSD1306 128x64 OLED 驱动实现
 *
 * 关键设计：
 *   1) 驱动内部维护 1KB 显存镜像，所有绘图先落在 RAM，避免每画一个像素就发一次 I2C；
 *   2) flush 按"页"分块搬运（默认 4 页一块 = 512 字节），兼顾事务开销与栈占用；
 *   3) 上电初始化序列来自 SSD1306 数据手册 Table 9-1（128x64 推荐值）：
 *      MUX=63, 显示偏移 0, 起始行 0, 电荷泵 0x14, 内存寻址=水平, SEG 重映射,
 *      COM 反向扫描, COM 引脚配置 0x12, 对比度 0x7F, 预充电 0xF1, VCOMH 0x40
 */
#include "oled_ssd1306.h"
#include "oled_font.h"
#include "band_port.h"
#include <string.h>
#include <stdio.h>

int ssd1306_send_cmd(ssd1306_t *dev, uint8_t cmd)
{
    uint8_t ctrl = SSD1306_CTRL_CMD_SINGLE;

    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    /* 0x80 + cmd：单条命令 */
    return dev->bus->i2c_write(dev->addr7, NULL, 0u, (const uint8_t[]){ctrl, cmd}, 2u);
}

int ssd1306_send_cmds(ssd1306_t *dev, const uint8_t *cmds, size_t len)
{
    uint8_t buf[32];
    size_t i;

    if ((dev == NULL) || (cmds == NULL) || (len == 0u) || (len > (sizeof(buf) - 1u))) {
        return SENSOR_ERR_PARAM;
    }
    /* 0x00 + 命令流：一次事务发多条命令，适合初始化序列 */
    buf[0] = SSD1306_CTRL_CMD_STREAM;
    for (i = 0u; i < len; i++) {
        buf[1u + i] = cmds[i];
    }
    return dev->bus->i2c_write(dev->addr7, NULL, 0u, buf, len + 1u);
}

int ssd1306_send_data(ssd1306_t *dev, const uint8_t *data, size_t len)
{
    uint8_t buf[SSD1306_WIDTH + 1u];

    if ((dev == NULL) || (data == NULL) || (len == 0u) || (len > SSD1306_WIDTH)) {
        return SENSOR_ERR_PARAM;
    }
    /* 0x40 + 数据流 */
    buf[0] = SSD1306_CTRL_DATA_STREAM;
    memcpy(&buf[1], data, len);
    return dev->bus->i2c_write(dev->addr7, NULL, 0u, buf, len + 1u);
}

int ssd1306_init(ssd1306_t *dev, const sensor_bus_t *bus, uint8_t addr7)
{
    /* 初始化序列（带参数的命令成对出现） */
    static const uint8_t init_seq[] = {
        SSD1306_CMD_DISPLAY_OFF,
        SSD1306_CMD_SET_CLOCK_DIV, 0x80,          /* 振荡频率 8, 分频 1 */
        SSD1306_CMD_SET_MULTIPLEX, 0x3F,          /* MUX = 64 行 */
        SSD1306_CMD_SET_DISPLAY_OFFSET, 0x00,
        SSD1306_CMD_SET_START_LINE | 0x00,
        SSD1306_CMD_CHARGE_PUMP, 0x14,            /* 内部电荷泵使能（3.3V 供电必需） */
        SSD1306_CMD_MEM_ADDR_MODE, 0x00,          /* 水平寻址：写完一页自动跳下一页 */
        SSD1306_CMD_SEG_REMAP | 0x01,
        SSD1306_CMD_COM_SCAN_DEC,
        SSD1306_CMD_SET_COM_PINS, 0x12,           /* 交替 COM，适配 128x64 面板 */
        SSD1306_CMD_SET_CONTRAST, 0x7F,
        SSD1306_CMD_SET_PRECHARGE, 0xF1,          /* 相位 1=1, 相位 2=15 */
        SSD1306_CMD_SET_VCOMH, 0x40,
        SSD1306_CMD_ENTIRE_DISPLAY_RAM,           /* 显示跟随显存 */
        SSD1306_CMD_NORMAL_DISPLAY,
        SSD1306_CMD_DEACTIVATE_SCROLL,
        SSD1306_CMD_DISPLAY_ON
    };

    if ((dev == NULL) || (bus == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    memset(dev, 0, sizeof(*dev));
    dev->bus = bus;
    dev->addr7 = addr7;
    dev->contrast = 0x7Fu;

    if (ssd1306_send_cmds(dev, init_seq, sizeof(init_seq)) != SENSOR_OK) {
        dev->online = 0u;
        return SENSOR_ERR_NACK;
    }
    ssd1306_clear(dev);
    if (ssd1306_flush(dev) != SENSOR_OK) {
        dev->online = 0u;
        return SENSOR_ERR_BUS;
    }
    dev->online = 1u;
    return SENSOR_OK;
}

int ssd1306_display_on(ssd1306_t *dev, uint8_t on)
{
    return ssd1306_send_cmd(dev, (on != 0u) ? SSD1306_CMD_DISPLAY_ON : SSD1306_CMD_DISPLAY_OFF);
}

int ssd1306_set_contrast(ssd1306_t *dev, uint8_t contrast)
{
    int rc;

    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    /* 对比度是两字节命令：0x81 + 值 */
    {
        uint8_t cmds[2];
        cmds[0] = SSD1306_CMD_SET_CONTRAST;
        cmds[1] = contrast;
        rc = ssd1306_send_cmds(dev, cmds, 2u);
    }
    if (rc == SENSOR_OK) {
        dev->contrast = contrast;
    }
    return rc;
}

int ssd1306_invert(ssd1306_t *dev, uint8_t invert)
{
    dev->inverted = (invert != 0u) ? 1u : 0u;
    return ssd1306_send_cmd(dev, (invert != 0u) ? SSD1306_CMD_INVERSE_DISPLAY
                                                : SSD1306_CMD_NORMAL_DISPLAY);
}

int ssd1306_sleep(ssd1306_t *dev, uint8_t enable)
{
    /* 休眠只关显示与电荷泵，显存内容保留，唤醒后无需重绘 */
    return ssd1306_display_on(dev, (enable != 0u) ? 0u : 1u);
}

/* ------------------------------------------------------------------ */
/* 显存操作                                                            */
/* ------------------------------------------------------------------ */
void ssd1306_clear(ssd1306_t *dev)
{
    if (dev != NULL) {
        memset(dev->fb, 0x00, sizeof(dev->fb));
        dev->pixels_set = 0u;
    }
}

void ssd1306_fill(ssd1306_t *dev, uint8_t pattern)
{
    if (dev != NULL) {
        memset(dev->fb, pattern, sizeof(dev->fb));
    }
}

void ssd1306_draw_pixel(ssd1306_t *dev, int16_t x, int16_t y, uint8_t on)
{
    uint16_t idx;
    uint8_t bit;

    if (dev == NULL) {
        return;
    }
    if ((x < 0) || (y < 0) || (x >= SSD1306_WIDTH) || (y >= SSD1306_HEIGHT)) {
        return;   /* 越界裁剪，不做环绕，避免图案跑到屏幕另一边 */
    }
    idx = (uint16_t)(((uint16_t)y / 8u) * SSD1306_WIDTH + (uint16_t)x);
    bit = (uint8_t)(1u << ((uint8_t)y & 0x07u));
    if (on != 0u) {
        if ((dev->fb[idx] & bit) == 0u) {
            dev->pixels_set++;
        }
        dev->fb[idx] |= bit;
    } else {
        dev->fb[idx] &= (uint8_t)~bit;
    }
}

uint8_t ssd1306_get_pixel(const ssd1306_t *dev, int16_t x, int16_t y)
{
    uint16_t idx;
    uint8_t bit;

    if ((dev == NULL) || (x < 0) || (y < 0) || (x >= SSD1306_WIDTH) || (y >= SSD1306_HEIGHT)) {
        return 0u;
    }
    idx = (uint16_t)(((uint16_t)y / 8u) * SSD1306_WIDTH + (uint16_t)x);
    bit = (uint8_t)(1u << ((uint8_t)y & 0x07u));
    return ((dev->fb[idx] & bit) != 0u) ? 1u : 0u;
}

void ssd1306_draw_hline(ssd1306_t *dev, int16_t x, int16_t y, int16_t w, uint8_t on)
{
    int16_t i;
    for (i = 0; i < w; i++) {
        ssd1306_draw_pixel(dev, (int16_t)(x + i), y, on);
    }
}

void ssd1306_draw_vline(ssd1306_t *dev, int16_t x, int16_t y, int16_t h, uint8_t on)
{
    int16_t i;
    for (i = 0; i < h; i++) {
        ssd1306_draw_pixel(dev, x, (int16_t)(y + i), on);
    }
}

void ssd1306_draw_rect(ssd1306_t *dev, int16_t x, int16_t y, int16_t w, int16_t h, uint8_t on)
{
    ssd1306_draw_hline(dev, x, y, w, on);
    ssd1306_draw_hline(dev, x, (int16_t)(y + h - 1), w, on);
    ssd1306_draw_vline(dev, x, y, h, on);
    ssd1306_draw_vline(dev, (int16_t)(x + w - 1), y, h, on);
}

void ssd1306_draw_line(ssd1306_t *dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t on)
{
    /* Bresenham 整数直线算法 */
    int16_t dx = (int16_t)((x1 > x0) ? (x1 - x0) : (x0 - x1));
    int16_t dy = (int16_t)((y1 > y0) ? (y1 - y0) : (y0 - y1));
    int16_t sx = (int16_t)((x0 < x1) ? 1 : -1);
    int16_t sy = (int16_t)((y0 < y1) ? 1 : -1);
    int16_t err = (int16_t)(dx - dy);

    for (;;) {
        ssd1306_draw_pixel(dev, x0, y0, on);
        if ((x0 == x1) && (y0 == y1)) {
            break;
        }
        {
            int16_t e2 = (int16_t)(err * 2);
            if (e2 > -dy) {
                err = (int16_t)(err - dy);
                x0 = (int16_t)(x0 + sx);
            }
            if (e2 < dx) {
                err = (int16_t)(err + dx);
                y0 = (int16_t)(y0 + sy);
            }
        }
    }
}

void ssd1306_draw_char(ssd1306_t *dev, int16_t x, int16_t y, char c, uint8_t on)
{
    const uint8_t *g;
    uint8_t col;
    uint8_t row;

    if (dev == NULL) {
        return;
    }
    g = oled_font_glyph(c);
    for (col = 0u; col < OLED_FONT_WIDTH; col++) {
        uint8_t bits = g[col];
        for (row = 0u; row < OLED_FONT_HEIGHT; row++) {
            if ((bits & (uint8_t)(1u << row)) != 0u) {
                ssd1306_draw_pixel(dev, (int16_t)(x + col), (int16_t)(y + row), on);
            }
        }
    }
}

void ssd1306_draw_string(ssd1306_t *dev, int16_t x, int16_t y, const char *s, uint8_t on)
{
    int16_t cx = x;

    if ((dev == NULL) || (s == NULL)) {
        return;
    }
    while (*s != '\0') {
        if (cx > (SSD1306_WIDTH - OLED_FONT_WIDTH)) {
            break;   /* 右侧越界停止，不换行（手表屏幕排版由上层决定） */
        }
        ssd1306_draw_char(dev, cx, y, *s, on);
        cx = (int16_t)(cx + OLED_FONT_ADVANCE);
        s++;
    }
}

void ssd1306_draw_string_2x(ssd1306_t *dev, int16_t x, int16_t y, const char *s, uint8_t on)
{
    int16_t cx = x;
    uint8_t col;
    uint8_t row;

    if ((dev == NULL) || (s == NULL)) {
        return;
    }
    while (*s != '\0') {
        if (cx > (SSD1306_WIDTH - (OLED_FONT_WIDTH * 2))) {
            break;
        }
        {
            const uint8_t *g = oled_font_glyph(*s);
            for (col = 0u; col < OLED_FONT_WIDTH; col++) {
                for (row = 0u; row < OLED_FONT_HEIGHT; row++) {
                    if ((g[col] & (uint8_t)(1u << row)) != 0u) {
                        /* 每个点放大成 2x2 */
                        ssd1306_draw_pixel(dev, (int16_t)(cx + (col * 2)), (int16_t)(y + (row * 2)), on);
                        ssd1306_draw_pixel(dev, (int16_t)(cx + (col * 2) + 1), (int16_t)(y + (row * 2)), on);
                        ssd1306_draw_pixel(dev, (int16_t)(cx + (col * 2)), (int16_t)(y + (row * 2) + 1), on);
                        ssd1306_draw_pixel(dev, (int16_t)(cx + (col * 2) + 1), (int16_t)(y + (row * 2) + 1), on);
                    }
                }
            }
        }
        cx = (int16_t)(cx + (OLED_FONT_ADVANCE * 2));
        s++;
    }
}

void ssd1306_draw_u32(ssd1306_t *dev, int16_t x, int16_t y, uint32_t v, uint8_t on)
{
    char buf[12];
    int n = snprintf(buf, sizeof(buf), "%u", (unsigned)v);

    if (n > 0) {
        ssd1306_draw_string(dev, x, y, buf, on);
    }
}

/* ------------------------------------------------------------------ */
/* 刷新                                                                */
/* ------------------------------------------------------------------ */
int ssd1306_flush(ssd1306_t *dev)
{
    uint8_t page;

    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    /* 页模式下每页都要重新设置页地址 + 列地址 = 显存起点 */
    for (page = 0u; page < SSD1306_PAGES; page++) {
        uint8_t addr[3];
        const uint8_t *src = &dev->fb[(size_t)page * SSD1306_WIDTH];

        addr[0] = (uint8_t)(SSD1306_CMD_SET_PAGE | page);
        addr[1] = SSD1306_CMD_SET_LOW_COLUMN | 0x00u;
        addr[2] = SSD1306_CMD_SET_HIGH_COLUMN | 0x00u;
        if (ssd1306_send_cmds(dev, addr, 3u) != SENSOR_OK) {
            return SENSOR_ERR_BUS;
        }
        if (ssd1306_send_data(dev, src, SSD1306_WIDTH) != SENSOR_OK) {
            return SENSOR_ERR_BUS;
        }
        dev->flush_chunks = (uint16_t)(dev->flush_chunks + 1u);
    }
    return SENSOR_OK;
}
