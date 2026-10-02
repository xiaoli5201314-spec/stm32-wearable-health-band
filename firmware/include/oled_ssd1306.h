/*
 * oled_ssd1306.h -- SSD1306 128x64 OLED 显示驱动（I2C）
 *
 * 控制字节格式（I2C 从机地址 0x3C）：
 *   0x00 + cmd stream      后续字节全是命令
 *   0x40 + data stream     后续字节全是显存数据
 *   0x80 + single command
 *   0xC0 + single data
 *
 * 显存组织：8 页 x 128 列，每字节表示该页内一列的 8 个像素（bit0 在上）。
 * 驱动内部维护 1KB 显存镜像，flush 时按页分块搬运，避免每像素一次 I2C 事务。
 */
#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#include <stdint.h>
#include <stddef.h>
#include "sensor_iface.h"

#define SSD1306_ADDR7_DEFAULT   0x3C
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          64
#define SSD1306_PAGES           (SSD1306_HEIGHT / 8)      /* 8 */
#define SSD1306_FB_SIZE         (SSD1306_WIDTH * SSD1306_PAGES)  /* 1024 */

/* 命令 */
#define SSD1306_CMD_DISPLAY_OFF         0xAE
#define SSD1306_CMD_DISPLAY_ON          0xAF
#define SSD1306_CMD_SET_CLOCK_DIV       0xD5  /* 高 4bit 振荡频率，低 4bit 分频 */
#define SSD1306_CMD_SET_MULTIPLEX       0xA8
#define SSD1306_CMD_SET_DISPLAY_OFFSET  0xD3
#define SSD1306_CMD_SET_START_LINE      0x40
#define SSD1306_CMD_CHARGE_PUMP         0x8D  /* 0x14 = 内部电荷泵使能 */
#define SSD1306_CMD_MEM_ADDR_MODE       0x20  /* 0x00 水平 0x01 垂直 0x02 页 */
#define SSD1306_CMD_SEG_REMAP           0xA1  /* bit0=1 列地址 127 映射到 SEG0 */
#define SSD1306_CMD_COM_SCAN_DEC        0xC8
#define SSD1306_CMD_SET_COM_PINS        0xDA  /* 0x12 = 交替 COM，128x64 */
#define SSD1306_CMD_SET_CONTRAST        0x81
#define SSD1306_CMD_SET_PRECHARGE       0xD9
#define SSD1306_CMD_SET_VCOMH           0xDB
#define SSD1306_CMD_ENTIRE_DISPLAY_ON   0xA5
#define SSD1306_CMD_ENTIRE_DISPLAY_RAM  0xA4
#define SSD1306_CMD_NORMAL_DISPLAY      0xA6
#define SSD1306_CMD_INVERSE_DISPLAY     0xA7
#define SSD1306_CMD_DEACTIVATE_SCROLL   0x2E
#define SSD1306_CMD_SET_LOW_COLUMN      0x00  /* 0x00..0x0F */
#define SSD1306_CMD_SET_HIGH_COLUMN     0x10  /* 0x10..0x1F */
#define SSD1306_CMD_SET_PAGE            0xB0  /* 0xB0..0xB7 */

#define SSD1306_CTRL_CMD_STREAM         0x00
#define SSD1306_CTRL_DATA_STREAM        0x40
#define SSD1306_CTRL_CMD_SINGLE         0x80
#define SSD1306_CTRL_DATA_SINGLE        0xC0

typedef struct {
    const sensor_bus_t *bus;
    uint8_t  addr7;
    uint8_t  online;
    uint8_t  fb[SSD1306_FB_SIZE];
    uint8_t  contrast;
    uint8_t  inverted;
    uint16_t flush_chunks;   /* flush 产生了多少次 I2C 事务 */
    uint32_t pixels_set;
} ssd1306_t;

int  ssd1306_init(ssd1306_t *dev, const sensor_bus_t *bus, uint8_t addr7);
int  ssd1306_send_cmd(ssd1306_t *dev, uint8_t cmd);
int  ssd1306_send_cmds(ssd1306_t *dev, const uint8_t *cmds, size_t len);
int  ssd1306_send_data(ssd1306_t *dev, const uint8_t *data, size_t len);
int  ssd1306_display_on(ssd1306_t *dev, uint8_t on);
int  ssd1306_set_contrast(ssd1306_t *dev, uint8_t contrast);
int  ssd1306_invert(ssd1306_t *dev, uint8_t invert);
/* 显存操作 */
void ssd1306_clear(ssd1306_t *dev);
void ssd1306_fill(ssd1306_t *dev, uint8_t pattern);
void ssd1306_draw_pixel(ssd1306_t *dev, int16_t x, int16_t y, uint8_t on);
uint8_t ssd1306_get_pixel(const ssd1306_t *dev, int16_t x, int16_t y);
void ssd1306_draw_hline(ssd1306_t *dev, int16_t x, int16_t y, int16_t w, uint8_t on);
void ssd1306_draw_vline(ssd1306_t *dev, int16_t x, int16_t y, int16_t h, uint8_t on);
void ssd1306_draw_rect(ssd1306_t *dev, int16_t x, int16_t y, int16_t w, int16_t h, uint8_t on);
void ssd1306_draw_line(ssd1306_t *dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t on);
void ssd1306_draw_char(ssd1306_t *dev, int16_t x, int16_t y, char c, uint8_t on);
void ssd1306_draw_string(ssd1306_t *dev, int16_t x, int16_t y, const char *s, uint8_t on);
void ssd1306_draw_string_2x(ssd1306_t *dev, int16_t x, int16_t y, const char *s, uint8_t on);
/* 把一个字节的无符号十进制写到屏幕 */
void ssd1306_draw_u32(ssd1306_t *dev, int16_t x, int16_t y, uint32_t v, uint8_t on);
/* 全屏刷新：按页分成若干次 I2C 事务搬运 */
int  ssd1306_flush(ssd1306_t *dev);
/* 进入低功耗（0xAE）与恢复 */
int  ssd1306_sleep(ssd1306_t *dev, uint8_t enable);

#endif /* OLED_SSD1306_H */
