/*
 * oled_font.h -- 5x7 ASCII 点阵字库（自建，由 tools/gen_oled_font.py 生成）
 *
 * 每字符 5 列，每列 1 字节：bit0 = 顶行 ... bit6 = 底行（bit7 恒 0）。
 * 覆盖可打印 ASCII 0x20..0x7E。
 */
#ifndef OLED_FONT_H
#define OLED_FONT_H

#include <stdint.h>

#define OLED_FONT_FIRST_CHAR  0x20
#define OLED_FONT_LAST_CHAR   0x7E
#define OLED_FONT_WIDTH       5
#define OLED_FONT_HEIGHT      7
#define OLED_FONT_ADVANCE     6   /* 含 1 列字间距 */

extern const uint8_t oled_font5x7[][OLED_FONT_WIDTH];

/* 取字形，越界返回 '?' 的字形 */
const uint8_t *oled_font_glyph(char c);

#endif /* OLED_FONT_H */
