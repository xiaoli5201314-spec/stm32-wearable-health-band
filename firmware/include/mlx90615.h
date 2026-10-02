/*
 * mlx90615.h -- Melexis MLX90615 红外温度传感器驱动（SMBus + PEC）
 *
 * 器件特性：
 *   - 16bit SMBus 接口，7bit 地址 0x5A（出厂默认）
 *   - RAM: 0x06 = TA（环境温度），0x07 = TOBJ1（目标温度）
 *   - EEPROM: 0x10..0x1F = SMBus 地址(SA)，0x20..0x2F = 发射率(Emissivity)
 *   - 温度换算：T[K] = raw * 0.02，摄氏度 = raw * 0.02 - 273.15
 *   - 支持 PEC（Packet Error Code）：CRC-8，多项式 x^8+x^2+x+1 = 0x07，初值 0x00
 *
 * PEC 计算范围（SMBus 2.0 规范，Read Word）：
 *   Addr+W, Command, Addr+R, DataLow, DataHigh
 * Write Word 时为：Addr+W, Command, DataLow, DataHigh
 */
#ifndef MLX90615_H
#define MLX90615_H

#include <stdint.h>
#include <stddef.h>
#include "sensor_iface.h"

#define MLX90615_ADDR7_DEFAULT      0x5A
#define MLX90615_RAM_TA             0x06   /* 环境温度 */
#define MLX90615_RAM_TOBJ1          0x07   /* 目标（人体）温度 */
#define MLX90615_EEPROM_SA          0x10   /* SMBus 地址存储区 0x10..0x1F */
#define MLX90615_EEPROM_EMISSIVITY  0x20   /* 发射率存储区 0x20..0x2F */

#define MLX90615_LSB_KELVIN         0.02f  /* 1 LSB = 0.02 K */
#define MLX90615_KELVIN_OFFSET      273.15f
/* 发射率编码约定：E = code / 65535（取值范围 0.1 ~ 1.0），1.0 -> 0xFFFF。
 * 出厂值随批次不同，可用产线工装读取后回写，驱动只负责搬运与补偿计算。 */
#define MLX90615_EMISSIVITY_DEFAULT 0xFFFFu
#define MLX90615_PEC_POLY           0x07u

typedef struct {
    const sensor_bus_t *bus;
    uint8_t  addr7;
    uint8_t  online;
    uint16_t emissivity;
    /* 统计 */
    uint32_t reads;
    uint32_t pec_errors;    /* 读到坏 PEC 被拒绝的次数 */
    uint32_t nacks;
} mlx90615_t;

int  mlx90615_init(mlx90615_t *dev, const sensor_bus_t *bus, uint8_t addr7);
/* 读 16bit RAM 字（带 PEC 校验），raw 为原始码值 */
int  mlx90615_read_word(mlx90615_t *dev, uint8_t ram_cmd, uint16_t *raw);
/* 写 16bit 字（带 PEC） */
int  mlx90615_write_word(mlx90615_t *dev, uint8_t cmd, uint16_t value);
/* 读 EEPROM 区某地址（自动处理 0x10/0x20 段） */
int  mlx90615_read_eeprom(mlx90615_t *dev, uint8_t eeprom_addr, uint16_t *raw);

int  mlx90615_read_ambient_raw(mlx90615_t *dev, uint16_t *raw);
int  mlx90615_read_object_raw(mlx90615_t *dev, uint16_t *raw);
int  mlx90615_read_ambient_c(mlx90615_t *dev, float *celsius);
int  mlx90615_read_object_c(mlx90615_t *dev, float *celsius);
int  mlx90615_read_both_c(mlx90615_t *dev, float *object_c, float *ambient_c);
/* 人体体温估计：目标温度 + 发射率/环境补偿 */
int  mlx90615_read_body_temp_c(mlx90615_t *dev, float *body_c);
int  mlx90615_set_emissivity(mlx90615_t *dev, float emissivity);

/* --- 纯函数，便于单元测试 --- */
uint8_t mlx90615_crc8(const uint8_t *data, size_t len);           /* PEC 按字节流计算 */
uint8_t mlx90615_pec_read(uint8_t addr7, uint8_t cmd, uint16_t data);
uint8_t mlx90615_pec_write(uint8_t addr7, uint8_t cmd, uint16_t data);
float   mlx90615_raw_to_celsius(uint16_t raw);

#endif /* MLX90615_H */
