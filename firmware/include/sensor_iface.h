/*
 * sensor_iface.h -- 传感器抽象总线（I2C / ADC / GPIO / 时间戳）
 *
 * 设计目的：驱动层只调用本接口，不直接触碰 HAL 或寄存器，
 * 因此同一份驱动代码既能跑在 STM32F405 上（platform/bus_stm32.c），
 * 也能在 PC 上跑仿真（platform/bus_sim.c）完成单元测试。
 */
#ifndef SENSOR_IFACE_H
#define SENSOR_IFACE_H

#include <stdint.h>
#include <stddef.h>

/* I2C 传输返回值 */
#define SENSOR_OK            0
#define SENSOR_ERR_PARAM    (-1)
#define SENSOR_ERR_NACK     (-2)   /* 从机无应答：设备未焊 / 地址错 */
#define SENSOR_ERR_BUS      (-3)   /* 总线错误 / 仲裁丢失 */
#define SENSOR_ERR_TIMEOUT  (-4)
#define SENSOR_ERR_CRC      (-5)   /* PEC 校验失败 */

/*
 * I2C 传输语义（与 STM32 I2C 时序一致）：
 *   write: S | dev+W | reg[0..reg_len-1] | data[0..len-1] | P
 *   read : S | dev+W | reg[0..reg_len-1] | Sr | dev+R | data[0..len-1] | P
 * reg_len == 0 时表示"无寄存器地址阶段"的纯读/纯写（SMBus 快速命令、SSD1306 数据流）。
 */
typedef struct sensor_bus_s {
    int  (*i2c_write)(uint8_t dev7, const uint8_t *reg, size_t reg_len,
                      const uint8_t *data, size_t len);
    int  (*i2c_read)(uint8_t dev7, const uint8_t *reg, size_t reg_len,
                     uint8_t *data, size_t len);
    int  (*adc_read_raw)(uint8_t channel, uint16_t *raw);   /* 12bit 单次转换 */
    int  (*gpio_read)(uint8_t pin, uint8_t *level);         /* 中断脚电平 */
    uint32_t (*millis)(void);
    void (*delay_ms)(uint32_t ms);
} sensor_bus_t;

/* 全局总线实例：由平台层提供 */
const sensor_bus_t *sensor_bus_get(void);

/* ---------------- I2C 寄存器辅助（驱动层复用） ---------------- */
int sensor_reg_write8(const sensor_bus_t *bus, uint8_t dev7, uint8_t reg, uint8_t val);
int sensor_reg_read8(const sensor_bus_t *bus, uint8_t dev7, uint8_t reg, uint8_t *val);
int sensor_reg_write16(const sensor_bus_t *bus, uint8_t dev7, uint8_t reg, uint16_t val);
int sensor_reg_read16(const sensor_bus_t *bus, uint8_t dev7, uint8_t reg, uint16_t *val);
/* 连续读：MPU6050 的 14 字节突发读、SSD1306 的显存流都走它 */
int sensor_reg_read_buf(const sensor_bus_t *bus, uint8_t dev7, uint8_t reg,
                        uint8_t *buf, size_t len);
int sensor_reg_write_buf(const sensor_bus_t *bus, uint8_t dev7, uint8_t reg,
                         const uint8_t *buf, size_t len);
/* 读-改-写：只在需要时保留其它位 */
int sensor_reg_update_bits(const sensor_bus_t *bus, uint8_t dev7, uint8_t reg,
                           uint8_t mask, uint8_t val);

/* ================================================================== */
/* 仿真桩：在 PC 上把 I2C 从机模拟成一块 (7bit地址 -> 256B 寄存器) 映射 */
/* ================================================================== */
#define SENSOR_SIM_DEV_MAX   8
#define SENSOR_SIM_REG_MAX   256

typedef struct {
    uint8_t  present;                      /* 该地址上是否挂了设备 */
    uint8_t  regs[SENSOR_SIM_REG_MAX];
    uint32_t read_ops;                     /* 读事务计数，断言驱动真的在读写 */
    uint32_t write_ops;
    uint32_t nack_next;                    /* >0 时下一次事务返回 NACK（注入故障） */
    uint32_t crc_corrupt_next;             /* >0 时下一次读的最后一字节翻转（PEC 故障注入） */
    uint8_t  autoclear_reg;                /* 自清寄存器地址：写完后自动清 autoclear_mask 位，
                                              模拟 DEVICE_RESET / FIFO_RESET 这类硬件自清位 */
    uint8_t  autoclear_mask;
    uint8_t  smbus_pec;                    /* 1 = SMBus Read/Write Word + PEC 模式：
                                              命令字节是"字地址"（字节偏移 = 2*cmd），
                                              器件自己计算并校验 PEC（MLX90615 即如此） */
} sensor_sim_dev_t;

typedef struct {
    sensor_bus_t      bus;
    sensor_sim_dev_t  dev[SENSOR_SIM_DEV_MAX];
    uint16_t          adc[SENSOR_SIM_REG_MAX];
    uint8_t           gpio[SENSOR_SIM_REG_MAX];
    uint32_t          now_ms;
    uint32_t          i2c_ops;
} sensor_sim_t;

sensor_sim_t *sensor_sim_instance(void);   /* 单例，sensor_bus_get() 返回其 bus */
void sensor_sim_reset(void);
sensor_sim_dev_t *sensor_sim_add_device(uint8_t dev7);
sensor_sim_dev_t *sensor_sim_device(uint8_t dev7);
void sensor_sim_set_adc(uint8_t channel, uint16_t raw);
void sensor_sim_set_gpio(uint8_t pin, uint8_t level);
void sensor_sim_advance_ms(uint32_t ms);
/* 配置自清寄存器（例如 MPU6050 的 PWR_MGMT_1.DEVICE_RESET、USER_CTRL.FIFO_RESET） */
void sensor_sim_set_autoclear(uint8_t dev7, uint8_t reg, uint8_t mask);
/* 把设备切成 SMBus Word+PEC 模式（MLX90615/MLX90614 这类器件） */
void sensor_sim_set_smbus_pec(uint8_t dev7, uint8_t enable);

/* 便捷：往设备寄存器块里塞一段数据 */
void sensor_sim_poke(uint8_t dev7, uint8_t reg, const uint8_t *data, size_t len);
void sensor_sim_peek(uint8_t dev7, uint8_t reg, uint8_t *out, size_t len);

#endif /* SENSOR_IFACE_H */
