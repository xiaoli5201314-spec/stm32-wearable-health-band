/*
 * bus_sim.c -- sensor_iface 仿真实现
 *
 * 把 I2C 从机建模成"7bit 地址 -> 256 字节寄存器块"的映射表：
 *   - 写事务：reg[0] 为起始寄存器地址，后续字节依次写入（自动自增，与真实器件一致）
 *   - 读事务：reg[0] 为起始寄存器地址，按 len 连续读出
 *   - 支持注入 NACK 与数据位翻转，用于验证驱动的错误分支
 * 这样 MPU6050 / FT6236 / MLX90615 / SSD1306 四个驱动的寄存器时序
 * 都能在 PC 上被真正执行到，而不是被 #ifdef 掉。
 */
#include "sensor_iface.h"
#include "band_port.h"
#include <string.h>

static sensor_sim_t g_sim;
static int g_sim_ready;

/* 为了不额外占字段，这里用一张独立的地址表 */
static uint8_t g_sim_addr[SENSOR_SIM_DEV_MAX];

sensor_sim_dev_t *sensor_sim_device(uint8_t dev7)
{
    uint8_t i;

    for (i = 0u; i < SENSOR_SIM_DEV_MAX; i++) {
        if ((g_sim.dev[i].present != 0u) && (g_sim_addr[i] == dev7)) {
            return &g_sim.dev[i];
        }
    }
    return NULL;
}

sensor_sim_dev_t *sensor_sim_add_device(uint8_t dev7)
{
    uint8_t i;

    if (g_sim_ready == 0) {
        sensor_sim_reset();
    }
    for (i = 0u; i < SENSOR_SIM_DEV_MAX; i++) {
        if ((g_sim.dev[i].present != 0u) && (g_sim_addr[i] == dev7)) {
            return &g_sim.dev[i];
        }
    }
    for (i = 0u; i < SENSOR_SIM_DEV_MAX; i++) {
        if (g_sim.dev[i].present == 0u) {
            memset(&g_sim.dev[i], 0, sizeof(g_sim.dev[i]));
            g_sim.dev[i].present = 1u;
            g_sim_addr[i] = dev7;
            return &g_sim.dev[i];
        }
    }
    return NULL;
}

/* SMBus PEC 用的 CRC-8：poly 0x07, init 0x00（与 mlx90615 驱动里的实现一致） */
static uint8_t sim_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0u;
    size_t i;
    uint8_t b;

    for (i = 0u; i < len; i++) {
        crc ^= data[i];
        for (b = 0u; b < 8u; b++) {
            crc = (uint8_t)(((crc & 0x80u) != 0u) ? ((uint8_t)(crc << 1) ^ 0x07u)
                                                  : (uint8_t)(crc << 1));
        }
    }
    return crc;
}

static uint8_t sim_pec_read(uint8_t dev7, uint8_t cmd, const uint8_t *d)
{
    uint8_t buf[5];
    buf[0] = (uint8_t)((dev7 << 1) | 0x00u);
    buf[1] = cmd;
    buf[2] = (uint8_t)((dev7 << 1) | 0x01u);
    buf[3] = d[0];
    buf[4] = d[1];
    return sim_crc8(buf, sizeof(buf));
}

static uint8_t sim_pec_write(uint8_t dev7, uint8_t cmd, const uint8_t *d)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)((dev7 << 1) | 0x00u);
    buf[1] = cmd;
    buf[2] = d[0];
    buf[3] = d[1];
    return sim_crc8(buf, sizeof(buf));
}

static int sim_i2c_write(uint8_t dev7, const uint8_t *reg, size_t reg_len,
                         const uint8_t *data, size_t len)
{
    sensor_sim_dev_t *d;
    uint8_t addr;
    size_t i;

    g_sim.i2c_ops++;
    d = sensor_sim_device(dev7);
    if (d == NULL) {
        return SENSOR_ERR_NACK;
    }
    d->write_ops++;
    if (d->nack_next > 0u) {
        d->nack_next--;
        return SENSOR_ERR_NACK;
    }
    if ((reg == NULL) || (reg_len == 0u)) {
        return SENSOR_OK;
    }

    /* SMBus Write Word + PEC：命令是字地址，主机必须给出正确 PEC */
    if ((d->smbus_pec != 0u) && (reg_len == 1u) && (len == 3u) && (data != NULL)) {
        uint8_t want = sim_pec_write(dev7, reg[0], data);
        if (want != data[2]) {
            return SENSOR_ERR_CRC;   /* 主机算错 PEC：器件拒绝写入 */
        }
        d->regs[(uint8_t)(reg[0] * 2u)] = data[0];
        d->regs[(uint8_t)(reg[0] * 2u + 1u)] = data[1];
        return SENSOR_OK;
    }

    addr = reg[0];
    for (i = 0u; i < len; i++) {
        d->regs[(uint8_t)(addr + (uint8_t)i)] = (data != NULL) ? data[i] : 0u;
    }
    /* 模拟硬件自清位（DEVICE_RESET / FIFO_RESET 等）：写完立即归零 */
    if ((d->autoclear_mask != 0u) && (addr <= d->autoclear_reg) &&
        ((uint8_t)(addr + (uint8_t)len) > d->autoclear_reg)) {
        d->regs[d->autoclear_reg] &= (uint8_t)~d->autoclear_mask;
    }
    return SENSOR_OK;
}

static int sim_i2c_read(uint8_t dev7, const uint8_t *reg, size_t reg_len,
                        uint8_t *data, size_t len)
{
    sensor_sim_dev_t *d;
    uint8_t addr;
    size_t i;

    g_sim.i2c_ops++;
    d = sensor_sim_device(dev7);
    if (d == NULL) {
        return SENSOR_ERR_NACK;
    }
    d->read_ops++;
    if (d->nack_next > 0u) {
        d->nack_next--;
        return SENSOR_ERR_NACK;
    }
    if (data == NULL) {
        return SENSOR_ERR_PARAM;
    }

    /* SMBus Read Word + PEC：返回 [DataLow, DataHigh, PEC]，PEC 由器件计算 */
    if ((d->smbus_pec != 0u) && (reg_len == 1u) && (len == 3u) && (reg != NULL)) {
        uint8_t base = (uint8_t)(reg[0] * 2u);
        data[0] = d->regs[base];
        data[1] = d->regs[(uint8_t)(base + 1u)];
        data[2] = sim_pec_read(dev7, reg[0], data);
        if (d->crc_corrupt_next > 0u) {
            d->crc_corrupt_next--;
            data[2] = (uint8_t)(data[2] ^ 0x5Au);   /* 注入坏 PEC */
        }
        return SENSOR_OK;
    }

    addr = ((reg != NULL) && (reg_len > 0u)) ? reg[0] : 0u;
    for (i = 0u; i < len; i++) {
        data[i] = d->regs[(uint8_t)(addr + (uint8_t)i)];
    }
    if (d->crc_corrupt_next > 0u) {
        /* 故障注入：翻转最后一个字节（通常就是 PEC），验证驱动的校验分支 */
        d->crc_corrupt_next--;
        if (len > 0u) {
            data[len - 1u] = (uint8_t)(data[len - 1u] ^ 0x5Au);
        }
    }
    return SENSOR_OK;
}

static int sim_adc_read_raw(uint8_t channel, uint16_t *raw)
{
    if (raw == NULL) {
        return SENSOR_ERR_PARAM;
    }
    *raw = (uint16_t)(g_sim.adc[channel] & 0x0FFFu);   /* 12bit ADC */
    return SENSOR_OK;
}

static int sim_gpio_read(uint8_t pin, uint8_t *level)
{
    if (level == NULL) {
        return SENSOR_ERR_PARAM;
    }
    *level = g_sim.gpio[pin];
    return SENSOR_OK;
}

static uint32_t sim_millis(void)
{
    return g_sim.now_ms;
}

static void sim_delay_ms(uint32_t ms)
{
    g_sim.now_ms += ms;
}

void sensor_sim_reset(void)
{
    memset(&g_sim, 0, sizeof(g_sim));
    memset(g_sim_addr, 0, sizeof(g_sim_addr));
    g_sim.bus.i2c_write = sim_i2c_write;
    g_sim.bus.i2c_read = sim_i2c_read;
    g_sim.bus.adc_read_raw = sim_adc_read_raw;
    g_sim.bus.gpio_read = sim_gpio_read;
    g_sim.bus.millis = sim_millis;
    g_sim.bus.delay_ms = sim_delay_ms;
    g_sim_ready = 1;
}

sensor_sim_t *sensor_sim_instance(void)
{
    if (g_sim_ready == 0) {
        sensor_sim_reset();
    }
    return &g_sim;
}

const sensor_bus_t *sensor_bus_get(void)
{
    return &sensor_sim_instance()->bus;
}

void sensor_sim_set_adc(uint8_t channel, uint16_t raw)
{
    if (g_sim_ready == 0) {
        sensor_sim_reset();
    }
    g_sim.adc[channel] = (uint16_t)(raw & 0x0FFFu);   /* 12bit ADC */
}

void sensor_sim_set_gpio(uint8_t pin, uint8_t level)
{
    if (g_sim_ready == 0) {
        sensor_sim_reset();
    }
    g_sim.gpio[pin] = level;
}

void sensor_sim_set_autoclear(uint8_t dev7, uint8_t reg, uint8_t mask)
{
    sensor_sim_dev_t *d = sensor_sim_add_device(dev7);

    if (d != NULL) {
        d->autoclear_reg = reg;
        d->autoclear_mask = mask;
    }
}

void sensor_sim_set_smbus_pec(uint8_t dev7, uint8_t enable)
{
    sensor_sim_dev_t *d = sensor_sim_add_device(dev7);

    if (d != NULL) {
        d->smbus_pec = (enable != 0u) ? 1u : 0u;
    }
}

void sensor_sim_advance_ms(uint32_t ms)
{
    if (g_sim_ready == 0) {
        sensor_sim_reset();
    }
    g_sim.now_ms += ms;
}

void sensor_sim_poke(uint8_t dev7, uint8_t reg, const uint8_t *data, size_t len)
{
    sensor_sim_dev_t *d = sensor_sim_add_device(dev7);

    if ((d == NULL) || (data == NULL)) {
        return;
    }
    memcpy(&d->regs[reg], data, len);
}

void sensor_sim_peek(uint8_t dev7, uint8_t reg, uint8_t *out, size_t len)
{
    sensor_sim_dev_t *d = sensor_sim_device(dev7);

    if ((d == NULL) || (out == NULL)) {
        return;
    }
    memcpy(out, &d->regs[reg], len);
}

/* ================================================================== */
/* I2C 寄存器辅助（驱动层公共实现，目标板复用同一份）                   */
/* ================================================================== */
int sensor_reg_write8(const sensor_bus_t *bus, uint8_t dev7, uint8_t reg, uint8_t val)
{
    if ((bus == NULL) || (bus->i2c_write == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    return bus->i2c_write(dev7, &reg, 1u, &val, 1u);
}

int sensor_reg_read8(const sensor_bus_t *bus, uint8_t dev7, uint8_t reg, uint8_t *val)
{
    if ((bus == NULL) || (bus->i2c_read == NULL) || (val == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    return bus->i2c_read(dev7, &reg, 1u, val, 1u);
}

int sensor_reg_write16(const sensor_bus_t *bus, uint8_t dev7, uint8_t reg, uint16_t val)
{
    uint8_t buf[2];
    if ((bus == NULL) || (bus->i2c_write == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    buf[0] = (uint8_t)((val >> 8) & 0xFFu);   /* 器件内部多为大端 */
    buf[1] = (uint8_t)(val & 0xFFu);
    return bus->i2c_write(dev7, &reg, 1u, buf, 2u);
}

int sensor_reg_read16(const sensor_bus_t *bus, uint8_t dev7, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    int rc;

    if ((bus == NULL) || (bus->i2c_read == NULL) || (val == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    rc = bus->i2c_read(dev7, &reg, 1u, buf, 2u);
    if (rc != SENSOR_OK) {
        return rc;
    }
    *val = (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    return SENSOR_OK;
}

int sensor_reg_read_buf(const sensor_bus_t *bus, uint8_t dev7, uint8_t reg,
                        uint8_t *buf, size_t len)
{
    if ((bus == NULL) || (bus->i2c_read == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    return bus->i2c_read(dev7, &reg, 1u, buf, len);
}

int sensor_reg_write_buf(const sensor_bus_t *bus, uint8_t dev7, uint8_t reg,
                         const uint8_t *buf, size_t len)
{
    if ((bus == NULL) || (bus->i2c_write == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    return bus->i2c_write(dev7, &reg, 1u, buf, len);
}

int sensor_reg_update_bits(const sensor_bus_t *bus, uint8_t dev7, uint8_t reg,
                           uint8_t mask, uint8_t val)
{
    uint8_t cur = 0u;
    int rc;

    rc = sensor_reg_read8(bus, dev7, reg, &cur);
    if (rc != SENSOR_OK) {
        return rc;
    }
    cur = (uint8_t)((cur & (uint8_t)(~mask)) | (val & mask));
    return sensor_reg_write8(bus, dev7, reg, cur);
}
