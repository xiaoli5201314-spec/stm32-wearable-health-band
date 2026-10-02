/*
 * mlx90615.c -- Melexis MLX90615 红外测温驱动（SMBus + PEC）
 *
 * 读取一个 RAM 字的完整 SMBus 事务（Read Word + PEC）：
 *   S | 0x5A<<1|W | Cmd | Sr | 0x5A<<1|R | DataLow | DataHigh | PEC(from slave) | P
 * 从机返回 PEC，主机必须校验；不匹配说明总线受干扰或器件异常，本次数据作废。
 *
 * PEC 用 CRC-8：多项式 x^8 + x^2 + x + 1（0x07），初值 0x00，不反射，不异或输出。
 * 计算范围（SMBus 2.0 §6.4）：
 *   读：Addr+W, Command, Addr+R, DataLow, DataHigh
 *   写：Addr+W, Command, DataLow, DataHigh
 * 注意：PEC 覆盖的"地址字节"含 R/W 位，且读事务里包含重复起始后的地址字节。
 */
#include "mlx90615.h"
#include "band_port.h"
#include <string.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/* CRC-8 / PEC                                                         */
/* ------------------------------------------------------------------ */
uint8_t mlx90615_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00u;
    size_t i;
    uint8_t b;

    if (data == NULL) {
        return crc;
    }
    for (i = 0u; i < len; i++) {
        crc ^= data[i];
        for (b = 0u; b < 8u; b++) {
            if ((crc & 0x80u) != 0u) {
                crc = (uint8_t)((uint8_t)(crc << 1) ^ MLX90615_PEC_POLY);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

uint8_t mlx90615_pec_read(uint8_t addr7, uint8_t cmd, uint16_t data)
{
    uint8_t buf[5];

    buf[0] = (uint8_t)((addr7 << 1) | 0x00u);   /* Addr + W */
    buf[1] = cmd;
    buf[2] = (uint8_t)((addr7 << 1) | 0x01u);   /* Addr + R（重复起始） */
    buf[3] = (uint8_t)(data & 0xFFu);           /* DataLow 先传 */
    buf[4] = (uint8_t)((data >> 8) & 0xFFu);    /* DataHigh */
    return mlx90615_crc8(buf, sizeof(buf));
}

uint8_t mlx90615_pec_write(uint8_t addr7, uint8_t cmd, uint16_t data)
{
    uint8_t buf[4];

    buf[0] = (uint8_t)((addr7 << 1) | 0x00u);   /* Addr + W */
    buf[1] = cmd;
    buf[2] = (uint8_t)(data & 0xFFu);
    buf[3] = (uint8_t)((data >> 8) & 0xFFu);
    return mlx90615_crc8(buf, sizeof(buf));
}

float mlx90615_raw_to_celsius(uint16_t raw)
{
    /* 数据手册：T = raw * 0.02 K */
    return ((float)raw * MLX90615_LSB_KELVIN) - MLX90615_KELVIN_OFFSET;
}

/* ------------------------------------------------------------------ */
/* 器件访问                                                            */
/* ------------------------------------------------------------------ */
int mlx90615_init(mlx90615_t *dev, const sensor_bus_t *bus, uint8_t addr7)
{
    uint16_t emis = 0u;

    if ((dev == NULL) || (bus == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    memset(dev, 0, sizeof(*dev));
    dev->bus = bus;
    dev->addr7 = addr7;
    dev->emissivity = MLX90615_EMISSIVITY_DEFAULT;

    /* 读一次发射率 EEPROM：既能确认器件在线，也拿到实际设定值 */
    if (mlx90615_read_eeprom(dev, MLX90615_EEPROM_EMISSIVITY, &emis) == SENSOR_OK) {
        dev->emissivity = emis;
        dev->online = 1u;
    } else {
        /* EEPROM 区读失败时退化到"只读温度"，不阻塞整机启动 */
        uint16_t raw = 0u;
        if (mlx90615_read_word(dev, MLX90615_RAM_TA, &raw) == SENSOR_OK) {
            dev->online = 1u;
        } else {
            dev->online = 0u;
            return SENSOR_ERR_NACK;
        }
    }
    return SENSOR_OK;
}

int mlx90615_read_word(mlx90615_t *dev, uint8_t ram_cmd, uint16_t *raw)
{
    uint8_t buf[3];
    uint16_t data;
    uint8_t pec_expect;
    int rc;

    if ((dev == NULL) || (raw == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    dev->reads++;
    /* 读 3 字节：DataLow, DataHigh, PEC */
    rc = dev->bus->i2c_read(dev->addr7, &ram_cmd, 1u, buf, 3u);
    if (rc != SENSOR_OK) {
        if (rc == SENSOR_ERR_NACK) {
            dev->nacks++;
        }
        return rc;
    }
    data = (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    pec_expect = mlx90615_pec_read(dev->addr7, ram_cmd, data);
    if (pec_expect != buf[2]) {
        /* PEC 不匹配：数据不可信，必须拒绝而不是"凑合用" */
        dev->pec_errors++;
        return SENSOR_ERR_CRC;
    }
    *raw = data;
    return SENSOR_OK;
}

int mlx90615_write_word(mlx90615_t *dev, uint8_t cmd, uint16_t value)
{
    uint8_t buf[3];
    int rc;

    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    buf[0] = (uint8_t)(value & 0xFFu);
    buf[1] = (uint8_t)((value >> 8) & 0xFFu);
    buf[2] = mlx90615_pec_write(dev->addr7, cmd, value);
    rc = dev->bus->i2c_write(dev->addr7, &cmd, 1u, buf, 3u);
    if (rc != SENSOR_OK) {
        if (rc == SENSOR_ERR_NACK) {
            dev->nacks++;
        }
        return rc;
    }
    /* EEPROM 写入需要 ≥5ms 的擦写时间，期间不能访问器件 */
    band_delay_ms(10u);
    return SENSOR_OK;
}

int mlx90615_read_eeprom(mlx90615_t *dev, uint8_t eeprom_addr, uint16_t *raw)
{
    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    /* MLX90615 的 EEPROM 段：0x10..0x1F 存 SMBus 地址，0x20..0x2F 存发射率 */
    if (!(((eeprom_addr >= MLX90615_EEPROM_SA) && (eeprom_addr < (MLX90615_EEPROM_SA + 0x10u))) ||
          ((eeprom_addr >= MLX90615_EEPROM_EMISSIVITY) && (eeprom_addr < (MLX90615_EEPROM_EMISSIVITY + 0x10u))))) {
        return SENSOR_ERR_PARAM;
    }
    return mlx90615_read_word(dev, eeprom_addr, raw);
}

int mlx90615_read_ambient_raw(mlx90615_t *dev, uint16_t *raw)
{
    return mlx90615_read_word(dev, MLX90615_RAM_TA, raw);
}

int mlx90615_read_object_raw(mlx90615_t *dev, uint16_t *raw)
{
    return mlx90615_read_word(dev, MLX90615_RAM_TOBJ1, raw);
}

int mlx90615_read_ambient_c(mlx90615_t *dev, float *celsius)
{
    uint16_t raw = 0u;
    int rc;

    if (celsius == NULL) {
        return SENSOR_ERR_PARAM;
    }
    rc = mlx90615_read_ambient_raw(dev, &raw);
    if (rc != SENSOR_OK) {
        return rc;
    }
    *celsius = mlx90615_raw_to_celsius(raw);
    return SENSOR_OK;
}

int mlx90615_read_object_c(mlx90615_t *dev, float *celsius)
{
    uint16_t raw = 0u;
    int rc;

    if (celsius == NULL) {
        return SENSOR_ERR_PARAM;
    }
    rc = mlx90615_read_object_raw(dev, &raw);
    if (rc != SENSOR_OK) {
        return rc;
    }
    *celsius = mlx90615_raw_to_celsius(raw);
    return SENSOR_OK;
}

int mlx90615_read_both_c(mlx90615_t *dev, float *object_c, float *ambient_c)
{
    float o = 0.0f;
    float a = 0.0f;
    int rc;

    rc = mlx90615_read_object_c(dev, &o);
    if (rc != SENSOR_OK) {
        return rc;
    }
    rc = mlx90615_read_ambient_c(dev, &a);
    if (rc != SENSOR_OK) {
        return rc;
    }
    if (object_c != NULL) {
        *object_c = o;
    }
    if (ambient_c != NULL) {
        *ambient_c = a;
    }
    return SENSOR_OK;
}

/*
 * 体温估计：
 * 红外测温读的是"目标表面辐射温度"。手腕表面温度通常比核心体温低 2~3℃，
 * 且受环境温度影响。这里做一个经验补偿：
 *   T_body ≈ T_obj + k1 * (T_obj - T_amb) + k2
 * k1 修正散热梯度，k2 是皮表到核心的偏移，均可在产线上用耳温枪标定后写入参数区。
 */
int mlx90615_read_body_temp_c(mlx90615_t *dev, float *body_c)
{
    float obj = 0.0f;
    float amb = 0.0f;
    float grad;
    int rc;

    if (body_c == NULL) {
        return SENSOR_ERR_PARAM;
    }
    rc = mlx90615_read_both_c(dev, &obj, &amb);
    if (rc != SENSOR_OK) {
        return rc;
    }
    grad = obj - amb;
    /* 梯度大说明散热快（环境冷），需要更多补偿；梯度小说明接近热平衡 */
    *body_c = obj + (0.12f * grad) + 1.6f;

    /* 发射率偏离 1.0 时，读数偏低，按比例粗补偿 */
    if (dev->emissivity != 0u) {
        float e = (float)dev->emissivity / 65535.0f;
        if ((e > 0.5f) && (e <= 1.001f)) {
            *body_c += (1.0f - e) * 3.0f;
        }
    }
    return SENSOR_OK;
}

int mlx90615_set_emissivity(mlx90615_t *dev, float emissivity)
{
    uint16_t code;

    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    if ((emissivity < 0.1f) || (emissivity > 1.0f)) {
        return SENSOR_ERR_PARAM;
    }
    /* 编码：E = code / 65535，四舍五入取整 */
    code = (uint16_t)((emissivity * 65535.0f) + 0.5f);
    if (mlx90615_write_word(dev, MLX90615_EEPROM_EMISSIVITY, code) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    dev->emissivity = code;
    return SENSOR_OK;
}
