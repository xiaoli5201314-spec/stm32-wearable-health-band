/*
 * bus_stm32.c -- sensor_iface 的 STM32 目标实现（I2C1 + ADC1 + GPIO）
 *
 * I2C1 主机事务按 RM0090 §27 的时序实现：
 *   写：START -> SB -> 发地址(W) -> ADDR -> 发寄存器 -> 发数据 -> STOP
 *   读：START -> SB -> 发地址(W) -> ADDR -> 发寄存器 -> 重复START -> 地址(R)
 *       -> 收 N-1 字节 -> 关 ACK -> 收最后 1 字节 -> STOP
 * 每步都带超时，避免总线被拉死后固件卡在内核里。
 */
#ifndef BAND_HOST_SIM

#include "sensor_iface.h"
#include "band_config.h"
#include "band_port.h"
#include "stm32f405_regs.h"

#define I2C_TIMEOUT_LOOPS 200000u

static void i2c_gpio_init(void)
{
    /* PB6 = SCL, PB7 = SDA，复用开漏（I2C 必须开漏 + 上拉） */
    uint32_t moder = REG32(GPIO_MODER(PERIPH_GPIOB_BASE));
    moder &= ~((3u << 12) | (3u << 14));
    moder |= (GPIO_MODE_AF << 12) | (GPIO_MODE_AF << 14);
    REG32(GPIO_MODER(PERIPH_GPIOB_BASE)) = moder;
    REG32(GPIO_OTYPER(PERIPH_GPIOB_BASE)) |= (GPIO_OTYPE_OD << 6) | (GPIO_OTYPE_OD << 7);
    REG32(GPIO_OSPEEDR(PERIPH_GPIOB_BASE)) |= (GPIO_OSPEED_VHIGH << 12) | (GPIO_OSPEED_VHIGH << 14);
    REG32(GPIO_PUPDR(PERIPH_GPIOB_BASE)) |= (GPIO_PUPD_PULLUP << 12) | (GPIO_PUPD_PULLUP << 14);
    REG32(GPIO_AFRL(PERIPH_GPIOB_BASE)) =
        (REG32(GPIO_AFRL(PERIPH_GPIOB_BASE)) & ~0x0FF00000u) |
        (GPIO_AF_I2C1 << 24) | (GPIO_AF_I2C1 << 28);
}

static int i2c_init_bus(void)
{
    i2c_gpio_init();
    REG32(I2C_CR1) |= I2C_CR1_SWRST;
    REG32(I2C_CR1) &= ~I2C_CR1_SWRST;
    REG32(I2C_CR1) |= I2C_CR1_PE;

    /* 标准模式 100kHz：CCR = fPCLK1 / (2 * 100kHz) = 42MHz/200k = 210 */
    REG32(I2C_CCR) = (BAND_APB1_HZ / (2u * 100000u)) & 0x0FFFu;
    /* TRISE = (fPCLK1 * 1000ns) + 1 = 43 */
    REG32(I2C_TRISE) = ((BAND_APB1_HZ / 1000000u) + 1u) & 0x3Fu;
    return SENSOR_OK;
}

static int i2c_wait_flag(uint32_t reg, uint32_t mask, uint32_t expect)
{
    uint32_t guard = 0u;

    while (guard++ < I2C_TIMEOUT_LOOPS) {
        uint32_t v = REG32(reg);
        if ((v & mask) != expect) {
            continue;
        }
        return SENSOR_OK;
    }
    return SENSOR_ERR_TIMEOUT;
}

static void i2c_clear_af(void)
{
    REG32(I2C_SR1) &= ~I2C_SR1_AF;
}

static int i2c_start(void)
{
    REG32(I2C_CR1) |= I2C_CR1_START;
    if (i2c_wait_flag(I2C_SR1, I2C_SR1_SB, I2C_SR1_SB) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    return SENSOR_OK;
}

static int i2c_send_addr(uint8_t addr7, uint8_t read)
{
    REG32(I2C_DR) = (uint32_t)((uint32_t)(addr7 << 1) | (read != 0u ? 1u : 0u));
    if (i2c_wait_flag(I2C_SR1, I2C_SR1_ADDR, I2C_SR1_ADDR) != SENSOR_OK) {
        if ((REG32(I2C_SR1) & I2C_SR1_AF) != 0u) {
            i2c_clear_af();
            REG32(I2C_CR1) |= I2C_CR1_STOP;
            return SENSOR_ERR_NACK;
        }
        return SENSOR_ERR_BUS;
    }
    /* 读 SR1 再读 SR2 才能清 ADDR（RM0090 明确要求） */
    (void)REG32(I2C_SR1);
    (void)REG32(I2C_SR2);
    return SENSOR_OK;
}

static int i2c_write_bytes(const uint8_t *data, size_t len)
{
    size_t i;

    for (i = 0u; i < len; i++) {
        if (i2c_wait_flag(I2C_SR1, I2C_SR1_TXE, I2C_SR1_TXE) != SENSOR_OK) {
            return SENSOR_ERR_TIMEOUT;
        }
        REG32(I2C_DR) = (uint32_t)data[i];
    }
    return SENSOR_OK;
}

static void i2c_stop(void)
{
    REG32(I2C_CR1) |= I2C_CR1_STOP;
    /* 等总线真正空闲，否则下一次 START 会失败 */
    (void)i2c_wait_flag(I2C_SR2, I2C_SR2_BUSY, 0u);
}

static int stm_i2c_write(uint8_t dev7, const uint8_t *reg, size_t reg_len,
                         const uint8_t *data, size_t len)
{
    int rc;

    if (i2c_start() != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    rc = i2c_send_addr(dev7, 0u);
    if (rc != SENSOR_OK) {
        return rc;
    }
    if ((reg != NULL) && (reg_len > 0u)) {
        rc = i2c_write_bytes(reg, reg_len);
        if (rc != SENSOR_OK) {
            i2c_stop();
            return rc;
        }
    }
    if ((data != NULL) && (len > 0u)) {
        rc = i2c_write_bytes(data, len);
        if (rc != SENSOR_OK) {
            i2c_stop();
            return rc;
        }
    }
    i2c_stop();
    return SENSOR_OK;
}

static int stm_i2c_read(uint8_t dev7, const uint8_t *reg, size_t reg_len,
                        uint8_t *data, size_t len)
{
    size_t i;
    int rc;

    if ((data == NULL) || (len == 0u)) {
        return SENSOR_ERR_PARAM;
    }
    if (len > 1u) {
        REG32(I2C_CR1) |= I2C_CR1_ACK;
    } else {
        REG32(I2C_CR1) &= ~I2C_CR1_ACK;
    }

    if (i2c_start() != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    rc = i2c_send_addr(dev7, 0u);
    if (rc != SENSOR_OK) {
        return rc;
    }
    if ((reg != NULL) && (reg_len > 0u)) {
        rc = i2c_write_bytes(reg, reg_len);
        if (rc != SENSOR_OK) {
            i2c_stop();
            return rc;
        }
    }
    /* 重复起始 -> 读方向 */
    if (i2c_start() != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    rc = i2c_send_addr(dev7, 1u);
    if (rc != SENSOR_OK) {
        return rc;
    }

    for (i = 0u; i < len; i++) {
        if (i == (len - 1u)) {
            REG32(I2C_CR1) &= ~I2C_CR1_ACK;   /* 最后一字节回 NACK */
            REG32(I2C_CR1) |= I2C_CR1_STOP;
        }
        if (i2c_wait_flag(I2C_SR1, I2C_SR1_RXNE, I2C_SR1_RXNE) != SENSOR_OK) {
            return SENSOR_ERR_TIMEOUT;
        }
        data[i] = (uint8_t)(REG32(I2C_DR) & 0xFFu);
    }
    REG32(I2C_CR1) |= I2C_CR1_ACK;
    return SENSOR_OK;
}

/* ---------------- ADC1：电池分压 + VREFINT ---------------- */
static void adc_gpio_init(void)
{
    /* PC4 = ADC1_IN14，模拟输入 */
    uint32_t moder = REG32(GPIO_MODER(PERIPH_GPIOC_BASE));
    moder |= (GPIO_MODE_ANALOG << 8);
    REG32(GPIO_MODER(PERIPH_GPIOC_BASE)) = moder;
}

static int adc_init_once(void)
{
    static int inited;
    if (inited != 0) {
        return SENSOR_OK;
    }
    adc_gpio_init();
    REG32(ADC_CCR) |= ADC_CCR_TSVREFE;      /* 使能 VREFINT 通道 */
    REG32(ADC_CR1) = ADC_CR1_RES_12B;
    REG32(ADC_CR2) = ADC_CR2_ADON | ADC_CR2_EOCS;
    /* 采样时间：通道 14 用 SMPR1[14:12]，通道 17 用 SMPR1[23:21]，取 480 周期 */
    REG32(ADC_SMPR1) |= (7u << 12) | (7u << 21);
    inited = 1;
    return SENSOR_OK;
}

static int stm_adc_read_raw(uint8_t channel, uint16_t *raw)
{
    uint32_t guard = 0u;

    if (raw == NULL) {
        return SENSOR_ERR_PARAM;
    }
    (void)adc_init_once();

    /* 规则序列长度 1，第一个转换 = channel */
    REG32(ADC_SQR1) = 0u;
    REG32(ADC_SQR3) = (uint32_t)(channel & 0x1Fu);

    REG32(ADC_CR2) |= ADC_CR2_SWSTART;
    while (((REG32(ADC_SR) & ADC_SR_EOC) == 0u) && (guard++ < I2C_TIMEOUT_LOOPS)) {
    }
    if ((REG32(ADC_SR) & ADC_SR_EOC) == 0u) {
        return SENSOR_ERR_TIMEOUT;
    }
    *raw = (uint16_t)(REG32(ADC_DR) & 0xFFFFu);
    return SENSOR_OK;
}

static int stm_gpio_read(uint8_t pin, uint8_t *level)
{
    uint32_t base;
    uint32_t bit;

    if (level == NULL) {
        return SENSOR_ERR_PARAM;
    }
    /* 约定：pin 高 4bit 为端口号(0=A,1=B,2=C)，低 4bit 为引脚号 */
    switch ((pin >> 4) & 0x0Fu) {
    case 0u: base = PERIPH_GPIOA_BASE; break;
    case 1u: base = PERIPH_GPIOB_BASE; break;
    case 2u: base = PERIPH_GPIOC_BASE; break;
    default: return SENSOR_ERR_PARAM;
    }
    bit = (uint32_t)(pin & 0x0Fu);
    *level = (uint8_t)((REG32(GPIO_IDR(base)) >> bit) & 1u);
    return SENSOR_OK;
}

static uint32_t stm_millis(void)
{
    return band_millis();
}

static void stm_delay_ms(uint32_t ms)
{
    band_delay_ms(ms);
}

static sensor_bus_t g_stm_bus;
static int g_stm_bus_init;

const sensor_bus_t *sensor_bus_get(void)
{
    if (g_stm_bus_init == 0) {
        g_stm_bus.i2c_write = stm_i2c_write;
        g_stm_bus.i2c_read = stm_i2c_read;
        g_stm_bus.adc_read_raw = stm_adc_read_raw;
        g_stm_bus.gpio_read = stm_gpio_read;
        g_stm_bus.millis = stm_millis;
        g_stm_bus.delay_ms = stm_delay_ms;
        (void)i2c_init_bus();
        g_stm_bus_init = 1;
    }
    return &g_stm_bus;
}

#endif /* !BAND_HOST_SIM */
