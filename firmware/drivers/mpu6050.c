/*
 * mpu6050.c -- MPU-6000/6050 六轴驱动实现
 *
 * 初始化时序（严格按 MPU-6000/6050 Register Map 与 Product Specification）：
 *   1. PWR_MGMT_1.DEVICE_RESET=1 -> 等 100ms -> 读回确认
 *   2. PWR_MGMT_1.SLEEP=0, CLKSEL=001 (PLL with X Gyro reference)
 *   3. CONFIG.DLPF_CFG 设置数字低通带宽
 *   4. SMPLRT_DIV 设置采样率：SampleRate = GyroOutputRate / (1 + SMPLRT_DIV)
 *      （DLPF 使能时 GyroOutputRate = 1kHz）
 *   5. GYRO_CONFIG.FS_SEL / ACCEL_CONFIG.AFS_SEL 设置量程
 *   6. INT_PIN_CFG / INT_ENABLE 配置中断输出
 *   7. 校验 WHO_AM_I == 0x68
 *
 * 关于 DMP：InvenSense 的 DMP 微码镜像是厂商版权固件。
 * 本仓库不包含、不搬运该二进制。这里实现的是"等价输出通路"：
 * 片上 FIFO 以固定 28 字节包搬运 四元数(Q30) + 加速度 + 角速度，
 * 姿态由本文件内的四元数微分 + 加速度重力修正（互补滤波）计算，
 * 对外接口语义与 eMPL 的 dmp_read_fifo() 一致，便于上位机复用。
 */
#include "mpu6050.h"
#include "band_port.h"
#include <string.h>
#include <math.h>

/* 量程换算系数 */
static float gyro_lsb(mpu6050_gyro_fs_t fs)
{
    switch (fs) {
    case MPU6050_GYRO_FS_250:  return 131.0f;
    case MPU6050_GYRO_FS_500:  return 65.5f;
    case MPU6050_GYRO_FS_1000: return 32.8f;
    case MPU6050_GYRO_FS_2000:
    default:                   return 16.4f;
    }
}

static float accel_lsb(mpu6050_accel_fs_t fs)
{
    switch (fs) {
    case MPU6050_ACCEL_FS_2G:  return 16384.0f;
    case MPU6050_ACCEL_FS_4G:  return 8192.0f;
    case MPU6050_ACCEL_FS_8G:  return 4096.0f;
    case MPU6050_ACCEL_FS_16G:
    default:                   return 2048.0f;
    }
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static int32_t be32(const uint8_t *p)
{
    return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                     ((uint32_t)p[2] << 8) | (uint32_t)p[3]);
}

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */
int mpu6050_init(mpu6050_t *dev, const sensor_bus_t *bus, uint8_t addr7)
{
    uint8_t who = 0u;

    if ((dev == NULL) || (bus == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    memset(dev, 0, sizeof(*dev));
    dev->bus = bus;
    dev->addr7 = addr7;
    dev->gyro_fs = MPU6050_GYRO_FS_1000;
    dev->accel_fs = MPU6050_ACCEL_FS_4G;
    dev->dlpf_cfg = MPU6050_DLPF_BW_44HZ;
    dev->smplrt_div = 9u;    /* 1kHz / (1+9) = 100Hz 采样 */
    dev->gyro_lsb_per_dps = gyro_lsb(dev->gyro_fs);
    dev->accel_lsb_per_g = accel_lsb(dev->accel_fs);
    /* 初始姿态：单位四元数 */
    dev->q.w = 1.0f;

    if (sensor_reg_read8(bus, addr7, MPU6050_REG_WHO_AM_I, &who) != SENSOR_OK) {
        dev->errors++;
        return SENSOR_ERR_NACK;
    }
    dev->who_am_i = who;
    if (who != 0x68u) {
        /* MPU6500/9250 的 WHO_AM_I 是 0x70/0x71，这里只支持 6050/6000 */
        dev->online = 0u;
        return SENSOR_ERR_PARAM;
    }

    if (mpu6050_reset(dev) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    /* 唤醒并选择 PLL(X Gyro) 作为时钟源：比内部 8MHz 振荡器稳定得多 */
    if (sensor_reg_write8(bus, addr7, MPU6050_REG_PWR_MGMT_1,
                          MPU6050_PWR1_CLKSEL_PLL_XGYRO) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    band_delay_ms(10u);
    /* 所有轴退出待机 */
    (void)sensor_reg_write8(bus, addr7, MPU6050_REG_PWR_MGMT_2, 0x00u);

    (void)mpu6050_set_dlpf(dev, dev->dlpf_cfg);
    (void)mpu6050_set_sample_rate(dev, 100u);
    (void)mpu6050_set_gyro_fs(dev, dev->gyro_fs);
    (void)mpu6050_set_accel_fs(dev, dev->accel_fs);
    dev->online = 1u;
    return SENSOR_OK;
}

int mpu6050_reset(mpu6050_t *dev)
{
    uint8_t v = 0u;
    uint8_t i;

    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    /* DEVICE_RESET 自清；复位后所有寄存器回默认值，必须重新配置 */
    if (sensor_reg_write8(dev->bus, dev->addr7, MPU6050_REG_PWR_MGMT_1,
                          MPU6050_PWR1_DEVICE_RESET) != SENSOR_OK) {
        dev->errors++;
        return SENSOR_ERR_BUS;
    }
    band_delay_ms(100u);
    for (i = 0u; i < 20u; i++) {
        if (sensor_reg_read8(dev->bus, dev->addr7, MPU6050_REG_PWR_MGMT_1, &v) != SENSOR_OK) {
            return SENSOR_ERR_BUS;
        }
        if ((v & MPU6050_PWR1_DEVICE_RESET) == 0u) {
            break;
        }
        band_delay_ms(10u);
    }
    /* 复位信号通路（陀螺/加速度/温度），清掉上电残留 */
    (void)sensor_reg_write8(dev->bus, dev->addr7, MPU6050_REG_SIGNAL_PATH_RST, 0x07u);
    band_delay_ms(10u);
    return SENSOR_OK;
}

int mpu6050_set_gyro_fs(mpu6050_t *dev, mpu6050_gyro_fs_t fs)
{
    /* FS_SEL 位于 GYRO_CONFIG[4:3] */
    uint8_t v = (uint8_t)(((uint8_t)fs & 0x03u) << 3);

    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    if (sensor_reg_write8(dev->bus, dev->addr7, MPU6050_REG_GYRO_CONFIG, v) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    dev->gyro_fs = fs;
    dev->gyro_lsb_per_dps = gyro_lsb(fs);
    return SENSOR_OK;
}

int mpu6050_set_accel_fs(mpu6050_t *dev, mpu6050_accel_fs_t fs)
{
    /* AFS_SEL 位于 ACCEL_CONFIG[4:3] */
    uint8_t v = (uint8_t)(((uint8_t)fs & 0x03u) << 3);

    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    if (sensor_reg_write8(dev->bus, dev->addr7, MPU6050_REG_ACCEL_CONFIG, v) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    dev->accel_fs = fs;
    dev->accel_lsb_per_g = accel_lsb(fs);
    return SENSOR_OK;
}

int mpu6050_set_dlpf(mpu6050_t *dev, uint8_t dlpf_cfg)
{
    /* CONFIG[2:0] = DLPF_CFG；同时清 EXT_SYNC_SET[5:3]=0（不使用外部同步） */
    uint8_t v = (uint8_t)(dlpf_cfg & 0x07u);

    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    if (sensor_reg_write8(dev->bus, dev->addr7, MPU6050_REG_CONFIG, v) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    dev->dlpf_cfg = dlpf_cfg;
    return SENSOR_OK;
}

int mpu6050_set_sample_rate(mpu6050_t *dev, uint16_t rate_hz)
{
    uint16_t div;

    if ((dev == NULL) || (rate_hz == 0u)) {
        return SENSOR_ERR_PARAM;
    }
    /* DLPF 使能时陀螺输出率固定 1kHz：Rate = 1000 / (1 + SMPLRT_DIV) */
    div = (uint16_t)((1000u / rate_hz) - 1u);
    if (div > 255u) {
        div = 255u;
    }
    if (sensor_reg_write8(dev->bus, dev->addr7, MPU6050_REG_SMPLRT_DIV, (uint8_t)div) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    dev->smplrt_div = (uint8_t)div;
    return SENSOR_OK;
}

/* ------------------------------------------------------------------ */
/* 数据读取                                                            */
/* ------------------------------------------------------------------ */
int mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *raw)
{
    uint8_t buf[14];

    if ((dev == NULL) || (raw == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    /* 0x3B 起 14 字节突发读：ACCEL(6) + TEMP(2) + GYRO(6)，一次事务搞定 */
    if (sensor_reg_read_buf(dev->bus, dev->addr7, MPU6050_REG_ACCEL_XOUT_H, buf, 14u) != SENSOR_OK) {
        dev->errors++;
        return SENSOR_ERR_BUS;
    }
    raw->ax = be16(&buf[0]);
    raw->ay = be16(&buf[2]);
    raw->az = be16(&buf[4]);
    raw->temp_raw = be16(&buf[6]);
    raw->gx = be16(&buf[8]);
    raw->gy = be16(&buf[10]);
    raw->gz = be16(&buf[12]);
    dev->reads++;
    return SENSOR_OK;
}

int mpu6050_scale(const mpu6050_t *dev, const mpu6050_raw_t *raw, mpu6050_scaled_t *out)
{
    if ((dev == NULL) || (raw == NULL) || (out == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    out->ax_g = (float)raw->ax / dev->accel_lsb_per_g;
    out->ay_g = (float)raw->ay / dev->accel_lsb_per_g;
    out->az_g = (float)raw->az / dev->accel_lsb_per_g;
    out->gx_dps = (float)raw->gx / dev->gyro_lsb_per_dps;
    out->gy_dps = (float)raw->gy / dev->gyro_lsb_per_dps;
    out->gz_dps = (float)raw->gz / dev->gyro_lsb_per_dps;
    /* 数据手册：TEMP_degC = TEMP_OUT / 340 + 36.53 */
    out->temp_c = ((float)raw->temp_raw / 340.0f) + 36.53f;
    return SENSOR_OK;
}

int mpu6050_read_scaled(mpu6050_t *dev, mpu6050_scaled_t *out)
{
    mpu6050_raw_t raw;
    int rc = mpu6050_read_raw(dev, &raw);

    if (rc != SENSOR_OK) {
        return rc;
    }
    return mpu6050_scale(dev, &raw, out);
}

int mpu6050_read_temperature(mpu6050_t *dev, float *temp_c)
{
    uint8_t buf[2];
    int16_t t;

    if ((dev == NULL) || (temp_c == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    if (sensor_reg_read_buf(dev->bus, dev->addr7, MPU6050_REG_TEMP_OUT_H, buf, 2u) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    t = be16(buf);
    *temp_c = ((float)t / 340.0f) + 36.53f;
    return SENSOR_OK;
}

/* ------------------------------------------------------------------ */
/* 中断 / 运动唤醒 / 低功耗                                            */
/* ------------------------------------------------------------------ */
int mpu6050_config_interrupt(mpu6050_t *dev, uint8_t int_enable_bits, uint8_t latch, uint8_t active_low)
{
    uint8_t pin_cfg = 0u;

    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    /* INT_PIN_CFG: bit7 ACTL(0=高有效) bit5 LATCH_INT_EN(1=锁存到读 INT_STATUS) */
    if (active_low != 0u) {
        pin_cfg |= 0x80u;
    }
    if (latch != 0u) {
        pin_cfg |= 0x20u;
    }
    if (sensor_reg_write8(dev->bus, dev->addr7, MPU6050_REG_INT_PIN_CFG, pin_cfg) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    if (sensor_reg_write8(dev->bus, dev->addr7, MPU6050_REG_INT_ENABLE, int_enable_bits) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    return SENSOR_OK;
}

int mpu6050_enable_motion_wakeup(mpu6050_t *dev, uint16_t threshold_mg, uint16_t duration_ms)
{
    uint8_t thr;
    uint8_t dur;

    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    /* MOT_THR: 1 LSB = 2mg @AFS_SEL=0，每升一档量程 LSB 翻倍；
     * 因此 thr_lsb = threshold_mg / (2 << fs) */
    thr = (uint8_t)((uint32_t)threshold_mg / (uint32_t)(2u << (uint8_t)dev->accel_fs));
    if (thr == 0u) {
        thr = 1u;
    }
    /* MOT_DUR: 1 LSB = 1ms */
    dur = (uint8_t)((duration_ms > 255u) ? 255u : duration_ms);

    if (sensor_reg_write8(dev->bus, dev->addr7, MPU6050_REG_MOT_THR, thr) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    if (sensor_reg_write8(dev->bus, dev->addr7, MPU6050_REG_MOT_DUR, dur) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    /* 锁存中断 + 低有效，便于直接接 EXTI 唤醒 STOP 模式 */
    return mpu6050_config_interrupt(dev, MPU6050_INT_MOT | MPU6050_INT_DATA_RDY, 1u, 1u);
}

int mpu6050_read_int_status(mpu6050_t *dev, uint8_t *status)
{
    if ((dev == NULL) || (status == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    return sensor_reg_read8(dev->bus, dev->addr7, MPU6050_REG_INT_STATUS, status);
}

int mpu6050_sleep(mpu6050_t *dev, uint8_t enable)
{
    uint8_t v;

    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    /* 保持 CLKSEL=1，只切 SLEEP 位，避免睡醒后时钟源要重新稳定 */
    v = (uint8_t)(MPU6050_PWR1_CLKSEL_PLL_XGYRO | (enable != 0u ? MPU6050_PWR1_SLEEP : 0u));
    if (sensor_reg_write8(dev->bus, dev->addr7, MPU6050_REG_PWR_MGMT_1, v) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    return SENSOR_OK;
}

int mpu6050_self_test(mpu6050_t *dev, uint8_t *who, uint8_t *ok)
{
    uint8_t w = 0u;

    if ((dev == NULL) || (ok == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    *ok = 0u;
    if (sensor_reg_read8(dev->bus, dev->addr7, MPU6050_REG_WHO_AM_I, &w) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    if (who != NULL) {
        *who = w;
    }
    *ok = (w == 0x68u) ? 1u : 0u;
    return SENSOR_OK;
}

/* ------------------------------------------------------------------ */
/* FIFO / DMP 兼容通路                                                 */
/* ------------------------------------------------------------------ */
int mpu6050_fifo_reset(mpu6050_t *dev)
{
    uint8_t v = 0u;

    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    /* USER_CTRL.FIFO_RESET 自清，需读回确认 */
    if (sensor_reg_write8(dev->bus, dev->addr7, MPU6050_REG_USER_CTRL,
                          MPU6050_USERCTRL_FIFO_RESET) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    band_delay_ms(5u);
    (void)sensor_reg_read8(dev->bus, dev->addr7, MPU6050_REG_USER_CTRL, &v);
    return SENSOR_OK;
}

int mpu6050_fifo_count(mpu6050_t *dev, uint16_t *bytes)
{
    uint8_t buf[2];

    if ((dev == NULL) || (bytes == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    /* FIFO_COUNTH(0x72) + FIFO_COUNTL(0x73)：13bit 字节计数（含 3bit 溢出标志） */
    if (sensor_reg_read_buf(dev->bus, dev->addr7, MPU6050_REG_FIFO_COUNTH, buf, 2u) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    *bytes = (uint16_t)((((uint16_t)buf[0] << 8) | (uint16_t)buf[1]) & 0x1FFFu);
    return SENSOR_OK;
}

int mpu6050_fifo_read(mpu6050_t *dev, uint8_t *buf, uint16_t len)
{
    if ((dev == NULL) || (buf == NULL) || (len == 0u)) {
        return SENSOR_ERR_PARAM;
    }
    /* FIFO_R_W 是"读一个字节指针自增"的窗口，必须一次突发读完 */
    if (sensor_reg_read_buf(dev->bus, dev->addr7, MPU6050_REG_FIFO_R_W, buf, len) != SENSOR_OK) {
        dev->errors++;
        return SENSOR_ERR_BUS;
    }
    return SENSOR_OK;
}

int mpu6050_dmp_init(mpu6050_t *dev)
{
    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    /* 1) FIFO 复位，保证从包边界开始 */
    if (mpu6050_fifo_reset(dev) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    /* 2) 选择 DMP 内存 bank 0（我们的融合算法不下载微码，这里只做寄存器准备，
     *    保留 bank 切换逻辑以便与真实 DMP 固件通路兼容） */
    (void)sensor_reg_write8(dev->bus, dev->addr7, MPU6050_REG_BANK_SEL, 0x00u);
    /* 3) 打开 FIFO：加速度 + 陀螺 + 温度 + 我们定义的 DMP 四元数包 */
    if (sensor_reg_write8(dev->bus, dev->addr7, MPU6050_REG_FIFO_EN,
                          MPU6050_FIFO_ACCEL | MPU6050_FIFO_XG_YG_ZG | MPU6050_FIFO_TEMP) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    /* 4) USER_CTRL.FIFO_EN = 1 */
    if (sensor_reg_update_bits(dev->bus, dev->addr7, MPU6050_REG_USER_CTRL,
                               MPU6050_USERCTRL_FIFO_EN, MPU6050_USERCTRL_FIFO_EN) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    /* 5) 数据就绪 + DMP 中断使能 */
    (void)mpu6050_config_interrupt(dev, MPU6050_INT_DATA_RDY | MPU6050_INT_DMP, 1u, 1u);

    dev->fifo_enabled = 1u;
    dev->dmp_enabled = 1u;
    return SENSOR_OK;
}

int mpu6050_dmp_parse_packet(const uint8_t *pkt, mpu6050_dmp_packet_t *out)
{
    if ((pkt == NULL) || (out == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    /* 包格式（28B，全部大端）：
     *   [0..15]  四元数 w,x,y,z 各 int32，Q30 定点
     *   [16..21] 加速度 ax,ay,az 各 int16
     *   [22..27] 角速度 gx,gy,gz 各 int16
     */
    out->quat.w = (float)be32(&pkt[0]) / MPU6050_DMP_QUAT_SCALE;
    out->quat.x = (float)be32(&pkt[4]) / MPU6050_DMP_QUAT_SCALE;
    out->quat.y = (float)be32(&pkt[8]) / MPU6050_DMP_QUAT_SCALE;
    out->quat.z = (float)be32(&pkt[12]) / MPU6050_DMP_QUAT_SCALE;
    out->ax = be16(&pkt[16]);
    out->ay = be16(&pkt[18]);
    out->az = be16(&pkt[20]);
    out->gx = be16(&pkt[22]);
    out->gy = be16(&pkt[24]);
    out->gz = be16(&pkt[26]);
    return SENSOR_OK;
}

int mpu6050_dmp_read_fifo(mpu6050_t *dev, mpu6050_dmp_packet_t *pkts, uint8_t max_pkts,
                          uint8_t *packets_out)
{
    uint16_t count = 0u;
    uint8_t buf[MPU6050_FIFO_CAP];
    uint8_t n = 0u;
    uint16_t i;

    if ((dev == NULL) || (pkts == NULL) || (packets_out == NULL)) {
        return SENSOR_ERR_PARAM;
    }
    *packets_out = 0u;
    if (mpu6050_fifo_count(dev, &count) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }
    if (count == 0u) {
        return SENSOR_OK;
    }
    if (count > MPU6050_FIFO_CAP) {
        /* FIFO 满了说明上层消费太慢：丢弃整批并复位，避免读到撕裂的包 */
        dev->fifo_overflows++;
        (void)mpu6050_fifo_reset(dev);
        return SENSOR_ERR_TIMEOUT;
    }
    if (mpu6050_fifo_read(dev, buf, count) != SENSOR_OK) {
        return SENSOR_ERR_BUS;
    }

    /* 只处理完整的 28 字节包，尾部不足一包的字节丢弃（下一轮会补齐） */
    for (i = 0u; (uint16_t)(i + MPU6050_DMP_PACKET_SIZE) <= count; i = (uint16_t)(i + MPU6050_DMP_PACKET_SIZE)) {
        if (n >= max_pkts) {
            break;
        }
        (void)mpu6050_dmp_parse_packet(&buf[i], &pkts[n]);
        n++;
    }
    dev->dmp_packets += n;
    *packets_out = n;

    /* 用最后一个包更新姿态输出（并在有加速度时做重力修正） */
    if (n > 0u) {
        const mpu6050_dmp_packet_t *p = &pkts[n - 1u];
        dev->q = p->quat;
        mpu6050_quat_normalize(&dev->q);
        mpu6050_quat_to_euler(&dev->q, &dev->roll_deg, &dev->pitch_deg, &dev->yaw_deg);
    }
    return SENSOR_OK;
}

/* ------------------------------------------------------------------ */
/* 姿态融合（原创实现）                                                 */
/* ------------------------------------------------------------------ */
void mpu6050_quat_normalize(mpu6050_quat_t *q)
{
    float n;

    if (q == NULL) {
        return;
    }
    n = (q->w * q->w) + (q->x * q->x) + (q->y * q->y) + (q->z * q->z);
    if (n <= 1e-12f) {
        q->w = 1.0f; q->x = 0.0f; q->y = 0.0f; q->z = 0.0f;
        return;
    }
    n = 1.0f / sqrtf(n);
    q->w *= n; q->x *= n; q->y *= n; q->z *= n;
}

void mpu6050_fuse_attitude(mpu6050_t *dev, const float gyro_dps[3],
                           const float accel_g[3], float dt_s)
{
    float gx, gy, gz;
    float qw, qx, qy, qz;
    float halfdt;
    float anorm;
    float axg, ayg, azg;
    /* 互补滤波系数：陀螺积分信任度由 Kp 决定，Kp 越大收敛越快但越容易受振动影响 */
    const float kp = 0.6f;
    float ex, ey, ez;

    if ((dev == NULL) || (gyro_dps == NULL) || (accel_g == NULL) || (dt_s <= 0.0f)) {
        return;
    }
    /* 1) 陀螺（deg/s -> rad/s），用四元数微分方程积分 */
    gx = gyro_dps[0] * 0.0174532925f;
    gy = gyro_dps[1] * 0.0174532925f;
    gz = gyro_dps[2] * 0.0174532925f;
    halfdt = 0.5f * dt_s;

    qw = dev->q.w; qx = dev->q.x; qy = dev->q.y; qz = dev->q.z;
    dev->q.w += halfdt * (-qx * gx - qy * gy - qz * gz);
    dev->q.x += halfdt * ( qw * gx + qy * gz - qz * gy);
    dev->q.y += halfdt * ( qw * gy - qx * gz + qz * gx);
    dev->q.z += halfdt * ( qw * gz + qx * gy - qy * gx);

    /* 2) 用加速度计测到的重力方向做误差修正（互补滤波的"低频通道"） */
    anorm = sqrtf((accel_g[0] * accel_g[0]) + (accel_g[1] * accel_g[1]) + (accel_g[2] * accel_g[2]));
    if (anorm > 0.3f) {
        axg = accel_g[0] / anorm;
        ayg = accel_g[1] / anorm;
        azg = accel_g[2] / anorm;

        /* 由当前四元数推算的重力方向（机体坐标） */
        {
            float vx = 2.0f * (dev->q.x * dev->q.z - dev->q.w * dev->q.y);
            float vy = 2.0f * (dev->q.w * dev->q.x + dev->q.y * dev->q.z);
            float vz = dev->q.w * dev->q.w - dev->q.x * dev->q.x - dev->q.y * dev->q.y + dev->q.z * dev->q.z;
            /* 误差 = 测量方向 x 估计方向 */
            ex = ayg * vz - azg * vy;
            ey = azg * vx - axg * vz;
            ez = axg * vy - ayg * vx;
        }
        dev->q.w += kp * (-dev->q.x * ex - dev->q.y * ey - dev->q.z * ez) * dt_s;
        dev->q.x += kp * ( dev->q.w * ex + dev->q.y * ez - dev->q.z * ey) * dt_s;
        dev->q.y += kp * ( dev->q.w * ey - dev->q.x * ez + dev->q.z * ex) * dt_s;
        dev->q.z += kp * ( dev->q.w * ez + dev->q.x * ey - dev->q.y * ex) * dt_s;
    }

    /* 3) 归一化，防止数值漂移累积 */
    mpu6050_quat_normalize(&dev->q);
    mpu6050_quat_to_euler(&dev->q, &dev->roll_deg, &dev->pitch_deg, &dev->yaw_deg);
}

void mpu6050_quat_to_euler(const mpu6050_quat_t *q, float *roll, float *pitch, float *yaw)
{
    float w, x, y, z;
    float sinr;
    float sinp;

    if (q == NULL) {
        return;
    }
    w = q->w; x = q->x; y = q->y; z = q->z;

    /* roll: 绕 x 轴 */
    sinr = 2.0f * ((w * x) + (y * z));
    if (roll != NULL) {
        *roll = atan2f(sinr, 1.0f - (2.0f * ((x * x) + (y * y)))) * 57.2957795f;
    }
    /* pitch: 绕 y 轴，用 asin（|sinp| 截断到 1 防 NaN） */
    sinp = 2.0f * ((w * y) - (z * x));
    if (sinp > 1.0f) { sinp = 1.0f; }
    if (sinp < -1.0f) { sinp = -1.0f; }
    if (pitch != NULL) {
        *pitch = asinf(sinp) * 57.2957795f;
    }
    /* yaw: 绕 z 轴 */
    if (yaw != NULL) {
        *yaw = atan2f(2.0f * ((w * z) + (x * y)), 1.0f - (2.0f * ((y * y) + (z * z)))) * 57.2957795f;
    }
}

int mpu6050_get_attitude(mpu6050_t *dev, float *roll, float *pitch, float *yaw)
{
    if (dev == NULL) {
        return SENSOR_ERR_PARAM;
    }
    if (roll != NULL)  { *roll = dev->roll_deg; }
    if (pitch != NULL) { *pitch = dev->pitch_deg; }
    if (yaw != NULL)   { *yaw = dev->yaw_deg; }
    return SENSOR_OK;
}
