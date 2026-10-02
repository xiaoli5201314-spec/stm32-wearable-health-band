/*
 * mpu6050.h -- InvenSense MPU-6000/6050 六轴驱动
 *
 * 覆盖：寄存器级初始化（量程 / DLPF / 采样率 / 中断）、突发读原始数据、
 *       温度读取、片上 FIFO 与 DMP 兼容四元数包解析、姿态融合。
 *
 * 说明：DMP 微码镜像是厂商版权固件，本仓库不包含、不复制；
 *       这里实现的是"与 DMP 等价的输出通路"：片上 FIFO 按固定包格式搬运
 *       四元数 + 原始数据，姿态由本仓库自研的互补滤波 / 四元数微分计算，
 *       对外接口与 eMPL 的 dmp_read_fifo() 语义一致（见 mpu6050_dmp_read_fifo）。
 */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stddef.h>
#include "sensor_iface.h"

/* ---------------- 寄存器地址（MPU-6000/6050 Register Map） ---------------- */
#define MPU6050_REG_SELF_TEST_X      0x0D
#define MPU6050_REG_SMPLRT_DIV       0x19  /* 采样率分频：Rate = GyroOut/(1+DIV) */
#define MPU6050_REG_CONFIG           0x1A  /* DLPF_CFG[2:0] + EXT_SYNC_SET[5:3] */
#define MPU6050_REG_GYRO_CONFIG      0x1B  /* FS_SEL[4:3] 量程 ±250/500/1000/2000 dps */
#define MPU6050_REG_ACCEL_CONFIG     0x1C  /* AFS_SEL[4:3] 量程 ±2/4/8/16 g */
#define MPU6050_REG_MOT_THR          0x1F  /* 运动检测阈值 (1 LSB = 2mg @±2g) */
#define MPU6050_REG_MOT_DUR          0x20  /* 运动检测持续时间 (1 LSB = 1ms) */
#define MPU6050_REG_ZRMOT_THR        0x21
#define MPU6050_REG_ZRMOT_DUR        0x22
#define MPU6050_REG_FIFO_EN          0x23  /* 各数据源写入 FIFO 使能 */
#define MPU6050_REG_I2C_MST_CTRL     0x24  /* 作主机时用于外挂磁力计 */
#define MPU6050_REG_INT_PIN_CFG      0x37  /* INT_LEVEL / INT_OPEN / LATCH_INT_EN */
#define MPU6050_REG_INT_ENABLE       0x38  /* DATA_RDY_EN / DMP_INT_EN / MOT_EN */
#define MPU6050_REG_INT_STATUS       0x3A
#define MPU6050_REG_ACCEL_XOUT_H     0x3B  /* 0x3B..0x40 加速度 3x int16 大端 */
#define MPU6050_REG_TEMP_OUT_H       0x41  /* 温度 int16 大端 */
#define MPU6050_REG_GYRO_XOUT_H      0x43  /* 0x43..0x48 角速度 3x int16 大端 */
#define MPU6050_REG_SIGNAL_PATH_RST  0x68  /* 陀螺/加速度/温度信号通路复位 */
#define MPU6050_REG_USER_CTRL        0x6A  /* FIFO_EN / I2C_MST_EN / FIFO_RESET */
#define MPU6050_REG_PWR_MGMT_1       0x6B  /* DEVICE_RESET / SLEEP / CLKSEL */
#define MPU6050_REG_PWR_MGMT_2       0x6C  /* 各轴待机 (STBY_XA..) */
#define MPU6050_REG_BANK_SEL         0x6D  /* DMP 内存 bank 选择 */
#define MPU6050_REG_MEM_START_ADDR   0x6E
#define MPU6050_REG_MEM_R_W          0x6F
#define MPU6050_REG_DMP_CFG_1        0x70
#define MPU6050_REG_DMP_CFG_2        0x71
#define MPU6050_REG_FIFO_COUNTH      0x72
#define MPU6050_REG_FIFO_COUNTL      0x73
#define MPU6050_REG_FIFO_R_W         0x74
#define MPU6050_REG_WHO_AM_I         0x75  /* 固定 0x68 */

/* PWR_MGMT_1 位定义 */
#define MPU6050_PWR1_DEVICE_RESET    0x80
#define MPU6050_PWR1_SLEEP           0x40
#define MPU6050_PWR1_CYCLE           0x20
#define MPU6050_PWR1_TEMP_DIS        0x08
#define MPU6050_PWR1_CLKSEL_PLL_XGYRO 0x01

/* USER_CTRL 位定义 */
#define MPU6050_USERCTRL_FIFO_EN     0x40
#define MPU6050_USERCTRL_I2C_MST_EN  0x20
#define MPU6050_USERCTRL_FIFO_RESET  0x04

/* INT_ENABLE / INT_STATUS 位定义 */
#define MPU6050_INT_DATA_RDY         0x01
#define MPU6050_INT_DMP               0x02
#define MPU6050_INT_PLL_RDY           0x04
#define MPU6050_INT_MOT               0x40
#define MPU6050_INT_FIFO_OFLOW        0x10

/* FIFO_EN 位定义：决定 FIFO 里有哪些数据源 */
#define MPU6050_FIFO_TEMP            0x80
#define MPU6050_FIFO_XG_YG_ZG        0x70
#define MPU6050_FIFO_ACCEL           0x08
#define MPU6050_FIFO_SLV0            0x01

/* SMPLRT_DIV / CONFIG 常用取值对应的 DLPF 带宽 */
#define MPU6050_DLPF_BW_260HZ        0x00
#define MPU6050_DLPF_BW_184HZ        0x01
#define MPU6050_DLPF_BW_94HZ         0x02
#define MPU6050_DLPF_BW_44HZ         0x03
#define MPU6050_DLPF_BW_21HZ         0x04
#define MPU6050_DLPF_BW_10HZ         0x05
#define MPU6050_DLPF_BW_5HZ          0x06

typedef enum {
    MPU6050_GYRO_FS_250 = 0,   /* 131 LSB/(°/s) */
    MPU6050_GYRO_FS_500,       /* 65.5 */
    MPU6050_GYRO_FS_1000,      /* 32.8 */
    MPU6050_GYRO_FS_2000       /* 16.4 */
} mpu6050_gyro_fs_t;

typedef enum {
    MPU6050_ACCEL_FS_2G = 0,   /* 16384 LSB/g */
    MPU6050_ACCEL_FS_4G,       /* 8192 */
    MPU6050_ACCEL_FS_8G,       /* 4096 */
    MPU6050_ACCEL_FS_16G       /* 2048 */
} mpu6050_accel_fs_t;

/* DMP 兼容 FIFO 包：四元数(4 x int32, Q30) + 加速度(3 x int16) + 角速度(3 x int16) */
#define MPU6050_DMP_PACKET_SIZE      28
#define MPU6050_DMP_QUAT_SCALE       1073741824.0f   /* 2^30 */
#define MPU6050_FIFO_CAP             512

typedef struct {
    int16_t ax, ay, az;    /* 原始 LSB */
    int16_t gx, gy, gz;
    int16_t temp_raw;
} mpu6050_raw_t;

typedef struct {
    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;
    float temp_c;
} mpu6050_scaled_t;

typedef struct {
    float w, x, y, z;      /* 单位四元数 */
} mpu6050_quat_t;

typedef struct {
    int32_t ax, ay, az;    /* FIFO 里的原始 LSB */
    int32_t gx, gy, gz;
    mpu6050_quat_t quat;
} mpu6050_dmp_packet_t;

typedef struct {
    const sensor_bus_t *bus;
    uint8_t   addr7;
    uint8_t   online;
    uint8_t   dmp_enabled;
    uint8_t   fifo_enabled;
    uint8_t   who_am_i;
    mpu6050_gyro_fs_t  gyro_fs;
    mpu6050_accel_fs_t accel_fs;
    uint8_t   dlpf_cfg;
    uint8_t   smplrt_div;
    float     gyro_lsb_per_dps;  /* 量程换算系数 */
    float     accel_lsb_per_g;

    /* 姿态融合状态（自研互补滤波） */
    mpu6050_quat_t q;
    float     roll_deg, pitch_deg, yaw_deg;
    uint32_t  dmp_packets;       /* FIFO 里解析出的包数 */
    uint32_t  fifo_overflows;
    uint32_t  errors;
    uint32_t  reads;
} mpu6050_t;

/* ---------------- API ---------------- */
int  mpu6050_init(mpu6050_t *dev, const sensor_bus_t *bus, uint8_t addr7);
int  mpu6050_reset(mpu6050_t *dev);
int  mpu6050_set_gyro_fs(mpu6050_t *dev, mpu6050_gyro_fs_t fs);
int  mpu6050_set_accel_fs(mpu6050_t *dev, mpu6050_accel_fs_t fs);
int  mpu6050_set_dlpf(mpu6050_t *dev, uint8_t dlpf_cfg);
int  mpu6050_set_sample_rate(mpu6050_t *dev, uint16_t rate_hz);
int  mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *raw);
int  mpu6050_scale(const mpu6050_t *dev, const mpu6050_raw_t *raw, mpu6050_scaled_t *out);
int  mpu6050_read_scaled(mpu6050_t *dev, mpu6050_scaled_t *out);
int  mpu6050_read_temperature(mpu6050_t *dev, float *temp_c);
/* 运动检测中断：用于休眠期间唤醒 */
int  mpu6050_enable_motion_wakeup(mpu6050_t *dev, uint16_t threshold_mg, uint16_t duration_ms);
int  mpu6050_read_int_status(mpu6050_t *dev, uint8_t *status);
int  mpu6050_config_interrupt(mpu6050_t *dev, uint8_t int_enable_bits, uint8_t latch, uint8_t active_low);
int  mpu6050_sleep(mpu6050_t *dev, uint8_t enable);
int  mpu6050_self_test(mpu6050_t *dev, uint8_t *who, uint8_t *ok);

/* ---------------- FIFO / DMP ---------------- */
int  mpu6050_fifo_reset(mpu6050_t *dev);
int  mpu6050_dmp_init(mpu6050_t *dev);
int  mpu6050_fifo_count(mpu6050_t *dev, uint16_t *bytes);
int  mpu6050_fifo_read(mpu6050_t *dev, uint8_t *buf, uint16_t len);
/* 解析一个 28 字节 DMP 包 */
int  mpu6050_dmp_parse_packet(const uint8_t *pkt, mpu6050_dmp_packet_t *out);
/* 等价于 eMPL dmp_read_fifo：从 FIFO 取出若干包，更新姿态；packets_out 为取出包数 */
int  mpu6050_dmp_read_fifo(mpu6050_t *dev, mpu6050_dmp_packet_t *pkts, uint8_t max_pkts,
                           uint8_t *packets_out);
/* 自研姿态融合：用陀螺积分 + 加速度重力方向做互补修正 */
void mpu6050_fuse_attitude(mpu6050_t *dev, const float gyro_dps[3],
                           const float accel_g[3], float dt_s);
void mpu6050_quat_to_euler(const mpu6050_quat_t *q, float *roll, float *pitch, float *yaw);
void mpu6050_quat_normalize(mpu6050_quat_t *q);
int  mpu6050_get_attitude(mpu6050_t *dev, float *roll, float *pitch, float *yaw);

#endif /* MPU6050_H */
