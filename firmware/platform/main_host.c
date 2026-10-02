/*
 * main_host.c -- PC 端演示入口
 *
 * 把整份固件（驱动 + 调度器 + 上行链路 + 低功耗）在 PC 上跑起来，
 * 用 sensor_iface 仿真桩伪造传感器数据，观察任务调度、计步、体温、
 * 电量、WebSocket 状态与低功耗状态机的实际行为。
 *
 * 用法：
 *   ./build/band_demo [秒数] [ws_host] [ws_port]
 */
#include "band_app.h"
#include "band_config.h"
#include "band_port.h"
#include "band_platform.h"
#include "sensor_iface.h"
#include "mpu6050.h"
#include "ft6236.h"
#include "mlx90615.h"
#include "oled_ssd1306.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* 把仿真板子摆成"器件都在线"的样子 */
static void provision_sim_board(void)
{
    uint8_t v;
    uint8_t regs[14];
    uint8_t accel[6];
    uint8_t ee[2];
    uint8_t ram[4];
    uint16_t ta = 14915u;
    uint16_t to = 15500u;
    uint16_t emis = 0xFFFFu;

    sensor_sim_reset();   /* 会同时初始化仿真总线 */

    sensor_sim_add_device(BAND_I2C_ADDR_MPU6050);
    v = 0x68u;
    sensor_sim_poke(BAND_I2C_ADDR_MPU6050, 0x75u, &v, 1u);
    sensor_sim_set_autoclear(BAND_I2C_ADDR_MPU6050, 0x6Bu, 0x80u);
    sensor_sim_set_autoclear(BAND_I2C_ADDR_MPU6050, 0x6Au, 0x04u);
    accel[0] = 0x00; accel[1] = 0x00;
    accel[2] = 0x00; accel[3] = 0x00;
    accel[4] = 0x20; accel[5] = 0x00;
    sensor_sim_poke(BAND_I2C_ADDR_MPU6050, 0x3Bu, accel, sizeof(accel));
    sensor_sim_poke(BAND_I2C_ADDR_MPU6050, 0x41u, (const uint8_t *)"\xF0\xB0", 2u);

    sensor_sim_add_device(BAND_I2C_ADDR_FT6236);
    v = 0x11u;
    sensor_sim_poke(BAND_I2C_ADDR_FT6236, 0x8Cu, &v, 1u);
    memset(regs, 0, sizeof(regs));
    sensor_sim_poke(BAND_I2C_ADDR_FT6236, 0x02u, regs, sizeof(regs));

    sensor_sim_add_device(BAND_I2C_ADDR_MLX90615);
    sensor_sim_set_smbus_pec(BAND_I2C_ADDR_MLX90615, 1u);
    ram[0] = (uint8_t)(ta & 0xFFu);
    ram[1] = (uint8_t)(ta >> 8);
    ram[2] = (uint8_t)(to & 0xFFu);
    ram[3] = (uint8_t)(to >> 8);
    sensor_sim_poke(BAND_I2C_ADDR_MLX90615, 0x0Cu, ram, sizeof(ram));
    ee[0] = (uint8_t)(emis & 0xFFu);
    ee[1] = (uint8_t)(emis >> 8);
    sensor_sim_poke(BAND_I2C_ADDR_MLX90615, 0x40u, ee, sizeof(ee));

    sensor_sim_add_device(BAND_I2C_ADDR_SSD1306);

    sensor_sim_set_adc(17u, 1520u);   /* VREFINT */
    sensor_sim_set_adc(14u, 2500u);   /* VBAT 分压 */
    sensor_sim_set_gpio(0x20u, 1u);   /* 按键未按下 */
}

/* 往仿真 MPU6050 里写一帧加速度（单位 mg），模拟"佩戴者"的动作 */
static void sim_imu_set_mg(int32_t ax_mg, int32_t ay_mg, int32_t az_mg)
{
    /* ±4g 量程：1g = 8192 LSB */
    int32_t ax = (ax_mg * 8192) / 1000;
    int32_t ay = (ay_mg * 8192) / 1000;
    int32_t az = (az_mg * 8192) / 1000;
    uint8_t regs[6];

    regs[0] = (uint8_t)((ax >> 8) & 0xFF); regs[1] = (uint8_t)(ax & 0xFF);
    regs[2] = (uint8_t)((ay >> 8) & 0xFF); regs[3] = (uint8_t)(ay & 0xFF);
    regs[4] = (uint8_t)((az >> 8) & 0xFF); regs[5] = (uint8_t)(az & 0xFF);
    sensor_sim_poke(BAND_I2C_ADDR_MPU6050, 0x3Bu, regs, sizeof(regs));
}

int main(int argc, char **argv)
{
    uint32_t seconds = (argc > 1) ? (uint32_t)atoi(argv[1]) : 6u;
    const char *host = (argc > 2) ? argv[2] : NULL;
    uint16_t port = (argc > 3) ? (uint16_t)atoi(argv[3]) : (uint16_t)WS_DEFAULT_PORT;
    uint32_t tick;
    uint32_t total_ticks = seconds * 50u;   /* 每 20ms 一个 tick，与 SENSOR 任务周期一致 */

    printf("======================================================\n");
    printf(" %s v%s 主机端演示\n", BAND_FW_NAME, BAND_FW_VERSION_STR);
    printf(" MCU 目标: %s   仿真: 传感器走 sensor_iface 桩\n", BAND_MCU_PART);
    printf(" 运行时长: %u 秒\n", (unsigned)seconds);
    printf("======================================================\n");

    provision_sim_board();
    if (band_app_init(host, port, NULL) != 0) {
        printf("band_app_init 失败\n");
        return 1;
    }

    /* 顺手做一次器件自检输出 */
    {
        mpu6050_t imu;
        ft6236_dev_t tp;
        mlx90615_t ir;
        float body = 0.0f, amb = 0.0f;
        uint8_t who = 0u, ok = 0u;
        uint8_t id = 0u;

        if (mpu6050_init(&imu, sensor_bus_get(), BAND_I2C_ADDR_MPU6050) == SENSOR_OK) {
            (void)mpu6050_self_test(&imu, &who, &ok);
            printf("[self-test] MPU6050 WHO_AM_I=0x%02X ok=%u\n", who, ok);
        } else {
            printf("[self-test] MPU6050 未响应\n");
        }
        if (ft6236_init(&tp, sensor_bus_get(), BAND_I2C_ADDR_FT6236, 128u, 128u) == SENSOR_OK) {
            (void)ft6236_read_chip_id(&tp, &id);
            printf("[self-test] FT6236 chip_id=0x%02X\n", id);
        } else {
            printf("[self-test] FT6236 未响应\n");
        }
        if (mlx90615_init(&ir, sensor_bus_get(), BAND_I2C_ADDR_MLX90615) == SENSOR_OK) {
            (void)mlx90615_read_both_c(&ir, &body, &amb);
            printf("[self-test] MLX90615 obj=%.2fC amb=%.2fC\n", (double)body, (double)amb);
        } else {
            printf("[self-test] MLX90615 未响应\n");
        }
    }

    /* 按 20ms 步进运行：前 1/3 时间静置，之后模拟 100 步/分步行，
     * 这样能在 PC 上直接看到计步、步频、姿态与低功耗状态机的真实变化。 */
    for (tick = 0u; tick < total_ticks; tick++) {
        float t = (float)tick * 0.02f;
        int walking = (tick > (total_ticks / 3u)) ? 1 : 0;

        if (walking != 0) {
            /* 100 步/分 = 1.667Hz，加速度模值在 700~1300mg 之间摆动 */
            float phase = 2.0f * 3.14159265f * 1.6667f * t;
            int32_t mag = 1000 + (int32_t)(300.0f * sinf(phase));
            sim_imu_set_mg(0, 0, mag);
        } else {
            sim_imu_set_mg(0, 0, 1000);
        }

        band_app_run_ms(20u);

        if ((tick % 50u) == 49u) {
            const band_snapshot_t *s = band_app_snapshot();
            printf("[t=%2us] steps=%u cadence=%u hr=%u body=%.2fC batt=%umV/%u%% "
                   "roll=%.1f pitch=%.1f link=%s pending=%u %s\n",
                   (unsigned)(tick / 50u) + 1u, (unsigned)s->steps,
                   (unsigned)s->cadence_spm, (unsigned)s->heart_rate,
                   (double)s->body_temp_c, (unsigned)s->battery_mv,
                   (unsigned)s->battery_pct, (double)s->roll_deg, (double)s->pitch_deg,
                   (band_app_uplink()->link_up != 0u) ? "ONLINE" : "OFFLINE",
                   (unsigned)uplink_pending_count(band_app_uplink()),
                   (walking != 0) ? "[步行中]" : "[静置]");
        }
    }

    band_app_dump_stats();
    return 0;
}
