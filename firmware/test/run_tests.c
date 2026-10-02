/*
 * run_tests.c -- 单元测试总入口
 *
 * 逐套件运行，汇总通过/失败数；任何失败都返回非 0，便于 CI/脚本判定。
 * 额外做一次"整机冒烟"：初始化整个应用（含全部驱动与 5 个任务），
 * 跑 2000 个调度周期（=2 秒虚拟时间），确认没有崩溃且任务都被调度到。
 */
#include "test_util.h"
#include "band_app.h"
#include "sensor_iface.h"
#include "sensor_iface.h"
#include "mpu6050.h"
#include "ft6236.h"
#include "mlx90615.h"
#include "oled_ssd1306.h"
#include "band_config.h"
#include <string.h>
#include <stdio.h>

/* 把仿真器件摆成"整机在线"的样子 */
static void provision_sim_board(void)
{
    uint8_t v;
    uint8_t ee[2];

    sensor_sim_reset();

    /* MPU6050 */
    sensor_sim_add_device(BAND_I2C_ADDR_MPU6050);
    v = 0x68u;
    sensor_sim_poke(BAND_I2C_ADDR_MPU6050, 0x75u, &v, 1u);
    sensor_sim_set_autoclear(BAND_I2C_ADDR_MPU6050, 0x6Bu, 0x80u);
    sensor_sim_set_autoclear(BAND_I2C_ADDR_MPU6050, 0x6Au, 0x04u);
    {
        /* 静止摆放：Z 轴 1g；四元数包全 0 -> 融合会收敛 */
        uint8_t accel[6];
        accel[0] = 0x00; accel[1] = 0x00;
        accel[2] = 0x00; accel[3] = 0x00;
        accel[4] = 0x20; accel[5] = 0x00;   /* az = 8192 @±4g = 1g */
        sensor_sim_poke(BAND_I2C_ADDR_MPU6050, 0x3Bu, accel, sizeof(accel));
        sensor_sim_poke(BAND_I2C_ADDR_MPU6050, 0x41u, (const uint8_t *)"\xF0\xB0", 2u);
    }

    /* FT6236 */
    sensor_sim_add_device(BAND_I2C_ADDR_FT6236);
    v = 0x11u;
    sensor_sim_poke(BAND_I2C_ADDR_FT6236, 0x8Cu, &v, 1u);
    {
        uint8_t regs[14];
        memset(regs, 0, sizeof(regs));
        regs[1] = 0x00u;   /* 无触摸 */
        sensor_sim_poke(BAND_I2C_ADDR_FT6236, 0x02u, regs, sizeof(regs));
    }

    /* MLX90615（SMBus Word + 器件自算 PEC） */
    sensor_sim_add_device(BAND_I2C_ADDR_MLX90615);
    sensor_sim_set_smbus_pec(BAND_I2C_ADDR_MLX90615, 1u);
    {
        uint8_t ram[4];
        uint16_t ta = 14915u;    /* 25.15C */
        uint16_t to = 15500u;    /* 36.85C */
        ram[0] = (uint8_t)(ta & 0xFFu);
        ram[1] = (uint8_t)(ta >> 8);
        ram[2] = (uint8_t)(to & 0xFFu);
        ram[3] = (uint8_t)(to >> 8);
        sensor_sim_poke(BAND_I2C_ADDR_MLX90615, 0x0Cu, ram, sizeof(ram));
    }
    {
        uint16_t emis = 0xFFFFu;
        ee[0] = (uint8_t)(emis & 0xFFu);
        ee[1] = (uint8_t)(emis >> 8);
        sensor_sim_poke(BAND_I2C_ADDR_MLX90615, 0x40u, ee, sizeof(ee));
    }

    /* SSD1306 */
    sensor_sim_add_device(BAND_I2C_ADDR_SSD1306);

    /* 电池：VREFINT 1520 -> VDDA 3259mV；VBAT 码 2500 -> 3978mV */
    sensor_sim_set_adc(17u, 1520u);
    sensor_sim_set_adc(14u, 2500u);

    /* 按键：高电平（未按下） */
    sensor_sim_set_gpio(0x20u, 1u);
}

static void test_app_smoke(void)
{
    band_app_t *app;
    const band_snapshot_t *snap;
    os_sched_t *sched;
    band_uplink_t *up;
    int i;

    TEST_CASE("整机冒烟：全部驱动初始化 + 5 个任务跑 2000 个调度周期");
    provision_sim_board();
    TEST_ASSERT_EQ_INT(band_app_init("127.0.0.1", 0u, NULL), 0, "band_app_init 成功");

    app = band_app_instance();
    TEST_ASSERT(app != NULL, "应用实例");
    sched = band_app_sched();
    TEST_ASSERT(sched != NULL, "调度器实例");
    TEST_ASSERT_EQ_UINT(sched->task_count, 5u, "5 个任务");
    TEST_ASSERT(sched_find_task(sched, "SENSOR") != NULL, "SENSOR 任务存在");
    TEST_ASSERT(sched_find_task(sched, "KEY") != NULL, "KEY 任务存在");
    TEST_ASSERT(sched_find_task(sched, "UI") != NULL, "UI 任务存在");
    TEST_ASSERT(sched_find_task(sched, "COMM") != NULL, "COMM 任务存在");
    TEST_ASSERT(sched_find_task(sched, "POWER") != NULL, "POWER 任务存在");
    TEST_ASSERT(sched_find_task(sched, "SENSOR")->prio > sched_find_task(sched, "COMM")->prio,
                "采集任务优先级应高于通信任务");

    TEST_ASSERT_EQ_INT(band_app_run_steps(2000u), 0, "跑 2000 个周期不崩溃");

    for (i = 0; i < (int)sched->task_count; i++) {
        TEST_ASSERT(sched->task[i].runs > 0u, "每个任务都应被调度到");
    }
    snap = band_app_snapshot();
    TEST_ASSERT(snap != NULL, "快照可读");
    printf("     [data] 任务运行次数: SENSOR=%u KEY=%u UI=%u COMM=%u POWER=%u\n",
           (unsigned)sched_find_task(sched, "SENSOR")->runs,
           (unsigned)sched_find_task(sched, "KEY")->runs,
           (unsigned)sched_find_task(sched, "UI")->runs,
           (unsigned)sched_find_task(sched, "COMM")->runs,
           (unsigned)sched_find_task(sched, "POWER")->runs);
    printf("     [data] 2 秒后快照: 步数=%u 心率=%u 体温=%.2fC 电池=%umV/%u%%\n",
           (unsigned)snap->steps, (unsigned)snap->heart_rate,
           (double)snap->body_temp_c, (unsigned)snap->battery_mv,
           (unsigned)snap->battery_pct);

    TEST_CASE("整机冒烟：驱动确实访问了总线（不是空实现）");
    {
        sensor_sim_t *sim = sensor_sim_instance();
        TEST_ASSERT(sim->i2c_ops > 0u, "I2C 总线有事务");
        TEST_ASSERT(sensor_sim_device(BAND_I2C_ADDR_MPU6050)->read_ops > 0u, "读过 MPU6050");
        TEST_ASSERT(sensor_sim_device(BAND_I2C_ADDR_MPU6050)->write_ops > 0u, "写过 MPU6050");
        TEST_ASSERT(sensor_sim_device(BAND_I2C_ADDR_SSD1306)->write_ops > 0u, "写过 OLED");
        printf("     [data] I2C 事务总数 = %u\n", (unsigned)sim->i2c_ops);
    }

    TEST_CASE("整机冒烟：长时间运行（模拟 60 秒）后进入过低功耗状态");
    up = band_app_uplink();
    TEST_ASSERT(up != NULL, "上行链路实例");
    TEST_ASSERT_EQ_INT(band_app_run_steps(60000u), 0, "再跑 60000 个周期");
    {
        power_mgr_t *pm = band_app_power();
        TEST_ASSERT_EQ_UINT(pm->sleep_count >= 1u, 1u, "空闲足够久后应至少休眠过一次");
        printf("     [data] 低功耗统计: 休眠 %u 次, 唤醒 %u 次, 累计休眠 %u ms\n",
               (unsigned)pm->sleep_count, (unsigned)pm->wake_count,
               (unsigned)pm->total_sleep_ms);
    }

    TEST_CASE("整机冒烟：链路不可用时数据进入离线缓存（不发真实网络包）");
    TEST_ASSERT_EQ_UINT(uplink_pending_count(up) > 0u, 1u, "断网时上传的数据应被缓存");
    printf("     [data] 离线缓存条数 = %u, 在线发送 %u 帧\n",
           (unsigned)uplink_pending_count(up), (unsigned)up->st.tx_frames);

    band_app_dump_stats();
}

int main(void)
{
    memset(&g_tc, 0, sizeof(g_tc));

    printf("======================================================\n");
    printf(" stm32-wearable-health-band 固件单元测试\n");
    printf(" 编译标准: C99   目标: PC 仿真（传感器走 sensor_iface 仿真桩）\n");
    printf("======================================================\n");

    (void)test_frame_codec_run();
    (void)test_step_counter_run();
    (void)test_ws_client_run();
    (void)test_power_mgr_run();
    (void)test_drivers_run();
    (void)test_uplink_run();

    test_suite_begin("app_smoke");
    test_app_smoke();
    (void)test_suite_end();

    printf("\n======================================================\n");
    printf(" 汇总: 用例 %d 个, 断言 %d 条, 失败 %d 条\n",
           g_tc.cases, g_tc.checks, g_tc.failed);
    printf(" 结果: %s\n", (g_tc.failed == 0) ? "ALL TESTS PASSED" : "FAILED");
    printf("======================================================\n");
    return (g_tc.failed == 0) ? 0 : 1;
}
