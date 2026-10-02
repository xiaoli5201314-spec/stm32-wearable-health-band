/*
 * test_drivers.c -- 驱动层与调度器测试
 *
 * 覆盖验收标准第 1 条与简历"功能驱动开发"技术点：
 *   - ring buffer / offline cache 的边界与覆盖策略
 *   - MPU6050：WHO_AM_I 校验、量程/采样率寄存器写入、14 字节突发读、
 *               FIFO/DMP 包解析、四元数姿态融合收敛
 *   - FT6236：坐标解析、手势（单击/滑动/长按）、monitor 模式
 *   - MLX90615：SMBus PEC 校验（含故障注入必须拒绝）、温度换算、体温补偿
 *   - SSD1306：画点/画线/字库渲染/flush 事务次数
 *   - 电池：VREFINT 校准、分压换算、OCV 查表、库仑计
 *   - RTC：BCD 编解码、日历换算、写保护解锁、唤醒定时器配置
 *   - IWDG：超时参数选择、任务级监督喂狗、复位现场回读
 *   - 调度器：优先级抢占、同优先级时间片轮转、信号量、消息邮箱
 */
#include "test_util.h"
#include "sensor_iface.h"
#include "ring_buffer.h"
#include "mpu6050.h"
#include "ft6236.h"
#include "mlx90615.h"
#include "oled_ssd1306.h"
#include "oled_font.h"
#include "battery.h"
#include "rtc.h"
#include "iwdg.h"
#include "task_sched.h"
#include "band_platform.h"
#include <string.h>
#include <math.h>
#include <stdint.h>

/* 邮箱消息（固定 16 字节，与 MBOX_MSG_LEN 一致） */
typedef struct {
    uint8_t a;
    uint8_t b;
    uint8_t pad[14];
} band_msg_probe_t;

/* ================================================================== */
/* 1) 环形缓冲与离线缓存                                               */
/* ================================================================== */
static void test_ring_buffer(void)
{
    band_ring_t rb;
    uint8_t out[64];

    TEST_CASE("字节环形缓冲：写入/读出/回绕/容量");
    ring_init(&rb);
    TEST_ASSERT_EQ_UINT(ring_used(&rb), 0u, "初始为空");
    TEST_ASSERT_EQ_UINT(ring_write(&rb, (const uint8_t *)"abcdef", 6u), 6u, "写 6 字节");
    TEST_ASSERT_EQ_UINT(ring_used(&rb), 6u, "已用 6");
    TEST_ASSERT_EQ_UINT(ring_get(&rb, 0u), (uint8_t)'a', "最旧字节");
    TEST_ASSERT_EQ_UINT(ring_get(&rb, 5u), (uint8_t)'f', "最新字节");
    TEST_ASSERT_EQ_UINT(ring_peek(&rb, out, 3u), 3u, "peek 3 字节");
    TEST_ASSERT(memcmp(out, "abc", 3u) == 0, "peek 不移动读指针");
    TEST_ASSERT_EQ_UINT(ring_used(&rb), 6u, "peek 后长度不变");
    TEST_ASSERT_EQ_UINT(ring_read(&rb, out, 4u), 4u, "读 4 字节");
    TEST_ASSERT(memcmp(out, "abcd", 4u) == 0, "读出内容");
    TEST_ASSERT_EQ_UINT(ring_used(&rb), 2u, "剩余 2");
    TEST_ASSERT_EQ_UINT(ring_free(&rb), (uint16_t)(RING_BUFFER_CAP - 2u), "剩余空间");
    TEST_ASSERT_EQ_UINT(ring_drop(&rb, 10u), 2u, "drop 超过长度只删到空");

    TEST_CASE("环形缓冲溢出策略：丢最旧、保最新");
    ring_init(&rb);
    {
        uint8_t big[RING_BUFFER_CAP + 100u];
        size_t i;
        for (i = 0u; i < sizeof(big); i++) {
            big[i] = (uint8_t)(i & 0xFFu);
        }
        TEST_ASSERT_EQ_UINT(ring_write(&rb, big, sizeof(big)), RING_BUFFER_CAP,
                            "写入量被裁到容量");
        TEST_ASSERT(rb.dropped >= 100u, "记录了丢弃字节数");
        TEST_ASSERT_EQ_UINT(ring_get(&rb, (uint16_t)(RING_BUFFER_CAP - 1u)),
                            (uint8_t)((sizeof(big) - 1u) & 0xFFu), "保留的是最新字节");
    }

    TEST_CASE("环形缓冲参数校验");
    ring_init(&rb);
    TEST_ASSERT_EQ_UINT(ring_write(NULL, (const uint8_t *)"x", 1u), 0u, "空句柄写返回 0");
    TEST_ASSERT_EQ_UINT(ring_write(&rb, NULL, 1u), 0u, "空数据返回 0");
    TEST_ASSERT_EQ_UINT(ring_read(&rb, out, 4u), 0u, "空缓冲读返回 0");
    TEST_ASSERT_EQ_UINT(ring_get(&rb, 0u), 0u, "越界 get 返回 0");
    ring_reset(&rb);
    TEST_ASSERT_EQ_UINT(ring_used(&rb), 0u, "reset 后为空");
}

static void test_offline_cache(void)
{
    offline_cache_t c;
    resend_ctx_t r;
    offline_rec_t rec;
    uint16_t seqs[8];
    int i;
    uint16_t s;

    TEST_CASE("离线缓存：入队分配递增序号、按序保存");
    offline_cache_init(&c);
    for (i = 0; i < 8; i++) {
        char buf[16];
        (void)snprintf(buf, sizeof(buf), "rec%d", i);
        s = offline_cache_push(&c, (uint8_t)(0x10 + i), (const uint8_t *)buf, (uint16_t)strlen(buf));
        TEST_ASSERT(s != 0u, "push 应返回非零序号");
        seqs[i] = s;
    }
    TEST_ASSERT_EQ_UINT(c.count, 8u, "8 条记录");
    TEST_ASSERT_EQ_UINT(offline_cache_oldest_seq(&c), seqs[0], "最旧序号");
    TEST_ASSERT_EQ_UINT(offline_cache_newest_seq(&c), seqs[7], "最新序号");
    TEST_ASSERT_EQ_INT(offline_cache_find(&c, seqs[3], &rec), 0, "按键号查找");
    TEST_ASSERT_EQ_STR((const char *)rec.data, "rec3", "记录内容");
    TEST_ASSERT_EQ_INT(offline_cache_contains(&c, 9999u), 0, "不存在的序号");
    TEST_ASSERT_EQ_INT(offline_cache_find(&c, 9999u, &rec), -1, "找不到返回 -1");

    TEST_CASE("离线缓存：按键号删除并统计");
    TEST_ASSERT_EQ_INT(offline_cache_remove(&c, seqs[2]), 0, "删除成功");
    TEST_ASSERT_EQ_UINT(c.count, 7u, "剩 7 条");
    TEST_ASSERT_EQ_INT(offline_cache_contains(&c, seqs[2]), 0, "已删除");
    TEST_ASSERT_EQ_INT(offline_cache_remove(&c, seqs[2]), -1, "重复删除返回 -1");
    TEST_ASSERT_EQ_UINT(c.acked_total, 1u, "确认计数");
    TEST_ASSERT_EQ_INT(offline_cache_find(&c, seqs[3], &rec), 0, "删除后其它记录仍完好");
    TEST_ASSERT_EQ_STR((const char *)rec.data, "rec3", "内容未错位");
    TEST_ASSERT_EQ_INT(offline_cache_find(&c, seqs[7], &rec), 0, "最后一条也完好");
    TEST_ASSERT_EQ_STR((const char *)rec.data, "rec7", "最后一条内容未错位");

    TEST_CASE("缓存满时覆盖最旧记录并计数");
    {
        offline_cache_t c2;
        int k;
        offline_cache_init(&c2);
        for (k = 0; k < (int)OFFLINE_SLOT_MAX + 5; k++) {
            uint8_t payload[4];
            payload[0] = (uint8_t)k;
            (void)offline_cache_push(&c2, MSG_HEALTH_BATCH, payload, 4u);
        }
        TEST_ASSERT_EQ_UINT(c2.count, OFFLINE_SLOT_MAX, "容量上限");
        TEST_ASSERT_EQ_UINT(c2.dropped_total, 5u, "覆盖计数");
        TEST_ASSERT_EQ_UINT(offline_cache_oldest_seq(&c2), 6u, "最旧序号为第 6 条（1..5 被覆盖）");
        TEST_ASSERT_EQ_UINT(c2.stored_total, (uint32_t)OFFLINE_SLOT_MAX + 5u, "累计入队");
    }

    TEST_CASE("超长载荷与空指针被拒绝");
    {
        uint8_t big[OFFLINE_REC_MAX + 1];
        memset(big, 0, sizeof(big));
        TEST_ASSERT_EQ_UINT(offline_cache_push(&c, MSG_LOG, big, sizeof(big)), 0u, "超长拒绝");
        TEST_ASSERT_EQ_UINT(offline_cache_push(&c, MSG_LOG, NULL, 1u), 0u, "空指针拒绝");
    }

    TEST_CASE("补传会话：按序号升序、每条只发一次（不重复）");
    offline_cache_init(&r.cache);
    memset(&r, 0, sizeof(r));
    r.cache.next_seq = 1u;
    for (i = 0; i < 6; i++) {
        uint8_t p[2];
        p[0] = (uint8_t)i;
        p[1] = 0u;
        (void)offline_cache_push(&r.cache, MSG_OFFLINE_BATCH, p, 2u);
    }
    resend_begin(&r);
    TEST_ASSERT_EQ_UINT(r.session_rounds, 1u, "会话轮次");
    {
        uint16_t expect_seq = 1u;
        uint32_t n = 0u;
        while (resend_next(&r, &rec) == 1) {
            TEST_ASSERT_EQ_UINT(rec.seq, expect_seq, "必须严格按序号升序补传");
            expect_seq++;
            n++;
        }
        TEST_ASSERT_EQ_UINT(n, 6u, "恰好取出 6 条");
        TEST_ASSERT_EQ_UINT(r.sent_this_session, 6u, "本会话发送计数");
    }
    TEST_ASSERT_EQ_INT(resend_next(&r, &rec), 0, "同一会话再取应为 0（不重复）");

    TEST_CASE("补传：收到 ACK 后从缓存移除，重复 ACK 计入去重统计");
    TEST_ASSERT_EQ_INT(resend_ack(&r, 1u), 0, "ACK 序号 1");
    TEST_ASSERT_EQ_INT(resend_ack(&r, 1u), -1, "重复 ACK 返回 -1");
    TEST_ASSERT_EQ_UINT(r.dup_suppressed, 1u, "重复计数");
    TEST_ASSERT_EQ_UINT(offline_cache_contains(&r.cache, 1u), 0u, "已移除");
    TEST_ASSERT_EQ_UINT(resend_pending_count(&r), 5u, "剩 5 条待补传");

    TEST_CASE("补传：未 ACK 的记录在新会话中重发（至少一次投递）");
    resend_begin(&r);
    {
        uint32_t n = 0u;
        while (resend_next(&r, &rec) == 1) {
            TEST_ASSERT(rec.seq != 1u, "已 ACK 的序号不应重发");
            n++;
        }
        TEST_ASSERT_EQ_UINT(n, 5u, "应重发 5 条未确认记录");
    }
    /* 全部确认后缓存为空 */
    {
        uint16_t sq;
        for (sq = 2u; sq <= 6u; sq++) {
            TEST_ASSERT_EQ_INT(resend_ack(&r, sq), 0, "逐条确认");
        }
        TEST_ASSERT_EQ_UINT(resend_pending_count(&r), 0u, "缓存清空");
        resend_begin(&r);
        TEST_ASSERT_EQ_INT(resend_next(&r, &rec), 0, "没有待补传记录");
    }
}

/* ================================================================== */
/* 2) MPU6050                                                          */
/* ================================================================== */
static void test_mpu6050(void)
{
    mpu6050_t dev;
    mpu6050_raw_t raw;
    mpu6050_scaled_t sc;
    uint8_t who = 0u;
    uint8_t ok = 0u;
    const sensor_bus_t *bus;

    TEST_CASE("初始化：WHO_AM_I 校验 + 寄存器配置落盘");
    sensor_sim_reset();
    bus = sensor_bus_get();
    sensor_sim_add_device(BAND_I2C_ADDR_MPU6050);
    /* 器件只在 WHO_AM_I 被置为 0x68 时才认为在线 */
    sensor_sim_poke(BAND_I2C_ADDR_MPU6050, MPU6050_REG_WHO_AM_I, (const uint8_t *)"\x68", 1u);
    /* 模拟 DEVICE_RESET 自清 */
    sensor_sim_set_autoclear(BAND_I2C_ADDR_MPU6050, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
    sensor_sim_set_autoclear(BAND_I2C_ADDR_MPU6050, MPU6050_REG_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET);

    TEST_ASSERT_EQ_INT(mpu6050_init(&dev, bus, BAND_I2C_ADDR_MPU6050), SENSOR_OK, "初始化成功");
    TEST_ASSERT_EQ_UINT(dev.online, 1u, "在线标志");
    TEST_ASSERT_EQ_UINT(dev.who_am_i, 0x68u, "WHO_AM_I");
    TEST_ASSERT_EQ_INT(mpu6050_self_test(&dev, &who, &ok), SENSOR_OK, "自检");
    TEST_ASSERT_EQ_UINT(who, 0x68u, "自检读回器件 ID");
    TEST_ASSERT_EQ_UINT(ok, 1u, "自检通过");

    sensor_sim_peek(BAND_I2C_ADDR_MPU6050, MPU6050_REG_PWR_MGMT_1, &who, 1u);
    TEST_ASSERT_EQ_UINT(who & MPU6050_PWR1_SLEEP, 0u, "必须已退出 SLEEP");
    TEST_ASSERT_EQ_UINT(who & 0x07u, MPU6050_PWR1_CLKSEL_PLL_XGYRO, "时钟源应为 PLL(X Gyro)");
    sensor_sim_peek(BAND_I2C_ADDR_MPU6050, MPU6050_REG_CONFIG, &who, 1u);
    TEST_ASSERT_EQ_UINT(who & 0x07u, MPU6050_DLPF_BW_44HZ, "DLPF 配置");
    sensor_sim_peek(BAND_I2C_ADDR_MPU6050, MPU6050_REG_GYRO_CONFIG, &who, 1u);
    TEST_ASSERT_EQ_UINT((who >> 3) & 0x03u, (uint8_t)MPU6050_GYRO_FS_1000, "陀螺量程 ±1000dps");
    sensor_sim_peek(BAND_I2C_ADDR_MPU6050, MPU6050_REG_ACCEL_CONFIG, &who, 1u);
    TEST_ASSERT_EQ_UINT((who >> 3) & 0x03u, (uint8_t)MPU6050_ACCEL_FS_4G, "加速度量程 ±4g");
    sensor_sim_peek(BAND_I2C_ADDR_MPU6050, MPU6050_REG_SMPLRT_DIV, &who, 1u);
    TEST_ASSERT_EQ_UINT(who, 9u, "100Hz 采样：SMPLRT_DIV = 1000/100 - 1 = 9");

    TEST_CASE("WHO_AM_I 错误（例如焊成 MPU6500）应拒绝初始化");
    {
        mpu6050_t d2;
        sensor_sim_reset();
        sensor_sim_add_device(BAND_I2C_ADDR_MPU6050);
        sensor_sim_poke(BAND_I2C_ADDR_MPU6050, MPU6050_REG_WHO_AM_I, (const uint8_t *)"\x70", 1u);
        TEST_ASSERT(mpu6050_init(&d2, sensor_bus_get(), BAND_I2C_ADDR_MPU6050) != SENSOR_OK,
                    "ID 不匹配应失败");
        TEST_ASSERT_EQ_UINT(d2.online, 0u, "不应标记在线");
    }
    TEST_CASE("I2C NACK（器件未焊）应返回错误而不是死循环");
    {
        mpu6050_t d3;
        sensor_sim_reset();
        TEST_ASSERT(mpu6050_init(&d3, sensor_bus_get(), BAND_I2C_ADDR_MPU6050) != SENSOR_OK,
                    "无器件应失败");
    }

    TEST_CASE("14 字节突发读 + 量程换算 + 温度公式");
    sensor_sim_reset();
    bus = sensor_bus_get();
    sensor_sim_add_device(BAND_I2C_ADDR_MPU6050);
    sensor_sim_poke(BAND_I2C_ADDR_MPU6050, MPU6050_REG_WHO_AM_I, (const uint8_t *)"\x68", 1u);
    sensor_sim_set_autoclear(BAND_I2C_ADDR_MPU6050, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
    sensor_sim_set_autoclear(BAND_I2C_ADDR_MPU6050, MPU6050_REG_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET);
    TEST_ASSERT_EQ_INT(mpu6050_init(&dev, bus, BAND_I2C_ADDR_MPU6050), SENSOR_OK, "重新初始化");
    {
        /* ±4g 量程：1g = 8192 LSB；±1000dps：1dps = 32.8 LSB
         * 摆放：Z 轴朝上 -> az = +8192；绕 X 转 100 dps -> gx = 3280
         * 温度：TEMP_OUT = 340*(25 - 36.53) = -3920 -> 大端 0xF0B0 */
        uint8_t pkt[14];
        pkt[0] = 0x00; pkt[1] = 0x00;     /* ax = 0 */
        pkt[2] = 0x00; pkt[3] = 0x00;     /* ay = 0 */
        pkt[4] = 0x20; pkt[5] = 0x00;     /* az = 8192 */
        pkt[6] = 0xF0; pkt[7] = 0xB0;     /* temp = -3920 -> 25.0 C */
        pkt[8] = 0x0C; pkt[9] = 0xD0;     /* gx = 3280 -> 100 dps */
        pkt[10] = 0x00; pkt[11] = 0x00;
        pkt[12] = 0x00; pkt[13] = 0x00;
        sensor_sim_poke(BAND_I2C_ADDR_MPU6050, MPU6050_REG_ACCEL_XOUT_H, pkt, sizeof(pkt));

        TEST_ASSERT_EQ_INT(mpu6050_read_raw(&dev, &raw), SENSOR_OK, "读原始数据");
        TEST_ASSERT_EQ_INT(raw.az, 8192, "az 原始值");
        TEST_ASSERT_EQ_INT(raw.gx, 3280, "gx 原始值");
        TEST_ASSERT_EQ_INT(raw.temp_raw, -3920, "温度原始值");
        TEST_ASSERT_EQ_INT(mpu6050_scale(&dev, &raw, &sc), SENSOR_OK, "换算");
        TEST_ASSERT_EQ_FLOAT(sc.az_g, 1.0f, 0.001f, "az 应为 1g");
        TEST_ASSERT_EQ_FLOAT(sc.gx_dps, 100.0f, 0.1f, "gx 应为 100 dps");
        TEST_ASSERT_EQ_FLOAT(sc.temp_c, 25.0f, 0.05f, "温度应为 25 摄氏度");
    }

    TEST_CASE("量程切换后换算系数同步更新");
    TEST_ASSERT_EQ_INT(mpu6050_set_accel_fs(&dev, MPU6050_ACCEL_FS_2G), SENSOR_OK, "切 ±2g");
    TEST_ASSERT_EQ_FLOAT(dev.accel_lsb_per_g, 16384.0f, 0.01f, "±2g -> 16384 LSB/g");
    TEST_ASSERT_EQ_INT(mpu6050_set_gyro_fs(&dev, MPU6050_GYRO_FS_250), SENSOR_OK, "切 ±250dps");
    TEST_ASSERT_EQ_FLOAT(dev.gyro_lsb_per_dps, 131.0f, 0.01f, "±250dps -> 131 LSB/dps");
    TEST_ASSERT_EQ_INT(mpu6050_set_sample_rate(&dev, 200u), SENSOR_OK, "200Hz");
    {
        uint8_t v = 0u;
        sensor_sim_peek(BAND_I2C_ADDR_MPU6050, MPU6050_REG_SMPLRT_DIV, &v, 1u);
        TEST_ASSERT_EQ_UINT(v, 4u, "200Hz -> DIV = 1000/200 - 1 = 4");
    }
    TEST_ASSERT_EQ_INT(mpu6050_set_dlpf(&dev, MPU6050_DLPF_BW_21HZ), SENSOR_OK, "DLPF 21Hz");
    {
        uint8_t v = 0u;
        sensor_sim_peek(BAND_I2C_ADDR_MPU6050, MPU6050_REG_CONFIG, &v, 1u);
        TEST_ASSERT_EQ_UINT(v & 0x07u, MPU6050_DLPF_BW_21HZ, "DLPF 寄存器");
    }

    TEST_CASE("运动唤醒配置（MOT_THR 按量程折算，1 LSB = 2mg @±2g）");
    TEST_ASSERT_EQ_INT(mpu6050_set_accel_fs(&dev, MPU6050_ACCEL_FS_2G), SENSOR_OK, "±2g");
    TEST_ASSERT_EQ_INT(mpu6050_enable_motion_wakeup(&dev, 160u, 20u), SENSOR_OK, "配置运动唤醒");
    {
        uint8_t thr = 0u;
        uint8_t dur = 0u;
        uint8_t pin = 0u;
        sensor_sim_peek(BAND_I2C_ADDR_MPU6050, MPU6050_REG_MOT_THR, &thr, 1u);
        sensor_sim_peek(BAND_I2C_ADDR_MPU6050, MPU6050_REG_MOT_DUR, &dur, 1u);
        sensor_sim_peek(BAND_I2C_ADDR_MPU6050, MPU6050_REG_INT_PIN_CFG, &pin, 1u);
        TEST_ASSERT_EQ_UINT(thr, 80u, "160mg / 2mg = 80");
        TEST_ASSERT_EQ_UINT(dur, 20u, "持续时间 20ms");
        TEST_ASSERT_EQ_UINT(pin & 0x20u, 0x20u, "中断锁存使能");
        TEST_ASSERT_EQ_UINT(pin & 0x80u, 0x80u, "中断低有效（接 EXTI 唤醒）");
    }

    TEST_CASE("SLEEP 位切换（休眠时保留 CLKSEL）");
    TEST_ASSERT_EQ_INT(mpu6050_sleep(&dev, 1u), SENSOR_OK, "进 SLEEP");
    {
        uint8_t v = 0u;
        sensor_sim_peek(BAND_I2C_ADDR_MPU6050, MPU6050_REG_PWR_MGMT_1, &v, 1u);
        TEST_ASSERT_EQ_UINT(v & MPU6050_PWR1_SLEEP, MPU6050_PWR1_SLEEP, "SLEEP 置位");
        TEST_ASSERT_EQ_UINT(v & 0x07u, MPU6050_PWR1_CLKSEL_PLL_XGYRO, "CLKSEL 保持不变");
    }
    TEST_ASSERT_EQ_INT(mpu6050_sleep(&dev, 0u), SENSOR_OK, "退出 SLEEP");
}

static void test_mpu6050_fifo_dmp(void)
{
    mpu6050_t dev;
    const sensor_bus_t *bus;
    mpu6050_dmp_packet_t pkts[4];
    uint8_t n = 0u;
    uint16_t cnt = 0u;

    TEST_CASE("FIFO 计数读取（13bit 字节计数，高位是溢出标志）");
    sensor_sim_reset();
    bus = sensor_bus_get();
    sensor_sim_add_device(BAND_I2C_ADDR_MPU6050);
    sensor_sim_poke(BAND_I2C_ADDR_MPU6050, MPU6050_REG_WHO_AM_I, (const uint8_t *)"\x68", 1u);
    sensor_sim_set_autoclear(BAND_I2C_ADDR_MPU6050, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
    sensor_sim_set_autoclear(BAND_I2C_ADDR_MPU6050, MPU6050_REG_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET);
    TEST_ASSERT_EQ_INT(mpu6050_init(&dev, bus, BAND_I2C_ADDR_MPU6050), SENSOR_OK, "初始化");
    TEST_ASSERT_EQ_INT(mpu6050_dmp_init(&dev), SENSOR_OK, "DMP/FIFO 初始化");
    TEST_ASSERT_EQ_UINT(dev.fifo_enabled, 1u, "FIFO 使能");
    TEST_ASSERT_EQ_UINT(dev.dmp_enabled, 1u, "DMP 通路使能");
    {
        uint8_t v = 0u;
        sensor_sim_peek(BAND_I2C_ADDR_MPU6050, MPU6050_REG_USER_CTRL, &v, 1u);
        TEST_ASSERT_EQ_UINT(v & MPU6050_USERCTRL_FIFO_EN, MPU6050_USERCTRL_FIFO_EN, "USER_CTRL.FIFO_EN");
        TEST_ASSERT_EQ_UINT(v & MPU6050_USERCTRL_FIFO_RESET, 0u, "FIFO_RESET 应自清");
    }

    sensor_sim_poke(BAND_I2C_ADDR_MPU6050, MPU6050_REG_FIFO_COUNTH,
                    (const uint8_t *)"\x00\x38", 2u);   /* 56 字节 = 2 个 28 字节包 */
    TEST_ASSERT_EQ_INT(mpu6050_fifo_count(&dev, &cnt), SENSOR_OK, "读 FIFO 计数");
    TEST_ASSERT_EQ_UINT(cnt, 56u, "FIFO 计数 56");

    TEST_CASE("DMP 包解析：Q30 四元数 + 原始加速度/角速度");
    {
        uint8_t fifo[56];
        uint8_t pkt[MPU6050_DMP_PACKET_SIZE];
        mpu6050_dmp_packet_t one;
        /* 构造单位四元数 w=2^30 -> 0x40000000；小端序在 FIFO 里是大端 */
        memset(fifo, 0, sizeof(fifo));
        /* 包 1：w=0x40000000, x=y=z=0, az=8192, gx=328 */
        fifo[0] = 0x40; fifo[1] = 0x00; fifo[2] = 0x00; fifo[3] = 0x00;
        fifo[20] = 0x20; fifo[21] = 0x00;   /* az */
        fifo[22] = 0x01; fifo[23] = 0x48;   /* gx = 328 */
        sensor_sim_poke(BAND_I2C_ADDR_MPU6050, MPU6050_REG_FIFO_R_W, fifo, sizeof(fifo));

        memcpy(pkt, fifo, sizeof(pkt));
        TEST_ASSERT_EQ_INT(mpu6050_dmp_parse_packet(pkt, &one), SENSOR_OK, "解析单包");
        TEST_ASSERT_EQ_FLOAT(one.quat.w, 1.0f, 1e-6f, "w = 2^30/2^30 = 1.0");
        TEST_ASSERT_EQ_FLOAT(one.quat.x, 0.0f, 1e-6f, "x = 0");
        TEST_ASSERT_EQ_INT(one.az, 8192, "az");
        TEST_ASSERT_EQ_INT(one.gx, 328, "gx");

        TEST_ASSERT_EQ_INT(mpu6050_dmp_read_fifo(&dev, pkts, 4u, &n), SENSOR_OK, "读 FIFO");
        TEST_ASSERT_EQ_UINT(n, 2u, "应取出 2 个包");
        TEST_ASSERT_EQ_FLOAT(pkts[0].quat.w, 1.0f, 1e-6f, "包 1 四元数");
        TEST_ASSERT_EQ_FLOAT(dev.q.w, 1.0f, 1e-6f, "dev 姿态被更新");
        TEST_ASSERT_EQ_FLOAT(dev.roll_deg, 0.0f, 0.1f, "横滚角 0");
        TEST_ASSERT_EQ_FLOAT(dev.pitch_deg, 0.0f, 0.1f, "俯仰角 0");

        /* 尾部不足一包时应丢弃而不是解析出半个包 */
        sensor_sim_poke(BAND_I2C_ADDR_MPU6050, MPU6050_REG_FIFO_COUNTH,
                        (const uint8_t *)"\x00\x1C", 2u);   /* 28 -> 1 包 */
        TEST_ASSERT_EQ_INT(mpu6050_dmp_read_fifo(&dev, pkts, 4u, &n), SENSOR_OK, "整包");
        TEST_ASSERT_EQ_UINT(n, 1u, "1 个包");
        sensor_sim_poke(BAND_I2C_ADDR_MPU6050, MPU6050_REG_FIFO_COUNTH,
                        (const uint8_t *)"\x00\x10", 2u);   /* 16 -> 0 个完整包 */
        TEST_ASSERT_EQ_INT(mpu6050_dmp_read_fifo(&dev, pkts, 4u, &n), SENSOR_OK, "半包");
        TEST_ASSERT_EQ_UINT(n, 0u, "不足一包应丢弃");
    }

    TEST_CASE("FIFO 溢出保护：超过缓冲容量时复位 FIFO 并报错");
    sensor_sim_poke(BAND_I2C_ADDR_MPU6050, MPU6050_REG_FIFO_COUNTH,
                    (const uint8_t *)"\x1F\xFF", 2u);   /* 8191 字节，远超 512 */
    TEST_ASSERT_EQ_INT(mpu6050_dmp_read_fifo(&dev, pkts, 4u, &n), SENSOR_ERR_TIMEOUT,
                       "溢出应返回超时/错误");
    TEST_ASSERT_EQ_UINT(dev.fifo_overflows, 1u, "溢出计数");
}

static void test_mpu6050_attitude(void)
{
    mpu6050_t dev;
    float gyro[3];
    float accel[3];
    int i;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    TEST_CASE("姿态融合：静止水平放置应收敛到 0 度");
    memset(&dev, 0, sizeof(dev));
    dev.q.w = 1.0f;
    gyro[0] = 0.0f; gyro[1] = 0.0f; gyro[2] = 0.0f;
    accel[0] = 0.0f; accel[1] = 0.0f; accel[2] = 1.0f;
    for (i = 0; i < 500; i++) {
        mpu6050_fuse_attitude(&dev, gyro, accel, 0.01f);
    }
    (void)mpu6050_get_attitude(&dev, &roll, &pitch, &yaw);
    TEST_ASSERT_EQ_FLOAT(roll, 0.0f, 0.5f, "横滚角应收敛到 0");
    TEST_ASSERT_EQ_FLOAT(pitch, 0.0f, 0.5f, "俯仰角应收敛到 0");
    {
        float n = sqrtf(dev.q.w * dev.q.w + dev.q.x * dev.q.x + dev.q.y * dev.q.y + dev.q.z * dev.q.z);
        TEST_ASSERT_EQ_FLOAT(n, 1.0f, 1e-3f, "四元数应保持单位长度");
    }

    TEST_CASE("姿态融合：绕 X 轴倾斜 30 度 -> 横滚角收敛到 30 度");
    /* 加速度计测到的重力方向（机体坐标）= R_x^T * (0,0,1) = (0, sin30, cos30) */
    memset(&dev, 0, sizeof(dev));
    dev.q.w = 1.0f;
    gyro[0] = 0.0f; gyro[1] = 0.0f; gyro[2] = 0.0f;
    accel[0] = 0.0f;
    accel[1] = sinf(30.0f * 3.14159265f / 180.0f);
    accel[2] = cosf(30.0f * 3.14159265f / 180.0f);
    for (i = 0; i < 3000; i++) {
        mpu6050_fuse_attitude(&dev, gyro, accel, 0.005f);
    }
    (void)mpu6050_get_attitude(&dev, &roll, &pitch, &yaw);
    printf("     [data] 绕 X 30 度：收敛 roll=%.2f pitch=%.2f yaw=%.2f（四元数 w=%.4f x=%.4f）\n",
           (double)roll, (double)pitch, (double)yaw, (double)dev.q.w, (double)dev.q.x);
    TEST_ASSERT_EQ_FLOAT(roll, 30.0f, 2.0f, "横滚角应接近 30 度");
    TEST_ASSERT_EQ_FLOAT(pitch, 0.0f, 2.0f, "俯仰角应保持 0");

    TEST_CASE("姿态融合：绕 Y 轴倾斜 30 度 -> 俯仰角收敛到 30 度");
    /* 重力方向 = R_y^T * (0,0,1) = (-sin30, 0, cos30) */
    memset(&dev, 0, sizeof(dev));
    dev.q.w = 1.0f;
    gyro[0] = 0.0f; gyro[1] = 0.0f; gyro[2] = 0.0f;
    accel[0] = -sinf(30.0f * 3.14159265f / 180.0f);
    accel[1] = 0.0f;
    accel[2] = cosf(30.0f * 3.14159265f / 180.0f);
    for (i = 0; i < 3000; i++) {
        mpu6050_fuse_attitude(&dev, gyro, accel, 0.005f);
    }
    (void)mpu6050_get_attitude(&dev, &roll, &pitch, &yaw);
    printf("     [data] 绕 Y 30 度：收敛 roll=%.2f pitch=%.2f yaw=%.2f\n",
           (double)roll, (double)pitch, (double)yaw);
    TEST_ASSERT_EQ_FLOAT(pitch, 30.0f, 2.0f, "俯仰角应接近 30 度");

    TEST_CASE("姿态融合：陀螺积分方向正确（绕 Z 轴 90dps 转 1 秒 ≈ 90 度偏航）");
    memset(&dev, 0, sizeof(dev));
    dev.q.w = 1.0f;
    gyro[0] = 0.0f; gyro[1] = 0.0f; gyro[2] = 90.0f;
    /* 加速度方向给成"纯重力沿 z"，让修正项不干扰偏航 */
    accel[0] = 0.0f; accel[1] = 0.0f; accel[2] = 1.0f;
    for (i = 0; i < 200; i++) {
        mpu6050_fuse_attitude(&dev, gyro, accel, 0.005f);   /* 200 * 5ms = 1s */
    }
    (void)mpu6050_get_attitude(&dev, &roll, &pitch, &yaw);
    printf("     [data] 陀螺积分 1 秒后 yaw=%.2f 度（期望约 90）\n", (double)yaw);
    TEST_ASSERT_EQ_FLOAT(yaw, 90.0f, 6.0f, "偏航角应接近 90 度");

    TEST_CASE("姿态融合：零 dt 与空指针应安全返回");
    mpu6050_fuse_attitude(&dev, gyro, accel, 0.0f);
    mpu6050_fuse_attitude(NULL, gyro, accel, 0.01f);
    mpu6050_fuse_attitude(&dev, NULL, accel, 0.01f);
    mpu6050_fuse_attitude(&dev, gyro, NULL, 0.01f);
    mpu6050_quat_normalize(NULL);
    mpu6050_quat_to_euler(NULL, &roll, &pitch, &yaw);
    TEST_ASSERT(1, "不应崩溃");
}

/* ================================================================== */
/* 3) FT6236                                                           */
/* ================================================================== */
static void test_ft6236(void)
{
    ft6236_dev_t dev;
    ft6236_t touch;
    const sensor_bus_t *bus;
    uint8_t id = 0u;

    TEST_CASE("初始化：读芯片 ID (0x8C 应为 0x11)，写 monitor 参数");
    sensor_sim_reset();
    bus = sensor_bus_get();
    sensor_sim_add_device(BAND_I2C_ADDR_FT6236);
    sensor_sim_poke(BAND_I2C_ADDR_FT6236, FT6236_REG_FOCALTECH_ID,
                    (const uint8_t *)"\x11", 1u);
    sensor_sim_poke(BAND_I2C_ADDR_FT6236, FT6236_REG_LIB_VERSION_H,
                    (const uint8_t *)"\x01\x02", 2u);
    TEST_ASSERT_EQ_INT(ft6236_init(&dev, bus, BAND_I2C_ADDR_FT6236, 128u, 128u), SENSOR_OK,
                       "初始化成功");
    TEST_ASSERT_EQ_UINT(dev.online, 1u, "在线");
    TEST_ASSERT_EQ_UINT(dev.chip_id, 0x11u, "芯片 ID");
    TEST_ASSERT_EQ_UINT(dev.lib_version_h, 0x01u, "库版本高字节");
    TEST_ASSERT_EQ_INT(ft6236_read_chip_id(&dev, &id), SENSOR_OK, "读 ID");
    TEST_ASSERT_EQ_UINT(id, 0x11u, "ID 值");
    {
        uint8_t v = 0u;
        sensor_sim_peek(BAND_I2C_ADDR_FT6236, FT6236_REG_PERIODACTIVE, &v, 1u);
        TEST_ASSERT_EQ_UINT(v, 12u, "活动模式周期 12ms");
        sensor_sim_peek(BAND_I2C_ADDR_FT6236, FT6236_REG_PERIODMONITOR, &v, 1u);
        TEST_ASSERT_EQ_UINT(v, 30u, "monitor 周期 30ms");
        sensor_sim_peek(BAND_I2C_ADDR_FT6236, FT6236_REG_TH_GROUP, &v, 1u);
        TEST_ASSERT_EQ_UINT(v, 40u, "阈值组 40");
        sensor_sim_peek(BAND_I2C_ADDR_FT6236, FT6236_REG_INT_MODE, &v, 1u);
        TEST_ASSERT_EQ_UINT(v, 0u, "中断触发模式");
    }

    TEST_CASE("芯片 ID 不匹配应拒绝（例如焊了 GT911）");
    {
        ft6236_dev_t d2;
        sensor_sim_reset();
        sensor_sim_add_device(BAND_I2C_ADDR_FT6236);
        sensor_sim_poke(BAND_I2C_ADDR_FT6236, FT6236_REG_FOCALTECH_ID,
                        (const uint8_t *)"\x39", 1u);
        TEST_ASSERT(ft6236_init(&d2, sensor_bus_get(), BAND_I2C_ADDR_FT6236, 128u, 128u) != SENSOR_OK,
                    "ID 错误应失败");
    }

    TEST_CASE("触摸坐标解析：X/Y 高 4 位在 XH/YH 的低半字节");
    sensor_sim_reset();
    bus = sensor_bus_get();
    sensor_sim_add_device(BAND_I2C_ADDR_FT6236);
    sensor_sim_poke(BAND_I2C_ADDR_FT6236, FT6236_REG_FOCALTECH_ID, (const uint8_t *)"\x11", 1u);
    TEST_ASSERT_EQ_INT(ft6236_init(&dev, bus, BAND_I2C_ADDR_FT6236, 128u, 128u), SENSOR_OK, "初始化");
    {
        /* GEST_ID=0, TD_STATUS=1, 点1: XH=0x01|event(0=down) -> x=0x0123=291, Y=0x0045=69 */
        uint8_t regs[14];
        memset(regs, 0, sizeof(regs));
        regs[0] = 0x00u;   /* GEST_ID */
        regs[1] = 0x01u;   /* TD_STATUS = 1 点 */
        regs[2] = 0x01u;   /* P1_XH: event=0(down), X 高 4 位 = 1 */
        regs[3] = 0x23u;   /* P1_XL */
        regs[4] = 0x00u;   /* P1_YH: event=0, Y 高 4 位 = 0 */
        regs[5] = 0x45u;   /* P1_YL */
        regs[6] = 0x10u;   /* weight */
        regs[7] = 0x05u;   /* area */
        sensor_sim_poke(BAND_I2C_ADDR_FT6236, FT6236_REG_GEST_ID, regs, sizeof(regs));

        memset(&touch, 0, sizeof(touch));
        TEST_ASSERT_EQ_INT(ft6236_read_touch(&dev, &touch, 1000u), 1, "应解析出 1 个触摸点");
        TEST_ASSERT_EQ_UINT(touch.points, 1u, "点数");
        TEST_ASSERT_EQ_UINT(touch.pt[0].x, 0x123u, "X = 291");
        TEST_ASSERT_EQ_UINT(touch.pt[0].y, 0x045u, "Y = 69");
        TEST_ASSERT_EQ_UINT(touch.pt[0].event, FT6236_EVENT_DOWN, "事件 = 按下");
        TEST_ASSERT_EQ_UINT(touch.pt[0].weight, 0x10u, "压力值");
        TEST_ASSERT_EQ_UINT(dev.polls, 1u, "轮询计数");
        TEST_ASSERT_EQ_UINT(touch.pressed, 1u, "按下状态");
    }

    TEST_CASE("手势：单击（按下 120ms 后抬起，位移 <12px）");
    {
        uint8_t regs[14];
        memset(&touch, 0, sizeof(touch));   /* 每个手势场景都要从"无触摸"状态开始 */
        memset(regs, 0, sizeof(regs));
        regs[1] = 0x01u;
        regs[2] = 0x00u; regs[3] = 0x40u;   /* x = 64 */
        regs[4] = 0x00u; regs[5] = 0x50u;   /* y = 80 */
        sensor_sim_poke(BAND_I2C_ADDR_FT6236, FT6236_REG_GEST_ID, regs, sizeof(regs));
        (void)ft6236_read_touch(&dev, &touch, 1000u);
        /* 抬起：event = 1 (UP) */
        regs[2] = 0x40u;   /* event 位 bit6 = 1 -> UP */
        regs[3] = 0x42u;
        sensor_sim_poke(BAND_I2C_ADDR_FT6236, FT6236_REG_GEST_ID, regs, sizeof(regs));
        (void)ft6236_read_touch(&dev, &touch, 1120u);
        TEST_ASSERT_EQ_UINT(touch.tap, 1u, "应识别为单击");
        TEST_ASSERT_EQ_UINT(touch.pressed, 0u, "已抬起");
    }

    TEST_CASE("手势：长按（按住 900ms）");
    {
        uint8_t regs[14];
        memset(&touch, 0, sizeof(touch));
        memset(regs, 0, sizeof(regs));
        regs[1] = 0x01u;
        regs[2] = 0x00u; regs[3] = 0x20u;
        regs[4] = 0x00u; regs[5] = 0x20u;
        sensor_sim_poke(BAND_I2C_ADDR_FT6236, FT6236_REG_GEST_ID, regs, sizeof(regs));
        (void)ft6236_read_touch(&dev, &touch, 5000u);
        regs[2] = 0x80u;   /* event = 2 (CONTACT) */
        sensor_sim_poke(BAND_I2C_ADDR_FT6236, FT6236_REG_GEST_ID, regs, sizeof(regs));
        (void)ft6236_read_touch(&dev, &touch, 5900u);
        TEST_ASSERT_EQ_UINT(touch.long_press, 1u, "900ms 应判为长按");
        TEST_ASSERT_EQ_UINT(touch.tap, 0u, "长按不应同时是单击");
    }

    TEST_CASE("手势：水平滑动（位移 60px，时长 300ms）");
    {
        uint8_t regs[14];
        memset(&touch, 0, sizeof(touch));
        memset(regs, 0, sizeof(regs));
        regs[1] = 0x01u;
        regs[2] = 0x00u; regs[3] = 0x10u;   /* x = 16 */
        regs[4] = 0x00u; regs[5] = 0x40u;
        sensor_sim_poke(BAND_I2C_ADDR_FT6236, FT6236_REG_GEST_ID, regs, sizeof(regs));
        (void)ft6236_read_touch(&dev, &touch, 7000u);
        /* 移动到 x = 76 */
        regs[2] = 0x80u; regs[3] = 0x4Cu;
        sensor_sim_poke(BAND_I2C_ADDR_FT6236, FT6236_REG_GEST_ID, regs, sizeof(regs));
        (void)ft6236_read_touch(&dev, &touch, 7300u);
        /* 抬起 */
        regs[2] = 0xC0u;
        sensor_sim_poke(BAND_I2C_ADDR_FT6236, FT6236_REG_GEST_ID, regs, sizeof(regs));
        (void)ft6236_read_touch(&dev, &touch, 7400u);
        TEST_ASSERT_EQ_INT(touch.swipe_x, 1, "应识别为右滑");
        TEST_ASSERT_EQ_UINT(touch.tap, 0u, "滑动不应算单击");
        printf("     [data] 滑动方向 = %d（+1 表示右滑）\n", (int)touch.swipe_x);
    }

    TEST_CASE("monitor 模式切换（休眠节能 + 保持唤醒能力）");
    TEST_ASSERT_EQ_INT(ft6236_set_power_mode(&dev, 1u), SENSOR_OK, "进 monitor");
    TEST_ASSERT_EQ_INT(ft6236_in_monitor(&dev), 1, "monitor 状态");
    {
        uint8_t v = 0u;
        sensor_sim_peek(BAND_I2C_ADDR_FT6236, FT6236_REG_CTRL, &v, 1u);
        TEST_ASSERT_EQ_UINT(v & 0x01u, 0x01u, "CTRL bit0 使能自动进 monitor");
    }
    TEST_ASSERT_EQ_INT(ft6236_set_power_mode(&dev, 0u), SENSOR_OK, "回到活动模式");
    TEST_ASSERT_EQ_INT(ft6236_in_monitor(&dev), 0, "活动状态");

    TEST_CASE("纯解析函数：非法参数与多点裁剪");
    {
        ft6236_t t2;
        uint8_t regs[14];
        memset(regs, 0, sizeof(regs));
        regs[1] = 0x05u;   /* 声称 5 个点，但本驱动只跟踪 2 个 */
        TEST_ASSERT_EQ_INT(ft6236_parse_regs(regs, sizeof(regs), 128u, 128u, &t2), FT6236_MAX_POINTS,
                           "点数被裁剪到最大支持数");
        TEST_ASSERT_EQ_UINT(t2.points, FT6236_MAX_POINTS, "点数上限");
        TEST_ASSERT_EQ_INT(ft6236_parse_regs(NULL, 14u, 128u, 128u, &t2), SENSOR_ERR_PARAM, "空指针");
        TEST_ASSERT_EQ_INT(ft6236_parse_regs(regs, 4u, 128u, 128u, &t2), SENSOR_ERR_PARAM,
                           "数据不足");
    }
}

/* ================================================================== */
/* 4) MLX90615                                                         */
/* ================================================================== */
static void test_mlx90615(void)
{
    mlx90615_t dev;
    const sensor_bus_t *bus;
    uint16_t raw = 0u;
    float obj = 0.0f;
    float amb = 0.0f;
    uint16_t obj_raw;

    TEST_CASE("PEC（CRC-8, poly 0x07）标准校验");
    {
        /* CRC-8/ATM 标准向量：'1'..'9' -> 0xF4 */
        uint8_t crc = mlx90615_crc8((const uint8_t *)"123456789", 9u);
        TEST_ASSERT_EQ_UINT(crc, 0xF4u, "CRC-8 \"123456789\" 应为 0xF4");
        TEST_ASSERT_EQ_UINT(mlx90615_crc8(NULL, 0u), 0u, "空输入 CRC-8 应为 0");
    }

    TEST_CASE("温度换算：raw * 0.02 - 273.15");
    TEST_ASSERT_EQ_FLOAT(mlx90615_raw_to_celsius(0u), -273.15f, 0.01f, "0K -> -273.15C");
    TEST_ASSERT_EQ_FLOAT(mlx90615_raw_to_celsius(14915u), 25.15f, 0.01f, "14915 -> 25.15C");
    TEST_ASSERT_EQ_FLOAT(mlx90615_raw_to_celsius(15480u), 36.45f, 0.01f, "15480 -> 36.45C");

    TEST_CASE("SMBus Read Word + PEC 校验通过");
    sensor_sim_reset();
    bus = sensor_bus_get();
    sensor_sim_add_device(BAND_I2C_ADDR_MLX90615);
    sensor_sim_set_smbus_pec(BAND_I2C_ADDR_MLX90615, 1u);
    {
        /* 器件按"字地址"组织：字地址 cmd 的 2 个数据字节在 regs[2*cmd], regs[2*cmd+1]，
         * 第 3 个字节（PEC）由器件实时计算，不占地址空间 —— 与真实 MLX90615 一致。 */
        uint8_t ram[4];
        uint16_t ta_raw = 14915u;    /* 25.15 C 环境 */
        uint16_t to_raw = 15500u;    /* 36.85 C 目标 */
        /* 字地址 0x06 = TA -> 字节偏移 0x0C；字地址 0x07 = TOBJ1 -> 字节偏移 0x0E */
        ram[0] = (uint8_t)(ta_raw & 0xFFu);
        ram[1] = (uint8_t)(ta_raw >> 8);
        ram[2] = (uint8_t)(to_raw & 0xFFu);
        ram[3] = (uint8_t)(to_raw >> 8);
        sensor_sim_poke(BAND_I2C_ADDR_MLX90615, MLX90615_RAM_TA * 2u, ram, sizeof(ram));
    }
    /* EEPROM 发射率：字地址 0x20 -> 字节偏移 0x40 */
    {
        uint8_t ee[2];
        uint16_t emis = MLX90615_EMISSIVITY_DEFAULT;
        ee[0] = (uint8_t)(emis & 0xFFu);
        ee[1] = (uint8_t)(emis >> 8);
        sensor_sim_poke(BAND_I2C_ADDR_MLX90615, MLX90615_EEPROM_EMISSIVITY * 2u, ee, sizeof(ee));
    }
    TEST_ASSERT_EQ_INT(mlx90615_init(&dev, bus, BAND_I2C_ADDR_MLX90615), SENSOR_OK, "初始化");
    TEST_ASSERT_EQ_UINT(dev.online, 1u, "在线");
    TEST_ASSERT_EQ_UINT(dev.emissivity, MLX90615_EMISSIVITY_DEFAULT, "发射率读回");

    TEST_ASSERT_EQ_INT(mlx90615_read_ambient_raw(&dev, &raw), SENSOR_OK, "读环境温度");
    TEST_ASSERT_EQ_UINT(raw, 14915u, "环境原始值");
    obj_raw = 15500u;
    TEST_ASSERT_EQ_INT(mlx90615_read_object_raw(&dev, &raw), SENSOR_OK, "读目标温度");
    TEST_ASSERT_EQ_UINT(raw, obj_raw, "目标原始值");
    TEST_ASSERT_EQ_INT(mlx90615_read_both_c(&dev, &obj, &amb), SENSOR_OK, "读双温度");
    TEST_ASSERT_EQ_FLOAT(obj, 36.85f, 0.05f, "目标温度 36.85C");
    TEST_ASSERT_EQ_FLOAT(amb, 25.15f, 0.05f, "环境温度 25.15C");
    TEST_ASSERT_EQ_UINT(dev.pec_errors, 0u, "无 PEC 错误");

    TEST_CASE("PEC 校验失败必须拒绝数据（故障注入）");
    {
        uint32_t before = dev.pec_errors;
        sensor_sim_device(BAND_I2C_ADDR_MLX90615)->crc_corrupt_next = 1u;
        TEST_ASSERT_EQ_INT(mlx90615_read_object_raw(&dev, &raw), SENSOR_ERR_CRC,
                           "坏 PEC 应返回 CRC 错误");
        TEST_ASSERT_EQ_UINT(dev.pec_errors, before + 1u, "PEC 错误计数 +1");
    }

    TEST_CASE("I2C NACK 应被正确上报");
    {
        uint32_t before = dev.nacks;
        sensor_sim_device(BAND_I2C_ADDR_MLX90615)->nack_next = 1u;
        TEST_ASSERT_EQ_INT(mlx90615_read_ambient_raw(&dev, &raw), SENSOR_ERR_NACK, "NACK 上报");
        TEST_ASSERT_EQ_UINT(dev.nacks, before + 1u, "NACK 计数");
    }

    TEST_CASE("体温估计：目标温 + 散热梯度补偿 + 发射率补偿");
    {
        float body = 0.0f;
        TEST_ASSERT_EQ_INT(mlx90615_read_body_temp_c(&dev, &body), SENSOR_OK, "读体温");
        /* obj=36.85, amb=25.15 -> grad=11.70 -> body = 36.85 + 1.404 + 1.6 = 39.854
         * 发射率=1.0 -> 补偿 0 */
        printf("     [data] MLX90615 obj=%.2f amb=%.2f -> 估计体温 %.2f C\n",
               (double)obj, (double)amb, (double)body);
        TEST_ASSERT(body > 36.0f && body < 40.0f, "体温估计应在合理区间");
        TEST_ASSERT_EQ_FLOAT(body, 39.854f, 0.05f, "体温补偿公式");
    }

    TEST_CASE("发射率写入（带 PEC）与非法值拒绝");
    TEST_ASSERT_EQ_INT(mlx90615_set_emissivity(&dev, 0.95f), SENSOR_OK, "写发射率 0.95");
    {
        uint16_t code = (uint16_t)(0.95f * 65535.0f + 0.5f);
        uint8_t ee[2];
        sensor_sim_peek(BAND_I2C_ADDR_MLX90615, MLX90615_EEPROM_EMISSIVITY * 2u, ee, sizeof(ee));
        TEST_ASSERT_EQ_UINT((uint16_t)(ee[0] | ((uint16_t)ee[1] << 8)), code, "写入的编码值");
        TEST_ASSERT_EQ_UINT(dev.emissivity, code, "回声发射率编码");
    }
    TEST_ASSERT_EQ_INT(mlx90615_set_emissivity(&dev, 1.5f), SENSOR_ERR_PARAM, "非法值拒绝");
    TEST_ASSERT_EQ_INT(mlx90615_set_emissivity(&dev, 0.0f), SENSOR_ERR_PARAM, "0 拒绝");

    TEST_CASE("主机写 PEC 算错时器件应拒绝（仿真校验写 PEC）");
    {
        uint8_t bad[3];
        uint8_t cmd = 0x20u;
        bad[0] = 0x00u;
        bad[1] = 0x00u;
        bad[2] = 0x5Au;   /* 故意错误的 PEC */
        TEST_ASSERT_EQ_INT(sensor_bus_get()->i2c_write(BAND_I2C_ADDR_MLX90615, &cmd, 1u,
                                                       bad, 3u), SENSOR_ERR_CRC,
                           "写 PEC 错误应被器件拒绝");
    }

    TEST_CASE("EEPROM 地址范围校验");
    TEST_ASSERT_EQ_INT(mlx90615_read_eeprom(&dev, 0x00u, &raw), SENSOR_ERR_PARAM, "非 EEPROM 段拒绝");
    TEST_ASSERT_EQ_INT(mlx90615_read_eeprom(&dev, 0x30u, &raw), SENSOR_ERR_PARAM, "越界拒绝");
    TEST_ASSERT_EQ_INT(mlx90615_read_word(NULL, MLX90615_RAM_TA, &raw), SENSOR_ERR_PARAM, "空指针");
}

/* ================================================================== */
/* 5) SSD1306                                                          */
/* ================================================================== */
static void test_ssd1306(void)
{
    ssd1306_t oled;
    const sensor_bus_t *bus;
    sensor_sim_dev_t *simdev;

    TEST_CASE("初始化序列下发成功并清零显存");
    sensor_sim_reset();
    bus = sensor_bus_get();
    sensor_sim_add_device(BAND_I2C_ADDR_SSD1306);
    TEST_ASSERT_EQ_INT(ssd1306_init(&oled, bus, BAND_I2C_ADDR_SSD1306), SENSOR_OK, "初始化");
    TEST_ASSERT_EQ_UINT(oled.online, 1u, "在线");
    simdev = sensor_sim_device(BAND_I2C_ADDR_SSD1306);
    TEST_ASSERT(simdev != NULL, "仿真器件存在");
    TEST_ASSERT(simdev->write_ops >= 9u, "至少 9 次 I2C 事务（初始化 + 8 页 flush）");
    {
        uint32_t i;
        uint32_t nonzero = 0u;
        for (i = 0u; i < sizeof(oled.fb); i++) {
            if (oled.fb[i] != 0u) {
                nonzero++;
            }
        }
        TEST_ASSERT_EQ_UINT(nonzero, 0u, "显存应被清零");
    }

    TEST_CASE("画点/取点/越界裁剪");
    ssd1306_clear(&oled);
    ssd1306_draw_pixel(&oled, 0, 0, 1);
    TEST_ASSERT_EQ_UINT(ssd1306_get_pixel(&oled, 0, 0), 1u, "(0,0) 已点亮");
    TEST_ASSERT_EQ_UINT(oled.fb[0], 0x01u, "字节 0 bit0 置位");
    ssd1306_draw_pixel(&oled, 0, 7, 1);
    TEST_ASSERT_EQ_UINT(oled.fb[0], 0x81u, "字节 0 bit7 置位");
    ssd1306_draw_pixel(&oled, 127, 63, 1);
    TEST_ASSERT_EQ_UINT(ssd1306_get_pixel(&oled, 127, 63), 1u, "右下角");
    ssd1306_draw_pixel(&oled, -1, 0, 1);     /* 越界：忽略 */
    ssd1306_draw_pixel(&oled, 0, 64, 1);     /* 越界：忽略 */
    ssd1306_draw_pixel(&oled, 128, 0, 1);    /* 越界：忽略 */
    TEST_ASSERT_EQ_UINT(ssd1306_get_pixel(&oled, -1, 0), 0u, "越界读返回 0");
    ssd1306_draw_pixel(&oled, 10, 10, 1);
    ssd1306_draw_pixel(&oled, 10, 10, 0);
    TEST_ASSERT_EQ_UINT(ssd1306_get_pixel(&oled, 10, 10), 0u, "可以擦除像素");
    TEST_ASSERT_EQ_UINT(oled.pixels_set, 4u, "点亮像素计数（含被擦除的那个）");

    TEST_CASE("画线（Bresenham）/矩形");
    ssd1306_clear(&oled);
    ssd1306_draw_hline(&oled, 0, 20, 10, 1);
    TEST_ASSERT_EQ_UINT(ssd1306_get_pixel(&oled, 0, 20), 1u, "水平线起点");
    TEST_ASSERT_EQ_UINT(ssd1306_get_pixel(&oled, 9, 20), 1u, "水平线终点");
    TEST_ASSERT_EQ_UINT(ssd1306_get_pixel(&oled, 10, 20), 0u, "水平线之外");
    ssd1306_draw_vline(&oled, 5, 0, 8, 1);
    TEST_ASSERT_EQ_UINT(ssd1306_get_pixel(&oled, 5, 7), 1u, "垂直线终点");
    ssd1306_draw_line(&oled, 0, 0, 10, 10, 1);
    TEST_ASSERT_EQ_UINT(ssd1306_get_pixel(&oled, 5, 5), 1u, "对角线中点");
    ssd1306_clear(&oled);
    ssd1306_draw_rect(&oled, 2, 2, 10, 10, 1);
    TEST_ASSERT_EQ_UINT(ssd1306_get_pixel(&oled, 2, 2), 1u, "矩形左上角");
    TEST_ASSERT_EQ_UINT(ssd1306_get_pixel(&oled, 11, 11), 1u, "矩形右下角");
    TEST_ASSERT_EQ_UINT(ssd1306_get_pixel(&oled, 6, 6), 0u, "矩形内部不填充");

    TEST_CASE("字库渲染：'A' / 'H' / 数字 / 越界字符回退到 '?'");
    ssd1306_clear(&oled);
    ssd1306_draw_char(&oled, 0, 0, 'A', 1);
    TEST_ASSERT(oled.pixels_set > 5u, "字符应点亮若干像素");
    {
        /* 'A' 的形状：
         *   ..#..
         *   .#.#.
         *   #...#
         *   #...#
         *   #####
         *   #...#
         *   #...#
         * 顶点在 (2,0)，中间横杠在 y=4
         */
        TEST_ASSERT_EQ_UINT(ssd1306_get_pixel(&oled, 2, 0), 1u, "'A' 顶点应在 (2,0)");
        TEST_ASSERT_EQ_UINT(ssd1306_get_pixel(&oled, 0, 0), 0u, "'A' 左上角应为空");
        TEST_ASSERT_EQ_UINT(ssd1306_get_pixel(&oled, 2, 4), 1u, "'A' 中间横杠在 (2,4)");
        TEST_ASSERT_EQ_UINT(ssd1306_get_pixel(&oled, 1, 4), 1u, "'A' 中间横杠向左延伸");
    }
    ssd1306_clear(&oled);
    ssd1306_draw_string(&oled, 0, 0, "HR 078", 1);
    TEST_ASSERT(oled.pixels_set > 20u, "字符串渲染");
    {
        const uint8_t *g = oled_font_glyph('?');
        const uint8_t *g2 = oled_font_glyph((char)0x01);
        TEST_ASSERT(memcmp(g, g2, OLED_FONT_WIDTH) == 0, "不可打印字符回退到 '?'");
        TEST_ASSERT(oled_font_glyph('A') != NULL, "取字形");
    }

    TEST_CASE("flush：8 页各自设置地址并写 128 字节");
    ssd1306_clear(&oled);
    ssd1306_draw_pixel(&oled, 0, 0, 1);
    {
        uint32_t before = simdev->write_ops;
        oled.flush_chunks = 0u;
        TEST_ASSERT_EQ_INT(ssd1306_flush(&oled), SENSOR_OK, "flush 成功");
        TEST_ASSERT_EQ_UINT(oled.flush_chunks, SSD1306_PAGES, "8 次页搬运");
        TEST_ASSERT_EQ_UINT(simdev->write_ops - before, SSD1306_PAGES * 2u,
                            "每页 2 次事务（设地址 + 写数据）");
    }

    TEST_CASE("对比度/反色/休眠命令");
    TEST_ASSERT_EQ_INT(ssd1306_set_contrast(&oled, 0x55u), SENSOR_OK, "设置对比度");
    TEST_ASSERT_EQ_UINT(oled.contrast, 0x55u, "对比度已保存");
    TEST_ASSERT_EQ_INT(ssd1306_invert(&oled, 1u), SENSOR_OK, "反色");
    TEST_ASSERT_EQ_UINT(oled.inverted, 1u, "反色标志");
    TEST_ASSERT_EQ_INT(ssd1306_sleep(&oled, 1u), SENSOR_OK, "休眠");
    TEST_ASSERT_EQ_INT(ssd1306_display_on(&oled, 1u), SENSOR_OK, "唤醒");

    TEST_CASE("事务参数校验");
    TEST_ASSERT_EQ_INT(ssd1306_send_cmd(NULL, 0xAEu), SENSOR_ERR_PARAM, "空句柄");
    TEST_ASSERT_EQ_INT(ssd1306_send_cmds(&oled, NULL, 1u), SENSOR_ERR_PARAM, "空命令");
    {
        uint8_t big[200];
        memset(big, 0, sizeof(big));
        TEST_ASSERT_EQ_INT(ssd1306_send_cmds(&oled, big, sizeof(big)), SENSOR_ERR_PARAM,
                           "命令流超长应拒绝");
        TEST_ASSERT_EQ_INT(ssd1306_send_data(&oled, big, sizeof(big)), SENSOR_ERR_PARAM,
                           "数据超过一页（128 字节）应拒绝");
    }
}

/* ================================================================== */
/* 6) 电池                                                             */
/* ================================================================== */
static void test_battery(void)
{
    battery_dev_t bat;
    const sensor_bus_t *bus;

    TEST_CASE("VDDA 由 VREFINT 反推（LDO 偏差校准）");
    TEST_ASSERT_EQ_UINT(battery_compute_vdda_mv(1520u), 3259u,
                        "1520 码 -> 1210*4095/1520 = 3259mV");
    TEST_ASSERT_EQ_UINT(battery_compute_vdda_mv(0u), BAT_ADC_VREF_MV, "0 码回退到标称值");
    TEST_ASSERT_EQ_UINT(battery_compute_vdda_mv(1400u), 3539u, "1400 码 -> 3539mV");

    TEST_CASE("电池电压 = 码值 * VDDA / 4095 * 分压比 2");
    {
        /* VREFINT=1520 -> VDDA=3259mV；VBAT 码 2500 -> 2500*3259/4095 = 1989mV, *2 = 3978mV */
        uint16_t mv = battery_compute_vbat_mv(2500u, 1520u);
        TEST_ASSERT_EQ_UINT(mv, 3978u, "电池电压换算");
        TEST_ASSERT_EQ_UINT(battery_compute_vbat_mv(4095u, 1520u), 6518u, "满码");
        TEST_ASSERT_EQ_UINT(battery_compute_vbat_mv(0u, 1520u), 0u, "零码");
    }

    TEST_CASE("OCV 查表 + 线性插值");
    TEST_ASSERT_EQ_UINT(battery_percent_from_mv(4300u), 100u, "高于满电 -> 100%");
    TEST_ASSERT_EQ_UINT(battery_percent_from_mv(4200u), 100u, "4200mV -> 100%");
    TEST_ASSERT_EQ_UINT(battery_percent_from_mv(4000u), 78u, "4000mV -> 78%");
    TEST_ASSERT_EQ_UINT(battery_percent_from_mv(3900u), 63u, "3900mV -> 63%");
    TEST_ASSERT_EQ_UINT(battery_percent_from_mv(3800u), 47u, "3800mV -> 47%");
    TEST_ASSERT_EQ_UINT(battery_percent_from_mv(3300u), 0u, "3300mV -> 0%");
    TEST_ASSERT_EQ_UINT(battery_percent_from_mv(3000u), 0u, "低于截止 -> 0%");
    {
        uint8_t mid = battery_percent_from_mv(3850u);
        TEST_ASSERT_EQ_UINT(mid, 55u, "3850mV -> 55%");
        mid = battery_percent_from_mv(3975u);
        TEST_ASSERT(mid > 71u && mid < 78u, "插值应落在 74 附近");
        printf("     [data] 3975mV 插值电量 = %u%%\n", (unsigned)mid);
    }

    TEST_CASE("剩余容量 -> 百分比");
    TEST_ASSERT_EQ_UINT(battery_percent_from_mah(180.0f, 180.0f), 100u, "满容量");
    TEST_ASSERT_EQ_UINT(battery_percent_from_mah(90.0f, 180.0f), 50u, "半容量");
    TEST_ASSERT_EQ_UINT(battery_percent_from_mah(-5.0f, 180.0f), 0u, "负数归零");
    TEST_ASSERT_EQ_UINT(battery_percent_from_mah(10.0f, 0.0f), 0u, "零容量保护");

    TEST_CASE("采样链路：过采样 + 滤波 + 状态标志");
    sensor_sim_reset();
    bus = sensor_bus_get();
    TEST_ASSERT_EQ_INT(battery_init(&bat, bus), SENSOR_OK, "初始化");
    sensor_sim_set_adc(BAT_ADC_CH_VREFINT, 1520u);
    sensor_sim_set_adc(BAT_ADC_CH_VBAT, 2500u);
    TEST_ASSERT_EQ_INT(battery_sample(&bat, 1000u), SENSOR_OK, "采样");
    TEST_ASSERT_EQ_UINT(bat.data.vrefint_raw, 1520u, "VREFINT 原始码");
    TEST_ASSERT_EQ_UINT(bat.data.adc_vbat_raw, 2500u, "VBAT 原始码");
    TEST_ASSERT_EQ_UINT(bat.data.vdda_mv, 3259u, "VDDA");
    TEST_ASSERT_EQ_UINT(bat.data.vbat_mv, 3978u, "电池电压");
    /* 静置时 c-> OCV(3978mV -> 74%) 与库仑计(先被 OCV 校正到 74%) 一致 */
    TEST_ASSERT_EQ_UINT(bat.data.percent, 74u, "静置电量应为 OCV 查表值 74%");
    printf("     [data] 采样结果: vdda=%umV vbat=%umV percent=%u%%\n",
           (unsigned)bat.data.vdda_mv, (unsigned)bat.data.vbat_mv, (unsigned)bat.data.percent);

    TEST_CASE("库仑计积分：放电与充电");
    {
        float before = bat.remaining_mah;
        TEST_ASSERT_EQ_INT(battery_coulomb_update(&bat, -20, 3600000u / 20u), SENSOR_OK,
                           "放电 20mA 3 分钟");
        TEST_ASSERT(bat.remaining_mah < before, "放电后剩余容量下降");
        printf("     [data] 放电 20mA x 180s: %.3f -> %.3f mAh\n",
               (double)before, (double)bat.remaining_mah);
        before = bat.remaining_mah;
        TEST_ASSERT_EQ_INT(battery_coulomb_update(&bat, 100, 3600000u / 100u), SENSOR_OK,
                           "充电 100mA 36s");
        TEST_ASSERT(bat.remaining_mah > before, "充电后剩余容量上升");
    }

    TEST_CASE("充电/低电/严重低电标志");
    {
        /* 重新初始化以复位滤波器与库仑计：首采样直接用新电压，便于验证阈值逻辑 */
        TEST_ASSERT_EQ_INT(battery_init(&bat, bus), SENSOR_OK, "重新初始化");
        sensor_sim_set_adc(BAT_ADC_CH_VBAT, 500u);   /* 500*3259/4095*2 = 795mV -> 0% */
        (void)battery_sample(&bat, 2000u);
        TEST_ASSERT_EQ_UINT(bat.data.vbat_mv, 794u, "低压换算（500*3259/4095=397, x2=794）");
        TEST_ASSERT_EQ_UINT(bat.data.percent, 0u, "电量 0");
        TEST_ASSERT_EQ_UINT(bat.data.low_battery, 1u, "低电标志");
        TEST_ASSERT_EQ_UINT(bat.data.critical, 1u, "严重低电标志");

        TEST_ASSERT_EQ_INT(battery_init(&bat, bus), SENSOR_OK, "再次重新初始化");
        sensor_sim_set_adc(BAT_ADC_CH_VBAT, 2650u);  /* 2650*3259/4095*2 = 4217mV -> 满电 */
        (void)battery_sample(&bat, 3000u);
        TEST_ASSERT_EQ_UINT(bat.data.percent, 100u, "满电 100%");
        TEST_ASSERT_EQ_UINT(bat.data.low_battery, 0u, "低电标志清除");
        TEST_ASSERT_EQ_UINT(bat.data.critical, 0u, "严重低电标志清除");
        TEST_ASSERT_EQ_UINT(bat.data.charging, 1u, "电压 >=4.15V 判为充电中");
    }

    TEST_CASE("ADC 读失败应上报错误");
    {
        battery_dev_t b2;
        (void)battery_init(&b2, bus);
        /* 用一个没有 adc_read_raw 的总线 */
        {
            sensor_bus_t broken;
            memset(&broken, 0, sizeof(broken));
            broken.i2c_write = bus->i2c_write;
            broken.i2c_read = bus->i2c_read;
            broken.millis = bus->millis;
            (void)battery_init(&b2, &broken);
            TEST_ASSERT_EQ_INT(battery_sample(&b2, 100u), SENSOR_ERR_PARAM, "缺 adc 回调应返回错误");
        }
    }
}

/* ================================================================== */
/* 7) RTC                                                              */
/* ================================================================== */
static void test_rtc(void)
{
    rtc_dev_t rtc;
    rtc_time_t t;
    rtc_time_t back;
    const rtc_iface_t *iface;

    TEST_CASE("BCD 编解码");
    TEST_ASSERT_EQ_UINT(rtc_bin2bcd(59u), 0x59u, "59 -> 0x59");
    TEST_ASSERT_EQ_UINT(rtc_bcd2bin(0x59u), 59u, "0x59 -> 59");
    TEST_ASSERT_EQ_UINT(rtc_bin2bcd(9u), 0x09u, "9 -> 0x09");
    TEST_ASSERT_EQ_UINT(rtc_bcd2bin(0x09u), 9u, "0x09 -> 9");

    TEST_CASE("闰年与月天数");
    TEST_ASSERT_EQ_UINT(rtc_is_leap(2000u), 1u, "2000 是闰年");
    TEST_ASSERT_EQ_UINT(rtc_is_leap(1900u), 0u, "1900 不是闰年");
    TEST_ASSERT_EQ_UINT(rtc_is_leap(2024u), 1u, "2024 是闰年");
    TEST_ASSERT_EQ_UINT(rtc_is_leap(2023u), 0u, "2023 不是闰年");
    TEST_ASSERT_EQ_UINT(rtc_days_in_month(2024u, 2u), 29u, "2024-02 -> 29 天");
    TEST_ASSERT_EQ_UINT(rtc_days_in_month(2023u, 2u), 28u, "2023-02 -> 28 天");
    TEST_ASSERT_EQ_UINT(rtc_days_in_month(2023u, 12u), 31u, "12 月 -> 31 天");
    TEST_ASSERT_EQ_UINT(rtc_days_in_month(2023u, 13u), 0u, "非法月返回 0");

    TEST_CASE("星期推算（Sakamoto）");
    TEST_ASSERT_EQ_UINT(rtc_weekday_from_date(2024u, 1u, 1u), 1u, "2024-01-01 是周一");
    TEST_ASSERT_EQ_UINT(rtc_weekday_from_date(2000u, 1u, 1u), 6u, "2000-01-01 是周六");
    TEST_ASSERT_EQ_UINT(rtc_weekday_from_date(2023u, 6u, 15u), 4u, "2023-06-15 是周四");

    TEST_CASE("日历 <-> Unix 秒换算往返");
    memset(&t, 0, sizeof(t));
    t.year = 24u; t.month = 6u; t.day = 15u;
    t.hour = 13u; t.minute = 45u; t.second = 30u;
    {
        uint32_t unix_s = rtc_time_to_unix(&t);
        TEST_ASSERT_EQ_INT(rtc_unix_to_time(unix_s, &back), 0, "反算成功");
        TEST_ASSERT_EQ_UINT(back.year, 24u, "年");
        TEST_ASSERT_EQ_UINT(back.month, 6u, "月");
        TEST_ASSERT_EQ_UINT(back.day, 15u, "日");
        TEST_ASSERT_EQ_UINT(back.hour, 13u, "时");
        TEST_ASSERT_EQ_UINT(back.minute, 45u, "分");
        TEST_ASSERT_EQ_UINT(back.second, 30u, "秒");
        TEST_ASSERT_EQ_UINT(back.weekday, 6u, "2024-06-15 是周六");
        printf("     [data] 2024-06-15 13:45:30 -> unix=%u (2000 基准)\n", (unsigned)unix_s);
    }
    {
        /* 边界：2000-01-01 00:00:00 -> 0 */
        memset(&t, 0, sizeof(t));
        t.year = 0u; t.month = 1u; t.day = 1u;
        TEST_ASSERT_EQ_UINT(rtc_time_to_unix(&t), 0u, "基准时刻为 0");
    }
    {
        /* 闰日：2024-02-29 */
        memset(&t, 0, sizeof(t));
        t.year = 24u; t.month = 2u; t.day = 29u;
        TEST_ASSERT_EQ_UINT(rtc_time_is_valid(&t), 1u, "2024-02-29 合法");
        t.year = 23u;
        TEST_ASSERT_EQ_UINT(rtc_time_is_valid(&t), 0u, "2023-02-29 非法");
    }
    TEST_CASE("时间合法性校验");
    memset(&t, 0, sizeof(t));
    t.year = 24u; t.month = 12u; t.day = 31u; t.hour = 23u; t.minute = 59u; t.second = 59u;
    TEST_ASSERT_EQ_UINT(rtc_time_is_valid(&t), 1u, "边界值合法");
    t.second = 60u;
    TEST_ASSERT_EQ_UINT(rtc_time_is_valid(&t), 0u, "秒 60 非法");
    t.second = 0u; t.month = 13u;
    TEST_ASSERT_EQ_UINT(rtc_time_is_valid(&t), 0u, "月 13 非法");
    TEST_ASSERT_EQ_UINT(rtc_time_is_valid(NULL), 0u, "空指针");

    TEST_CASE("寄存器级：写保护解锁、预分频、写读时间、唤醒定时器");
    iface = rtc_platform_iface();
    TEST_ASSERT(iface != NULL, "平台 RTC 接口");
    TEST_ASSERT_EQ_INT(rtc_init(&rtc, iface), 0, "RTC 初始化");
    TEST_ASSERT_EQ_UINT(rtc.inited, 1u, "初始化标志");
    {
        uint32_t prer = 0u;
        TEST_ASSERT_EQ_INT(iface->reg_read(RTC_OFF_PRER, &prer), 0, "读 PRER");
        TEST_ASSERT_EQ_UINT((prer >> 16) & 0x7Fu, 127u, "异步分频 127");
        TEST_ASSERT_EQ_UINT(prer & 0x7FFFu, 255u, "同步分频 255 -> 1Hz");
    }
    {
        uint32_t wpr = 0u;
        TEST_ASSERT_EQ_INT(iface->reg_read(RTC_OFF_WPR, &wpr), 0, "读 WPR");
        TEST_ASSERT(wpr != RTC_WPR_KEY2, "写完后必须重新上锁");
    }

    memset(&t, 0, sizeof(t));
    t.year = 24u; t.month = 6u; t.day = 15u; t.weekday = 6u;
    t.hour = 8u; t.minute = 30u; t.second = 0u;
    TEST_ASSERT_EQ_INT(rtc_set_time(&rtc, &t), 0, "设置时间");
    TEST_ASSERT_EQ_UINT(rtc.set_count, 1u, "设置计数");
    memset(&back, 0, sizeof(back));
    TEST_ASSERT_EQ_INT(rtc_get_time(&rtc, &back), 0, "读回时间");
    TEST_ASSERT_EQ_UINT(back.hour, 8u, "小时");
    TEST_ASSERT_EQ_UINT(back.minute, 30u, "分钟");
    TEST_ASSERT_EQ_UINT(back.day, 15u, "日");
    TEST_ASSERT_EQ_UINT(back.month, 6u, "月");
    TEST_ASSERT_EQ_UINT(back.year, 24u, "年");
    TEST_ASSERT_EQ_UINT(back.weekday, 6u, "星期");

    TEST_CASE("写保护生效：未解锁时直接写 TR 应被拒绝（仿真里模拟硬件行为）");
    {
        uint32_t before = 0u;
        uint32_t after = 0u;
        TEST_ASSERT_EQ_INT(iface->reg_read(RTC_OFF_TR, &before), 0, "读 TR");
        TEST_ASSERT_EQ_INT(iface->reg_write(RTC_OFF_TR, 0x00000000u), -1, "未解锁写 TR 应失败");
        TEST_ASSERT_EQ_INT(iface->reg_read(RTC_OFF_TR, &after), 0, "再读 TR");
        TEST_ASSERT_EQ_UINT(after, before, "值未被改动");
    }

    TEST_CASE("唤醒定时器配置与标志清除");
    TEST_ASSERT_EQ_INT(rtc_enable_wakeup(&rtc, 60u), 0, "配置 60 秒唤醒");
    {
        uint32_t wutr = 0u;
        uint32_t cr = 0u;
        TEST_ASSERT_EQ_INT(iface->reg_read(RTC_OFF_WUTR, &wutr), 0, "读 WUTR");
        TEST_ASSERT_EQ_UINT(wutr, 59u, "WUTR = 秒数 - 1");
        TEST_ASSERT_EQ_INT(iface->reg_read(RTC_OFF_CR, &cr), 0, "读 CR");
        TEST_ASSERT_EQ_UINT(cr & RTC_CR_WUTE, RTC_CR_WUTE, "WUTE 使能");
        TEST_ASSERT_EQ_UINT(cr & RTC_CR_WUTIE, RTC_CR_WUTIE, "WUTIE 中断使能");
    }
    TEST_ASSERT_EQ_UINT(rtc.wakeup_count, 1u, "唤醒配置计数");
    TEST_ASSERT_EQ_INT(rtc_disable_wakeup(&rtc), 0, "关闭唤醒");
    {
        uint32_t cr = 0u;
        TEST_ASSERT_EQ_INT(iface->reg_read(RTC_OFF_CR, &cr), 0, "读 CR");
        TEST_ASSERT_EQ_UINT(cr & RTC_CR_WUTE, 0u, "WUTE 已清");
    }

    TEST_CASE("闹钟配置（屏蔽日期 -> 每天定时）");
    memset(&t, 0, sizeof(t));
    t.hour = 7u; t.minute = 0u; t.second = 0u;
    TEST_ASSERT_EQ_INT(rtc_set_alarm(&rtc, &t, 1u), 0, "设置每日闹钟");
    {
        uint32_t alrmar = 0u;
        TEST_ASSERT_EQ_INT(iface->reg_read(RTC_OFF_ALRMAR, &alrmar), 0, "读 ALRMAR");
        TEST_ASSERT_EQ_UINT((alrmar >> 31) & 1u, 1u, "MSK4 屏蔽日期");
        TEST_ASSERT_EQ_UINT((alrmar >> 16) & 0x3Fu, 0x07u, "小时字段 = BCD 07");
        TEST_ASSERT_EQ_UINT((alrmar >> 8) & 0x7Fu, 0x00u, "分钟字段");
    }
    TEST_ASSERT_EQ_UINT(rtc.alarm_count, 1u, "闹钟配置计数");

    TEST_CASE("非法时间拒绝写入");
    memset(&t, 0, sizeof(t));
    t.month = 13u;
    TEST_ASSERT_EQ_INT(rtc_set_time(&rtc, &t), -1, "非法时间应拒绝");
}

/* ================================================================== */
/* 8) IWDG                                                             */
/* ================================================================== */
static void test_iwdg(void)
{
    iwdg_t wd;
    const iwdg_iface_t *iface;

    TEST_CASE("超时计算：T = (4 * 2^PR) * (RLR+1) / f_LSI");
    /* PR=0 -> 4 分频；RLR=7999 -> 4*8000/32000 = 1s */
    TEST_ASSERT_EQ_UINT(iwdg_compute_timeout_ms(0u, 7999u, 32000u), 1000u, "1 秒");
    /* PR=3 -> 32 分频；RLR=999 -> 32*1000/32000 = 1s */
    TEST_ASSERT_EQ_UINT(iwdg_compute_timeout_ms(3u, 999u, 32000u), 1000u, "1 秒（其它组合）");
    /* PR=6 -> 256 分频；RLR=1249 -> 256*1250/32000 = 10s */
    TEST_ASSERT_EQ_UINT(iwdg_compute_timeout_ms(6u, 1249u, 32000u), 10000u, "10 秒");
    TEST_ASSERT_EQ_UINT(iwdg_compute_timeout_ms(8u, 100u, 32000u), 0u, "非法预分频返回 0");

    TEST_CASE("自动选择最接近的超时配置（RLR 不得溢出 12bit）");
    {
        uint8_t pr = 0u;
        uint16_t rlr = 0u;
        uint32_t actual = 0u;
        TEST_ASSERT_EQ_INT(iwdg_pick_config(2000u, 32000u, &pr, &rlr, &actual), 0, "2 秒");
        TEST_ASSERT(actual >= 2000u, "实际超时不得小于请求值");
        TEST_ASSERT(actual <= 2100u, "也不应偏远");
        printf("     [data] 请求 2000ms -> PR=%u RLR=%u 实际=%ums\n",
               (unsigned)pr, (unsigned)rlr, (unsigned)actual);
        TEST_ASSERT_EQ_INT(iwdg_pick_config(500u, 32000u, &pr, &rlr, &actual), 0, "500ms");
        TEST_ASSERT(actual >= 500u && actual <= 600u, "500ms 精度");
        TEST_ASSERT_EQ_INT(iwdg_pick_config(20000u, 32000u, &pr, &rlr, &actual), 0, "20 秒");
        TEST_ASSERT(actual >= 20000u, "20 秒不得小于请求值");
        TEST_ASSERT(rlr <= 4095u, "RLR 必须落在 12bit 内");
    }

    TEST_CASE("初始化写入 PR/RLR 并等待 SR 更新完成");
    iface = iwdg_platform_iface();
    TEST_ASSERT(iface != NULL, "平台 IWDG 接口");
    TEST_ASSERT_EQ_INT(iwdg_init(&wd, iface, 2000u), 0, "初始化");
    TEST_ASSERT(wd.timeout_ms >= 2000u, "超时时间");
    {
        uint32_t pr = 0u;
        uint32_t rlr = 0u;
        TEST_ASSERT_EQ_INT(iface->reg_read(IWDG_OFF_PR, &pr), 0, "读 PR");
        TEST_ASSERT_EQ_INT(iface->reg_read(IWDG_OFF_RLR, &rlr), 0, "读 RLR");
        TEST_ASSERT_EQ_UINT(pr, wd.prescaler, "PR 已写入");
        TEST_ASSERT_EQ_UINT(rlr, wd.reload, "RLR 已写入");
    }
    TEST_ASSERT_EQ_INT(iwdg_start(&wd), 0, "启动看门狗");
    TEST_ASSERT_EQ_UINT(wd.enabled, 1u, "使能标志");
    TEST_ASSERT(wd.feeds >= 1u, "启动时先喂一次");

    TEST_CASE("未解锁时写 PR/RLR 应被拒绝");
    {
        /* 直接写，不经过驱动（驱动内部会先解锁） */
        TEST_ASSERT_EQ_INT(iface->reg_write(IWDG_OFF_PR, 5u), -1, "未解锁写 PR 应失败");
    }

    TEST_CASE("任务级监督：全部心跳正常才喂狗");
    {
        /* 注意：心跳上报与监督喂狗必须用同一个时间基准。
         * 这里以 iface->now_ms() 为基准（真实工程里传调度器的 sched_millis）。 */
        uint32_t base = iface->now_ms();

        TEST_ASSERT_EQ_INT(iwdg_register_task(&wd, "SENSOR", 100u), 0, "注册 SENSOR");
        TEST_ASSERT_EQ_INT(iwdg_register_task(&wd, "UI", 300u), 1, "注册 UI");
        TEST_ASSERT_EQ_INT(iwdg_register_task(&wd, "COMM", 300u), 2, "注册 COMM");
        (void)iwdg_task_alive(&wd, "SENSOR", base);
        (void)iwdg_task_alive(&wd, "UI", base);
        (void)iwdg_task_alive(&wd, "COMM", base);

        {
            uint32_t feeds_before = wd.feeds;
            TEST_ASSERT_EQ_INT(iwdg_supervised_feed(&wd, base), 1, "全部心跳新 -> 放行");
            TEST_ASSERT_EQ_UINT(wd.feeds, feeds_before + 1u, "喂狗一次");
            TEST_ASSERT_EQ_INT(iwdg_supervised_feed(&wd, base + 50u), 1, "50ms 后仍在预算内 -> 放行");
        }

        TEST_CASE("任务超期 -> 拒绝喂狗并记下罪魁任务（让硬件复位）");
        {
            uint32_t blocked_before = wd.blocked_feeds;
            /* SENSOR / COMM 持续心跳，UI 停更：到 base+400 时 UI 已超 300ms 预算 */
            (void)iwdg_task_alive(&wd, "SENSOR", base + 350u);
            (void)iwdg_task_alive(&wd, "COMM", base + 350u);
            TEST_ASSERT_EQ_INT(iwdg_supervised_feed(&wd, base + 400u), 0, "应拒绝喂狗");
            TEST_ASSERT_EQ_UINT(wd.blocked_feeds, blocked_before + 1u, "拒绝计数");
            TEST_ASSERT(wd.last_fault_task >= 0, "记录了罪魁任务");
            printf("     [data] 超期任务 = %s (idx=%d), 累计拒绝喂狗 = %u\n",
                   iwdg_fault_task_name(&wd), (int)wd.last_fault_task,
                   (unsigned)wd.blocked_feeds);
            TEST_ASSERT_EQ_STR(iwdg_fault_task_name(&wd), "UI", "罪魁应是停止心跳的 UI 任务");
        }

        TEST_CASE("复位现场回读：能从备份寄存器取回罪魁任务名");
        {
            uint32_t missed_before = wd.task[1].missed;
            rtc_iwdg_sim_trigger_reset(1u);   /* 模拟 IWDG 复位，罪魁 = idx 1 (UI) */
            TEST_ASSERT_EQ_INT(iwdg_check_reset(&wd), 1, "应检测到 IWDG 复位");
            TEST_ASSERT_EQ_UINT(wd.resets_detected, 1u, "复位计数");
            TEST_ASSERT_EQ_INT((int)wd.last_fault_task, 1, "罪魁任务下标");
            TEST_ASSERT_EQ_STR(iwdg_fault_task_name(&wd), "UI", "罪魁任务名");
            TEST_ASSERT(wd.task[1].missed >= missed_before, "该任务超期计数增加");
            TEST_ASSERT_EQ_INT(iwdg_check_reset(&wd), 0, "标志已清除，第二次不应再报");
        }

        TEST_CASE("参数校验");
        TEST_ASSERT_EQ_INT(iwdg_register_task(&wd, NULL, 100u), -1, "空名字拒绝");
        TEST_ASSERT_EQ_INT(iwdg_init(NULL, iface, 1000u), -1, "空句柄拒绝");
        TEST_ASSERT_EQ_INT(iwdg_init(&wd, NULL, 1000u), -1, "空接口拒绝");
        TEST_ASSERT_EQ_INT(iwdg_task_alive(&wd, "NOT_REGISTERED", 0u), -1, "未注册任务");
    }
}

/* ================================================================== */
/* 9) 调度器                                                           */
/* ================================================================== */
static uint32_t g_task_runs[6];
static os_sched_t *g_sched_ptr;

static void dummy_task(void *arg)
{
    uint8_t idx = (uint8_t)(uintptr_t)arg;
    if (idx < 6u) {
        g_task_runs[idx]++;
    }
}

static void test_scheduler(void)
{
    os_sched_t s;

    TEST_CASE("任务注册与优先级选择：高优先级先跑");
    memset(g_task_runs, 0, sizeof(g_task_runs));
    sched_init(&s, 0u);
    TEST_ASSERT(sched_add_task(&s, "LOW", dummy_task, (void *)(uintptr_t)0u, 1u, 10u, 0u) >= 0, "注册 LOW");
    TEST_ASSERT(sched_add_task(&s, "HIGH", dummy_task, (void *)(uintptr_t)1u, 5u, 10u, 0u) >= 0, "注册 HIGH");
    (void)sched_run_once(&s);
    TEST_ASSERT_EQ_UINT(g_task_runs[1], 1u, "第一次应跑高优先级");
    TEST_ASSERT_EQ_UINT(g_task_runs[0], 0u, "低优先级还没跑");
    (void)sched_run_once(&s);
    TEST_ASSERT_EQ_UINT(g_task_runs[1], 2u, "高优先级持续占用（协作式 run-to-yield）");

    TEST_CASE("同优先级时间片轮转");
    memset(g_task_runs, 0, sizeof(g_task_runs));
    sched_init(&s, 0u);
    (void)sched_add_task(&s, "A", dummy_task, (void *)(uintptr_t)0u, 3u, 2u, 0u);
    (void)sched_add_task(&s, "B", dummy_task, (void *)(uintptr_t)1u, 3u, 2u, 0u);
    {
        int i;
        for (i = 0; i < 8; i++) {
            (void)sched_run_once(&s);
            sched_tick(&s, 1u);   /* 每次消耗 1ms，2ms 时间片 -> 每任务连续跑 2 次 */
        }
    }
    TEST_ASSERT_EQ_UINT(g_task_runs[0], 4u, "A 应跑 4 次");
    TEST_ASSERT_EQ_UINT(g_task_runs[1], 4u, "B 应跑 4 次");
    TEST_ASSERT(s.context_switches >= 2u, "应有上下文切换");
    printf("     [data] 轮转结果 A=%u B=%u switches=%u\n",
           (unsigned)g_task_runs[0], (unsigned)g_task_runs[1], (unsigned)s.context_switches);

    TEST_CASE("周期任务：跑完自动阻塞，到期被 tick 唤醒");
    memset(g_task_runs, 0, sizeof(g_task_runs));
    sched_init(&s, 0u);
    (void)sched_add_task(&s, "P20", dummy_task, (void *)(uintptr_t)2u, 2u, 5u, 20u);
    {
        int i;
        for (i = 0; i < 100; i++) {   /* 推进 100ms */
            (void)sched_run_once(&s);
            sched_tick(&s, 1u);
        }
    }
    /* 20ms 周期 -> 100ms 内跑 5~6 次 */
    TEST_ASSERT(g_task_runs[2] >= 5u && g_task_runs[2] <= 6u, "周期任务次数应符合节拍");
    printf("     [data] 20ms 周期任务在 100ms 内运行 %u 次\n", (unsigned)g_task_runs[2]);

    TEST_CASE("task_sleep_ms 阻塞与唤醒");
    memset(g_task_runs, 0, sizeof(g_task_runs));
    sched_init(&s, 0u);
    (void)sched_add_task(&s, "S", dummy_task, (void *)(uintptr_t)3u, 2u, 5u, 0u);
    {
        os_task_t *t = sched_find_task(&s, "S");
        TEST_ASSERT(t != NULL, "找到任务");
        TEST_ASSERT_EQ_UINT(t->state, (uint16_t)TASK_READY, "初始 READY");
        (void)sched_run_once(&s);
        TEST_ASSERT_EQ_UINT(g_task_runs[3], 1u, "运行一次");
        task_sleep_ms(&s, 50u);   /* 由"当前任务"阻塞（此处即 S） */
        TEST_ASSERT_EQ_UINT(t->state, (uint16_t)TASK_BLOCKED, "已阻塞");
        sched_tick(&s, 10u);
        (void)sched_run_once(&s);
        TEST_ASSERT_EQ_UINT(g_task_runs[3], 1u, "阻塞期间不运行");
        sched_tick(&s, 50u);
        (void)sched_run_once(&s);
        TEST_ASSERT_EQ_UINT(g_task_runs[3], 2u, "到期后恢复运行");
    }

    TEST_CASE("信号量：计数、唤醒等待者、溢出保护");
    {
        os_sem_t sem;
        os_sched_t s2;
        os_task_t *t;
        sched_init(&s2, 0u);
        (void)sched_add_task(&s2, "W", dummy_task, (void *)(uintptr_t)4u, 2u, 5u, 0u);
        os_sem_init(&sem, "touch", 0, 2);

        TEST_ASSERT_EQ_INT(os_sem_count(&sem), 0, "初始计数 0");
        /* 任务上下文里 pend 会阻塞 */
        (void)sched_run_once(&s2);
        TEST_ASSERT_EQ_INT(os_sem_pend(&s2, &sem, 100u), -1, "无资源应返回 -1 并阻塞");
        t = sched_find_task(&s2, "W");
        TEST_ASSERT_EQ_UINT(t->state, (uint16_t)TASK_BLOCKED, "任务被阻塞");
        TEST_ASSERT(t->wait_obj == &sem, "阻塞在信号量上");

        TEST_ASSERT_EQ_INT(os_sem_post(&s2, &sem), 0, "释放信号量");
        TEST_ASSERT_EQ_INT(os_sem_count(&sem), 1, "计数 1");
        TEST_ASSERT_EQ_UINT(t->state, (uint16_t)TASK_READY, "等待者被唤醒");
        TEST_ASSERT_EQ_INT(os_sem_pend(&s2, &sem, 100u), 0, "这次能立刻拿到");
        TEST_ASSERT_EQ_INT(os_sem_count(&sem), 0, "计数回到 0");
        TEST_ASSERT_EQ_INT(os_sem_pend(&s2, &sem, 0u), -1, "超时 0 直接失败");
        TEST_ASSERT_EQ_INT(os_sem_post(&s2, &sem), 0, "再次释放");
        TEST_ASSERT_EQ_INT(os_sem_post(&s2, &sem), 0, "释放到上限");
        TEST_ASSERT_EQ_INT(os_sem_post(&s2, &sem), -1, "超过上限应拒绝");
        TEST_ASSERT_EQ_INT(os_sem_try_pend(&s2, &sem), 0, "非阻塞获取");
        TEST_ASSERT_EQ_INT(os_sem_try_pend(&s2, &sem), 0, "再取一个");
        TEST_ASSERT_EQ_INT(os_sem_try_pend(&s2, &sem), -1, "空了返回 -1");
        TEST_ASSERT_EQ_INT(os_sem_try_pend(&s2, NULL), -1, "空句柄");
        TEST_ASSERT_EQ_UINT(sem.posts, 3u, "post 计数");
    }

    TEST_CASE("邮箱：投递、消费、满/空边界、等待者唤醒");
    {
        os_mbox_t mbox;
        os_sched_t s3;
        os_task_t *t;
        band_msg_probe_t msg;
        band_msg_probe_t out;
        sched_init(&s3, 0u);
        (void)sched_add_task(&s3, "R", dummy_task, (void *)(uintptr_t)5u, 2u, 5u, 0u);
        os_mbox_init(&mbox, "cmd", 2u);
        TEST_ASSERT_EQ_UINT(os_mbox_count(&mbox), 0u, "初始为空");

        memset(&msg, 0, sizeof(msg));
        msg.a = 0xAAu;
        TEST_ASSERT_EQ_INT(os_mbox_post(&s3, &mbox, &msg), 0, "投递 1");
        msg.a = 0xBBu;
        TEST_ASSERT_EQ_INT(os_mbox_post(&s3, &mbox, &msg), 0, "投递 2");
        msg.a = 0xCCu;
        TEST_ASSERT_EQ_INT(os_mbox_post(&s3, &mbox, &msg), -1, "邮箱满应拒绝");
        TEST_ASSERT_EQ_UINT(mbox.overflow, 1u, "溢出计数");
        TEST_ASSERT_EQ_UINT(os_mbox_count(&mbox), 2u, "队列长度 2");

        memset(&out, 0, sizeof(out));
        TEST_ASSERT_EQ_INT(os_mbox_try_pend(&s3, &mbox, &out), 0, "取第一条");
        TEST_ASSERT_EQ_UINT(out.a, 0xAAu, "FIFO 顺序");
        TEST_ASSERT_EQ_INT(os_mbox_try_pend(&s3, &mbox, &out), 0, "取第二条");
        TEST_ASSERT_EQ_UINT(out.a, 0xBBu, "FIFO 顺序");
        TEST_ASSERT_EQ_INT(os_mbox_try_pend(&s3, &mbox, &out), -1, "空了返回 -1");

        /* 阻塞等待 */
        (void)sched_run_once(&s3);
        TEST_ASSERT_EQ_INT(os_mbox_pend(&s3, &mbox, &out, 100u), -1, "无消息应阻塞");
        t = sched_find_task(&s3, "R");
        TEST_ASSERT_EQ_UINT(t->state, (uint16_t)TASK_BLOCKED, "任务阻塞");
        TEST_ASSERT(t->wait_obj == &mbox, "阻塞在邮箱上");
        msg.a = 0xDDu;
        TEST_ASSERT_EQ_INT(os_mbox_post(&s3, &mbox, &msg), 0, "投递唤醒");
        TEST_ASSERT_EQ_UINT(t->state, (uint16_t)TASK_READY, "等待者被唤醒");
        TEST_ASSERT_EQ_INT(os_mbox_try_pend(&s3, &mbox, &out), 0, "取到消息");
        TEST_ASSERT_EQ_UINT(out.a, 0xDDu, "内容正确");
        TEST_ASSERT_EQ_INT(os_mbox_pend(&s3, &mbox, &out, 0u), -1, "超时 0 直接失败");
        TEST_ASSERT_EQ_INT(os_mbox_post(&s3, NULL, &msg), -1, "空邮箱");
        TEST_ASSERT_EQ_INT(os_mbox_post(&s3, &mbox, NULL), -1, "空消息");
    }

    TEST_CASE("挂起/恢复与统计输出");
    {
        os_sched_t s4;
        os_task_t *t;
        sched_init(&s4, 0u);
        (void)sched_add_task(&s4, "X", dummy_task, (void *)(uintptr_t)0u, 2u, 5u, 0u);
        task_suspend(&s4, "X");
        t = sched_find_task(&s4, "X");
        TEST_ASSERT_EQ_UINT(t->state, (uint16_t)TASK_SUSPENDED, "已挂起");
        memset(g_task_runs, 0, sizeof(g_task_runs));
        TEST_ASSERT_EQ_INT(sched_run_once(&s4), 0, "挂起任务不应被调度");
        TEST_ASSERT_EQ_UINT(g_task_runs[0], 0u, "确实没跑");
        task_resume(&s4, "X");
        TEST_ASSERT_EQ_UINT(t->state, (uint16_t)TASK_READY, "已恢复");
        TEST_ASSERT_EQ_INT(sched_run_once(&s4), 1, "恢复后能跑");
        TEST_ASSERT_EQ_UINT(g_task_runs[0], 1u, "运行计数");
        TEST_ASSERT(sched_find_task(&s4, "NOTHERE") == NULL, "查不到返回 NULL");
        TEST_ASSERT_EQ_UINT(sched_millis(&s4), 0u, "虚拟时间");
        sched_dump_stats(&s4);
        (void)g_sched_ptr;
    }

    TEST_CASE("参数校验：任务数上限 / 非法优先级");
    {
        os_sched_t s5;
        int i;
        sched_init(&s5, 0u);
        for (i = 0; i < (int)SCHED_TASK_MAX; i++) {
            TEST_ASSERT(sched_add_task(&s5, "T", dummy_task, NULL, 1u, 5u, 0u) >= 0, "注册成功");
        }
        TEST_ASSERT_EQ_INT(sched_add_task(&s5, "OVER", dummy_task, NULL, 1u, 5u, 0u), -1,
                           "超过上限应拒绝");
        {
            os_sched_t s6;
            sched_init(&s6, 0u);
            TEST_ASSERT_EQ_INT(sched_add_task(&s6, "BADPRIO", dummy_task, NULL,
                                              SCHED_PRIO_LEVELS, 5u, 0u), -1,
                               "非法优先级应拒绝");
            TEST_ASSERT_EQ_INT(sched_add_task(&s6, "NOFN", NULL, NULL, 1u, 5u, 0u), -1,
                               "空函数应拒绝");
        }
    }
}

int test_drivers_run(void)
{
    test_suite_begin("drivers_and_sched");
    test_ring_buffer();
    test_offline_cache();
    test_mpu6050();
    test_mpu6050_fifo_dmp();
    test_mpu6050_attitude();
    test_ft6236();
    test_mlx90615();
    test_ssd1306();
    test_battery();
    test_rtc();
    test_iwdg();
    test_scheduler();
    return test_suite_end();
}
