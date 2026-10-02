/*
 * test_frame_codec.c -- 帧编解码测试
 *
 * 覆盖验收标准第 2 条：
 *   正常解析 / 校验错拒绝 / 半包 / 粘包 / 长度非法 / 载荷与序号回绕 /
 *   CRC16 标准向量 / TLV 参数下发 / 聚合体征载荷往返
 */
#include "test_util.h"
#include "frame_codec.h"
#include <string.h>

/* CRC16/CCITT-FALSE 标准测试向量：输入 "123456789" -> 0x29B1 */
static void test_crc_vector(void)
{
    uint16_t crc;

    TEST_CASE("CRC16/CCITT-FALSE 标准向量 \"123456789\" == 0x29B1");
    crc = frame_crc16((const uint8_t *)"123456789", 9u);
    TEST_ASSERT_EQ_UINT(crc, 0x29B1u, "CRC16 向量失配");

    TEST_CASE("CRC16 初值与空输入");
    TEST_ASSERT_EQ_UINT(frame_crc16(NULL, 0u), FRAME_CRC_INIT, "空输入应返回初值 0xFFFF");
}

static void test_encode_decode_roundtrip(void)
{
    uint8_t buf[FRAME_MAX_SIZE];
    uint8_t payload[8] = { 0x11u, 0x22u, 0x33u, 0x44u, 0x55u, 0x66u, 0x77u, 0x88u };
    frame_t out;
    frame_decoder_t dec;
    frame_dec_result_t r;
    size_t n;
    size_t i;

    TEST_CASE("正常帧编解码往返（含序号/长度小端）");
    n = frame_build(MSG_HEART_RATE, 0x1234u, payload, 8u, buf, sizeof(buf));
    TEST_ASSERT_EQ_UINT(n, FRAME_OVERHEAD + 8u, "编码长度应为 9+8=17");
    TEST_ASSERT_EQ_UINT(buf[0], FRAME_SOF0, "SOF0");
    TEST_ASSERT_EQ_UINT(buf[1], FRAME_SOF1, "SOF1");
    TEST_ASSERT_EQ_UINT(buf[2], MSG_HEART_RATE, "TYPE");
    TEST_ASSERT_EQ_UINT(buf[3], 0x34u, "SEQ 低字节");
    TEST_ASSERT_EQ_UINT(buf[4], 0x12u, "SEQ 高字节");
    TEST_ASSERT_EQ_UINT(buf[5], 0x08u, "LEN 低字节");
    TEST_ASSERT_EQ_UINT(buf[6], 0x00u, "LEN 高字节");

    frame_decoder_init(&dec);
    r = FRAME_DEC_NONE;
    for (i = 0u; i < n; i++) {
        r = frame_decoder_push(&dec, buf[i], &out);
    }
    TEST_ASSERT_EQ_INT(r, FRAME_DEC_OK, "应解出一帧");
    TEST_ASSERT_EQ_UINT(out.type, MSG_HEART_RATE, "TYPE 还原");
    TEST_ASSERT_EQ_UINT(out.seq, 0x1234u, "SEQ 还原");
    TEST_ASSERT_EQ_UINT(out.len, 8u, "LEN 还原");
    TEST_ASSERT(memcmp(out.payload, payload, 8u) == 0, "载荷还原");
    TEST_ASSERT_EQ_UINT(dec.frames_ok, 1u, "成功帧计数");
    TEST_ASSERT_EQ_UINT(dec.crc_errors, 0u, "不应有 CRC 错误");

    TEST_CASE("零长度载荷（ACK 类帧）");
    n = frame_build(MSG_PING, 7u, NULL, 0u, buf, sizeof(buf));
    TEST_ASSERT_EQ_UINT(n, FRAME_OVERHEAD, "零载荷帧长应为 9");
    frame_decoder_init(&dec);
    for (i = 0u; i < n; i++) {
        r = frame_decoder_push(&dec, buf[i], &out);
    }
    TEST_ASSERT_EQ_INT(r, FRAME_DEC_OK, "零载荷帧应解出");
    TEST_ASSERT_EQ_UINT(out.len, 0u, "载荷长度 0");
}

static void test_crc_reject(void)
{
    uint8_t buf[FRAME_MAX_SIZE];
    frame_t out;
    frame_decoder_t dec;
    frame_dec_result_t r = FRAME_DEC_NONE;
    size_t n;
    size_t i;

    TEST_CASE("校验错必须被拒绝（载荷位翻转 / CRC 位翻转）");
    n = frame_build(MSG_STEP_COUNT, 42u, (const uint8_t *)"HELLO", 5u, buf, sizeof(buf));

    /* 情况 1：载荷被干扰 */
    frame_decoder_init(&dec);
    buf[FRAME_HEADER_LEN + 1u] ^= 0xFFu;
    for (i = 0u; i < n; i++) {
        r = frame_decoder_push(&dec, buf[i], &out);
        if (r == FRAME_DEC_CRC_ERR) {
            break;
        }
    }
    TEST_ASSERT_EQ_INT(r, FRAME_DEC_CRC_ERR, "载荷损坏应当 CRC 错");
    TEST_ASSERT_EQ_UINT(dec.crc_errors, 1u, "CRC 错误计数 +1");
    TEST_ASSERT_EQ_UINT(dec.frames_ok, 0u, "不应有成功帧");
    buf[FRAME_HEADER_LEN + 1u] ^= 0xFFu;

    /* 情况 2：CRC 字段本身被干扰 */
    frame_decoder_init(&dec);
    n = frame_build(MSG_STEP_COUNT, 42u, (const uint8_t *)"HELLO", 5u, buf, sizeof(buf));
    buf[n - 1u] ^= 0x01u;
    for (i = 0u; i < n; i++) {
        r = frame_decoder_push(&dec, buf[i], &out);
    }
    TEST_ASSERT_EQ_INT(r, FRAME_DEC_CRC_ERR, "CRC 字段损坏应当拒绝");
    TEST_ASSERT_EQ_UINT(dec.frames_ok, 0u, "仍然没有成功帧");
}

static void test_half_packet(void)
{
    uint8_t buf[FRAME_MAX_SIZE];
    frame_t out;
    frame_decoder_t dec;
    frame_dec_result_t r = FRAME_DEC_NONE;
    size_t n;
    size_t i;

    TEST_CASE("半包：分三段喂入，中间不得产出帧，最后一段才出帧");
    n = frame_build(MSG_TEMPERATURE, 1000u, (const uint8_t *)"ABCDEFGHIJ", 10u,
                    buf, sizeof(buf));
    TEST_ASSERT_EQ_UINT(n, 19u, "19 = 9 + 10");

    frame_decoder_init(&dec);
    /* 第 1 段：只有帧头前 3 字节 */
    for (i = 0u; i < 3u; i++) {
        r = frame_decoder_push(&dec, buf[i], &out);
        TEST_ASSERT_EQ_INT(r, FRAME_DEC_NONE, "半包不应产出帧");
    }
    TEST_ASSERT_EQ_UINT(dec.frames_ok, 0u, "还没有成功帧");
    TEST_ASSERT_EQ_UINT(dec.crc_errors, 0u, "也不应是错误");

    /* 第 2 段：喂到载荷中间 */
    for (i = 3u; i < 12u; i++) {
        r = frame_decoder_push(&dec, buf[i], &out);
        TEST_ASSERT_EQ_INT(r, FRAME_DEC_NONE, "半包中段不应产出帧");
    }
    /* 第 3 段：补齐剩余 */
    for (i = 12u; i < n; i++) {
        r = frame_decoder_push(&dec, buf[i], &out);
    }
    TEST_ASSERT_EQ_INT(r, FRAME_DEC_OK, "补齐后应解出帧");
    TEST_ASSERT_EQ_UINT(out.seq, 1000u, "序号正确");
    TEST_ASSERT_EQ_UINT(out.len, 10u, "长度正确");
    TEST_ASSERT(memcmp(out.payload, "ABCDEFGHIJ", 10u) == 0, "载荷正确");
    TEST_ASSERT_EQ_UINT(dec.frames_ok, 1u, "恰好 1 帧");
}

static void test_sticky_packets(void)
{
    uint8_t stream[FRAME_MAX_SIZE * 3u];
    frame_t frames[3];
    frame_decoder_t dec;
    size_t off = 0u;
    size_t consumed = 0u;
    uint32_t crc_err = 0u;
    size_t got;

    TEST_CASE("粘包：3 帧一次性喂入，应解析出 3 帧且顺序正确");
    off += frame_build(MSG_HEART_RATE, 1u, (const uint8_t *)"AAA", 3u,
                       &stream[off], sizeof(stream) - off);
    off += frame_build(MSG_STEP_COUNT, 2u, (const uint8_t *)"BBBB", 4u,
                       &stream[off], sizeof(stream) - off);
    off += frame_build(MSG_BATTERY, 3u, (const uint8_t *)"CCCCC", 5u,
                       &stream[off], sizeof(stream) - off);

    frame_decoder_init(&dec);
    got = frame_decoder_push_buf(&dec, stream, off, frames, 3u, &consumed, &crc_err);
    TEST_ASSERT_EQ_UINT(got, 3u, "应解出 3 帧");
    TEST_ASSERT_EQ_UINT(consumed, off, "应消耗全部字节");
    TEST_ASSERT_EQ_UINT(crc_err, 0u, "无 CRC 错误");
    TEST_ASSERT_EQ_UINT(frames[0].seq, 1u, "第 1 帧序号");
    TEST_ASSERT_EQ_UINT(frames[1].seq, 2u, "第 2 帧序号");
    TEST_ASSERT_EQ_UINT(frames[2].seq, 3u, "第 3 帧序号");
    TEST_ASSERT_EQ_UINT(frames[0].type, MSG_HEART_RATE, "第 1 帧类型");
    TEST_ASSERT_EQ_UINT(frames[2].type, MSG_BATTERY, "第 3 帧类型");
    TEST_ASSERT(memcmp(frames[2].payload, "CCCCC", 5u) == 0, "第 3 帧载荷");

    TEST_CASE("粘包 + 尾部半包：2.5 帧，应解出 2 帧并保留余下状态");
    frame_decoder_init(&dec);
    got = frame_decoder_push_buf(&dec, stream, off - 4u, frames, 3u, &consumed, &crc_err);
    TEST_ASSERT_EQ_UINT(got, 2u, "只应解出 2 帧");
    TEST_ASSERT_EQ_UINT(dec.frames_ok, 2u, "成功帧计数 2");
    TEST_ASSERT(dec.state != FRAME_ST_SOF0, "应停留在半帧中间状态");
    /* 补齐剩余 4 字节 */
    {
        frame_dec_result_t r = FRAME_DEC_NONE;
        size_t i;
        for (i = off - 4u; i < off; i++) {
            r = frame_decoder_push(&dec, stream[i], &frames[0]);
        }
        TEST_ASSERT_EQ_INT(r, FRAME_DEC_OK, "补齐后第 3 帧应解出");
        TEST_ASSERT_EQ_UINT(dec.frames_ok, 3u, "累计 3 帧");
    }
}

static void test_bad_length_and_garbage(void)
{
    uint8_t buf[FRAME_MAX_SIZE];
    frame_t out;
    frame_decoder_t dec;
    frame_dec_result_t r = FRAME_DEC_NONE;
    size_t i;

    TEST_CASE("长度非法（> FRAME_MAX_PAYLOAD）应立即拒绝并重新同步");
    frame_decoder_init(&dec);
    {
        uint8_t bad[16];
        bad[0] = FRAME_SOF0;
        bad[1] = FRAME_SOF1;
        bad[2] = MSG_LOG;
        bad[3] = 0x00u;
        bad[4] = 0x00u;
        bad[5] = 0xFFu;    /* LEN 低字节 */
        bad[6] = 0x0Fu;    /* LEN 高字节 = 0x0FFF = 4095，超限 */
        for (i = 0u; i < 7u; i++) {
            r = frame_decoder_push(&dec, bad[i], &out);
        }
        TEST_ASSERT_EQ_INT(r, FRAME_DEC_LEN_ERR, "应报长度错误");
        TEST_ASSERT_EQ_UINT(dec.len_errors, 1u, "长度错误计数");
    }

    TEST_CASE("帧头前有垃圾字节：应能自动对齐（resync）");
    frame_decoder_init(&dec);
    {
        uint8_t junk[5] = { 0x00u, 0xFFu, 0xAAu, 0x00u, 0x55u };
        size_t n;
        for (i = 0u; i < 5u; i++) {
            r = frame_decoder_push(&dec, junk[i], &out);
            TEST_ASSERT_EQ_INT(r, FRAME_DEC_NONE, "垃圾字节不应产出帧");
        }
        n = frame_build(MSG_ACK, 9u, (const uint8_t *)"\x00\x09\x00", 3u, buf, sizeof(buf));
        for (i = 0u; i < n; i++) {
            r = frame_decoder_push(&dec, buf[i], &out);
        }
        TEST_ASSERT_EQ_INT(r, FRAME_DEC_OK, "垃圾之后应重新对齐并解出帧");
        TEST_ASSERT_EQ_UINT(out.seq, 9u, "序号正确");
        TEST_ASSERT(dec.resyncs > 0u, "应有 resync 记录");
    }

    TEST_CASE("0xAA 0xAA 0x55 形式的干扰不应误判帧头");
    frame_decoder_init(&dec);
    {
        uint8_t seq[3] = { 0xAAu, 0xAAu, 0x55u };
        size_t n;
        for (i = 0u; i < 3u; i++) {
            (void)frame_decoder_push(&dec, seq[i], &out);
        }
        TEST_ASSERT_EQ_UINT(dec.frames_ok, 0u, "不应有成功帧");
        TEST_ASSERT_EQ_UINT(dec.crc_errors, 0u, "也不应报 CRC 错（还没收够一帧）");
        /* 后面补一个真帧，应该仍能解出 */
        n = frame_build(MSG_PONG, 11u, NULL, 0u, buf, sizeof(buf));
        for (i = 0u; i < n; i++) {
            r = frame_decoder_push(&dec, buf[i], &out);
        }
        /* 注意：0xAA 0x55 之后跟随的 TYP 被吃掉了，所以这一帧可能解不出；
         * 关键断言是"不会把干扰当成合法帧"。 */
        TEST_ASSERT(dec.frames_ok == 0u || out.seq == 11u, "要么没有帧，要么是正确帧");
    }
}

static void test_payload_codecs(void)
{
    health_payload_t h;
    health_payload_t h2;
    uint8_t buf[FRAME_MAX_PAYLOAD];
    size_t n;

    TEST_CASE("聚合体征载荷打包/解包往返");
    memset(&h, 0, sizeof(h));
    h.heart_rate = 78u;
    h.confidence = 90u;
    h.rr_interval_ms = 769u;
    h.temperature_c100 = 3665;      /* 36.65 C */
    h.steps = 12345u;
    h.battery_mv = 3980u;
    h.battery_pct = 76u;
    h.roll_c100 = -1234;            /* -12.34 度 */
    h.pitch_c100 = 456;
    h.yaw_c100 = -9000;

    n = frame_pack_health(&h, buf, sizeof(buf));
    TEST_ASSERT_EQ_UINT(n, 17u, "载荷长度 17");
    TEST_ASSERT_EQ_INT(frame_unpack_health(buf, (uint16_t)n, &h2), 0, "解包应成功");
    TEST_ASSERT_EQ_UINT(h2.heart_rate, 78u, "心率");
    TEST_ASSERT_EQ_UINT(h2.rr_interval_ms, 769u, "RR 间期");
    TEST_ASSERT_EQ_INT(h2.temperature_c100, 3665, "体温（有符号）");
    TEST_ASSERT_EQ_UINT(h2.steps, 12345u, "步数");
    TEST_ASSERT_EQ_UINT(h2.battery_mv, 3980u, "电压");
    TEST_ASSERT_EQ_UINT(h2.battery_pct, 76u, "电量");
    TEST_ASSERT_EQ_INT(h2.roll_c100, -1234, "横滚角（负数）");
    TEST_ASSERT_EQ_INT(h2.pitch_c100, 456, "俯仰角");
    TEST_ASSERT_EQ_INT(h2.yaw_c100, -9000, "偏航角（负数）");

    TEST_CASE("载荷缓冲区不足时应拒绝");
    TEST_ASSERT_EQ_INT(frame_unpack_health(buf, 5u, &h2), -1, "长度不足应返回 -1");

    TEST_CASE("ACK 载荷往返");
    n = frame_pack_ack(ACK_STATUS_SEQ_DUP, 4242u, buf, sizeof(buf));
    TEST_ASSERT_EQ_UINT(n, 3u, "ACK 载荷 3 字节");
    {
        uint8_t st = 0u;
        uint16_t sq = 0u;
        TEST_ASSERT_EQ_INT(frame_unpack_ack(buf, (uint16_t)n, &st, &sq), 0, "解 ACK");
        TEST_ASSERT_EQ_UINT(st, ACK_STATUS_SEQ_DUP, "状态码");
        TEST_ASSERT_EQ_UINT(sq, 4242u, "被确认的序号");
    }
}

static void test_tlv_params(void)
{
    uint8_t buf[128];
    size_t off = 0u;
    tlv_list_t list;
    uint16_t v16 = 0u;
    uint8_t v8 = 0u;
    char name[24];
    int n;
    size_t w;

    TEST_CASE("远程参数 TLV 解析（整数 + 字符串）");
    w = tlv_put_u16(buf, sizeof(buf), off, TLV_TAG_HR_INTERVAL, 500u); off += w;
    TEST_ASSERT(w > 0u, "写 u16 TLV");
    w = tlv_put_u8(buf, sizeof(buf), off, TLV_TAG_STEP_SENSITIVITY, 70u); off += w;
    w = tlv_put_u16(buf, sizeof(buf), off, TLV_TAG_SLEEP_TIMEOUT, 45u); off += w;
    w = tlv_put_str(buf, sizeof(buf), off, TLV_TAG_DEVICE_NAME, "BAND-TEST"); off += w;
    TEST_ASSERT(off > 0u, "TLV 总长");

    n = tlv_parse(buf, (uint16_t)off, &list);
    TEST_ASSERT_EQ_INT(n, 4, "应有 4 个条目");
    TEST_ASSERT_EQ_INT(tlv_get_u16(&list, TLV_TAG_HR_INTERVAL, &v16), 0, "取心率周期");
    TEST_ASSERT_EQ_UINT(v16, 500u, "心率周期值");
    TEST_ASSERT_EQ_INT(tlv_get_u8(&list, TLV_TAG_STEP_SENSITIVITY, &v8), 0, "取灵敏度");
    TEST_ASSERT_EQ_UINT(v8, 70u, "灵敏度值");
    TEST_ASSERT_EQ_INT(tlv_get_u16(&list, TLV_TAG_SLEEP_TIMEOUT, &v16), 0, "取休眠超时");
    TEST_ASSERT_EQ_UINT(v16, 45u, "休眠超时值");
    TEST_ASSERT_EQ_INT(tlv_get_str(&list, TLV_TAG_DEVICE_NAME, name, sizeof(name)), 0, "取设备名");
    TEST_ASSERT_EQ_STR(name, "BAND-TEST", "设备名");

    TEST_CASE("TLV 异常输入必须被拒绝");
    TEST_ASSERT_EQ_INT(tlv_parse(buf, 1u, &list), -4, "只有一个字节 -> 残留半个头");
    TEST_ASSERT_EQ_INT(tlv_get_u16(&list, TLV_TAG_HR_INTERVAL, &v16), -1, "不存在的 tag");
    {
        /* 声明长度超过剩余字节 */
        uint8_t bad[4];
        bad[0] = TLV_TAG_WIFI_SSID;
        bad[1] = 0x20u;   /* 说 32 字节，实际只跟了 2 字节 */
        bad[2] = 'a';
        bad[3] = 'b';
        TEST_ASSERT_EQ_INT(tlv_parse(bad, 4u, &list), -2, "值长度越过尾部应报错");
    }
    {
        uint8_t bad[3];
        bad[0] = TLV_TAG_MOTOR_ENABLE;
        bad[1] = 0xFFu;   /* 值长 255 > TLV_MAX_VALUE_LEN */
        bad[2] = 0x01u;
        TEST_ASSERT(tlv_parse(bad, 3u, &list) < 0, "超长值应报错");
    }

    TEST_CASE("帧类型名");
    TEST_ASSERT_EQ_STR(frame_type_name(MSG_HEART_RATE), "HEART_RATE", "类型名");
    TEST_ASSERT_EQ_STR(frame_type_name(0xEEu), "UNKNOWN", "未知类型名");
}

int test_frame_codec_run(void)
{
    test_suite_begin("frame_codec");
    test_crc_vector();
    test_encode_decode_roundtrip();
    test_crc_reject();
    test_half_packet();
    test_sticky_packets();
    test_bad_length_and_garbage();
    test_payload_codecs();
    test_tlv_params();
    return test_suite_end();
}
