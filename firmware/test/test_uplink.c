/*
 * test_uplink.c -- 上行链路与补传去重测试（离线，不发真实网络包）
 *
 * 覆盖验收标准第 4 条：断网缓存 N 条 -> 重连后按序号补传且不重复（测试断言）。
 *
 * 用内存回环传输模拟"服务端"，脚本化地：
 *   1) 正常握手 -> 在线发送（立即到达服务端）
 *   2) 服务端断开 -> 后续数据进入离线缓存
 *   3) 重连 -> 自动补传，断言序号严格递增、无重复
 *   4) 服务端逐条回 ACK -> 缓存被清空；重复 ACK 被识别
 *   5) 远程参数下发（TLV）生效，非法参数被拒绝
 */
#include "test_util.h"
#include "uplink.h"
#include "band_port.h"
#include <string.h>
#include <stdio.h>

/* ================================================================== */
/* 回环传输：把客户端发出的字节当作"服务端收到"记录下来                  */
/* ================================================================== */
#define SRV_BUF 32768

typedef struct {
    net_transport_t base;
    uint8_t  rx[SRV_BUF];     /* 客户端 -> 服务端（我们记录并解析） */
    size_t   rx_len;
    uint8_t  tx[SRV_BUF];     /* 服务端 -> 客户端（脚本填入） */
    size_t   tx_len;
    size_t   tx_pos;
    int      open;
    int      drop_after_frames;  /* >0 时收满 N 帧后自动断开 */
    int      frames_seen;
} srv_t;

static srv_t g_srv;

/* 服务端解析到的应用帧序号（用于断言按序、不重复） */
static uint16_t g_rx_seq[256];
static uint16_t g_rx_type[256];
static uint32_t g_rx_count;
static uint32_t g_dup_count;
static uint32_t g_order_violations;

static int srv_connect(struct net_transport_s *self, const char *host, uint16_t port,
                       uint32_t timeout_ms)
{
    (void)host; (void)port; (void)timeout_ms;
    ((srv_t *)self)->open = 1;
    ((srv_t *)self)->tx_pos = 0u;
    return NET_OK;
}

static int srv_send(struct net_transport_s *self, const uint8_t *buf, size_t len)
{
    srv_t *s = (srv_t *)self;

    if (s->open == 0) {
        return NET_ERR_CLOSED;
    }
    if ((s->rx_len + len) > sizeof(s->rx)) {
        return NET_ERR_SOCKET;
    }
    memcpy(&s->rx[s->rx_len], buf, len);
    s->rx_len += len;
    return (int)len;
}

static int srv_recv(struct net_transport_s *self, uint8_t *buf, size_t cap, uint32_t timeout_ms)
{
    srv_t *s = (srv_t *)self;
    size_t avail;
    size_t n;
    (void)timeout_ms;

    if (s->open == 0) {
        return NET_ERR_CLOSED;
    }
    avail = s->tx_len - s->tx_pos;
    if (avail == 0u) {
        return 0;
    }
    n = (avail < cap) ? avail : cap;
    memcpy(buf, &s->tx[s->tx_pos], n);
    s->tx_pos += n;
    return (int)n;
}

static void srv_close(struct net_transport_s *self)
{
    ((srv_t *)self)->open = 0;
}

static int srv_is_open(const struct net_transport_s *self)
{
    return ((const srv_t *)self)->open;
}

static void srv_set_timeout(struct net_transport_s *self, uint32_t timeout_ms)
{
    (void)self; (void)timeout_ms;
}

static void srv_reset(void)
{
    memset(&g_srv, 0, sizeof(g_srv));
    g_srv.base.name = "srv-loop";
    g_srv.base.connect = srv_connect;
    g_srv.base.send = srv_send;
    g_srv.base.recv = srv_recv;
    g_srv.base.close = srv_close;
    g_srv.base.is_open = srv_is_open;
    g_srv.base.set_timeout = srv_set_timeout;
}

static void srv_push(const char *resp)
{
    size_t n = strlen(resp);
    if ((g_srv.tx_len + n) <= sizeof(g_srv.tx)) {
        memcpy(&g_srv.tx[g_srv.tx_len], resp, n);
        g_srv.tx_len += n;
    }
}

static void srv_push_bytes(const uint8_t *d, size_t n)
{
    if ((g_srv.tx_len + n) <= sizeof(g_srv.tx)) {
        memcpy(&g_srv.tx[g_srv.tx_len], d, n);
        g_srv.tx_len += n;
    }
}

/* 用固定 Key 预置 101 响应（uplink 内部用 ws_client，握手机制同前） */
static const uint8_t k_nonce[16] = {
    't', 'h', 'e', ' ', 's', 'a', 'm', 'p', 'l', 'e', ' ', 'n', 'o', 'n', 'c', 'e'
};
static const char *k_accept = "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=";

static void srv_push_101(void)
{
    char resp[512];
    (void)snprintf(resp, sizeof(resp),
                   "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                   "Connection: Upgrade\r\nSec-WebSocket-Accept: %s\r\n\r\n", k_accept);
    srv_push(resp);
}

static void srv_reset_rx_log(void)
{
    g_rx_count = 0u;
    g_dup_count = 0u;
    g_order_violations = 0u;
}

/* 解析服务端收到的所有 WebSocket 帧，抽出其中的应用帧序号。
 * 每次都从缓冲区头部重新扫描，因此可重复调用（幂等）。 */
static void srv_parse_received(void)
{
    size_t off = 0u;
    frame_decoder_t dec;

    srv_reset_rx_log();
    g_srv.frames_seen = 0;
    frame_decoder_init(&dec);

    /* 跳过握手请求（以 \r\n\r\n 结束） */
    {
        size_t i;
        for (i = 0u; (i + 3u) < g_srv.rx_len; i++) {
            if ((g_srv.rx[i] == '\r') && (g_srv.rx[i + 1u] == '\n') &&
                (g_srv.rx[i + 2u] == '\r') && (g_srv.rx[i + 3u] == '\n')) {
                off = i + 4u;
                break;
            }
        }
    }

    while (off < g_srv.rx_len) {
        uint8_t opcode = 0u;
        uint8_t fin = 0u;
        uint8_t masked = 0u;
        uint8_t mask[4];
        const uint8_t *payload = NULL;
        size_t plen = 0u;
        size_t used = 0u;
        int rc;
        size_t i;

        rc = ws_decode_frame_simple(&g_srv.rx[off], g_srv.rx_len - off, &opcode, &fin,
                                   &masked, mask, &payload, &plen, &used);
        if (rc != 0) {
            break;
        }
        off += used;
        if ((opcode != WS_OP_BINARY) && (opcode != WS_OP_TEXT)) {
            continue;   /* 跳过 ping/pong/close */
        }
        g_srv.frames_seen++;
        /* 服务端必须校验客户端帧确实带了掩码（RFC6455 强制要求） */
        if (masked == 0u) {
            printf("     [warn] 收到未掩码的客户端帧！\n");
        }
        for (i = 0u; i < plen; i++) {
            frame_t f;
            frame_dec_result_t r = frame_decoder_push(&dec, payload[i], &f);
            if (r == FRAME_DEC_OK) {
                uint32_t k;
                if (g_rx_count < 256u) {
                    /* 去重检查：同一序号是否已经收过 */
                    for (k = 0u; k < g_rx_count; k++) {
                        if (g_rx_seq[k] == f.seq) {
                            g_dup_count++;
                            break;
                        }
                    }
                    /* 顺序检查：序号必须严格递增 */
                    if ((g_rx_count > 0u) && ((int16_t)(f.seq - g_rx_seq[g_rx_count - 1u]) <= 0)) {
                        g_order_violations++;
                    }
                    g_rx_seq[g_rx_count] = f.seq;
                    g_rx_type[g_rx_count] = f.type;
                    g_rx_count++;
                }
            }
        }
    }
}

/* 构造一条 ACK 的 WebSocket 帧（服务端 -> 客户端，不带掩码）——保留用于扩展 */

/* 构造一条"应用层 ACK 帧"（外面再套 WebSocket 帧） */
static void srv_push_app_ack(uint16_t seq)
{
    uint8_t app[FRAME_MAX_SIZE];
    uint8_t ws[FRAME_MAX_SIZE + 16];
    uint8_t payload[3];
    size_t app_len;
    size_t ws_len;

    payload[0] = ACK_STATUS_OK;
    payload[1] = (uint8_t)(seq & 0xFFu);
    payload[2] = (uint8_t)((seq >> 8) & 0xFFu);
    app_len = frame_build(MSG_ACK, seq, payload, 3u, app, sizeof(app));
    ws_len = ws_encode_frame(WS_OP_BINARY, 1u, app, app_len, NULL, ws, sizeof(ws));
    srv_push_bytes(ws, ws_len);
}

static void srv_push_app_param(void)
{
    uint8_t tlv[64];
    uint8_t app[FRAME_MAX_SIZE];
    uint8_t ws[FRAME_MAX_SIZE + 16];
    size_t off = 0u;
    size_t app_len;
    size_t ws_len;

    off += tlv_put_u16(tlv, sizeof(tlv), off, TLV_TAG_HR_INTERVAL, 800u);
    off += tlv_put_u8(tlv, sizeof(tlv), off, TLV_TAG_STEP_SENSITIVITY, 72u);
    off += tlv_put_u16(tlv, sizeof(tlv), off, TLV_TAG_UPLOAD_INTERVAL, 3000u);
    off += tlv_put_str(tlv, sizeof(tlv), off, TLV_TAG_DEVICE_NAME, "BAND-REmote");
    app_len = frame_build(MSG_PARAM_SET, 900u, tlv, (uint16_t)off, app, sizeof(app));
    ws_len = ws_encode_frame(WS_OP_BINARY, 1u, app, app_len, NULL, ws, sizeof(ws));
    srv_push_bytes(ws, ws_len);
}

/* ================================================================== */
/* 测试                                                                */
/* ================================================================== */
static band_uplink_t g_up;
static uint32_t g_param_cb_count;

static void on_param(void *user, const band_params_t *p)
{
    (void)user; (void)p;
    g_param_cb_count++;
}

static void test_online_send(void)
{
    uint8_t payload[8];

    TEST_CASE("在线：握手后直接发送，序号递增，服务端按序收到");
    srv_reset();
    srv_reset_rx_log();
    uplink_init(&g_up, &g_srv.base, "127.0.0.1", 9001u, "/band");
    g_up.ws.cfg.entropy_set = 1u;
    memcpy(g_up.ws.cfg.entropy, k_nonce, 16u);
    uplink_set_param_callback(&g_up, on_param, NULL);
    g_param_cb_count = 0u;

    srv_push_101();
    TEST_ASSERT_EQ_INT(uplink_connect(&g_up, 2000u), NET_OK, "连接成功");
    TEST_ASSERT_EQ_UINT(g_up.link_up, 1u, "链路 ONLINE");
    TEST_ASSERT_EQ_STR(uplink_state_str(&g_up), "ONLINE", "状态字符串");

    memset(payload, 0, sizeof(payload));
    TEST_ASSERT_EQ_INT(uplink_send(&g_up, MSG_HEART_RATE, payload, 4u, NULL), 0,
                       "在线发送应返回 0（已发出）");
    TEST_ASSERT_EQ_INT(uplink_send(&g_up, MSG_STEP_COUNT, payload, 4u, NULL), 0, "第二帧");
    TEST_ASSERT_EQ_INT(uplink_send(&g_up, MSG_TEMPERATURE, payload, 4u, NULL), 0, "第三帧");
    TEST_ASSERT_EQ_UINT(g_up.st.sent_online, 3u, "在线发送计数");
    TEST_ASSERT_EQ_UINT(g_up.st.cached_offline, 0u, "不应有离线缓存");
    TEST_ASSERT_EQ_UINT(uplink_pending_count(&g_up), 0u, "缓存为空");

    srv_parse_received();
    TEST_ASSERT_EQ_UINT(g_rx_count, 3u, "服务端收到 3 帧");
    TEST_ASSERT_EQ_UINT(g_rx_seq[0], 1u, "第 1 帧序号 1");
    TEST_ASSERT_EQ_UINT(g_rx_seq[1], 2u, "第 2 帧序号 2");
    TEST_ASSERT_EQ_UINT(g_rx_seq[2], 3u, "第 3 帧序号 3");
    TEST_ASSERT_EQ_UINT(g_rx_type[0], MSG_HEART_RATE, "第 1 帧类型");
    TEST_ASSERT_EQ_UINT(g_dup_count, 0u, "无重复序号");
    TEST_ASSERT_EQ_UINT(g_order_violations, 0u, "顺序无违例");
    printf("     [data] 在线直发: TX=%u 帧, 服务端收到 %u 帧, 序号 %u..%u\n",
           (unsigned)g_up.st.tx_frames, (unsigned)g_rx_count,
           (unsigned)g_rx_seq[0], (unsigned)g_rx_seq[g_rx_count - 1u]);
}

static void test_offline_cache_and_resend(void)
{
    uint8_t payload[8];
    int i;
    uint16_t cached_seqs[10];
    uint32_t rx_after_reconnect;

    TEST_CASE("断线：链路不可用时数据自动进入离线缓存");
    memset(payload, 0, sizeof(payload));
    /* 模拟服务端拔线：下一次 recv 返回 CLOSED */
    srv_close(&g_srv.base);
    (void)uplink_poll(&g_up, 1000u);
    TEST_ASSERT_EQ_UINT(g_up.link_up, 0u, "链路已断开");
    TEST_ASSERT_EQ_INT((int)g_up.ws.state, (int)WS_ST_ERROR, "WebSocket 进入 ERROR");

    srv_reset_rx_log();
    for (i = 0; i < 8; i++) {
        int rc = uplink_send(&g_up, MSG_OFFLINE_BATCH, payload, 6u, &cached_seqs[i]);
        TEST_ASSERT_EQ_INT(rc, 1, "离线发送应返回 1（已缓存）");
        TEST_ASSERT(cached_seqs[i] != 0u, "分配了序号");
    }
    TEST_ASSERT_EQ_UINT(g_up.st.cached_offline, 8u, "离线缓存 8 条");
    TEST_ASSERT_EQ_UINT(uplink_pending_count(&g_up), 8u, "待补传 8 条");
    for (i = 1; i < 8; i++) {
        TEST_ASSERT_EQ_UINT(cached_seqs[i], (uint16_t)(cached_seqs[0] + (uint16_t)i),
                            "缓存序号连续递增");
    }
    printf("     [data] 断网期间缓存 %u 条，序号 %u..%u\n",
           (unsigned)uplink_pending_count(&g_up),
           (unsigned)cached_seqs[0], (unsigned)cached_seqs[7]);

    TEST_CASE("重连后自动补传：按序号升序、不重复");
    /* 只重置"服务端"，客户端 uplink 实例保持原样 —— 这样补传的正是断网期间
     * 缓存下来的那 8 条（序号 4..11），与真实场景一致。 */
    srv_reset();
    TEST_ASSERT_EQ_UINT(uplink_pending_count(&g_up), 8u, "8 条待补传仍在缓存中");

    /* 上线：on_state 回调触发 resend_begin，uplink_poll 触发 burst */
    srv_push_101();
    TEST_ASSERT_EQ_INT(uplink_connect(&g_up, 2000u), NET_OK, "重连成功");
    TEST_ASSERT(g_up.offline.session_rounds >= 1u, "补传会话已开启（每次重连开一轮）");

    (void)uplink_poll(&g_up, 2000u);
    (void)uplink_poll(&g_up, 2010u);

    srv_parse_received();
    rx_after_reconnect = g_rx_count;
    TEST_ASSERT_EQ_UINT(rx_after_reconnect, 8u, "服务端应收到 8 条补传");
    TEST_ASSERT_EQ_UINT(g_dup_count, 0u, "补传不得有重复序号");
    TEST_ASSERT_EQ_UINT(g_order_violations, 0u, "补传必须按序号升序");
    for (i = 0; i < 8; i++) {
        TEST_ASSERT_EQ_UINT(g_rx_seq[i], (uint16_t)(cached_seqs[i]), "补传序号应等于缓存时的序号");
        TEST_ASSERT_EQ_UINT(g_rx_type[i], MSG_OFFLINE_BATCH, "补传保留原帧类型");
    }
    TEST_ASSERT_EQ_UINT(g_up.st.resent, 8u, "补传统计");
    printf("     [data] 重连补传: 服务端收到 %u 条, 重复 %u 条, 顺序违例 %u 条, 序号 %u..%u\n",
           (unsigned)rx_after_reconnect, (unsigned)g_dup_count,
           (unsigned)g_order_violations,
           (unsigned)g_rx_seq[0], (unsigned)g_rx_seq[rx_after_reconnect - 1u]);

    TEST_CASE("重复轮询不会重发（同一会话内序号只发一次）");
    (void)uplink_poll(&g_up, 2020u);
    (void)uplink_poll(&g_up, 2030u);
    srv_parse_received();
    TEST_ASSERT_EQ_UINT(g_rx_count, 8u, "仍是 8 条，没有重复发送");
    TEST_ASSERT_EQ_UINT(g_dup_count, 0u, "无重复");

    TEST_CASE("逐条 ACK 后缓存清空，重复 ACK 被识别");
    {
        uint8_t k;
        /* 按"实际被补传的序号"逐条确认（这里就是缓存时分配的 4..11） */
        for (k = 0u; k < 8u; k++) {
            srv_push_app_ack(cached_seqs[k]);
        }
        (void)uplink_poll(&g_up, 2040u);
        TEST_ASSERT_EQ_UINT(g_up.st.acks_rx, 8u, "收到 8 个 ACK");
        TEST_ASSERT_EQ_UINT(uplink_pending_count(&g_up), 0u, "缓存已清空");
        TEST_ASSERT_EQ_UINT(g_up.last_acked_seq, cached_seqs[7], "确认水位线");

        /* 再发一次第 3 条的 ACK：属于重复确认 */
        srv_push_app_ack(cached_seqs[2]);
        (void)uplink_poll(&g_up, 2050u);
        TEST_ASSERT_EQ_UINT(g_up.st.acks_rx, 9u, "第 9 个 ACK 收到");
        TEST_ASSERT_EQ_UINT(g_up.st.dup_acks, 1u, "重复 ACK 被识别");
        TEST_ASSERT_EQ_UINT(uplink_pending_count(&g_up), 0u, "缓存仍为空");
    }
}

static void test_dedup_under_loss(void)
{
    uint8_t payload[8];
    int i;

    TEST_CASE("链路中途再次断开：只有未确认的记录会在新会话重发（不重复投递到应用层）");
    srv_reset();
    srv_reset_rx_log();
    uplink_init(&g_up, &g_srv.base, "127.0.0.1", 9001u, "/band");
    g_up.ws.cfg.entropy_set = 1u;
    memcpy(g_up.ws.cfg.entropy, k_nonce, 16u);
    uplink_set_param_callback(&g_up, on_param, NULL);

    memset(payload, 0, sizeof(payload));
    for (i = 0; i < 6; i++) {
        (void)uplink_send(&g_up, MSG_HEALTH_BATCH, payload, 5u, NULL);
    }
    TEST_ASSERT_EQ_UINT(uplink_pending_count(&g_up), 6u, "6 条待补传");

    srv_push_101();
    TEST_ASSERT_EQ_INT(uplink_connect(&g_up, 2000u), NET_OK, "上线");
    (void)uplink_poll(&g_up, 1000u);
    srv_parse_received();
    TEST_ASSERT_EQ_UINT(g_rx_count, 6u, "第一轮发出 6 条");

    /* 只确认前 3 条，然后断线 */
    srv_push_app_ack(1u);
    srv_push_app_ack(2u);
    srv_push_app_ack(3u);
    (void)uplink_poll(&g_up, 1010u);
    TEST_ASSERT_EQ_UINT(uplink_pending_count(&g_up), 3u, "剩 3 条未确认");

    srv_close(&g_srv.base);
    (void)uplink_poll(&g_up, 1020u);
    TEST_ASSERT_EQ_UINT(g_up.link_up, 0u, "已断线");

    TEST_CASE("重连后只补传未确认的 4/5/6，且不重复");
    srv_reset();
    srv_reset_rx_log();
    srv_push_101();
    TEST_ASSERT_EQ_INT(uplink_connect(&g_up, 2000u), NET_OK, "第二次重连");
    (void)uplink_poll(&g_up, 2000u);
    srv_parse_received();
    TEST_ASSERT_EQ_UINT(g_rx_count, 3u, "只补传 3 条未确认记录");
    TEST_ASSERT_EQ_UINT(g_dup_count, 0u, "无重复序号");
    TEST_ASSERT_EQ_UINT(g_order_violations, 0u, "序号升序");
    if (g_rx_count == 3u) {
        TEST_ASSERT_EQ_UINT(g_rx_seq[0], 4u, "第 1 条是 4");
        TEST_ASSERT_EQ_UINT(g_rx_seq[1], 5u, "第 2 条是 5");
        TEST_ASSERT_EQ_UINT(g_rx_seq[2], 6u, "第 3 条是 6");
    }
    printf("     [data] 部分确认后重连: 补传 %u 条, 序号 %u/%u/%u\n",
           (unsigned)g_rx_count,
           (unsigned)g_rx_seq[0], (unsigned)g_rx_seq[1], (unsigned)g_rx_seq[2]);
}

static void test_remote_params(void)
{
    uint8_t tlv[64];
    size_t off = 0u;
    uint8_t bad_tlv[8];

    TEST_CASE("远程参数下发（TLV）生效并回调上层");
    {
        uint16_t before_version = g_up.params.param_version;
        g_param_cb_count = 0u;
        off = 0u;
        off += tlv_put_u16(tlv, sizeof(tlv), off, TLV_TAG_HR_INTERVAL, 750u);
        off += tlv_put_u8(tlv, sizeof(tlv), off, TLV_TAG_STEP_SENSITIVITY, 65u);
        off += tlv_put_u16(tlv, sizeof(tlv), off, TLV_TAG_UPLOAD_INTERVAL, 2000u);
        off += tlv_put_u16(tlv, sizeof(tlv), off, TLV_TAG_SLEEP_TIMEOUT, 30u);
        off += tlv_put_u8(tlv, sizeof(tlv), off, TLV_TAG_LCD_BRIGHTNESS, 50u);
        off += tlv_put_u8(tlv, sizeof(tlv), off, TLV_TAG_MOTOR_ENABLE, 0u);
        off += tlv_put_str(tlv, sizeof(tlv), off, TLV_TAG_WIFI_SSID, "my-ap");
        off += tlv_put_str(tlv, sizeof(tlv), off, TLV_TAG_WIFI_PASS, "12345678");
        off += tlv_put_str(tlv, sizeof(tlv), off, TLV_TAG_DEVICE_NAME, "BAND-007");

        TEST_ASSERT_EQ_INT(uplink_apply_params(&g_up, tlv, (uint16_t)off), 0, "应用参数");
        TEST_ASSERT_EQ_UINT(g_up.params.hr_interval_ms, 750u, "心率周期");
        TEST_ASSERT_EQ_UINT(g_up.params.step_sensitivity, 65u, "计步灵敏度");
        TEST_ASSERT_EQ_UINT(g_up.params.upload_interval_ms, 2000u, "上传周期");
        TEST_ASSERT_EQ_UINT(g_up.params.sleep_timeout_s, 30u, "休眠超时");
        TEST_ASSERT_EQ_UINT(g_up.params.lcd_brightness, 50u, "背光");
        TEST_ASSERT_EQ_UINT(g_up.params.motor_enable, 0u, "马达开关");
        TEST_ASSERT_EQ_STR(g_up.params.wifi_ssid, "my-ap", "WiFi SSID");
        TEST_ASSERT_EQ_STR(g_up.params.wifi_pass, "12345678", "WiFi 密码");
        TEST_ASSERT_EQ_STR(g_up.params.device_name, "BAND-007", "设备名");
        TEST_ASSERT_EQ_UINT(g_up.params.param_version, before_version + 1u, "参数版本递增");
        TEST_ASSERT_EQ_UINT(g_param_cb_count, 1u, "参数变化回调被调用");
        TEST_ASSERT_EQ_UINT(g_up.st.params_applied, 1u, "参数应用计数");
    }

    TEST_CASE("越界参数被拒绝但整包仍算应用（保持可用性）");
    {
        off = 0u;
        off += tlv_put_u16(tlv, sizeof(tlv), off, TLV_TAG_HR_INTERVAL, 5u);      /* 太小 */
        off += tlv_put_u8(tlv, sizeof(tlv), off, TLV_TAG_STEP_SENSITIVITY, 200u); /* >100 */
        off += tlv_put_u16(tlv, sizeof(tlv), off, TLV_TAG_UPLOAD_INTERVAL, 300u); /* 太小 */
        {
            uint16_t hr_before = g_up.params.hr_interval_ms;
            uint8_t se_before = g_up.params.step_sensitivity;
            TEST_ASSERT_EQ_INT(uplink_apply_params(&g_up, tlv, (uint16_t)off), 0, "整包被接受");
            TEST_ASSERT_EQ_UINT(g_up.params.hr_interval_ms, hr_before, "非法心率周期被忽略");
            TEST_ASSERT_EQ_UINT(g_up.params.step_sensitivity, se_before, "非法灵敏度被忽略");
            TEST_ASSERT(g_up.st.param_errors >= 2u, "记录了参数错误");
        }
    }

    TEST_CASE("TLV 格式非法 -> 整包拒绝（不做部分应用）");
    {
        uint16_t hr_before = g_up.params.hr_interval_ms;
        bad_tlv[0] = TLV_TAG_HR_INTERVAL;
        bad_tlv[1] = 0x20u;   /* 声称 32 字节，实际只有 2 字节 */
        bad_tlv[2] = 0x10u;
        bad_tlv[3] = 0x27u;
        TEST_ASSERT_EQ_INT(uplink_apply_params(&g_up, bad_tlv, 4u), -1, "格式非法应拒绝");
        TEST_ASSERT_EQ_UINT(g_up.params.hr_interval_ms, hr_before, "参数未被改动");
        TEST_ASSERT_EQ_INT(uplink_apply_params(&g_up, NULL, 4u), -1, "空指针拒绝");
        TEST_ASSERT_EQ_INT(uplink_apply_params(&g_up, bad_tlv, 0u), -1, "空载荷拒绝");
    }

    TEST_CASE("参数编码回读（用于上报给 App 确认）");
    {
        uint8_t out[128];
        size_t n = uplink_encode_params(&g_up.params, out, sizeof(out));
        tlv_list_t list;
        uint16_t v16 = 0u;
        TEST_ASSERT(n > 0u, "编码成功");
        TEST_ASSERT_EQ_INT(tlv_parse(out, (uint16_t)n, &list), 7, "7 个 TLV 条目");
        TEST_ASSERT_EQ_INT(tlv_get_u16(&list, TLV_TAG_HR_INTERVAL, &v16), 0, "取心率周期");
        TEST_ASSERT_EQ_UINT(v16, g_up.params.hr_interval_ms, "回读值与结构体一致");
    }

    TEST_CASE("通过 WebSocket 收到下行参数帧，链路层自动应用");
    {
        uint16_t before = g_up.params.hr_interval_ms;
        uint32_t rx_before = g_up.st.rx_frames;
        srv_push_app_param();
        (void)uplink_poll(&g_up, 3000u);
        TEST_ASSERT_EQ_UINT(g_up.params.hr_interval_ms, 800u, "心率周期被远程改写");
        TEST_ASSERT(g_up.params.hr_interval_ms != before, "确实发生了变化");
        TEST_ASSERT_EQ_UINT(g_up.params.step_sensitivity, 72u, "灵敏度被改写");
        TEST_ASSERT_EQ_STR(g_up.params.device_name, "BAND-REmote", "设备名被改写");
        TEST_ASSERT_EQ_UINT(g_up.st.rx_frames, rx_before + 1u, "下行帧计数 +1");
    }
}

static void test_reset_behavior(void)
{
    uint8_t payload[4];

    TEST_CASE("发送失败（服务端拒收）时数据仍被缓存，不丢");
    srv_reset();
    srv_reset_rx_log();
    uplink_init(&g_up, &g_srv.base, "127.0.0.1", 9001u, "/band");
    g_up.ws.cfg.entropy_set = 1u;
    memcpy(g_up.ws.cfg.entropy, k_nonce, 16u);
    memset(payload, 0, sizeof(payload));
    (void)uplink_send(&g_up, MSG_LOG, payload, 2u, NULL);
    TEST_ASSERT_EQ_UINT(uplink_pending_count(&g_up), 1u, "离线时入缓存");

    /* 未连接时直接补传 burst 不应崩溃 */
    TEST_ASSERT_EQ_UINT(uplink_resend_burst(&g_up, 4u), 0u, "未连接时补传返回 0");
    TEST_ASSERT_EQ_UINT(uplink_pending_count(&g_up), 1u, "记录仍在缓存中");

    TEST_CASE("uplink_send_raw_frame 未连接返回错误码");
    {
        uint8_t raw[FRAME_MAX_SIZE];
        size_t n = frame_build(MSG_PING, 1u, NULL, 0u, raw, sizeof(raw));
        TEST_ASSERT_EQ_INT(uplink_send_raw_frame(&g_up, raw, n), -2, "未连接返回 -2");
        TEST_ASSERT_EQ_INT(uplink_send_raw_frame(NULL, raw, n), -1, "空句柄返回 -1");
    }

    TEST_CASE("统计重置与断开");
    uplink_reset_stats(&g_up);
    TEST_ASSERT_EQ_UINT(g_up.st.tx_frames, 0u, "统计清零");
    uplink_disconnect(&g_up);
    TEST_ASSERT_EQ_UINT(g_up.link_up, 0u, "已断开");
    TEST_ASSERT_EQ_STR(uplink_state_str(&g_up), "CLOSED", "状态 CLOSED");

    TEST_CASE("默认参数合理性");
    {
        band_params_t p;
        uplink_default_params(&p);
        TEST_ASSERT(p.hr_interval_ms >= 100u, "心率周期有下界");
        TEST_ASSERT(p.upload_interval_ms >= 500u, "上传周期有下界");
        TEST_ASSERT(p.sleep_timeout_s > 0u, "休眠超时必须为正");
        TEST_ASSERT_EQ_UINT(p.motor_enable, 1u, "马达默认开启");
        TEST_ASSERT(p.device_name[0] != '\0', "设备名非空");
    }

    TEST_CASE("空指针安全性");
    uplink_reset_stats(NULL);
    uplink_disconnect(NULL);
    uplink_poll(NULL, 0u);
    TEST_ASSERT_EQ_UINT(uplink_pending_count(NULL), 0u, "空句柄返回 0");
    TEST_ASSERT_EQ_STR(uplink_state_str(NULL), "?", "空句柄状态");
    TEST_ASSERT_EQ_INT(uplink_send(NULL, MSG_PING, NULL, 0u, NULL), -1, "空句柄发送失败");
}

int test_uplink_run(void)
{
    test_suite_begin("uplink_and_resend");
    test_online_send();
    test_offline_cache_and_resend();
    test_dedup_under_loss();
    test_remote_params();
    test_reset_behavior();
    return test_suite_end();
}
