/*
 * test_ws_client.c -- WebSocket 客户端离线单元测试
 *
 * 覆盖验收标准第 3 条（不依赖网络的那一半）：
 *   - SHA-1 / Base64 标准向量（RFC6455 §1.3 握手示例）
 *   - Upgrade 请求字段完整性
 *   - 101 响应解析与 Sec-WebSocket-Accept 校验（含错误响应被拒绝）
 *   - RFC6455 帧掩码编解码（客户端发必须 MASK=1；服务端帧带掩码应判协议错）
 *   - ping/pong、close 握手、分片重组、半包/粘包
 *   - 断线与指数退避重连
 * 真实 TCP 联调见 tools/ws_mock_server.py + test/ws_live_client.c（make live-test）。
 */
#include "test_util.h"
#include "ws_client.h"
#include "band_port.h"
#include <string.h>
#include <stdio.h>

/* ================================================================== */
/* 内存回环传输：脚本化的"服务端"                                       */
/* ================================================================== */
#define LOOP_BUF 16384

typedef struct {
    net_transport_t base;
    uint8_t  rx[LOOP_BUF];
    size_t   rx_len;
    size_t   rx_pos;
    uint8_t  tx[LOOP_BUF];
    size_t   tx_len;
    int      open;
    uint32_t connects;
    int      fail_connect_times;   /* >0 时接下来 N 次 connect 失败 */
    int      drop_on_recv;         /* 1 = 下一次 recv 返回 NET_ERR_CLOSED */
    int      refuse_send;
} loop_tp_t;

static loop_tp_t g_loop;

static int loop_connect(struct net_transport_s *self, const char *host, uint16_t port,
                        uint32_t timeout_ms)
{
    loop_tp_t *t = (loop_tp_t *)self;
    (void)host; (void)port; (void)timeout_ms;

    if (t->fail_connect_times > 0) {
        t->fail_connect_times--;
        return NET_ERR_CONNECT;
    }
    /* 注意：不清空 rx —— 测试脚本会先把 101 响应放进来 */
    t->open = 1;
    t->connects++;
    return NET_OK;
}

static int loop_send(struct net_transport_s *self, const uint8_t *buf, size_t len)
{
    loop_tp_t *t = (loop_tp_t *)self;

    if ((t->open == 0) || (t->refuse_send != 0)) {
        return NET_ERR_CLOSED;
    }
    if ((t->tx_len + len) > sizeof(t->tx)) {
        return NET_ERR_SOCKET;
    }
    memcpy(&t->tx[t->tx_len], buf, len);
    t->tx_len += len;
    return (int)len;
}

static int loop_recv(struct net_transport_s *self, uint8_t *buf, size_t cap, uint32_t timeout_ms)
{
    loop_tp_t *t = (loop_tp_t *)self;
    size_t avail;
    size_t n;
    (void)timeout_ms;

    if (t->drop_on_recv != 0) {
        t->drop_on_recv = 0;
        t->open = 0;
        return NET_ERR_CLOSED;
    }
    if (t->open == 0) {
        return NET_ERR_CLOSED;
    }
    avail = t->rx_len - t->rx_pos;
    if (avail == 0u) {
        return 0;   /* 超时无数据 */
    }
    n = (avail < cap) ? avail : cap;
    memcpy(buf, &t->rx[t->rx_pos], n);
    t->rx_pos += n;
    return (int)n;
}

static void loop_close(struct net_transport_s *self)
{
    loop_tp_t *t = (loop_tp_t *)self;
    t->open = 0;
}

static int loop_is_open(const struct net_transport_s *self)
{
    const loop_tp_t *t = (const loop_tp_t *)self;
    return t->open;
}

static void loop_set_timeout(struct net_transport_s *self, uint32_t timeout_ms)
{
    (void)self; (void)timeout_ms;
}

static void loop_reset(void)
{
    memset(&g_loop, 0, sizeof(g_loop));
    g_loop.base.name = "loop";
    g_loop.base.connect = loop_connect;
    g_loop.base.send = loop_send;
    g_loop.base.recv = loop_recv;
    g_loop.base.close = loop_close;
    g_loop.base.is_open = loop_is_open;
    g_loop.base.set_timeout = loop_set_timeout;
}

static void loop_clear(void)
{
    g_loop.rx_len = 0u;
    g_loop.rx_pos = 0u;
    g_loop.tx_len = 0u;
}

static void loop_push(const uint8_t *data, size_t len)
{
    if ((g_loop.rx_len + len) <= sizeof(g_loop.rx)) {
        memcpy(&g_loop.rx[g_loop.rx_len], data, len);
        g_loop.rx_len += len;
    }
}

static void loop_push_str(const char *s)
{
    loop_push((const uint8_t *)s, strlen(s));
}

/* 固定熵 -> 固定 Key -> 可预先生成 101 响应 */
static const uint8_t k_rfc_nonce[16] = {
    't', 'h', 'e', ' ', 's', 'a', 'm', 'p', 'l', 'e', ' ', 'n', 'o', 'n', 'c', 'e'
};
#define RFC_KEY_B64    "dGhlIHNhbXBsZSBub25jZQ=="
#define RFC_ACCEPT_B64 "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="

static void loop_push_101_ok(void)
{
    char resp[512];
    (void)snprintf(resp, sizeof(resp),
                   "HTTP/1.1 101 Switching Protocols\r\n"
                   "Upgrade: websocket\r\n"
                   "Connection: Upgrade\r\n"
                   "Sec-WebSocket-Accept: %s\r\n"
                   "\r\n", RFC_ACCEPT_B64);
    loop_push_str(resp);
}

/* ================================================================== */
/* 收到的消息记录                                                      */
/* ================================================================== */
static uint8_t  g_msg_buf[1024];
static size_t   g_msg_len;
static uint8_t  g_msg_opcode;
static uint32_t g_msg_count;
static ws_state_t g_last_new_state;

static void on_msg(void *user, uint8_t opcode, const uint8_t *payload, size_t len)
{
    (void)user;
    g_msg_opcode = opcode;
    g_msg_len = (len < sizeof(g_msg_buf)) ? len : sizeof(g_msg_buf);
    memcpy(g_msg_buf, payload, g_msg_len);
    g_msg_count++;
}

static void on_state(void *user, ws_state_t old_st, ws_state_t new_st)
{
    (void)user; (void)old_st;
    g_last_new_state = new_st;
}

/* ================================================================== */
/* 1) 原语：SHA-1 / Base64                                             */
/* ================================================================== */
static void test_primitives(void)
{
    uint8_t digest[20];
    static const uint8_t expect_abc[20] = {
        0xA9u, 0x99u, 0x3Eu, 0x36u, 0x47u, 0x06u, 0x81u, 0x6Au, 0xBAu, 0x3Eu,
        0x25u, 0x71u, 0x78u, 0x50u, 0xC2u, 0x6Cu, 0x9Cu, 0xD0u, 0xD8u, 0x9Du
    };
    char b64[64];
    uint8_t dec[64];
    size_t n;
    int dn;

    TEST_CASE("SHA-1 标准向量：\"abc\"");
    band_sha1((const uint8_t *)"abc", 3u, digest);
    TEST_ASSERT(memcmp(digest, expect_abc, 20u) == 0, "SHA1(abc) 失配");

    TEST_CASE("SHA-1 空输入（跨块填充路径）");
    band_sha1(NULL, 0u, digest);
    TEST_ASSERT_EQ_UINT(digest[0], 0xDAu, "SHA1(\"\") 首字节应为 DA");
    TEST_ASSERT_EQ_UINT(digest[1], 0x39u, "SHA1(\"\") 次字节应为 39");

    TEST_CASE("SHA-1 长输入（>64 字节，多块处理）");
    {
        uint8_t big[200];
        size_t i;
        for (i = 0u; i < sizeof(big); i++) {
            big[i] = (uint8_t)('a' + (i % 26u));
        }
        band_sha1(big, sizeof(big), digest);
        TEST_ASSERT(digest[0] != 0u || digest[1] != 0u, "长输入应产生摘要");
    }

    TEST_CASE("Base64 编码/解码往返");
    n = band_base64_encode(k_rfc_nonce, 16u, b64, sizeof(b64));
    TEST_ASSERT_EQ_UINT(n, 24u, "16 字节应编成 24 个字符");
    TEST_ASSERT_EQ_STR(b64, RFC_KEY_B64, "RFC 示例 nonce 的 base64");
    dn = band_base64_decode(b64, n, dec, sizeof(dec));
    TEST_ASSERT_EQ_INT(dn, 16, "解码应得 16 字节");
    TEST_ASSERT(memcmp(dec, k_rfc_nonce, 16u) == 0, "解码内容一致");

    TEST_CASE("Base64 三种填充长度");
    (void)band_base64_encode((const uint8_t *)"a", 1u, b64, sizeof(b64));
    TEST_ASSERT_EQ_STR(b64, "YQ==", "1 字节 -> 两个等号");
    (void)band_base64_encode((const uint8_t *)"ab", 2u, b64, sizeof(b64));
    TEST_ASSERT_EQ_STR(b64, "YWI=", "2 字节 -> 一个等号");
    (void)band_base64_encode((const uint8_t *)"abc", 3u, b64, sizeof(b64));
    TEST_ASSERT_EQ_STR(b64, "YWJj", "3 字节 -> 无填充");

    TEST_CASE("Base64 非法字符应被拒绝");
    TEST_ASSERT_EQ_INT(band_base64_decode("!!!!", 4u, dec, sizeof(dec)), -1, "非法字符返回 -1");
}

/* ================================================================== */
/* 2) 握手                                                             */
/* ================================================================== */
static void test_handshake_rfc_vector(void)
{
    char key[WS_KEY_B64_LEN + 1];
    char accept[WS_ACCEPT_LEN + 1];
    char req[512];
    int n;

    TEST_CASE("RFC6455 1.3 示例：Key -> Accept");
    ws_build_key(key, k_rfc_nonce);
    TEST_ASSERT_EQ_STR(key, RFC_KEY_B64, "握手 Key");
    TEST_ASSERT_EQ_INT(ws_compute_accept(key, accept), 0, "计算 Accept");
    TEST_ASSERT_EQ_STR(accept, RFC_ACCEPT_B64, "Accept 必须等于 RFC 示例值");

    TEST_CASE("Upgrade 请求包含全部必需字段");
    n = ws_build_handshake("127.0.0.1", 9001u, "/band", key, req, sizeof(req));
    TEST_ASSERT(n > 0, "应生成请求");
    TEST_ASSERT(strstr(req, "GET /band HTTP/1.1\r\n") != NULL, "请求行");
    TEST_ASSERT(strstr(req, "Host: 127.0.0.1:9001\r\n") != NULL, "Host 头");
    TEST_ASSERT(strstr(req, "Upgrade: websocket\r\n") != NULL, "Upgrade 头");
    TEST_ASSERT(strstr(req, "Connection: Upgrade\r\n") != NULL, "Connection 头");
    TEST_ASSERT(strstr(req, "Sec-WebSocket-Version: 13\r\n") != NULL, "版本 13");
    TEST_ASSERT(strstr(req, "Sec-WebSocket-Key: " RFC_KEY_B64 "\r\n") != NULL, "Key 头");
    TEST_ASSERT(strstr(req, "\r\n\r\n") != NULL, "以空行结束");
    TEST_ASSERT_EQ_STR(strrchr(req, '\n') + 1, "", "空行之后没有多余内容");

    TEST_CASE("101 响应解析（头名大小写不敏感）");
    {
        const char *resp =
            "HTTP/1.1 101 Switching Protocols\r\n"
            "upgrade: WebSocket\r\n"
            "connection: upgrade\r\n"
            "Sec-WebSocket-Accept: " RFC_ACCEPT_B64 "\r\n"
            "\r\n";
        TEST_ASSERT_EQ_INT(ws_parse_handshake_response(resp, strlen(resp), key, accept), 0,
                           "合法响应应解析成功");
        TEST_ASSERT_EQ_STR(accept, RFC_ACCEPT_B64, "解析出的 Accept");
    }

    TEST_CASE("错误响应必须被拒绝");
    {
        const char *bad_status =
            "HTTP/1.1 400 Bad Request\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
            "Sec-WebSocket-Accept: " RFC_ACCEPT_B64 "\r\n\r\n";
        TEST_ASSERT_EQ_INT(ws_parse_handshake_response(bad_status, strlen(bad_status), key, accept),
                           -2, "非 101 应拒绝");
    }
    {
        const char *bad_accept =
            "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
            "Sec-WebSocket-Accept: AAAAAAAAAAAAAAAAAAAAAAAAAAA=\r\n\r\n";
        TEST_ASSERT_EQ_INT(ws_parse_handshake_response(bad_accept, strlen(bad_accept), key, accept),
                           -7, "Accept 不匹配应拒绝");
    }
    {
        const char *no_accept =
            "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n\r\n";
        TEST_ASSERT_EQ_INT(ws_parse_handshake_response(no_accept, strlen(no_accept), key, accept),
                           -6, "缺 Accept 头应拒绝");
    }
    {
        const char *no_upgrade =
            "HTTP/1.1 101 Switching Protocols\r\nConnection: Upgrade\r\n"
            "Sec-WebSocket-Accept: " RFC_ACCEPT_B64 "\r\n\r\n";
        TEST_ASSERT_EQ_INT(ws_parse_handshake_response(no_upgrade, strlen(no_upgrade), key, accept),
                           -3, "缺 Upgrade 头应拒绝");
    }
}

/* ================================================================== */
/* 3) 帧编解码                                                         */
/* ================================================================== */
static void test_frame_mask_codec(void)
{
    uint8_t out[512];
    ws_frame_hdr_t hdr;
    uint8_t payload[512];
    size_t n;
    size_t consumed = 0u;
    uint8_t mask[4] = { 0x37u, 0xFAu, 0x21u, 0x3Du };
    const char *text = "Hello";
    int i;

    TEST_CASE("客户端帧必须带掩码，且载荷可被正确还原");
    n = ws_encode_frame(WS_OP_BINARY, 1u, (const uint8_t *)text, 5u, mask, out, sizeof(out));
    TEST_ASSERT_EQ_UINT(n, 2u + 4u + 5u, "短帧总长 = 2 + 4 + 5");
    TEST_ASSERT_EQ_UINT(out[0], 0x82u, "FIN=1 | opcode=2(binary)");
    TEST_ASSERT((out[1] & 0x80u) != 0u, "MASK 位必须为 1");
    TEST_ASSERT_EQ_UINT(out[1] & 0x7Fu, 5u, "载荷长度 5");
    TEST_ASSERT(memcmp(&out[2], mask, 4u) == 0, "掩码键在头部");
    TEST_ASSERT(memcmp(&out[6], text, 5u) != 0, "载荷必须已被异或掩码");
    for (i = 0; i < 5; i++) {
        TEST_ASSERT_EQ_UINT((uint8_t)(out[6 + i] ^ mask[i & 3]), (uint8_t)text[i],
                            "逐字节解掩码应还原原文");
    }

    TEST_CASE("解码自己的帧：头部字段与载荷一致");
    TEST_ASSERT_EQ_INT(ws_decode_frame(out, n, &hdr, payload, sizeof(payload), &consumed), 0,
                       "解码应成功");
    TEST_ASSERT_EQ_UINT(hdr.opcode, WS_OP_BINARY, "opcode");
    TEST_ASSERT_EQ_UINT(hdr.fin, 1u, "FIN");
    TEST_ASSERT_EQ_UINT(hdr.masked, 1u, "masked");
    TEST_ASSERT_EQ_UINT(hdr.payload_len, 5u, "payload_len");
    TEST_ASSERT_EQ_UINT(consumed, n, "消耗字节数");
    TEST_ASSERT(memcmp(payload, text, 5u) == 0, "解掩码后的载荷");

    TEST_CASE("扩展长度：126 编码（300 字节）");
    {
        uint8_t big[300];
        size_t j;
        for (j = 0u; j < sizeof(big); j++) {
            big[j] = (uint8_t)(j & 0xFFu);
        }
        n = ws_encode_frame(WS_OP_BINARY, 1u, big, 300u, mask, out, sizeof(out));
        TEST_ASSERT_EQ_UINT(n, 4u + 4u + 300u, "300 字节 -> 2 字节扩展长度");
        TEST_ASSERT_EQ_UINT(out[1] & 0x7Fu, 126u, "长度标志 126");
        TEST_ASSERT_EQ_INT(ws_decode_frame(out, n, &hdr, payload, sizeof(payload), &consumed), 0,
                           "解码长帧");
        TEST_ASSERT_EQ_UINT(hdr.payload_len, 300u, "长度还原");
        TEST_ASSERT(memcmp(payload, big, 300u) == 0, "长帧载荷还原");
    }

    TEST_CASE("无掩码帧（服务端 -> 客户端）解析");
    n = ws_encode_frame(WS_OP_TEXT, 1u, (const uint8_t *)"Hi", 2u, NULL, out, sizeof(out));
    TEST_ASSERT_EQ_UINT(n, 4u, "无掩码短帧 = 2 + 2");
    TEST_ASSERT((out[1] & 0x80u) == 0u, "MASK 位应为 0");
    TEST_ASSERT_EQ_INT(ws_decode_frame(out, n, &hdr, payload, sizeof(payload), &consumed), 0,
                       "解析无掩码帧");
    TEST_ASSERT(memcmp(payload, "Hi", 2u) == 0, "载荷直接可读");

    TEST_CASE("半包：数据不足返回 1");
    n = ws_encode_frame(WS_OP_BINARY, 1u, (const uint8_t *)text, 5u, mask, out, sizeof(out));
    TEST_ASSERT_EQ_INT(ws_decode_frame(out, 4u, &hdr, payload, sizeof(payload), &consumed), 1,
                       "只给 4 字节应报需要更多数据");
    TEST_ASSERT_EQ_INT(ws_decode_frame(out, 2u, &hdr, payload, sizeof(payload), &consumed), 1,
                       "只给 2 字节同样需要更多数据");

    TEST_CASE("非法帧必须被拒绝（RSV 非 0 / 控制帧 FIN=0 / 控制帧过长）");
    {
        uint8_t bad[8] = { 0x92u, 0x02u, 'h', 'i', 0u, 0u, 0u, 0u };
        TEST_ASSERT_EQ_INT(ws_decode_frame(bad, 4u, &hdr, payload, sizeof(payload), &consumed),
                           -1, "RSV!=0 应拒绝");
    }
    {
        uint8_t bad[8] = { 0x09u, 0x00u, 0u, 0u, 0u, 0u, 0u, 0u };
        TEST_ASSERT_EQ_INT(ws_decode_frame(bad, 2u, &hdr, payload, sizeof(payload), &consumed),
                           -1, "控制帧 FIN=0 应拒绝");
    }
    {
        uint8_t bad[130];
        memset(bad, 0, sizeof(bad));
        bad[0] = 0x89u;   /* FIN=1 ping */
        bad[1] = 126u;    /* 控制帧不允许扩展长度 */
        bad[2] = 0x00u;
        bad[3] = 0x80u;
        TEST_ASSERT_EQ_INT(ws_decode_frame(bad, sizeof(bad), &hdr, payload, sizeof(payload),
                                           &consumed), -1, "控制帧长度 >125 应拒绝");
    }
    {
        uint8_t small[3] = { 0x82u, 0x85u, 0x00u };
        TEST_ASSERT_EQ_INT(ws_decode_frame(small, sizeof(small), &hdr, payload, 2u, &consumed),
                           1, "帧未收完时应报告需要更多数据");
    }
    {
        /* 完整帧但调用方缓冲不足 -> 必须拒绝，不能截断写入 */
        uint8_t big[64];
        size_t enc_len;
        memset(big, 'x', sizeof(big));
        enc_len = ws_encode_frame(WS_OP_BINARY, 1u, big, 32u, NULL, out, sizeof(out));
        TEST_ASSERT_EQ_INT(ws_decode_frame(out, enc_len, &hdr, payload, 2u, &consumed), -1,
                           "目标缓冲不足应拒绝");
    }
    TEST_CASE("编码输出缓冲不足返回 0");
    {
        uint8_t tiny[4];
        TEST_ASSERT_EQ_UINT(ws_encode_frame(WS_OP_BINARY, 1u, (const uint8_t *)"hello", 5u,
                                            NULL, tiny, sizeof(tiny)), 0u, "缓冲不足返回 0");
    }
}

/* ================================================================== */
/* 4) 完整握手 + 收发 + 保活 + 关闭                                    */
/* ================================================================== */
static void test_end_to_end_loopback(void)
{
    ws_client_t ws;
    uint8_t bin[3] = { 0xAAu, 0x55u, 0x01u };

    TEST_CASE("端到端：握手 -> 发二进制 -> 收二进制 -> ping/pong -> close");
    loop_reset();
    ws_client_init(&ws, &g_loop.base);
    ws_client_set_entropy(&ws, k_rfc_nonce, sizeof(k_rfc_nonce));
    ws_client_set_callbacks(&ws, on_msg, NULL, on_state);
    loop_push_101_ok();

    TEST_ASSERT_EQ_INT(ws_connect(&ws, "127.0.0.1", 9001u, "/band", 2000u), NET_OK,
                       "握手应成功");
    TEST_ASSERT_EQ_INT((int)ws.state, (int)WS_ST_OPEN, "状态应为 OPEN");
    TEST_ASSERT_EQ_STR(ws.hs_key_b64, RFC_KEY_B64, "使用的握手 Key");
    TEST_ASSERT_EQ_STR(ws.hs_accept, RFC_ACCEPT_B64, "校验通过的 Accept");
    TEST_ASSERT_EQ_UINT(ws.handshake_ok, 1u, "握手成功计数");
    TEST_ASSERT_EQ_INT((int)g_last_new_state, (int)WS_ST_OPEN, "状态回调应被触发");
    TEST_ASSERT(strstr((const char *)g_loop.tx, "Upgrade: websocket") != NULL,
                "请求确实发出了");

    TEST_CASE("发送应用数据：抓包校验 MASK 位与内容");
    loop_clear();
    TEST_ASSERT_EQ_INT(ws_send_binary(&ws, bin, sizeof(bin)), 0, "发送应成功");
    TEST_ASSERT_EQ_UINT(g_loop.tx_len, 2u + 4u + 3u, "应是一个带掩码的短帧");
    TEST_ASSERT_EQ_UINT(g_loop.tx[0], 0x82u, "二进制帧头");
    TEST_ASSERT((g_loop.tx[1] & 0x80u) != 0u, "客户端帧必须 MASK=1");
    {
        uint8_t key[4];
        memcpy(key, &g_loop.tx[2], 4u);
        TEST_ASSERT_EQ_UINT((uint8_t)(g_loop.tx[6] ^ key[0]), 0xAAu, "解掩码后第 1 字节");
        TEST_ASSERT_EQ_UINT((uint8_t)(g_loop.tx[7] ^ key[1]), 0x55u, "解掩码后第 2 字节");
        TEST_ASSERT_EQ_UINT((uint8_t)(g_loop.tx[8] ^ key[2]), 0x01u, "解掩码后第 3 字节");
    }

    TEST_CASE("接收服务端消息（一条 recv 里两帧，验证粘包）");
    g_msg_count = 0u;
    {
        uint8_t f1[16];
        uint8_t f2[16];
        size_t n1 = ws_encode_frame(WS_OP_BINARY, 1u, (const uint8_t *)"ONE", 3u, NULL, f1, sizeof(f1));
        size_t n2 = ws_encode_frame(WS_OP_TEXT, 1u, (const uint8_t *)"TWO", 3u, NULL, f2, sizeof(f2));
        loop_push(f1, n1);
        loop_push(f2, n2);
    }
    (void)ws_poll(&ws, 10u);
    TEST_ASSERT_EQ_UINT(g_msg_count, 2u, "应回调两次");
    TEST_ASSERT_EQ_UINT(g_msg_opcode, WS_OP_TEXT, "最后一次是文本帧");
    TEST_ASSERT(g_msg_len == 3u && memcmp(g_msg_buf, "TWO", 3u) == 0, "最后一次内容为 TWO");
    TEST_ASSERT_EQ_UINT(ws.rx_frames, 2u, "收到帧计数");

    TEST_CASE("半包：先给 3 字节再补齐，不应提前回调");
    g_msg_count = 0u;
    {
        uint8_t f[32];
        size_t n = ws_encode_frame(WS_OP_BINARY, 1u, (const uint8_t *)"PARTIAL", 7u, NULL, f, sizeof(f));
        loop_push(f, 3u);
        (void)ws_poll(&ws, 10u);
        TEST_ASSERT_EQ_UINT(g_msg_count, 0u, "半包不应触发回调");
        loop_push(&f[3], n - 3u);
        (void)ws_poll(&ws, 10u);
        TEST_ASSERT_EQ_UINT(g_msg_count, 1u, "补齐后应回调一次");
        TEST_ASSERT(g_msg_len == 7u && memcmp(g_msg_buf, "PARTIAL", 7u) == 0, "内容正确");
    }

    TEST_CASE("ping 自动回 pong（载荷必须与 ping 一致）");
    loop_clear();
    loop_push((const uint8_t *)"\x89\x02xy", 4u);   /* 未掩码 ping，载荷 "xy" */
    (void)ws_poll(&ws, 10u);
    TEST_ASSERT_EQ_UINT(g_loop.tx_len, 2u + 4u + 2u, "pong 长度 = 2 + 4 + 2");
    TEST_ASSERT_EQ_UINT(g_loop.tx[0], 0x8Au, "回帧 opcode 应为 pong(0xA)");
    TEST_ASSERT((g_loop.tx[1] & 0x80u) != 0u, "pong 也必须掩码");
    {
        uint8_t key[4];
        memcpy(key, &g_loop.tx[2], 4u);
        TEST_ASSERT_EQ_UINT((uint8_t)(g_loop.tx[6] ^ key[0]), (uint8_t)'x', "pong 载荷回显 x");
        TEST_ASSERT_EQ_UINT((uint8_t)(g_loop.tx[7] ^ key[1]), (uint8_t)'y', "pong 载荷回显 y");
    }

    TEST_CASE("主动 ping 与收到 pong 后的状态清理");
    TEST_ASSERT_EQ_INT(ws_send_ping(&ws, NULL, 0u), 0, "主动 ping");
    TEST_ASSERT_EQ_UINT(ws.ping_tx, 1u, "ping 计数");
    TEST_ASSERT_EQ_UINT(ws.ping_pending, 1u, "等待 pong");
    loop_push((const uint8_t *)"\x8A\x00", 2u);   /* pong，载荷长度 0 */
    (void)ws_poll(&ws, 10u);
    TEST_ASSERT_EQ_UINT(ws.pong_rx, 1u, "pong 计数");
    TEST_ASSERT_EQ_UINT(ws.ping_pending, 0u, "待答标志已清");

    TEST_CASE("分片消息（text 首片 + continuation 末片）重组");
    g_msg_count = 0u;
    {
        uint8_t frag1[16];
        uint8_t frag2[16];
        size_t n1 = ws_encode_frame(WS_OP_TEXT, 0u, (const uint8_t *)"HEL", 3u, NULL, frag1, sizeof(frag1));
        size_t n2 = ws_encode_frame(WS_OP_CONT, 1u, (const uint8_t *)"LO!", 3u, NULL, frag2, sizeof(frag2));
        loop_push(frag1, n1);
        loop_push(frag2, n2);
    }
    (void)ws_poll(&ws, 10u);
    TEST_ASSERT_EQ_UINT(g_msg_count, 1u, "分片只在末片回调一次");
    TEST_ASSERT(g_msg_len == 6u && memcmp(g_msg_buf, "HELLO!", 6u) == 0, "重组内容");

    TEST_CASE("服务端发带掩码的帧 -> 协议错并断开（RFC6455 5.1）");
    {
        uint8_t bad[16];
        size_t n = ws_encode_frame(WS_OP_TEXT, 1u, (const uint8_t *)"evil", 4u,
                                   (const uint8_t *)"\x01\x02\x03\x04", bad, sizeof(bad));
        loop_push(bad, n);
        (void)ws_poll(&ws, 10u);
        TEST_ASSERT_EQ_INT((int)ws.state, (int)WS_ST_ERROR, "应判协议错并进入 ERROR");
        TEST_ASSERT(ws.protocol_errors >= 1u, "协议错计数");
        TEST_ASSERT_EQ_UINT(g_loop.open, 0, "TCP 应已关闭");
    }

    TEST_CASE("close 握手：回 close 帧并进入 CLOSED");
    loop_reset();
    ws_client_init(&ws, &g_loop.base);
    ws_client_set_entropy(&ws, k_rfc_nonce, sizeof(k_rfc_nonce));
    ws_client_set_callbacks(&ws, on_msg, NULL, on_state);
    loop_push_101_ok();
    TEST_ASSERT_EQ_INT(ws_connect(&ws, "127.0.0.1", 9001u, "/band", 2000u), NET_OK, "重新握手");
    loop_clear();
    {
        uint8_t cl[8];
        size_t n = ws_encode_frame(WS_OP_CLOSE, 1u, (const uint8_t *)"\x03\xE8", 2u, NULL, cl, sizeof(cl));
        loop_push(cl, n);
    }
    (void)ws_poll(&ws, 10u);
    TEST_ASSERT_EQ_UINT(ws.close_rx, 1u, "close 计数");
    TEST_ASSERT_EQ_INT((int)ws.state, (int)WS_ST_CLOSED, "状态应为 CLOSED");
    TEST_ASSERT_EQ_UINT(g_loop.tx[0], 0x88u, "应回 close 帧");
    TEST_ASSERT_EQ_UINT(g_loop.open, 0, "TCP 应已关闭");
}

/* ================================================================== */
/* 5) 断线与重连                                                       */
/* ================================================================== */
static void test_disconnect_and_reconnect(void)
{
    ws_client_t ws;
    int i;

    TEST_CASE("对端断开 -> ERROR；退避后重连成功");
    loop_reset();
    ws_client_init(&ws, &g_loop.base);
    ws_client_set_entropy(&ws, k_rfc_nonce, sizeof(k_rfc_nonce));
    ws_client_set_callbacks(&ws, on_msg, NULL, NULL);
    loop_push_101_ok();
    TEST_ASSERT_EQ_INT(ws_connect(&ws, "127.0.0.1", 9001u, "/band", 2000u), NET_OK, "首次握手");
    TEST_ASSERT_EQ_UINT(ws.handshake_ok, 1u, "握手成功");

    g_loop.drop_on_recv = 1;
    (void)ws_poll(&ws, 10u);
    TEST_ASSERT_EQ_INT((int)ws.state, (int)WS_ST_ERROR, "应进入 ERROR 状态");
    TEST_ASSERT_EQ_UINT(ws.tp->is_open(ws.tp), 0u, "TCP 已关闭");

    TEST_CASE("指数退避：连续失败后 backoff 单调不减且不超上限");
    g_loop.fail_connect_times = 100;
    for (i = 0; i < 6; i++) {
        uint32_t before = ws.backoff_ms;
        (void)ws_reconnect(&ws, 200u);
        TEST_ASSERT(ws.backoff_ms >= before, "退避时间应单调不减");
        TEST_ASSERT(ws.backoff_ms <= WS_RECONNECT_MAX_MS, "退避不超过上限");
    }
    printf("     [data] 6 次失败后 backoff = %u ms, reconnect_count = %u\n",
           (unsigned)ws.backoff_ms, (unsigned)ws.reconnect_count);
    TEST_ASSERT(ws.reconnect_count >= 6u, "重连尝试计数");

    TEST_CASE("恢复网络后重连成功：状态回到 OPEN，退避复位");
    g_loop.fail_connect_times = 0;
    loop_clear();
    loop_push_101_ok();
    TEST_ASSERT_EQ_INT(ws_reconnect(&ws, 2000u), NET_OK, "重连应成功");
    TEST_ASSERT_EQ_INT((int)ws.state, (int)WS_ST_OPEN, "回到 OPEN");
    TEST_ASSERT_EQ_UINT(ws.retry_count, 0u, "成功后重试计数清零");
    TEST_ASSERT_EQ_UINT(ws.backoff_ms, WS_RECONNECT_BASE_MS, "退避回到基数");

    TEST_CASE("handshake 失败（403）应返回错误并置 ERROR");
    loop_reset();
    ws_client_init(&ws, &g_loop.base);
    ws_client_set_entropy(&ws, k_rfc_nonce, sizeof(k_rfc_nonce));
    loop_push_str("HTTP/1.1 403 Forbidden\r\n\r\n");
    TEST_ASSERT(ws_connect(&ws, "127.0.0.1", 9001u, "/band", 500u) != NET_OK, "403 应失败");
    TEST_ASSERT(ws.handshake_fail >= 1u, "握手失败计数");
    TEST_ASSERT_EQ_INT((int)ws.state, (int)WS_ST_ERROR, "状态 ERROR");
}

/* ================================================================== */
/* 6) 边界与状态名                                                     */
/* ================================================================== */
static void test_misc(void)
{
    TEST_CASE("状态名映射完整");
    TEST_ASSERT_EQ_STR(ws_state_name(WS_ST_CLOSED), "CLOSED", "CLOSED");
    TEST_ASSERT_EQ_STR(ws_state_name(WS_ST_OPEN), "OPEN", "OPEN");
    TEST_ASSERT_EQ_STR(ws_state_name(WS_ST_ERROR), "ERROR", "ERROR");
    TEST_ASSERT_EQ_STR(ws_state_name(WS_ST_HANDSHAKE), "HANDSHAKE", "HANDSHAKE");
    TEST_ASSERT_EQ_STR(ws_state_name(WS_ST_CONNECTING), "CONNECTING", "CONNECTING");

    TEST_CASE("未连接时发送应失败而不是崩溃");
    {
        ws_client_t ws;
        loop_reset();
        ws_client_init(&ws, &g_loop.base);
        TEST_ASSERT(ws_send_binary(&ws, (const uint8_t *)"x", 1u) != 0, "未连接发二进制应失败");
        TEST_ASSERT(ws_send_text(&ws, "x") != 0, "未连接发文本应失败");
        TEST_ASSERT(ws_send_ping(&ws, NULL, 0u) != 0, "未连接 ping 应失败");
        TEST_ASSERT_EQ_INT(ws_send_ping(&ws, NULL, 200u), NET_ERR_PARAM, "ping 载荷 >125 应拒绝");
        ws_close(&ws, WS_CLOSE_NORMAL, "bye");   /* 不应崩溃 */
        TEST_ASSERT_EQ_INT((int)ws.state, (int)WS_ST_CLOSED, "仍是 CLOSED");
    }

    TEST_CASE("未注入熵时每次握手使用不同随机数（防重放）");
    {
        uint32_t e1 = band_entropy();
        uint32_t e2 = band_entropy();
        uint32_t e3 = band_entropy();
        TEST_ASSERT(!((e1 == e2) && (e2 == e3)), "band_entropy 不应输出常数列");
    }

    TEST_CASE("16 字节随机数能生成合法的 24 字符 base64 Key（16 字节需 2 个填充位）");
    {
        char key[WS_KEY_B64_LEN + 1];
        uint8_t ent[WS_KEY_LEN];
        size_t i;
        for (i = 0u; i < sizeof(ent); i++) {
            ent[i] = (uint8_t)(band_entropy() & 0xFFu);
        }
        ws_build_key(key, ent);
        TEST_ASSERT_EQ_UINT(strlen(key), WS_KEY_B64_LEN, "Key 长度应为 24 字符");
        TEST_ASSERT(strncmp(&key[22], "==", 2u) == 0, "16 字节 base64 末尾应为 ==");
    }
}

int test_ws_client_run(void)
{
    test_suite_begin("ws_client");
    test_primitives();
    test_handshake_rfc_vector();
    test_frame_mask_codec();
    test_end_to_end_loopback();
    test_disconnect_and_reconnect();
    test_misc();
    return test_suite_end();
}
