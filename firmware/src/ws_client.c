/*
 * ws_client.c -- 手写 WebSocket (RFC6455) 客户端
 *
 * 本文件不依赖任何第三方库，全部为原创实现：
 *   - SHA-1 (FIPS 180-1)      ：用于 Sec-WebSocket-Accept
 *   - Base64 编解码            ：用于握手 Key / Accept
 *   - HTTP/1.1 Upgrade 握手    ：构造请求 + 校验 101 响应
 *   - RFC6455 帧编解码         ：客户端发出的帧强制 MASK=1，服务端帧必须无掩码
 *   - 控制帧                  ：ping / pong / close 握手
 *   - 分片重组                ：continuation 帧累积
 *   - 断线检测 + 指数退避重连
 *
 * 数据流向：
 *   uplink.c ──ws_send_binary()──> [MASK 编码] ──tp->send()──> TCP/Wi-Fi
 *   TCP/Wi-Fi ──tp->recv()──> rx[] ──ws_decode_frame()──> on_msg 回调 / 协议处理
 */
#include "ws_client.h"
#include "band_port.h"
#include <string.h>
#include <stdio.h>

/* ================================================================== */
/* SHA-1 (FIPS 180-1)                                                  */
/* ================================================================== */
typedef struct {
    uint32_t h[5];
    uint64_t total_len;
    uint8_t  buf[64];
    size_t   buf_len;
} sha1_ctx_t;

static uint32_t rotl32(uint32_t v, uint32_t n)
{
    return (uint32_t)((v << n) | (v >> (32u - n)));
}

static void sha1_init(sha1_ctx_t *c)
{
    c->h[0] = 0x67452301u;
    c->h[1] = 0xEFCDAB89u;
    c->h[2] = 0x98BADCFEu;
    c->h[3] = 0x10325476u;
    c->h[4] = 0xC3D2E1F0u;
    c->total_len = 0u;
    c->buf_len = 0u;
}

static void sha1_block(sha1_ctx_t *c, const uint8_t *p)
{
    uint32_t w[80];
    uint32_t a, b, d, e, f, k, tmp;
    uint32_t cc;
    uint8_t i;

    for (i = 0u; i < 16u; i++) {
        w[i] = ((uint32_t)p[i * 4u] << 24) | ((uint32_t)p[i * 4u + 1u] << 16) |
               ((uint32_t)p[i * 4u + 2u] << 8) | (uint32_t)p[i * 4u + 3u];
    }
    for (i = 16u; i < 80u; i++) {
        w[i] = rotl32(w[i - 3u] ^ w[i - 8u] ^ w[i - 14u] ^ w[i - 16u], 1u);
    }

    a = c->h[0]; b = c->h[1]; cc = c->h[2]; d = c->h[3]; e = c->h[4];

    for (i = 0u; i < 80u; i++) {
        if (i < 20u) {
            f = (b & cc) | ((~b) & d);
            k = 0x5A827999u;
        } else if (i < 40u) {
            f = b ^ cc ^ d;
            k = 0x6ED9EBA1u;
        } else if (i < 60u) {
            f = (b & cc) | (b & d) | (cc & d);
            k = 0x8F1BBCDCu;
        } else {
            f = b ^ cc ^ d;
            k = 0xCA62C1D6u;
        }
        tmp = rotl32(a, 5u) + f + e + k + w[i];
        e = d;
        d = cc;
        cc = rotl32(b, 30u);
        b = a;
        a = tmp;
    }

    c->h[0] += a; c->h[1] += b; c->h[2] += cc; c->h[3] += d; c->h[4] += e;
}

static void sha1_update(sha1_ctx_t *c, const uint8_t *data, size_t len)
{
    size_t i = 0u;

    c->total_len += (uint64_t)len;
    while (i < len) {
        size_t space = 64u - c->buf_len;
        size_t n = ((len - i) < space) ? (len - i) : space;
        memcpy(&c->buf[c->buf_len], &data[i], n);
        c->buf_len += n;
        i += n;
        if (c->buf_len == 64u) {
            sha1_block(c, c->buf);
            c->buf_len = 0u;
        }
    }
}

static void sha1_final(sha1_ctx_t *c, uint8_t out[20])
{
    uint64_t bits = c->total_len * 8u;
    uint8_t pad = 0x80u;
    uint8_t zero = 0x00u;
    uint8_t lenbuf[8];
    uint8_t i;

    sha1_update(c, &pad, 1u);
    while (c->buf_len != 56u) {
        sha1_update(c, &zero, 1u);
    }
    for (i = 0u; i < 8u; i++) {
        lenbuf[i] = (uint8_t)((bits >> ((7u - i) * 8u)) & 0xFFu);
    }
    sha1_update(c, lenbuf, 8u);

    for (i = 0u; i < 5u; i++) {
        out[i * 4u]      = (uint8_t)((c->h[i] >> 24) & 0xFFu);
        out[i * 4u + 1u] = (uint8_t)((c->h[i] >> 16) & 0xFFu);
        out[i * 4u + 2u] = (uint8_t)((c->h[i] >> 8) & 0xFFu);
        out[i * 4u + 3u] = (uint8_t)(c->h[i] & 0xFFu);
    }
}

void band_sha1(const uint8_t *data, size_t len, uint8_t digest[20])
{
    sha1_ctx_t c;

    sha1_init(&c);
    if ((data != NULL) && (len > 0u)) {
        sha1_update(&c, data, len);
    }
    sha1_final(&c, digest);
}

/* ================================================================== */
/* Base64                                                              */
/* ================================================================== */
static const char kB64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t band_base64_encode(const uint8_t *src, size_t src_len, char *out, size_t out_cap)
{
    size_t i;
    size_t o = 0u;

    if ((src == NULL) || (out == NULL) || (src_len == 0u)) {
        return 0u;
    }
    if (out_cap < (((src_len + 2u) / 3u) * 4u + 1u)) {
        return 0u;
    }
    for (i = 0u; i + 2u < src_len; i += 3u) {
        uint32_t v = ((uint32_t)src[i] << 16) | ((uint32_t)src[i + 1u] << 8) | (uint32_t)src[i + 2u];
        out[o++] = kB64[(v >> 18) & 0x3Fu];
        out[o++] = kB64[(v >> 12) & 0x3Fu];
        out[o++] = kB64[(v >> 6) & 0x3Fu];
        out[o++] = kB64[v & 0x3Fu];
    }
    if ((src_len - i) == 1u) {
        uint32_t v = (uint32_t)src[i] << 16;
        out[o++] = kB64[(v >> 18) & 0x3Fu];
        out[o++] = kB64[(v >> 12) & 0x3Fu];
        out[o++] = '=';
        out[o++] = '=';
    } else if ((src_len - i) == 2u) {
        uint32_t v = ((uint32_t)src[i] << 16) | ((uint32_t)src[i + 1u] << 8);
        out[o++] = kB64[(v >> 18) & 0x3Fu];
        out[o++] = kB64[(v >> 12) & 0x3Fu];
        out[o++] = kB64[(v >> 6) & 0x3Fu];
        out[o++] = '=';
    }
    out[o] = '\0';
    return o;
}

static int b64_index(char c)
{
    const char *p = strchr(kB64, c);
    if (p == NULL) {
        return -1;
    }
    return (int)(p - kB64);
}

int band_base64_decode(const char *src, size_t src_len, uint8_t *out, size_t out_cap)
{
    size_t i;
    size_t o = 0u;
    uint32_t acc = 0u;
    uint8_t bits = 0u;

    if ((src == NULL) || (out == NULL)) {
        return -1;
    }
    for (i = 0u; i < src_len; i++) {
        int v;
        char c = src[i];
        if ((c == '=') || (c == '\r') || (c == '\n') || (c == ' ')) {
            continue;
        }
        v = b64_index(c);
        if (v < 0) {
            return -1;
        }
        acc = (acc << 6) | (uint32_t)v;
        bits = (uint8_t)(bits + 6u);
        if (bits >= 8u) {
            bits = (uint8_t)(bits - 8u);
            if (o >= out_cap) {
                return -1;
            }
            out[o++] = (uint8_t)((acc >> bits) & 0xFFu);
        }
    }
    return (int)o;
}

/* ================================================================== */
/* 握手                                                                */
/* ================================================================== */
void ws_build_key(char out_b64[WS_KEY_B64_LEN + 1], const uint8_t entropy[WS_KEY_LEN])
{
    if ((out_b64 == NULL) || (entropy == NULL)) {
        return;
    }
    (void)band_base64_encode(entropy, WS_KEY_LEN, out_b64, WS_KEY_B64_LEN + 1u);
}

int ws_compute_accept(const char *key_b64, char out[WS_ACCEPT_LEN + 1])
{
    char cat[WS_KEY_B64_LEN + 64];
    uint8_t digest[20];
    size_t n;
    size_t d;

    if ((key_b64 == NULL) || (out == NULL)) {
        return -1;
    }
    n = strlen(key_b64);
    if ((n + strlen(WS_GUID)) >= sizeof(cat)) {
        return -1;
    }
    memcpy(cat, key_b64, n);
    memcpy(&cat[n], WS_GUID, strlen(WS_GUID));
    n += strlen(WS_GUID);

    band_sha1((const uint8_t *)cat, n, digest);
    d = band_base64_encode(digest, sizeof(digest), out, WS_ACCEPT_LEN + 1u);
    if (d != WS_ACCEPT_LEN) {
        return -1;
    }
    return 0;
}

int ws_build_handshake(const char *host, uint16_t port, const char *path,
                       const char *key_b64, char *out, size_t out_cap)
{
    int n;

    if ((host == NULL) || (path == NULL) || (key_b64 == NULL) || (out == NULL)) {
        return -1;
    }
    n = snprintf(out, out_cap,
                 "GET %s HTTP/1.1\r\n"
                 "Host: %s:%u\r\n"
                 "Upgrade: websocket\r\n"
                 "Connection: Upgrade\r\n"
                 "Sec-WebSocket-Key: %s\r\n"
                 "Sec-WebSocket-Version: 13\r\n"
                 "User-Agent: stm32-wearable-health-band/%s\r\n"
                 "\r\n",
                 path, host, (unsigned)port, key_b64, BAND_FW_VERSION_STR);
    if ((n <= 0) || ((size_t)n >= out_cap)) {
        return -1;
    }
    return n;
}

/* 大小写不敏感的子串查找（HTTP 头名大小写不敏感） */
static const char *find_ci(const char *hay, const char *needle)
{
    size_t nlen = strlen(needle);
    const char *p;

    for (p = hay; *p != '\0'; p++) {
        size_t i;
        for (i = 0u; i < nlen; i++) {
            char a = p[i];
            char b = needle[i];
            if (a >= 'A' && a <= 'Z') {
                a = (char)(a - 'A' + 'a');
            }
            if (b >= 'A' && b <= 'Z') {
                b = (char)(b - 'A' + 'a');
            }
            if ((a != b) || (a == '\0')) {
                break;
            }
        }
        if (i == nlen) {
            return p;
        }
    }
    return NULL;
}

int ws_parse_handshake_response(const char *resp, size_t len,
                                const char *key_b64, char accept_out[WS_ACCEPT_LEN + 1])
{
    char expect[WS_ACCEPT_LEN + 1];
    const char *p;

    if ((resp == NULL) || (key_b64 == NULL)) {
        return -1;
    }
    if (len == 0u) {
        return -1;
    }
    /* 1) 状态行必须是 101 */
    if (strncmp(resp, "HTTP/1.1 101", 12u) != 0) {
        if (strncmp(resp, "HTTP/1.0 101", 12u) != 0) {
            return -2;
        }
    }
    /* 2) 必须有 Upgrade: websocket 与 Connection: Upgrade（大小写不敏感） */
    if (find_ci(resp, "upgrade:") == NULL) {
        return -3;
    }
    if (find_ci(resp, "websocket") == NULL) {
        return -3;
    }
    if (find_ci(resp, "connection:") == NULL) {
        return -4;
    }
    /* 3) Sec-WebSocket-Accept 必须等于 base64(SHA1(key + GUID)) */
    if (ws_compute_accept(key_b64, expect) != 0) {
        return -5;
    }
    p = find_ci(resp, "sec-websocket-accept:");
    if (p == NULL) {
        return -6;
    }
    p += strlen("sec-websocket-accept:");
    while ((*p == ' ') || (*p == '\t')) {
        p++;
    }
    if (strncmp(p, expect, WS_ACCEPT_LEN) != 0) {
        return -7;
    }
    if (accept_out != NULL) {
        memcpy(accept_out, expect, WS_ACCEPT_LEN);
        accept_out[WS_ACCEPT_LEN] = '\0';
    }
    return 0;
}

/* ================================================================== */
/* RFC6455 帧编解码                                                    */
/* ================================================================== */
size_t ws_encode_frame(uint8_t opcode, uint8_t fin, const uint8_t *payload, size_t len,
                       const uint8_t mask_key[4], uint8_t *out, size_t out_cap)
{
    size_t header;
    size_t i;
    int masked = (mask_key != NULL) ? 1 : 0;

    if (out == NULL) {
        return 0u;
    }
    if ((len > 0u) && (payload == NULL)) {
        return 0u;
    }
    header = 2u;
    if (len > 125u) {
        header += (len <= 0xFFFFu) ? 2u : 8u;
    }
    if (masked != 0) {
        header += 4u;
    }
    if (out_cap < (header + len)) {
        return 0u;
    }

    out[0] = (uint8_t)((fin != 0u ? 0x80u : 0x00u) | (opcode & 0x0Fu));
    if (len <= 125u) {
        out[1] = (uint8_t)((masked != 0 ? 0x80u : 0x00u) | (uint8_t)len);
    } else if (len <= 0xFFFFu) {
        out[1] = (uint8_t)((masked != 0 ? 0x80u : 0x00u) | 126u);
        out[2] = (uint8_t)((len >> 8) & 0xFFu);
        out[3] = (uint8_t)(len & 0xFFu);
    } else {
        uint8_t k;
        out[1] = (uint8_t)((masked != 0 ? 0x80u : 0x00u) | 127u);
        for (k = 0u; k < 8u; k++) {
            out[2u + k] = (uint8_t)((len >> ((7u - k) * 8u)) & 0xFFu);
        }
    }

    if (masked != 0) {
        memcpy(&out[header - 4u], mask_key, 4u);
        for (i = 0u; i < len; i++) {
            out[header + i] = (uint8_t)(payload[i] ^ mask_key[i & 3u]);
        }
    } else {
        for (i = 0u; i < len; i++) {
            out[header + i] = payload[i];
        }
    }
    return header + len;
}

int ws_parse_frame_header(const uint8_t *buf, size_t len, ws_frame_hdr_t *hdr)
{
    size_t need;
    uint8_t b0;
    uint8_t b1;

    if ((buf == NULL) || (hdr == NULL)) {
        return -1;
    }
    if (len < 2u) {
        return 1;
    }
    b0 = buf[0];
    b1 = buf[1];

    hdr->fin = (uint8_t)((b0 & 0x80u) ? 1u : 0u);
    hdr->rsv = (uint8_t)((b0 & 0x70u) >> 4);
    hdr->opcode = (uint8_t)(b0 & 0x0Fu);
    hdr->masked = (uint8_t)((b1 & 0x80u) ? 1u : 0u);

    /* RSV 必须为 0（未协商任何扩展） */
    if (hdr->rsv != 0u) {
        return -1;
    }

    need = 2u;
    if ((b1 & 0x7Fu) <= 125u) {
        hdr->payload_len = (uint64_t)(b1 & 0x7Fu);
    } else if ((b1 & 0x7Fu) == 126u) {
        need += 2u;
        if (len < need) {
            return 1;
        }
        hdr->payload_len = ((uint64_t)buf[2] << 8) | (uint64_t)buf[3];
    } else {
        uint8_t k;
        need += 8u;
        if (len < need) {
            return 1;
        }
        hdr->payload_len = 0u;
        for (k = 0u; k < 8u; k++) {
            hdr->payload_len = (hdr->payload_len << 8) | (uint64_t)buf[2u + k];
        }
    }
    if (hdr->masked != 0u) {
        need += 4u;
        if (len < need) {
            return 1;
        }
        memcpy(hdr->mask, &buf[need - 4u], 4u);
    }
    hdr->header_len = need;
    return 0;
}

int ws_decode_frame(const uint8_t *buf, size_t len, ws_frame_hdr_t *hdr,
                    uint8_t *payload_out, size_t payload_cap, size_t *consumed)
{
    int rc;
    size_t i;
    int is_control;

    if ((buf == NULL) || (hdr == NULL)) {
        return -1;
    }
    rc = ws_parse_frame_header(buf, len, hdr);
    if (rc != 0) {
        return rc;
    }
    is_control = (hdr->opcode >= 0x8u) ? 1 : 0;

    /* 控制帧的结构约束与"数据是否到齐"无关，必须先判断，
     * 否则一个声明了超长载荷的控制帧会被误判成"半包"而无限等待。 */
    if ((is_control != 0) && ((hdr->fin == 0u) || (hdr->payload_len > 125u))) {
        return -1;
    }
    if (len < (hdr->header_len + (size_t)hdr->payload_len)) {
        return 1;   /* 半包 */
    }
    if ((size_t)hdr->payload_len > payload_cap) {
        return -1;   /* 调用方缓冲不足 */
    }

    for (i = 0u; i < (size_t)hdr->payload_len; i++) {
        uint8_t v = buf[hdr->header_len + i];
        if (hdr->masked != 0u) {
            v = (uint8_t)(v ^ hdr->mask[i & 3u]);
        }
        if (payload_out != NULL) {
            payload_out[i] = v;
        }
    }
    if (consumed != NULL) {
        *consumed = hdr->header_len + (size_t)hdr->payload_len;
    }
    return 0;
}

/* 便捷版本：内部使用静态缓冲，仅适用于单线程顺序解码（固件中即是如此） */
static uint8_t g_simple_buf[WS_MAX_FRAME_PAYLOAD];

int ws_decode_frame_simple(const uint8_t *buf, size_t len, uint8_t *opcode, uint8_t *fin,
                           uint8_t *masked, uint8_t mask_key[4],
                           const uint8_t **payload, size_t *payload_len, size_t *consumed)
{
    ws_frame_hdr_t hdr;
    size_t used = 0u;
    int rc;

    rc = ws_decode_frame(buf, len, &hdr, g_simple_buf, sizeof(g_simple_buf), &used);
    if (rc != 0) {
        return rc;
    }
    if (opcode != NULL)   { *opcode = hdr.opcode; }
    if (fin != NULL)      { *fin = hdr.fin; }
    if (masked != NULL)   { *masked = hdr.masked; }
    if (mask_key != NULL) { memcpy(mask_key, hdr.mask, 4u); }
    if (payload != NULL)  { *payload = g_simple_buf; }
    if (payload_len != NULL) { *payload_len = (size_t)hdr.payload_len; }
    if (consumed != NULL) { *consumed = used; }
    return 0;
}

/* ================================================================== */
/* 客户端状态机                                                        */
/* ================================================================== */
const char *ws_state_name(ws_state_t st)
{
    switch (st) {
    case WS_ST_CLOSED:      return "CLOSED";
    case WS_ST_CONNECTING:  return "CONNECTING";
    case WS_ST_HANDSHAKE:   return "HANDSHAKE";
    case WS_ST_OPEN:        return "OPEN";
    case WS_ST_CLOSING:     return "CLOSING";
    case WS_ST_ERROR:       return "ERROR";
    default:                return "?";
    }
}

static void ws_set_state(ws_client_t *ws, ws_state_t st)
{
    ws_state_t old = ws->state;

    if (old == st) {
        return;
    }
    ws->state = st;
    if (ws->on_state != NULL) {
        ws->on_state(ws->user, old, st);
    }
}

void ws_client_init(ws_client_t *ws, net_transport_t *tp)
{
    if (ws == NULL) {
        return;
    }
    memset(ws, 0, sizeof(*ws));
    ws->tp = (tp != NULL) ? tp : net_socket_get();
    ws->state = WS_ST_CLOSED;
    ws->auto_reconnect = 1u;
    ws->backoff_ms = WS_RECONNECT_BASE_MS;
    strncpy(ws->cfg.path, WS_DEFAULT_PATH, sizeof(ws->cfg.path) - 1u);
    ws->cfg.port = (uint16_t)WS_DEFAULT_PORT;
}

void ws_client_set_callbacks(ws_client_t *ws, ws_msg_cb_t on_msg, void *user, ws_state_cb_t on_state)
{
    if (ws == NULL) {
        return;
    }
    ws->on_msg = on_msg;
    ws->user = user;
    ws->on_state = on_state;
}

void ws_client_set_entropy(ws_client_t *ws, const uint8_t *entropy, size_t len)
{
    if ((ws == NULL) || (entropy == NULL) || (len < WS_KEY_LEN)) {
        return;
    }
    memcpy(ws->cfg.entropy, entropy, WS_KEY_LEN);
    ws->cfg.entropy_set = 1u;
}

void ws_client_reset_stats(ws_client_t *ws)
{
    if (ws == NULL) {
        return;
    }
    ws->tx_frames = 0u;
    ws->rx_frames = 0u;
    ws->tx_bytes = 0u;
    ws->rx_bytes = 0u;
    ws->ping_tx = 0u;
    ws->pong_rx = 0u;
    ws->close_rx = 0u;
    ws->handshake_ok = 0u;
    ws->handshake_fail = 0u;
    ws->protocol_errors = 0u;
    ws->reconnect_count = 0u;
    ws->decoded_msgs = 0u;
}

/* 生成 16 字节握手随机数：每次握手都必须不同，否则服务端会认为是重放 */
static void ws_gen_entropy(ws_client_t *ws)
{
    uint32_t a;
    uint32_t b;
    uint32_t c;
    uint32_t d;
    uint8_t i;

    if (ws->cfg.entropy_set != 0u) {
        return;
    }
    a = band_entropy();
    b = band_entropy() ^ (uint32_t)(uintptr_t)ws;
    c = band_entropy() ^ (uint32_t)band_millis();
    d = (uint32_t)(ws->handshake_ok + 0x9E3779B9u);
    for (i = 0u; i < 4u; i++) {
        /* xorshift32 混合，避免 band_entropy 在低熵平台上输出强相关 */
        a ^= a << 13; a ^= a >> 17; a ^= a << 5;
        ws->cfg.entropy[i * 4u + 0u] = (uint8_t)(a & 0xFFu);
        ws->cfg.entropy[i * 4u + 1u] = (uint8_t)((a >> 8) & 0xFFu);
        b ^= b << 13; b ^= b >> 17; b ^= b << 5;
        ws->cfg.entropy[i * 4u + 2u] = (uint8_t)(b & 0xFFu);
        c ^= c << 13; c ^= c >> 17; c ^= c << 5;
        ws->cfg.entropy[i * 4u + 3u] = (uint8_t)((c ^ d) & 0xFFu);
    }
}

/* 发送一个已编码好的 WebSocket 帧（客户端帧强制掩码） */
static int ws_send_raw_frame(ws_client_t *ws, uint8_t opcode, const uint8_t *payload, size_t len)
{
    uint8_t mask[4];
    uint8_t hdr[WS_TX_BUF];
    size_t total;
    int rc;
    uint32_t r = band_entropy();
    size_t i;

    if ((ws == NULL) || (ws->tp == NULL) || (ws->tp->is_open(ws->tp) == 0)) {
        return NET_ERR_CLOSED;
    }
    if ((len + 14u) > sizeof(hdr)) {
        return NET_ERR_PARAM;
    }
    /* RFC6455 §5.3：客户端必须使用新的随机掩码键 */
    for (i = 0u; i < 4u; i++) {
        r ^= r << 13; r ^= r >> 17; r ^= r << 5;
        mask[i] = (uint8_t)(r & 0xFFu);
    }
    total = ws_encode_frame(opcode, 1u, payload, len, mask, hdr, sizeof(hdr));
    if (total == 0u) {
        return NET_ERR_PARAM;
    }
    rc = ws->tp->send(ws->tp, hdr, total);
    if (rc != (int)total) {
        return NET_ERR_SOCKET;
    }
    ws->tx_frames++;
    ws->tx_bytes += (uint32_t)total;
    ws->last_tx_ms = band_millis();
    return 0;
}

int ws_send_binary(ws_client_t *ws, const uint8_t *data, size_t len)
{
    if (ws->state != WS_ST_OPEN) {
        return NET_ERR_CLOSED;
    }
    return ws_send_raw_frame(ws, WS_OP_BINARY, data, len);
}

int ws_send_text(ws_client_t *ws, const char *text)
{
    if ((text == NULL) || (ws->state != WS_ST_OPEN)) {
        return NET_ERR_CLOSED;
    }
    return ws_send_raw_frame(ws, WS_OP_TEXT, (const uint8_t *)text, strlen(text));
}

int ws_send_ping(ws_client_t *ws, const uint8_t *payload, size_t len)
{
    int rc;

    if (len > 125u) {
        return NET_ERR_PARAM;
    }
    rc = ws_send_raw_frame(ws, WS_OP_PING, payload, len);
    if (rc == 0) {
        ws->ping_tx++;
        ws->ping_sent_ms = band_millis();
        ws->ping_pending = 1u;
    }
    return rc;
}

int ws_send_pong(ws_client_t *ws, const uint8_t *payload, size_t len)
{
    if (len > 125u) {
        len = 125u;
    }
    return ws_send_raw_frame(ws, WS_OP_PONG, payload, len);
}

/* 断开底层连接 */
static void ws_drop(ws_client_t *ws, ws_state_t st)
{
    if ((ws != NULL) && (ws->tp != NULL)) {
        ws->tp->close(ws->tp);
    }
    if (ws != NULL) {
        ws->rx_len = 0u;
        ws->hs_rx_len = 0u;
        ws->frag_len = 0u;
        ws->ping_pending = 0u;
        ws_set_state(ws, st);
    }
}

void ws_close(ws_client_t *ws, uint16_t code, const char *reason)
{
    uint8_t buf[125];
    size_t rlen = 0u;

    if (ws == NULL) {
        return;
    }
    if ((ws->state == WS_ST_OPEN) || (ws->state == WS_ST_HANDSHAKE)) {
        buf[0] = (uint8_t)((code >> 8) & 0xFFu);
        buf[1] = (uint8_t)(code & 0xFFu);
        rlen = 2u;
        if (reason != NULL) {
            size_t n = strlen(reason);
            if (n > 123u) {
                n = 123u;
            }
            memcpy(&buf[2], reason, n);
            rlen += n;
        }
        (void)ws_send_raw_frame(ws, WS_OP_CLOSE, buf, rlen);
        ws_set_state(ws, WS_ST_CLOSING);
    }
    ws_drop(ws, WS_ST_CLOSED);
}

/* ------------------------------------------------------------------ */
/* 握手驱动                                                            */
/* ------------------------------------------------------------------ */
/* 返回 1 = 完成，0 = 还需数据，<0 = 失败 */
static int ws_drive_handshake(ws_client_t *ws, uint32_t timeout_ms)
{
    char req[512];
    char accept[WS_ACCEPT_LEN + 1];
    int n;
    uint32_t deadline = band_millis() + timeout_ms;
    int rc;

    if (ws->state == WS_ST_HANDSHAKE) {
        /* 已发请求，继续收响应 */
    } else {
        ws_gen_entropy(ws);
        ws_build_key(ws->hs_key_b64, ws->cfg.entropy);
        n = ws_build_handshake(ws->cfg.host, ws->cfg.port, ws->cfg.path, ws->hs_key_b64,
                               req, sizeof(req));
        if (n < 0) {
            ws->handshake_fail++;
            return -1;
        }
        if (ws->tp->send(ws->tp, (const uint8_t *)req, (size_t)n) != n) {
            ws->handshake_fail++;
            return -1;
        }
        ws->hs_rx_len = 0u;
        ws->tx_bytes += (uint32_t)n;
        ws->last_tx_ms = band_millis();
        ws_set_state(ws, WS_ST_HANDSHAKE);
    }

    for (;;) {
        int got;
        if (ws->hs_rx_len >= (sizeof(ws->hs_rx) - 1u)) {
            ws->handshake_fail++;
            return -1;
        }
        got = ws->tp->recv(ws->tp, (uint8_t *)&ws->hs_rx[ws->hs_rx_len],
                           sizeof(ws->hs_rx) - 1u - ws->hs_rx_len, 200u);
        if (got == NET_ERR_CLOSED) {
            ws->handshake_fail++;
            return -1;
        }
        if (got > 0) {
            ws->hs_rx_len += (size_t)got;
            ws->hs_rx[ws->hs_rx_len] = '\0';
            ws->rx_bytes += (uint32_t)got;
            /* 响应头以空行结束 */
            if (strstr(ws->hs_rx, "\r\n\r\n") == NULL) {
                if (band_time_after(band_millis(), deadline)) {
                    ws->handshake_fail++;
                    return -1;
                }
                continue;
            }
            rc = ws_parse_handshake_response(ws->hs_rx, ws->hs_rx_len, ws->hs_key_b64, accept);
            if (rc != 0) {
                ws->handshake_fail++;
                return -2;
            }
            memcpy(ws->hs_accept, accept, sizeof(ws->hs_accept));
            ws->handshake_ok++;
            ws->retry_count = 0u;
            ws->backoff_ms = WS_RECONNECT_BASE_MS;
            ws->last_rx_ms = band_millis();
            /* 101 之后可能紧跟 WebSocket 数据帧，把它们保留在 rx 缓冲 */
            {
                const char *tail = strstr(ws->hs_rx, "\r\n\r\n");
                size_t off = (size_t)(tail - ws->hs_rx) + 4u;
                size_t leftover = ws->hs_rx_len - off;
                if (leftover > 0u) {
                    if (leftover > sizeof(ws->rx)) {
                        leftover = sizeof(ws->rx);
                    }
                    memcpy(ws->rx, &ws->hs_rx[off], leftover);
                    ws->rx_len = leftover;
                } else {
                    ws->rx_len = 0u;
                }
            }
            ws_set_state(ws, WS_ST_OPEN);
            return 1;
        }
        if (band_time_after(band_millis(), deadline)) {
            ws->handshake_fail++;
            return -3;
        }
    }
}

int ws_connect(ws_client_t *ws, const char *host, uint16_t port, const char *path,
               uint32_t timeout_ms)
{
    int rc;

    if ((ws == NULL) || (host == NULL)) {
        return NET_ERR_PARAM;
    }
    /* 注意：ws_reconnect()/uplink_connect() 会把 ws->cfg.host 再传回来，
     * 此时 src 与 dst 是同一块内存，strncpy 属于未定义行为（重叠），
     * 必须先判断是否真的需要拷贝。 */
    if (host != ws->cfg.host) {
        strncpy(ws->cfg.host, host, sizeof(ws->cfg.host) - 1u);
        ws->cfg.host[sizeof(ws->cfg.host) - 1u] = '\0';
    }
    ws->cfg.port = port;
    if ((path != NULL) && (path != ws->cfg.path)) {
        strncpy(ws->cfg.path, path, sizeof(ws->cfg.path) - 1u);
        ws->cfg.path[sizeof(ws->cfg.path) - 1u] = '\0';
    }

    ws_set_state(ws, WS_ST_CONNECTING);
    rc = ws->tp->connect(ws->tp, host, port, timeout_ms);
    if (rc != NET_OK) {
        ws->handshake_fail++;
        ws_set_state(ws, WS_ST_ERROR);
        return rc;
    }
    ws->hs_rx_len = 0u;

    /* 握手阶段内部轮询，超时时间留给整个握手过程 */
    for (;;) {
        int h = ws_drive_handshake(ws, timeout_ms);
        if (h == 1) {
            return NET_OK;
        }
        if (h < 0) {
            ws_drop(ws, WS_ST_ERROR);
            return (h == -3) ? NET_ERR_TIMEOUT : NET_ERR_CONNECT;
        }
    }
}

int ws_reconnect(ws_client_t *ws, uint32_t timeout_ms)
{
    int rc;

    if (ws == NULL) {
        return NET_ERR_PARAM;
    }
    if (ws->tp->is_open(ws->tp) != 0) {
        ws->tp->close(ws->tp);
    }
    ws->reconnect_count++;
    ws->retry_count++;
    rc = ws_connect(ws, ws->cfg.host, ws->cfg.port, ws->cfg.path, timeout_ms);
    if (rc != NET_OK) {
        /* 指数退避：200ms -> 400 -> 800 -> ... 上限 8s，避免把路由器打爆 */
        uint32_t back = WS_RECONNECT_BASE_MS;
        uint8_t i;
        for (i = 0u; i < ws->retry_count && i < 8u; i++) {
            back <<= 1;
            if (back >= WS_RECONNECT_MAX_MS) {
                back = WS_RECONNECT_MAX_MS;
                break;
            }
        }
        ws->backoff_ms = back;
        ws->next_retry_ms = band_millis() + back;
    }
    return rc;
}

/* ------------------------------------------------------------------ */
/* 收包与协议处理                                                      */
/* ------------------------------------------------------------------ */
static void ws_dispatch_message(ws_client_t *ws, uint8_t opcode, const uint8_t *payload, size_t len)
{
    ws->decoded_msgs++;
    if ((ws->on_msg != NULL) && ((opcode == WS_OP_TEXT) || (opcode == WS_OP_BINARY))) {
        ws->on_msg(ws->user, opcode, payload, len);
    }
}

/* 处理一个已解出的帧；返回 0 继续，<0 需要断开连接 */
static int ws_handle_frame(ws_client_t *ws, const ws_frame_hdr_t *hdr,
                           const uint8_t *payload, size_t len)
{
    switch (hdr->opcode) {
    case WS_OP_PING:
        /* 收到 ping 必须回 pong，且 pong 载荷与 ping 完全一致 */
        (void)ws_send_pong(ws, payload, len);
        break;

    case WS_OP_PONG:
        ws->pong_rx++;
        ws->ping_pending = 0u;
        break;

    case WS_OP_CLOSE:
        ws->close_rx++;
        /* 回一个 close 完成关闭握手，然后关闭 TCP */
        (void)ws_send_raw_frame(ws, WS_OP_CLOSE, payload, (len >= 2u) ? 2u : len);
        ws_drop(ws, WS_ST_CLOSED);
        return -1;

    case WS_OP_TEXT:
    case WS_OP_BINARY:
        if (hdr->fin != 0u) {
            ws_dispatch_message(ws, hdr->opcode, payload, len);
        } else {
            /* 分片起点 */
            ws->frag_opcode = hdr->opcode;
            ws->frag_len = (len <= sizeof(ws->frag)) ? len : sizeof(ws->frag);
            memcpy(ws->frag, payload, ws->frag_len);
        }
        break;

    case WS_OP_CONT: {
        size_t space = sizeof(ws->frag) - ws->frag_len;
        size_t n = (len < space) ? len : space;
        memcpy(&ws->frag[ws->frag_len], payload, n);
        ws->frag_len += n;
        if (hdr->fin != 0u) {
            ws_dispatch_message(ws, ws->frag_opcode, ws->frag, ws->frag_len);
            ws->frag_len = 0u;
            ws->frag_opcode = 0u;
        }
        break;
    }

    default:
        /* 未知 opcode：按协议错误处理 */
        ws->protocol_errors++;
        return -1;
    }
    return 0;
}

int ws_poll(ws_client_t *ws, uint32_t timeout_ms)
{
    int got;
    int processed = 0;

    if (ws == NULL) {
        return NET_ERR_PARAM;
    }

    if (ws->state == WS_ST_HANDSHAKE) {
        int h = ws_drive_handshake(ws, timeout_ms);
        if (h < 0) {
            ws_drop(ws, WS_ST_ERROR);
        }
        return 0;
    }

    if (ws->state == WS_ST_ERROR) {
        /* 自动重连：退避时间到了才尝试，避免在断网时疯狂重试 */
        if ((ws->auto_reconnect != 0u) &&
            (!band_time_after(ws->next_retry_ms, band_millis()))) {
            (void)ws_reconnect(ws, timeout_ms);
        }
        return 0;
    }

    if (ws->state != WS_ST_OPEN) {
        return 0;
    }

    /* 1) 收数据 */
    if (ws->rx_len < sizeof(ws->rx)) {
        got = ws->tp->recv(ws->tp, &ws->rx[ws->rx_len], sizeof(ws->rx) - ws->rx_len,
                           timeout_ms);
        if (got == NET_ERR_CLOSED) {
            /* 对端断开：进入 ERROR，等待退避重连 */
            ws_drop(ws, WS_ST_ERROR);
            ws->next_retry_ms = band_millis() + ws->backoff_ms;
            return processed;
        }
        if (got > 0) {
            ws->rx_len += (size_t)got;
            ws->last_rx_ms = band_millis();
            ws->rx_bytes += (uint32_t)got;
        }
    }

    /* 2) 解帧（一个缓冲里可能有多帧 —— 粘包；也可能只有半帧 —— 半包） */
    for (;;) {
        ws_frame_hdr_t hdr;
        uint8_t payload[WS_MAX_FRAME_PAYLOAD];
        size_t used = 0u;
        int rc;

        rc = ws_decode_frame(ws->rx, ws->rx_len, &hdr, payload, sizeof(payload), &used);
        if (rc == 1) {
            break;   /* 半包：留待下次 recv */
        }
        if (rc < 0) {
            ws->protocol_errors++;
            ws_drop(ws, WS_ST_ERROR);
            ws->next_retry_ms = band_millis() + ws->backoff_ms;
            return processed;
        }
        /* RFC6455 §5.1：服务端发往客户端的帧必须不带掩码。
         * 解码器是双向通用的，所以这条策略放在客户端状态机里。 */
        if (hdr.masked != 0u) {
            ws->protocol_errors++;
            ws_drop(ws, WS_ST_ERROR);
            ws->next_retry_ms = band_millis() + ws->backoff_ms;
            return processed;
        }
        ws->rx_frames++;
        processed += (int)used;

        /* 移出已消费字节 */
        if (used < ws->rx_len) {
            memmove(ws->rx, &ws->rx[used], ws->rx_len - used);
        }
        ws->rx_len -= used;

        if (ws_handle_frame(ws, &hdr, payload, (size_t)hdr.payload_len) < 0) {
            return processed;
        }
        if (ws->state != WS_ST_OPEN) {
            return processed;
        }
        if (ws->rx_len == 0u) {
            break;
        }
    }

    /* 3) 保活：长时间没发数据就 ping，ping 超时未收到 pong 则判定链路已死 */
    {
        uint32_t now = band_millis();
        if ((ws->ping_pending != 0u) &&
            (band_time_after(now, ws->ping_sent_ms + WS_PONG_TIMEOUT_MS))) {
            ws->ping_pending = 0u;
            ws_drop(ws, WS_ST_ERROR);
            ws->next_retry_ms = now + ws->backoff_ms;
            return processed;
        }
        if ((ws->ping_pending == 0u) &&
            (band_time_after(now, ws->last_tx_ms + WS_PING_INTERVAL_MS))) {
            (void)ws_send_ping(ws, NULL, 0u);
        }
    }
    return processed;
}
