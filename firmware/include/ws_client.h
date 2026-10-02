/*
 * ws_client.h -- 手写 WebSocket (RFC6455) 客户端
 *
 * 覆盖内容（不依赖任何第三方库）：
 *   1) HTTP/1.1 Upgrade 握手请求构造（含 Sec-WebSocket-Key 随机数 + base64）
 *   2) 101 Switching Protocols 响应解析 + Sec-WebSocket-Accept 校验（自研 SHA-1）
 *   3) RFC6455 数据帧/控制帧编解码，客户端发出的帧一律掩码（MASK=1）
 *   4) ping/pong 保活、close 握手、分片（continuation）重组
 *   5) 断线检测 + 指数退避重连
 */
#ifndef WS_CLIENT_H
#define WS_CLIENT_H

#include <stdint.h>
#include <stddef.h>
#include "band_config.h"
#include "net_transport.h"

typedef enum {
    WS_ST_CLOSED = 0,      /* 未连接 */
    WS_ST_CONNECTING,      /* TCP 连接中 */
    WS_ST_HANDSHAKE,       /* 已发 Upgrade，等 101 */
    WS_ST_OPEN,            /* 握手完成，可收发 */
    WS_ST_CLOSING,         /* 已发 close，等对端 close */
    WS_ST_ERROR            /* 出错，待重连 */
} ws_state_t;

typedef struct {
    char     host[64];
    uint16_t port;
    char     path[64];
    uint8_t  entropy[WS_KEY_LEN];  /* 握手随机数（测试可注入以对齐 RFC 向量） */
    uint8_t  entropy_set;
} ws_cfg_t;

/* 收到一个完整数据消息时回调（opcode 为 WS_OP_TEXT/BINARY，payload 已解掩码） */
typedef void (*ws_msg_cb_t)(void *user, uint8_t opcode, const uint8_t *payload, size_t len);
/* 连接状态变化回调 */
typedef void (*ws_state_cb_t)(void *user, ws_state_t old_st, ws_state_t new_st);

/* RFC6455 帧头 */
typedef struct {
    uint8_t  fin;
    uint8_t  rsv;
    uint8_t  opcode;
    uint8_t  masked;
    uint8_t  mask[4];
    uint64_t payload_len;
    size_t   header_len;   /* 含扩展长度与掩码键 */
} ws_frame_hdr_t;

typedef struct {
    ws_state_t       state;
    ws_cfg_t         cfg;
    net_transport_t *tp;
    ws_msg_cb_t      on_msg;
    void            *user;
    ws_state_cb_t    on_state;
    uint8_t          auto_reconnect;

    /* 握手缓冲 */
    char    hs_rx[WS_HANDSHAKE_BUF];
    size_t  hs_rx_len;
    char    hs_key_b64[WS_KEY_B64_LEN + 1];
    char    hs_accept[WS_ACCEPT_LEN + 1];

    /* TCP 接收缓冲 */
    uint8_t rx[WS_RX_BUF];
    size_t  rx_len;

    /* 分片重组 */
    uint8_t  frag[WS_MAX_FRAME_PAYLOAD];
    size_t   frag_len;
    uint8_t  frag_opcode;

    /* 计时 */
    uint32_t last_rx_ms;
    uint32_t last_tx_ms;
    uint32_t ping_sent_ms;
    uint8_t  ping_pending;

    /* 重连退避 */
    uint32_t backoff_ms;
    uint32_t next_retry_ms;
    uint8_t  retry_count;

    /* 统计 */
    uint32_t tx_frames;
    uint32_t rx_frames;
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t ping_tx;
    uint32_t pong_rx;
    uint32_t close_rx;
    uint32_t handshake_ok;
    uint32_t handshake_fail;
    uint32_t protocol_errors;
    uint32_t reconnect_count;
    uint32_t decoded_msgs;
} ws_client_t;

/* ---------------- 生命周期 ---------------- */
void ws_client_init(ws_client_t *ws, net_transport_t *tp);
void ws_client_set_callbacks(ws_client_t *ws, ws_msg_cb_t on_msg, void *user, ws_state_cb_t on_state);
/* 为测试注入固定随机数（RFC6455 §1.3 向量）；正常运行时用 band_entropy 生成 */
void ws_client_set_entropy(ws_client_t *ws, const uint8_t *entropy, size_t len);
int  ws_connect(ws_client_t *ws, const char *host, uint16_t port, const char *path, uint32_t timeout_ms);
/* 驱动状态机：处理握手、收包、协议帧、保活、退避重连。返回已处理的字节数 */
int  ws_poll(ws_client_t *ws, uint32_t timeout_ms);
void ws_close(ws_client_t *ws, uint16_t code, const char *reason);
void ws_client_reset_stats(ws_client_t *ws);

/* ---------------- 收发 ---------------- */
int  ws_send_binary(ws_client_t *ws, const uint8_t *data, size_t len);
int  ws_send_text(ws_client_t *ws, const char *text);
int  ws_send_ping(ws_client_t *ws, const uint8_t *payload, size_t len);
int  ws_send_pong(ws_client_t *ws, const uint8_t *payload, size_t len);
/* 自底向上的单帧编解码，测试可直接调用 */
size_t ws_encode_frame(uint8_t opcode, uint8_t fin, const uint8_t *payload, size_t len,
                       const uint8_t mask_key[4], uint8_t *out, size_t out_cap);
/* 解析一个帧头（不做解掩码）。返回：0 成功，1 数据不足，-1 协议错误 */
int    ws_parse_frame_header(const uint8_t *buf, size_t len, ws_frame_hdr_t *hdr);
/* 解析一个完整帧并把载荷解掩码到 payload_out。
 * 返回：0 成功，1 数据不足，-1 协议错误（含服务端错误掩码/控制帧超长等情况） */
int    ws_decode_frame(const uint8_t *buf, size_t len, ws_frame_hdr_t *hdr,
                       uint8_t *payload_out, size_t payload_cap, size_t *consumed);
/* 整帧入参的一步到位版本（内部带 1KB 静态缓冲，见实现说明） */
int    ws_decode_frame_simple(const uint8_t *buf, size_t len, uint8_t *opcode, uint8_t *fin,
                              uint8_t *masked, uint8_t mask_key[4],
                              const uint8_t **payload, size_t *payload_len, size_t *consumed);

/* 握手要素（测试直接验证） */
void ws_build_key(char out_b64[WS_KEY_B64_LEN + 1], const uint8_t entropy[WS_KEY_LEN]);
int  ws_compute_accept(const char *key_b64, char out[WS_ACCEPT_LEN + 1]);
int  ws_build_handshake(const char *host, uint16_t port, const char *path,
                        const char *key_b64, char *out, size_t out_cap);
/* 解析 101 响应；返回 0 成功，<0 失败（错误码见实现） */
int  ws_parse_handshake_response(const char *resp, size_t len,
                                 const char *key_b64, char accept_out[WS_ACCEPT_LEN + 1]);

/* base64 / SHA1 原语（自研实现，测试向量覆盖） */
size_t band_base64_encode(const uint8_t *src, size_t src_len, char *out, size_t out_cap);
int    band_base64_decode(const char *src, size_t src_len, uint8_t *out, size_t out_cap);
void   band_sha1(const uint8_t *data, size_t len, uint8_t digest[20]);

const char *ws_state_name(ws_state_t st);

/* 断线重连调用（内部由 ws_poll 触发，也可外部强制） */
int  ws_reconnect(ws_client_t *ws, uint32_t timeout_ms);

#endif /* WS_CLIENT_H */
