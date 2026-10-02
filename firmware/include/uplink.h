/*
 * uplink.h -- 数据上行链路：应用帧 <-> WebSocket <-> 手机 App
 *
 * 职责：
 *   1) 把体征帧经 WebSocket 二进制消息发给 App/服务器；
 *   2) 在线时带序号直发；离线时把整帧塞进离线环形缓存；
 *   3) 重连后按序号升序补传，同一会话内同序号只发一次；
 *   4) 解析对端下行帧：ACK / 远程参数下发 / 时间同步 / ping；
 *   5) 断线检测与指数退避重连（由 ws_client 提供底层能力）。
 *
 * 为什么把整帧（而不是裸载荷）放进缓存：补传时不需要重新组帧，
 * 序号与校验和保持与首次发送完全一致，App 端可以直接按序号去重。
 */
#ifndef UPLINK_H
#define UPLINK_H

#include <stdint.h>
#include <stddef.h>
#include "band_config.h"
#include "frame_codec.h"
#include "ring_buffer.h"
#include "ws_client.h"

/* 远程可下发的运行参数 */
typedef struct {
    uint16_t hr_interval_ms;      /* 心率采样周期 */
    uint8_t  step_sensitivity;    /* 计步灵敏度 0..100 */
    uint16_t upload_interval_ms;  /* 上传周期 */
    uint16_t sleep_timeout_s;     /* 自动休眠超时（秒） */
    uint8_t  lcd_brightness;      /* 背光 0..100 */
    uint8_t  motor_enable;        /* 振动马达开关 */
    uint32_t param_version;       /* 每次下发递增，便于确认生效 */
    char     device_name[24];
    char     wifi_ssid[32];
    char     wifi_pass[32];
} band_params_t;

/* 链路统计，用于自检与现场诊断 */
typedef struct {
    uint32_t sent_online;     /* 在线直发帧数 */
    uint32_t cached_offline;  /* 离线缓存帧数 */
    uint32_t resent;          /* 补传帧数 */
    uint32_t acks_rx;
    uint32_t nacks_rx;
    uint32_t dup_acks;        /* 重复 ACK（已确认过的序号又收到一次） */
    uint32_t params_applied;
    uint32_t param_errors;
    uint32_t time_syncs;
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t reconnects;
    uint32_t encode_errors;
} uplink_stats_t;

typedef void (*uplink_param_cb_t)(void *user, const band_params_t *params);

typedef struct {
    ws_client_t     ws;
    resend_ctx_t    offline;
    frame_decoder_t dec;          /* 一条 WS 消息内可能半包/粘包，统一走状态机 */
    band_params_t   params;
    uint16_t        tx_seq;
    uint16_t        last_acked_seq;
    uint8_t         link_up;
    uint8_t         auto_resend;  /* 重连后自动补传 */
    uint8_t         connected_once;
    uint8_t         verbose;      /* 1 = 打印每一帧的收发明细（现场联调用） */
    uint32_t        last_poll_ms;
    uint32_t        last_conn_attempt_ms;
    net_transport_t *tp;
    uplink_stats_t  st;
    uplink_param_cb_t on_param;
    void           *user;
} band_uplink_t;

/* ---------------- 生命周期 ---------------- */
void uplink_init(band_uplink_t *u, net_transport_t *tp,
                 const char *host, uint16_t port, const char *path);
void uplink_set_param_callback(band_uplink_t *u, uplink_param_cb_t cb, void *user);
void uplink_default_params(band_params_t *p);
int  uplink_connect(band_uplink_t *u, uint32_t timeout_ms);
void uplink_disconnect(band_uplink_t *u);
void uplink_reset_stats(band_uplink_t *u);

/* 驱动链路状态机：握手/收包/保活/重连/补传。now_ms 由上层传入便于测试 */
void uplink_poll(band_uplink_t *u, uint32_t now_ms);

/* ---------------- 发送 ---------------- */
/* 发送一个应用帧：在线直发，离线自动入缓存。返回 0 成功（发出或已缓存），<0 失败。
 * out_seq 可空，回传该帧序号 */
int uplink_send(band_uplink_t *u, uint8_t type, const uint8_t *payload, uint16_t len,
                uint16_t *out_seq);
/* 发送一帧已编码好的原始帧（补传路径使用） */
int uplink_send_raw_frame(band_uplink_t *u, const uint8_t *raw, size_t raw_len);

/* ---------------- 补传 ---------------- */
/* 开始一轮补传会话（重连成功后调用） */
void uplink_resend_begin(band_uplink_t *u);
/* 连发最多 burst 条待补传记录，返回实际发出条数 */
uint32_t uplink_resend_burst(band_uplink_t *u, uint32_t burst);
uint16_t uplink_pending_count(const band_uplink_t *u);

/* ---------------- 下行帧处理（供测试直接调用） ---------------- */
void uplink_handle_frame(band_uplink_t *u, const frame_t *f);
int  uplink_apply_params(band_uplink_t *u, const uint8_t *tlv, uint16_t len);
/* 把当前参数编码成 TLV（用于回给 App 做确认） */
size_t uplink_encode_params(const band_params_t *p, uint8_t *out, size_t cap);

const char *uplink_state_str(const band_uplink_t *u);

#endif /* UPLINK_H */
