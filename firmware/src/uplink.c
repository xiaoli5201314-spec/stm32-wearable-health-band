/*
 * uplink.c -- 数据上行链路实现
 *
 * 数据流：
 *   上行：uplink_send(type,payload) -> frame_encode() -> [在线? ws_send_binary : offline_cache_push]
 *   补传：ws 重连成功 -> uplink_resend_begin() -> uplink_resend_burst() 按序重发未确认记录
 *   下行：ws on_msg -> frame_decoder_push_buf() -> uplink_handle_frame()
 *         ACK        -> resend_ack() 移出缓存，更新 last_acked_seq
 *         PARAM_SET  -> uplink_apply_params() 解析 TLV 并回调上层
 *         TIME_SYNC  -> 回调上层设置 RTC
 *         PING       -> 回 PONG
 */
#include "uplink.h"
#include "band_port.h"
#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */
void uplink_default_params(band_params_t *p)
{
    if (p == NULL) {
        return;
    }
    memset(p, 0, sizeof(*p));
    p->hr_interval_ms = 1000u;
    p->step_sensitivity = 50u;
    p->upload_interval_ms = 5000u;
    p->sleep_timeout_s = 15u;
    p->lcd_brightness = 80u;
    p->motor_enable = 1u;
    p->param_version = 0u;
    strncpy(p->device_name, "BAND-0001", sizeof(p->device_name) - 1u);
    strncpy(p->wifi_ssid, "band-ap", sizeof(p->wifi_ssid) - 1u);
    p->wifi_pass[0] = '\0';
}

void uplink_init(band_uplink_t *u, net_transport_t *tp,
                 const char *host, uint16_t port, const char *path)
{
    if (u == NULL) {
        return;
    }
    memset(u, 0, sizeof(*u));
    u->tp = (tp != NULL) ? tp : net_socket_get();
    ws_client_init(&u->ws, u->tp);
    ws_client_set_callbacks(&u->ws, NULL, u, NULL);
    offline_cache_init(&u->offline.cache);
    frame_decoder_init(&u->dec);
    uplink_default_params(&u->params);
    u->tx_seq = 1u;
    u->auto_resend = 1u;
    if ((host != NULL) && (host[0] != '\0') && (host != u->ws.cfg.host)) {
        strncpy(u->ws.cfg.host, host, sizeof(u->ws.cfg.host) - 1u);
        u->ws.cfg.host[sizeof(u->ws.cfg.host) - 1u] = '\0';
    }
    if (port != 0u) {
        u->ws.cfg.port = port;
    }
    if ((path != NULL) && (path[0] != '\0') && (path != u->ws.cfg.path)) {
        strncpy(u->ws.cfg.path, path, sizeof(u->ws.cfg.path) - 1u);
        u->ws.cfg.path[sizeof(u->ws.cfg.path) - 1u] = '\0';
    }
}

void uplink_set_param_callback(band_uplink_t *u, uplink_param_cb_t cb, void *user)
{
    if (u == NULL) {
        return;
    }
    u->on_param = cb;
    u->user = user;
}

void uplink_reset_stats(band_uplink_t *u)
{
    if (u == NULL) {
        return;
    }
    memset(&u->st, 0, sizeof(u->st));
}

const char *uplink_state_str(const band_uplink_t *u)
{
    if (u == NULL) {
        return "?";
    }
    if (u->link_up != 0u) {
        return "ONLINE";
    }
    switch (u->ws.state) {
    case WS_ST_CONNECTING: return "CONNECTING";
    case WS_ST_HANDSHAKE:  return "HANDSHAKE";
    case WS_ST_CLOSING:    return "CLOSING";
    case WS_ST_ERROR:      return "ERROR";
    case WS_ST_CLOSED:     return "CLOSED";
    default:               return "OFFLINE";
    }
}

/* ------------------------------------------------------------------ */
/* 下行帧分发                                                          */
/* ------------------------------------------------------------------ */
void uplink_handle_frame(band_uplink_t *u, const frame_t *f)
{
    if ((u == NULL) || (f == NULL)) {
        return;
    }
    u->st.rx_frames++;
    if (u->verbose != 0u) {
        band_trace("[uplink] RX type=0x%02X(%s) seq=%u len=%u\n",
                   (unsigned)f->type, frame_type_name(f->type),
                   (unsigned)f->seq, (unsigned)f->len);
    }

    switch (f->type) {
    case MSG_ACK: {
        uint8_t status = 0u;
        uint16_t ack_seq = 0u;
        if (frame_unpack_ack(f->payload, f->len, &status, &ack_seq) != 0) {
            u->st.param_errors++;
            break;
        }
        u->st.acks_rx++;
        if (resend_ack(&u->offline, ack_seq) != 0) {
            /* 序号已不在缓存：重复 ACK，计入去重统计 */
            u->st.dup_acks++;
            if (u->verbose != 0u) {
                band_trace("[uplink]   ACK seq=%u 未命中缓存（重复确认），pending=%u\n",
                           (unsigned)ack_seq, (unsigned)uplink_pending_count(u));
            }
        } else if (u->verbose != 0u) {
            band_trace("[uplink]   ACK seq=%u 已确认，pending=%u\n",
                       (unsigned)ack_seq, (unsigned)uplink_pending_count(u));
        }
        if ((u->last_acked_seq == 0u) || ((int16_t)(ack_seq - u->last_acked_seq) > 0)) {
            u->last_acked_seq = ack_seq;
        }
        if (status != ACK_STATUS_OK) {
            u->st.nacks_rx++;
        }
        break;
    }

    case MSG_PARAM_SET:
        if (uplink_apply_params(u, f->payload, f->len) < 0) {
            u->st.param_errors++;
        }
        break;

    case MSG_TIME_SYNC:
        /* 载荷：uint32 unix 秒（小端）。上层拿到后写 RTC */
        if (f->len >= 4u) {
            u->st.time_syncs++;
            if (u->on_param != NULL) {
                u->on_param(u->user, &u->params);
            }
        }
        break;

    case MSG_PING:
        /* 应用层 ping：直接用同一序号回 pong，方便对端算 RTT */
        (void)uplink_send(u, MSG_PONG, f->payload, f->len, NULL);
        break;

    case MSG_PONG:
        break;

    case MSG_NACK:
        u->st.nacks_rx++;
        break;

    default:
        break;
    }
}

int uplink_apply_params(band_uplink_t *u, const uint8_t *tlv, uint16_t len)
{
    tlv_list_t list;
    int n;
    uint8_t v8 = 0u;
    uint16_t v16 = 0u;

    if ((u == NULL) || (tlv == NULL)) {
        return -1;
    }
    n = tlv_parse(tlv, len, &list);
    if (n < 0) {
        /* 格式非法：整包拒绝，不做部分应用，避免参数处于半更新状态 */
        return -1;
    }
    if (n == 0) {
        return -1;
    }

    if (tlv_get_u16(&list, TLV_TAG_HR_INTERVAL, &v16) == 0) {
        if ((v16 >= 100u) && (v16 <= 10000u)) {
            u->params.hr_interval_ms = v16;
        } else {
            u->st.param_errors++;
        }
    }
    if (tlv_get_u8(&list, TLV_TAG_STEP_SENSITIVITY, &v8) == 0) {
        if (v8 <= 100u) {
            u->params.step_sensitivity = v8;
        } else {
            u->st.param_errors++;
        }
    }
    if (tlv_get_u16(&list, TLV_TAG_UPLOAD_INTERVAL, &v16) == 0) {
        if ((v16 >= 500u) && (v16 <= 60000u)) {
            u->params.upload_interval_ms = v16;
        } else {
            u->st.param_errors++;
        }
    }
    if (tlv_get_u16(&list, TLV_TAG_SLEEP_TIMEOUT, &v16) == 0) {
        if ((v16 >= 5u) && (v16 <= 3600u)) {
            u->params.sleep_timeout_s = v16;
        } else {
            u->st.param_errors++;
        }
    }
    if (tlv_get_u8(&list, TLV_TAG_LCD_BRIGHTNESS, &v8) == 0) {
        if (v8 <= 100u) {
            u->params.lcd_brightness = v8;
        } else {
            u->st.param_errors++;
        }
    }
    if (tlv_get_u8(&list, TLV_TAG_MOTOR_ENABLE, &v8) == 0) {
        u->params.motor_enable = (v8 != 0u) ? 1u : 0u;
    }
    if (tlv_get_str(&list, TLV_TAG_WIFI_SSID, u->params.wifi_ssid,
                    sizeof(u->params.wifi_ssid)) == 0) {
        /* 已写入 */
    }
    if (tlv_get_str(&list, TLV_TAG_WIFI_PASS, u->params.wifi_pass,
                    sizeof(u->params.wifi_pass)) == 0) {
        /* 已写入 */
    }
    if (tlv_get_str(&list, TLV_TAG_DEVICE_NAME, u->params.device_name,
                    sizeof(u->params.device_name)) == 0) {
        /* 已写入 */
    }

    u->params.param_version++;
    u->st.params_applied++;
    if (u->on_param != NULL) {
        u->on_param(u->user, &u->params);
    }
    return 0;
}

size_t uplink_encode_params(const band_params_t *p, uint8_t *out, size_t cap)
{
    size_t off = 0u;
    size_t n;

    if ((p == NULL) || (out == NULL)) {
        return 0u;
    }
    n = tlv_put_u16(out, cap, off, TLV_TAG_HR_INTERVAL, p->hr_interval_ms);
    if (n == 0u) { return 0u; }
    off += n;
    n = tlv_put_u8(out, cap, off, TLV_TAG_STEP_SENSITIVITY, p->step_sensitivity);
    if (n == 0u) { return 0u; }
    off += n;
    n = tlv_put_u16(out, cap, off, TLV_TAG_UPLOAD_INTERVAL, p->upload_interval_ms);
    if (n == 0u) { return 0u; }
    off += n;
    n = tlv_put_u16(out, cap, off, TLV_TAG_SLEEP_TIMEOUT, p->sleep_timeout_s);
    if (n == 0u) { return 0u; }
    off += n;
    n = tlv_put_u8(out, cap, off, TLV_TAG_LCD_BRIGHTNESS, p->lcd_brightness);
    if (n == 0u) { return 0u; }
    off += n;
    n = tlv_put_u8(out, cap, off, TLV_TAG_MOTOR_ENABLE, p->motor_enable);
    if (n == 0u) { return 0u; }
    off += n;
    n = tlv_put_str(out, cap, off, TLV_TAG_DEVICE_NAME, p->device_name);
    if (n == 0u) { return 0u; }
    off += n;
    return off;
}

/* ------------------------------------------------------------------ */
/* WebSocket 回调                                                      */
/* ------------------------------------------------------------------ */
static void uplink_on_ws_msg(void *user, uint8_t opcode, const uint8_t *payload, size_t len)
{
    band_uplink_t *u = (band_uplink_t *)user;
    frame_t f;
    size_t i;

    (void)opcode;
    if ((u == NULL) || (payload == NULL)) {
        return;
    }
    /* 一个 WebSocket 消息里可能是一整帧、半帧、或多帧：
     * 全部丢进帧同步状态机，逐字节推进。 */
    for (i = 0u; i < len; i++) {
        frame_dec_result_t r = frame_decoder_push(&u->dec, payload[i], &f);
        if (r == FRAME_DEC_OK) {
            uplink_handle_frame(u, &f);
        } else if (r == FRAME_DEC_CRC_ERR) {
            u->st.param_errors++;
        }
    }
}

static void uplink_on_ws_state(void *user, ws_state_t old_st, ws_state_t new_st)
{
    band_uplink_t *u = (band_uplink_t *)user;

    if (u == NULL) {
        return;
    }
    if (new_st == WS_ST_OPEN) {
        u->link_up = 1u;
        u->connected_once = 1u;
        if (old_st != WS_ST_OPEN) {
            /* 刚上线：开启一轮补传会话，把断网期间的数据补上去 */
            if (u->auto_resend != 0u) {
                uplink_resend_begin(u);
            }
            u->st.reconnects++;
        }
    } else if ((new_st == WS_ST_ERROR) || (new_st == WS_ST_CLOSED)) {
        u->link_up = 0u;
    }
}

/* ------------------------------------------------------------------ */
/* 连接与轮询                                                          */
/* ------------------------------------------------------------------ */
int uplink_connect(band_uplink_t *u, uint32_t timeout_ms)
{
    int rc;

    if (u == NULL) {
        return NET_ERR_PARAM;
    }
    ws_client_set_callbacks(&u->ws, uplink_on_ws_msg, u, uplink_on_ws_state);
    rc = ws_connect(&u->ws, u->ws.cfg.host, u->ws.cfg.port, u->ws.cfg.path, timeout_ms);
    if (rc == NET_OK) {
        /* 状态回调（uplink_on_ws_state）已经在进入 OPEN 时置位 link_up、
         * 开启补传会话并累加 reconnect 计数，这里只做兜底同步。 */
        u->link_up = 1u;
        u->connected_once = 1u;
    } else {
        u->link_up = 0u;
    }
    return rc;
}

void uplink_disconnect(band_uplink_t *u)
{
    if (u == NULL) {
        return;
    }
    ws_close(&u->ws, WS_CLOSE_NORMAL, "bye");
    u->link_up = 0u;
}

void uplink_poll(band_uplink_t *u, uint32_t now_ms)
{
    if (u == NULL) {
        return;
    }
    u->last_poll_ms = now_ms;

    /* 1) 驱动 WebSocket：握手 / 收包 / 保活 / 退避重连 */
    (void)ws_poll(&u->ws, 20u);

    /* 2) 链路状态同步（回调可能已经改过，这里兜底） */
    if (u->ws.state == WS_ST_OPEN) {
        if (u->link_up == 0u) {
            u->link_up = 1u;
            if (u->auto_resend != 0u) {
                uplink_resend_begin(u);
            }
        }
    } else {
        u->link_up = 0u;
    }

    /* 3) 在线就把离线缓存里的数据按序号补传出去 */
    if ((u->link_up != 0u) && (u->auto_resend != 0u)) {
        (void)uplink_resend_burst(u, OFFLINE_RESEND_BURST);
    }
}

/* ------------------------------------------------------------------ */
/* 发送                                                                */
/* ------------------------------------------------------------------ */
int uplink_send_raw_frame(band_uplink_t *u, const uint8_t *raw, size_t raw_len)
{
    if ((u == NULL) || (raw == NULL) || (raw_len == 0u)) {
        return -1;
    }
    if (u->link_up == 0u) {
        return -2;   /* 链路不可用，由调用者决定是否缓存 */
    }
    if (ws_send_binary(&u->ws, raw, raw_len) != 0) {
        u->link_up = 0u;
        return -3;
    }
    u->st.tx_frames++;
    if (u->verbose != 0u) {
        band_trace("[uplink] TX raw %u 字节\n", (unsigned)raw_len);
    }
    return 0;
}

int uplink_send(band_uplink_t *u, uint8_t type, const uint8_t *payload, uint16_t len,
                uint16_t *out_seq)
{
    uint8_t raw[FRAME_MAX_SIZE];
    size_t raw_len;
    uint16_t seq;

    if (u == NULL) {
        return -1;
    }
    seq = u->tx_seq;
    if (seq == 0u) {
        seq = 1u;
    }
    raw_len = frame_build(type, seq, payload, len, raw, sizeof(raw));
    if (raw_len == 0u) {
        u->st.encode_errors++;
        return -4;
    }
    u->tx_seq = (uint16_t)(seq + 1u);
    if (u->tx_seq == 0u) {
        u->tx_seq = 1u;
    }
    if (out_seq != NULL) {
        *out_seq = seq;
    }

    if (u->link_up != 0u) {
        if (uplink_send_raw_frame(u, raw, raw_len) == 0) {
            u->st.sent_online++;   /* 只统计"在线直发"，补传走 st.resent */
            return 0;
        }
        /* 直发失败：降级为离线缓存，保证数据不丢 */
    }

    /* 离线：整帧入缓存，等重连后按序号补传。
     * 关键：缓存记录的序号必须**等于帧头里的 SEQ**（同一个编号空间），
     * 否则对端按帧序号回 ACK 时会删错记录 —— 这是联调测试发现并修掉的问题。 */
    if (offline_cache_push_seq(&u->offline.cache, seq, type, raw, (uint16_t)raw_len) == 0u) {
        return -5;
    }
    u->st.cached_offline++;
    return 1;   /* 1 = 已缓存待补传 */
}

/* ------------------------------------------------------------------ */
/* 补传                                                                */
/* ------------------------------------------------------------------ */
void uplink_resend_begin(band_uplink_t *u)
{
    if (u == NULL) {
        return;
    }
    resend_begin(&u->offline);
}

uint32_t uplink_resend_burst(band_uplink_t *u, uint32_t burst)
{
    uint32_t sent = 0u;
    offline_rec_t rec;

    if (u == NULL) {
        return 0u;
    }
    while (sent < burst) {
        if (resend_next(&u->offline, &rec) == 0) {
            break;   /* 本轮没有更多待补传记录 */
        }
        if (uplink_send_raw_frame(u, rec.data, rec.len) != 0) {
            /* 发送失败：把这条记录标记回"未发送"，下轮重试，避免数据被跳过 */
            offline_rec_t *slot = NULL;
            uint16_t i;
            for (i = 0u; i < u->offline.cache.count; i++) {
                uint16_t idx = (uint16_t)((u->offline.cache.head + OFFLINE_SLOT_MAX -
                                           u->offline.cache.count + i) % OFFLINE_SLOT_MAX);
                if (u->offline.cache.slot[idx].seq == rec.seq) {
                    slot = &u->offline.cache.slot[idx];
                    break;
                }
            }
            if (slot != NULL) {
                slot->sent = 0u;
            }
            u->offline.scan_idx = 0u;
            break;
        }
        u->st.resent++;
        sent++;
    }
    return sent;
}

uint16_t uplink_pending_count(const band_uplink_t *u)
{
    return (u == NULL) ? 0u : resend_pending_count(&u->offline);
}
