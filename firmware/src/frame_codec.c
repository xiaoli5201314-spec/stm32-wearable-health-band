/*
 * frame_codec.c -- 自定义应用层帧编解码 + 帧同步状态机 + 载荷编解码
 *
 * 帧格式（小端）：
 *   +--------+--------+--------+--------+---------+--------+
 *   | 0xAA   | 0x55   | TYPE   | SEQ    | LEN     | PAYLOAD| CRC16 |
 *   | SOF0   | SOF1   | 1B     | 2B LE  | 2B LE   | LEN B  | 2B LE |
 *   +--------+--------+--------+--------+---------+--------+
 *   CRC16/CCITT-FALSE (poly 0x1021, init 0xFFFF) 覆盖 TYPE..PAYLOAD 末字节
 *
 * 状态机设计要点：
 *   - 逐字节推进，天然支持"半包"（返回 FRAME_DEC_NONE 保留中间状态）
 *   - 一次调用只产出一帧，调用方循环喂入剩余字节即可处理"粘包"
 *   - 载荷长度超限立即报错并回到 SOF0 搜索，避免被畸形长度卡死
 *   - CRC 错只丢弃当前帧，不丢弃缓冲区，因为后续字节可能已经是新帧头
 */
#include "frame_codec.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/* CRC16/CCITT-FALSE：poly=0x1021 init=0xFFFF refin=false refout=false */
/* ------------------------------------------------------------------ */
uint16_t frame_crc16_update(uint16_t crc, uint8_t byte)
{
    uint8_t i;

    crc ^= (uint16_t)((uint16_t)byte << 8);
    for (i = 0u; i < 8u; i++) {
        if ((crc & 0x8000u) != 0u) {
            crc = (uint16_t)(((uint16_t)(crc << 1)) ^ FRAME_CRC_POLY);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t frame_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = FRAME_CRC_INIT;
    size_t i;

    if (data == NULL) {
        return crc;
    }
    for (i = 0u; i < len; i++) {
        crc = frame_crc16_update(crc, data[i]);
    }
    return crc;
}

/* ------------------------------------------------------------------ */
/* 编码                                                                */
/* ------------------------------------------------------------------ */
size_t frame_encode(const frame_t *f, uint8_t *out, size_t out_cap)
{
    size_t total;
    uint16_t crc;
    size_t i;

    if ((f == NULL) || (out == NULL)) {
        return 0u;
    }
    if (f->len > FRAME_MAX_PAYLOAD) {
        return 0u;
    }
    total = FRAME_OVERHEAD + (size_t)f->len;
    if (out_cap < total) {
        return 0u;
    }

    out[0] = FRAME_SOF0;
    out[1] = FRAME_SOF1;
    out[2] = f->type;
    out[3] = (uint8_t)(f->seq & 0xFFu);
    out[4] = (uint8_t)((f->seq >> 8) & 0xFFu);
    out[5] = (uint8_t)(f->len & 0xFFu);
    out[6] = (uint8_t)((f->len >> 8) & 0xFFu);
    for (i = 0u; i < (size_t)f->len; i++) {
        out[FRAME_HEADER_LEN + i] = f->payload[i];
    }

    /* CRC 覆盖 TYPE..PAYLOAD，即 out[2] 起 (1+2+2+len) 个字节 */
    crc = frame_crc16(&out[2], (size_t)(5u + (size_t)f->len));
    out[FRAME_HEADER_LEN + f->len] = (uint8_t)(crc & 0xFFu);
    out[FRAME_HEADER_LEN + f->len + 1u] = (uint8_t)((crc >> 8) & 0xFFu);
    return total;
}

size_t frame_build(uint8_t type, uint16_t seq, const uint8_t *payload,
                   uint16_t len, uint8_t *out, size_t out_cap)
{
    frame_t f;

    if (len > FRAME_MAX_PAYLOAD) {
        return 0u;
    }
    f.type = type;
    f.seq = seq;
    f.len = len;
    if ((payload != NULL) && (len > 0u)) {
        memcpy(f.payload, payload, (size_t)len);
    }
    return frame_encode(&f, out, out_cap);
}

/* ------------------------------------------------------------------ */
/* 解码：帧同步状态机                                                   */
/* ------------------------------------------------------------------ */
void frame_decoder_init(frame_decoder_t *d)
{
    if (d == NULL) {
        return;
    }
    memset(d, 0, sizeof(*d));
    d->state = FRAME_ST_SOF0;
}

/* 回到搜索帧头的状态；若当前字节可能是新 SOF0 则保留 */
static void frame_decoder_resync(frame_decoder_t *d, uint8_t byte)
{
    d->state = (byte == FRAME_SOF0) ? FRAME_ST_SOF1 : FRAME_ST_SOF0;
    d->idx = 0u;
    d->crc_calc = FRAME_CRC_INIT;
    d->resyncs++;
}

frame_dec_result_t frame_decoder_push(frame_decoder_t *d, uint8_t byte, frame_t *out)
{
    if (d == NULL) {
        return FRAME_DEC_NONE;
    }
    d->bytes_in++;

    switch (d->state) {
    case FRAME_ST_SOF0:
        if (byte == FRAME_SOF0) {
            d->state = FRAME_ST_SOF1;
        }
        break;

    case FRAME_ST_SOF1:
        if (byte == FRAME_SOF1) {
            d->state = FRAME_ST_TYPE;
        } else if (byte == FRAME_SOF0) {
            /* 0xAA 0xAA... 后一个 0xAA 可能是新帧头，停在 SOF1 */
            d->state = FRAME_ST_SOF1;
        } else {
            d->state = FRAME_ST_SOF0;
            d->resyncs++;
        }
        break;

    case FRAME_ST_TYPE:
        d->type = byte;
        d->crc_calc = frame_crc16_update(FRAME_CRC_INIT, byte);
        d->state = FRAME_ST_SEQ_LO;
        break;

    case FRAME_ST_SEQ_LO:
        d->seq = byte;
        d->crc_calc = frame_crc16_update(d->crc_calc, byte);
        d->state = FRAME_ST_SEQ_HI;
        break;

    case FRAME_ST_SEQ_HI:
        d->seq = (uint16_t)(d->seq | ((uint16_t)byte << 8));
        d->crc_calc = frame_crc16_update(d->crc_calc, byte);
        d->state = FRAME_ST_LEN_LO;
        break;

    case FRAME_ST_LEN_LO:
        d->len = byte;
        d->crc_calc = frame_crc16_update(d->crc_calc, byte);
        d->state = FRAME_ST_LEN_HI;
        break;

    case FRAME_ST_LEN_HI:
        d->len = (uint16_t)(d->len | ((uint16_t)byte << 8));
        d->crc_calc = frame_crc16_update(d->crc_calc, byte);
        if (d->len > FRAME_MAX_PAYLOAD) {
            /* 长度非法：不可能是一帧，回到搜索态 */
            d->len_errors++;
            frame_decoder_resync(d, byte);
            return FRAME_DEC_LEN_ERR;
        }
        if (d->len == 0u) {
            d->state = FRAME_ST_CRC_LO;
        } else {
            d->idx = 0u;
            d->state = FRAME_ST_PAYLOAD;
        }
        break;

    case FRAME_ST_PAYLOAD:
        d->payload[d->idx] = byte;
        d->idx = (uint16_t)(d->idx + 1u);
        d->crc_calc = frame_crc16_update(d->crc_calc, byte);
        if (d->idx >= d->len) {
            d->state = FRAME_ST_CRC_LO;
        }
        break;

    case FRAME_ST_CRC_LO:
        d->crc_rx = byte;
        d->state = FRAME_ST_CRC_HI;
        break;

    case FRAME_ST_CRC_HI:
        d->crc_rx = (uint16_t)(d->crc_rx | ((uint16_t)byte << 8));
        d->state = FRAME_ST_SOF0;
        if (d->crc_rx != d->crc_calc) {
            d->crc_errors++;
            d->idx = 0u;
            d->crc_calc = FRAME_CRC_INIT;
            return FRAME_DEC_CRC_ERR;
        }
        if (out != NULL) {
            out->type = d->type;
            out->seq = d->seq;
            out->len = d->len;
            if (d->len > 0u) {
                memcpy(out->payload, d->payload, (size_t)d->len);
            }
        }
        d->frames_ok++;
        d->idx = 0u;
        d->crc_calc = FRAME_CRC_INIT;
        return FRAME_DEC_OK;

    default:
        d->state = FRAME_ST_SOF0;
        break;
    }
    return FRAME_DEC_NONE;
}

size_t frame_decoder_push_buf(frame_decoder_t *d, const uint8_t *buf, size_t len,
                              frame_t *out, size_t out_cap, size_t *consumed,
                              uint32_t *crc_errors)
{
    size_t i;
    size_t n = 0u;
    uint32_t crc_err = 0u;

    if ((d == NULL) || (buf == NULL)) {
        if (consumed != NULL) {
            *consumed = 0u;
        }
        return 0u;
    }
    for (i = 0u; i < len; i++) {
        frame_dec_result_t r = frame_decoder_push(d, buf[i], (n < out_cap) ? &out[n] : NULL);
        if (r == FRAME_DEC_OK) {
            n++;
        } else if (r == FRAME_DEC_CRC_ERR) {
            crc_err++;
        } else {
            /* 继续 */
        }
    }
    if (consumed != NULL) {
        *consumed = len;   /* 逐字节状态机总是消耗完全部字节 */
    }
    if (crc_errors != NULL) {
        *crc_errors = crc_err;
    }
    return n;
}

/* ------------------------------------------------------------------ */
/* 载荷编解码                                                          */
/* ------------------------------------------------------------------ */
/* 聚合体征包布局（小端）：
 *   0  heart_rate   u8
 *   1  confidence   u8
 *   2  rr_ms        u16
 *   4  temp_c100    i16
 *   6  steps        u16
 *   8  battery_mv   u16
 *   10 battery_pct  u8
 *   11 roll_c100    i16
 *   13 pitch_c100   i16
 *   15 yaw_c100     i16
 *   = 17 字节
 */
#define HEALTH_PAYLOAD_LEN 17u

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

size_t frame_pack_health(const health_payload_t *h, uint8_t *out, size_t out_cap)
{
    if ((h == NULL) || (out == NULL) || (out_cap < HEALTH_PAYLOAD_LEN)) {
        return 0u;
    }
    out[0] = h->heart_rate;
    out[1] = h->confidence;
    put_u16(&out[2], h->rr_interval_ms);
    put_u16(&out[4], (uint16_t)h->temperature_c100);
    put_u16(&out[6], h->steps);
    put_u16(&out[8], h->battery_mv);
    out[10] = h->battery_pct;
    put_u16(&out[11], (uint16_t)h->roll_c100);
    put_u16(&out[13], (uint16_t)h->pitch_c100);
    put_u16(&out[15], (uint16_t)h->yaw_c100);
    return HEALTH_PAYLOAD_LEN;
}

int frame_unpack_health(const uint8_t *p, uint16_t len, health_payload_t *out)
{
    if ((p == NULL) || (out == NULL) || (len < HEALTH_PAYLOAD_LEN)) {
        return -1;
    }
    out->heart_rate = p[0];
    out->confidence = p[1];
    out->rr_interval_ms = get_u16(&p[2]);
    out->temperature_c100 = (int16_t)get_u16(&p[4]);
    out->steps = get_u16(&p[6]);
    out->battery_mv = get_u16(&p[8]);
    out->battery_pct = p[10];
    out->roll_c100 = (int16_t)get_u16(&p[11]);
    out->pitch_c100 = (int16_t)get_u16(&p[13]);
    out->yaw_c100 = (int16_t)get_u16(&p[15]);
    return 0;
}

size_t frame_pack_ack(uint8_t status, uint16_t ack_seq, uint8_t *out, size_t out_cap)
{
    if ((out == NULL) || (out_cap < 3u)) {
        return 0u;
    }
    out[0] = status;
    put_u16(&out[1], ack_seq);
    return 3u;
}

int frame_unpack_ack(const uint8_t *p, uint16_t len, uint8_t *status, uint16_t *ack_seq)
{
    if ((p == NULL) || (len < 3u)) {
        return -1;
    }
    if (status != NULL) {
        *status = p[0];
    }
    if (ack_seq != NULL) {
        *ack_seq = get_u16(&p[1]);
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* TLV                                                                 */
/* ------------------------------------------------------------------ */
int tlv_parse(const uint8_t *p, uint16_t len, tlv_list_t *out)
{
    uint16_t off = 0u;

    if ((p == NULL) || (out == NULL)) {
        return -1;
    }
    memset(out, 0, sizeof(*out));

    while ((off + 2u) <= (uint16_t)len) {
        uint8_t tag = p[off];
        uint8_t vl = p[off + 1u];
        if ((off + 2u + (uint16_t)vl) > (uint16_t)len) {
            return -2;   /* 值长度越过载荷尾部 */
        }
        if (out->count >= TLV_MAX_COUNT) {
            /* 条目超限：多出的忽略但整体仍算解析成功 */
            break;
        }
        if (vl > TLV_MAX_VALUE_LEN) {
            return -3;
        }
        out->item[out->count].tag = tag;
        out->item[out->count].len = vl;
        if (vl > 0u) {
            memcpy(out->item[out->count].value, &p[off + 2u], (size_t)vl);
        }
        out->count = (uint8_t)(out->count + 1u);
        off = (uint16_t)(off + 2u + (uint16_t)vl);
    }
    if (off != (uint16_t)len) {
        return -4;   /* 尾部残留半个 TLV 头 */
    }
    return (int)out->count;
}

const tlv_item_t *tlv_find(const tlv_list_t *l, uint8_t tag)
{
    uint8_t i;

    if (l == NULL) {
        return NULL;
    }
    for (i = 0u; i < l->count; i++) {
        if (l->item[i].tag == tag) {
            return &l->item[i];
        }
    }
    return NULL;
}

int tlv_get_u8(const tlv_list_t *l, uint8_t tag, uint8_t *out)
{
    const tlv_item_t *it = tlv_find(l, tag);
    if ((it == NULL) || (it->len != 1u)) {
        return -1;
    }
    *out = it->value[0];
    return 0;
}

int tlv_get_u16(const tlv_list_t *l, uint8_t tag, uint16_t *out)
{
    const tlv_item_t *it = tlv_find(l, tag);
    if ((it == NULL) || (it->len != 2u)) {
        return -1;
    }
    *out = get_u16(it->value);
    return 0;
}

int tlv_get_str(const tlv_list_t *l, uint8_t tag, char *buf, size_t buf_cap)
{
    const tlv_item_t *it = tlv_find(l, tag);
    if ((it == NULL) || (buf == NULL) || (buf_cap == 0u)) {
        return -1;
    }
    if ((size_t)it->len + 1u > buf_cap) {
        return -1;
    }
    memcpy(buf, it->value, (size_t)it->len);
    buf[it->len] = '\0';
    return 0;
}

size_t tlv_put_u8(uint8_t *out, size_t cap, size_t off, uint8_t tag, uint8_t v)
{
    if ((out == NULL) || ((off + 3u) > cap)) {
        return 0u;
    }
    out[off] = tag;
    out[off + 1u] = 1u;
    out[off + 2u] = v;
    return 3u;
}

size_t tlv_put_u16(uint8_t *out, size_t cap, size_t off, uint8_t tag, uint16_t v)
{
    if ((out == NULL) || ((off + 4u) > cap)) {
        return 0u;
    }
    out[off] = tag;
    out[off + 1u] = 2u;
    put_u16(&out[off + 2u], v);
    return 4u;
}

size_t tlv_put_str(uint8_t *out, size_t cap, size_t off, uint8_t tag, const char *s)
{
    size_t n;

    if ((out == NULL) || (s == NULL)) {
        return 0u;
    }
    n = strlen(s);
    if (n > TLV_MAX_VALUE_LEN) {
        n = TLV_MAX_VALUE_LEN;
    }
    if ((off + 2u + n) > cap) {
        return 0u;
    }
    out[off] = tag;
    out[off + 1u] = (uint8_t)n;
    if (n > 0u) {
        memcpy(&out[off + 2u], s, n);
    }
    return 2u + n;
}

const char *frame_type_name(uint8_t type)
{
    switch (type) {
    case MSG_HEART_RATE:   return "HEART_RATE";
    case MSG_STEP_COUNT:   return "STEP_COUNT";
    case MSG_TEMPERATURE:  return "TEMPERATURE";
    case MSG_BATTERY:      return "BATTERY";
    case MSG_ATTITUDE:     return "ATTITUDE";
    case MSG_HEALTH_BATCH: return "HEALTH_BATCH";
    case MSG_ACK:          return "ACK";
    case MSG_NACK:         return "NACK";
    case MSG_PARAM_SET:    return "PARAM_SET";
    case MSG_PARAM_QUERY:  return "PARAM_QUERY";
    case MSG_PARAM_REPORT: return "PARAM_REPORT";
    case MSG_TIME_SYNC:    return "TIME_SYNC";
    case MSG_OFFLINE_BATCH:return "OFFLINE_BATCH";
    case MSG_PING:         return "PING";
    case MSG_PONG:         return "PONG";
    case MSG_LOG:          return "LOG";
    case MSG_OTA_BEGIN:    return "OTA_BEGIN";
    default:               return "UNKNOWN";
    }
}
