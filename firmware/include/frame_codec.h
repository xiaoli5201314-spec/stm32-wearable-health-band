/*
 * frame_codec.h -- 自定义应用层帧编解码 + 帧同步状态机
 *
 * 字节序统一小端（LE）。解码器是纯状态机，可逐字节喂入，
 * 因此天然支持 UART / TCP 的"半包"与"粘包"。
 */
#ifndef FRAME_CODEC_H
#define FRAME_CODEC_H

#include <stdint.h>
#include <stddef.h>
#include "band_config.h"

typedef struct {
    uint8_t  type;
    uint16_t seq;
    uint16_t len;
    uint8_t  payload[FRAME_MAX_PAYLOAD];
} frame_t;

/* 解码状态机内部状态 */
typedef enum {
    FRAME_ST_SOF0 = 0,
    FRAME_ST_SOF1,
    FRAME_ST_TYPE,
    FRAME_ST_SEQ_LO,
    FRAME_ST_SEQ_HI,
    FRAME_ST_LEN_LO,
    FRAME_ST_LEN_HI,
    FRAME_ST_PAYLOAD,
    FRAME_ST_CRC_LO,
    FRAME_ST_CRC_HI
} frame_state_t;

typedef struct {
    frame_state_t state;
    uint8_t  type;
    uint16_t seq;
    uint16_t len;
    uint16_t idx;
    uint16_t crc_rx;       /* 从帧尾收到的 CRC（小端拼装） */
    uint16_t crc_calc;     /* 边收边算的 CRC16 */
    uint8_t  payload[FRAME_MAX_PAYLOAD];

    /* 统计：用于断言与线上诊断 */
    uint32_t bytes_in;
    uint32_t frames_ok;
    uint32_t crc_errors;    /* CRC 校验失败被拒绝的帧 */
    uint32_t len_errors;    /* 载荷长度超过 FRAME_MAX_PAYLOAD */
    uint32_t resyncs;       /* 帧头丢失后重新对齐的次数 */
} frame_decoder_t;

typedef enum {
    FRAME_DEC_NONE     = 0,   /* 继续喂字节 */
    FRAME_DEC_OK       = 1,   /* 解出一帧 */
    FRAME_DEC_CRC_ERR  = -1,  /* 校验错，帧被拒绝 */
    FRAME_DEC_LEN_ERR  = -2   /* 长度非法，帧被拒绝 */
} frame_dec_result_t;

/* ---------------- 编码 ---------------- */
uint16_t frame_crc16(const uint8_t *data, size_t len);
uint16_t frame_crc16_update(uint16_t crc, uint8_t byte);
/* 把 frame 编码进 out，返回写入字节数；out_cap 不足返回 0 */
size_t frame_encode(const frame_t *f, uint8_t *out, size_t out_cap);
/* 便捷构造并编码 */
size_t frame_build(uint8_t type, uint16_t seq, const uint8_t *payload,
                   uint16_t len, uint8_t *out, size_t out_cap);

/* ---------------- 解码 ---------------- */
void frame_decoder_init(frame_decoder_t *d);
/* 逐字节喂入；返回 FRAME_DEC_NONE 表示还需要更多字节 */
frame_dec_result_t frame_decoder_push(frame_decoder_t *d, uint8_t byte, frame_t *out);
/* 批量喂入，最多解出 out_cap 帧；*consumed 返回消耗的字节数（用于粘包循环） */
size_t frame_decoder_push_buf(frame_decoder_t *d, const uint8_t *buf, size_t len,
                              frame_t *out, size_t out_cap, size_t *consumed,
                              uint32_t *crc_errors);

/* ---------------- 载荷编解码 ---------------- */
typedef struct {
    uint8_t  heart_rate;      /* bpm */
    uint8_t  confidence;      /* 0..100 */
    uint16_t rr_interval_ms;  /* 相邻心搏间隔 */
    int16_t  temperature_c100;/* 体温，0.01 摄氏度 */
    uint16_t steps;
    uint16_t battery_mv;
    uint8_t  battery_pct;
    int16_t  roll_c100;       /* 姿态角，0.01 度 */
    int16_t  pitch_c100;
    int16_t  yaw_c100;
} health_payload_t;

size_t  frame_pack_health(const health_payload_t *h, uint8_t *out, size_t out_cap);
int     frame_unpack_health(const uint8_t *p, uint16_t len, health_payload_t *out);

/* 远程参数下发：TLV 解析结果 */
typedef struct {
    uint8_t  tag;
    uint8_t  len;
    uint8_t  value[TLV_MAX_VALUE_LEN];
} tlv_item_t;

typedef struct {
    tlv_item_t item[TLV_MAX_COUNT];
    uint8_t    count;
} tlv_list_t;

/* 解析 TLV 载荷；返回解析成功的条目数，<0 表示格式错误 */
int tlv_parse(const uint8_t *p, uint16_t len, tlv_list_t *out);
const tlv_item_t *tlv_find(const tlv_list_t *l, uint8_t tag);
/* 从 TLV 取整数（1/2/4 字节小端） */
int tlv_get_u8(const tlv_list_t *l, uint8_t tag, uint8_t *out);
int tlv_get_u16(const tlv_list_t *l, uint8_t tag, uint16_t *out);
/* 取字符串（补 '\0'），buf_cap 不足返回 -1 */
int tlv_get_str(const tlv_list_t *l, uint8_t tag, char *buf, size_t buf_cap);

/* TLV 组包辅助 */
size_t tlv_put_u8(uint8_t *out, size_t cap, size_t off, uint8_t tag, uint8_t v);
size_t tlv_put_u16(uint8_t *out, size_t cap, size_t off, uint8_t tag, uint16_t v);
size_t tlv_put_str(uint8_t *out, size_t cap, size_t off, uint8_t tag, const char *s);

/* ACK 载荷：status(1) + ack_seq(2) */
size_t frame_pack_ack(uint8_t status, uint16_t ack_seq, uint8_t *out, size_t out_cap);
int    frame_unpack_ack(const uint8_t *p, uint16_t len, uint8_t *status, uint16_t *ack_seq);

const char *frame_type_name(uint8_t type);

#endif /* FRAME_CODEC_H */
