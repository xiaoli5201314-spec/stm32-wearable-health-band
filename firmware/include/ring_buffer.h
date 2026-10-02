/*
 * ring_buffer.h -- 字节环形缓冲 + 离线记录环形缓存 + 补传会话
 *
 * 两个层次：
 *   1) band_ring_t     : 字节流环形缓冲，用于 UART/WebSocket 收包解帧。
 *   2) offline_cache_t : 记录级环形缓存（断网期间缓存待上传的体征记录），
 *                        配合 resend_ctx_t 实现"按序号补传且不重复"。
 */
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stddef.h>
#include "band_config.h"

/* ================================================================== */
/* 1) 字节环形缓冲                                                      */
/* ================================================================== */
typedef struct {
    uint8_t  buf[RING_BUFFER_CAP];
    uint16_t head;      /* 下一个写入位置 */
    uint16_t tail;      /* 下一个读出位置 */
    uint16_t used;      /* 当前有效字节数 */
    uint32_t written;   /* 累计写入（统计用） */
    uint32_t read;      /* 累计读出 */
    uint32_t dropped;   /* 因溢出丢弃的字节数 */
} band_ring_t;

void     ring_init(band_ring_t *rb);
size_t   ring_write(band_ring_t *rb, const uint8_t *data, size_t len);   /* 返回实际写入 */
size_t   ring_read(band_ring_t *rb, uint8_t *out, size_t len);           /* 返回实际读出 */
size_t   ring_peek(const band_ring_t *rb, uint8_t *out, size_t len);
size_t   ring_drop(band_ring_t *rb, size_t len);
void     ring_reset(band_ring_t *rb);
uint16_t ring_used(const band_ring_t *rb);
uint16_t ring_free(const band_ring_t *rb);
uint8_t  ring_get(const band_ring_t *rb, uint16_t idx);   /* idx=0 表示最旧字节 */

/* ================================================================== */
/* 2) 离线记录环形缓存 + 补传                                          */
/* ================================================================== */
typedef struct {
    uint16_t seq;                    /* 单调递增序号，补传去重的唯一依据 */
    uint16_t len;
    uint8_t  type;                   /* 原始帧类型，补传时保留语义 */
    uint8_t  sent;                   /* 本补传会话内是否已发送（随记录一起搬移，
                                        删除记录不会错位 -> 不会重复发送） */
    uint8_t  data[OFFLINE_REC_MAX];
} offline_rec_t;

typedef struct {
    offline_rec_t slot[OFFLINE_SLOT_MAX];
    uint16_t      head;              /* 下一个写入槽 */
    uint16_t      count;             /* 当前有效记录数 */
    uint16_t      next_seq;          /* 下一条记录分配的序号 */
    uint32_t      stored_total;      /* 累计入队 */
    uint32_t      dropped_total;     /* 因缓存满被覆盖的最旧记录数 */
    uint32_t      acked_total;       /* 累计被确认并移除 */
} offline_cache_t;

typedef struct {
    offline_cache_t cache;
    uint16_t        scan_idx;              /* 本轮扫描位置 */
    uint16_t        session_ack_watermark; /* 本会话已确认到的最大序号 */
    uint8_t         session_active;
    uint32_t        session_rounds;
    uint32_t        sent_this_session;
    uint32_t        dup_suppressed;        /* 被去重逻辑拦下的重复发送次数 */
} resend_ctx_t;

void offline_cache_init(offline_cache_t *c);
/* 入队一条待补传记录，序号由缓存自己分配；返回记录序号，0 表示失败 */
uint16_t offline_cache_push(offline_cache_t *c, uint8_t type,
                            const uint8_t *data, uint16_t len);
/* 入队并**指定序号**：补传去重是按"帧序号"做的，缓存里的 seq 必须与
 * 帧头里的 SEQ 完全一致，否则 ACK 会删错记录（这一点由联调测试发现并修正）。*/
uint16_t offline_cache_push_seq(offline_cache_t *c, uint16_t seq, uint8_t type,
                                const uint8_t *data, uint16_t len);
/* 按序号精确移除（收到 ACK 后调用）；返回 0 成功，-1 未找到 */
int  offline_cache_remove(offline_cache_t *c, uint16_t seq);
int  offline_cache_find(const offline_cache_t *c, uint16_t seq, offline_rec_t *out);
uint16_t offline_cache_oldest_seq(const offline_cache_t *c);
uint16_t offline_cache_newest_seq(const offline_cache_t *c);
int  offline_cache_contains(const offline_cache_t *c, uint16_t seq);

/* --- 补传会话：断线重连后调用 begin，逐条 next，收到 ACK 后 ack --- */
void resend_begin(resend_ctx_t *r);
/* 取出下一条待补传记录（严格按序号升序，同一会话内每条最多返回一次）
 * 返回 1 取到，0 本轮已无待补传记录 */
int  resend_next(resend_ctx_t *r, offline_rec_t *out);
/* 收到对端 ACK：从缓存移除该序号；返回 0 成功 -1 未知序号 */
int  resend_ack(resend_ctx_t *r, uint16_t seq);
/* 收到已在缓存中但属于"本轮已发过"的 ACK 时，计入 dup_suppressed */
void resend_note_duplicate(resend_ctx_t *r);
uint16_t resend_pending_count(const resend_ctx_t *r);

#endif /* RING_BUFFER_H */
