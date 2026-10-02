/*
 * ring_buffer.c -- 字节环形缓冲 + 离线记录环形缓存 + 补传会话
 *
 * 全部为纯内存操作，无动态分配，可在 PC 上直接单元测试。
 */
#include "ring_buffer.h"
#include <string.h>

/* ================================================================== */
/* 1) 字节环形缓冲                                                      */
/* ================================================================== */
void ring_init(band_ring_t *rb)
{
    if (rb == NULL) {
        return;
    }
    memset(rb, 0, sizeof(*rb));
}

size_t ring_write(band_ring_t *rb, const uint8_t *data, size_t len)
{
    size_t i;
    size_t space;

    if ((rb == NULL) || (data == NULL)) {
        return 0u;
    }
    /* 单次写入超过整个容量时，直接保留"最新的 CAP 个字节"（旧数据无意义） */
    if (len > (size_t)RING_BUFFER_CAP) {
        size_t skip = len - (size_t)RING_BUFFER_CAP;
        rb->dropped += (uint32_t)skip;
        data += skip;
        len = (size_t)RING_BUFFER_CAP;
    }

    space = (size_t)(RING_BUFFER_CAP - rb->used);
    if (len > space) {
        /* 溢出策略：丢最旧数据保最新 —— 体征数据"新"比"全"重要 */
        size_t over = len - space;
        (void)ring_drop(rb, over);
        rb->dropped += (uint32_t)over;
        len = space;
    }
    for (i = 0u; i < len; i++) {
        rb->buf[rb->head] = data[i];
        rb->head = (uint16_t)((rb->head + 1u) % RING_BUFFER_CAP);
    }
    rb->used = (uint16_t)(rb->used + (uint16_t)len);
    rb->written += (uint32_t)len;
    return len;
}

size_t ring_read(band_ring_t *rb, uint8_t *out, size_t len)
{
    size_t i;
    size_t n;

    if ((rb == NULL) || (out == NULL)) {
        return 0u;
    }
    n = (len < (size_t)rb->used) ? len : (size_t)rb->used;
    for (i = 0u; i < n; i++) {
        out[i] = rb->buf[rb->tail];
        rb->tail = (uint16_t)((rb->tail + 1u) % RING_BUFFER_CAP);
    }
    rb->used = (uint16_t)(rb->used - (uint16_t)n);
    rb->read += (uint32_t)n;
    return n;
}

size_t ring_peek(const band_ring_t *rb, uint8_t *out, size_t len)
{
    size_t i;
    size_t n;
    uint16_t idx;

    if ((rb == NULL) || (out == NULL)) {
        return 0u;
    }
    n = (len < (size_t)rb->used) ? len : (size_t)rb->used;
    idx = rb->tail;
    for (i = 0u; i < n; i++) {
        out[i] = rb->buf[idx];
        idx = (uint16_t)((idx + 1u) % RING_BUFFER_CAP);
    }
    return n;
}

size_t ring_drop(band_ring_t *rb, size_t len)
{
    size_t n;

    if (rb == NULL) {
        return 0u;
    }
    n = (len < (size_t)rb->used) ? len : (size_t)rb->used;
    rb->tail = (uint16_t)((rb->tail + (uint16_t)n) % RING_BUFFER_CAP);
    rb->used = (uint16_t)(rb->used - (uint16_t)n);
    return n;
}

void ring_reset(band_ring_t *rb)
{
    if (rb == NULL) {
        return;
    }
    rb->head = 0u;
    rb->tail = 0u;
    rb->used = 0u;
}

uint16_t ring_used(const band_ring_t *rb)
{
    return (rb == NULL) ? 0u : rb->used;
}

uint16_t ring_free(const band_ring_t *rb)
{
    return (rb == NULL) ? 0u : (uint16_t)(RING_BUFFER_CAP - rb->used);
}

uint8_t ring_get(const band_ring_t *rb, uint16_t idx)
{
    if ((rb == NULL) || (idx >= rb->used)) {
        return 0u;
    }
    return rb->buf[(uint16_t)((rb->tail + idx) % RING_BUFFER_CAP)];
}

/* ================================================================== */
/* 2) 离线记录环形缓存                                                  */
/* ================================================================== */
void offline_cache_init(offline_cache_t *c)
{
    if (c == NULL) {
        return;
    }
    memset(c, 0, sizeof(*c));
    c->next_seq = 1u;   /* 0 保留给"无效/未分配" */
}

/* 物理槽位 -> 逻辑序号（0 = 最旧） */
static uint16_t offline_slot_of(const offline_cache_t *c, uint16_t logical)
{
    return (uint16_t)((c->head + OFFLINE_SLOT_MAX - c->count + logical) % OFFLINE_SLOT_MAX);
}

uint16_t offline_cache_push(offline_cache_t *c, uint8_t type,
                            const uint8_t *data, uint16_t len)
{
    uint16_t seq;

    if (c == NULL) {
        return 0u;
    }
    seq = c->next_seq;
    if (seq == 0u) {
        seq = 1u;   /* 序号回绕时跳过 0 */
    }
    return offline_cache_push_seq(c, seq, type, data, len);
}

uint16_t offline_cache_push_seq(offline_cache_t *c, uint16_t seq, uint8_t type,
                                const uint8_t *data, uint16_t len)
{
    offline_rec_t *rec;

    if ((c == NULL) || (data == NULL) || (len > OFFLINE_REC_MAX) || (seq == 0u)) {
        return 0u;
    }
    if (c->count >= OFFLINE_SLOT_MAX) {
        /* 缓存满：覆盖最旧记录并计数。
         * 注意：head 是"下一个写入位置"，当 count == MAX 时它正好就是最旧槽，
         * 因此这里只计数、不移动 head —— 移动 head 会破坏逻辑时序，
         * 导致只读逻辑按错位的顺序取记录。 */
        c->dropped_total++;
    }

    rec = &c->slot[c->head];
    rec->seq = seq;
    rec->len = len;
    rec->type = type;
    rec->sent = 0u;
    if (len > 0u) {
        memcpy(rec->data, data, (size_t)len);
    }
    c->head = (uint16_t)((c->head + 1u) % OFFLINE_SLOT_MAX);
    if (c->count < OFFLINE_SLOT_MAX) {
        c->count = (uint16_t)(c->count + 1u);
    }
    /* 让自动分配序号始终排在已用序号之后 */
    if ((uint16_t)(c->next_seq - seq) > 0u) {
        /* next_seq 已经领先，不动 */
    } else {
        c->next_seq = (uint16_t)(seq + 1u);
        if (c->next_seq == 0u) {
            c->next_seq = 1u;
        }
    }
    c->stored_total++;
    return seq;
}

int offline_cache_find(const offline_cache_t *c, uint16_t seq, offline_rec_t *out)
{
    uint16_t i;

    if (c == NULL) {
        return -1;
    }
    for (i = 0u; i < c->count; i++) {
        const offline_rec_t *rec = &c->slot[offline_slot_of(c, i)];
        if (rec->seq == seq) {
            if (out != NULL) {
                *out = *rec;
            }
            return 0;
        }
    }
    return -1;
}

int offline_cache_contains(const offline_cache_t *c, uint16_t seq)
{
    return (offline_cache_find(c, seq, NULL) == 0) ? 1 : 0;
}

/* 把逻辑下标 logical 处的记录删掉：把它之后的记录整体前移一格 */
static void offline_cache_remove_at(offline_cache_t *c, uint16_t logical)
{
    uint16_t i;
    uint16_t slot = offline_slot_of(c, logical);
    uint16_t next = (uint16_t)((slot + 1u) % OFFLINE_SLOT_MAX);

    for (i = (uint16_t)(logical + 1u); i < c->count; i++) {
        c->slot[slot] = c->slot[next];
        slot = next;
        next = (uint16_t)((next + 1u) % OFFLINE_SLOT_MAX);
    }
    c->head = (uint16_t)((c->head + OFFLINE_SLOT_MAX - 1u) % OFFLINE_SLOT_MAX);
    c->count = (uint16_t)(c->count - 1u);
}

int offline_cache_remove(offline_cache_t *c, uint16_t seq)
{
    uint16_t i;

    if (c == NULL) {
        return -1;
    }
    for (i = 0u; i < c->count; i++) {
        if (c->slot[offline_slot_of(c, i)].seq == seq) {
            offline_cache_remove_at(c, i);
            c->acked_total++;
            return 0;
        }
    }
    return -1;
}

uint16_t offline_cache_oldest_seq(const offline_cache_t *c)
{
    if ((c == NULL) || (c->count == 0u)) {
        return 0u;
    }
    return c->slot[offline_slot_of(c, 0u)].seq;
}

uint16_t offline_cache_newest_seq(const offline_cache_t *c)
{
    if ((c == NULL) || (c->count == 0u)) {
        return 0u;
    }
    return c->slot[offline_slot_of(c, (uint16_t)(c->count - 1u))].seq;
}

/* ================================================================== */
/* 3) 补传会话                                                          */
/* ================================================================== */
void resend_begin(resend_ctx_t *r)
{
    uint16_t i;

    if (r == NULL) {
        return;
    }
    /* 新会话：清除所有记录的"本会话已发"标记，未确认的记录会在本会话重发 */
    for (i = 0u; i < OFFLINE_SLOT_MAX; i++) {
        r->cache.slot[i].sent = 0u;
    }
    r->scan_idx = 0u;
    r->session_ack_watermark = 0u;
    r->session_active = 1u;
    r->session_rounds++;
    r->sent_this_session = 0u;
    /* 注意：dup_suppressed 是全局去重统计，不在此清零 */
}

int resend_next(resend_ctx_t *r, offline_rec_t *out)
{
    offline_cache_t *c;

    if ((r == NULL) || (out == NULL) || (r->session_active == 0u)) {
        return 0;
    }
    c = &r->cache;
    while (r->scan_idx < c->count) {
        uint16_t logical = r->scan_idx;
        uint16_t slot = (uint16_t)((c->head + OFFLINE_SLOT_MAX - c->count + logical) % OFFLINE_SLOT_MAX);
        offline_rec_t *rec = &c->slot[slot];
        r->scan_idx++;

        /* 去重核心：同一会话内同一序号最多发送一次 */
        if (rec->sent != 0u) {
            r->dup_suppressed++;
            continue;
        }
        /* 已被确认过的（序号 <= 水位线）不再重发 */
        if ((r->session_ack_watermark != 0u) &&
            ((int16_t)(rec->seq - r->session_ack_watermark) <= 0)) {
            continue;
        }
        *out = *rec;
        rec->sent = 1u;
        r->sent_this_session++;
        return 1;
    }
    return 0;
}

int resend_ack(resend_ctx_t *r, uint16_t seq)
{
    if (r == NULL) {
        return -1;
    }
    if (offline_cache_contains(&r->cache, seq) == 0) {
        /* 序号已不在缓存中 -> 重复 ACK，计入去重统计 */
        r->dup_suppressed++;
        return -1;
    }
    if (offline_cache_remove(&r->cache, seq) != 0) {
        return -1;
    }
    if ((r->session_ack_watermark == 0u) || ((int16_t)(seq - r->session_ack_watermark) > 0)) {
        r->session_ack_watermark = seq;
    }
    /* 删除导致槽位前移，重新从头扫描；已发送标记随记录搬移，不会造成重复发送 */
    r->scan_idx = 0u;
    return 0;
}

void resend_note_duplicate(resend_ctx_t *r)
{
    if (r != NULL) {
        r->dup_suppressed++;
    }
}

uint16_t resend_pending_count(const resend_ctx_t *r)
{
    if (r == NULL) {
        return 0u;
    }
    return r->cache.count;
}
