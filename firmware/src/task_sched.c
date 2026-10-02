/*
 * task_sched.c -- 轻量协作式任务调度器 + 信号量 + 消息邮箱
 *
 * 调度规则（对应 docs/DESIGN.md 的任务优先级表）：
 *   1. 任意时刻只有一个任务在"运行"，任务函数返回即让出 CPU（run-to-yield）；
 *   2. 选择策略：优先级高者先跑；同优先级按 rr_cursor 轮转，且每个任务
 *      最多连续占用 slice_ms 的时间片（时间片耗尽即换人）；
 *   3. 周期任务（period_ms > 0）跑完自动进入 BLOCKED，到期由 sched_tick 唤醒，
 *      因此周期抖动只取决于 tick 精度，不取决于其它任务的执行时间；
 *   4. 阻塞原语不做栈切换：os_sem_pend / os_mbox_pend 在无法立即获取时把
 *      当前任务标记 BLOCKED 并返回 -1，任务函数直接 return，下一轮再试。
 *      这保证每个任务只有一份栈，最坏执行时间可静态分析。
 *
 * 时间基准是虚拟的：由 sched_tick() 推进，所以单元测试可以完全确定性地
 * 复现调度序列，不需要真的睡 100ms。
 */
#include "task_sched.h"
#include "band_port.h"
#include <stdio.h>
#include <string.h>

#define INVALID_TASK 0xFFu

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */
void sched_init(os_sched_t *s, uint32_t now_ms)
{
    if (s == NULL) {
        return;
    }
    memset(s, 0, sizeof(*s));
    s->now_ms = now_ms;
    s->current = INVALID_TASK;
}

int sched_add_task(os_sched_t *s, const char *name, task_fn_t fn, void *arg,
                   uint8_t prio, uint16_t slice_ms, uint32_t period_ms)
{
    os_task_t *t;

    if ((s == NULL) || (fn == NULL) || (s->task_count >= SCHED_TASK_MAX)) {
        return -1;
    }
    if (prio >= SCHED_PRIO_LEVELS) {
        return -1;
    }
    t = &s->task[s->task_count];
    memset(t, 0, sizeof(*t));
    t->name = (name != NULL) ? name : "task";
    t->fn = fn;
    t->arg = arg;
    t->prio = prio;
    t->slice_ms = (slice_ms == 0u) ? SCHED_DEFAULT_SLICE_MS : slice_ms;
    t->period_ms = period_ms;
    t->state = TASK_READY;
    t->wait_kind = TASK_WAIT_NONE;
    t->wait_obj = NULL;
    s->task_count = (uint8_t)(s->task_count + 1u);
    return (int)(s->task_count - 1u);
}

os_task_t *sched_find_task(os_sched_t *s, const char *name)
{
    uint8_t i;

    if ((s == NULL) || (name == NULL)) {
        return NULL;
    }
    for (i = 0u; i < s->task_count; i++) {
        if ((s->task[i].name != NULL) && (strcmp(s->task[i].name, name) == 0)) {
            return &s->task[i];
        }
    }
    return NULL;
}

os_task_t *sched_current_task(os_sched_t *s)
{
    if ((s == NULL) || (s->current == INVALID_TASK) || (s->current >= s->task_count)) {
        return NULL;
    }
    return &s->task[s->current];
}

uint32_t sched_millis(const os_sched_t *s)
{
    return (s == NULL) ? 0u : s->now_ms;
}

/* ------------------------------------------------------------------ */
/* 时间推进                                                            */
/* ------------------------------------------------------------------ */
void sched_tick(os_sched_t *s, uint32_t elapsed_ms)
{
    uint8_t i;

    if (s == NULL) {
        return;
    }
    s->now_ms += elapsed_ms;
    s->ticks++;

    /* 把 elapsed 记到刚运行过的任务头上，用于 CPU 占用统计 */
    if ((s->current != INVALID_TASK) && (s->current < s->task_count)) {
        os_task_t *cur = &s->task[s->current];
        cur->cpu_ms += elapsed_ms;
        uint32_t used = (uint32_t)cur->used_ms + elapsed_ms;
        cur->used_ms = (uint16_t)((used > 0xFFFFu) ? 0xFFFFu : used);
    }

    /* 唤醒到期任务 */
    for (i = 0u; i < s->task_count; i++) {
        os_task_t *t = &s->task[i];
        if (t->state != TASK_BLOCKED) {
            continue;
        }
        if ((int32_t)(s->now_ms - t->wake_at_ms) >= 0) {
            t->state = TASK_READY;
            t->wait_kind = TASK_WAIT_NONE;
            t->wait_obj = NULL;
            t->used_ms = 0u;
        }
    }
}

/* ------------------------------------------------------------------ */
/* 选择下一个任务                                                      */
/* ------------------------------------------------------------------ */
static int task_ready(const os_task_t *t)
{
    return (t->state == TASK_READY) || (t->state == TASK_RUNNING);
}

static uint8_t select_next(os_sched_t *s)
{
    int prio;

    /* 规则 2 的前半：当前任务若仍有时间片且没有更高优先级就绪任务，继续跑 */
    if ((s->current != INVALID_TASK) && (s->current < s->task_count)) {
        os_task_t *cur = &s->task[s->current];
        int higher_ready = 0;
        uint8_t i;
        for (i = 0u; i < s->task_count; i++) {
            const os_task_t *t = &s->task[i];
            if ((i != s->current) && task_ready(t) && (t->prio > cur->prio)) {
                higher_ready = 1;
                break;
            }
        }
        if ((higher_ready == 0) && task_ready(cur) && (cur->used_ms < cur->slice_ms)) {
            return s->current;
        }
    }

    for (prio = (int)SCHED_PRIO_LEVELS - 1; prio >= 0; prio--) {
        uint8_t n;
        uint8_t k;
        uint8_t best = INVALID_TASK;

        n = 0u;
        for (k = 0u; k < s->task_count; k++) {
            if ((s->task[k].prio == (uint8_t)prio) && task_ready(&s->task[k])) {
                n++;
            }
        }
        if (n == 0u) {
            continue;
        }

        /* 同优先级轮转：从 rr_cursor 开始找第一个就绪任务 */
        for (k = 0u; k < s->task_count; k++) {
            uint8_t idx = (uint8_t)((s->rr_cursor[prio] + k) % s->task_count);
            if ((s->task[idx].prio == (uint8_t)prio) && task_ready(&s->task[idx])) {
                best = idx;
                break;
            }
        }
        if (best != INVALID_TASK) {
            s->rr_cursor[prio] = (uint8_t)((best + 1u) % s->task_count);
            return best;
        }
    }
    return INVALID_TASK;
}

int sched_run_once(os_sched_t *s)
{
    uint8_t idx;
    os_task_t *t;
    uint8_t switched;

    if (s == NULL) {
        return 0;
    }

    idx = select_next(s);
    if (idx == INVALID_TASK) {
        s->current = INVALID_TASK;
        return 0;
    }

    switched = (idx != s->current) ? 1u : 0u;
    if (switched != 0u) {
        s->context_switches++;
        s->current = idx;
    }
    t = &s->task[idx];
    t->state = TASK_RUNNING;
    t->last_start_ms = s->now_ms;
    if (switched != 0u) {
        t->used_ms = 0u;
    }

    /* 执行一步；任务函数返回即让出 */
    t->fn(t->arg);
    t->runs++;

    if (t->state == TASK_RUNNING) {
        if (t->period_ms > 0u) {
            /* 周期任务：跑完自动等下一个周期 */
            t->state = TASK_BLOCKED;
            t->wait_kind = TASK_WAIT_DELAY;
            t->wait_obj = NULL;
            t->wake_at_ms = s->now_ms + t->period_ms;
        } else {
            t->state = TASK_READY;
        }
    }
    return 1;
}

void sched_start(os_sched_t *s)
{
    if (s == NULL) {
        return;
    }
    s->running = 1u;
    s->stop_requested = 0u;
    while (s->stop_requested == 0u) {
        (void)sched_run_once(s);
    }
    s->running = 0u;
}

void sched_stop(os_sched_t *s)
{
    if (s != NULL) {
        s->stop_requested = 1u;
    }
}

/* ------------------------------------------------------------------ */
/* 任务控制                                                            */
/* ------------------------------------------------------------------ */
void sched_block_current(os_sched_t *s, uint32_t timeout_ms)
{
    os_task_t *cur = sched_current_task(s);

    if (cur == NULL) {
        return;
    }
    cur->state = TASK_BLOCKED;
    if (timeout_ms > SCHED_MAX_BLOCK_MS) {
        timeout_ms = SCHED_MAX_BLOCK_MS;
    }
    cur->wake_at_ms = s->now_ms + timeout_ms;
}

void task_yield(os_sched_t *s)
{
    os_task_t *cur = sched_current_task(s);

    if (cur == NULL) {
        return;
    }
    cur->state = TASK_READY;
    cur->used_ms = cur->slice_ms;   /* 主动让出：本时间片立即结束 */
}

void task_sleep_ms(os_sched_t *s, uint32_t ms)
{
    os_task_t *cur = sched_current_task(s);

    if (cur == NULL) {
        return;
    }
    cur->state = TASK_BLOCKED;
    cur->wait_kind = TASK_WAIT_DELAY;
    cur->wait_obj = NULL;
    cur->wake_at_ms = s->now_ms + ms;
}

void task_suspend(os_sched_t *s, const char *name)
{
    os_task_t *t = sched_find_task(s, name);
    if (t != NULL) {
        t->state = TASK_SUSPENDED;
    }
}

void task_resume(os_sched_t *s, const char *name)
{
    os_task_t *t = sched_find_task(s, name);
    if ((t != NULL) && (t->state == TASK_SUSPENDED)) {
        t->state = TASK_READY;
        t->used_ms = 0u;
    }
}

void sched_wake_waiters(os_sched_t *s, const void *obj, task_wait_t kind)
{
    uint8_t i;

    if (s == NULL) {
        return;
    }
    for (i = 0u; i < s->task_count; i++) {
        os_task_t *t = &s->task[i];
        if ((t->state == TASK_BLOCKED) && (t->wait_kind == kind) && (t->wait_obj == obj)) {
            t->state = TASK_READY;
            t->wait_kind = TASK_WAIT_NONE;
            t->wait_obj = NULL;
            t->used_ms = 0u;
        }
    }
}

/* ------------------------------------------------------------------ */
/* 信号量                                                              */
/* ------------------------------------------------------------------ */
void os_sem_init(os_sem_t *sem, const char *name, int32_t initial, int32_t max)
{
    if (sem == NULL) {
        return;
    }
    memset(sem, 0, sizeof(*sem));
    sem->count = initial;
    sem->max = (max <= 0) ? 1 : max;
    sem->name = (name != NULL) ? name : "sem";
}

int os_sem_post(os_sched_t *s, os_sem_t *sem)
{
    if (sem == NULL) {
        return -1;
    }
    if (sem->count >= sem->max) {
        return -1;   /* 计数溢出：调用者逻辑有误（重复 post） */
    }
    sem->count++;
    sem->posts++;
    sched_wake_waiters(s, sem, TASK_WAIT_SEM);
    return 0;
}

int os_sem_pend(os_sched_t *s, os_sem_t *sem, uint32_t timeout_ms)
{
    if (sem == NULL) {
        return -1;
    }
    sem->pends++;
    if (sem->count > 0) {
        sem->count--;
        return 0;
    }
    /* 拿不到：把当前任务挂到该信号量上，任务函数应当立刻 return */
    if ((s != NULL) && (sched_current_task(s) != NULL)) {
        os_task_t *cur = sched_current_task(s);
        if (timeout_ms == 0u) {
            sem->timeouts++;
            return -1;
        }
        sched_block_current(s, timeout_ms);
        cur->wait_kind = TASK_WAIT_SEM;
        cur->wait_obj = sem;
    } else {
        sem->timeouts++;
    }
    return -1;
}

int os_sem_try_pend(os_sched_t *s, os_sem_t *sem)
{
    (void)s;
    if (sem == NULL) {
        return -1;
    }
    if (sem->count > 0) {
        sem->count--;
        sem->pends++;
        return 0;
    }
    return -1;
}

int32_t os_sem_count(const os_sem_t *sem)
{
    return (sem == NULL) ? 0 : sem->count;
}

/* ------------------------------------------------------------------ */
/* 消息邮箱                                                            */
/* ------------------------------------------------------------------ */
void os_mbox_init(os_mbox_t *m, const char *name, uint8_t depth)
{
    uint8_t cap = (uint8_t)((MBOX_DEPTH_SENSOR > MBOX_DEPTH_COMM) ? MBOX_DEPTH_SENSOR : MBOX_DEPTH_COMM);

    if (m == NULL) {
        return;
    }
    memset(m, 0, sizeof(*m));
    m->depth = (depth > cap) ? cap : depth;
    m->name = (name != NULL) ? name : "mbox";
}

int os_mbox_post(os_sched_t *s, os_mbox_t *m, const void *msg)
{
    if ((m == NULL) || (msg == NULL)) {
        return -1;
    }
    if (m->count >= m->depth) {
        m->overflow++;
        return -1;   /* 邮箱满：上层应丢弃或降级处理 */
    }
    memcpy(m->msg[m->head], msg, MBOX_MSG_LEN);
    m->head = (uint8_t)((m->head + 1u) % m->depth);
    m->count = (uint8_t)(m->count + 1u);
    m->posted++;
    sched_wake_waiters(s, m, TASK_WAIT_MBOX);
    return 0;
}

int os_mbox_pend(os_sched_t *s, os_mbox_t *m, void *msg, uint32_t timeout_ms)
{
    if (m == NULL) {
        return -1;
    }
    m->pended++;
    if (m->count > 0) {
        if (msg != NULL) {
            memcpy(msg, m->msg[m->tail], MBOX_MSG_LEN);
        }
        m->tail = (uint8_t)((m->tail + 1u) % m->depth);
        m->count = (uint8_t)(m->count - 1u);
        return 0;
    }
    if ((s != NULL) && (sched_current_task(s) != NULL)) {
        if (timeout_ms == 0u) {
            return -1;
        }
        sched_block_current(s, timeout_ms);
        os_task_t *cur = sched_current_task(s);
        cur->wait_kind = TASK_WAIT_MBOX;
        cur->wait_obj = m;
    }
    return -1;
}

int os_mbox_try_pend(os_sched_t *s, os_mbox_t *m, void *msg)
{
    (void)s;
    if ((m == NULL) || (m->count == 0u)) {
        return -1;
    }
    if (msg != NULL) {
        memcpy(msg, m->msg[m->tail], MBOX_MSG_LEN);
    }
    m->tail = (uint8_t)((m->tail + 1u) % m->depth);
    m->count = (uint8_t)(m->count - 1u);
    m->pended++;
    return 0;
}

uint8_t os_mbox_count(const os_mbox_t *m)
{
    return (m == NULL) ? 0u : m->count;
}

/* ------------------------------------------------------------------ */
/* 统计                                                                */
/* ------------------------------------------------------------------ */
void sched_dump_stats(const os_sched_t *s)
{
    uint8_t i;

    if (s == NULL) {
        return;
    }
    band_trace("[sched] now=%u ms ticks=%u switches=%u idle=%u ms\n",
               (unsigned)s->now_ms, (unsigned)s->ticks,
               (unsigned)s->context_switches, (unsigned)s->idle_ms);
    band_trace("[sched] %-16s %4s %6s %8s %8s %6s\n",
               "task", "prio", "period", "runs", "cpu_ms", "state");
    for (i = 0u; i < s->task_count; i++) {
        const os_task_t *t = &s->task[i];
        band_trace("[sched] %-16s %4u %6u %8u %8u %6d\n",
                   t->name, (unsigned)t->prio, (unsigned)t->period_ms,
                   (unsigned)t->runs, (unsigned)t->cpu_ms, (int)t->state);
    }
}
