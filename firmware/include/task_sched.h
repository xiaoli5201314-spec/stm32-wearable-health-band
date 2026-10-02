/*
 * task_sched.h -- 轻量协作式任务调度器（原创实现，非 uCOS/FreeRTOS 代码）
 *
 * 保留 RTOS 的经典语义，但把"抢占"换成"协作"：
 *   - 任务是一个被反复调用的函数，每次调用完成一小步工作后返回（run-to-yield）；
 *   - 调度器在每次调用之间做决策：优先级高者先跑，同优先级按时间片轮转；
 *   - 阻塞原语（延时 / 信号量 / 邮箱）不会挂起调用栈，而是把任务标记为 BLOCKED，
 *     由调度循环在下一个 tick 重新评估，因此不需要每个任务独立的栈和汇编切换。
 *
 * 这样在手表这种"事件稀疏 + 必须在低功耗下可预测"的场景里，
 * 比抢占式内核更容易分析最坏执行时间，也彻底规避了第三方内核的授权问题。
 */
#ifndef TASK_SCHED_H
#define TASK_SCHED_H

#include <stdint.h>
#include <stddef.h>
#include "band_config.h"

typedef enum {
    TASK_UNUSED = 0,
    TASK_READY,
    TASK_RUNNING,
    TASK_BLOCKED,      /* 延时 / 等信号量 / 等邮箱 */
    TASK_SUSPENDED,
    TASK_STOPPED
} task_state_t;

typedef void (*task_fn_t)(void *arg);

typedef enum {
    TASK_WAIT_NONE = 0,
    TASK_WAIT_DELAY,
    TASK_WAIT_SEM,
    TASK_WAIT_MBOX
} task_wait_t;

typedef struct {
    const char  *name;
    task_fn_t    fn;
    void        *arg;
    uint8_t      prio;          /* 越大越紧急 */
    uint16_t     slice_ms;      /* 时间片预算 */
    uint32_t     period_ms;     /* 0 = 只要就绪就运行；>0 = 周期节拍 */
    uint32_t     last_start_ms;
    uint16_t     used_ms;       /* 本时间片已消耗 */
    uint32_t     wake_at_ms;    /* BLOCKED 的唤醒时刻 */
    task_state_t state;
    task_wait_t  wait_kind;     /* 阻塞原因 */
    const void  *wait_obj;      /* 阻塞在哪个对象上（sem/mbox 指针） */
    /* 统计 */
    uint32_t     runs;
    uint32_t     cpu_ms;
    uint32_t     deadline_misses;
    uint32_t     max_latency_ms;
} os_task_t;

/* ---------------- 信号量 ---------------- */
typedef struct {
    int32_t  count;
    int32_t  max;
    uint32_t posts;
    uint32_t pends;
    uint32_t timeouts;
    const char *name;
} os_sem_t;

/* ---------------- 消息邮箱（定长消息队列） ---------------- */
typedef struct {
    uint8_t  msg[MBOX_DEPTH_SENSOR > MBOX_DEPTH_COMM ? MBOX_DEPTH_SENSOR : MBOX_DEPTH_COMM][MBOX_MSG_LEN];
    uint8_t  depth;
    uint8_t  head;
    uint8_t  tail;
    uint8_t  count;
    uint32_t posted;
    uint32_t pended;
    uint32_t overflow;
    const char *name;
} os_mbox_t;

/* ---------------- 调度器 ---------------- */
typedef struct {
    os_task_t   task[SCHED_TASK_MAX];
    uint8_t     task_count;
    uint8_t     current;         /* 当前运行任务下标，0xFF 空闲 */
    uint32_t    now_ms;
    uint32_t    idle_ms;
    uint32_t    context_switches;
    uint8_t     running;
    uint8_t     stop_requested;
    /* 每优先级轮转游标，实现同优先级时间片轮转 */
    uint8_t     rr_cursor[SCHED_PRIO_LEVELS];
    uint32_t    ticks;
} os_sched_t;

void     sched_init(os_sched_t *s, uint32_t now_ms);
int      sched_add_task(os_sched_t *s, const char *name, task_fn_t fn, void *arg,
                        uint8_t prio, uint16_t slice_ms, uint32_t period_ms);
void     sched_start(os_sched_t *s);
void     sched_stop(os_sched_t *s);
/* 推进一次调度决策 + 执行一个任务的一步；返回 1 表示跑了任务，0 表示空闲 */
int      sched_run_once(os_sched_t *s);
/* 时间推进：唤醒到期的延时任务 */
void     sched_tick(os_sched_t *s, uint32_t elapsed_ms);
uint32_t sched_millis(const os_sched_t *s);
os_task_t *sched_current_task(os_sched_t *s);
os_task_t *sched_find_task(os_sched_t *s, const char *name);

/* 任务主动让出：本次调用结束，回到调度器 */
void     task_yield(os_sched_t *s);
/* 任务阻塞 ms 毫秒（由调度器在到期后唤醒） */
void     task_sleep_ms(os_sched_t *s, uint32_t ms);
void     task_suspend(os_sched_t *s, const char *name);
void     task_resume(os_sched_t *s, const char *name);

/* ---------------- 信号量 ---------------- */
void os_sem_init(os_sem_t *sem, const char *name, int32_t initial, int32_t max);
int  os_sem_post(os_sched_t *s, os_sem_t *sem);                 /* 0 ok, -1 溢出 */
int  os_sem_pend(os_sched_t *s, os_sem_t *sem, uint32_t timeout_ms); /* 0 ok, -1 超时 */
int  os_sem_try_pend(os_sched_t *s, os_sem_t *sem);             /* 非阻塞 */
int32_t os_sem_count(const os_sem_t *sem);

/* ---------------- 邮箱 ---------------- */
void os_mbox_init(os_mbox_t *m, const char *name, uint8_t depth);
int  os_mbox_post(os_sched_t *s, os_mbox_t *m, const void *msg);           /* 0 ok, -1 满 */
int  os_mbox_pend(os_sched_t *s, os_mbox_t *m, void *msg, uint32_t timeout_ms);
int  os_mbox_try_pend(os_sched_t *s, os_mbox_t *m, void *msg);
uint8_t os_mbox_count(const os_mbox_t *m);

/* 把任务阻塞原因登记到调度器（供 pend 使用） */
void sched_block_current(os_sched_t *s, uint32_t timeout_ms);
/* 内部：把等待 obj 的任务唤醒 */
void sched_wake_waiters(os_sched_t *s, const void *obj, task_wait_t kind);

/* 运行时统计打印（PC 仿真下输出到 stdout，目标板上走 trace） */
void sched_dump_stats(const os_sched_t *s);

#endif /* TASK_SCHED_H */
