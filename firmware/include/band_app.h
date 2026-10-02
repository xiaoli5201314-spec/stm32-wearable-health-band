/*
 * band_app.h -- 应用层对外接口（PC 自检与目标板 main 都从这里进入）
 */
#ifndef BAND_APP_H
#define BAND_APP_H

#include <stdint.h>
#include "band_config.h"
#include "task_sched.h"
#include "power_mgr.h"
#include "uplink.h"
#include "iwdg.h"

/* 体征快照：界面与上行共用的数据结构 */
typedef struct {
    uint8_t  heart_rate;
    uint8_t  hr_confidence;
    uint16_t rr_ms;
    float    body_temp_c;
    float    ambient_temp_c;
    uint16_t steps;
    uint16_t cadence_spm;
    uint8_t  step_confidence;
    float    roll_deg, pitch_deg, yaw_deg;
    uint16_t battery_mv;
    uint8_t  battery_pct;
    uint8_t  charging;
    uint32_t uptime_s;
} band_snapshot_t;

/* 应用上下文做成不透明句柄：外部只能通过下面的访问器取子模块 */
typedef struct band_app_s band_app_t;

band_app_t      *band_app_instance(void);
os_sched_t      *band_app_sched(void);
power_mgr_t     *band_app_power(void);
band_uplink_t   *band_app_uplink(void);
iwdg_t          *band_app_iwdg(void);
const band_snapshot_t *band_app_snapshot(void);

/* 初始化：ws_host 为 NULL 表示用默认 127.0.0.1:9001/band */
int  band_app_init(const char *ws_host, uint16_t ws_port, const char *ws_path);
/* 确定性推进：跑 steps 个调度周期，每个周期消耗 1ms 虚拟时间（单元测试用） */
int  band_app_run_steps(uint32_t steps);
/* 真实时间推进：跑 ms 毫秒（PC 联调 / 现场演示用） */
void band_app_run_ms(uint32_t ms);
void band_app_dump_stats(void);

#endif /* BAND_APP_H */
