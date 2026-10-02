/*
 * ws_live_client.c -- 真实 TCP 联调客户端（配合 tools/ws_mock_server.py）
 *
 * 流程（对应验收标准第 3、4 条）：
 *   1) TCP 连接 + WebSocket 握手（真实 socket，不是内存桩）
 *   2) 发一条体征数据；等对端回一条数据（ACK / 远程参数下发）
 *   3) 服务端按脚本断开连接 -> 客户端检测到断线
 *   4) 断网期间缓存 N 条记录
 *   5) 重连 -> 自动按序号补传 N 条 -> 收回 ACK -> 缓存清空
 *
 * 所有实测数据直接打印到 stdout，可被 tools/run_live_test.sh 捕获进 README。
 */
#include "uplink.h"
#include "band_port.h"
#include "frame_codec.h"
#include "net_transport.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static uint32_t g_param_events;

static void on_param(void *user, const band_params_t *p)
{
    (void)user;
    g_param_events++;
    printf("[client]   参数更新 v%u: hr_interval=%ums sensitivity=%u upload=%ums "
           "sleep=%us name=%s\n",
           (unsigned)p->param_version, (unsigned)p->hr_interval_ms,
           (unsigned)p->step_sensitivity, (unsigned)p->upload_interval_ms,
           (unsigned)p->sleep_timeout_s, p->device_name);
}

/* 一直 poll 直到条件满足或超时 */
static int wait_until(band_uplink_t *u, uint32_t timeout_ms, int want_online,
                      uint32_t target_frames)
{
    uint32_t start = band_millis();

    for (;;) {
        uplink_poll(u, band_millis());
        if (want_online != 0) {
            if (u->link_up != 0) {
                return 0;
            }
        } else {
            if (u->link_up == 0) {
                return 0;
            }
        }
        if (target_frames > 0u && u->st.rx_frames >= target_frames) {
            return 0;
        }
        if ((band_millis() - start) > timeout_ms) {
            return -1;
        }
        band_delay_ms(5u);
    }
}

int main(int argc, char **argv)
{
    band_uplink_t up;
    const char *host = (argc > 1) ? argv[1] : "127.0.0.1";
    uint16_t port = (argc > 2) ? (uint16_t)atoi(argv[2]) : (uint16_t)WS_DEFAULT_PORT;
    uint8_t payload[FRAME_MAX_PAYLOAD];
    uint16_t seq = 0u;
    int i;
    int rc;
    int offline_n = 8;

    printf("======================================================\n");
    printf("[client] stm32-wearable-health-band WebSocket 实测客户端\n");
    printf("[client] 目标 %s:%u%s\n", host, (unsigned)port, WS_DEFAULT_PATH);
    printf("======================================================\n");

    (void)net_platform_init();
    uplink_init(&up, net_socket_get(), host, port, WS_DEFAULT_PATH);
    uplink_set_param_callback(&up, on_param, NULL);
    if (argc > 3) {
        up.verbose = 1u;   /* 打印每一帧收发/确认明细，便于现场排查 */
    }

    /* ---------- 1) 握手 ---------- */
    printf("[client] 1) 发起 TCP 连接与 HTTP Upgrade 握手 ...\n");
    rc = uplink_connect(&up, 3000u);
    if (rc != NET_OK) {
        printf("[client] !! 握手失败 rc=%d（请确认 mock server 已启动）\n", rc);
        return 2;
    }
    printf("[client]    握手成功: state=%s\n", uplink_state_str(&up));
    printf("[client]    Sec-WebSocket-Key   = %s\n", up.ws.hs_key_b64);
    printf("[client]    Sec-WebSocket-Accept= %s\n", up.ws.hs_accept);
    printf("[client]    握手成功计数=%u\n", (unsigned)up.ws.handshake_ok);

    /* ---------- 2) 发一条数据 ---------- */
    printf("[client] 2) 发送一条体征数据（MSG_HEART_RATE）...\n");
    memset(payload, 0, sizeof(payload));
    payload[0] = 78u;    /* bpm */
    payload[1] = 92u;    /* 置信度 */
    rc = uplink_send(&up, MSG_HEART_RATE, payload, 4u, &seq);
    printf("[client]    已发送 seq=%u rc=%d, tx_frames=%u\n",
           (unsigned)seq, rc, (unsigned)up.ws.tx_frames);

    /* ---------- 3) 收一条数据 ---------- */
    printf("[client] 3) 等待对端下行数据 ...\n");
    payload[0] = 91u;
    payload[1] = 88u;
    (void)uplink_send(&up, MSG_STEP_COUNT, payload, 4u, &seq);
    printf("[client]    已发送 seq=%u（共 2 帧，服务端脚本设定收满即断开）\n", (unsigned)seq);
    if (wait_until(&up, 2000u, 1, 1u) == 0) {
        printf("[client]    收到下行: rx_frames=%u acks=%u params_applied=%u\n",
               (unsigned)up.st.rx_frames, (unsigned)up.st.acks_rx,
               (unsigned)up.st.params_applied);
    } else {
        printf("[client]    !! 未在 2 秒内收到下行数据\n");
    }

    /* ---------- 4) 断线检测 ---------- */
    printf("[client] 4) 等待服务端主动断开 ...\n");
    /* 应用层策略：确认"这一轮要模拟断网"之后先关掉自动重连，
     * 否则 ws_poll 会在 200ms 退避后自己连回来，缓存阶段就变成在线直发了。 */
    if (wait_until(&up, 3000u, 0, 0u) == 0) {
        up.ws.auto_reconnect = 0u;
        printf("[client]    已检测到断线: ws_state=%s link=%s（已暂停自动重连）\n",
               ws_state_name(up.ws.state), uplink_state_str(&up));
    } else {
        printf("[client]    !! 未检测到断线（服务端 --drop-after 可能未生效）\n");
    }

    /* ---------- 5) 断网期间缓存 N 条 ---------- */
    printf("[client] 5) 断网期间缓存 %d 条记录 ...\n", offline_n);
    for (i = 0; i < offline_n; i++) {
        memset(payload, 0, sizeof(payload));
        payload[0] = (uint8_t)(60 + i);
        payload[1] = (uint8_t)(i * 3);
        payload[2] = (uint8_t)(i & 0xFFu);
        rc = uplink_send(&up, MSG_OFFLINE_BATCH, payload, 6u, &seq);
        printf("[client]    缓存第 %d 条: seq=%u 返回=%d（1=已入离线缓存）\n",
               i + 1, (unsigned)seq, rc);
    }
    printf("[client]    离线缓存条数 = %u\n", (unsigned)uplink_pending_count(&up));

    /* ---------- 6) 重连 + 补传 ---------- */
    printf("[client] 6) 恢复网络 -> 重连并按序号补传 ...\n");
    up.ws.auto_reconnect = 1u;
    rc = uplink_connect(&up, 3000u);
    if (rc != NET_OK) {
        printf("[client] !! 重连失败 rc=%d\n", rc);
        return 3;
    }
    printf("[client]    重连成功: state=%s, 补传会话轮次=%u\n",
           uplink_state_str(&up), (unsigned)up.offline.session_rounds);

    {
        uint32_t start = band_millis();
        while ((band_millis() - start) < 5000u) {
            uplink_poll(&up, band_millis());
            if (uplink_pending_count(&up) == 0u) {
                break;
            }
            band_delay_ms(10u);
        }
    }

    printf("[client] ---------- 实测结果汇总 ----------\n");
    printf("[client] 连接次数(重连计数) = %u\n", (unsigned)up.st.reconnects);
    printf("[client] 在线直发帧数       = %u\n", (unsigned)up.st.sent_online);
    printf("[client] 离线缓存帧数       = %u\n", (unsigned)up.st.cached_offline);
    printf("[client] 补传帧数           = %u\n", (unsigned)up.st.resent);
    printf("[client] 收到 ACK 数        = %u\n", (unsigned)up.st.acks_rx);
    printf("[client] 重复/未跟踪 ACK 数 = %u（在线直发帧的确认没有进补传跟踪表）\n",
           (unsigned)up.st.dup_acks);
    printf("[client] 收到下行帧数       = %u\n", (unsigned)up.st.rx_frames);
    printf("[client] 远程参数应用次数   = %u\n", (unsigned)up.st.params_applied);
    printf("[client] 剩余待补传条数     = %u\n", (unsigned)uplink_pending_count(&up));
    printf("[client] 补传去重拦截次数   = %u\n", (unsigned)up.offline.dup_suppressed);
    printf("[client] WebSocket 统计: tx_frames=%u rx_frames=%u ping=%u pong=%u "
           "handshake_ok=%u proto_err=%u\n",
           (unsigned)up.ws.tx_frames, (unsigned)up.ws.rx_frames,
           (unsigned)up.ws.ping_tx, (unsigned)up.ws.pong_rx,
           (unsigned)up.ws.handshake_ok, (unsigned)up.ws.protocol_errors);

    {
        int ok = 1;
        if (uplink_pending_count(&up) != 0u) {
            printf("[client] !! 断言失败: 补传未被全部确认\n");
            ok = 0;
        }
        if (up.st.resent != (uint32_t)offline_n) {
            printf("[client] !! 断言失败: 补传帧数 %u != 缓存 %d\n",
                   (unsigned)up.st.resent, offline_n);
            ok = 0;
        }
        if (up.st.params_applied == 0u) {
            printf("[client] !! 断言失败: 未收到远程参数下发\n");
            ok = 0;
        }
        printf("[client] 自检结论: %s\n", (ok != 0) ? "PASS" : "FAIL");
        uplink_disconnect(&up);
        net_platform_deinit();
        return ok ? 0 : 1;
    }
}
