/*
 * net_wifi_at.c -- 目标板 TCP 传输：USART + Wi-Fi 模组 AT 指令
 *
 * 与手机 App 的链路是：
 *   STM32 USART1 <--115200 8N1--> Wi-Fi 模组 <--TCP--> 服务器/手机热点
 * 模组用 AT 指令驱动（ESP-AT / 通用 Wi-Fi 模组指令集）：
 *   AT                     -> OK                 探测模组
 *   AT+CWMODE=1            -> OK                  Station 模式
 *   AT+CWJAP="ssid","pwd"  -> OK                 连热点（本工程在配网流程中调用）
 *   AT+CIPMUX=0            -> OK                 单连接
 *   AT+CIPSTART="TCP","host",port -> OK / CONNECT 建立 TCP
 *   AT+CIPSEND=<len>       -> '>' 提示符，随后写 len 字节
 *   +++                    -> 退出透传
 *
 * 说明：本文件在 PC 上也会被编译（保证语法与逻辑始终有效），
 * 只是 PC 上 uart_platform_iface() 指向虚拟 USART，不会真的连模组。
 */
#include "net_transport.h"
#include "band_platform.h"
#include "band_port.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define AT_BUF       256u
#define AT_CMD_TO_MS 2000u

typedef struct {
    net_transport_t base;
    int      linked;
    int      cipmux_set;
    char     rx[AT_BUF];
    size_t   rx_len;
    uint32_t connects;
} wifi_tp_t;

static wifi_tp_t g_wifi;

static void at_flush_rx(void)
{
    uint8_t tmp[64];
    int guard = 0;

    while (guard++ < 64) {
        int n = uart_platform_iface()->read(tmp, sizeof(tmp), 0u);
        if (n <= 0) {
            break;
        }
    }
    g_wifi.rx_len = 0u;
    g_wifi.rx[0] = '\0';
}

static void at_send(const char *cmd)
{
    size_t n = strlen(cmd);
    (void)uart_platform_iface()->write((const uint8_t *)cmd, n);
    (void)uart_platform_iface()->write((const uint8_t *)"\r\n", 2u);
}

/* 等待包含 expect 的响应行；返回 1 命中，0 超时 */
static int at_wait_for(const char *expect, uint32_t timeout_ms)
{
    uint32_t deadline = band_millis() + timeout_ms;
    uint8_t tmp[32];

    for (;;) {
        int n = uart_platform_iface()->read(tmp, sizeof(tmp), 20u);
        if (n > 0) {
            size_t i;
            for (i = 0u; i < (size_t)n; i++) {
                if (g_wifi.rx_len < (AT_BUF - 1u)) {
                    g_wifi.rx[g_wifi.rx_len++] = (char)tmp[i];
                    g_wifi.rx[g_wifi.rx_len] = '\0';
                } else {
                    /* 缓冲满：保留尾部，避免把关键响应挤掉 */
                    memmove(g_wifi.rx, &g_wifi.rx[AT_BUF / 2u], AT_BUF / 2u);
                    g_wifi.rx_len = AT_BUF / 2u;
                }
            }
            if (strstr(g_wifi.rx, expect) != NULL) {
                return 1;
            }
            if (strstr(g_wifi.rx, "ERROR") != NULL) {
                return 0;
            }
        }
        if ((int32_t)(band_millis() - deadline) >= 0) {
            return 0;
        }
    }
}

static int wifi_connect(struct net_transport_s *self, const char *host, uint16_t port,
                        uint32_t timeout_ms)
{
    char cmd[160];
    (void)self;

    (void)net_platform_init();
    at_flush_rx();

    /* 1) 模组在线检测 */
    at_send("AT");
    if (at_wait_for("OK", AT_CMD_TO_MS) == 0) {
        return NET_ERR_CONNECT;
    }
    /* 2) 单连接模式 */
    if (g_wifi.cipmux_set == 0) {
        at_send("AT+CIPMUX=0");
        if (at_wait_for("OK", AT_CMD_TO_MS) == 0) {
            return NET_ERR_CONNECT;
        }
        g_wifi.cipmux_set = 1;
    }
    /* 3) 建立 TCP */
    (void)snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u", host, (unsigned)port);
    at_send(cmd);
    if (at_wait_for("CONNECT", timeout_ms) == 0) {
        return NET_ERR_CONNECT;
    }
    g_wifi.linked = 1;
    g_wifi.connects++;
    return NET_OK;
}

static int wifi_send(struct net_transport_s *self, const uint8_t *buf, size_t len)
{
    char cmd[32];
    (void)self;

    if (g_wifi.linked == 0) {
        return NET_ERR_CLOSED;
    }
    (void)snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u", (unsigned)len);
    at_flush_rx();
    at_send(cmd);
    if (at_wait_for(">", AT_CMD_TO_MS) == 0) {
        return NET_ERR_SOCKET;
    }
    if (uart_platform_iface()->write(buf, len) != (int)len) {
        return NET_ERR_SOCKET;
    }
    if (at_wait_for("SEND OK", AT_CMD_TO_MS) == 0) {
        return NET_ERR_SOCKET;
    }
    return (int)len;
}

static int wifi_recv(struct net_transport_s *self, uint8_t *buf, size_t cap, uint32_t timeout_ms)
{
    int n;
    (void)self;

    if (g_wifi.linked == 0) {
        return NET_ERR_CLOSED;
    }
    /* 模组透传/半透传下 +IPD 数据由驱动层剥离，这里直接读有效载荷 */
    n = uart_platform_iface()->read(buf, cap, timeout_ms);
    if (n < 0) {
        return NET_ERR_SOCKET;
    }
    return n;
}

static void wifi_close(struct net_transport_s *self)
{
    (void)self;
    if (g_wifi.linked != 0) {
        at_flush_rx();
        at_send("AT+CIPCLOSE");
        (void)at_wait_for("OK", 500u);
        g_wifi.linked = 0;
    }
}

static int wifi_is_open(const struct net_transport_s *self)
{
    (void)self;
    return g_wifi.linked;
}

static void wifi_set_timeout(struct net_transport_s *self, uint32_t timeout_ms)
{
    (void)self;
    (void)timeout_ms;
}

net_transport_t *net_wifi_at_get(void)
{
    g_wifi.base.name = "uart-wifi-at";
    g_wifi.base.connect = wifi_connect;
    g_wifi.base.send = wifi_send;
    g_wifi.base.recv = wifi_recv;
    g_wifi.base.close = wifi_close;
    g_wifi.base.is_open = wifi_is_open;
    g_wifi.base.set_timeout = wifi_set_timeout;
    return &g_wifi.base;
}
