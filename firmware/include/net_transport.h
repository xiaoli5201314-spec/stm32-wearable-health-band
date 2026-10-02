/*
 * net_transport.h -- TCP 传输抽象（PC socket / STM32 + Wi-Fi 模组 AT 指令）
 *
 * WebSocket 客户端只依赖本抽象，因此同一份 RFC6455 实现：
 *   - PC 仿真：直接走 BSD/POSIX socket（platform/net_socket.c）
 *   - 目标板：走 USART + Wi-Fi 模组 AT 透传（platform/net_wifi_at.c）
 */
#ifndef NET_TRANSPORT_H
#define NET_TRANSPORT_H

#include <stdint.h>
#include <stddef.h>

#define NET_OK              0
#define NET_ERR_PARAM     (-1)
#define NET_ERR_SOCKET    (-2)
#define NET_ERR_CONNECT   (-3)
#define NET_ERR_TIMEOUT   (-4)
#define NET_ERR_CLOSED    (-5)
#define NET_ERR_NOTIMPL   (-6)
#define NET_ERR_AGAIN     (-7)

typedef struct net_transport_s {
    const char *name;
    /* 建立连接；timeout_ms 为整体超时 */
    int  (*connect)(struct net_transport_s *self, const char *host, uint16_t port,
                    uint32_t timeout_ms);
    /* 发送全部 len 字节，成功返回 len */
    int  (*send)(struct net_transport_s *self, const uint8_t *buf, size_t len);
    /* 接收：>0 实际字节；0 = 超时无数据；NET_ERR_CLOSED = 对端关闭；其它 <0 出错 */
    int  (*recv)(struct net_transport_s *self, uint8_t *buf, size_t cap, uint32_t timeout_ms);
    void (*close)(struct net_transport_s *self);
    int  (*is_open)(const struct net_transport_s *self);
    void (*set_timeout)(struct net_transport_s *self, uint32_t timeout_ms);
} net_transport_t;

/* ---- 具体实现工厂 ---- */
net_transport_t *net_socket_get(void);     /* PC: BSD socket */
net_transport_t *net_wifi_at_get(void);    /* STM32: USART + AT 透传 */

/* 平台初始化（Windows 需要 WSAStartup） */
int  net_platform_init(void);
void net_platform_deinit(void);

/* 已连接计数/重连计数，便于测试断言 */
uint32_t net_transport_connect_count(void);
void     net_transport_reset_stats(void);

#endif /* NET_TRANSPORT_H */
