/*
 * net_socket.c -- PC 仿真下的 TCP 传输实现（BSD / POSIX socket）
 *
 * 目标板上走的是 USART + Wi-Fi 模组，但 WebSocket 协议层是一样的，
 * 因此这里提供同构的 net_transport_t，使 RFC6455 代码可在 PC 上完整验证。
 *
 * 语义约定：
 *   connect() 非阻塞 connect + select 超时，成功后切回阻塞模式并按需设置收发超时
 *   recv()    >0 实际字节；0 超时；NET_ERR_CLOSED 对端关闭；其它 <0 错误
 */
#include "net_transport.h"
#include "band_port.h"

#include <string.h>
#include <stdio.h>

#if defined(_WIN32)
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  define BANDSOCK          SOCKET
#  define BAND_INVALID      INVALID_SOCKET
#  define band_close_sock(s) closesocket(s)
#  define BAND_LASTERR      WSAGetLastError()
#  define BAND_EWOULDBLOCK  WSAEWOULDBLOCK
#  define BAND_EINPROGRESS  WSAEWOULDBLOCK
#else
#  include <sys/types.h>
#  include <sys/socket.h>
#  include <sys/select.h>
#  include <sys/time.h>
#  include <netinet/in.h>
#  include <netinet/tcp.h>
#  include <arpa/inet.h>
#  include <netdb.h>
#  include <unistd.h>
#  include <fcntl.h>
#  include <errno.h>
#  define BANDSOCK          int
#  define BAND_INVALID      (-1)
#  define band_close_sock(s) close(s)
#  define BAND_LASTERR      errno
#  define BAND_EWOULDBLOCK  EWOULDBLOCK
#  define BAND_EINPROGRESS  EINPROGRESS
#endif

typedef struct {
    net_transport_t base;
    BANDSOCK        fd;
    int             connected;
    uint32_t        timeout_ms;
} socket_tp_t;

static socket_tp_t g_sock;
static uint32_t g_connect_count;
#if defined(_WIN32)
static int g_wsa_inited;
#endif

int net_platform_init(void)
{
#if defined(_WIN32)
    WSADATA wsa;
    if (g_wsa_inited == 0) {
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
            return NET_ERR_SOCKET;
        }
        g_wsa_inited = 1;
    }
#endif
    return NET_OK;
}

void net_platform_deinit(void)
{
#if defined(_WIN32)
    if (g_wsa_inited != 0) {
        WSACleanup();
        g_wsa_inited = 0;
    }
#endif
}

uint32_t net_transport_connect_count(void)
{
    return g_connect_count;
}

void net_transport_reset_stats(void)
{
    g_connect_count = 0u;
}

static void sock_set_blocking(BANDSOCK fd, int blocking)
{
#if defined(_WIN32)
    u_long mode = (blocking != 0) ? 0u : 1u;
    (void)ioctlsocket(fd, FIONBIO, &mode);
#else
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        return;
    }
    if (blocking != 0) {
        flags = (int)(flags & ~O_NONBLOCK);
    } else {
        flags |= O_NONBLOCK;
    }
    (void)fcntl(fd, F_SETFL, flags);
#endif
}

static int sock_connect(struct net_transport_s *self, const char *host, uint16_t port,
                        uint32_t timeout_ms)
{
    socket_tp_t *s = (socket_tp_t *)self;
    struct addrinfo hints;
    struct addrinfo *res = NULL;
    struct addrinfo *it;
    char portstr[8];
    int rc = NET_ERR_CONNECT;

    if ((s == NULL) || (host == NULL)) {
        return NET_ERR_PARAM;
    }
    (void)net_platform_init();
    if (s->fd != BAND_INVALID) {
        band_close_sock(s->fd);
        s->fd = BAND_INVALID;
    }
    s->connected = 0;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    (void)snprintf(portstr, sizeof(portstr), "%u", (unsigned)port);

    if (getaddrinfo(host, portstr, &hints, &res) != 0) {
        return NET_ERR_CONNECT;
    }

    for (it = res; it != NULL; it = it->ai_next) {
        BANDSOCK fd;
        int one = 1;

        fd = socket(it->ai_family, it->ai_socktype, it->ai_protocol);
        if (fd == BAND_INVALID) {
            continue;
        }
        (void)setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (const char *)&one, sizeof(one));

        /* 非阻塞 connect + select 超时，避免断网时卡死整个固件 */
        sock_set_blocking(fd, 0);
        if (connect(fd, it->ai_addr, (int)it->ai_addrlen) == 0) {
            rc = NET_OK;
        } else {
            int err = BAND_LASTERR;
            if ((err == BAND_EINPROGRESS) || (err == BAND_EWOULDBLOCK)) {
                fd_set wfds;
                struct timeval tv;
                FD_ZERO(&wfds);
                FD_SET(fd, &wfds);
                tv.tv_sec = (long)(timeout_ms / 1000u);
                tv.tv_usec = (long)((timeout_ms % 1000u) * 1000u);
                if (select((int)(fd + 1), NULL, &wfds, NULL, &tv) > 0) {
                    int soerr = 0;
#if defined(_WIN32)
                    int slen = (int)sizeof(soerr);
#else
                    socklen_t slen = sizeof(soerr);
#endif
                    if (getsockopt(fd, SOL_SOCKET, SO_ERROR, (char *)&soerr, &slen) == 0 && soerr == 0) {
                        rc = NET_OK;
                    }
                } else {
                    rc = NET_ERR_TIMEOUT;
                }
            }
        }
        if (rc == NET_OK) {
            sock_set_blocking(fd, 1);
            s->fd = fd;
            s->connected = 1;
            g_connect_count++;
            break;
        }
        band_close_sock(fd);
    }
    freeaddrinfo(res);
    return rc;
}

static int sock_send(struct net_transport_s *self, const uint8_t *buf, size_t len)
{
    socket_tp_t *s = (socket_tp_t *)self;
    size_t sent = 0u;

    if ((s == NULL) || (s->fd == BAND_INVALID) || (s->connected == 0)) {
        return NET_ERR_CLOSED;
    }
    while (sent < len) {
#if defined(_WIN32)
        int n = send(s->fd, (const char *)&buf[sent], (int)(len - sent), 0);
#else
        ssize_t n = send(s->fd, &buf[sent], len - sent, 0);
#endif
        if (n <= 0) {
            s->connected = 0;
            return NET_ERR_CLOSED;
        }
        sent += (size_t)n;
    }
    return (int)sent;
}

static int sock_recv(struct net_transport_s *self, uint8_t *buf, size_t cap, uint32_t timeout_ms)
{
    socket_tp_t *s = (socket_tp_t *)self;
    fd_set rfds;
    struct timeval tv;
    int sel;

    if ((s == NULL) || (s->fd == BAND_INVALID) || (s->connected == 0)) {
        return NET_ERR_CLOSED;
    }
    if (cap == 0u) {
        return 0;
    }
    FD_ZERO(&rfds);
    FD_SET(s->fd, &rfds);
    tv.tv_sec = (long)(timeout_ms / 1000u);
    tv.tv_usec = (long)((timeout_ms % 1000u) * 1000u);
    sel = select((int)(s->fd + 1), &rfds, NULL, NULL, &tv);
    if (sel == 0) {
        return 0;   /* 超时无数据 */
    }
    if (sel < 0) {
        s->connected = 0;
        return NET_ERR_SOCKET;
    }
    {
#if defined(_WIN32)
        int n = recv(s->fd, (char *)buf, (int)cap, 0);
#else
        ssize_t n = recv(s->fd, buf, cap, 0);
#endif
        if (n == 0) {
            s->connected = 0;
            return NET_ERR_CLOSED;   /* 对端关闭 */
        }
        if (n < 0) {
            s->connected = 0;
            return NET_ERR_SOCKET;
        }
        return (int)n;
    }
}

static void sock_close(struct net_transport_s *self)
{
    socket_tp_t *s = (socket_tp_t *)self;

    if ((s != NULL) && (s->fd != BAND_INVALID)) {
        band_close_sock(s->fd);
        s->fd = BAND_INVALID;
    }
    if (s != NULL) {
        s->connected = 0;
    }
}

static int sock_is_open(const struct net_transport_s *self)
{
    const socket_tp_t *s = (const socket_tp_t *)self;
    return ((s != NULL) && (s->fd != BAND_INVALID) && (s->connected != 0)) ? 1 : 0;
}

static void sock_set_timeout(struct net_transport_s *self, uint32_t timeout_ms)
{
    socket_tp_t *s = (socket_tp_t *)self;
    if (s != NULL) {
        s->timeout_ms = timeout_ms;
    }
}

net_transport_t *net_socket_get(void)
{
    g_sock.base.name = "tcp-socket";
    g_sock.base.connect = sock_connect;
    g_sock.base.send = sock_send;
    g_sock.base.recv = sock_recv;
    g_sock.base.close = sock_close;
    g_sock.base.is_open = sock_is_open;
    g_sock.base.set_timeout = sock_set_timeout;
    if (g_sock.fd == 0 && g_sock.connected == 0) {
        g_sock.fd = BAND_INVALID;
    }
    if (g_sock.timeout_ms == 0u) {
        g_sock.timeout_ms = 1000u;
    }
    return &g_sock.base;
}
