#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""本地 WebSocket 服务端（mock），用于验证 C 客户端的握手 / 收发 / 断线重连 / 补传。

只依赖标准库（socket / base64 / hashlib / struct / argparse），不依赖 websockets 等第三方包。

行为（可按参数调整）：
  * 解析 HTTP/1.1 Upgrade 请求，校验 Sec-WebSocket-Key，返回 101 Switching Protocols
  * 严格按 RFC6455 解码客户端帧（并断言客户端帧必须带掩码，否则报协议错误）
  * 连接 #1：每收到一帧就回一个 ACK 帧；收到第 1 帧时额外下发一条 MSG_PARAM_SET
            （远程参数下发）；收到 --drop-after 帧后主动关闭连接（模拟网络中断）
  * 连接 #2 及以后：对每一帧回 ACK —— 客户端据此把离线缓存逐条清掉
  * 全程按序号去重，统计重复/乱序，退出时打印一行 JSON 摘要

用法：
  python tools/ws_mock_server.py --host 127.0.0.1 --port 9001 --drop-after 2
"""

import argparse
import base64
import hashlib
import json
import os
import signal
import socket
import struct
import sys
import time


class _StopServer(Exception):
    """收到 SIGTERM/SIGINT 时用来干净地跳出 accept 循环并打印 SUMMARY。"""
    pass


def _install_signal_handlers():
    def _handler(signum, frame):
        raise _StopServer()
    for sig in (signal.SIGTERM, signal.SIGINT):
        try:
            signal.signal(sig, _handler)
        except (ValueError, OSError):
            pass

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

OP_CONT, OP_TEXT, OP_BINARY = 0x0, 0x1, 0x2
OP_CLOSE, OP_PING, OP_PONG = 0x8, 0x9, 0xA

MSG_ACK = 0x10
MSG_PARAM_SET = 0x20

SOF0, SOF1 = 0xAA, 0x55


# ----------------------------------------------------------------------
# 应用层帧（与固件 frame_codec.c 完全一致）
# ----------------------------------------------------------------------
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def app_encode(msg_type: int, seq: int, payload: bytes) -> bytes:
    body = struct.pack("<BH", msg_type, seq) + struct.pack("<H", len(payload)) + payload
    crc = crc16_ccitt(body)
    return bytes([SOF0, SOF1]) + body + struct.pack("<H", crc)


class AppFrameParser:
    """与固件同构的帧同步状态机（半包/粘包都能处理）。"""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []
        while True:
            if len(self.buf) < 9:
                break
            if self.buf[0] != SOF0 or self.buf[1] != SOF1:
                self.buf.pop(0)
                continue
            msg_type = self.buf[2]
            seq = struct.unpack_from("<H", self.buf, 3)[0]
            length = struct.unpack_from("<H", self.buf, 5)[0]
            total = 9 + length
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:7 + length])
            rx_crc = struct.unpack_from("<H", self.buf, 7 + length)[0]
            del self.buf[:total]
            if crc16_ccitt(body) != rx_crc:
                out.append(("CRC_ERR", seq, b""))
                continue
            out.append((msg_type, seq, body[5:]))
        return out


# ----------------------------------------------------------------------
# RFC6455 帧
# ----------------------------------------------------------------------
def encode_frame(opcode: int, payload: bytes) -> bytes:
    """服务端 -> 客户端：不加掩码。"""
    n = len(payload)
    if n <= 125:
        head = bytes([0x80 | opcode, n])
    elif n <= 0xFFFF:
        head = bytes([0x80 | opcode, 126]) + struct.pack("!H", n)
    else:
        head = bytes([0x80 | opcode, 127]) + struct.pack("!Q", n)
    return head + payload


def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionResetError("peer closed")
        buf += chunk
    return buf


def read_frame(sock: socket.socket):
    """返回 (opcode, payload)；客户端帧必须带掩码，否则抛协议错误。"""
    b0, b1 = recv_exact(sock, 2)
    fin = b0 & 0x80
    opcode = b0 & 0x0F
    masked = b1 & 0x80
    length = b1 & 0x7F
    if length == 126:
        length = struct.unpack("!H", recv_exact(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", recv_exact(sock, 8))[0]
    if not masked:
        raise ValueError("客户端帧未使用掩码（违反 RFC6455 5.3）")
    key = recv_exact(sock, 4)
    data = bytearray(recv_exact(sock, length))
    for i in range(len(data)):
        data[i] ^= key[i % 4]
    if opcode >= 0x8 and fin == 0:
        raise ValueError("控制帧 FIN 必须为 1")
    return opcode, bytes(data)


def do_handshake(conn: socket.socket) -> str:
    req = b""
    while b"\r\n\r\n" not in req:
        chunk = conn.recv(1024)
        if not chunk:
            raise ConnectionResetError("closed during handshake")
        req += chunk
    text = req.decode("latin-1")
    lines = text.split("\r\n")
    request_line = lines[0] if lines else ""
    headers = {}
    for line in lines[1:]:
        if ":" in line:
            k, v = line.split(":", 1)
            headers[k.strip().lower()] = v.strip()

    key = headers.get("sec-websocket-key", "")
    accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
    resp = (
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: %s\r\n"
        "\r\n" % accept
    )
    conn.sendall(resp.encode())
    return "request_line=%s|key=%s|accept=%s|upgrade=%s|version=%s" % (
        request_line, key, accept,
        headers.get("upgrade", "?"), headers.get("sec-websocket-version", "?"))


# ----------------------------------------------------------------------
# 业务
# ----------------------------------------------------------------------
class Stats(object):
    def __init__(self):
        self.connections = 0
        self.frames_total = 0
        self.frames_by_conn = []
        self.seqs = []
        self.duplicates = 0
        self.order_violations = 0
        self.crc_errors = 0
        self.mask_ok = True
        self.param_payload = None


def build_param_set() -> bytes:
    """构造远程参数下发 TLV：心率周期 800ms / 灵敏度 72 / 上传周期 3000ms / 设备名"""
    tlv = b""
    tlv += bytes([0x01, 2]) + struct.pack("<H", 800)          # HR_INTERVAL
    tlv += bytes([0x02, 1, 72])                                # STEP_SENSITIVITY
    tlv += bytes([0x03, 2]) + struct.pack("<H", 3000)         # UPLOAD_INTERVAL
    tlv += bytes([0x04, 2]) + struct.pack("<H", 45)           # SLEEP_TIMEOUT
    name = b"MOCK-APP"
    tlv += bytes([0x09, len(name)]) + name                    # DEVICE_NAME
    return tlv


def serve(args, stats: Stats):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(4)
    srv.settimeout(args.timeout)
    print("[server] 监听 %s:%d（drop-after=%d, timeout=%.0fs）"
          % (args.host, args.port, args.drop_after, args.timeout), flush=True)

    deadline = time.time() + args.timeout
    while time.time() < deadline:
        try:
            conn, peer = srv.accept()
        except socket.timeout:
            break
        except _StopServer:
            break
        except OSError:
            break

        # 先完成握手再编号：脚本里的"端口就绪探测"会建立一次纯 TCP 连接然后立刻关闭，
        # 那种连接不算一次业务连接，否则会打乱 drop-after / ack-from-conn 的编号。
        try:
            conn.settimeout(3.0)
            hs = do_handshake(conn)
        except (ConnectionResetError, socket.timeout, ValueError, OSError) as exc:
            print("[server] 忽略未完成握手的连接 %s:%d（%s）"
                  % (peer[0], peer[1], exc), flush=True)
            try:
                conn.close()
            except OSError:
                pass
            continue

        stats.connections += 1
        cid = stats.connections
        frames_this_conn = 0
        parser = AppFrameParser()
        print("[server] --- 连接 #%d 来自 %s:%d ---" % (cid, peer[0], peer[1]), flush=True)
        try:
            print("[server] 握手完成: %s" % hs, flush=True)
            while True:
                opcode, payload = read_frame(conn)
                if opcode == OP_CLOSE:
                    print("[server] 收到 close 帧，回 close 并关闭", flush=True)
                    conn.sendall(encode_frame(OP_CLOSE, payload[:2]))
                    break
                if opcode == OP_PING:
                    conn.sendall(encode_frame(OP_PONG, payload))
                    continue
                if opcode == OP_PONG:
                    continue
                if opcode not in (OP_TEXT, OP_BINARY):
                    continue

                frames_this_conn += 1
                stats.frames_total += 1
                print("[server] 收到 WS 二进制帧 #%d：%d 字节（已解掩码）"
                      % (frames_this_conn, len(payload)), flush=True)

                for msg_type, seq, body in parser.feed(payload):
                    if msg_type == "CRC_ERR":
                        stats.crc_errors += 1
                        print("[server]   !! 应用帧 CRC 校验失败 seq=%d" % seq, flush=True)
                        continue
                    if seq in stats.seqs:
                        stats.duplicates += 1
                        print("[server]   !! 重复序号 seq=%d（补传去重失败？）" % seq, flush=True)
                    if stats.seqs and seq <= stats.seqs[-1]:
                        stats.order_violations += 1
                    stats.seqs.append(seq)
                    print("[server]   应用帧: type=0x%02X seq=%d len=%d" %
                          (msg_type, seq, len(body)), flush=True)

                # 第 1 帧后下发一条远程参数（验证双向通信）
                if cid == 1 and frames_this_conn == 1:
                    stats.param_payload = build_param_set()
                    app = app_encode(MSG_PARAM_SET, 1, stats.param_payload)
                    conn.sendall(encode_frame(OP_BINARY, app))
                    print("[server]   已下发 MSG_PARAM_SET（TLV %d 字节）"
                          % len(stats.param_payload), flush=True)
                    # 同时回一个 ACK
                    conn.sendall(encode_frame(OP_BINARY,
                                              app_encode(MSG_ACK, 1, bytes([0, 1, 0]))))
                    print("[server]   已回 MSG_ACK(seq=1)", flush=True)

                # 连接 #2 起对每一帧回 ACK（客户端据此清理离线缓存）
                if cid >= args.ack_from_conn:
                    if not (cid == 1 and frames_this_conn == 1):
                        last_seq = stats.seqs[-1] if stats.seqs else 0
                        conn.sendall(encode_frame(OP_BINARY,
                                                  app_encode(MSG_ACK, last_seq,
                                                             bytes([0, last_seq & 0xFF,
                                                                    (last_seq >> 8) & 0xFF]))))
                        print("[server]   已回 MSG_ACK(seq=%d)" % last_seq, flush=True)

                if args.drop_after > 0 and cid == 1 and frames_this_conn >= args.drop_after:
                    print("[server] 达到 --drop-after=%d，主动断开连接 #%d（模拟断网；"
                          "只在 #1 连接上断开，重连后不再断）" % (args.drop_after, cid), flush=True)
                    break
        except (ConnectionResetError, socket.timeout, ValueError, OSError) as exc:
            print("[server] 连接 #%d 结束: %s" % (cid, exc), flush=True)
        finally:
            stats.frames_by_conn.append(frames_this_conn)
            try:
                conn.close()
            except OSError:
                pass

        if stats.connections >= args.max_connections:
            print("[server] 达到 --max-connections=%d，停止接受新连接"
                  % args.max_connections, flush=True)
            break

        # 客户端已经断开且服务够了预期连接数：直接收尾并打印 SUMMARY，
        # 不再空等 accept 超时（脚本里就能拿到统计结果）。
        if args.stop_after_connections > 0 and stats.connections >= args.stop_after_connections:
            print("[server] 已完成 %d 个连接，收尾并输出统计"
                  % stats.connections, flush=True)
            break

    srv.close()

    summary = {
        "connections": stats.connections,
        "frames_by_conn": stats.frames_by_conn,
        "frames_total": stats.frames_total,
        "unique_seqs": len(set(stats.seqs)),
        "duplicates": stats.duplicates,
        "order_violations": stats.order_violations,
        "crc_errors": stats.crc_errors,
        "seqs": stats.seqs,
        "param_pushed": stats.param_payload is not None,
    }
    print("[server] 收到的应用帧序号序列: %s" % stats.seqs, flush=True)
    print("[server] SUMMARY_JSON %s" % json.dumps(summary, ensure_ascii=False), flush=True)
    return 0 if (stats.duplicates == 0 and stats.crc_errors == 0
                 and stats.order_violations == 0) else 1


def main():
    ap = argparse.ArgumentParser(description="本地 WebSocket mock 服务端（纯标准库）")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9001)
    ap.add_argument("--drop-after", type=int, default=2,
                    help="连接 #1 收满该帧数后主动断开（0 = 不断开）")
    ap.add_argument("--ack-from-conn", type=int, default=2,
                    help="从第几个连接开始对每帧回 ACK（默认 2，即只确认补传）")
    ap.add_argument("--max-connections", type=int, default=6,
                    help="最多接受多少个连接后停止（防客户端异常时无限重连）")
    ap.add_argument("--stop-after-connections", type=int, default=2,
                    help="完成这么多连接后直接收尾并打印统计（0 = 一直等到超时）")
    ap.add_argument("--timeout", type=float, default=25.0, help="整体运行时长上限（秒）")
    args = ap.parse_args()
    stats = Stats()
    _install_signal_handlers()
    try:
        return serve(args, stats)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
