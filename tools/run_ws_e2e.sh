#!/bin/sh
# 端到端联调脚本：启动本地 WebSocket mock 服务端 -> 跑 C 客户端 -> 比对并打印统计
#
# 用法（在仓库任意位置执行）：
#   sh tools/run_ws_e2e.sh [客户端可执行文件] [端口]
# 或通过 make：
#   make -C firmware live-test
#
# 说明：mock server 与 C 客户端都在同一台 Linux/WSL 主机上跑，
# 客户端用 127.0.0.1:<port> 直连，不需要任何端口转发。
# 全部只用 Python 标准库 + POSIX shell。
set -u

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
ROOT=$(dirname "$SCRIPT_DIR")
cd "$ROOT" || exit 1

CLIENT="${1:-firmware/build/ws_live_client}"
PORT="${2:-9001}"
HOST="127.0.0.1"
PY="${PYTHON:-python3}"
LOG_SRV="${TMPDIR:-/tmp}/ws_mock_server_$$.log"
LOG_CLI="${TMPDIR:-/tmp}/ws_live_client_$$.log"
RC=0

case "$CLIENT" in
    /*) ;;                       # 绝对路径
    *)  CLIENT="$ROOT/$CLIENT" ;; # 相对仓库根
esac

if [ ! -x "$CLIENT" ]; then
    echo "找不到客户端可执行文件: $CLIENT（先执行 make -C firmware build/ws_live_client）"
    exit 1
fi

echo "=== 启动 mock WebSocket 服务端（纯标准库，无第三方依赖）==="
"$PY" tools/ws_mock_server.py --host "$HOST" --port "$PORT" \
      --drop-after 2 --ack-from-conn 2 --timeout 25 > "$LOG_SRV" 2>&1 &
SRV_PID=$!

# 等端口就绪（最多 5 秒）
i=0
while [ $i -lt 50 ]; do
    if "$PY" - "$HOST" "$PORT" <<'PYEOF'
import socket, sys
s = socket.socket()
s.settimeout(0.2)
try:
    s.connect((sys.argv[1], int(sys.argv[2])))
    sys.exit(0)
except Exception:
    sys.exit(1)
finally:
    s.close()
PYEOF
    then
        break
    fi
    i=$((i + 1))
    sleep 0.1
done
echo "=== 服务端已就绪（pid=$SRV_PID, ${HOST}:${PORT}）==="
echo

echo "=== 运行 C 客户端（真实 TCP + RFC6455 握手 + 掩码编解码）==="
"$CLIENT" "$HOST" "$PORT" 2>&1 | tee "$LOG_CLI"

if grep -q "自检结论: PASS" "$LOG_CLI"; then
    CLI_RC=0
else
    CLI_RC=1
fi

# 等服务端自然结束（accept 超时或 break）
sleep 1
if kill -0 "$SRV_PID" 2>/dev/null; then
    kill "$SRV_PID" 2>/dev/null
fi
wait "$SRV_PID" 2>/dev/null
SRV_RC=$?

# 服务端只要没因为重复/乱序/CRC 出错而返回 1 就算通过；被 kill 视为正常收尾
case "$SRV_RC" in
    0|143|1) SRV_RC_WAS=$SRV_RC ;;
    *)       SRV_RC_WAS=$SRV_RC ;;
esac

echo
echo "=== mock 服务端完整输出 ==="
cat "$LOG_SRV"

echo
echo "=== mock 服务端 SUMMARY ==="
grep "SUMMARY_JSON" "$LOG_SRV" || echo "（未拿到 SUMMARY_JSON）"

echo
echo "=== 结果 ==="
echo "客户端判定: $([ $CLI_RC -eq 0 ] && echo PASS || echo FAIL)"
if grep -q '"duplicates": 0' "$LOG_SRV" && grep -q '"order_violations": 0' "$LOG_SRV" \
   && grep -q '"crc_errors": 0' "$LOG_SRV"; then
    echo "服务端判定: PASS（无重复序号 / 无乱序 / 无 CRC 错误）"
else
    echo "服务端判定: FAIL（存在重复序号、乱序或 CRC 错误）"
    RC=1
fi
echo "（服务端日志: $LOG_SRV）"

if [ $CLI_RC -ne 0 ]; then
    RC=1
fi
rm -f "$LOG_CLI"
exit $RC
