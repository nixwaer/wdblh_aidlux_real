#!/usr/bin/env bash
# Modified 2026-11-11: WDBLH-only portable configuration and startup.
# 解析 WDBLH 串口并 exec 控制器。给 systemd / 手动启动 共用。
# 端口解析优先级：显式环境变量 > udev 软链 > sysfs 按 USB VID:PID 探测 > 默认值
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/wdblh_env.sh"

PROJECT=${WDBLH_PROJECT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}
CONTROL_BIN=${WDBLH_CONTROL_BIN:-$PROJECT/cmake_build_cpu/bin/rl_real_wdblh}
IMU_BAUD=${WDBLH_IMU_BAUD:-921600}
MOTOR_BAUD=${WDBLH_MOTOR_BAUD:-4000000}

find_tty_by_usb_id() {  # $1=vid $2=pid
    local vid=$1 pid=$2 tty dev cur i v p up
    for tty in /sys/class/tty/tty*; do
        dev=$tty/device
        [ -e "$dev" ] || continue
        cur=$(readlink -f "$dev")
        for i in 1 2 3 4 5 6; do
            v=$(cat "$cur/idVendor" 2>/dev/null)
            p=$(cat "$cur/idProduct" 2>/dev/null)
            if [ -n "$v" ] && [ -n "$p" ]; then
                if [ "${v,,}" = "${vid,,}" ] && [ "${p,,}" = "${pid,,}" ]; then
                    echo "/dev/${tty##*/}"; return 0
                fi
                break
            fi
            up=$(dirname "$cur")
            [ "$up" = "$cur" ] && break
            cur=$up
        done
    done
    return 1
}

resolve_port() {  # $1=软链 $2=vid $3=pid $4=默认
    if [ -e "$1" ]; then echo "$1"; return 0; fi
    local found; found=$(find_tty_by_usb_id "$2" "$3") || true
    [ -n "$found" ] && { echo "$found"; return 0; }
    echo "$4"
}

IMU_PORT=${WDBLH_IMU_PORT:-$(resolve_port /dev/wdblh_imu 6877 4d55 /dev/ttyACM0)}
MOTOR_PORT=${WDBLH_MOTOR_PORT:-$(resolve_port /dev/wdblh_motor 1a86 55d3 /dev/ttyCH343USB0)}

[ -x "$CONTROL_BIN" ] || { echo "ERROR: $CONTROL_BIN 不存在或不可执行" >&2; exit 1; }
echo "[run_wdblh] $CONTROL_BIN $IMU_PORT $MOTOR_PORT $IMU_BAUD $MOTOR_BAUD" >&2
exec "$CONTROL_BIN" "$IMU_PORT" "$MOTOR_PORT" "$IMU_BAUD" "$MOTOR_BAUD"
