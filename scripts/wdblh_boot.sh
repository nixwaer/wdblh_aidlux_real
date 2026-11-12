#!/bin/bash
# Modified 2026-11-11: WDBLH-only portable configuration and startup.
# WDBLH unified boot launcher
#   1. Start the HTTPS/WebSocket browser gamepad bridge as the configured non-root user.
#   2. Start the WDBLH motor controller as root.
# Called by wdblh.service; it is also safe to run manually with sudo.

set -eu
source "$(dirname "${BASH_SOURCE[0]}")/wdblh_env.sh"

PROJECT=${WDBLH_PROJECT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}
BRIDGE_DIR=$PROJECT/gamepad_bridge
BRIDGE_RUN=$BRIDGE_DIR/run.sh
BRIDGE_SERVER=$BRIDGE_DIR/server.py
MONITOR=$BRIDGE_DIR/himloco_monitor.py
BRIDGE_USER=$WDBLH_BRIDGE_USER
CONTROL_BIN=${WDBLH_CONTROL_BIN:-$PROJECT/cmake_build_cpu/bin/rl_real_wdblh}
CONTROL_DAEMON=$BRIDGE_DIR/wdblh_control_daemon.py

BOOT_LOG=$WDBLH_BOOT_LOG
BRIDGE_LOG=$WDBLH_BRIDGE_LOG
CONTROL_LOG=$WDBLH_CONTROL_LOG
if [[ $(id -u "$BRIDGE_USER") == 0 ]]; then
    echo "Set WDBLH_BRIDGE_USER to an existing non-root account." >&2
    exit 1
fi
BRIDGE_GROUP=$(id -gn "$BRIDGE_USER")
mkdir -p "$(dirname "$BOOT_LOG")" "$(dirname "$BRIDGE_LOG")" "$(dirname "$CONTROL_LOG")"

# The system service runs as root. Manual starts use sudo once at script level;
# do not use nested "sudo -n", which used to fail without a cached password.
if [ "$(id -u)" -ne 0 ]; then
    echo "Please run: sudo $0"
    exit 1
fi

for required in "$CONTROL_BIN" "$BRIDGE_RUN" "$CONTROL_DAEMON" "$PROJECT/scripts/run_wdblh.sh"; do
    if [ ! -f "$required" ]; then
        echo "ERROR: required file missing: $required" >&2
        exit 1
    fi
done
if [ ! -x "$CONTROL_BIN" ] || [ ! -x "$BRIDGE_RUN" ]; then
    echo "ERROR: controller or gamepad launcher is not executable" >&2
    exit 1
fi

# Prevent systemd and a manual invocation from racing during the startup wait.
exec 9>/run/wdblh_boot.lock
if ! flock -n 9; then
    echo "$(date '+%F %T') WDBLH autostart is already in progress" >>"$BOOT_LOG"
    exit 0
fi

echo "启动进度写入 $BOOT_LOG；先等待 USB 10 秒，Wi-Fi 最多等待 30 秒。"
echo "另开终端查看：tail -n 30 -F $BOOT_LOG"
exec >>"$BOOT_LOG" 2>&1

log() {
    echo "$(date '+%F %T') $*"
}

process_running() {
    pgrep -f "$1" >/dev/null 2>&1
}

log "=== WDBLH unified autostart ==="

# Give USB and networking time to settle. The controller also performs its own
# serial-device retries after this delay.
sleep 10

# Root-only local controller used by the unprivileged gamepad bridge. It owns
# no HTTP port and accepts only fixed commands over /run/wdblh-control.sock.
if process_running "$CONTROL_DAEMON"; then
    log "WDBLH control daemon already running"
else
    log "Starting WDBLH control daemon"
    nohup python3 "$CONTROL_DAEMON" >>"$BOOT_LOG" 2>&1 </dev/null 9>&- &
fi

# ── 1. Browser gamepad bridge ─────────────────────────────
if process_running "$BRIDGE_SERVER"; then
    log "Gamepad bridge already running"
else
    # Prefer the Wi-Fi address used by the phone. Wait up to 30 seconds for
    # NetworkManager/DHCP; fall back to server.py auto-detection if unavailable.
    gamepad_host=${WDBLH_BRIDGE_HOST:-}
    if [[ -z $gamepad_host && -n ${WDBLH_NETWORK_INTERFACE:-} ]]; then
        for _attempt in $(seq 1 30); do
            gamepad_host=$(ip -4 -o addr show dev "$WDBLH_NETWORK_INTERFACE" scope global 2>/dev/null \
                | awk 'NR==1 {split($4, address, "/"); print address[1]}')
            [[ -n $gamepad_host ]] && break
            sleep 1
        done
    fi

    touch "$BRIDGE_LOG"
    chown "$BRIDGE_USER:$BRIDGE_GROUP" "$BRIDGE_LOG"
    if [ -n "$gamepad_host" ]; then
        log "Starting gamepad bridge at https://$gamepad_host:$WDBLH_BRIDGE_PORT/"
        runuser --preserve-environment -u "$BRIDGE_USER" -- nohup "$BRIDGE_RUN" --host "$gamepad_host" \
            >>"$BRIDGE_LOG" 2>&1 </dev/null 9>&- &
    else
        log "WARNING: Wi-Fi IP unavailable; starting bridge with auto-detection"
        runuser --preserve-environment -u "$BRIDGE_USER" -- nohup "$BRIDGE_RUN" \
            >>"$BRIDGE_LOG" 2>&1 </dev/null 9>&- &
    fi

    bridge_ready=false
    for _attempt in $(seq 1 15); do
        if process_running "$BRIDGE_SERVER"; then
            bridge_ready=true
            break
        fi
        sleep 1
    done
    if $bridge_ready; then
        log "Gamepad bridge started"
    else
        log "ERROR: gamepad bridge did not start; see $BRIDGE_LOG"
        exit 1
    fi
fi

# ── 2. WDBLH motion controller ────────────────────────────────
controller_started=false
if process_running "$CONTROL_BIN"; then
    log "WDBLH controller already running"
elif process_running "$MONITOR"; then
    # Both programs bind localhost UDP 9877. Starting the controller while the
    # standalone monitor owns it would silently disable browser control.
    log "ERROR: himloco_monitor.py is using UDP 9877; controller not started"
    exit 1
else
    log "Starting WDBLH controller: $CONTROL_BIN"
    WDBLH_CONTROL_BIN="$CONTROL_BIN" bash "$PROJECT/scripts/run_wdblh.sh" \
        >>"$CONTROL_LOG" 2>&1 </dev/null 9>&- &
    controller_pid=$!
    sleep 3
    if kill -0 "$controller_pid" 2>/dev/null; then
        controller_started=true
        log "WDBLH controller started (PID $controller_pid)"
    else
        log "ERROR: WDBLH controller exited during startup; see $CONTROL_LOG"
        exit 1
    fi
fi

# Logs are files; desktop applications are not part of the boot chain.

log "=== WDBLH unified autostart finished ==="
# Explicitly unlock as well as closing fd 9. Background children already close
# fd 9 above, so no long-running service can keep the startup lock alive.
flock -u 9
