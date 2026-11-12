#!/usr/bin/env bash
# Modified 2026-11-11: render paths/user before optional installation; no start.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/wdblh_env.sh"
dry_run=0
enable_service=1
while [[ $# -gt 0 ]]; do
    case $1 in
        --dry-run) dry_run=1; shift ;;
        --no-enable) enable_service=0; shift ;;
        --user) export WDBLH_BRIDGE_USER=${2:?--user requires an account}; shift 2 ;;
        *) echo "Usage: $0 [--user ACCOUNT] [--dry-run] [--no-enable]" >&2; exit 2 ;;
    esac
done
[[ $(id -u "$WDBLH_BRIDGE_USER") != 0 ]] || {
    echo "Specify an existing non-root bridge account with --user." >&2; exit 1;
}
staged=$(mktemp -d)
trap 'rm -rf "$staged"' EXIT
render_args=(--project "$WDBLH_PROJECT" --user "$WDBLH_BRIDGE_USER")
if [[ -n ${WDBLH_ENV_FILE:-} ]]; then render_args+=(--env-file "$WDBLH_ENV_FILE"); fi
python3 "$WDBLH_PROJECT/scripts/render_wdblh_service.py" "${render_args[@]}" >"$staged/wdblh.service"
if [[ $dry_run == 1 ]]; then
    cat "$staged/wdblh.service"
    exit 0
fi
[[ $(id -u) == 0 ]] || { echo "Installation requires sudo; --dry-run does not." >&2; exit 1; }
for file in "$WDBLH_CONTROL_BIN" "$WDBLH_PROJECT/scripts/wdblh_boot.sh" \
    "$WDBLH_PROJECT/gamepad_bridge/run.sh" "$WDBLH_PROJECT/gamepad_bridge/wdblh_control_daemon.py"; do
    [[ -f $file ]] || { echo "Missing: $file" >&2; exit 1; }
done
if [[ -f /etc/rc.local ]] && grep -Eq '^[[:space:]]*[^#[:space:]].*(wdblh_boot\.sh|rl_real_wdblh)' /etc/rc.local; then
    echo "Remove the duplicate WDBLH launcher in /etc/rc.local first." >&2; exit 1
fi
systemd-analyze verify "$staged/wdblh.service"
unit=/etc/systemd/system/wdblh.service
if [[ -e $unit || -L $unit ]]; then cp -a "$unit" "$unit.backup.$(date +%Y%m%d_%H%M%S)"; fi
unit_stage=$(mktemp /etc/systemd/system/.wdblh.service.XXXXXX)
trap 'rm -rf "$staged"; rm -f "${unit_stage:-}"' EXIT
install -m 0644 "$staged/wdblh.service" "$unit_stage"
mv -fT "$unit_stage" "$unit"
systemctl unmask --runtime wdblh.service
systemctl daemon-reload
if [[ $enable_service == 1 ]]; then systemctl enable wdblh.service; fi
echo "Installed service. No controller was started or restarted. Enable requested: $enable_service"
echo "Boot log: $WDBLH_BOOT_LOG"
echo "Controller log: $WDBLH_CONTROL_LOG"
