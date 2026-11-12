#!/usr/bin/env bash
# Modified 2026-11-11: install serial aliases only; service install is separate.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [[ ${1:-} == --dry-run ]]; then cat "$HERE/99-wdblh.rules"; exit 0; fi
[[ $# == 0 ]] || { echo "Usage: $0 [--dry-run]" >&2; exit 2; }
[[ $(id -u) == 0 ]] || { echo "Use sudo to install udev rules." >&2; exit 1; }
install -m 0644 "$HERE/99-wdblh.rules" /etc/udev/rules.d/99-wdblh.rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty --action=add
echo "Installed serial aliases only. No service was installed or started."
