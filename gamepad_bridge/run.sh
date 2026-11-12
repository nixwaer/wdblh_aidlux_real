#!/usr/bin/env bash
# Modified 2026-11-11: WDBLH-only portable configuration and startup.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../scripts/wdblh_env.sh"

bridge_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
deps_dir="$bridge_dir/.deps"

if ! PYTHONPATH="$deps_dir" python3 -c "import aiohttp, cryptography, qrcode" 2>/dev/null; then
  echo "首次运行，正在安装隔离依赖到 $deps_dir"
  python3 -m pip install --target "$deps_dir" -r "$bridge_dir/requirements.txt"
fi

exec env PYTHONPATH="$deps_dir${PYTHONPATH:+:$PYTHONPATH}" \
  python3 "$bridge_dir/server.py" "$@"
