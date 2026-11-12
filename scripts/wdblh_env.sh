#!/usr/bin/env bash
# Modified 2026-11-11: shared relocatable configuration for every shell launcher.
# Source this file. The selected configuration is trusted shell code.
if [[ ${_WDBLH_ENV_LOADED:-0} == 1 ]]; then return 0; fi
_WDBLH_ENV_LOADED=1
_wdblh_here=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
_wdblh_config=${WDBLH_ENV_FILE:-$_wdblh_here/config/wdblh.env}

# Preserve explicit environment settings over values from the config file.
declare -A _wdblh_overrides=()
while IFS= read -r _wdblh_name; do
    [[ $_wdblh_name == WDBLH_* ]] && _wdblh_overrides["$_wdblh_name"]=${!_wdblh_name}
done < <(compgen -e)
if [[ -f $_wdblh_config ]]; then
    set -a
    source "$_wdblh_config"
    set +a
elif [[ -n ${WDBLH_ENV_FILE:-} ]]; then
    echo "WDBLH_ENV_FILE does not exist: $_wdblh_config" >&2
    return 1
fi
for _wdblh_name in "${!_wdblh_overrides[@]}"; do
    export "$_wdblh_name=${_wdblh_overrides[$_wdblh_name]}"
done
export WDBLH_PROJECT=${WDBLH_PROJECT:-$_wdblh_here}
export WDBLH_CONTROL_BIN=${WDBLH_CONTROL_BIN:-$WDBLH_PROJECT/cmake_build_cpu/bin/rl_real_wdblh}
export WDBLH_POLICY_DIR=${WDBLH_POLICY_DIR:-$WDBLH_PROJECT/policy}
export WDBLH_LOG_DIR=${WDBLH_LOG_DIR:-$WDBLH_PROJECT/logs}
export WDBLH_BOOT_LOG=${WDBLH_BOOT_LOG:-$WDBLH_LOG_DIR/boot.log}
export WDBLH_BRIDGE_LOG=${WDBLH_BRIDGE_LOG:-$WDBLH_LOG_DIR/bridge.log}
export WDBLH_CONTROL_LOG=${WDBLH_CONTROL_LOG:-$WDBLH_LOG_DIR/controller.log}
export WDBLH_BRIDGE_USER=${WDBLH_BRIDGE_USER:-${SUDO_USER:-$(id -un)}}
export WDBLH_IMU_BAUD=${WDBLH_IMU_BAUD:-921600}
export WDBLH_MOTOR_BAUD=${WDBLH_MOTOR_BAUD:-4000000}
export WDBLH_CONTROL_CPU=${WDBLH_CONTROL_CPU:-4}
export WDBLH_RL_CPU=${WDBLH_RL_CPU:-5}
export WDBLH_BRIDGE_BIND=${WDBLH_BRIDGE_BIND:-0.0.0.0}
export WDBLH_BRIDGE_PORT=${WDBLH_BRIDGE_PORT:-8443}
if [[ -n ${WDBLH_LIBRARY_PATH:-} ]]; then
    export LD_LIBRARY_PATH="$WDBLH_LIBRARY_PATH${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
unset _wdblh_here _wdblh_name _wdblh_overrides _wdblh_config
