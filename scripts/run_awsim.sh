#!/usr/bin/env bash
# Start the built AWSIM for WHILL player (see scripts/build_player.sh).
#
#   scripts/run_awsim.sh            # foreground, Ctrl-C to stop
#   BG=1 scripts/run_awsim.sh       # background, waits until ROS 2 is up
#   scripts/run_awsim.sh --stop     # stop a background instance
#
# Environment: ROS_DOMAIN_ID (default 0), AWSIM_BIN (path of the player),
#              AWSIM_ARGS (extra Unity player arguments).
set -euo pipefail
source "$(dirname "$0")/unity_lib.sh"

BIN="${AWSIM_BIN:-${REPO_ROOT}/Builds/AWSIM_for_WHILL/AWSIM_for_WHILL.x86_64}"
LOG="${AWSIM_LOG:-${REPO_ROOT}/Logs/awsim_player.log}"

running_pids() { pgrep -f "^${BIN}" 2>/dev/null || true; }

if [[ "${1:-}" == "--stop" ]]; then
    pids="$(running_pids)"
    [[ -z "${pids}" ]] && { echo "AWSIM is not running."; exit 0; }
    # shellcheck disable=SC2086
    kill ${pids} 2>/dev/null || true
    for _ in $(seq 1 20); do [[ -z "$(running_pids)" ]] && break; sleep 0.5; done
    # shellcheck disable=SC2046
    [[ -n "$(running_pids)" ]] && kill -9 $(running_pids) 2>/dev/null || true
    echo "Stopped."
    exit 0
fi

if [[ ! -x "${BIN}" ]]; then
    echo "!! ${BIN} not found. Build it first: scripts/build_player.sh" >&2
    exit 1
fi
# Two instances publish the same topics (/ground_truth/odom, ...) and listen on the
# same OptimizerBridge port, so every client would silently read mixed data.
if [[ -n "$(running_pids)" ]]; then
    echo "!! AWSIM is already running (PID $(running_pids)). Stop it: scripts/run_awsim.sh --stop" >&2
    exit 1
fi

prepare_unity_env
prepare_dds_env
mkdir -p "$(dirname "${LOG}")"
echo "AWSIM         : ${BIN}"
echo "Log           : ${LOG}"
echo "ROS_DOMAIN_ID : ${ROS_DOMAIN_ID}"
echo "RMW           : ${RMW_IMPLEMENTATION}"
echo "DISPLAY       : ${DISPLAY:-(not set)}"

# shellcheck disable=SC2086
ARGS=(-logFile "${LOG}" -screen-fullscreen 0 -screen-width 1280 -screen-height 720 ${AWSIM_ARGS:-})
if [[ "${BG:-0}" == "1" ]]; then
    nohup "${BIN}" "${ARGS[@]}" >/dev/null 2>&1 &
    echo "Started in background (PID $!). Waiting for OptimizerBridge (127.0.0.1:50051)..."
    for _ in $(seq 1 180); do
        if ss -ltn 2>/dev/null | grep -q '127.0.0.1:50051 '; then
            echo "Ready."
            exit 0
        fi
        sleep 1
    done
    echo "!! AWSIM did not become ready in 180 s. See ${LOG}" >&2
    exit 1
fi
exec "${BIN}" "${ARGS[@]}"
