#!/usr/bin/env bash
# Common helpers for the scripts that start Unity (source this file).
#
#   source "$(dirname "$0")/unity_lib.sh"
#   UNITY_BIN="$(find_unity_bin)"
#   prepare_unity_env

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Unity version required by the project. ProjectSettings/ProjectVersion.txt is the
# source of truth: opening the project with another version upgrades scenes and
# prefabs silently.
unity_version() {
    sed -n 's/^m_EditorVersion: //p' "${REPO_ROOT}/ProjectSettings/ProjectVersion.txt"
}

# Path of the Unity Editor binary. Override with UNITY_BIN=/path/to/Unity.
find_unity_bin() {
    local version bin
    version="$(unity_version)"
    if [[ -n "${UNITY_BIN:-}" ]]; then
        echo "${UNITY_BIN}"
        return 0
    fi
    bin="${HOME}/Unity/Hub/Editor/${version}/Editor/Unity"
    if [[ ! -x "${bin}" ]]; then
        bin="$(find "${HOME}" /opt -maxdepth 6 -type f -name Unity -path "*${version}*" -perm -u+x 2>/dev/null | head -1)"
    fi
    if [[ -z "${bin}" || ! -x "${bin}" ]]; then
        echo "!! Unity ${version} not found. Install it with Unity Hub or set UNITY_BIN." >&2
        return 1
    fi
    echo "${bin}"
}

# Environment for Unity:
#  * Remove ROS 2 from the environment. Ros2ForUnity is a "standalone" build that ships
#    its own ROS 2 Humble libraries; with /opt/ros/humble sourced, the host libraries
#    are loaded first and Unity crashes or silently publishes nothing.
#  * Lower the open file limit. With a very large `ulimit -n` (1048576 on some Ubuntu
#    setups) Unity's Mono aborts with "Assertion `fd < sysconf(_SC_OPEN_MAX)' failed".
prepare_unity_env() {
    local strip
    strip() {
        local IFS=':' out=() p
        for p in ${1:-}; do
            [[ "${p}" == /opt/ros/* ]] && continue
            out+=("${p}")
        done
        local IFS=':'
        echo "${out[*]:-}"
    }
    export LD_LIBRARY_PATH="$(strip "${LD_LIBRARY_PATH:-}")"
    export PYTHONPATH="$(strip "${PYTHONPATH:-}")"
    export PATH="$(strip "${PATH}")"
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION CMAKE_PREFIX_PATH
    if [[ "$(ulimit -n)" -gt 65536 ]]; then
        ulimit -n 65536
    fi
}

# DDS settings shared by AWSIM and the ROS 2 tools of this repository.
prepare_dds_env() {
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
    export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${REPO_ROOT}/config/cyclonedds.xml}"
}

# Unity opens a project in one process only. A batchmode run against an open project
# waits for the lock forever without a message, so refuse early.
ensure_editor_closed() {
    if pgrep -f "^[^ ]*Editor/Unity .*-projectPath ${REPO_ROOT}( |$)" >/dev/null 2>&1; then
        echo "!! The Unity Editor has this project open. Close it first." >&2
        return 1
    fi
    rm -f "${REPO_ROOT}/Temp/UnityLockfile"
}

# Run a static method of an editor script in batchmode.
#   unity_batch <log file> -executeMethod Class.Method [extra args...]
unity_batch() {
    local log="$1"
    shift
    local bin
    bin="$(find_unity_bin)" || return 1
    ensure_editor_closed || return 1
    prepare_unity_env
    mkdir -p "$(dirname "${log}")"
    echo "Unity: ${bin}"
    echo "Log  : ${log}"
    "${bin}" -batchmode -nographics -quit -projectPath "${REPO_ROOT}" -logFile "${log}" "$@"
}
