#!/usr/bin/env bash
# Measure RAM, VRAM, CPU and frame rate of AWSIM for 1-16 infrastructure LiDARs and 1-16 WHILLs
# (docs/RESOURCE_USAGE.md). Uses the default player (Builds/AWSIM_for_WHILL); the copies are
# made at start-up by Assets/Awsim/Scripts/Benchmark/ResourceBenchmarkSpawner.cs.
#
#   scripts/bench_resources.sh                 # every condition of config/resource_bench.yaml (~25 min)
#   scripts/bench_resources.sh --only 4,1      # one condition (LiDARs,WHILLs)
#   scripts/bench_resources.sh --report DIR    # summary and figures again from a finished run
#
# Per condition: start AWSIM with AWSIM_BENCH_*, wait until it is ready, command all WHILLs to
# spin in place, check the sensor topics, record the AWSIM process for `measure` s after
# `warmup` s, stop AWSIM. Close other AWSIM instances first (their load would be measured too).
# Needs on the host: python3 with psutil, nvidia-smi. Output: logs/resource_bench/<date>/.
set -euo pipefail
source "$(dirname "$0")/unity_lib.sh"
cd "${REPO_ROOT}"
CONFIG="${REPO_ROOT}/config/resource_bench.yaml"
ONLY=""
REPORT=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --only) ONLY="$2"; shift 2 ;;
        --report) REPORT="$2"; shift 2 ;;
        -h|--help) sed -n 2,14p "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

report() {   # matplotlib / yaml from the RL image; the repository is /workspace in the container
    local dir
    dir="$(realpath --relative-to="${REPO_ROOT}" "$1")"
    IMAGE=awsim-for-whill/dreamer scripts/ros2_shell.sh python3 scripts/bench/report.py "${dir}"
}
if [[ -n "${REPORT}" ]]; then
    report "${REPORT}"
    exit 0
fi

eval "$(python3 - "${CONFIG}" <<'EOF'
import sys, yaml
b = yaml.safe_load(open(sys.argv[1]))["bench"]
t, c = b["timing"], b["command"]
print('CONDITIONS="' + " ".join(f"{i},{w}" for i, w in b["conditions"]) + '"')
print(f'LIDAR_SPACING={b["lidar_spacing"]}')
print(f'VEHICLE_LIDAR={1 if b.get("vehicle_lidar", True) else 0}')
print('WHILL_POSITIONS="' + ";".join(f"{f}:{r}" for f, r in b["whill_positions"]) + '"')
print(f'CMD_LINEAR={c["linear"]}; CMD_ANGULAR={c["angular"]}')
for k in ["settle", "warmup", "topic_check_after", "topic_check_duration", "measure", "sample_interval", "cooldown"]:
    print(f'T_{k.upper()}={t[k]}')
EOF
)"
[[ -n "${ONLY}" ]] && CONDITIONS="${ONLY}"

if pgrep -f "AWSIM_for_WHILL.x86_64" >/dev/null; then
    echo "!! AWSIM is running; stop it first: scripts/run_awsim.sh --stop" >&2
    exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
OUT="${REPO_ROOT}/logs/resource_bench/$(date +%Y%m%d_%H%M%S)"
mkdir -p "${OUT}"
cp "${CONFIG}" "${OUT}/config.yaml"
cp Builds/AWSIM_for_WHILL/BUILD_INFO.txt "${OUT}/" 2>/dev/null || true
CMD_NAME="awsim-bench-cmd-$$"

stop_command() { docker rm -f "${CMD_NAME}" >/dev/null 2>&1 || true; }
cleanup() { stop_command; scripts/run_awsim.sh --stop >/dev/null 2>&1 || true; }
trap cleanup EXIT INT TERM

echo "AWSIM resource measurement -> ${OUT}"
echo "conditions (LiDARs,WHILLs): ${CONDITIONS}"
echo "GPU without AWSIM, ${T_MEASURE} s ..."
{
    echo "device_gpu_util_percent,device_mem_used_mib"
    for _ in $(seq 1 "${T_MEASURE}"); do
        nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader,nounits | tr -d ' '
        sleep "${T_SAMPLE_INTERVAL}"
    done
} > "${OUT}/gpu_baseline_no_awsim.csv"

for cond in ${CONDITIONS}; do
    lidars="${cond%,*}"
    whills="${cond#*,}"
    dir="${OUT}/infra${lidars}_whill${whills}"
    mkdir -p "${dir}"
    echo "--- infrastructure LiDARs ${lidars} / WHILLs ${whills} ---"

    # 1. start (BG=1 returns when the OptimizerBridge listens, i.e. the scene is loaded)
    started="$(AWSIM_BENCH_INFRA_LIDARS="${lidars}" AWSIM_BENCH_WHILLS="${whills}" \
        AWSIM_BENCH_LIDAR_SPACING="${LIDAR_SPACING}" AWSIM_BENCH_WHILL_POSITIONS="${WHILL_POSITIONS}" \
        AWSIM_BENCH_VEHICLE_LIDAR="${VEHICLE_LIDAR}" \
        AWSIM_LOG="${dir}/awsim.log" BG=1 scripts/run_awsim.sh)" || { echo "!! AWSIM did not start, see ${dir}/awsim.log" >&2; continue; }
    pid="$(sed -n 's/.*PID \([0-9]*\).*/\1/p' <<<"${started}" | head -1)"
    echo "  AWSIM pid ${pid}"
    sleep "${T_SETTLE}"

    # 2. command: all WHILLs subscribe the same cmd_vel / gear topics
    CONTAINER_NAME="${CMD_NAME}" scripts/ros2_shell.sh bash -c \
        "ros2 topic pub --once /control/command/gear_cmd autoware_vehicle_msgs/msg/GearCommand '{command: 2}' >/dev/null;
         exec ros2 topic pub -r 10 /whill/controller/cmd_vel geometry_msgs/msg/Twist \
             '{linear: {x: ${CMD_LINEAR}}, angular: {z: ${CMD_ANGULAR}}}' >/dev/null" </dev/null >/dev/null 2>&1 &
    cmd_start=$(date +%s)

    # 3. check that every LiDAR and camera publishes
    sleep "${T_TOPIC_CHECK_AFTER}"
    lidar_topics=(/infrastructure/lidar_0/points)
    for ((i = 2; i <= lidars; i++)); do lidar_topics+=("/bench/infra_lidar_${i}/points"); done
    camera_topics=(/sensing/camera/front/image_raw)
    for ((i = 2; i <= whills; i++)); do camera_topics+=("/bench/whill_${i}/camera/image_raw"); done
    scripts/ros2_shell.sh python3 scripts/bench/topic_check.py --duration "${T_TOPIC_CHECK_DURATION}" \
        --lidar "${lidar_topics[@]}" --camera "${camera_topics[@]}" </dev/null 2>/dev/null \
        | grep '^{' | tail -1 > "${dir}/topics.json" || true
    echo "  topics: $(cat "${dir}/topics.json")"

    # 4. record the AWSIM process
    remain=$(( T_WARMUP - ($(date +%s) - cmd_start) ))
    (( remain > 0 )) && sleep "${remain}"
    log_from=$(wc -l < "${dir}/awsim.log")
    echo "  recording ${T_MEASURE} s ..."
    python3 scripts/bench/sample.py --pid "${pid}" --duration "${T_MEASURE}" \
        --interval "${T_SAMPLE_INTERVAL}" --out "${dir}/samples.csv"
    tail -n +"$((log_from + 1))" "${dir}/awsim.log" | grep '\[ResourceBenchmark\] \(status\|fps\)' > "${dir}/status.log" || true
    grep '\[ResourceBenchmark\] \(spawned\|infra lidar\|whill [0-9]\|vehicle lidar\)' "${dir}/awsim.log" > "${dir}/spawn.log" || true

    # 5. stop
    stop_command
    scripts/run_awsim.sh --stop >/dev/null
    sleep "${T_COOLDOWN}"
done

report "${OUT}"
echo "done: ${OUT}"
