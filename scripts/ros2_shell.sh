#!/usr/bin/env bash
# Run a command (default: interactive bash) in the ROS 2 tools container with the
# same DDS settings as AWSIM. The repository is mounted at /workspace.
#
#   scripts/ros2_shell.sh                                   # interactive shell
#   scripts/ros2_shell.sh ros2 topic list
#   IMAGE=awsim-for-whill/dreamer scripts/ros2_shell.sh python3 rl/scripts/random_agent.py
#
# Build the images first (see docker/compose.yaml): ros2, then dreamer.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="${IMAGE:-awsim-for-whill/ros2:latest}"
TTY_ARGS=()
[[ -t 0 && -t 1 ]] && TTY_ARGS=(-it)
GPU_ARGS=()
[[ "${IMAGE}" == *dreamer* ]] && GPU_ARGS=(--gpus all)
# Pass JAX / XLA settings through when they are set on the host.
for v in XLA_FLAGS XLA_PYTHON_CLIENT_MEM_FRACTION JAX_PLATFORMS; do
    [[ -n "${!v:-}" ]] && GPU_ARGS+=(-e "${v}=${!v}")
done
if [[ $# -eq 0 ]]; then
    set -- bash
fi
exec docker run --rm "${TTY_ARGS[@]}" "${GPU_ARGS[@]}" \
    --network host --ipc host \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=file:///workspace/config/cyclonedds.xml \
    -e HOME=/tmp \
    -u "$(id -u):$(id -g)" \
    -v "${REPO_ROOT}:/workspace" -w /workspace \
    "${IMAGE}" bash -c 'source /opt/ros/humble/setup.bash && exec "$@"' -- "$@"
