#!/usr/bin/env bash
# Rebuild the infrastructure LiDARs of the corridor scene from config/infra_lidar.yaml.
#   scripts/setup_infra_lidar.sh [config.yaml] [scene.unity]
# Close the Unity Editor before running.
set -euo pipefail
source "$(dirname "$0")/unity_lib.sh"
CONFIG="${1:-config/infra_lidar.yaml}"
SCENE="${2:-Assets/Awsim/Scenes/AutowareSimulationDemo.unity}"
LOG="${REPO_ROOT}/Logs/setup_infra_lidar.log"
cd "${REPO_ROOT}"
if ! unity_batch "${LOG}" -executeMethod Awsim.InfraLidar.Editor.InfraLidarSetup.SetupBatch \
        -infraLidarConfig "${CONFIG}" -infraLidarScene "${SCENE}"; then
    echo "!! Failed. Last lines of ${LOG}:" >&2
    tail -40 "${LOG}" >&2
    exit 1
fi
grep "\[InfraLidarSetup\]" "${LOG}" || true
echo "Updated ${SCENE}. Review it with: git diff --stat"
