#!/usr/bin/env bash
# Regenerate the default scene (West Shinjuku) and the corridor scene (DreamerV3) from their
# layouts in WhillSceneTools.cs and the infrastructure LiDAR configs:
#   Assets/Awsim/Scenes/AutowareSimulationDemo.unity  <- config/infra_lidar.yaml
#   Assets/Awsim/Scenes/WhillCorridor.unity           <- config/infra_lidar_corridor.yaml
# Close the Unity Editor first.
set -euo pipefail
source "$(dirname "$0")/unity_lib.sh"
cd "${REPO_ROOT}"
LOG="${REPO_ROOT}/Logs/setup_scenes.log"
if ! unity_batch "${LOG}" -executeMethod Awsim.WhillModelCR.Editor.WhillSceneTools.SetupScenesBatch; then
    echo "!! Failed. Last lines of ${LOG}:" >&2
    grep -E "error CS|Exception" "${LOG}" | head -20 >&2 || tail -40 "${LOG}" >&2
    exit 1
fi
grep -E "\[WhillSceneTools\]|\[InfraLidarSetup\]" "${LOG}" || true
