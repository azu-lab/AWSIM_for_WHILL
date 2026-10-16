#!/usr/bin/env bash
# Open this project in the Unity Editor with the environment AWSIM needs:
# ROS 2 removed from the environment (Ros2ForUnity ships its own libraries) and the same DDS
# settings as scripts/run_awsim.sh / scripts/ros2_shell.sh, so that a scene played in the Editor
# talks to the ROS 2 tools and to rl/ like the built player.
#
#   scripts/open_editor.sh                                       # default scene (West Shinjuku)
#   scripts/open_editor.sh Assets/Awsim/Scenes/WhillCorridor.unity
#   ROS_DOMAIN_ID=77 scripts/open_editor.sh
#
# Use it e.g. to place obstacles for reinforcement learning (docs/DREAMER_TRAINING.md §5):
# edit the scene, save it (Ctrl+S), then press Play or rebuild the player (scripts/build_player.sh).
set -euo pipefail
source "$(dirname "$0")/unity_lib.sh"
SCENE="${1:-Assets/Awsim/Scenes/AutowareSimulationDemo.unity}"
BIN="$(find_unity_bin)"
ensure_editor_closed
prepare_unity_env
prepare_dds_env
echo "Unity         : ${BIN}"
echo "Scene         : ${SCENE}"
echo "ROS_DOMAIN_ID : ${ROS_DOMAIN_ID}"
if [[ ! -f "${REPO_ROOT}/${SCENE}" ]]; then
    echo "!! ${SCENE} not found (give a path relative to the repository, e.g. Assets/Awsim/Scenes/WhillCorridor.unity)" >&2
    exit 1
fi
# -awsimScene is read by Assets/Awsim/Scripts/Editor/WhillModelCR/EditorStartupScene.cs
exec "${BIN}" -projectPath "${REPO_ROOT}" -awsimScene "${SCENE}"
