#!/usr/bin/env bash
# One-time setup after cloning:
#   1. check Unity 6000.0.34f1,
#   2. download + import the West Shinjuku map (AWSIM Externals, 1.4 GB, CC BY-NC),
#   3. build the ROS 2 tools Docker image,
#   4. build the AWSIM player (default scene: WHILL Model CR in West Shinjuku).
#
#   scripts/setup.sh                  # all steps
#   WITH_DREAMER=1 scripts/setup.sh   # also the reinforcement-learning image (DreamerV3, rl/)
#   WITH_CORRIDOR=1 scripts/setup.sh  # also the player of the corridor scene (rl task corridor_box)
#
# Then:  scripts/run_awsim.sh   and   scripts/ros2_shell.sh python3 ros2_examples/drive_test.py
set -euo pipefail
source "$(dirname "$0")/unity_lib.sh"
cd "${REPO_ROOT}"

echo "=== [1/4] Unity"
find_unity_bin >/dev/null || exit 1
echo "    $(find_unity_bin) ($(unity_version))"
if [[ ! -f third_party/dreamerv3/README.md ]]; then
    git submodule update --init third_party/dreamerv3
fi

echo "=== [2/4] West Shinjuku map"
scripts/fetch_shinjuku_map.sh

echo "=== [3/4] Docker image(s)"
docker compose -f docker/compose.yaml build ros2
if [[ "${WITH_DREAMER:-0}" == "1" ]]; then
    docker compose -f docker/compose.yaml build dreamer
fi

echo "=== [4/4] AWSIM player"
scripts/build_player.sh
if [[ "${WITH_CORRIDOR:-0}" == "1" ]]; then
    AWSIM_BUILD_SCENE=Assets/Awsim/Scenes/WhillCorridor.unity \
    AWSIM_BUILD_OUTPUT="${REPO_ROOT}/Builds/AWSIM_for_WHILL_Corridor/AWSIM_for_WHILL.x86_64" \
        scripts/build_player.sh
fi

cat <<MSG

Setup finished. Start AWSIM and drive the WHILL:
  scripts/run_awsim.sh                                                  # terminal 1
  scripts/ros2_shell.sh python3 ros2_examples/drive_test.py            # terminal 2: automatic test
  scripts/ros2_shell.sh python3 ros2_examples/teleop_keyboard.py       #   or keys in this terminal
MSG
