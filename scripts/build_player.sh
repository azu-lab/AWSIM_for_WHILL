#!/usr/bin/env bash
# Build the Linux standalone player of AWSIM for WHILL.
#
#   scripts/build_player.sh                       # -> Builds/AWSIM_for_WHILL/AWSIM_for_WHILL.x86_64
#   AWSIM_BUILD_OUTPUT=/path/AWSIM.x86_64 scripts/build_player.sh
#
# Close the Unity Editor before running. The first build takes 10-30 minutes.
set -euo pipefail
source "$(dirname "$0")/unity_lib.sh"
cd "${REPO_ROOT}"
export AWSIM_BUILD_SCENE="${AWSIM_BUILD_SCENE:-Assets/Awsim/Scenes/AutowareSimulationDemo.unity}"
export AWSIM_BUILD_OUTPUT="${AWSIM_BUILD_OUTPUT:-${REPO_ROOT}/Builds/AWSIM_for_WHILL/AWSIM_for_WHILL.x86_64}"
LOG="${REPO_ROOT}/Logs/build_player.log"

status=0
unity_batch "${LOG}" -executeMethod Awsim.WhillModelCR.Editor.WhillPlayerBuilder.BuildLinux64 || status=$?
if [[ ${status} -ne 0 || ! -x "${AWSIM_BUILD_OUTPUT}" ]]; then
    echo "!! Build failed (exit ${status}). Last lines of ${LOG}:" >&2
    grep -E "error CS|\[WhillPlayerBuilder\]" "${LOG}" | tail -20 >&2 || tail -40 "${LOG}" >&2
    exit 1
fi

# Record which source the binary was built from.
cat > "$(dirname "${AWSIM_BUILD_OUTPUT}")/BUILD_INFO.txt" <<INFO
built_at: $(date -Is)
scene:    ${AWSIM_BUILD_SCENE}
commit:   $(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || echo unknown)$(git -C "${REPO_ROOT}" diff --quiet 2>/dev/null || echo " (with uncommitted changes)")
unity:    $(unity_version)
INFO
echo "Built: ${AWSIM_BUILD_OUTPUT}"
cat "$(dirname "${AWSIM_BUILD_OUTPUT}")/BUILD_INFO.txt"
