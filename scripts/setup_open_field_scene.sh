#!/usr/bin/env bash
# Regenerate Assets/Awsim/Scenes/WhillOpenField.unity (obstacle-free scene for the
# vehicle parameter optimization) from the corridor scene. Close the Unity Editor first.
set -euo pipefail
source "$(dirname "$0")/unity_lib.sh"
cd "${REPO_ROOT}"
LOG="${REPO_ROOT}/Logs/setup_open_field_scene.log"
if ! unity_batch "${LOG}" -executeMethod Awsim.WhillModelCR.Editor.WhillSceneTools.CreateOpenFieldSceneBatch; then
    echo "!! Failed. Last lines of ${LOG}:" >&2
    tail -40 "${LOG}" >&2
    exit 1
fi
grep "\[WhillSceneTools\]" "${LOG}" || true
