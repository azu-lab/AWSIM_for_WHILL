#!/usr/bin/env bash
# Check the scene for missing scripts / prefabs / references (batchmode).
#   scripts/validate_project.sh [scene.unity] [report.txt]
set -euo pipefail
source "$(dirname "$0")/unity_lib.sh"
SCENE="${1:-Assets/Awsim/Scenes/AutowareSimulationDemo.unity}"
REPORT="${2:-${REPO_ROOT}/Logs/validation_report.txt}"
LOG="${REPO_ROOT}/Logs/validate_project.log"
cd "${REPO_ROOT}"
status=0
unity_batch "${LOG}" -executeMethod Awsim.WhillModelCR.Editor.ProjectValidator.ValidateBatch \
    -validateScene "${SCENE}" -validateOut "${REPORT}" || status=$?
if grep -q "error CS" "${LOG}"; then
    echo "!! Compile errors:" >&2
    grep "error CS" "${LOG}" | sort -u >&2
    exit 1
fi
[[ -f "${REPORT}" ]] && cat "${REPORT}"
if [[ ${status} -ne 0 ]]; then
    echo "!! Validation failed (exit ${status}). See ${LOG}" >&2
    exit 1
fi
echo "Validation passed."
