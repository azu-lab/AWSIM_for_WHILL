#!/usr/bin/env bash
# Download the West Shinjuku map of AWSIM (AWSIM "Externals") and import it into the project.
#
#   scripts/fetch_shinjuku_map.sh
#
# The map is not part of this repository: it is 1.4 GB and licensed CC BY-NC, and upstream
# AWSIM distributes it separately on its GitHub Releases. The default scene of this project
# (Assets/Awsim/Scenes/AutowareSimulationDemo.unity) needs it: without it the WHILL has no ground.
#
# Environment:
#   SHINJUKU_PACKAGE  path of an already downloaded Shinjuku.unitypackage (skips the download)
#   CACHE_DIR         where to store the download (default: ~/.cache/awsim_for_whill)
#
# Close the Unity Editor before running. The import takes 5-15 minutes.
set -euo pipefail
source "$(dirname "$0")/unity_lib.sh"
cd "${REPO_ROOT}"

AWSIM_RELEASE="v2.0.0"   # the externals release that matches AWSIM fa1dc65
URL="https://github.com/tier4/AWSIM/releases/download/${AWSIM_RELEASE}/Shinjuku.unitypackage"
CACHE_DIR="${CACHE_DIR:-${HOME}/.cache/awsim_for_whill}"
PACKAGE="${SHINJUKU_PACKAGE:-${CACHE_DIR}/Shinjuku.unitypackage}"
EXTERNALS_DIR="${REPO_ROOT}/Assets/Awsim/Externals"
LOG="${REPO_ROOT}/Logs/import_shinjuku.log"
MIN_SIZE=1400000000   # the complete package is ~1.47 GB

imported() {
    [[ -n "$(find "${EXTERNALS_DIR}" -mindepth 1 ! -name '.gitkeep' -print -quit 2>/dev/null)" ]]
}

if imported; then
    echo "The Shinjuku map is already imported (${EXTERNALS_DIR})."
    echo "To import again, delete everything in it except .gitkeep."
    exit 0
fi

# --- 1. download (resumable) ---
if [[ -f "${PACKAGE}" && "$(stat -c%s "${PACKAGE}")" -gt ${MIN_SIZE} ]]; then
    echo "[1/2] Using ${PACKAGE} ($(numfmt --to=iec "$(stat -c%s "${PACKAGE}")"))"
else
    mkdir -p "$(dirname "${PACKAGE}")"
    echo "[1/2] Downloading ${URL}"
    echo "      -> ${PACKAGE} (about 1.4 GB, CC BY-NC)"
    curl -L -C - --fail --progress-bar -o "${PACKAGE}" "${URL}"
    if [[ "$(stat -c%s "${PACKAGE}")" -le ${MIN_SIZE} ]]; then
        echo "!! The download looks incomplete. Run the script again to resume." >&2
        exit 1
    fi
fi

# --- 2. import (same as Assets > Import Package > Custom Package) ---
# Without -nographics: some material/shader imports need the GPU.
bin="$(find_unity_bin)"
ensure_editor_closed
prepare_unity_env
mkdir -p "$(dirname "${LOG}")"
echo "[2/2] Importing into ${EXTERNALS_DIR} (5-15 min). Log: ${LOG}"
"${bin}" -batchmode -quit -projectPath "${REPO_ROOT}" -importPackage "${PACKAGE}" -logFile "${LOG}"

if imported; then
    echo "Imported:"
    find "${EXTERNALS_DIR}" -mindepth 1 -maxdepth 1 ! -name '.gitkeep' -printf '  %f\n'
else
    echo "!! Unity finished but ${EXTERNALS_DIR} is still empty. See ${LOG}" >&2
    exit 1
fi
