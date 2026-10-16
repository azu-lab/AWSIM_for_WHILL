#!/usr/bin/env bash
# Download a trained model (GitHub Release asset of this repository) into pretrained/.
#
#   scripts/fetch_pretrained.sh                         # default model (DreamerV3, task shinjuku_box)
#   PRETRAINED_ARCHIVE=/path/x.tar.gz scripts/fetch_pretrained.sh   # use a downloaded archive
#
# Then:  IMAGE=awsim-for-whill/dreamer scripts/ros2_shell.sh \
#            python3 rl/evaluate.py --logdir pretrained/shinjuku_box_dreamerv3 --episodes 10
#
# The model is licensed under CC BY-NC 4.0 (see pretrained/<name>/LICENSE.txt).
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
NAME="${PRETRAINED_NAME:-shinjuku_box_dreamerv3}"
TAG="${PRETRAINED_TAG:-pretrained-v1}"
BASE_URL="${PRETRAINED_BASE_URL:-https://github.com/azu-lab/AWSIM_for_WHILL/releases/download/${TAG}}"
DEST="${REPO_ROOT}/pretrained"
mkdir -p "${DEST}"

if [[ -n "${PRETRAINED_ARCHIVE:-}" ]]; then
    ARCHIVE="${PRETRAINED_ARCHIVE}"
    [[ -f "${ARCHIVE}.sha256" ]] && cp "${ARCHIVE}.sha256" "${DEST}/${NAME}.tar.gz.sha256.expected"
else
    ARCHIVE="${DEST}/${NAME}.tar.gz"
    echo "Downloading ${BASE_URL}/${NAME}.tar.gz"
    curl -fL --retry 3 -o "${ARCHIVE}" "${BASE_URL}/${NAME}.tar.gz"
    curl -fsSL -o "${DEST}/${NAME}.tar.gz.sha256.expected" "${BASE_URL}/${NAME}.tar.gz.sha256" || true
fi

if [[ -s "${DEST}/${NAME}.tar.gz.sha256.expected" ]]; then
    expected="$(awk '{print $1}' "${DEST}/${NAME}.tar.gz.sha256.expected")"
    actual="$(sha256sum "${ARCHIVE}" | awk '{print $1}')"
    if [[ "${expected}" != "${actual}" ]]; then
        echo "!! SHA-256 mismatch for ${ARCHIVE}: expected ${expected}, got ${actual}" >&2
        exit 1
    fi
    echo "SHA-256 OK"
    rm -f "${DEST}/${NAME}.tar.gz.sha256.expected"
else
    echo "Note: no checksum available, not verified"
fi

rm -rf "${DEST:?}/${NAME}"
tar -xzf "${ARCHIVE}" -C "${DEST}"
echo "Extracted: ${DEST}/${NAME}"
cat "${DEST}/${NAME}/LICENSE.txt" | head -3
