#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPOSITORY_DIR=$(cd -- "${SCRIPT_DIR}/.." && pwd)
BUILD_JOBS=${BUILD_JOBS:-2}

if ! [[ "${BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]]; then
    echo "BUILD_JOBS must be a positive integer, got: ${BUILD_JOBS}" >&2
    exit 1
fi

echo "Building with ${BUILD_JOBS} parallel compilation job(s)."

docker build \
    --build-arg "BUILD_JOBS=${BUILD_JOBS}" \
    --tag dronecode/roscon-25-workshop:latest \
    --file "${SCRIPT_DIR}/Dockerfile" \
    "${REPOSITORY_DIR}"
