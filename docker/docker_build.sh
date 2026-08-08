#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPOSITORY_DIR=$(cd -- "${SCRIPT_DIR}/.." && pwd)

docker build \
    --tag dronecode/roscon-25-workshop:latest \
    --file "${SCRIPT_DIR}/Dockerfile" \
    "${REPOSITORY_DIR}"
