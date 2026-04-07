#!/usr/bin/env bash

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IDF_DIR="${ROOT_DIR}/esp-idf"

if [[ ! -f "${IDF_DIR}/export.sh" ]]; then
    echo "ESP-IDF is not set up at ${IDF_DIR}. Run ./scripts/setup-esp-idf.sh first." >&2
    return 1 2>/dev/null || exit 1
fi

export IDF_PATH="${IDF_DIR}"
# shellcheck source=/dev/null
. "${IDF_DIR}/export.sh"
