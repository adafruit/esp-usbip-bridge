#!/usr/bin/env bash

# BASH_SOURCE is unset under zsh, the macOS default shell. Same idiom as
# esp-idf/export.sh.
if [ -n "${BASH_SOURCE-}" ]; then
    _SELF="${BASH_SOURCE[0]}"
elif [ -n "${ZSH_VERSION-}" ]; then
    # shellcheck disable=SC2296
    _SELF="${(%):-%x}"
else
    _SELF="$0"
fi
ROOT_DIR="$(cd "$(dirname "${_SELF}")/.." && pwd)"
IDF_DIR="${ROOT_DIR}/esp-idf"

if [[ ! -f "${IDF_DIR}/export.sh" ]]; then
    echo "ESP-IDF is not set up at ${IDF_DIR}. Run ./scripts/setup-esp-idf.sh first." >&2
    return 1 2>/dev/null || exit 1
fi

export IDF_PATH="${IDF_DIR}"
# shellcheck source=/dev/null
. "${IDF_DIR}/export.sh"
