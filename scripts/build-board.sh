#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! command -v idf.py >/dev/null 2>&1; then
    # shellcheck source=/dev/null
    source "${ROOT_DIR}/scripts/idf-env.sh"
fi

BOARD="${1:-}"
ACTION="build"

if [[ -z "${BOARD}" ]]; then
    echo "Usage: $0 <p4-function-ev|s3-usb-otg> [build|flash|monitor|clean]" >&2
    exit 1
fi

if [[ $# -ge 2 ]]; then
    ACTION="${2}"
fi

if [[ $# -gt 2 ]]; then
    echo "Usage: $0 <p4-function-ev|s3-usb-otg> [build|flash|monitor|clean]" >&2
    exit 1
fi

case "${BOARD}" in
    p4-function-ev)
        TARGET="esp32p4"
        BUILD_DIR="build-p4-function-ev"
        SDKCONFIG_FILE="sdkconfig.p4-function-ev"
        DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.p4-function-ev"
        ;;
    s3-usb-otg)
        TARGET="esp32s3"
        BUILD_DIR="build-s3-usb-otg"
        SDKCONFIG_FILE="sdkconfig.s3-usb-otg"
        DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.s3-usb-otg"
        ;;
    *)
        echo "Unknown board: ${BOARD}" >&2
        exit 1
        ;;
esac

cd "${ROOT_DIR}"

if [[ "${ACTION}" == "clean" ]]; then
    echo "Cleaning ${BOARD}: removing ${SDKCONFIG_FILE} and ${BUILD_DIR}"
    rm -f "${SDKCONFIG_FILE}"
    rm -rf "${BUILD_DIR}"
    exit 0
fi

idf.py -B "${BUILD_DIR}" \
    -DIDF_TARGET="${TARGET}" \
    -DSDKCONFIG="${SDKCONFIG_FILE}" \
    -DSDKCONFIG_DEFAULTS="${DEFAULTS}" \
    "${ACTION}"
