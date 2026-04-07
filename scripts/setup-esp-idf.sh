#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IDF_VERSION="v6.0-beta2"
IDF_DIR="${ROOT_DIR}/esp-idf"

if [[ ! -e "${IDF_DIR}/.git" ]]; then
    echo "Cloning ESP-IDF ${IDF_VERSION} into ${IDF_DIR}"
    git clone --branch "${IDF_VERSION}" --depth 1 https://github.com/espressif/esp-idf.git "${IDF_DIR}"
else
    echo "Updating existing ESP-IDF checkout in ${IDF_DIR} to ${IDF_VERSION}"
    if ! git -C "${IDF_DIR}" rev-parse --verify --quiet "refs/tags/${IDF_VERSION}" >/dev/null; then
        git -C "${IDF_DIR}" fetch --tags origin "${IDF_VERSION}" --depth 1
    fi
    git -C "${IDF_DIR}" checkout "${IDF_VERSION}"
fi

git -C "${IDF_DIR}" submodule update --init --recursive --depth 1

"${IDF_DIR}/install.sh" esp32p4 esp32s3

echo
echo "ESP-IDF ${IDF_VERSION} is installed."
echo "Run: source scripts/idf-env.sh"
