#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IDF_DIR="${ROOT_DIR}/esp-idf"

# Check out exactly what the submodule pins. The pin is on the adafruit fork and
# carries patches that are in no upstream tag, notably "hal: usb_dwc: Add FS-only
# mode", which the USB_DWC_FSLS_ONLY define in CMakeLists.txt depends on.
git -C "${ROOT_DIR}" submodule update --init esp-idf

git -C "${IDF_DIR}" submodule update --init --recursive --depth 1

"${IDF_DIR}/install.sh" esp32p4 esp32s3

echo
echo "ESP-IDF $(git -C "${IDF_DIR}" describe --tags --always) is installed."
echo "Run: source scripts/idf-env.sh"
