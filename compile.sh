#!/usr/bin/env bash
# Build the dspico-debug test firmware (PSRAM loop-test + SD detect).
# Default SDK path matches the existing dspico-firmware build.
set -euo pipefail

cd "$(dirname "$(realpath "$0")")"

if command -v nproc >/dev/null 2>&1; then
    NPROC="$(nproc)"
else
    NPROC="$(getconf _NPROCESSORS_ONLN)"
fi

# Reuse an already-populated picotool (installed to ~/.local) so the SDK does
# not try to git-clone it (no network needed at build time).
export PICO_SDK_PATH="${PICO_SDK_PATH:-/home/jerry/pico-sdk-2.3.0}"
export CMAKE_POLICY_VERSION_MINIMUM=3.5

echo "[>] Configuring with CMake (PICO_SDK_PATH=${PICO_SDK_PATH}).."
rm -rf build/ || true
mkdir -p build
cmake -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo -B build/ .

echo "[>] Building DSpico_debug.."
CMAKE_BUILD_PARALLEL_LEVEL="${NPROC}" cmake --build build

echo "[>] Done. UF2 at build/DSpico_debug.uf2"
