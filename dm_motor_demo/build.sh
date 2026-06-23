#!/bin/bash
# Quick build script for dm_motor_demo
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== Building dm_motor_demo ==="

rm -rf build
mkdir -p build
cd build

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo ""
echo "=== Build complete ==="
echo "Binary: $SCRIPT_DIR/build/dm_motor_demo"
echo ""
echo "Usage:"
echo "  ./dm_motor_demo            # use default motor CAN ID 0x01"
echo "  ./dm_motor_demo 2          # use motor CAN ID 0x02"
echo ""
echo "Note: If USB permission denied, run:"
echo "  sudo chmod 666 /dev/ttyUSB*"
