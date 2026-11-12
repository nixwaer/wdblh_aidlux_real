#!/usr/bin/env bash
# Build and run only tests which do not access devices or start a service.
set -euo pipefail
PROJECT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
build_dir=${1:-$PROJECT/build-offline}
test_python=${WDBLH_PYTHON:-python3}
cmake -S "$PROJECT" -B "$build_dir" \
    -DENABLE_AIDLITE=OFF -DENABLE_ONNX=OFF -DENABLE_LIBTORCH=OFF \
    -DWDBLH_BUILD_CONTROLLER=ON -DWDBLH_BUILD_DIAGNOSTICS=ON -DBUILD_TESTING=ON
cmake --build "$build_dir" -j"${WDBLH_BUILD_JOBS:-2}"
ctest --test-dir "$build_dir" --output-on-failure
PYTHONDONTWRITEBYTECODE=1 WDBLH_PROTOCOL_DUMP="$build_dir/bin/test_protocol_dump" \
    "$test_python" "$PROJECT/tests/test_offline.py"
