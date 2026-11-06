#!/bin/sh
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
GNSS_OUTPUT="${TMPDIR:-/tmp}/navgraph_gnss_cpp_tests"
INS_OUTPUT="${TMPDIR:-/tmp}/navgraph_ins_cpp_tests"
EKF_OUTPUT="${TMPDIR:-/tmp}/navgraph_ekf_cpp_tests"
FGO_OUTPUT="${TMPDIR:-/tmp}/navgraph_fgo_cpp_tests"

xcrun clang++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I"$ROOT/cpp/include" \
  "$ROOT/cpp/gnss/gnss.cpp" \
  "$ROOT/cpp/tests/gnss_core_test.cpp" \
  -o "$GNSS_OUTPUT"

"$GNSS_OUTPUT"

xcrun clang++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I"$ROOT/cpp/include" \
  "$ROOT/cpp/ins/ins.cpp" \
  "$ROOT/cpp/tests/ins_core_test.cpp" \
  -o "$INS_OUTPUT"

"$INS_OUTPUT"

xcrun clang++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I"$ROOT/cpp/include" \
  "$ROOT/cpp/gnss/gnss.cpp" \
  "$ROOT/cpp/ins/ins.cpp" \
  "$ROOT/cpp/core/frame.cpp" \
  "$ROOT/cpp/ekf/ekf.cpp" \
  "$ROOT/cpp/tests/ekf_core_test.cpp" \
  -o "$EKF_OUTPUT"

"$EKF_OUTPUT"

xcrun clang++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I"$ROOT/cpp/include" \
  "$ROOT/cpp/gnss/gnss.cpp" \
  "$ROOT/cpp/core/frame.cpp" \
  "$ROOT/cpp/fgo/fgo.cpp" \
  "$ROOT/cpp/tests/fgo_core_test.cpp" \
  -o "$FGO_OUTPUT"

"$FGO_OUTPUT"
