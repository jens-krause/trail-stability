#!/bin/sh
# Runs the Metal kernel emulation test (no Mac needed): MSL text -> C++ -> compare with CPU path.
set -e
cd "$(dirname "$0")/.."
out=${TMPDIR:-/tmp}/trailstab_metal_emu
mkdir -p "$out"
python3 - "$out/msl_as_cpp.inc" <<'EOF'
import sys, re
s = open('ofx/metal_kernels.h').read()
src = s[s.index('R"MSL(') + 6 : s.index(')MSL"')]
open(sys.argv[1], 'w').write(src)
EOF
${CXX:-g++} -std=c++17 -O1 -Wno-attributes -Icore -Iofx -Itools -I"$out" \
  tools/metal_emu_test.cpp core/warp.cpp core/threadpool.cpp -pthread -o "$out/metal_emu_test"
"$out/metal_emu_test"
