#!/usr/bin/env bash
set -euo pipefail
SA_LIVO_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SA_LIVO_WORKSPACE"
archive=.deps/src/ceres-2.1.0.tar.gz
mkdir -p .deps/src
if [[ ! -f "$archive" ]]; then
  curl -fL --retry 2 https://codeload.github.com/ceres-solver/ceres-solver/tar.gz/refs/tags/2.1.0 -o "$archive"
fi
sha256sum -c scripts/ceres-2.1.0.sha256
if [[ ! -f .deps/src/ceres-solver-2.1.0/CMakeLists.txt ]]; then
  tar -xzf "$archive" -C .deps/src
fi
cmake -S .deps/src/ceres-solver-2.1.0 -B .deps/build/ceres \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$SA_LIVO_WORKSPACE/.deps/install" \
  -DCMAKE_C_COMPILER=/usr/bin/gcc-9 -DCMAKE_CXX_COMPILER=/usr/bin/g++-9 \
  -DCMAKE_IGNORE_PREFIX_PATH=/usr/local \
  -DEigen3_DIR=/usr/lib/cmake/eigen3 \
  -Dglog_DIR=/usr/lib/x86_64-linux-gnu/cmake/glog \
  -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF \
  -DBUILD_SHARED_LIBS=ON -DSUITESPARSE=OFF -DCXSPARSE=OFF \
  -DLAPACK=OFF -DEIGENSPARSE=ON -DMINIGLOG=OFF
cmake --build .deps/build/ceres --target install -j2
