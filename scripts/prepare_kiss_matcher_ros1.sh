#!/usr/bin/env bash
set -euo pipefail
if [[ "${SA_LIVO_KISS_CLEAN_ENV:-0}" != 1 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-hyh}" LANG=C.UTF-8 PATH=/usr/bin:/bin \
    SA_LIVO_KISS_CLEAN_ENV=1 bash "$0" "$@"
fi
SA_LIVO_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SA_LIVO_WORKSPACE"
vendor_root="$SA_LIVO_WORKSPACE/src/kiss_matcher"
prefix="$SA_LIVO_WORKSPACE/.deps/install"
if [[ -f "$vendor_root/ROS1_SOURCES.sha256" ]]; then
  (cd "$vendor_root" && sha256sum --check --quiet ROS1_SOURCES.sha256)
fi
common_args=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/g++-9
  -DCMAKE_C_COMPILER=/usr/bin/gcc-9 -DCMAKE_INSTALL_PREFIX="$prefix"
  -DCMAKE_PREFIX_PATH="$prefix" -DEigen3_DIR=/usr/lib/cmake/eigen3
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON
  -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF -DBUILD_TESTS=OFF)
/usr/bin/cmake -S "$vendor_root/robin" -B .deps/build/robin "${common_args[@]}" \
  -DROBIN_VENDOR_ROOT="$vendor_root" -DROBIN_OFFLINE=ON -DUSE_SYSTEM_EIGEN3=ON -DPMC_BUILD_SHARED=OFF
/usr/bin/cmake --build .deps/build/robin --target install -j2
/usr/bin/cmake -S "$vendor_root/kiss_matcher" -B .deps/build/kiss_matcher "${common_args[@]}" \
  -DUSE_SYSTEM_EIGEN3=ON -DUSE_SYSTEM_TBB=ON -DUSE_SYSTEM_ROBIN=ON -DUSE_CCACHE=OFF -DKISS_MATCHER_OFFLINE=ON
/usr/bin/cmake --build .deps/build/kiss_matcher --target install -j2
/usr/bin/cmake -S "$vendor_root/small_gicp" -B .deps/build/small_gicp "${common_args[@]}" \
  -DBUILD_WITH_OPENMP=ON -DBUILD_WITH_TBB=OFF -DBUILD_WITH_MARCH_NATIVE=OFF \
  -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_PYTHON_BINDINGS=OFF
/usr/bin/cmake --build .deps/build/small_gicp --target install -j2
