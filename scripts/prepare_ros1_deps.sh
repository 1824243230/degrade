#!/usr/bin/env bash
set -euo pipefail
SA_LIVO_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SA_LIVO_WORKSPACE"
mkdir -p .deps/src
fetch_source() {
  local source_dir="$1" source_url="$2" source_revision="$3"
  if [[ ! -d "$source_dir/.git" ]]; then
    git clone "$source_url" "$source_dir"
    git -C "$source_dir" checkout --detach "$source_revision"
  fi
  if [[ "$(git -C "$source_dir" rev-parse HEAD)" != "$source_revision" ]]; then
    echo "Unexpected source revision in $source_dir; expected $source_revision" >&2
    exit 1
  fi
}
fetch_source .deps/src/Sophus https://github.com/strasdat/Sophus.git de0f8d3d92bf776271e16de56d1803940ebccab9
fetch_source .deps/src/Livox-SDK2 https://github.com/Livox-SDK/Livox-SDK2.git c0796f04c143143899c87a773d9f6b7136453c0b
cmake -S .deps/src/Sophus -B .deps/build/Sophus \
  -DCMAKE_INSTALL_PREFIX="$SA_LIVO_WORKSPACE/.deps/install" \
  -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF \
  -DSOPHUS_USE_BASIC_LOGGING=ON -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON \
  -DEigen3_DIR=/usr/lib/cmake/eigen3
cmake --build .deps/build/Sophus --target install -j2
cmake -S .deps/src/Livox-SDK2 -B .deps/build/Livox-SDK2 \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$SA_LIVO_WORKSPACE/.deps/install" \
  -DCMAKE_C_COMPILER=/usr/bin/gcc-9 -DCMAKE_CXX_COMPILER=/usr/bin/g++-9
cmake --build .deps/build/Livox-SDK2 --target install -j2
bash scripts/prepare_livox_ros1.sh
bash scripts/prepare_kiss_matcher_ros1.sh
bash scripts/prepare_ceres_ros1.sh
