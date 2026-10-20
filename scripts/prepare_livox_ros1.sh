#!/usr/bin/env bash
# Restore the pinned driver without modifying a user's existing checkout.
set -euo pipefail
SA_LIVO_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SA_LIVO_WORKSPACE"
driver="src/livox_ros_driver2"
if [[ -f "$driver/package.xml" && -f "$driver/CMakeLists.txt" ]]; then
  exit 0
fi
if [[ -d "$driver" && -n "$(ls -A "$driver")" ]]; then
  echo "Existing incomplete Livox source at $driver; refusing to overwrite it" >&2
  exit 1
fi
archive=".deps/src/livox_ros_driver2-21445540.tar.gz"
mkdir -p .deps/src "$driver"
if [[ ! -f "$archive" ]]; then
  curl --fail --location --retry 2 \
    https://codeload.github.com/Livox-SDK/livox_ros_driver2/tar.gz/21445540f0d100dc86a7e6df312dd70bbdb4afdf \
    --output "$archive"
fi
echo "b0f25c87ce8be7bc2edcf5f81fd0e2c3a4c0f55a5f8a1a7f44bbc7ffe7fda664  $archive" | sha256sum --check
tar -xzf "$archive" --strip-components=1 -C "$driver"
/usr/bin/python3 - "$driver" <<'PY'
from pathlib import Path
import sys
root=Path(sys.argv[1])
path=root/'CMakeLists.txt'
text=path.read_text().split('if(ROS_EDITION STREQUAL "ROS1")',1)[1].split('else(ROS_EDITION STREQUAL "ROS2")',1)[0]
text=text.replace('set(CMAKE_CXX_STANDARD 14)','set(CMAKE_CXX_STANDARD 17)')
text=text.replace('find_library(LIVOX_LIDAR_SDK_LIBRARY  liblivox_lidar_sdk_static.a    /usr/local/lib)',
    'find_library(LIVOX_LIDAR_SDK_LIBRARY NAMES livox_lidar_sdk_static REQUIRED)\n  find_path(LIVOX_SDK_INCLUDE_DIR livox_lidar_api.h REQUIRED)')
text=text.replace('${APR_INCLUDE_DIRS}\n','${APR_INCLUDE_DIRS}\n    ${LIVOX_SDK_INCLUDE_DIR}\n')
path.write_text(text)
(root/'package.xml').write_text((root/'package_ROS1.xml').read_text())
PY
