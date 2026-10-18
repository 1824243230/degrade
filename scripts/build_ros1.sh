#!/usr/bin/env bash
set -euo pipefail
SA_LIVO_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SA_LIVO_WORKSPACE"
# Start from a ROS-independent shell, including when launched from a ROS2 terminal.
if [[ "${SA_LIVO_CLEAN_ENV:-0}" != 1 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-hyh}" LANG=C.UTF-8 \
    PATH=/usr/bin:/bin SA_LIVO_CLEAN_ENV=1 bash "$0" "$@"
fi
set +u
source /opt/ros/noetic/setup.bash
set -u
export ROS_HOME="$SA_LIVO_WORKSPACE/.ros"
export ROS_LOG_DIR="$ROS_HOME/log"
mkdir -p "$ROS_LOG_DIR"
export CMAKE_PREFIX_PATH="$SA_LIVO_WORKSPACE/.deps/install:$CMAKE_PREFIX_PATH"
catkin_make -j2 -l2 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=/usr/bin/gcc-9 \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-9 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DPCL_DIR=/usr/lib/x86_64-linux-gnu/cmake/pcl \
  -DEigen3_DIR=/usr/share/eigen3/cmake \
  -DSophus_DIR="$SA_LIVO_WORKSPACE/.deps/install/share/sophus/cmake" \
  -DCATKIN_ENABLE_TESTING=ON \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$@"
