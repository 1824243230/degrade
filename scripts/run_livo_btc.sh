#!/usr/bin/env bash
set -euo pipefail
SA_LIVO_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${SA_LIVO_CLEAN_ENV:-0}" != 1 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-hyh}" LANG=C.UTF-8 PATH=/usr/bin:/bin \
    SA_LIVO_CLEAN_ENV=1 bash "$0" "$@"
fi
cd "$SA_LIVO_WORKSPACE"
set +u
source /opt/ros/noetic/setup.bash
source devel/setup.bash
set -u
export ROS_HOME="$SA_LIVO_WORKSPACE/.ros"
export ROS_LOG_DIR="$ROS_HOME/log"
export ROS_IP=127.0.0.1
export LD_LIBRARY_PATH="$SA_LIVO_WORKSPACE/.deps/install/lib:$LD_LIBRARY_PATH"
exec /usr/bin/python3 scripts/run_livo_btc.py "$@"
