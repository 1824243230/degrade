#!/usr/bin/env bash
# Workspace entry point; compiler and dependency settings live in build_ros1.sh.
set -euo pipefail

SA_LIVO_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'EOF'
用法：./build.sh [--deps] [catkin_make 参数...]

编译 ROS1 Noetic 工作区，默认使用 Release、GCC 9 和 2 个并行任务。
可从任意目录调用；构建产物位于工作区 build/ 和 devel/。

选项：
  --deps      先准备固定版本的第三方依赖到 .deps/install，再编译
              （可能需要网络；系统依赖需预先安装）
  -h, --help  显示此帮助
  --          后续参数全部传给 catkin_make

示例：
  ./build.sh
  ./build.sh --deps
  ./build.sh -j4 -l4
  ./build.sh -DCMAKE_BUILD_TYPE=Debug
  ./build.sh install

编译后，在工作区根目录加载运行环境：
  source /opt/ros/noetic/setup.bash
  source devel/setup.bash
EOF
}

prepare_deps=false
while (( $# > 0 )); do
  case "$1" in
    --deps) prepare_deps=true; shift ;;
    -h|--help) usage; exit 0 ;;
    --) shift; break ;;
    *) break ;;
  esac
done

cd -- "$SA_LIVO_WORKSPACE"
if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  printf '错误：未找到 /opt/ros/noetic/setup.bash，请先安装 ROS1 Noetic。\n' >&2
  exit 1
fi

if [[ "$prepare_deps" == true ]]; then
  bash "$SA_LIVO_WORKSPACE/scripts/prepare_ros1_deps.sh"
fi

exec bash "$SA_LIVO_WORKSPACE/scripts/build_ros1.sh" "$@"
