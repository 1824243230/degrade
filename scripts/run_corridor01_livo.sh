#!/usr/bin/env bash
set -euo pipefail
SA_LIVO_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SA_LIVO_WORKSPACE"
SA_LIVO_CORRIDOR_EXTERNAL=false
SA_LIVO_CORRIDOR_PREPARE_ONLY=false
SA_LIVO_CORRIDOR_MAP=""
while (( $# > 0 )); do
  case "$1" in
    --external-bag) SA_LIVO_CORRIDOR_EXTERNAL=true; shift ;;
    --prepare-only) SA_LIVO_CORRIDOR_PREPARE_ONLY=true; shift ;;
    --localize)
      if [[ $# -lt 2 || ! -f "$2" || ! -f "$2.yaml" ]]; then
        printf '重定位需要已有地图及元信息：--localize /path/optimized_map.pcd（同时需有 .pcd.yaml）。\n' >&2
        exit 2
      fi
      SA_LIVO_CORRIDOR_MAP="$(realpath -- "$2")"; shift 2 ;;
    -h|--help)
      printf '%s\n' \
        '用法：bash scripts/run_corridor01_livo.sh [--prepare-only|--external-bag] [--localize MAP.pcd] [roslaunch 参数...]' \
        '默认准备并直接回放 Corridor01，无需另开 rosbag play。' \
        '--prepare-only  只准备转换后的输入。' \
        '--external-bag  启动订阅并等待另一个终端播放转换后的输入。' \
        '--localize MAP  使用导出的地图进行启动重定位和失效恢复；默认模式为 BTC 回环建图。' \
        '示例：bash scripts/run_corridor01_livo.sh --external-bag use_rviz:=true loop_enabled:=true'
      exit 0 ;;
    *) break ;;
  esac
done
if [[ ! -x devel/lib/sa_livo/rectify_mei_bag || ! -x devel/lib/sa_livo/fastlivo_mapping ]]; then
  bash scripts/build_ros1.sh
fi
set +u
source /opt/ros/noetic/setup.bash
source devel/setup.bash
set -u
export ROS_HOME="$SA_LIVO_WORKSPACE/.ros"
export ROS_LOG_DIR="$ROS_HOME/log"
export LD_LIBRARY_PATH="$SA_LIVO_WORKSPACE/.deps/install/lib:${LD_LIBRARY_PATH:-}"
mkdir -p "$ROS_LOG_DIR"
SA_LIVO_CORRIDOR_ROOT="$SA_LIVO_WORKSPACE/validation/corridor01_avia_livo"
SA_LIVO_CORRIDOR_INPUT="$SA_LIVO_CORRIDOR_ROOT/input"
SA_LIVO_CORRIDOR_PROFILE="$SA_LIVO_WORKSPACE/src/sa_livo/config/corridor01_avia_livo_profile.yaml"
if [[ ! -d "$SA_LIVO_CORRIDOR_INPUT" ]]; then
  echo "首次准备 Corridor01：Avia + Avia IMU + RGB；图像 MEI 校正和消息转换。"
  /usr/bin/python3 scripts/prepare_livo_dataset.py \
    --profile "$SA_LIVO_CORRIDOR_PROFILE" --output "$SA_LIVO_CORRIDOR_INPUT"
else
  /usr/bin/python3 - "$SA_LIVO_CORRIDOR_PROFILE" "$SA_LIVO_CORRIDOR_INPUT" <<'PY'
import json,sys
from pathlib import Path
sys.path.insert(0,'scripts')
from prepare_livo_dataset import digest,absolute
import yaml
profile_path,prepared=map(Path,sys.argv[1:])
if not (prepared/'normalized_livo.bag').is_file() or not (prepared/'input_identity.json').is_file():
 raise SystemExit('输入准备未完成，请检查 validation/corridor01_avia_livo/input 下的日志。')
profile=yaml.safe_load(profile_path.read_text())
identity=json.loads((prepared/'input_identity.json').read_text())
checks={'profile_sha256':profile_path,'input_sha256':absolute(profile['bag']),
 'calibration_sha256':absolute(profile['calibration']),
 'converter_sha256':Path('devel/lib/sa_livo/rectify_mei_bag')}
for key,path in checks.items():
 if digest(path)!=identity[key]:
  raise SystemExit('缓存与当前输入不一致（'+key+'），请将旧 input 目录移走后重新准备。')
print('复用已核验的 Corridor01 派生输入。')
PY
fi
if [[ "$SA_LIVO_CORRIDOR_PREPARE_ONLY" == true ]]; then
  echo "准备完成：$SA_LIVO_CORRIDOR_INPUT"
  exit 0
fi
mkdir -p "$SA_LIVO_CORRIDOR_ROOT/runs"
SA_LIVO_CORRIDOR_RUN="$(mktemp -d "$SA_LIVO_CORRIDOR_ROOT/runs/run_XXXXXXXX")"
SA_LIVO_CORRIDOR_CALIBRATION_ID="$(/usr/bin/python3 - "$SA_LIVO_CORRIDOR_INPUT/input_identity.json" <<'PY'
import json,sys
print('corridor01_avia_' + json.load(open(sys.argv[1]))['calibration_sha256'])
PY
)"
# Save a run-specific sensor snapshot so legacy trajectory files also remain distinct.
/usr/bin/python3 - "$SA_LIVO_CORRIDOR_INPUT/parameters.yaml" "$SA_LIVO_CORRIDOR_RUN/parameters.yaml" <<'PY'
from pathlib import Path
import sys,yaml
source,target=map(Path,sys.argv[1:])
params=yaml.safe_load(source.read_text())
params['evo']['seq_name']='corridor01_avia_' + target.parent.name
target.write_text(yaml.safe_dump(params,sort_keys=False))
PY
echo "结果目录：$SA_LIVO_CORRIDOR_RUN"
SA_LIVO_CORRIDOR_BAG="$SA_LIVO_CORRIDOR_INPUT/normalized_livo.bag"
SA_LIVO_CORRIDOR_SIM_TIME=false
if [[ "$SA_LIVO_CORRIDOR_EXTERNAL" == true ]]; then
  SA_LIVO_CORRIDOR_BAG=""
  SA_LIVO_CORRIDOR_SIM_TIME=true
  echo "等待外部 rosbag play，请在另一个终端执行："
  printf 'rosbag play --clock --delay=2 %q\n' "$SA_LIVO_CORRIDOR_INPUT/normalized_livo.bag"
else
  echo "进程内读取原 bag 的派生数据，无需另开 rosbag play。"
fi
if [[ -n "$SA_LIVO_CORRIDOR_MAP" ]]; then
  echo "启动 Corridor01 固定地图重定位：$SA_LIVO_CORRIDOR_MAP"
  exec roslaunch sa_livo localization.launch \
    sensor_params_file:="$SA_LIVO_CORRIDOR_RUN/parameters.yaml" \
    camera_params_file:="$SA_LIVO_CORRIDOR_INPUT/camera.yaml" \
    localization_overrides_file:="$SA_LIVO_WORKSPACE/src/sa_livo/config/corridor01_localization.yaml" \
    map_path:="$SA_LIVO_CORRIDOR_MAP" \
    calibration_id:="$SA_LIVO_CORRIDOR_CALIBRATION_ID" \
    direct_bag_path:="$SA_LIVO_CORRIDOR_BAG" \
    use_sim_time:="$SA_LIVO_CORRIDOR_SIM_TIME" \
    output_dir:="$SA_LIVO_CORRIDOR_RUN" \
    "$@"
fi
echo "启动 Corridor01 LIVO 建图（默认启用 BTC 回环）。"
exec roslaunch sa_livo mapping_livo_loop.launch \
  params_file:="$SA_LIVO_CORRIDOR_RUN/parameters.yaml" \
  camera_params_file:="$SA_LIVO_CORRIDOR_INPUT/camera.yaml" \
  direct_bag_path:="$SA_LIVO_CORRIDOR_BAG" \
  use_sim_time:="$SA_LIVO_CORRIDOR_SIM_TIME" \
  output_dir:="$SA_LIVO_CORRIDOR_RUN" \
  loop_enabled:=true \
  calibration_id:="$SA_LIVO_CORRIDOR_CALIBRATION_ID" \
  "$@"
