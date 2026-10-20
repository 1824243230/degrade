#!/usr/bin/env bash
set -euo pipefail
SA_LIVO_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${SA_LIVO_CLEAN_ENV:-0}" != 1 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-hyh}" LANG=C.UTF-8 \
    PATH=/usr/bin:/bin SA_LIVO_CLEAN_ENV=1 bash "$0" "$@"
fi
cd "$SA_LIVO_WORKSPACE"
SA_LIVO_INPUT="${1:-$SA_LIVO_WORKSPACE/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag}"
SA_LIVO_OUTPUT="${2:-$SA_LIVO_WORKSPACE/validation/long_corridor}"
SA_LIVO_REFERENCE="${3:-$SA_LIVO_WORKSPACE/Long_Corridor_Rosbag/ground_truth_path.csv}"
SA_LIVO_PARAMS="${4:-$SA_LIVO_WORKSPACE/src/sa_livo/config/long_corridor.yaml}"
mkdir -p "$SA_LIVO_OUTPUT"
SA_LIVO_OUTPUT="$(cd "$SA_LIVO_OUTPUT" && pwd)"
test -f "$SA_LIVO_INPUT"
test -f "$SA_LIVO_REFERENCE"
test -f "$SA_LIVO_PARAMS"
if [[ ! -x devel/lib/sa_livo/decode_velodyne_bag || ! -x devel/lib/sa_livo/fastlivo_mapping ]]; then
  bash scripts/build_ros1.sh
fi
set +u
source /opt/ros/noetic/setup.bash
source devel/setup.bash
set -u
export ROS_HOME="$SA_LIVO_WORKSPACE/.ros"
export ROS_LOG_DIR="$ROS_HOME/log"
export ROS_IP=127.0.0.1
export MPLCONFIGDIR="$SA_LIVO_OUTPUT/.matplotlib"
export LC_ALL=C
mkdir -p "$ROS_LOG_DIR"
SA_LIVO_CALIBRATION=/opt/ros/noetic/share/velodyne_pointcloud/params/VLP16db.yaml
SA_LIVO_DECODED="$SA_LIVO_OUTPUT/decoded_lio.bag"
# Check input identity and decoding inputs before reusing the derived bag.
/usr/bin/python3 - "$SA_LIVO_INPUT" "$SA_LIVO_CALIBRATION" "$SA_LIVO_OUTPUT" <<'PY'
import hashlib,json,sys
from pathlib import Path
src,cal,out=map(Path,sys.argv[1:])
identity=dict(input=str(src.resolve()),size=src.stat().st_size,mtime_ns=src.stat().st_mtime_ns,
              calibration_sha256=hashlib.sha256(cal.read_bytes()).hexdigest(),
              converter_sha256=hashlib.sha256(Path('devel/lib/sa_livo/decode_velodyne_bag').read_bytes()).hexdigest())
manifest=out/'input_identity.json'
if (out/'decoded_lio.bag').exists():
 if not manifest.exists() or json.loads(manifest.read_text()) != identity:
  raise SystemExit('Cached input differs; use a new output directory.')
else:
 manifest.write_text(json.dumps(identity,indent=2)+'\n')
PY
if [[ ! -f "$SA_LIVO_DECODED" ]]; then
  devel/lib/sa_livo/decode_velodyne_bag "$SA_LIVO_INPUT" "$SA_LIVO_DECODED" "$SA_LIVO_CALIBRATION" \
    > "$SA_LIVO_OUTPUT/decode.log" 2>&1
fi
SA_LIVO_PORT="$(/usr/bin/python3 - <<'PY'
import socket
with socket.socket() as s:
 s.bind(('127.0.0.1',0)); print(s.getsockname()[1])
PY
)"
export ROS_MASTER_URI="http://127.0.0.1:$SA_LIVO_PORT"
cp "$SA_LIVO_PARAMS" "$SA_LIVO_OUTPUT/parameters.yaml"
/usr/bin/time -f '{"wall_seconds": %e, "user_seconds": %U, "system_seconds": %S, "max_rss_kib": %M, "exit_code": %x}' \
  -o "$SA_LIVO_OUTPUT/runtime.json" \
  roslaunch --port "$SA_LIVO_PORT" sa_livo mapping_long_corridor.launch \
    direct_bag_path:="$SA_LIVO_DECODED" params_file:="$SA_LIVO_PARAMS" \
  > "$SA_LIVO_OUTPUT/replay.log" 2>&1
cp src/sa_livo/Log/result/long_corridor_lio.txt "$SA_LIVO_OUTPUT/trajectory.tum"
cp src/sa_livo/Log/result/long_corridor_lio_quality.csv "$SA_LIVO_OUTPUT/quality.csv"
# A required ROS node failure does not always propagate through roslaunch's exit code.
/usr/bin/python3 - "$SA_LIVO_OUTPUT/replay.log" <<'PY'
import re,sys
text=open(sys.argv[1]).read()
if 'SA-LIVO failed:' in text or '[ DIRECT BAG ] finished:' not in text:
 raise SystemExit('Replay did not finish successfully; inspect replay.log')
if not re.search(r'finished: imu=\d+ lidar=\d+ img=0',text):
 raise SystemExit('Expected image-free LiDAR/IMU replay')
PY
/usr/bin/python3 scripts/evaluate_long_corridor.py "$SA_LIVO_OUTPUT/trajectory.tum" \
  "$SA_LIVO_REFERENCE" --quality "$SA_LIVO_OUTPUT/quality.csv" --output "$SA_LIVO_OUTPUT" \
  > "$SA_LIVO_OUTPUT/evaluation.log"
echo "Validation complete: $SA_LIVO_OUTPUT/metrics.json"
