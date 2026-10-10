#!/usr/bin/env bash
# Subscribe and record today's sensor data. Does not publish robot commands.
set -euo pipefail

if [[ ${1:-} == --help || $# != 1 ]]; then
  cat <<'USAGE'
Usage: bash scripts/record_d1.sh RUN_LABEL

Source the verified Go2 ROS 2 Humble environment before running.
Suggested labels: smoke, static, straight, turn, loop_01, loop_02.
Stop each recording with Ctrl+C and wait for rosbag to close.
Default output: /media/yaorz/Elements/unitree_go2/experiments/2026-10-10_D1_go2_1.
GO2_DATA_ROOT can explicitly select another output root.
Records sensor/state/TF topics; never publishes motion commands.
USAGE
  if [[ ${1:-} == --help ]]; then exit 0; else exit 2; fi
fi

run_label=$1
if [[ ! $run_label =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}$ ]]; then
  printf 'Run label must use 1–64 letters, digits, underscores or hyphens.\n' >&2
  exit 2
fi
if [[ ${ROS_DISTRO:-} != humble ]] || ! command -v ros2 >/dev/null; then
  printf 'Source the verified Go2 ROS 2 Humble environment first.\n' >&2
  exit 2
fi

workspace_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
data_root=${GO2_DATA_ROOT:-/media/yaorz/Elements/unitree_go2}
python3 "$workspace_root/scripts/check_external_storage.py" --root "$data_root" --probe
session_root="$data_root/experiments/2026-10-10_D1_go2_1"
run_dir="$session_root/${run_label}_$(TZ=Asia/Shanghai date +%H%M%S)_$$"
mkdir -p "$run_dir"

# Read-only discovery before starting the recorder. Save actual topic types.
timeout 20s ros2 topic list --spin-time 3 --no-daemon -t > "$run_dir/topics.txt"
python3 - "$run_dir/topics.txt" <<'PY'
from pathlib import Path
import sys
text = Path(sys.argv[1]).read_text()
required = {
    '/utlidar/cloud_deskewed': 'sensor_msgs/msg/PointCloud2',
    '/utlidar/robot_odom': 'nav_msgs/msg/Odometry',
}
for topic, type_name in required.items():
    if f'{topic} [{type_name}]' not in text:
        raise SystemExit(f'Missing expected topic/type: {topic} [{type_name}]. Check connection first.')
if not any(f'{topic} [unitree_go/msg/SportModeState]' in text
           for topic in ('/sportmodestate', '/lf/sportmodestate')):
    raise SystemExit('No SportModeState topic discovered. Check connection and state output first.')
PY

for topic in /utlidar/cloud_deskewed /utlidar/robot_odom /sportmodestate /lf/sportmodestate /tf /tf_static; do
  topic_label=${topic//\//_}
  timeout 10s ros2 topic info --spin-time 2 --no-daemon --verbose "$topic" \
    > "$run_dir/topic${topic_label}.txt" 2>&1 || true
done

python3 - "$run_dir" "$run_label" "$workspace_root" "$data_root" <<'PY'
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import os
import shutil
import subprocess
import sys
import yaml
run_dir = Path(sys.argv[1])
workspace = Path(sys.argv[3])
revision = subprocess.run(
    ['git', '-C', str(workspace), 'rev-parse', 'HEAD'],
    capture_output=True, text=True,
).stdout.strip()
metadata = {
    'session_id': '2026-10-10_D1',
    'robot_id': 'go2_1',
    'run_label': sys.argv[2],
    'data_root': sys.argv[4],
    'workspace_root': str(workspace),
    'prepared_at': datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
    'git_revision': revision,
    'ros_distro': os.environ.get('ROS_DISTRO'),
    'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION'),
    'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', 'default'),
    'clock_mode': 'recorder system clock; retain original message header timestamps',
    'control_mode': 'manual remote control; recorder publishes no motion commands',
    'operator': 'TO_FILL',
    'observer': 'TO_FILL',
    'scene': 'TO_FILL',
    'initial_pose_reference': 'TO_FILL',
    'robot_firmware': 'TO_FILL',
    'source_timestamp_meaning': 'TO_CONFIRM',
    'result': 'NOT_YET_RECORDED',
}
(run_dir / 'run_metadata.yaml').write_text(yaml.safe_dump(metadata, allow_unicode=True, sort_keys=False))
(run_dir / 'events.csv').write_text('time_local,event,notes\n')
(run_dir / 'source_status.txt').write_text(subprocess.run(
    ['git', '-C', str(workspace), 'status', '--short', '--', 'src', 'docs'],
    capture_output=True, text=True,
).stdout)
config_dir = run_dir / 'config'
config_dir.mkdir()
for config in (workspace / 'src').glob('go2*/config/*.yaml'):
    target = config_dir / config.parent.parent.name / config.name
    target.parent.mkdir(exist_ok=True)
    shutil.copy2(config, target)
PY

printf '\nRecording directory: %s\n' "$run_dir"
printf 'Use manual remote control for the registered sequence. Ctrl+C stops recording.\n'
exec ros2 bag record --storage sqlite3 --output "$run_dir/bag" \
  --max-bag-size 1073741824 \
  --regex '^/(utlidar/(cloud|cloud_deskewed|cloud_base|robot_odom|imu)|sportmodestate|lf/sportmodestate|tf|tf_static)$'
