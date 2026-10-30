#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
mkdir -p /tmp/nav2-demo

python3 - <<'PY'
from pathlib import Path
import yaml

config = yaml.safe_load(
    Path('/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz').read_text())
config['Visualization Manager']['Views']['Current'].update(X=0.0, Y=0.0, Scale=70)
Path('/tmp/nav2-demo/view.rviz').write_text(yaml.safe_dump(config, sort_keys=False))
PY

pids=()
cleanup() {
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid"
    fi
  done
  wait
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

Xvfb :99 -screen 0 1440x900x24 -nolisten tcp \
  > /tmp/nav2-demo/xvfb.log 2>&1 &
pids+=("$!")
for attempt in {1..50}; do
  kill -0 "${pids[0]}"
  if [ -S /tmp/.X11-unix/X99 ]; then
    break
  fi
  sleep 0.1
done
test -S /tmp/.X11-unix/X99
openbox > /tmp/nav2-demo/openbox.log 2>&1 &
pids+=("$!")
x11vnc -display :99 -localhost -nopw -forever -shared -rfbport 5900 \
  > /tmp/nav2-demo/vnc.log 2>&1 &
pids+=("$!")
websockify --web=/usr/share/novnc 6080 localhost:5900 \
  > /tmp/nav2-demo/novnc.log 2>&1 &
pids+=("$!")
ros2 launch nav2_bringup tb3_simulation_launch.py \
  headless:="${HEADLESS:-False}" use_rviz:=True \
  rviz_config_file:=/tmp/nav2-demo/view.rviz \
  > /tmp/nav2-demo/bringup.log 2>&1 &
pids+=("$!")

for attempt in {1..100}; do
  if wmctrl -l | grep -q RViz; then
    wmctrl -r RViz -b add,maximized_vert,maximized_horz
    break
  fi
  sleep 0.1
done

echo 'Nav2 demo started. Logs: /tmp/nav2-demo; browser: port 6080/vnc.html'
wait -n "${pids[@]}"
echo 'A demo component exited; stopping the remaining components.' >&2
exit 1
