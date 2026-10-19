#!/usr/bin/env bash
# Runs box_tracker.py with the venv that holds torch + ultralytics (the ROS
# environment's PYTHONPATH still provides rclpy, cv_bridge and vision_msgs).
PY="${SM_PEGASUS_DEMO_1_TRACKER_PYTHON:-$HOME/venvs/box_tracker/bin/python}"
if [[ ! -x "$PY" ]]; then
  echo "box_tracker.sh: $PY not found - create the venv (see README, Box tracker)" >&2
  exit 1
fi
exec "$PY" "$(dirname "$(readlink -f "$0")")/box_tracker.py" "$@"
