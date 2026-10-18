#!/usr/bin/env bash
# Gazebo (diff_drive, GUI) + STM32 serial bridge, no ROS processes.
# Usage: ./run.sh [port] [baud]
set -e
PORT=${1:-/dev/ttyACM0}
BAUD=${2:-2000000}
HERE=$(cd "$(dirname "$0")" && pwd)

# Gazebo Harmonic installed with ROS 2 Jazzy lives under the ROS prefix.
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi

trap 'kill $(jobs -p) 2>/dev/null; wait' EXIT INT TERM

gz sim -r "$HERE/diff_drive_stm32.sdf" &
# System Python on purpose: pyserial and protobuf live there, not in ~/.venv.
/usr/bin/python3 -u "$HERE/stm32_gz.py" --port "$PORT" --baud "$BAUD" &
wait
