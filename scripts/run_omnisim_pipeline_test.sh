#!/usr/bin/env bash

# Copyright 2026 Reece Holland
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -euo pipefail

OMNISIM_HOME="${OMNISIM_HOME:-$HOME/omnisim}"
FAULT_INJECTION_WS="${FAULT_INJECTION_WS:-$HOME/fault_injection_ws}"
SCENARIO_FILE="${SCENARIO_FILE:-$FAULT_INJECTION_WS/src/ros2_fault_injection/config/omnisim_faults.yaml}"
REPORT_FILE="${REPORT_FILE:-$HOME/tmp/omnisim_fault_report.md}"
WORLD_FILE="${WORLD_FILE:-$OMNISIM_HOME/projects/samples/demos/worlds/chat/omnilink_husky.omniworld}"
TIMEOUT_SECONDS="${TIMEOUT_SECONDS:-65.0}"
TEST_LINEAR_X="${TEST_LINEAR_X:--0.3}"
TEST_ANGULAR_Z="${TEST_ANGULAR_Z:-0.0}"
CLEAN_START="${CLEAN_START:-true}"

HARNESS_URL="${HARNESS_URL:-http://127.0.0.1:6789}"
BRIDGE_URL="${BRIDGE_URL:-http://127.0.0.1:8765}"

HARNESS_PID=""
BRIDGE_PIDS=()
CMD_VEL_PID=""
START_STATE_FILE=""
END_STATE_FILE=""

cleanup()
{
  for pid in "$CMD_VEL_PID" "$HARNESS_PID" "${BRIDGE_PIDS[@]}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
}

clean_existing_processes()
{
  if [[ "$CLEAN_START" != "true" ]]; then
    return
  fi

  echo "Stopping existing OmniSim test processes..."
  pkill -f "omnisim_harness.py" 2>/dev/null || true
  pkill -f "omnilink_mobile_bridge.py" 2>/dev/null || true
  pkill -f "harness_supervisor.py" 2>/dev/null || true
  pkill -f "ros2 run omnisim_ros2 command_node" 2>/dev/null || true
  pkill -f "ros2 run omnisim_ros2 odom_node" 2>/dev/null || true
  pkill -f "omnisim_ros2/lib/omnisim_ros2/clock_node" 2>/dev/null || true
  pkill -f "omnisim_ros2/lib/omnisim_ros2/command_node" 2>/dev/null || true
  pkill -f "omnisim_ros2/lib/omnisim_ros2/odom_node" 2>/dev/null || true
  pkill -f "omnisim_ros2/lib/omnisim_ros2/robot_state_node" 2>/dev/null || true
  pkill -f "omnisim_ros2/lib/omnisim_ros2/sensor_node" 2>/dev/null || true
  pkill -f "omnisim_ros2/lib/omnisim_ros2/simulation_interfaces_node" 2>/dev/null || true
  pkill -f "fault_scenario_runner_node" 2>/dev/null || true
  pkill -f "ros2 topic pub.*cmd_vel_raw" 2>/dev/null || true
  sleep 2
}

source_ros_setup()
{
  local setup_file="$1"

  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

wait_for_http()
{
  local url="$1"
  local description="$2"
  local attempts="${3:-60}"

  for _ in $(seq 1 "$attempts"); do
    if curl -fsS "$url" >/dev/null 2>&1; then
      echo "Ready: $description"
      return 0
    fi

    sleep 1
  done

  echo "Timed out waiting for $description at $url" >&2
  return 1
}

print_omnisim_diagnostics()
{
  echo
  echo "OmniSim diagnostics:"
  echo "--- $HARNESS_URL/sim/state ---"
  curl -fsS "$HARNESS_URL/sim/state" 2>/dev/null || true
  echo
  echo "--- $BRIDGE_URL/get_robot_state ---"
  curl -fsS "$BRIDGE_URL/get_robot_state" 2>/dev/null || true
  echo
  echo "--- listening ports ---"
  ss -ltnp | grep -E "6789|6790|8765" || true
  echo "--- recent OmniSim log ---"
  tail -80 "$OMNISIM_HOME/omnisim_log.txt" 2>/dev/null || true
}

start_harness_if_needed()
{
  if curl -fsS "$HARNESS_URL/sim/state" >/dev/null 2>&1; then
    echo "Using existing OmniSim harness at $HARNESS_URL"
    return
  fi

  echo "Starting OmniSim harness..."
  (
    cd "$OMNISIM_HOME"
    OMNISIM_NO_WINDOW=1 python3 -m omnisim harness --auto-port --engine-mode realtime
  ) &
  HARNESS_PID="$!"

  wait_for_http "$HARNESS_URL/sim/state" "OmniSim harness"
}

load_world()
{
  echo "Loading OmniSim world: $WORLD_FILE"
  if ! curl -fsS -X POST "$HARNESS_URL/world/load" \
    -H "Content-Type: application/json" \
    -d "{\"path\":\"$WORLD_FILE\"}" >/tmp/ros2_fault_injection_omnisim_world_load.json; then
    echo "Failed to request OmniSim world load" >&2
    print_omnisim_diagnostics
    return 1
  fi

  echo "Waiting for robot bridge at $BRIDGE_URL..."
  if ! wait_for_http "$BRIDGE_URL/get_robot_state" "OmniSim robot bridge" 180; then
    echo "World load response:" >&2
    cat /tmp/ros2_fault_injection_omnisim_world_load.json >&2 || true
    print_omnisim_diagnostics
    return 1
  fi
}

start_ros_bridge()
{
  echo "Starting OmniSim ROS command and odom nodes..."
  source_ros_setup "$OMNISIM_HOME/packages/omnisim-ros2/install/setup.bash"

  ros2 run omnisim_ros2 command_node \
    --ros-args \
    -p use_sim_time:=false &
  BRIDGE_PIDS+=("$!")

  ros2 run omnisim_ros2 odom_node \
    --ros-args \
    -p use_sim_time:=false \
    -r /odom:=/odom_raw &
  BRIDGE_PIDS+=("$!")

  wait_for_topic "/odom_raw" "raw odometry"
}

wait_for_topic()
{
  local topic="$1"
  local description="$2"
  local attempts="${3:-60}"

  for _ in $(seq 1 "$attempts"); do
    if ros2 topic list | grep -qx "$topic"; then
      echo "Ready: $description topic $topic"
      return 0
    fi

    sleep 1
  done

  echo "Timed out waiting for $description topic $topic" >&2
  return 1
}

start_command_publisher()
{
  echo "Publishing test velocity commands on /cmd_vel_raw..."
  echo "Command: linear.x=$TEST_LINEAR_X angular.z=$TEST_ANGULAR_Z"
  ros2 topic pub -r 10 /cmd_vel_raw geometry_msgs/msg/Twist \
    "{linear: {x: $TEST_LINEAR_X}, angular: {z: $TEST_ANGULAR_Z}}" \
    >/tmp/ros2_fault_injection_cmd_vel_raw.log 2>&1 &
  CMD_VEL_PID="$!"
}

capture_robot_state()
{
  local output_file="$1"

  curl -fsS "$BRIDGE_URL/get_robot_state" > "$output_file"
}

append_motion_summary()
{
  local scenario_exit_code="$1"

  if [[ -z "$START_STATE_FILE" ]] || [[ -z "$END_STATE_FILE" ]]; then
    return
  fi

  python3 - "$REPORT_FILE" "$START_STATE_FILE" "$END_STATE_FILE" "$scenario_exit_code" <<'PY'
import json
import math
import sys

report_file, start_file, end_file, scenario_exit_code = sys.argv[1:5]

with open(start_file, "r", encoding="utf-8") as f:
    start = json.load(f)

with open(end_file, "r", encoding="utf-8") as f:
    end = json.load(f)

start_x = float(start.get("x", 0.0))
start_y = float(start.get("y", 0.0))
end_x = float(end.get("x", 0.0))
end_y = float(end.get("y", 0.0))
dx = end_x - start_x
dy = end_y - start_y
distance = math.hypot(dx, dy)
start_sim_time = start.get("sim_time", "unknown")
end_sim_time = end.get("sim_time", "unknown")

with open(report_file, "a", encoding="utf-8") as f:
    f.write("\n## OmniSim Motion Summary\n\n")
    f.write("| Field | Value |\n")
    f.write("| --- | --- |\n")
    f.write(f"| Scenario runner exit code | `{scenario_exit_code}` |\n")
    f.write(f"| Start pose | x={start_x:.6f}, y={start_y:.6f}, yaw={float(start.get('yaw', 0.0)):.6f} |\n")
    f.write(f"| End pose | x={end_x:.6f}, y={end_y:.6f}, yaw={float(end.get('yaw', 0.0)):.6f} |\n")
    f.write(f"| Delta pose | dx={dx:.6f}, dy={dy:.6f} |\n")
    f.write(f"| Planar distance travelled | {distance:.6f} m |\n")
    f.write(f"| Start sim time | `{start_sim_time}` |\n")
    f.write(f"| End sim time | `{end_sim_time}` |\n")
    f.write(f"| Final bridge mode | `{end.get('mode', 'unknown')}` |\n")
    f.write(f"| Final bridge fault | `{end.get('fault')}` |\n")
PY
}

run_scenario()
{
  mkdir -p "$(dirname "$REPORT_FILE")"

  source_ros_setup "$FAULT_INJECTION_WS/install/setup.bash"

  echo "Running fault scenario: $SCENARIO_FILE"
  ros2 run ros2_fault_injection fault_scenario_runner_node \
    --ros-args \
    -p scenario_file:="$SCENARIO_FILE" \
    -p timeout:="$TIMEOUT_SECONDS" \
    -p report_file:="$REPORT_FILE"
}

main()
{
  trap cleanup EXIT

  if [[ ! -f "$SCENARIO_FILE" ]]; then
    echo "Scenario file does not exist: $SCENARIO_FILE" >&2
    exit 1
  fi

  if [[ ! -f "$WORLD_FILE" ]]; then
    echo "World file does not exist: $WORLD_FILE" >&2
    exit 1
  fi

  source_ros_setup /opt/ros/jazzy/setup.bash
  clean_existing_processes

  start_harness_if_needed
  load_world
  start_ros_bridge
  start_command_publisher

  START_STATE_FILE="$(mktemp)"
  END_STATE_FILE="$(mktemp)"
  capture_robot_state "$START_STATE_FILE"

  scenario_exit_code=0
  run_scenario || scenario_exit_code="$?"

  capture_robot_state "$END_STATE_FILE"
  append_motion_summary "$scenario_exit_code"

  echo
  echo "Report written to: $REPORT_FILE"

  exit "$scenario_exit_code"
}

main "$@"
