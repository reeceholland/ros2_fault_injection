#!/usr/bin/env bash
# Called by headless_sim.yml after building both workspaces and extracting Unity.
set -eo pipefail

source /opt/ros/"$ROS_DISTRO"/setup.bash
source "$ROVER_WS/install/setup.bash"
source "$FAULT_INJECTION_WS/install/setup.bash"

unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE \
  FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI ROS_LOCALHOST_ONLY

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID="${CI_ROS_DOMAIN_ID:-190}"
export CI_LOG_DIR="$CI_LOG_DIR/navigation"
export CI_TIMEOUT=900s

READINESS_TIMEOUT="${CI_READINESS_TIMEOUT:-60}"
FAULT_SERVICE="/fault_injection/set_fault_state"

mkdir -p "$CI_LOG_DIR" "$CI_LOG_DIR/readiness" "$CI_LOG_DIR/telemetry"

bag_pid=""
navigation_pid=""
declare -a logger_pids=()
declare -a all_pids=()

register_pid() {
  all_pids+=("$1")
}

stop_group() {
  local pid="$1"

  [[ -n "$pid" ]] || return 0
  kill -0 "$pid" 2>/dev/null || return 0

  kill -INT -- "-$pid" 2>/dev/null || true
  for ((i=0; i<100; i++)); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.1
  done

  kill -TERM -- "-$pid" 2>/dev/null || true
  for ((i=0; i<20; i++)); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.1
  done

  kill -KILL -- "-$pid" 2>/dev/null || true
}

cleanup() {
  local rc=$?
  trap - EXIT INT TERM

  for pid in "${all_pids[@]}"; do
    stop_group "$pid"
  done

  for pid in "${all_pids[@]}"; do
    wait "$pid" 2>/dev/null || true
  done

  printf '%s\n' "$rc" > "$CI_LOG_DIR/exit-code.txt"
  exit "$rc"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

fail_if_core_process_exited() {
  if [[ -n "$navigation_pid" ]] && ! kill -0 "$navigation_pid" 2>/dev/null; then
    set +e
    wait "$navigation_pid"
    local rc=$?
    set -e
    echo "Navigation/Unity process exited during startup/readiness (exit=$rc)" >&2
    (( rc != 0 )) || rc=1
    exit "$rc"
  fi

  if [[ -n "$bag_pid" ]] && ! kill -0 "$bag_pid" 2>/dev/null; then
    echo "ROS bag recorder exited unexpectedly" >&2
    cat "$CI_LOG_DIR/rosbag.log" >&2 || true
    exit 1
  fi
}

check_topic_ready() {
  local name="$1"
  local topic="$2"
  local message_type="$3"
  local logfile="$CI_LOG_DIR/readiness/${name}.log"
  local pid rc

  echo "READINESS $name: checking $topic"

  setsid timeout --signal=TERM --kill-after=5s "${READINESS_TIMEOUT}s" \
    ros2 topic echo --once --qos-reliability best_effort "$topic" "$message_type" \
    >"$logfile" 2>&1 &
  pid=$!
  register_pid "$pid"

  while kill -0 "$pid" 2>/dev/null; do
    fail_if_core_process_exited
    sleep 0.1
  done

  set +e
  wait "$pid"
  rc=$?
  set -e

  if (( rc != 0 )); then
    echo "READINESS $name: FAIL" >&2
    cat "$logfile" >&2 || true
    exit 1
  fi

  echo "READINESS $name: PASS"
}

check_action_ready() {
  local name="$1"
  local action="$2"
  local logfile="$CI_LOG_DIR/readiness/${name}.log"
  local pid rc

  echo "READINESS $name: checking $action"

  setsid timeout --signal=TERM --kill-after=5s "${READINESS_TIMEOUT}s" \
    bash -c '
      action="$1"
      until ros2 action list -t 2>/dev/null | grep -Fq "${action} ["; do
        sleep 0.25
      done
      ros2 action list -t | grep -F "${action} ["
    ' _ "$action" \
    >"$logfile" 2>&1 &
  pid=$!
  register_pid "$pid"

  while kill -0 "$pid" 2>/dev/null; do
    fail_if_core_process_exited
    sleep 0.1
  done

  set +e
  wait "$pid"
  rc=$?
  set -e

  if (( rc != 0 )); then
    echo "READINESS $name: FAIL" >&2
    cat "$logfile" >&2 || true
    exit 1
  fi

  echo "READINESS $name: PASS"
}

check_service_ready() {
  local name="$1"
  local service="$2"
  local logfile="$CI_LOG_DIR/readiness/${name}.log"
  local pid rc

  echo "READINESS $name: checking $service"

  setsid timeout --signal=TERM --kill-after=5s "${READINESS_TIMEOUT}s" \
    bash -c '
      service="$1"
      until type="$(ros2 service type "$service" 2>/dev/null)"; do
        sleep 0.25
      done
      printf "service: %s\ntype: %s\n" "$service" "$type"
    ' _ "$service" \
    >"$logfile" 2>&1 &
  pid=$!
  register_pid "$pid"

  while kill -0 "$pid" 2>/dev/null; do
    fail_if_core_process_exited
    sleep 0.1
  done

  set +e
  wait "$pid"
  rc=$?
  set -e

  if (( rc != 0 )); then
    echo "READINESS $name: FAIL" >&2
    cat "$logfile" >&2 || true
    exit 1
  fi

  echo "READINESS $name: PASS"
}

start_topic_logger() {
  local name="$1"
  local topic="$2"
  local logfile="$CI_LOG_DIR/telemetry/${name}.log"

  setsid ros2 topic echo "$topic" >"$logfile" 2>&1 &
  logger_pids+=("$!")
  register_pid "$!"
}

start_fault_service_watchdog() {
  local logfile="$CI_LOG_DIR/telemetry/fault-service.log"

  setsid bash -c '
    service="$1"
    while true; do
      timestamp="$(date --iso-8601=seconds)"
      if type="$(ros2 service type "$service" 2>/dev/null)"; then
        printf "%s available %s [%s]\n" "$timestamp" "$service" "$type"
      else
        printf "%s LOST %s\n" "$timestamp" "$service" >&2
        exit 1
      fi
      sleep 1
    done
  ' _ "$FAULT_SERVICE" >"$logfile" 2>&1 &

  logger_pids+=("$!")
  register_pid "$!"
}

# Capture diagnostics before the system under test starts.
setsid ros2 bag record -o "$CI_LOG_DIR/rosbag" \
  /clock /scan_raw /scan /map /odom /odom_raw /tf /tf_static \
  /cmd_vel /cmd_vel_smoothed /diff_drive_controller/cmd_vel \
  /platform/motors/cmd_raw /platform/motors/cmd /platform/motors/feedback \
  /navigate_to_pose/_action/feedback /navigate_to_pose/_action/status \
  /ci/ground_truth/odom /test/collision_status \
  > "$CI_LOG_DIR/rosbag.log" 2>&1 &
bag_pid=$!
register_pid "$bag_pid"

sleep 2
if ! kill -0 "$bag_pid" 2>/dev/null; then
  cat "$CI_LOG_DIR/rosbag.log"
  exit 1
fi

# Includes the supplied four waypoints and fault schedule from the pinned repo.
cp "$UNITY_TESTS_DIR/tools/ci/waypoints.json" "$CI_LOG_DIR/waypoints.json"
cp "$UNITY_TESTS_DIR/tools/ci/navigation_faults.yaml" "$CI_LOG_DIR/navigation_faults.yaml"

printf 'Unity scripts: %s\nRover: %s\nSimulator: %s\nSHA256: %s\n' \
  "$UNITY_TEST_COMMIT" "$ROVER_UNITY_SIMULATION_COMMIT" \
  "$SIMULATOR_URL" "$SIMULATOR_SHA256" \
  > "$CI_LOG_DIR/versions.txt"

# Run the original navigation command in the background only so this wrapper
# can perform readiness checks and detect an early Unity/navigation crash.
setsid bash -c '
  set -eo pipefail
  python3 "$FAULT_INJECTION_WS/tools/ci/xvfb_supervisor.py" \
    --log-dir "$CI_LOG_DIR" -- \
    timeout --signal=TERM --kill-after=20s 930s \
    bash "$1" --feedback-interval 5 \
    > >(tee "$2" >(awk "/OBSERVER (EVENT|FAIL)/ { print; fflush(); }" > "$4")) \
    2> >(tee "$3" >&2)
' _ \
  "$UNITY_TESTS_DIR/tools/ci/run_headless_navigation.sh" \
  "$CI_LOG_DIR/navigation-console.log" \
  "$CI_LOG_DIR/navigation-stderr.log" \
  "$CI_LOG_DIR/telemetry/observer.log" &
navigation_pid=$!
register_pid "$navigation_pid"

# Explicit types let echo wait for discovery; inference exits immediately when
# a topic has no publisher yet, which used to trigger premature cleanup.
# Report each readiness condition independently. These are bounded, and each
# probe also watches the navigation runner and rosbag for an early crash.
check_topic_ready "clock" "/clock" "rosgraph_msgs/msg/Clock"
check_topic_ready "scan-raw" "/scan_raw" "sensor_msgs/msg/LaserScan"
check_topic_ready "scan-injected" "/scan" "sensor_msgs/msg/LaserScan"
check_topic_ready "tf" "/tf" "tf2_msgs/msg/TFMessage"
check_action_ready "nav2" "/navigate_to_pose"
check_service_ready "fault-service" "$FAULT_SERVICE"

# The resilience observer is in-process, so its ROS-facing telemetry is checked
# through the streams it consumes rather than through a fictitious observer topic.
check_topic_ready "observer-ground-truth" "/ci/ground_truth/odom" "nav_msgs/msg/Odometry"
check_topic_ready "observer-command" "/platform/motors/cmd" "sensor_msgs/msg/JointState"
check_topic_ready "observer-collision" "/test/collision_status" "std_msgs/msg/String"

echo "All readiness checks passed."

# Separate human-readable telemetry logs, in addition to the rosbag.
start_topic_logger "clock" "/clock"
start_topic_logger "scan-raw" "/scan_raw"
start_topic_logger "scan-injected" "/scan"
start_topic_logger "tf" "/tf"
start_topic_logger "nav2-feedback" "/navigate_to_pose/_action/feedback"
start_topic_logger "nav2-status" "/navigate_to_pose/_action/status"
start_topic_logger "observer-ground-truth" "/ci/ground_truth/odom"
start_topic_logger "observer-command" "/platform/motors/cmd"
start_topic_logger "observer-collision" "/test/collision_status"
start_fault_service_watchdog

# Supervise the original navigation runner plus all required diagnostics.
while kill -0 "$navigation_pid" 2>/dev/null; do
  if ! kill -0 "$bag_pid" 2>/dev/null; then
    echo "ROS bag recorder exited during navigation" >&2
    cat "$CI_LOG_DIR/rosbag.log" >&2 || true
    exit 1
  fi

  for pid in "${logger_pids[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "Required telemetry logger/watchdog exited during navigation (pid=$pid)" >&2
      exit 1
    fi
  done

  sleep 0.1
done

set +e
wait "$navigation_pid"
status=$?
set -e

echo "Navigation/Unity process exited with status $status"
exit "$status"