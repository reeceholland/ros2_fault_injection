#!/usr/bin/env bash
# Called by headless_sim.yml after building both workspaces and extracting Unity.
#
# Required extra environment:
#   CI_FAULT_SERVICE   Fault-injection control service to watch.
#   CI_OBSERVER_TOPIC  Observer telemetry topic to record.
#
# Optional:
#   CI_READINESS_TIMEOUT   Maximum startup/readiness time (default: 60s).
#   CI_ROS_DOMAIN_ID       ROS domain for this CI run (default: 190).

set -euo pipefail

source /opt/ros/"$ROS_DISTRO"/setup.bash
source "$ROVER_WS/install/setup.bash"
source "$FAULT_INJECTION_WS/install/setup.bash"

unset \
  ROS_STATIC_PEERS \
  ROS_DISCOVERY_SERVER \
  FASTRTPS_DEFAULT_PROFILES_FILE \
  FASTDDS_DEFAULT_PROFILES_FILE \
  CYCLONEDDS_URI \
  ROS_LOCALHOST_ONLY

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID="${CI_ROS_DOMAIN_ID:-190}"

: "${CI_LOG_DIR:?CI_LOG_DIR must be set}"
: "${UNITY_TESTS_DIR:?UNITY_TESTS_DIR must be set}"
: "${UNITY_TEST_COMMIT:?UNITY_TEST_COMMIT must be set}"
: "${ROVER_UNITY_SIMULATION_COMMIT:?ROVER_UNITY_SIMULATION_COMMIT must be set}"
: "${SIMULATOR_URL:?SIMULATOR_URL must be set}"
: "${SIMULATOR_SHA256:?SIMULATOR_SHA256 must be set}"
: "${CI_FAULT_SERVICE:?CI_FAULT_SERVICE must name the fault-injection control service}"
: "${CI_OBSERVER_TOPIC:?CI_OBSERVER_TOPIC must name the observer telemetry topic}"

export CI_LOG_DIR="$CI_LOG_DIR/navigation"
export CI_TIMEOUT=900s

READINESS_TIMEOUT="${CI_READINESS_TIMEOUT:-60s}"

mkdir -p \
  "$CI_LOG_DIR" \
  "$CI_LOG_DIR/readiness" \
  "$CI_LOG_DIR/telemetry"

declare -a cleanup_pids=()
declare -a active_pids=()
declare -A process_name=()
declare -A process_kind=()

navigation_pid=""
bag_pid=""

register_process() {
  local pid="$1"
  local name="$2"
  local kind="$3"

  cleanup_pids+=("$pid")
  active_pids+=("$pid")
  process_name["$pid"]="$name"
  process_kind["$pid"]="$kind"
}

remove_active_pid() {
  local dead_pid="$1"
  local remaining=()
  local pid

  for pid in "${active_pids[@]}"; do
    if [[ "$pid" != "$dead_pid" ]]; then
      remaining+=("$pid")
    fi
  done

  active_pids=("${remaining[@]}")
}

stop_process_group() {
  local pid="$1"

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
  local pid

  trap - EXIT INT TERM

  for pid in "${cleanup_pids[@]}"; do
    stop_process_group "$pid"
  done

  for pid in "${cleanup_pids[@]}"; do
    wait "$pid" 2>/dev/null || true
  done

  printf '%s\n' "$rc" > "$CI_LOG_DIR/exit-code.txt"
  exit "$rc"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

record_readiness_result() {
  local name="$1"
  local result="$2"
  local detail="${3:-}"

  printf '%-22s %s%s\n' \
    "$name" \
    "$result" \
    "${detail:+ - $detail}"

  printf '%s\t%s\t%s\n' \
    "$name" \
    "$result" \
    "$detail" \
    >> "$CI_LOG_DIR/readiness/summary.tsv"
}

start_topic_readiness_check() {
  local name="$1"
  local topic="$2"
  local logfile="$CI_LOG_DIR/readiness/${name}.log"

  setsid timeout --signal=TERM --kill-after=5s "$READINESS_TIMEOUT" \
    env PYTHONUNBUFFERED=1 ros2 topic echo --once "$topic" \
    > "$logfile" 2>&1 &

  register_process "$!" "$name" "readiness"
}

start_service_readiness_check() {
  local name="$1"
  local service="$2"
  local logfile="$CI_LOG_DIR/readiness/${name}.log"

  setsid timeout --signal=TERM --kill-after=5s "$READINESS_TIMEOUT" \
    bash -c '
      service="$1"
      while true; do
        if type="$(ros2 service type "$service" 2>/dev/null)"; then
          printf "service: %s\ntype: %s\n" "$service" "$type"
          exit 0
        fi
        sleep 0.25
      done
    ' _ "$service" \
    > "$logfile" 2>&1 &

  register_process "$!" "$name" "readiness"
}

start_action_readiness_check() {
  local name="$1"
  local action="$2"
  local logfile="$CI_LOG_DIR/readiness/${name}.log"

  setsid timeout --signal=TERM --kill-after=5s "$READINESS_TIMEOUT" \
    bash -c '
      action="$1"
      while true; do
        listing="$(ros2 action list -t 2>/dev/null || true)"
        if grep -Fq "${action} [" <<< "$listing"; then
          grep -F "${action} [" <<< "$listing"
          exit 0
        fi
        sleep 0.25
      done
    ' _ "$action" \
    > "$logfile" 2>&1 &

  register_process "$!" "$name" "readiness"
}

start_topic_logger() {
  local name="$1"
  local topic="$2"
  local logfile="$CI_LOG_DIR/telemetry/${name}.log"

  setsid env PYTHONUNBUFFERED=1 \
    ros2 topic echo "$topic" \
    > "$logfile" 2>&1 &

  register_process "$!" "$name" "logger"
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
  ' _ "$CI_FAULT_SERVICE" \
    > "$logfile" 2>&1 &

  register_process "$!" "fault-service-watchdog" "logger"
}

start_navigation_runner() {
  setsid bash -c '
    set -eo pipefail

    xvfb-run -a timeout --signal=TERM --kill-after=20s 930s \
      bash "$1" --feedback-interval 5 \
      > >(tee "$2") \
      2> >(tee "$3" >&2)
  ' _ \
    "$UNITY_TESTS_DIR/tools/ci/run_headless_navigation.sh" \
    "$CI_LOG_DIR/navigation-console.log" \
    "$CI_LOG_DIR/navigation-stderr.log" &

  navigation_pid=$!
  register_process "$navigation_pid" "navigation-runner" "navigation"
}

# ---------------------------------------------------------------------------
# Version/configuration artifacts
# ---------------------------------------------------------------------------

cp "$UNITY_TESTS_DIR/tools/ci/waypoints.json" \
  "$CI_LOG_DIR/waypoints.json"

cp "$UNITY_TESTS_DIR/tools/ci/navigation_faults.yaml" \
  "$CI_LOG_DIR/navigation_faults.yaml"

printf 'Unity scripts: %s\nRover: %s\nSimulator: %s\nSHA256: %s\n' \
  "$UNITY_TEST_COMMIT" \
  "$ROVER_UNITY_SIMULATION_COMMIT" \
  "$SIMULATOR_URL" \
  "$SIMULATOR_SHA256" \
  > "$CI_LOG_DIR/versions.txt"

: > "$CI_LOG_DIR/readiness/summary.tsv"

# ---------------------------------------------------------------------------
# Start the bag recorder before the system under test so startup failures are
# captured too.
# ---------------------------------------------------------------------------

setsid ros2 bag record -o "$CI_LOG_DIR/rosbag" \
  /clock \
  /scan_raw \
  /scan \
  /map \
  /odom \
  /odom_raw \
  /tf \
  /tf_static \
  /cmd_vel \
  /cmd_vel_smoothed \
  /diff_drive_controller/cmd_vel \
  /platform/motors/cmd_raw \
  /platform/motors/cmd \
  /platform/motors/feedback \
  /navigate_to_pose/_action/feedback \
  /navigate_to_pose/_action/status \
  "$CI_OBSERVER_TOPIC" \
  > "$CI_LOG_DIR/rosbag.log" 2>&1 &

bag_pid=$!
register_process "$bag_pid" "rosbag" "rosbag"

sleep 2
if ! kill -0 "$bag_pid" 2>/dev/null; then
  cat "$CI_LOG_DIR/rosbag.log" >&2
  exit 1
fi

# Start the navigation/Unity runner in the background so we can supervise it
# while readiness checks are still pending.
start_navigation_runner

# ---------------------------------------------------------------------------
# Independent readiness checks.
#
# Each produces its own artifact. They run concurrently, so one missing
# endpoint does not prevent the other checks from reporting their result.
# ---------------------------------------------------------------------------

start_topic_readiness_check "clock" "/clock"
start_topic_readiness_check "scan-raw" "/scan_raw"
start_topic_readiness_check "scan-injected" "/scan"
start_topic_readiness_check "tf" "/tf"
start_action_readiness_check "nav2" "/navigate_to_pose"
start_service_readiness_check "fault-service" "$CI_FAULT_SERVICE"
start_topic_readiness_check "observer" "$CI_OBSERVER_TOPIC"

readiness_remaining=7

while (( readiness_remaining > 0 )); do
  dead_pid=""
  set +e
  wait -n -p dead_pid "${active_pids[@]}"
  rc=$?
  set -e

  if [[ -z "$dead_pid" ]]; then
    echo "Process supervisor lost track of child processes" >&2
    exit 1
  fi

  name="${process_name[$dead_pid]:-unknown}"
  kind="${process_kind[$dead_pid]:-unknown}"
  remove_active_pid "$dead_pid"

  case "$kind" in
    readiness)
      if (( rc == 0 )); then
        record_readiness_result "$name" "PASS"
        ((readiness_remaining -= 1))
      else
        record_readiness_result "$name" "FAIL" "exit=$rc"
        echo "Readiness check '$name' failed; see $CI_LOG_DIR/readiness/${name}.log" >&2
        exit 1
      fi
      ;;

    navigation)
      # The system under test must not exit before readiness is complete.
      record_readiness_result \
        "navigation-runner" \
        "FAIL" \
        "exited before readiness completed (exit=$rc)"
      echo "Navigation/Unity runner exited before readiness completed (exit=$rc)" >&2
      exit 1
      ;;

    rosbag)
      echo "ROS bag recorder exited during readiness (exit=$rc)" >&2
      cat "$CI_LOG_DIR/rosbag.log" >&2 || true
      exit 1
      ;;

    *)
      echo "Unexpected required process '$name' exited during readiness (exit=$rc)" >&2
      exit 1
      ;;
  esac
done

echo "All readiness checks passed."

# ---------------------------------------------------------------------------
# Separate human-readable telemetry streams.
#
# The rosbag remains the canonical full-fidelity capture; these logs make it
# easy to diagnose a single subsystem from CI artifacts without replaying it.
# ---------------------------------------------------------------------------

start_topic_logger "clock" "/clock"
start_topic_logger "scan-raw" "/scan_raw"
start_topic_logger "scan-injected" "/scan"
start_topic_logger "tf" "/tf"
start_topic_logger "tf-static" "/tf_static"
start_topic_logger "nav2-feedback" "/navigate_to_pose/_action/feedback"
start_topic_logger "nav2-status" "/navigate_to_pose/_action/status"
start_topic_logger "observer" "$CI_OBSERVER_TOPIC"
start_fault_service_watchdog

# ---------------------------------------------------------------------------
# Supervise all required long-running processes.
#
# A logger/watchdog/rosbag failure is fatal while navigation is still running.
# If navigation has already exited, its exit status is authoritative; the
# other processes may be naturally disappearing as the ROS graph shuts down.
# ---------------------------------------------------------------------------

while true; do
  dead_pid=""
  set +e
  wait -n -p dead_pid "${active_pids[@]}"
  rc=$?
  set -e

  if [[ -z "$dead_pid" ]]; then
    echo "Process supervisor lost track of child processes" >&2
    exit 1
  fi

  name="${process_name[$dead_pid]:-unknown}"
  kind="${process_kind[$dead_pid]:-unknown}"
  remove_active_pid "$dead_pid"

  if [[ "$dead_pid" == "$navigation_pid" ]]; then
    echo "Navigation/Unity runner exited with status $rc"
    exit "$rc"
  fi

  # Avoid turning normal teardown into a false infrastructure failure if the
  # navigation process has already completed but another child was reaped first.
  if ! kill -0 "$navigation_pid" 2>/dev/null; then
    set +e
    wait "$navigation_pid"
    navigation_rc=$?
    set -e
    echo "Navigation/Unity runner exited with status $navigation_rc"
    exit "$navigation_rc"
  fi

  echo "Required process '$name' exited unexpectedly with status $rc" >&2

  if [[ "$kind" == "rosbag" ]]; then
    cat "$CI_LOG_DIR/rosbag.log" >&2 || true
  elif [[ -f "$CI_LOG_DIR/telemetry/${name}.log" ]]; then
    tail -n 100 "$CI_LOG_DIR/telemetry/${name}.log" >&2 || true
  fi

  exit 1
done