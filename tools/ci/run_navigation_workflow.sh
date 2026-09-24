#!/usr/bin/env bash
# Called by headless_sim.yml after building both workspaces and extracting Unity.
set -eo pipefail
source /opt/ros/"$ROS_DISTRO"/setup.bash
source "$ROVER_WS/install/setup.bash"
source "$FAULT_INJECTION_WS/install/setup.bash"
unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID="${CI_ROS_DOMAIN_ID:-190}"
export CI_LOG_DIR="$CI_LOG_DIR/navigation"
export CI_TIMEOUT=900s
mkdir -p "$CI_LOG_DIR"
bag_pid=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$bag_pid" ]]; then
    kill -INT -- "-$bag_pid" 2>/dev/null || true
    for ((i=0; i<100; i++)); do
      kill -0 -- "-$bag_pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM -- "-$bag_pid" 2>/dev/null || true
    for ((i=0; i<20; i++)); do
      kill -0 -- "-$bag_pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL -- "-$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid ros2 bag record -o "$CI_LOG_DIR/rosbag" \
  /clock /scan_raw /scan /map /odom /odom_raw /tf /tf_static \
  /cmd_vel /cmd_vel_smoothed /diff_drive_controller/cmd_vel \
  /platform/motors/cmd_raw /platform/motors/cmd /platform/motors/feedback \
  /test/collision_status /ci/ground_truth/odom \
  /navigate_to_pose/_action/feedback /navigate_to_pose/_action/status \
  > "$CI_LOG_DIR/rosbag.log" 2>&1 &
bag_pid=$!
sleep 2
if ! kill -0 "$bag_pid" 2>/dev/null; then
  cat "$CI_LOG_DIR/rosbag.log"
  exit 1
fi
# Includes the supplied four waypoints and fault schedule from the pinned repo.
cp "$UNITY_TESTS_DIR/tools/ci/waypoints.json" "$CI_LOG_DIR/waypoints.json"
cp "$UNITY_TESTS_DIR/tools/ci/navigation_faults.yaml" "$CI_LOG_DIR/navigation_faults.yaml"
cp "$UNITY_TESTS_DIR/tools/ci/observer_limits.json" "$CI_LOG_DIR/observer_limits.json"
printf 'Unity scripts: %s\nRover: %s\nSimulator: %s\nSHA256: %s\n' \
  "$UNITY_TEST_COMMIT" "$ROVER_UNITY_SIMULATION_COMMIT" "$SIMULATOR_URL" "$SIMULATOR_SHA256" \
  > "$CI_LOG_DIR/versions.txt"
status=0
xvfb-run -a timeout --signal=TERM --kill-after=20s 930s \
  bash "$UNITY_TESTS_DIR/tools/ci/run_headless_navigation.sh" --feedback-interval 5 --dropout-duration 5 \
  > >(tee "$CI_LOG_DIR/navigation-console.log") \
  2> >(tee "$CI_LOG_DIR/navigation-stderr.log" >&2) || status=$?
# A dead recorder means this run did not capture the promised diagnostics.
if ! kill -0 "$bag_pid" 2>/dev/null; then
  echo 'ROS bag recorder exited during navigation' >&2
  [[ "$status" != 0 ]] || status=1
fi
printf '%s\n' "$status" > "$CI_LOG_DIR/exit-code.txt"
exit "$status"
