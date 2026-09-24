# Server navigation workflow

`.github/workflows/headless_sim.yml` builds the ROS workspaces, downloads the
checksum-pinned Unity Linux release and runs dropout followed by waypoint
navigation on the `self-hosted, linux, x64, ros2-jazzy` runner. Navigation is
skipped if dropout fails. Nothing needs to be copied manually to the server.

## Before dispatching

Publish the updated `rugged_rover` launch files, including the Unity Nav2/SLAM
launch options and Nav2 simulation-time fix. These changes were still local and
uncommitted when this workflow was extended. The previous pinned rover commit
does not contain those launch options.

The **Run workflow** form requires `rover_commit`: the full 40-character commit
SHA containing those changes, pushed to the rover repository. The workflow
checks out that exact commit and rejects missing launch options before building.
This preflight does not replace runtime verification of simulation-time behavior.

The form also requires `unity_test_commit`: a published full Unity repository
commit SHA containing the CI runner's explicit `use_motor_fault_injection:=true`
argument. The previous pinned revision does not include this compatibility fix.
The workflow checks this before building.
The form also requires `simulator_url` and `simulator_sha256`. Build a new Linux
player containing `CiObserverTelemetry`, publish its archive as a Unity repository
release asset, and supply its download URL and archive hash. The old v0.1.1 player
does not have observer telemetry. Source revision inputs are actually used by checkout;
they no longer silently fall back to hard-coded revisions.

## Results

The original dropout artifacts remain at the artifact root. Navigation files
are under `navigation/`: `test.log`, `results.json`, `junit.xml`, ROS/Unity logs,
recorded ROS bag, `waypoints.json`, `navigation_faults.yaml`, `versions.txt`, and
`exit-code.txt`. Early failures may prevent some files from being produced.
The Actions summary shows each navigation goal's result and final pose errors,
plus a resilience/collision section with failed assertions, contact objects/counts
and stopping measurements. Observer JSON/JUnit are under `navigation/observer/`.
The navigation console streams `OBSERVER EVENT` and `OBSERVER FAIL`, with failed
assertions also rendered as GitHub error annotations. Missing observer results are
shown as **NO RESULT**, never PASS.
Artifacts upload even when a test fails.

Navigation prints feedback every five seconds, uses the four map-frame points
from the pinned Unity repository and overrides dropout duration to five wall seconds.
Injection waits until the rover is moving. `effective_scenario.json` records the
actual schedule; `observer_limits.json` records the thresholds. The ROS bag includes
`/test/collision_status` and `/ci/ground_truth/odom` as well as scans and wheel commands.
Its runner limit is 900 seconds, outer limit 930 seconds and Actions step limit
17 minutes. The whole build/test job allows 60 minutes. ROS recording shares the
runner's localhost discovery and domain 190.

`run_navigation_workflow.sh` is a workflow helper, not a standalone installer;
it expects the paths and pinned-version environment variables set by the workflow.

Local reporting tests:

```bash
python3 -m unittest discover -s tools/ci -p test_observer_summary.py
```
