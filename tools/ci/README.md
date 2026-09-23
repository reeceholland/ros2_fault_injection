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
The player is release `ci-sim-v0.1.1`, checked against the archive SHA256
`f05d9191fbf28f7035b4594ccf90a71e7218aaec3bb712843abe2029cf267bec`.
Use the archive's hash, not the hash of its `SHA256SUMS` file.

## Results

The original dropout artifacts remain at the artifact root. Navigation files
are under `navigation/`: `test.log`, `results.json`, `junit.xml`, ROS/Unity logs,
recorded ROS bag, `waypoints.json`, `navigation_faults.yaml`, `versions.txt`, and
`exit-code.txt`. Early failures may prevent some files from being produced.
The Actions summary shows each navigation goal's result and final pose errors.
Artifacts upload even when a test fails.

Navigation prints every action feedback message, uses the four map-frame points
from the pinned Unity repository and injects a one-second scan dropout at Point 2.
Its runner limit is 900 seconds, outer limit 930 seconds and Actions step limit
17 minutes. The whole build/test job allows 60 minutes. ROS recording shares the
runner's localhost discovery and domain 190.

`run_navigation_workflow.sh` is a workflow helper, not a standalone installer;
it expects the paths and pinned-version environment variables set by the workflow.
