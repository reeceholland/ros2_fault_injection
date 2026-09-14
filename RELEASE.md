# Release Checklist

Use this checklist when preparing a tagged release of `ros2_fault_injection`.

## 1. Prepare the Branch

- Start from an up-to-date `main` branch.
- Create a release branch named `release/vX.Y.Z`.
- Confirm the working tree is clean before making release changes.

## 2. Update Metadata

- Update the version in `package.xml`.
- Update the version in `CMakeLists.txt`.
- Add a new entry to `CHANGELOG.rst`.
- Confirm the changelog only describes changes included in the release branch.

## 3. Build and Test

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ros2_fault_injection --allow-overriding ros2_fault_injection
colcon test --packages-select ros2_fault_injection --ctest-args --output-on-failure
colcon test-result --verbose