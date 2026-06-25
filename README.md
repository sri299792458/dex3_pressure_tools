# Dex3 Pressure Tools

Read-only ROS 2 Humble tools for visualizing, probing, and recording Unitree
Dex3-1 right-hand tactile pressure data.

This repository was created for the lab's right Dex3-1 hand marked `214-R-T`.
It does not publish Dex3 command messages.

## What Is Included

- `dex3_pressure_visualizer`: RViz marker overlay on the Dex3 hand URDF.
- `dex3_pressure_raw_audit`: unfiltered 9 x 12 pressure matrix audit.
- `dex3_pressure_session_recorder`: baseline + free-touch recording tool.
- `dex3_pressure_calibrator`: guided target-by-target mapping checker.
- `config/dex3_right_pressure_seed.yaml`: explicit 33-taxel pose table.
- `description_files/urdf/g1_29dof_dx3.urdf`: default RViz robot model.
- `docs/assets/dex3_right_pressure_mapping.png`: reference mapping image.

## Requirements

- ROS 2 Humble.
- `unitree_hg` message package available in the workspace.
- The hand state topic, normally `/lf/dex3/right/state` or `/dex3/right/state`.

See [Dependencies](docs/DEPENDENCIES.md) for fresh-workspace install commands
for Unitree's ROS 2 message packages.

## Build

Place this repository inside a ROS 2 workspace, for example:

```bash
mkdir -p ~/dex3_ws/src
cp -a ~/dex3_pressure_tools ~/dex3_ws/src/

source /opt/ros/humble/setup.bash
cd ~/dex3_ws
colcon build --packages-select dex3_pressure_tools --symlink-install
source install/setup.bash
```

If `unitree_hg` is built in another workspace, source that workspace before
building this package.

## Environment

```bash
source /opt/ros/humble/setup.bash
source ~/dex3_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
```

## Quick Start

The default launch starts `robot_state_publisher`, RViz, and the pressure
visualizer using the packaged `g1_29dof_dx3.urdf`. It subscribes to the
low-frequency hand state topic `/lf/dex3/right/state`:

```bash
ros2 launch dex3_pressure_tools dex3_pressure_visualizer.launch.py
```

If another launch file is already publishing `/robot_description`, TF, and
right-hand joint states:

```bash
ros2 launch dex3_pressure_tools dex3_pressure_visualizer.launch.py \
  start_robot_state_publisher:=false \
  publish_robot_joint_state_defaults:=false
```

For the high-rate state topic:

```bash
ros2 launch dex3_pressure_tools dex3_pressure_visualizer.launch.py \
  state_topic:=/dex3/right/state
```

## Signal Model

The visualizer uses a simple raw-count signal model:

```python
valid = finite(raw) and abs(raw - 30000) > 1000
baseline = median(first baseline_samples valid raw samples)
noise = p99(abs(baseline_samples - baseline))
delta = max(raw - baseline, 0)
```

`delta` is only an uncalibrated raw-count increase above untouched baseline. It
is useful for contact visualization and mapping checks, not physical force. See
[Math And Signals](docs/MATH_AND_SIGNALS.md) for the exact defaults and
[Observations And Operating Notes](docs/OBSERVATIONS.md) for the measured noise
and `30000` sentinel evidence.

## Recommended Lab Workflow

1. Run the raw audit once after connecting a hand.
2. Run a free-touch session to characterize noise and touch magnitudes.
3. Launch RViz visualization.
4. Adjust `config/dex3_right_pressure_seed.yaml` only when repeated touch
   evidence shows a marker pose is wrong.
5. Keep the generated JSON/NPZ logs with the experiment notes.

## Documentation Map

- [Dependencies](docs/DEPENDENCIES.md): install Unitree messages and explain the packaged URDF.
- [Lab Protocols](docs/LAB_PROTOCOLS.md): exact commands to run at the robot.
- [Observations And Operating Notes](docs/OBSERVATIONS.md): measured facts from hand `214-R-T`.
- [Math And Signals](docs/MATH_AND_SIGNALS.md): validity, baseline, noise, delta, and color equations.
- [Mapping And Marker Editing](docs/MAPPING_AND_MARKERS.md): YAML marker poses and mapping reference image.

External reference:

- [Meko G1/Dex3-1 article with pressure ID/cell figure](https://www.mekosrl.it/post/n-2-2-unitree-g1-umanoide-robot-tutto-ci%C3%B2-che-c-%C3%A8-da-sapere-sviluppo-applicazioni-sdk-descriz)

## Safety

These tools subscribe to `unitree_hg/msg/HandState` and publish only RViz
markers plus optional `/joint_states`. They do not publish to `/dex3/right/cmd`.
