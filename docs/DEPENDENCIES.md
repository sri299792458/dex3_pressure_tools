# Dependencies

This package includes a known working G1 29DoF + Dex3 URDF and its referenced
meshes so RViz works out of the box. It does not vendor Unitree's ROS 2 message
packages; lab members should know which `unitree_hg` definitions they are using.

## Required: `unitree_hg`

The pressure nodes import:

```python
from unitree_hg.msg import HandState
```

`unitree_hg` is provided by Unitree's ROS 2 support repository:

https://github.com/unitreerobotics/unitree_ros2

Unitree's G1 ROS 2 documentation also points users to the ROS 2 communication
routine:

https://support.unitree.com/home/en/G1_developer/ros2_communication_routine

### Install Into A Fresh Workspace

On Ubuntu 22.04 / ROS 2 Humble:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-vcstool \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosidl-generator-dds-idl \
  libyaml-cpp-dev
```

Create a workspace and import Unitree's source dependency:

```bash
mkdir -p ~/dex3_ws/src
cd ~/dex3_ws/src

git clone <dex3_pressure_tools_repo_url>
vcs import . < dex3_pressure_tools/dependencies.repos
```

Build:

```bash
source /opt/ros/humble/setup.bash
cd ~/dex3_ws
colcon build --packages-select \
  unitree_api unitree_go unitree_hg dex3_pressure_tools \
  --symlink-install
source install/setup.bash
```

Check:

```bash
ros2 interface show unitree_hg/msg/HandState
ros2 pkg executables dex3_pressure_tools
```

### Use An Existing Lab Workspace

If the lab machine already has `unitree_hg` built, source that workspace before
using this package:

```bash
source /opt/ros/humble/setup.bash
source ~/g1pilot_ws/install/setup.bash
source ~/dex3_ws/install/setup.bash
```

Check:

```bash
ros2 interface show unitree_hg/msg/HandState
```

## Packaged URDF

The visualizer publishes RViz markers attached to existing TF frames such as:

```text
right_hand_palm_link
right_hand_thumb_1_link
right_hand_thumb_2_link
right_hand_index_0_link
right_hand_index_1_link
right_hand_middle_0_link
right_hand_middle_1_link
```

The repository includes:

```text
description_files/urdf/g1_29dof_dx3.urdf
description_files/meshes/*.STL
```

The default launch uses this packaged model, so lab members do not need a
separate robot-description repository just to view the pressure markers. See
[Lab Protocols](LAB_PROTOCOLS.md) for launch commands.

## DDS Environment

For the physical hand:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
```

If topic discovery fails, follow Unitree's network-interface configuration in
their ROS 2 communication documentation. The exact interface name depends on the
Ethernet adapter connected to the robot.
