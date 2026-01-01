ROBOTARM – ROS 2 + Dynamixel XL430 + MoveIt 2
A ROS 2 Humble robot arm project using Dynamixel XL430 servos, ros2_control, and MoveIt 2 for motion planning and execution.
This repository demonstrates a full pipeline from URDF → hardware control → RViz → MoveIt planning → real motor motion.

✨ Features
ROS 2 Humble 
Dynamixel XL430-W250 servos
dynamixel_hardware_interface
ros2_control with joint_trajectory_controller
RViz visualization
MoveIt 2 motion planning and execution
Scalable from 1 DOF → 3 DOF → 6 DOF

🧱 Repository Structure (might not be updated yet)

<img width="493" height="256" alt="image" src="https://github.com/user-attachments/assets/1906cba9-de58-47c4-b016-8431de0f52b4" />

⚙️ System Requirements
| Component | Version           |
| --------- | ----------------- |
| OS        | Ubuntu 22.04      |
| ROS 2     | Humble            |
| Dynamixel | XL430-W250        |
| USB       | OpenRB-150 / U2D2 |
| MoveIt    | MoveIt 2          |

📦 Dependencies

`sudo apt update`

```
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui
```
  
🛠️ Build the Workspace

`cd ~/<YOURWORKSPACE>`

`colcon build --symlink-install`

`source install/setup.bash`

🔌 Hardware Setup
1. Set unique IDs for each XL430 using Dynamixel Wizard 2.0
2. Connect motors in daisy chain
3. Connect OpenRB/U2D2 via USB

Verify device:
`ls /dev/ttyACM*`

Expected:
/dev/ttyACM0

Demo Instruction:
1. bringup (ros2_control)

Run `ros2 launch arm_bringup bringup.launch.py`

Verify:

`ros2 control list_hardware_interfaces`

`ros2 control list_controllers`

Exepected:

joint_state_broadcaster  active

arm_controller           active

2.1. 🎮 Manual Control (RQT)

`ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller`

- Select arm_controller
- Move joints smoothly
- Motors should follow on RViz (visualize)

2.2. 🤖 MoveIt 2 Demo
Launch full MoveIt stack

`ros2 launch arm_moveit_config demo.launch.py`

This starts:
- move_group
- RViz Motion Planning panel
- ros2_control hardware interface

🧭 Planning in RViz
Open MotionPlanning panel
Select planning group: arm
Set target pose or joint goal
Click Plan
Click Execute

✔ Robot model moves

✔ Real Dynamixel motors move

✔ Rviz2 model move accordingly

3. Additional commands

Build/Rebuild:

`colcon build --symlink-install`

Remove previous build for a full rebuild:

`cd ~/{YOUR WORRKSPACE}`

`rm -rf build install log`

Sourcing: (you have to do this after open a new terminal or rebuild

`source /opt/ros/jazzy/setup.bash (for ROS2)`

`source install/setup.bash (for the installs)`

Send tradjectory directly to move the servos:

```bash
ros2 topic pub -1 /arm_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory \
"
joint_names:
- joint_1
- joint_2
- joint_3
- joint_4
- joint_5
- joint_6
points:
- positions:
  - 3.0
  - 3.0
  - 3.0
  - 3.0
  - 3.0
  - 3.0
  time_from_start:
    sec: 1
"
ros2 topic pub -1 /arm_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory \
"
joint_names:
- joint_1
- joint_2
- joint_3
- joint_4
- joint_5
- joint_6
points:
- positions:
  - 0
  - 0
  - 0
  - 0
  - 0
  - 0
  time_from_start:
    sec: 1
"
ros2 topic pub -1 /arm_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory \
"
joint_names:
- joint_1
- joint_2
- joint_3
- joint_4
- joint_5
- joint_6
points:
- positions:
  - 3.0
  - 3.0
  - 3.0
  - 3.0
  - 3.0
  - 3.0
  time_from_start:
    sec: 1
"

```
Control GUI:
`sudo apt install ros-jazzy-rqt-joint-controller-manager`
`rqt`

📈 Making Motion Faster

1. Increase limits (in `joint_limits.yaml`)

```
max_velocity: 3.0
max_acceleration: 6.0
```

2. Increase controller rate (in `ros2_controllers.yaml`)

```
controller_manager:
  update_rate: 200
```

3. Tune XL430 PID gains (Wizard or YAML)

🔮 Future Work

 Torque control

 Gazebo simulation

 Force sensing

 Gripper integration

 👤 Author

Jackson

Robotics / ROS 2 / Embedded Systems

GitHub: https://github.com/Jackson-byte-hub

Ref:

https://github.com/ros-controls/ros2_control_demos/blob/master/example_7/doc/userdoc.rst

https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface?tab=readme-ov-file

https://github.com/ROBOTIS-GIT/DynamixelSDK

https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface/blob/main/param/dxl_model/2xc430_w250.model

https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html
