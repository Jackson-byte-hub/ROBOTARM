ROBOTARM – ROS 2 + Dynamixel XL430 + MoveIt 2
A ROS 2 Jazzy robot arm project using Dynamixel XL430 servos, ros2_control, and MoveIt 2 for motion planning and execution.
This repository demonstrates a full pipeline from URDF → hardware control → RViz → MoveIt planning → real motor motion.

✨ Features
ROS 2 Jazzy Jalisco
Dynamixel XL430-W250 servos
dynamixel_hardware_interface
ros2_control with joint_trajectory_controller
RViz visualization
MoveIt 2 motion planning and execution
Scalable from 1 DOF → 3 DOF → 6 DOF

🧱 Repository Structure
<img width="493" height="256" alt="image" src="https://github.com/user-attachments/assets/1906cba9-de58-47c4-b016-8431de0f52b4" />

⚙️ System Requirements
| Component | Version           |
| --------- | ----------------- |
| OS        | Ubuntu 24.04      |
| ROS 2     | Jazzy             |
| Dynamixel | XL430-W250        |
| USB       | OpenRB-150 / U2D2 |
| MoveIt    | MoveIt 2          |

📦 Dependencies

sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-moveit \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui

🛠️ Build the Workspace

cd ~/xl430_arm_ws
colcon build --symlink-install
source install/setup.bash

🔌 Hardware Setup
1. Set unique IDs for each XL430 using Dynamixel Wizard 2.0
2. Connect motors in daisy chain
3. Connect OpenRB/U2D2 via USB

Verify device:
ls /dev/ttyACM*

Expected:
/dev/ttyACM0

Demo Instruction:
1. bringup (ros2_control)

ros2 launch arm_bringup bringup.launch.py

Verify:

ros2 control list_hardware_interfaces
ros2 control list_controllers

Exepected:

joint_state_broadcaster  active
arm_controller           active

2.1. 🎮 Manual Control (RQT)

ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller

- Select arm_controller
- Move joints smoothly
- Motors should follow on RViz (visualize)

2.2. 🤖 MoveIt 2 Demo
Launch full MoveIt stack

ros2 launch arm_moveit_config demo.launch.py

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
