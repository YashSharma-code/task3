🤖 Robotic Arm Teleoperation with Keyboard Inputs (ROS 2 + Gazebo)

This repository implements a teleoperation system for a 6DOF robotic arm (UR5) in Gazebo, controlled using keyboard inputs. The system uses ROS 2 Humble and publishes joint commands to simulate arm movement.

📽️ Demo - https://youtu.be/n-Ft9fARapg

It shows:

Starting the simulation

Launching the teleop node

Controlling the robot using the keyboard


📦 Package Overview

This repository contains the following ROS 2 packages:

teleop_arm: Publishes joint trajectory commands based on keyboard input.

ur_simulation_gazebo: Launches the UR5 robot in a Gazebo world with controllers configured.

📋 Dependencies

Ensure you have the following installed:

ROS 2 Humble

Gazebo Classic

ros-humble-joint-state-publisher

ros-humble-joint-trajectory-controller

Install missing packages using:
```
sudo apt update
sudo apt install ros-humble-joint-state-publisher ros-humble-joint-trajectory-controller 
```
⚙️ Building the Package

Clone the repository and place it under your ROS 2 workspace directory:
```
mkdir ~/teleop_ws/

cd ~/teleop_ws/

git clone (https://github.com/YashSharma-code/task3)

cd ~/teleop_ws/src/

git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation
```
Build the workspace:
```
cd ~/teleop_ws
colcon build
source install/setup.bash
```
🚀 Running the Simulation

Step 1: Launch the robot in Gazebo

This command launches Gazebo with the UR5 robot and its controllers:
```
ros2 launch teleop_arm sim_teleop.launch.py
```
This will automatically execute:
```
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5 use_fake_hardware:=true launch_rviz:=false
```
Step 2: Run the keyboard teleoperation node (in a separate terminal)
```
source ~/teleop_ws/install/setup.bash
ros2 run teleop_arm keyboard_teleop
```
⚠️ This node must be run in a regular terminal (not from launch file) due to raw keyboard input handling.

⌨️ Keyboard Controls

Joint : Increase/Decrease

Shoulder Pan : Q/A

Shoulder Lift : W/S

Elbow : E/D

Wrist 1 : R/F

Wrist 2 : T/G

Wrist 3 : Y/H

Each keypress sends a new joint trajectory with a small delta (0.1 rad).

🧪 Testing and Debugging

Verify that the robot spawns in Gazebo and is controllable.

Confirm /joint_trajectory_controller/joint_trajectory is being published to using:
```
ros2 topic echo /joint_trajectory_controller/joint_trajectory
```
Use rqt_graph to visualize the ROS computation graph.

👤 Author

Yash Sharma
