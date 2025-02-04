# Kuka_ROS_Manipulation
 
This repository contains ROS packages for manipulating a KUKA KR90 robotic arm using ROS 1 on Ubuntu 20.04. The packages provide tools for establishing communication between ROS and the robotic arm and executing trajectory planning and manipulation tasks.  

## Prerequisites  

- **Ubuntu 20.04**  
- **ROS 1 (Noetic recommended)**  

## Connecting ROS to the KUKA Robotic Arm  

To connect ROS to the robotic arm and launch the necessary nodes for **trajectory generation**, run the following command:  

```sh
roslaunch kuka_kr90_moveit_config moveit_planning_execution_rsi_laam.launch sim:=false robot_ip:=192.168.1.20
```
**Note:** Update `robot_ip` to match the actual IP address of the KUKA robotic arm.  

## Manipulating the KUKA Robotic Arm  

To launch the **robot arm manipulation package**, use:  

```sh
roslaunch kuka_advanced_manipulation_pkg kuka_advanced_manipulation.launch
```
[Watch the full video](kuka_demo.mp4)
