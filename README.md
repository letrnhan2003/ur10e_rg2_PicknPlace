# UR10e Robot Arm with RG2 Gripper Pick and Place Project

## Overview
This project is a part of a module taught at Vietnamese-German University (VGU), instructed by Dr. Dong Quang Huan. The goal of this project is to adapt the pick and place tutorial from the [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main/tutorials/pick_and_place) for the UR10e robot arm integrated with the OnRobot RG2 gripper. The objective is to automate the process of picking an object from inside a 3D printer and placing it at a desired location in a simulated environment using Unity. The project utilizes the Stochastic Trajectory Optimization for Motion Planning (STOMP) planner for motion planning.

## Table of Contents
- [Overview](#overview)
- [Installation](#installation)
  - [Prerequisites](#prerequisites)
  - [Clone the Repository](#clone-the-repository)
  - [Set Up ROS Workspace](#set-up-ros-workspace)
  - [Configure Unity Project](#configure-unity-project)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)

## Installation
To set up this project, follow these steps:

### Prerequisites
Ensure you have the following software installed:
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [Unity 2020.3](https://unity3d.com/get-unity/download/archive)
- [MoveIt](https://moveit.ros.org/install/)
- [URDF Importer](https://github.com/Unity-Technologies/URDF-Importer) in Unity

### Clone the Repository
```bash
git clone https://github.com/Trung2204/ur10e_rg2_PickAndPlace.git
cd ur10e_rg2_PickAndPlace
```

### Set Up ROS Workspace
```bash
cd ROS
catkin_make
source devel/setup.bash
```
### Configure Unity Project
1. Open Unity and load the `UnityProject` directory as your project.
2. Import the UR10e and RG2 gripper URDF files using the URDF Importer.

## Usage
Follow these steps to run the simulation:


## Project Structure
```
ur10e_rg2_PickAndPlace/
├── ROS/
│   ├── build/
│   ├── devel/
│   ├── src/
│   │   ├── ur10e_rg2_moveit/
│   │   │   ├── CMakeLists.txt
│   │   │   ├── package.xml
│   │   │   ├── config/
│   │   │   ├── launch/
│   │   ├── ur10e_rg2_urdf/
│   │   │   ├── CMakeLists.txt
│   │   │   ├── package.xml
│   │   │   ├── urdf/
│   │   │   ├── meshes/
├── UnityProject/
│   ├── Assets/
│   ├── Library/
│   ├── Logs/
│   ├── Packages/
│   ├── ProjectSettings/
│   ├── UserSettings/
├── README.md
```

## Contributing

## License