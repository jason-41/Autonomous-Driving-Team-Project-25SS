# Introduction to ROS – Course Project Autonomous Driving

## Overview 
This repository is part of the "Introduction to ROS" course project **Autonomous Driving**. It consists of a Unity-based driving simulation package and a dummy controller to help you understand how to control the vehicle.

## System Requirements
-**Operating System**: Ubuntu 20.04 (either natively installed or via Windows Subsystem for Linux - WSL)

-**ROS Version**: ROS Noetic

-**Required Tools**:
  - Standard ROS tools: `catkin`, `roscore`, etc.
  - Git
  - *Git LFS* (Large File Storage – required for downloading Unity binaries and large assets)
    To install Git LFS, run:
    
    ```bash
    sudo apt install git-lfs
    git lfs install
    ```

## Repository Structure
This repository contains two ROS packages:

- `simulation/`
  - Contains a Unity-based simulation environment that mimics a vehicle's sensing and movement in a virtual city.
  - Includes launch files, configuration scripts, and integration logic.

- `dummy_controller/`
  - A basic ROS node used to control the vehicle within the simulation.
  - Designed to demonstrate publishing control commands.

## Getting Started

### Clone the Repository to `/src` Folder in the Workspace

This's how your workspace should look like...

```text
your_workspace/
└── src/
    ├── simulation/
    ├── dummy_controller/
    ├── CMakeList.txt
    ├── setup_script.sh
    └── README.md
```

### Run the Setup Script

The script ensures that required simulation files and scripts are executable:

```bash
bash src/setup_script.sh
```
Note that you are free to modify this script file to helps others setup your project.

### Build the Workspace

```bash
catkin build
```
Upon successful build, `devel`, `build` and `logs` will be automatically generated in your workspace.

### Running the Simulation

To launch the simulation node, run the command **after having ```roscore``` running**:

```bash
source devel/setup.bash
roslaunch simulation simulation.launch
```
You should see a window popping up and you should be able to control the vehicle with arrow keys or WASD.

Note: this is only a minimal launch file to ensure that everything in Unity simulation is accessible by ROS. You could certainly build and run on your own launch file! 

### Running the Dummy Controller Node

Having **the roscore and the simulation node running**, run the node in another console:

```bash
source devel/setup.bash
rosrun dummy_controller dummy_controller_node
```
The vehicle should be able to accelerate and steer by itself.

## Customize the Simulation
The simulation node provides a variety of configurable parameters, including the sensor setup, the socket port etc. For details please refer to [the README.md in simulation pacakge](simulation/README.md).

## The Project Assignment
The detailed task description and submission requirements are available in the project PDF uploaded to Moodle. Please review that document carefully.

## Support
If you encounter issues or have questions, please reach me via [my e-mail](mailto:jiaming.zhang@tum.de). I would love to help as much as I could :).