# Project Description

## Overview  
This repository implements a lightweight ROS1-based autonomous driving stack, including global path planning (A*), short-term goal sequencing, trajectory generation, and a PID-based controller. A single setup script launches all necessary components and the vehicle simulation automatically.
The ROS-Unity bridge (TCP sensor streaming + UDP command transmission) with configurable simulation parameters and multi-node launch workflow can be used for reproducible experiments.  
### My contribution：
Implemented the short_term_planner, trajectory generator and the simple PID controller.

## Warning
  **`In case the performance is not as expected`**  
  Due to different CPU, memory and systems, we have observed a considerable difference in the performance on different computers.
  It is worth noticing that the car performs much better on a native Linux installation than on WSL2 and virtual machines, even on the same hardware.
  In the repository there is a .mp4 file: best_practice.mp4 - the video recording of our best practice.
  This may be seen as a reference of our best work.

## Environment & Requirements
- **Operating System:** Ubuntu 20.04 LTS  
- **ROS Distribution:** ROS Noetic  
- **Build Tool:** catkin (using `catkin build`)  
- **Dependencies:**  
  - Core ROS packages: `roscpp`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `tf`  
  - `octomap_server` (for optional 3D mapping upstream)  

No external third-party numerical libraries are required. Although OpenCV is being used implicitly, it comes along with ROS1 and does not require installation by hand.

## Installation & Build

1. **Install octomap server**  
   ```bash
   sudo apt update
   sudo apt install ros-noetic-octomap-server
   ```
   (optional)
   for visualization in RViz:
    ```bash
    sudo apt install ros-noetic-octomap-rviz-plugins
    ```

2. **Build the workspace**  
   ```bash
   catkin build
   ```
   
3. **Source the setup file**  
   ```bash
   source devel/setup.bash
   ```

## Run

From the repository root, execute the setup script:

```bash
./src/setup_script.sh
```

- Multiple terminal windows will open automatically.
- The vehicle simulation and ROS nodes (path planner, trajectory generator, controller, etc.) will start.
- The system begins operating based on incoming pose, twist, map, and traffic-state data.       |

## Troubleshooting

- **`./src/setup_script.sh: No such file or directory`**  
  Ensure you are in the repository root and the script exists. Make it executable:
  ```bash
  chmod +x ./src/setup_script.sh
  ```

- **`gnome-terminal: command not found`**  
  The script spawns new terminals via GNOME Terminal. Install it:
  ```bash
  sudo apt install gnome-terminal
  ```
  Or edit `setup_script.sh` to use `xterm`, `konsole`, or run commands directly in the background (`&`).
