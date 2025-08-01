#!/bin/bash

# Get the directory of the current script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Optional: Make Unity executable (only needed once, so can be commented out later)
chmod +x "$SCRIPT_DIR/simulation/scripts/run_unity.sh"
chmod +x "$SCRIPT_DIR/simulation/unity_sim/Build_Ubuntu/AD_Sim.x86_64"

# Launch the simulation (this already starts AD_Sim.x86_64)
gnome-terminal -- bash -c "roslaunch simulation simulation_demo.launch; exec bash"
sleep 10
# Launch the path planner
gnome-terminal -- bash -c "roslaunch path_planner path_planner.launch; exec bash"
sleep 5

# Run the trajectory generator node
gnome-terminal -- bash -c "rosrun trajectory_generator trajectory_generator_node; exec bash"
sleep 3

# Run the dummy controller node
gnome-terminal -- bash -c "rosrun dummy_controller dummy_controller_node; exec bash"
