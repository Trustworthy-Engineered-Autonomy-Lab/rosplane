#Author Kushagra
#if you want to see the whats going on in tmux just use 
# "tmux attach"  
#ctrl+c will close everything


#!/bin/bash

# Define colors for output
GREEN='\033[0;32m'
NC='\033[0m' # No Color

cleanup() {
    echo -e "${GREEN}Cleaning up processes...${NC}"
    
    # Kill RViz and waypoint publisher first
    kill $(jobs -p) 2>/dev/null

    # Kill any existing tmux sessions
    tmux kill-session -t rosplane_sim_session 2>/dev/null

    # Try to clear markers only if ROS is still running
    if pgrep -f "ros2" > /dev/null; then
        ros2 topic pub --once /current_path nav_msgs/msg/Path "{}" 2>/dev/null &
        ros2 topic pub --once /waypoint_path nav_msgs/msg/Path "{}" 2>/dev/null &
        sleep 1
    fi

    # Kill any remaining ros2 processes
    pkill -f "ros2"
    
    echo -e "${GREEN}Cleanup complete${NC}"
    exit
}
# Source ROS2 and workspace
echo -e "${GREEN}Sourcing ROS2 and workspace...${NC}"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Start orbit script
echo -e "${GREEN}Starting ROSplane orbit simulation...${NC}"
./rosplane_orbit.sh &

# Wait for simulation to initialize
sleep 5

# Start waypoint publisher
echo -e "${GREEN}Starting waypoint publisher...${NC}"
ros2 run rosplane_gcs rviz_waypoint_publisher &

# Wait for publisher to initialize
sleep 2

# Launch RViz2
echo -e "${GREEN}Launching RViz2...${NC}"
rviz2 -d rviz_scripts/test101.rviz

# When RViz2 is closed, cleanup background processes
echo -e "${GREEN}Cleaning up...${NC}"
kill $(jobs -p)

# Kill any remaining tmux sessions
tmux kill-session -t rosplane_sim_session