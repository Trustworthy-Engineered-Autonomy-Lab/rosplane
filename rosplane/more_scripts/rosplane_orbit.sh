#!/bin/bash

filepath='/home/kushagra/ros2_ws'
# Create a new tmux session
tmux new-session -d -s rosplane_sim_session

# Uncomment this line if you want to use your own tmux config
# tmux source-file ~/.tmux.conf

# Split the tmux window into 4 panes
tmux split-window -t rosplane_sim_session:0.0 -h
tmux split-window -t rosplane_sim_session:0.0 -v
tmux split-window -t rosplane_sim_session:0.2 -v
tmux split-window -t rosplane_sim_session:0.3 -v

# Arrange panes in each corner
tmux select-pane -t rosplane_sim_session:0.0
tmux select-pane -t rosplane_sim_session:0.1
tmux select-pane -t rosplane_sim_session:0.2
tmux select-pane -t rosplane_sim_session:0.3
tmux select-pane -t rosplane_sim_session:0.4

# Window placement reference:
# rosplane_sim_session:0.0 Top Left
# rosplane_sim_session:0.1 Bottom Left
# rosplane_sim_session:0.2 Top Right
# rosplane_sim_session:0.3 Bottom Right high
# rosplane_sim_session:0.4 Bottom Right low

# Send all of the panes to the working directory.
tmux send-keys -t rosplane_sim:0.0 "cd $filepath" C-m
tmux send-keys -t rosplane_sim:0.1 "cd $filepath" C-m
tmux send-keys -t rosplane_sim:0.2 "cd $filepath" C-m
tmux send-keys -t rosplane_sim:0.3 "cd $filepath" C-m 
tmux send-keys -t rosplane_sim:0.4 "cd $filepath" C-m 

# Run commands
tmux send-keys -t rosplane_sim_session:0.0 "ros2 launch rosflight_sim fixedwing.launch.py aricraft:=anaconda" C-m

sleep 2

tmux send-keys -t rosplane_sim_session:0.1 "ros2 run rosflight_io rosflight_io --ros-args -p udp:=true" C-m

tmux send-keys -t rosplane_sim_session:0.2 "ros2 launch rosplane_sim sim.launch.py aircraft:=anaconda" C-m

sleep 2

#tmux send-keys -t rosplane_sim_session:0.3 "ros2 service call /load_mission_from_file rosflight_msgs/srv/ParamFile '{filename: /home/kushagra/rosflight_ws_test/src/rosplane/rosplane/params/anaconda_autopilot_params.yaml}'" C-m

tmux send-keys -t rosplane_sim_session:0.3 "ros2 launch rosflight_sim fixedwing_init_firmware.launch.py" C-m

tmux send-keys -t rosplane_sim_session:0.3 "ros2 service call /calibrate_imu std_srvs/srv/Trigger" C-m

sleep 2

tmux send-keys -t rosplane_sim_session:0.3 "ros2 run rosflight_sim rc.py --ros-args --remap RC:=/fixedwing/RC" C-m 

sleep 5
tmux send-keys -t rosplane_sim_session:0.4 "ros2 service call /toggle_arm std_srvs/srv/Trigger" C-m
#sleep 1
tmux send-keys -t rosplane_sim_session:0.4 "ros2 service call /toggle_override std_srvs/srv/Trigger" C-m

# Add commands for 
# Attach to the tmux session
tmux attach-session -t rosplane_sim_session

# ----------------------------------------------------------------------------
