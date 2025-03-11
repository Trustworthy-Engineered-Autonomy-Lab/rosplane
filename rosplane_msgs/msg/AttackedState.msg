# Vehicle state 'x_hat' output from the estimator or from simulator 

std_msgs/Header header
rosplane_msgs/State original_state

bool is_attacked  # Is the value attack (boolean)
float32[3] delta_position	# Value added to north, east, down (m) by the attack

