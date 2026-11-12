std_msgs/Header header
float64 lateral_error
float64 heading_error
# Configured longitudinal target speed, m/s. This is not the gated command.
float64 target_speed
# Latest odometry twist.linear.x, m/s; signed and measured even at the goal.
float64 actual_speed
int32 nearest_path_index
string controller_name
