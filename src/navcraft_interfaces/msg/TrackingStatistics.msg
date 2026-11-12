# Tracking-error samples accumulated since the evaluator started.
# These are sample-weighted means, not goal results, distance or elapsed time.
std_msgs/Header header
string controller_name
uint64 samples
float64 mean_abs_lateral_error_m
float64 mean_abs_heading_error_rad
# -1 until the first sample arrives.
int32 last_nearest_path_index
