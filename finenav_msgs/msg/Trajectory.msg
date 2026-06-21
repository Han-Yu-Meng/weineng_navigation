# Constants, allow bitwise OR
uint32 TRAJ_EMPTY = 0
uint32 TRAJ_POSE = 1
uint32 TRAJ_VEL  = 2
uint32 TRAJ_ACCEL= 4
uint32 TRAJ_TIME = 8

std_msgs/Header header

# Masked fields
uint32 valid_fields

# Trajectory points
geometry_msgs/Pose[] poses
geometry_msgs/Twist[] twists
geometry_msgs/Accel[] accels
builtin_interfaces/Duration[] time_from_start