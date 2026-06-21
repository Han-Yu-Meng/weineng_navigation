# Navigation state constants
uint8 IDLE=0
uint8 NAVIGATING=1
uint8 GOAL_REACHED=2
uint8 STUCK=3
uint8 ABORTED=4
uint8 LOCALIZATION_ERROR=5

std_msgs/Header header

# Current navigation FSM state (see constants above)
uint8 state

# Odometric distance accumulated during the current navigation episode (m)
float64 distance_traveled

# Current body-frame linear / angular speed (m/s, rad/s)
float64 current_linear_vel
float64 current_angular_vel

# Active navigation goal
geometry_msgs/Pose goal_pose

