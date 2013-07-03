# industrial_msgs/RobotStatus
Header    header

industrial_msgs/RobotMode mode
industrial_msgs/TriState  e_stopped
industrial_msgs/TriState  drives_powered
industrial_msgs/TriState  motion_possible
industrial_msgs/TriState  in_motion
industrial_msgs/TriState  in_error

int32 error_code

# extended hek robot status
industrial_msgs/TriState  is_calibrated
ServoHealth[]             servo_health
