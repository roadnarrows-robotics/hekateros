### Hekateros hekateros_control/HekRobotStatusExtended message

# Base industrial_msgs/RobotStatus
Header    header    # standard header

industrial_msgs/RobotMode mode              # robot operation mode: AUTO MANUAL
industrial_msgs/TriState  e_stopped         # emergency stopped: TRUE FALSE
industrial_msgs/TriState  drives_powered    # servos powered: TRUE FALSE
industrial_msgs/TriState  motion_possible   # motion possbile: TRUE FALSE
industrial_msgs/TriState  in_motion         # in motion: TRUE FALSE
industrial_msgs/TriState  in_error          # in error condition: TRUE FALSE
int32 error_code                            # hekateros error code

# Hekateros extensions
industrial_msgs/TriState  is_calibrated # robot calibration status:
                                        #  UNCALIBRATED, CALIBRATING, CALIBRATED
ServoHealth[]             servo_health  # servo health
