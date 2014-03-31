### Hekateros hekateros_control/HekJointStateExtended message

# Base sensor_msgs/JointState
Header    header              # standard header

string[]       name           # joint name
float64[]      position       # joint position (radians)
float64[]      velocity       # joint velocity (percent of maximum)
float64[]      effort         # joint torque load (unitless scale)

# Hekateros extensions
uint8[]        master_servo_id  # master servo id
uint8[]        slave_servo_id   # slave servo id
int32[]        odometer_pos     # joint odometer position (encoder ticks)
int32[]        encoder_pos      # servo encoder position
int32[]        raw_speed        # servo raw speed (unitless scale)
HekOpState[]   op_state         # servo operation state:
                                #   UNCALIBRATED, CALIBRATING, CALIBRATED
industrial_msgs/TriState[] opt_switch1  # optical limit switch 1 state
industrial_msgs/TriState[] opt_switch2  # optical limit switch 2 state
