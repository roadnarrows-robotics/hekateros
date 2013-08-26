Header    header

string[]       name
float64[]      position
float64[]      velocity
float64[]      effort
uint8[]        master_servo_id
uint8[]        slave_servo_id 
int32[]        odometer_pos 
int32[]        encoder_pos 
int32[]        raw_speed 
HekOpState[]   op_state

industrial_msgs/TriState[] opt_switch1
industrial_msgs/TriState[] opt_switch2
