### Hekateros hekateros_control/ServoHealth message

int8    servo_id    # servo id
float32 temp        # servo temperature (C)
float32 voltage     # servo input voltage (Volts)
uint8   alarm       # servo or'ed alarm bits (see below)

### alarm bits 
uint8   ALARM_NONE         = 0      # no alarms
uint8   ALARM_VOLTAGE      = 1      # over/under voltage
uint8   ALARM_ANGLE        = 2      # exceed servo rotation angle limits
uint8   ALARM_TEMP         = 4      # over temperature
uint8   ALARM_CMD_RANGE    = 8      # bad command
uint8   ALARM_CHECKSUM     = 16     # bad packet checksum
uint8   ALARM_LOAD         = 32     # overloaded
uint8   ALARM_INSTRUCTION  = 64     # instruction error
