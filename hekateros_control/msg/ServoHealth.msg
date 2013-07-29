int8    servo_id
float32 temp
float32 voltage
uint8   alarm

### alarm bits 
uint8   ALARM_NONE         = 0
uint8   ALARM_VOLTAGE      = 1
uint8   ALARM_ANGLE        = 2
uint8   ALARM_TEMP         = 4
uint8   ALARM_CMD_RANGE    = 8
uint8   ALARM_CHECKSUM     = 16
uint8   ALARM_LOAD         = 32
uint8   ALARM_INSTRUCTION  = 64
