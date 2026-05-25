# 报警严重性枚举
uint8 WARNING=0
uint8 CRITICAL=1

# 报警字段
uint8 severity
string alarm_type
float32 x
float32 y
builtin_interfaces/Time timestamp
