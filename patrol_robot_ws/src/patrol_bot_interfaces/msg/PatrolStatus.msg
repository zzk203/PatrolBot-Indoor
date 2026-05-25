# 巡逻系统状态枚举
uint8 IDLE=0
uint8 PATROLLING=1
uint8 PAUSED=2
uint8 CHARGING=3
uint8 STOPPED=4

# 状态字段
uint8 state
int32 current_route_index
int32 current_waypoint_index
float32 battery_level
