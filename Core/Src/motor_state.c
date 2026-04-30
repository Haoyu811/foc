/* === 文件名: motor_state.c === */
#include "motor_state.h"

/* * 真正的全局变量定义与上电初始值分配 
 * 上电默认处于初始化状态，且默认需要校准
 */
volatile Motor_State_t MotorSys = {
    .State          = SYS_INIT,
    .Run_Flag       = 0,
    .Fault_Flag     = 0,
    .Calibrate_Flag = 1,
    .Position_Flag  = 0,
    .Z_Index_Flag   = 0
};

float target_speed_rpm = 1000.0f; 