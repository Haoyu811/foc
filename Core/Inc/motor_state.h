/* === 文件名: motor_state.h === */
#ifndef __MOTOR_STATE_H
#define __MOTOR_STATE_H

#include <stdint.h>

/* 电机系统主状态枚举 */
typedef enum {
    SYS_INIT = 0,    // 初始化阶段
    SYS_IDLE,        // 待机阶段 (PWM关闭，允许自由转动)
    SYS_RUNNING,     // 运行阶段 (闭环发波中)
    SYS_FAULT        // 故障阶段 (触发过流/过压，封锁脉冲)
} System_State_e;

/* 电机系统状态机结构体 */
typedef struct {
    System_State_e State;    // 系统当前主状态
    
    uint8_t Run_Flag;        // 运行使能指令: 0-停止发波, 1-启动发波
    uint8_t Fault_Flag;      // 硬件故障标志: 0-正常, 1-触发故障
    uint8_t Calibrate_Flag;  // 偏置校准标志: 0-校准完毕, 1-正在校准
    
    uint8_t Position_Flag;   // 编码器对齐预定位标志
    uint8_t Z_Index_Flag;    // 编码器Z相零点捕获标志
} Motor_State_t;

/* 对外暴露全局状态机实例 */
extern volatile Motor_State_t MotorSys;

#endif /* __MOTOR_STATE_H */
