/* === 文件名: sensorless.h === */
#ifndef __SENSORLESS_H
#define __SENSORLESS_H

#include <stdint.h>
#include "foc_math.h"
#include "motor_config.h"

/* --- 外部可获取的估算数据 --- */
extern float Angle_Est;       // 估算出的电角度 (rad)
extern float Speed_Est_RPM;   // 估算出的转速 (RPM)

/* --- 无感模块通用接口 --- */
void Sensorless_Init(void);

// 核心解算器：输入采样电流，内部更新估算角度与速度
void Sensorless_Resolver(float Id_fb, float Iq_fb, float Ualpha, float Ubeta);

// 获取需要叠加到电压轴上的注入电压 (若无需注入则返回0)
float Sensorless_Get_Inject_Vd(void);
float Sensorless_Get_Inject_Vq(void);

#endif /* __SENSORLESS_H */
