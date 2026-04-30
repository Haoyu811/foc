#ifndef __CONTROL_STRATEGY_H
#define __CONTROL_STRATEGY_H

#include "motor_config.h"
#include <stdint.h>

// 将速度环输出的定子电流 Is，根据策略分配为 Id 和 Iq 给定值
void Strategy_Get_Id_Iq_Ref(float Is_target, float *Id_Ref, float *Iq_Ref);

/**
 * @brief  开环 V/F 运行逻辑
 * @param  Udc          当前母线电压
 * @param  target_rpm   输入给定转速
 * @param  pwm_out      输出的占空比计数值指针
 */
// 
void Strategy_VF_Process(float target_rpm, float Udc, uint16_t *pwm_out);

/**
 * @brief  转子多段式强拖预定位算法 (须在固定频率的中断中调用)
 * @param  Udc    当前母线电压
 * @param  pwm_a  输出的A相占空比计数值指针
 * @param  pwm_b  输出的B相占空比计数值指针
 * @param  pwm_c  输出的C相占空比计数值指针
 */
void Strategy_Position_Align(float Udc, uint16_t *pwm_a, uint16_t *pwm_b, uint16_t *pwm_c);

#endif
