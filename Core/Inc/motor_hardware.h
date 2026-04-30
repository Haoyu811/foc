/* === 文件名: motor_hardware.h === */
#ifndef __MOTOR_HARDWARE_H
#define __MOTOR_HARDWARE_H

#include <stdint.h>
#include "motor_config.h" // 包含配置参数

/* ================= 硬件控制接口 ================= */
/**
 * @brief 电机底层外设总初始化 (包含继电器吸合、ADC校准、编码器启动、DAC启动等)
 */
void Motor_Hardware_Init(void);

/**
 * @brief 开启 TIM1 高级定时器 PWM 输出
 */
void Motor_PWM_Start(void);

/**
 * @brief 关闭 TIM1 高级定时器 PWM 输出 (进入高阻态，保护电机)
 */
void Motor_PWM_Stop(void);

/* ================= 数据获取接口 ================= */
/**
 * @brief 获取标幺化/实际物理值的相电流
 * @param pIu, pIv, pIw 提取出的三相电流 (A)
 * @param pIbus         提取出的母线电流 (A)
 */
void Motor_Get_Phase_Currents(float *pIu, float *pIv, float *pIw, float *pIbus);

/**
 * @brief 获取母线电压
 * @param pUdc 提取出的母线电压 (V)
 */
void Motor_Get_Bus_Voltage(float *pUdc);

/**
 * @brief 更新并获取编码器的电角度与实际机械转速
 * @param pElec_Angle_Rad 输出: 当前电角度 (弧度, 0~2π)
 * @param pSpeed_RPM      输出: 当前机械转速 (RPM)
 * @note  该函数内部包含差分运算，必须在固定频率 (PWM_FREQ) 的中断中被严格周期性调用
 */
void Motor_Update_Encoder_Data(float *pElec_Angle_Rad, float *pSpeed_RPM);

/**
 * @brief 更新 PWM 占空比到定时器寄存器
 * @param pwm_a, pwm_b, pwm_c 三相比较寄存器值 (0 ~ TIM1_PERIOD)
 */
void Motor_Set_PWM_Duty(uint16_t pwm_a, uint16_t pwm_b, uint16_t pwm_c);

/* ================= 调试输出接口 ================= */
/**
 * @brief DAC 实时观测输出
 * @param ch1_val 通道1数值 (0~4095)
 * @param ch2_val 通道2数值 (0~4095)
 */
void Motor_Debug_DAC_Output(uint16_t ch1_val, uint16_t ch2_val);

#endif /* __MOTOR_HARDWARE_H */
