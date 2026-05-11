#ifndef __FOC_H
#define __FOC_H

#include <stdint.h>
#include "foc_math.h"
#include "motor_config.h"

/* ================= 核心结构体定义 ================= */

/**
 * @brief PI 控制器结构体
 */
typedef struct {
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Error_Now;        // 当前误差
    float Error_Last;       // 上次误差 (备用于增量式, 当前为位置式)
    float Error_Integral;   // 误差积分
    float Out;              // 控制器输出
    float Out_Limit;        // 输出限幅绝对值
} PI_Controller_t;

/* ================= 全局控制器实例声明 ================= */
extern PI_Controller_t PI_Id;
extern PI_Controller_t PI_Iq;
extern PI_Controller_t PI_Speed;
extern PI_Controller_t PI_Phase; // 用于无感观测器 PLL

/* ================= 算法接口函数 ================= */

/**
 * @brief  初始化所有 PI 控制器参数 (从 motor_config.h 加载)
 */
void FOC_PI_Init(void);

/**
 * @brief  清除 PI 控制器的历史积分状态 (电机停机时必须调用，防止积分饱和炸机)
 */
void FOC_PI_Clear(void);

/**
 * @brief   普通PI控制器
 */
float PI_Calc_Standard(PI_Controller_t *pi, float target, float feedback);
/**
 * @brief   抗饱和积分PI控制器
 */
float PI_Calc_AntiWindup(PI_Controller_t *pi, float target, float feedback);
/**
 * @brief  通用的 PI 计算核心函数
 * @param  pi        PI结构体指针
 * @param  target    目标期望值
 * @param  feedback  实际反馈值
 * @return float     控制输出
 */
float PI_Calc(PI_Controller_t *pi, float target, float feedback);

/**
 * @brief  空间矢量脉宽调制 (SVPWM) 核心算法
 * @param  Ualpha   两相静止坐标系 Alpha 轴电压
 * @param  Ubeta    两相静止坐标系 Beta 轴电压
 * @param  Udc      当前母线电压
 * @param  pwm_a    输出 A 相占空比计数值 (传入指针)
 * @param  pwm_b    输出 B 相占空比计数值 (传入指针)
 * @param  pwm_c    输出 C 相占空比计数值 (传入指针)
 */
void SVPWM_Calc(float Ualpha, float Ubeta, float Udc, uint16_t *pwm_a, uint16_t *pwm_b, uint16_t *pwm_c);

/**
 * @brief  FOC 电流环完整计算流水线
 * @param  I_abc    输入: 三相电流反馈
 * @param  Angle    输入: 当前电角度 (弧度)
 * @param  Iq_Ref   输入: Q轴目标电流
 * @param  Id_Ref   输入: D轴目标电流
 * @param  Udc      输入: 当前母线电压
 * @param  pwm_out  输出: 计算得到的三个定时器 CCR 值数组 [pwm_a, pwm_b, pwm_c]
 */
void FOC_Current_Loop(const Vector_3Phase_t *I_abc, float Angle, float Iq_Ref, float Id_Ref, float Udc, float speed_act, uint16_t *pwm_out);

#endif /* __FOC_H */
