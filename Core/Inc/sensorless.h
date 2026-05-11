/* === 文件名: sensorless.h === */
#ifndef __SENSORLESS_H
#define __SENSORLESS_H

#include <stdint.h>
#include "foc_math.h"
#include "motor_config.h"

typedef struct {
    float Ialpha_est; // 虚拟电机的 alpha 轴电流
    float Ibeta_est;  // 虚拟电机的 beta 轴电流
    float Zalpha;     // 估算 alpha 反电动势
    float Zbeta;      // 估算 beta 反电动势
    float Ealpha_est; // 滤波后的估算 alpha 反电动势
    float Ebeta_est;  // 滤波后的估算 beta 反电动势
  } SMO_State_t;

/**
 * @brief 一阶低通滤波器状态结构体
 */
typedef struct {
    float Out_Last; // 上一次的滤波输出 (记忆状态)
    float Alpha;    // 滤波系数 (Ts * Wc)
} LPF_State_t;

/* --- 外部可获取的估算数据 --- */
extern float Angle_Est;       // 估算出的电角度 (rad)
extern float Speed_Est_RPM;   // 估算出的转速 (RPM)

/* --- 无感模块通用接口 --- */
void Sensorless_Init(void);

/**
 * @brief 初始化低通滤波器
 * @param lpf 滤波器结构体指针
 * @param Ts  系统采样周期 (例如 40kHz 则为 0.000025f)
 * @param Wc  截止频率 (rad/s)
 */
void LPF_Init(LPF_State_t *lpf, float Ts, float Wc);

// 核心解算器：输入采样电流，内部更新估算角度与速度
void Sensorless_Resolver(float Iad_fb, float Ibq_fb, float Ualpha, float Ubeta);

// 获取需要叠加到电压轴上的注入电压 (若无需注入则返回0)
float Sensorless_Get_Inject_Vd(void);
float Sensorless_Get_Inject_Vq(void);

/**
 * @brief 符号函数 (Sign)
 * @param error 电流误差
 * @return 1.0f, -1.0f, 或 0.0f
 */
float SMO_Sign(float error);

/**
 * @brief 标准 Sigmoid 函数 (映射至 -1 到 1)
 * @param error 电流误差
 * @param a     陡峭系数 (a 越大，曲线越接近阶跃方波；a 越小，曲线越平滑)
 * @return 平滑后的开关信号 (-1.0f ~ 1.0f)
 */
float SMO_Sigmoid(float error, float a);

/**
 * @brief 快速 Sigmoid 函数 (连续分数型)
 * @param error 电流误差
 * @param a     平滑系数 (a 越小，越接近阶跃方波；a 越大，越平缓。建议值 0.01~0.1)
 * @return 平滑后的开关信号 (-1.0f ~ 1.0f 之间的小数)
 */
float SMO_Fast_Sigmoid(float error, float a);

/**
 * @brief 初始化低通滤波器
 * @param lpf 滤波器结构体指针
 * @param Ts  系统采样周期 (例如 40kHz 则为 0.000025f)
 * @param Wc  截止频率 (rad/s)
 */
void LPF_Init(LPF_State_t *lpf, float Ts, float Wc);

/**
 * @brief 执行低通滤波计算
 * @param lpf   滤波器结构体指针
 * @param input 当前的新采样值
 * @return 滤波后的平滑输出
 */
float LPF_Calc(LPF_State_t *lpf, float input);

#endif /* __SENSORLESS_H */
