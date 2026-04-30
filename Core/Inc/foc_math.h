#ifndef __FOC_MATH_H
#define __FOC_MATH_H

#include <stdint.h>
#include "arm_math.h" // 使用 ARM 官方 DSP 库

/* ================= 常用数学常数宏定义 ================= */
#define MATH_PI             3.14159265f
#define MATH_2PI            6.28318531f
#define VALUE_SQRT3         1.73205081f
#define VALUE_SQRT3_2       0.86602540f    // √3 / 2
#define VALUE_1_SQRT3       0.57735027f    // 1 / √3

/* ================= FOC 核心变量结构体 ================= */
// 三相静止坐标系变量 (如 Ia, Ib, Ic 或 Ua, Ub, Uc)
typedef struct {
    float A;
    float B;
    float C;
} Vector_3Phase_t;

// 两相静止坐标系变量 (如 Ialpha, Ibeta 或 Ualpha, Ubeta)
typedef struct {
    float Alpha;
    float Beta;
} Vector_AlBe_t;

// 两相旋转坐标系变量 (如 Id, Iq 或 Ud, Uq)
typedef struct {
    float D;
    float Q;
} Vector_DQ_t;

// 正余弦三角函数缓存
typedef struct {
    float Sin;
    float Cos;
} Trig_Vals_t;

/* ================= 算法接口函数声明 ================= */

/**
 * @brief  计算电角度的正余弦值
 * @param  angle_rad 电角度 (弧度制, 0 ~ 2π)
 * @param  trig      输出的正余弦结构体指针
 */
void FOC_Trig_Calc(float angle_rad, Trig_Vals_t *trig);

/**
 * @brief  Clarke 变换 (三相静止 -> 两相静止) - 恒幅值变换， 指针输入+const 避免函数内误修改
 */
void Clarke_Transform(const Vector_3Phase_t *in, Vector_AlBe_t *out);

/**
 * @brief  逆 Clarke 变换 (两相静止 -> 三相静止)
 */
void Inv_Clarke_Transform(const Vector_AlBe_t *in, Vector_3Phase_t *out);

/**
 * @brief  Park 变换 (两相静止 -> 两相旋转)
 */
void Park_Transform(const Vector_AlBe_t *in, const Trig_Vals_t *trig, Vector_DQ_t *out);

/**
 * @brief  逆 Park 变换 (两相旋转 -> 两相静止)
 */
void Inv_Park_Transform(const Vector_DQ_t *in, const Trig_Vals_t *trig, Vector_AlBe_t *out);

#endif /* __FOC_MATH_H */