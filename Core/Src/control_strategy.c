/* === 文件名: control_strategy.c === */
#include "control_strategy.h"
#include "motor_state.h"     
#include "motor_hardware.h" 
#include "foc_math.h"
#include "foc.h" // 借用 SVPWM_Calc
#include "arm_math.h"

// 内部静态变量，用于 V/F 控制的角度积分
static float vf_angle = 0.0f;

extern uint16_t Acount;

// 实例化一个针对 V/F 的静态斜坡发生器
// Step = 变化率(RPM/s) * 离散周期 TS(s) 
static Ramp_Generator_t vf_ramp = {
    .Target = 0.0f, 
    .Current = 0.0f, 
    .Step = ACCEL_RATE * TS, 
    .Enable = 1
};

/* =======================================================
 * 通用斜坡发生器计算函数
 * ======================================================= */
float Calc_Ramp(Ramp_Generator_t *ramp, float target_val) 
{
    ramp->Target = target_val;
    
    // 如果斜坡功能开启
    if (ramp->Enable == 1) 
    {
        if (ramp->Current < ramp->Target) {
            ramp->Current += ramp->Step;
            // 防止超调
            if (ramp->Current > ramp->Target) ramp->Current = ramp->Target; 
        } 
        else if (ramp->Current > ramp->Target) {
            ramp->Current -= ramp->Step;
            if (ramp->Current < ramp->Target) ramp->Current = ramp->Target;
        }
    } 
    // 如果斜坡功能关闭 (阶跃直通)
    else 
    {
        ramp->Current = ramp->Target; 
    }
    
    return ramp->Current;
}

void Strategy_Clear(void) 
{
    // 电机停机时必须调用此函数，将速度斜坡“踩死”归零
    vf_ramp.Target = 0.0f;
    vf_ramp.Current = 0.0f;
    vf_angle = 0.0f; // 角度也归零，保证下次启动从 A 相平滑起步
}

void Strategy_Get_Id_Iq_Ref(float Is_target, float *Id_Ref, float *Iq_Ref) 
{
#if (CURRENT_CTRL_STRATEGY == CTRL_STRATEGY_FOC_ID0)
    /* ----------------------------------------------------
     * 策略 A: Id = 0 控制 (适合 SPM)
     * ----------------------------------------------------*/
    *Id_Ref = 0.0f;
    *Iq_Ref = Is_target; // 直接赋予带符号的目标电流，完美支持正反转

#elif (CURRENT_CTRL_STRATEGY == CTRL_STRATEGY_FOC_MTPA)
    /* ----------------------------------------------------
     * 策略 B: MTPA 最大转矩电流比 (适合 IPM)
     * ----------------------------------------------------*/
    
    // 1. 提取目标电流的绝对值和符号 (核心修复点，支持四象限运行)
    float Is_mag = fabsf(Is_target); 
    float sign = (Is_target < 0.0f) ? -1.0f : 1.0f;

    // 如果目标电流接近0，直接清零防抖
    if (Is_mag <= 0.001f) {
        *Id_Ref = 0.0f;
        *Iq_Ref = 0.0f;
        return;
    }

    // 2. 降级保护：如果 Ld >= Lq (错填参数或电机无凸极效应)，退化为 Id=0
    if (MOTOR_LD >= MOTOR_LQ) {
        *Id_Ref = 0.0f;
        *Iq_Ref = Is_target;
        return;
    }

    // 3. 解析法求 MTPA 的 Id
    // 公式: Id = (Flux - sqrt(Flux^2 + 8 * (Lq-Ld)^2 * Is^2)) / (4 * (Lq-Ld))
    float L_diff = MOTOR_LQ - MOTOR_LD;
    float temp = MOTOR_FLUX * MOTOR_FLUX + 8.0f * L_diff * L_diff * Is_mag * Is_mag;
    
    *Id_Ref = (MOTOR_FLUX - sqrtf(temp)) / (4.0f * L_diff); // Id 恒为负，用于利用磁阻转矩
    
    // 4. 求 Iq 并恢复符号 (赋予制动或反向运行能力)
    float Iq_calc = sqrtf(Is_mag * Is_mag - (*Id_Ref) * (*Id_Ref));
    *Iq_Ref = Iq_calc * sign;

#else
    *Id_Ref = 0.0f;
    *Iq_Ref = Is_target;
#endif
}

void Strategy_VF_Process(const Vector_3Phase_t *I_abc, float Angle, float target_rpm, float Udc, uint16_t *pwm_out) 
{
#if (CURRENT_CTRL_STRATEGY == CTRL_STRATEGY_VF)
    vf_ramp.Target = target_rpm;
    
    if (vf_ramp.Current < vf_ramp.Target) {
        vf_ramp.Current += vf_ramp.Step;
        if (vf_ramp.Current > vf_ramp.Target) vf_ramp.Current = vf_ramp.Target; // 防止超调
    } 
    else if (vf_ramp.Current > vf_ramp.Target) {
        vf_ramp.Current -= vf_ramp.Step;
        if (vf_ramp.Current < vf_ramp.Target) vf_ramp.Current = vf_ramp.Target;
    }

    // 取出经过斜坡平滑处理后的安全转速！
    float ramped_rpm = vf_ramp.Current;
    // 1. 速度转电频率 (Hz)
    float target_freq_hz = (ramped_rpm * POLE_PAIRS) / 60.0f; 
    
    // 2. 积分生成开环电角度
    vf_angle += MATH_2PI * target_freq_hz * TS;
    if (vf_angle >= MATH_2PI) vf_angle -= MATH_2PI;
    else if (vf_angle < 0.0f) vf_angle += MATH_2PI;

    FOC_Current_Loop(I_abc, vf_angle, 0.5, 0, Udc, ramped_rpm, pwm_out);

    /*// 3. 压频比计算 (V/F)
    // 注意：0.00165f 是个经验系数，你可能需要根据实际电机修改
    float v_f_ratio = 0.00165f; 
    float v_mag = fabsf(ramped_rpm) * v_f_ratio + 5.0f; // +5V 为低速电压提升，克服静摩擦
    
    // 安全限幅：不超过母线电压允许的最大不失真正弦波幅值
    float v_limit = Udc * VALUE_1_SQRT3;
    if (v_mag > v_limit) v_mag = v_limit;

    // 4. 生成虚拟电压矢量
    float Ualpha = v_mag * arm_cos_f32(vf_angle);
    float Ubeta  = v_mag * arm_sin_f32(vf_angle);
    
    // 5. 开环发波
    SVPWM_Calc(Ualpha, Ubeta, Udc, &pwm_out[0], &pwm_out[1], &pwm_out[2]);*/
#endif
}

void Strategy_Position_Align(float Udc, uint16_t *pwm_a, uint16_t *pwm_b, uint16_t *pwm_c)
{
    // 静态变量用于在 40kHz 中断中记录真实逝去的时间和状态
    static uint32_t align_ticks = 0;       
    static uint32_t last_check_ms = 0;     
    static uint16_t position_count = 0;    
    static uint16_t position_filter = 0;   
    
    // 计算当前定位序列已运行的真实毫秒数 (align_ticks * TS * 1000)
    uint32_t position_time_ms = (uint32_t)(align_ticks * TS * 1000.0f);
    
    float align_Ualpha = 0.0f;
    float align_Ubeta = 0.0f;

    /* 1. 执行 4 段拖拽电压矢量序列 */
    if (position_time_ms <= 1200) {
        align_Ualpha = 5.0f;   align_Ubeta = 0.0f;
    } 
    else if (position_time_ms <= 2400) {
        align_Ualpha = -2.5f;  align_Ubeta = 4.3f;
    } 
    else if (position_time_ms <= 3600) {
        align_Ualpha = -2.5f;  align_Ubeta = -4.3f;
    } 
    else {
        align_Ualpha = 5.0f;   align_Ubeta = 0.0f; 
    }

    /* 2. 大于 4800ms 后，开始执行防抖滤波检查 */
    if (position_time_ms > 4800) 
    {
        if (position_time_ms > last_check_ms) 
        {
            last_check_ms = position_time_ms;
            
            // 获取当前 RAW 编码器计数值
            uint16_t position_new_count = Motor_Get_Encoder_Count();
            
            // 计算绝对差值
            int16_t diff = (int16_t)position_new_count - (int16_t)position_count;
            if (diff < 0) diff = -diff; 
            
            if (diff < 5) {
                position_filter++;
                if (position_filter > 5) {
                    /* 🎉 定位成功！记录零点并退出 🎉 */
                    Acount = position_new_count;
                    
                    // 清理标志位，结束定位模式
                    MotorSys.Position_Flag = 0;
                    align_ticks = 0;
                    position_filter = 0;
                    last_check_ms = 0;
                    
                    FOC_PI_Clear(); // 退出前清空积分
                }
            } else {
                position_filter = 0; // 晃动则重新累计
            }
            position_count = position_new_count;
        }
    }

    /* 3. 执行开环电压计算 */
    SVPWM_Calc(align_Ualpha, align_Ubeta, Udc, pwm_a, pwm_b, pwm_c);
    
    /* 4. 时间刻度推进 */
    if (MotorSys.Position_Flag == 1) {
        align_ticks++;
    }
}