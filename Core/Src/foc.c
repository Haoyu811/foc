#include "foc.h"

/* ================= 实例化 PI 控制器 ================= */
PI_Controller_t PI_Id;
PI_Controller_t PI_Iq;
PI_Controller_t PI_Speed;
PI_Controller_t PI_Phase;

/* ================= 算法函数实现 ================= */

void FOC_PI_Init(void)
{
    // 从 motor_config.h 加载参数
    PI_Id.Kp = KP_ID;       PI_Id.Ki = KI_ID;       PI_Id.Out_Limit = UDC_NOMINAL * VALUE_1_SQRT3;
    PI_Iq.Kp = KP_IQ;       PI_Iq.Ki = KI_IQ;       PI_Iq.Out_Limit = UDC_NOMINAL * VALUE_1_SQRT3;
    
    PI_Speed.Kp = KP_SPD;   PI_Speed.Ki = KI_SPD;   PI_Speed.Out_Limit = 1.0f; // 额定电流标幺值或最大电流
    
    PI_Phase.Kp = KP_PLL;   PI_Phase.Ki = KI_PLL;   PI_Phase.Out_Limit = 10000.0f; // PLL 频率限幅
    
    FOC_PI_Clear();
}

void FOC_PI_Clear(void)
{
    PI_Id.Error_Integral = 0.0f;     PI_Id.Out = 0.0f;
    PI_Iq.Error_Integral = 0.0f;     PI_Iq.Out = 0.0f;
    PI_Speed.Error_Integral = 0.0f;  PI_Speed.Out = 0.0f;
    PI_Phase.Error_Integral = 0.0f;  PI_Phase.Out = 0.0f;
}

float PI_Calc_Standard(PI_Controller_t *pi, float target, float feedback)
{
    pi->Error_Now = target - feedback;
    
    pi->Error_Integral += pi->Error_Now; 
    
    // 比例+积分计算
    pi->Out = (pi->Kp * pi->Error_Now) + (pi->Ki * pi->Error_Integral);
    
    if(pi->Out > pi->Out_Limit) {
        pi->Out = pi->Out_Limit;
    } 
    else if(pi->Out < -pi->Out_Limit) {
        pi->Out = -pi->Out_Limit;
    }
    
    return pi->Out;
}

float PI_Calc_AntiWindup(PI_Controller_t *pi, float target, float feedback)
{
    pi->Error_Now = target - feedback;
    
    // 比例+积分计算
    pi->Out = (pi->Kp * pi->Error_Now) + (pi->Ki * pi->Error_Integral);
    
    // 积分项累加
    pi->Error_Integral += pi->Error_Now;
    
    // 抗积分饱和 (Anti-windup) 与输出限幅
    if(pi->Out > pi->Out_Limit) {
        pi->Out = pi->Out_Limit;
        // 如果输出已经饱和，且误差仍在推动积分朝着饱和方向走，则停止积分
        if(pi->Error_Now > 0) pi->Error_Integral -= pi->Error_Now; 
    } 
    else if(pi->Out < -pi->Out_Limit) {
        pi->Out = -pi->Out_Limit;
        if(pi->Error_Now < 0) pi->Error_Integral -= pi->Error_Now;
    }
    
    return pi->Out;
}

float PI_Calc(PI_Controller_t *pi, float target, float feedback) {
#if USE_ANTI_WINDUP
    return PI_Calc_AntiWindup(pi, target, feedback);
#else
    return PI_Calc_Standard(pi, target, feedback);
#endif
}

void SVPWM_Calc(float Ualpha, float Ubeta, float Udc, uint16_t *pwm_a, uint16_t *pwm_b, uint16_t *pwm_c)
{
    float X, Y, Z;
    float T1, T2;
    float Ta, Tb, Tc;
    uint8_t sector = 0;
    
    // 1. 判断扇区
    float ua = Ubeta;
    float ub = VALUE_SQRT3_2 * Ualpha - 0.5f * Ubeta;
    float uc = -VALUE_SQRT3_2 * Ualpha - 0.5f * Ubeta;

    sector = ((ua > 0) + ((ub > 0) << 1) + ((uc > 0) << 2));

    if (Udc < 10.0f) Udc = 10.0f; // 防止除以0，也防止电压过低

    // 2. 计算基本矢量作用时间 (归一化到开关周期)
    X = VALUE_SQRT3 * Ubeta / Udc;
    Y = (1.5f * Ualpha + VALUE_SQRT3_2 * Ubeta) / Udc;
    Z = Y - 3.0f * Ualpha / Udc;

    switch (sector) {
        case 1: T1 =  Z; T2 =  Y; break;
        case 2: T1 =  Y; T2 = -X; break;
        case 3: T1 = -Z; T2 =  X; break;
        case 4: T1 = -X; T2 =  Z; break;
        case 5: T1 =  X; T2 = -Y; break;
        default:T1 = -Y; T2 = -Z; break; // Case 6
    }

    // 3. 过调制处理 (Optimum)
    float sum_t12 = T1 + T2;
    if (sum_t12 > 1.0f) {
        T1 = T1 / sum_t12;
        T2 = 1.0f - T1; // 或者 T2 = T2 / sum_t12; 保持比例
    }

    // 4. 计算三相占空比时间
    Ta = 0.25f * (1.0f - T1 - T2); // 零矢量均匀分配 (七段式)
    Tb = Ta + T1 * 0.5f;
    Tc = Tb + T2 * 0.5f;

    // 5. 映射到定时器计数值 (CCR)
    uint16_t a=0, b=0, c=0;
    switch (sector) {
        case 1: a = (uint16_t)(Tb * TIM1_PERIOD); b = (uint16_t)(Ta * TIM1_PERIOD); c = (uint16_t)(Tc * TIM1_PERIOD); break;
        case 2: a = (uint16_t)(Ta * TIM1_PERIOD); b = (uint16_t)(Tc * TIM1_PERIOD); c = (uint16_t)(Tb * TIM1_PERIOD); break;
        case 3: a = (uint16_t)(Ta * TIM1_PERIOD); b = (uint16_t)(Tb * TIM1_PERIOD); c = (uint16_t)(Tc * TIM1_PERIOD); break;
        case 4: a = (uint16_t)(Tc * TIM1_PERIOD); b = (uint16_t)(Tb * TIM1_PERIOD); c = (uint16_t)(Ta * TIM1_PERIOD); break;
        case 5: a = (uint16_t)(Tc * TIM1_PERIOD); b = (uint16_t)(Ta * TIM1_PERIOD); c = (uint16_t)(Tb * TIM1_PERIOD); break;
        case 6: a = (uint16_t)(Tb * TIM1_PERIOD); b = (uint16_t)(Tc * TIM1_PERIOD); c = (uint16_t)(Ta * TIM1_PERIOD); break;
        default:a = TIM1_PERIOD / 2; b = TIM1_PERIOD / 2; c = TIM1_PERIOD / 2; break; // 扇区0保护
    }

    *pwm_a = a;
    *pwm_b = b;
    *pwm_c = c;
}

void FOC_Current_Loop(const Vector_3Phase_t *I_abc, float Angle, float Iq_Ref, float Id_Ref, float Udc, uint16_t *pwm_out)
{
    Vector_AlBe_t I_albe;
    Vector_DQ_t   I_dq;
    Vector_DQ_t   V_dq;
    Vector_AlBe_t V_albe;
    Trig_Vals_t   trig;

    // 1. 角度正余弦计算
    FOC_Trig_Calc(Angle, &trig);

    // 2. Clarke & Park 变换 (电流读入)
    Clarke_Transform(I_abc, &I_albe);
    Park_Transform(&I_albe, &trig, &I_dq);

    // 3. PI 闭环控制 (这里更新母线电压限幅以应对母线波动)
    float v_limit = Udc * VALUE_1_SQRT3; 
    PI_Id.Out_Limit = v_limit;
    PI_Iq.Out_Limit = v_limit;

    V_dq.D = PI_Calc(&PI_Id, Id_Ref, I_dq.D);
    V_dq.Q = PI_Calc(&PI_Iq, Iq_Ref, I_dq.Q);

    // 4. 逆 Park 变换 (电压输出)
    Inv_Park_Transform(&V_dq, &trig, &V_albe);

    // 5. SVPWM 计算发波占空比
    SVPWM_Calc(V_albe.Alpha, V_albe.Beta, Udc, &pwm_out[0], &pwm_out[1], &pwm_out[2]);
}