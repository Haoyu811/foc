/* === 文件名: sensorless.c === */
#include "sensorless.h"
#include "foc.h" // 借用 PI 算法结构体
#include "math.h"

float Angle_Est = 0.0f;
float we = 0.0f;
float Speed_Est_RPM = 0.0f;
float angle_error = 0.0f;

#if (CURRENT_SENSORLESS_MODE != SENSORLESS_NONE)
  static PI_Controller_t PI_PLL; // 锁相环专有PI
#endif

#if (CURRENT_SENSORLESS_MODE == SENSORLESS_HFI || CURRENT_SENSORLESS_MODE == SENSORLESS_HYBRID)
  // --- 仅在开启 HFI 时编译的代码 ---
  typedef struct { float Amp; float Freq; float Ts; float Step; float angle_hfi; float Out; } struct_HFI;
  static struct_HFI hfi;
  
#endif

#if (CURRENT_SENSORLESS_MODE == SENSORLESS_SMO || CURRENT_SENSORLESS_MODE == SENSORLESS_HYBRID)
  // --- 滑模观测器专有变量 --- 
  static SMO_State_t smo;
#endif

void Sensorless_Init(void) {
    /* 1. 彻底清零对外的全局输出变量 */
    Angle_Est = 0.0f;
    we = 0.0f;
    Speed_Est_RPM = 0.0f;
    angle_error = 0.0f;

#if (CURRENT_SENSORLESS_MODE != SENSORLESS_NONE)
    /* 2. 锁相环 (PLL) 参数初始化与历史积分彻底清零 */
    PI_PLL.Kp = KP_PLL; 
    PI_PLL.Ki = KI_PLL; 
    PI_PLL.Out_Limit = 10000.0f;
    PI_PLL.Error_Now = 0.0f;
    PI_PLL.Error_Integral = 0.0f; 
    PI_PLL.Out = 0.0f;
#endif

#if (CURRENT_SENSORLESS_MODE == SENSORLESS_SMO || CURRENT_SENSORLESS_MODE == SENSORLESS_HYBRID)
    /* 3. SMO 状态量清零 */
    smo.Ialpha_est = 0.0f;
    smo.Ibeta_est = 0.0f;
    smo.Ealpha_est = 0.0f;
    smo.Ebeta_est = 0.0f;
#endif

#if (CURRENT_SENSORLESS_MODE == SENSORLESS_HFI || CURRENT_SENSORLESS_MODE == SENSORLESS_HYBRID)
    /* 3. 高频注入 (HFI) 信号发生器状态清零 */
    hfi.Amp = HFI_AMP; 
    hfi.Freq = HFI_FREQ; 
    hfi.Ts = TS;
    hfi.Step = MATH_2PI * hfi.Freq * hfi.Ts;
    hfi.angle_hfi = 0.0f; 
    hfi.Out = 0.0f;
    
#endif
}

void LPF_Init(LPF_State_t *lpf, float Ts, float Wc) {
    lpf->Out_Last = 0.0f;
    lpf->Alpha = Ts * Wc;
    // 离散化安全保护：确保 Alpha 在 0~1 之间，防止发散
    if (lpf->Alpha > 1.0f) lpf->Alpha = 1.0f; 
}

void Sensorless_Resolver(float Iad_fb, float Ibq_fb, float Ualpha, float Ubeta) {

#if (CURRENT_SENSORLESS_MODE == SENSORLESS_HFI)
    // 1. 生成高频信号
#elif (CURRENT_SENSORLESS_MODE == SENSORLESS_SMO)
    // SMO 开关函数与反电动势提取计算 angle_error
    // 1. 计算电流误差
    float err_Ialpha = smo.Ialpha_est - Iad_fb;
    float err_Ibeta  = smo.Ibeta_est  - Ibq_fb;
    // 2. 开关函数估计反电动势
    smo.Zalpha = SMO_K * SMO_Sigmoid(err_Ialpha, SIGMOID_A);
    smo.Zalpha = SMO_K * SMO_Sigmoid(err_Ialpha, SIGMOID_A);

    // 3. 离散模型更新估算电流
    smo.Ialpha_est = TS_OVER_LD * (Ualpha - we * LD_MINUS_LQ * Ibq_fb - smo.Zalpha) + ONE_MINUS_TS_R_LD * smo.Ialpha_est;
    smo.Ibeta_est = TS_OVER_LD * (Ubeta + we * LD_MINUS_LQ * Iad_fb - smo.Zbeta) + ONE_MINUS_TS_R_LD * smo.Ibeta_est;

    // 4. PLL 锁相环计算角度误差 （误差 = -E_alpha * cos(theta) - E_beta * sin(theta)）
    float angle_error = -smo.Zalpha * arm_cos_f32(Angle_Est) - smo.Zbeta * arm_sin_f32(Angle_Est);

#endif

#if (CURRENT_SENSORLESS_MODE != SENSORLESS_NONE)
    // 统一锁相环 (PLL) 更新角度与速度
    PI_Calc(&PI_PLL, 0.0f, angle_error);
    Angle_Est += PI_PLL.Out * TS;
    if(Angle_Est >= MATH_2PI) Angle_Est -= MATH_2PI;
    else if(Angle_Est < 0.0f) Angle_Est += MATH_2PI;
    
    Speed_Est_RPM = (PI_PLL.Out * 60.0f) / (MATH_2PI * POLE_PAIRS); // (后续可加低通滤波)
#endif
}

float Sensorless_Get_Inject_Vd(void) {
#if (CURRENT_SENSORLESS_MODE == SENSORLESS_HFI || CURRENT_SENSORLESS_MODE == SENSORLESS_HYBRID)
    return hfi.Out; // D轴注入高频电压
#else
    return 0.0f; 
#endif
}

float Sensorless_Get_Inject_Vq(void) {
    return 0.0f; // Q轴通常不注入
}

float SMO_Sign(float error) {
    if (error > 0.0f) return 1.0f;
    if (error < 0.0f) return -1.0f;
    return 0.0f;
}

float SMO_Sigmoid(float error, float a) {
    // 限制指数范围，防止浮点数下溢/溢出导致硬错误
    float exp_term = -a * error;
    if (exp_term > 50.0f) exp_term = 50.0f;
    if (exp_term < -50.0f) exp_term = -50.0f;
    
    return (2.0f / (1.0f + expf(exp_term))) - 1.0f;
}

float SMO_Fast_Sigmoid(float error, float a) {
    return error / (fabsf(error) + a);
}

float LPF_Calc(LPF_State_t *lpf, float input) {
    // 公式：Y(n) = Y(n-1) + alpha * (X(n) - Y(n-1))
    lpf->Out_Last += lpf->Alpha * (input - lpf->Out_Last);
    return lpf->Out_Last;
}

