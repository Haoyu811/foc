/* === 文件名: sensorless.c === */
#include "sensorless.h"
#include "foc.h" // 借用 PI 算法结构体

float Angle_Est = 0.0f;
float Speed_Est_RPM = 0.0f;

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

#endif

void Sensorless_Init(void) {
    /* 1. 彻底清零对外的全局输出变量 */
    Angle_Est = 0.0f;
    Speed_Est_RPM = 0.0f;

#if (CURRENT_SENSORLESS_MODE != SENSORLESS_NONE)
    /* 2. 锁相环 (PLL) 参数初始化与历史积分彻底清零 */
    PI_PLL.Kp = KP_PLL; 
    PI_PLL.Ki = KI_PLL; 
    PI_PLL.Out_Limit = 10000.0f;
    PI_PLL.Error_Now = 0.0f;
    PI_PLL.Error_Integral = 0.0f; 
    PI_PLL.Out = 0.0f;
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

void Sensorless_Resolver(float Id_fb, float Iq_fb, float Ualpha, float Ubeta) {
    float angle_error = 0.0f;

#if (CURRENT_SENSORLESS_MODE == SENSORLESS_HFI)
    // 1. 生成高频信号
#elif (CURRENT_SENSORLESS_MODE == SENSORLESS_SMO)
    // SMO 开关函数与反电动势提取计算 angle_error
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