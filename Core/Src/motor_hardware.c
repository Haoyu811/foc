/* === 文件名: motor_hardware.c === */
#include "motor_hardware.h"
#include "motor_state.h"
#include "main.h"
#include "foc_math.h"

// 引入 STM32 HAL 库外设句柄
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
extern DAC_HandleTypeDef hdac1;

// ADC DMA 缓冲区 (仅在本文件内部可见，对外部算法屏蔽)
#define BUFFER_SIZE ((uint32_t) 4)
#define BUFFER_SIZE_VOLTAGE ((uint32_t) 1)
uint16_t aADCxConvertedData[BUFFER_SIZE];
uint16_t aADCxConvertedData_Voltage[BUFFER_SIZE_VOLTAGE];

// ADC 零偏值缓存
static uint16_t PhaseAOffset = 0, PhaseBOffset = 0, PhaseCOffset = 0, IBusOffset = 0;
static uint16_t UdcOffset = 0;

/* ================== 私有函数声明 ================== */
static void Motor_Calibrate_ADC_Offset(void);

/* ================== 接口函数实现 ================== */

void Motor_Hardware_Init(void)
{
    /* 1. 闭合主回路继电器 */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET); 
    HAL_Delay(100); 

    /* 2. STM32 硬件 ADC 内部线性度校准 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);

    /* 3. 必须先启动 TIM1 的通道 4 (维持 ADC 触发脉冲) */
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
    // 给系统 1ms 时间让定时器和中断稳住阵脚
    HAL_Delay(1); 

    /* 4. 电流零偏软件校准 (此时触发脉冲已就绪，ADC 可以正常拿到数据了) */
    Motor_Calibrate_ADC_Offset();

    /* 5. 启动编码器定时器 (TIM3) */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    /* 6. 启动 DAC 调试输出 */
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
}

void Motor_PWM_Start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // 维持 ADC 触发信号
}

void Motor_PWM_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    // 注意：通道4 (ADC触发) 根据需求可不关闭，以保持持续采样监测
    //HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
}

void Motor_Set_PWM_Duty(uint16_t pwm_a, uint16_t pwm_b, uint16_t pwm_c)
{
    // 底层安全限幅，防止占空比异常导致直通爆炸
    if(pwm_a > PWM_DUTY_LIMIT) pwm_a = PWM_DUTY_LIMIT; else if(pwm_a == 0) pwm_a = 1;
    if(pwm_b > PWM_DUTY_LIMIT) pwm_b = PWM_DUTY_LIMIT; else if(pwm_b == 0) pwm_b = 1;
    if(pwm_c > PWM_DUTY_LIMIT) pwm_c = PWM_DUTY_LIMIT; else if(pwm_c == 0) pwm_c = 1;

    TIM1->CCR1 = pwm_a;
    TIM1->CCR2 = pwm_b;
    TIM1->CCR3 = pwm_c;
}

void Motor_Get_Phase_Currents(float *pIu, float *pIv, float *pIw, float *pIbus)
{
    // 获取 ADC DMA 数据，减去零偏，并根据你原先的系数 (50.0f / 4096.0f) / 7.0f 进行标幺化/转换
    // 这里将 ADC_CURRENT_COEFF 定义在了 motor_config.h 中以消除魔法数字
    
    int16_t count_u = aADCxConvertedData[0] - PhaseAOffset;
    int16_t count_v = aADCxConvertedData[1] - PhaseBOffset;
    int16_t count_w = aADCxConvertedData[2] - PhaseCOffset;
    int16_t count_bus = aADCxConvertedData[3] - IBusOffset;

    *pIu   = (float)count_u * ADC_CURRENT_COEFF / 7.0f;
    *pIv   = (float)count_v * ADC_CURRENT_COEFF / 7.0f;
    *pIw   = (float)count_w * ADC_CURRENT_COEFF / 7.0f;
    *pIbus = (float)count_bus * ADC_CURRENT_COEFF / 7.0f;
}

void Motor_Get_Bus_Voltage(float *pUdc)
{
    *pUdc = (float)aADCxConvertedData_Voltage[0] * ADC_VOLTAGE_COEFF;
}

extern uint16_t Acount; // 确保声明了定位函数里求出的零偏

void Motor_Update_Encoder_Data(float *pElec_Angle_Rad, float *pSpeed_RPM)
{
    uint16_t encoder_new_count = __HAL_TIM_GET_COUNTER(&htim3);
    uint16_t angle_m_count;

    /* =======================================================
     * 1. 你的原始逻辑：反向补偿与 Acount 零偏对齐 
     * ======================================================= */
    if (encoder_new_count < Acount) {
        angle_m_count = Acount - encoder_new_count;
    } else {
        // 修正了你的 4095 - ... + 1 为 4096
        angle_m_count = 4096 - encoder_new_count + Acount; 
    }

    /* =======================================================
     * 2. 电角度计算 (直接映射为弧度制给 FOC 算法使用)
     * 等效于你的 (angle_m_count % 1365) * 3
     * ======================================================= */
    *pElec_Angle_Rad = ((float)((uint32_t)angle_m_count * POLE_PAIRS % ENCODER_LINES) / ENCODER_LINES) * MATH_2PI;

    /* =======================================================
     * 3. 你的 40 点滑动窗口差分测速 (升级为 O(1) 环形队列)
     * 在 40kHz 中断下，40个样本刚好是 1ms 的时间窗！
     * ======================================================= */
    #define SPEED_WINDOW 40
    static uint16_t po[SPEED_WINDOW] = {0};
    static uint8_t ptr = 0;

    // 取出 40 拍之前的历史机械角度
    uint16_t oldest_angle = po[ptr]; 
    // 存入当前最新机械角度
    po[ptr] = angle_m_count;         
    
    // 环形指针推进
    ptr++;
    if (ptr >= SPEED_WINDOW) ptr = 0; 

    /* =======================================================
     * 4. 速度防溢出 (Unwrap) 与 Q16 系数还原
     * ======================================================= */
    int16_t speed_count = (int16_t)angle_m_count - (int16_t)oldest_angle;

    // 替代你原有的 if(speed_count > -1500 && ...) 直接丢弃的做法，
    // 我们用标准的过零无损还原法，跨越 0 点时速度也不会断层：
    if (speed_count > (ENCODER_LINES / 2)) {
        speed_count -= ENCODER_LINES;
    } else if (speed_count < -(ENCODER_LINES / 2)) {
        speed_count += ENCODER_LINES;
    }

    // 你的原版缩放还原：speed_count * (960 / 65536.0f) = speed_count * 0.0146484375 * 1000 = speed_count * 14.6484
    // 因为物理公式 RPM = (差值 / 4096) * (1s / 0.001s) * 60 = 差值 * 14.6484
    // 两者完美吻合！
    *pSpeed_RPM = (float)speed_count * 14.6484375f;
}

void Motor_Debug_DAC_Output(uint16_t ch1_val, uint16_t ch2_val)
{
    // 安全限幅
    if(ch1_val > 4095) ch1_val = 4095;
    if(ch2_val > 4095) ch2_val = 4095;
    
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ch1_val);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, ch2_val);
}

/* ================== 私有校准函数实现 ================== */
static void Motor_Calibrate_ADC_Offset(void)
{
    uint32_t sum_A = 0, sum_B = 0, sum_C = 0, sum_Bus = 0;
    const uint8_t CALI_TIMES = 16;

    MotorSys.Calibrate_Flag = 1; // 开始校准

    for(int i = 0; i < CALI_TIMES; i++)
    {
        // 延时 2ms，由于中断在后台跑，这 2ms 内 DMA 早就把最新鲜的数据放进数组里了
        HAL_Delay(2); 
        
        sum_A   += aADCxConvertedData[0];
        sum_B   += aADCxConvertedData[1];
        sum_C   += aADCxConvertedData[2];
        sum_Bus += aADCxConvertedData[3];
    }

    MotorSys.Calibrate_Flag = 0; // 校准完毕

    PhaseAOffset = sum_A / CALI_TIMES;
    PhaseBOffset = sum_B / CALI_TIMES;
    PhaseCOffset = sum_C / CALI_TIMES;
    IBusOffset   = sum_Bus / CALI_TIMES;
}