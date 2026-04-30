/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_hardware.h"
#include "motor_state.h"
#include "foc.h"
#include "control_strategy.h"
#include "sensorless.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE ((uint32_t) 4)
#define BUFFER_SIZE_VOLTAGE ((uint32_t) 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc3;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

/* USER CODE BEGIN EV */
extern uint16_t aADCxConvertedData[BUFFER_SIZE];
extern uint16_t aADCxConvertedData_Voltage[BUFFER_SIZE_VOLTAGE];

extern float target_speed_rpm;
extern float speed_act_rpm;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}
/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */

void DMA1_Stream0_IRQHandler(void) 
{ 
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void) 
{ 
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3); 
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC_IRQHandler(void)          
{ 
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles ADC3 global interrupt.
  */
void ADC3_IRQHandler(void)         
{ 
  /* USER CODE BEGIN ADC3_IRQn 0 */

  /* USER CODE END ADC3_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc3); 
  /* USER CODE BEGIN ADC3_IRQn 1 */

  /* USER CODE END ADC3_IRQn 1 */
}
void TIM8_UP_TIM13_IRQHandler(void){ HAL_TIM_IRQHandler(&htim8); }

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
  // 检查 IPM_RFE (故障) 或 IPM_T (过热) 引脚
  if (__HAL_GPIO_EXTI_GET_IT(IPM_RFE_Pin) != 0x00U || 
      __HAL_GPIO_EXTI_GET_IT(IPM_T_Pin) != 0x00U)
  {
    Motor_PWM_Stop(); // 1. 立即封锁 PWM 脉冲
    MotorSys.Run_Flag = 0;     // 2. 清除运行标志位
    FOC_PI_Clear();   // 3. 清空积分器防积分饱和
  }
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(IPM_RFE_Pin);
  HAL_GPIO_EXTI_IRQHandler(IPM_T_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
}

/**
  * @brief EXTI [15:10] 中断 - 编码器 Z 相归零
  */
void EXTI15_10_IRQHandler(void)
{
    if(MotorSys.Position_Flag) {
        MotorSys.Position_Flag = 1;
    }
    HAL_GPIO_EXTI_IRQHandler(ENCODER_Z_Pin);
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)aADCxConvertedData, 4);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t *)aADCxConvertedData_Voltage, 1);

  HAL_TIM_IRQHandler(&htim1);
}

/* ========================================================================= */
/*  核心高频电流环：ADC 转换完成回调                    */
/* ========================================================================= */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1) 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

        float Ia, Ib, Ic, Ibus;
        float Udc;
        float elec_angle_rad;    
        float speed_act_rpm;     
        
        // 🚨 修复点 1：将独立的 a,b,c 合并为一个数组
        uint16_t pwm_out[3] = {0, 0, 0}; 
        
        Vector_3Phase_t I_abc;

        Motor_Get_Phase_Currents(&Ia, &Ib, &Ic, &Ibus);
        Motor_Get_Bus_Voltage(&Udc);
        Motor_Update_Encoder_Data(&elec_angle_rad, &speed_act_rpm);

        I_abc.A = Ia;
        I_abc.B = Ib;
        I_abc.C = Ic;

        /* =======================================================
         * 状态机优先级 1：转子预定位
         * ======================================================= */
        if (MotorSys.Position_Flag == 1)
        {
            MotorSys.State = SYS_INIT; 
            // 🚨 修复点 2：传入数组元素的地址
            Strategy_Position_Align(Udc, &pwm_out[0], &pwm_out[1], &pwm_out[2]);
            Motor_Set_PWM_Duty(pwm_out[0], pwm_out[1], pwm_out[2]);
        }
        /* =======================================================
         * 状态机优先级 2：正常运行发波
         * ======================================================= */
        else if (MotorSys.Run_Flag == 1)
        {
            MotorSys.State = SYS_RUNNING; 
            
          #if (CURRENT_CTRL_STRATEGY == CTRL_STRATEGY_VF)

            Strategy_VF_Process(200.0f, Udc, pwm_out); 
          #else
            float final_angle;
            float actual_speed_rpm;
            
          #if (CURRENT_SENSORLESS_MODE == SENSORLESS_NONE)
            final_angle = elec_angle_rad;   
            actual_speed_rpm = speed_act_rpm; 
          #else
            final_angle = Angle_Est;        
            actual_speed_rpm = Speed_Est_RPM;
          #endif

            float Is_target = PI_Calc(&PI_Speed, target_speed_rpm, actual_speed_rpm);

            float target_Id = 0.0f;
            float target_Iq = 0.0f;
            Strategy_Get_Id_Iq_Ref(Is_target, &target_Id, &target_Iq);
            
            FOC_Current_Loop(&I_abc, final_angle, target_Iq, target_Id, Udc, pwm_out);
          #endif
            
            Motor_Set_PWM_Duty(pwm_out[0], pwm_out[1], pwm_out[2]);
        }
        /* =======================================================
         * 🌟 状态机优先级 3：待机停机
         * ======================================================= */
        else
        {
            if(MotorSys.State != SYS_FAULT) {
                MotorSys.State = SYS_IDLE; 
            }
            Motor_Set_PWM_Duty(0, 0, 0); 
            FOC_PI_Clear();
        }

        uint16_t dac_ch1 = (uint16_t)((Ia * 10.0f) + 2048.0f); 
        uint16_t dac_ch2 = (uint16_t)((elec_angle_rad / MATH_2PI) * 4095.0f);
        Motor_Debug_DAC_Output(dac_ch1, dac_ch2);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }
}
