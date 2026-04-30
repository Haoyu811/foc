/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines  -----------*/
#define USER_BUTTON_Pin GPIO_PIN_0
#define USER_BUTTON_GPIO_Port GPIOA
#define DAC1_Pin GPIO_PIN_4
#define DAC1_GPIO_Port GPIOA
#define DAC2_Pin GPIO_PIN_5
#define DAC2_GPIO_Port GPIOA
#define IV_Pin GPIO_PIN_6
#define IV_GPIO_Port GPIOA
#define IW_Pin GPIO_PIN_4
#define IW_GPIO_Port GPIOC
#define Ibus_Pin GPIO_PIN_1
#define Ibus_GPIO_Port GPIOB
#define IU_Pin GPIO_PIN_11
#define IU_GPIO_Port GPIOF
#define RELAY_Pin GPIO_PIN_7
#define RELAY_GPIO_Port GPIOE
#define IPM_Un_Pin GPIO_PIN_8
#define IPM_Un_GPIO_Port GPIOE
#define IPM_U_Pin GPIO_PIN_9
#define IPM_U_GPIO_Port GPIOE
#define IPM_Vn_Pin GPIO_PIN_10
#define IPM_Vn_GPIO_Port GPIOE
#define IPM_V_Pin GPIO_PIN_11
#define IPM_V_GPIO_Port GPIOE
#define IPM_Wn_Pin GPIO_PIN_12
#define IPM_Wn_GPIO_Port GPIOE
#define IPM_W_Pin GPIO_PIN_13
#define IPM_W_GPIO_Port GPIOE
#define IPM_RFE_Pin GPIO_PIN_7
#define IPM_RFE_GPIO_Port GPIOC
#define IPM_RFE_EXTI_IRQn EXTI9_5_IRQn
#define IPM_T_Pin GPIO_PIN_8
#define IPM_T_GPIO_Port GPIOC
#define IPM_T_EXTI_IRQn EXTI9_5_IRQn
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define ENCODER_Z_Pin GPIO_PIN_15
#define ENCODER_Z_GPIO_Port GPIOG
#define ENCODER_Z_EXTI_IRQn EXTI15_10_IRQn
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LED_Y_Pin GPIO_PIN_1
#define LED_Y_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
