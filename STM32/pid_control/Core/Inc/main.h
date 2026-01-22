/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define Left_PWM_Pin GPIO_PIN_0
#define Left_PWM_GPIO_Port GPIOA
#define Right_PWM_Pin GPIO_PIN_1
#define Right_PWM_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define left_IN2_Pin GPIO_PIN_4
#define left_IN2_GPIO_Port GPIOC
#define Right_IN1_Pin GPIO_PIN_5
#define Right_IN1_GPIO_Port GPIOC
#define Left_IN1_Pin GPIO_PIN_13
#define Left_IN1_GPIO_Port GPIOB
#define Left_pulse_B_Pin GPIO_PIN_14
#define Left_pulse_B_GPIO_Port GPIOB
#define Left_pulse_B_EXTI_IRQn EXTI15_10_IRQn
#define Left_pulse_A_Pin GPIO_PIN_15
#define Left_pulse_A_GPIO_Port GPIOB
#define Left_pulse_A_EXTI_IRQn EXTI15_10_IRQn
#define Right_IN2_Pin GPIO_PIN_6
#define Right_IN2_GPIO_Port GPIOC
#define HOOK_Input_Pin GPIO_PIN_7
#define HOOK_Input_GPIO_Port GPIOC
#define Actuator1_Lift_Pin GPIO_PIN_8
#define Actuator1_Lift_GPIO_Port GPIOC
#define Actuator2_Lift_Pin GPIO_PIN_9
#define Actuator2_Lift_GPIO_Port GPIOC
#define Actuator1_HOOK_Pin GPIO_PIN_8
#define Actuator1_HOOK_GPIO_Port GPIOA
#define Actuator2_HOOK_Pin GPIO_PIN_9
#define Actuator2_HOOK_GPIO_Port GPIOA
#define Right_pulse_B_Pin GPIO_PIN_11
#define Right_pulse_B_GPIO_Port GPIOA
#define Right_pulse_B_EXTI_IRQn EXTI15_10_IRQn
#define Right_pulse_A_Pin GPIO_PIN_12
#define Right_pulse_A_GPIO_Port GPIOA
#define Right_pulse_A_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define RAZER_POWER1_Pin GPIO_PIN_8
#define RAZER_POWER1_GPIO_Port GPIOB
#define RAZER_POWER2_Pin GPIO_PIN_9
#define RAZER_POWER2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
