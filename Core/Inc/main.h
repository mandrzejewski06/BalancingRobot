/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
typedef enum bool {false = 0, true = 1} bool;
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

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define leftMotor_M1_Pin GPIO_PIN_5
#define leftMotor_M1_GPIO_Port GPIOA
#define leftMotor_M0_Pin GPIO_PIN_6
#define leftMotor_M0_GPIO_Port GPIOA
#define leftMotor_STEP_Pin GPIO_PIN_10
#define leftMotor_STEP_GPIO_Port GPIOB
#define leftMotor_DIR_Pin GPIO_PIN_8
#define leftMotor_DIR_GPIO_Port GPIOA
#define rightMotor_DIR_Pin GPIO_PIN_9
#define rightMotor_DIR_GPIO_Port GPIOA
#define LSM6_Interrupt_Pin GPIO_PIN_10
#define LSM6_Interrupt_GPIO_Port GPIOA
#define LSM6_Interrupt_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LSM6_Interrupt2_Pin GPIO_PIN_3
#define LSM6_Interrupt2_GPIO_Port GPIOB
#define LSM6_Interrupt2_EXTI_IRQn EXTI3_IRQn
#define rightMotor_M0_Pin GPIO_PIN_4
#define rightMotor_M0_GPIO_Port GPIOB
#define rightMotor_M1_Pin GPIO_PIN_5
#define rightMotor_M1_GPIO_Port GPIOB
#define rightMotor_STEP_Pin GPIO_PIN_6
#define rightMotor_STEP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
