/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_14
#define KEY1_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_15
#define KEY2_GPIO_Port GPIOC
#define ENCODE1_1_Pin GPIO_PIN_0
#define ENCODE1_1_GPIO_Port GPIOA
#define ENCODE1_2_Pin GPIO_PIN_1
#define ENCODE1_2_GPIO_Port GPIOA
#define W_TX_Pin GPIO_PIN_2
#define W_TX_GPIO_Port GPIOA
#define W_RX_Pin GPIO_PIN_3
#define W_RX_GPIO_Port GPIOA
#define IR_Pin GPIO_PIN_6
#define IR_GPIO_Port GPIOA
#define IR_EXTI_IRQn EXTI9_5_IRQn
#define ECHO_Pin GPIO_PIN_0
#define ECHO_GPIO_Port GPIOB
#define TRIG_Pin GPIO_PIN_1
#define TRIG_GPIO_Port GPIOB
#define B_TX_Pin GPIO_PIN_10
#define B_TX_GPIO_Port GPIOB
#define B_RX_Pin GPIO_PIN_11
#define B_RX_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_12
#define AIN1_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_13
#define BIN1_GPIO_Port GPIOB
#define MPU_INT_Pin GPIO_PIN_14
#define MPU_INT_GPIO_Port GPIOB
#define MPU_INT_EXTI_IRQn EXTI15_10_IRQn
#define NRF_IRQ_Pin GPIO_PIN_15
#define NRF_IRQ_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_8
#define PWM1_GPIO_Port GPIOA
#define DBG_TX_Pin GPIO_PIN_9
#define DBG_TX_GPIO_Port GPIOA
#define DBG_RX_Pin GPIO_PIN_10
#define DBG_RX_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_11
#define PWM2_GPIO_Port GPIOA
#define NRF_CSN_Pin GPIO_PIN_12
#define NRF_CSN_GPIO_Port GPIOA
#define NRF_CE_Pin GPIO_PIN_15
#define NRF_CE_GPIO_Port GPIOA
#define ENCODE2_2_Pin GPIO_PIN_6
#define ENCODE2_2_GPIO_Port GPIOB
#define ENCODE2_1_Pin GPIO_PIN_7
#define ENCODE2_1_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
