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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BAT_LEV_Pin GPIO_PIN_0
#define BAT_LEV_GPIO_Port GPIOA
#define ACC_X_Pin GPIO_PIN_1
#define ACC_X_GPIO_Port GPIOA
#define ACC_Y_Pin GPIO_PIN_2
#define ACC_Y_GPIO_Port GPIOA
#define ACC_Z_Pin GPIO_PIN_3
#define ACC_Z_GPIO_Port GPIOA
#define CS0_Pin GPIO_PIN_4
#define CS0_GPIO_Port GPIOC
#define CS1_Pin GPIO_PIN_5
#define CS1_GPIO_Port GPIOC
#define DRDY_Pin GPIO_PIN_0
#define DRDY_GPIO_Port GPIOB
#define DRDY_EXTI_IRQn EXTI0_IRQn
#define START_ADC_Pin GPIO_PIN_1
#define START_ADC_GPIO_Port GPIOB
#define RESET_ADC_Pin GPIO_PIN_2
#define RESET_ADC_GPIO_Port GPIOB
#define P1_2_Pin GPIO_PIN_12
#define P1_2_GPIO_Port GPIOB
#define P1_3_Pin GPIO_PIN_15
#define P1_3_GPIO_Port GPIOB
#define P1_7_Pin GPIO_PIN_8
#define P1_7_GPIO_Port GPIOD
#define P1_6_Pin GPIO_PIN_9
#define P1_6_GPIO_Port GPIOD
#define P0_2_Pin GPIO_PIN_10
#define P0_2_GPIO_Port GPIOD
#define P2_7_Pin GPIO_PIN_11
#define P2_7_GPIO_Port GPIOD
#define SWITCH_Pin GPIO_PIN_12
#define SWITCH_GPIO_Port GPIOD
#define CARD_PR_Pin GPIO_PIN_10
#define CARD_PR_GPIO_Port GPIOA
#define CHRG_STAT_Pin GPIO_PIN_1
#define CHRG_STAT_GPIO_Port GPIOD
#define LED_W_Pin GPIO_PIN_5
#define LED_W_GPIO_Port GPIOD
#define LED_B_Pin GPIO_PIN_6
#define LED_B_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_7
#define LED_R_GPIO_Port GPIOD
#define BOOT_STIM_Pin GPIO_PIN_4
#define BOOT_STIM_GPIO_Port GPIOB
#define RESET_STIM_Pin GPIO_PIN_5
#define RESET_STIM_GPIO_Port GPIOB
#define TRIG_OUT_Pin GPIO_PIN_0
#define TRIG_OUT_GPIO_Port GPIOE
#define TRIG_IN_Pin GPIO_PIN_1
#define TRIG_IN_GPIO_Port GPIOE
#define TRIG_IN_EXTI_IRQn EXTI1_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
