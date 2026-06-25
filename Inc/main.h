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
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"

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
#define TRACK_ADC_Pin LL_GPIO_PIN_0
#define TRACK_ADC_GPIO_Port GPIOA
#define VL53_XSHUT_FRONT_Pin LL_GPIO_PIN_1
#define VL53_XSHUT_FRONT_GPIO_Port GPIOA
#define BT_TX_Pin LL_GPIO_PIN_2
#define BT_TX_GPIO_Port GPIOA
#define BT_RX_Pin LL_GPIO_PIN_3
#define BT_RX_GPIO_Port GPIOA
#define HALL_RR_Pin LL_GPIO_PIN_4
#define HALL_RR_GPIO_Port GPIOA
#define HALL_RR_EXTI_IRQn EXTI4_IRQn
#define SHOCK_Pin LL_GPIO_PIN_6
#define SHOCK_GPIO_Port GPIOA
#define SHOCK_EXTI_IRQn EXTI9_5_IRQn
#define VL53_XSHUT_LEFT_Pin LL_GPIO_PIN_7
#define VL53_XSHUT_LEFT_GPIO_Port GPIOA
#define HALL_FL_Pin LL_GPIO_PIN_0
#define HALL_FL_GPIO_Port GPIOB
#define HALL_FL_EXTI_IRQn EXTI0_IRQn
#define HALL_FR_Pin LL_GPIO_PIN_1
#define HALL_FR_GPIO_Port GPIOB
#define HALL_FR_EXTI_IRQn EXTI1_IRQn
#define HALL_RL_Pin LL_GPIO_PIN_2
#define HALL_RL_GPIO_Port GPIOB
#define HALL_RL_EXTI_IRQn EXTI2_IRQn
#define VL53_XSHUT_RIGHT_Pin LL_GPIO_PIN_8
#define VL53_XSHUT_RIGHT_GPIO_Port GPIOA
#define TMS_Pin LL_GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin LL_GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
