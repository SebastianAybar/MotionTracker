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
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

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
#define INT2_Pin GPIO_PIN_2
#define INT2_GPIO_Port GPIOC
#define INT2_EXTI_IRQn EXTI2_IRQn
#define INT1_Pin GPIO_PIN_3
#define INT1_GPIO_Port GPIOC
#define INT1_EXTI_IRQn EXTI3_IRQn
#define VBAT_ADC_Pin GPIO_PIN_4
#define VBAT_ADC_GPIO_Port GPIOC
#define HSE_ENABLE_Pin GPIO_PIN_0
#define HSE_ENABLE_GPIO_Port GPIOB
#define V3_ENABLE_Pin GPIO_PIN_10
#define V3_ENABLE_GPIO_Port GPIOC
#define USB_DETECT_Pin GPIO_PIN_11
#define USB_DETECT_GPIO_Port GPIOC
#define CHARGE_ENABLE_Pin GPIO_PIN_0
#define CHARGE_ENABLE_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOD
#define VBAT_ADC_ENABLE_Pin GPIO_PIN_4
#define VBAT_ADC_ENABLE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
