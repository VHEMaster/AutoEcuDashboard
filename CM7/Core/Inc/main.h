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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BL_CTRL_Pin GPIO_PIN_5
#define BL_CTRL_GPIO_Port GPIOK
#define BL_EN_Pin GPIO_PIN_8
#define BL_EN_GPIO_Port GPIOF
#define DSI_EN_Pin GPIO_PIN_2
#define DSI_EN_GPIO_Port GPIOH
#define DSI_IRQ_Pin GPIO_PIN_3
#define DSI_IRQ_GPIO_Port GPIOH
#define FLEXIO_WAKE_Pin GPIO_PIN_4
#define FLEXIO_WAKE_GPIO_Port GPIOC
#define FLEXIO_INT_Pin GPIO_PIN_1
#define FLEXIO_INT_GPIO_Port GPIOB
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define FLEXIO_NRST_Pin GPIO_PIN_5
#define FLEXIO_NRST_GPIO_Port GPIOC
#define FLEXIO_SYNC_Pin GPIO_PIN_0
#define FLEXIO_SYNC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
