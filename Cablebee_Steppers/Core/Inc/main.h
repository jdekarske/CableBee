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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define XSTOP_Pin GPIO_PIN_0
#define XSTOP_GPIO_Port GPIOC
#define YSTOP_Pin GPIO_PIN_1
#define YSTOP_GPIO_Port GPIOC
#define ZSTOP_Pin GPIO_PIN_2
#define ZSTOP_GPIO_Port GPIOC
#define ZDIR_Pin GPIO_PIN_5
#define ZDIR_GPIO_Port GPIOC
#define ZSTP_Pin GPIO_PIN_0
#define ZSTP_GPIO_Port GPIOB
#define ZEN_Pin GPIO_PIN_1
#define ZEN_GPIO_Port GPIOB
#define YDIR_Pin GPIO_PIN_2
#define YDIR_GPIO_Port GPIOB
#define YSTP_Pin GPIO_PIN_10
#define YSTP_GPIO_Port GPIOB
#define YEN_Pin GPIO_PIN_11
#define YEN_GPIO_Port GPIOB
#define XDIR_Pin GPIO_PIN_12
#define XDIR_GPIO_Port GPIOB
#define XSTP_Pin GPIO_PIN_13
#define XSTP_GPIO_Port GPIOB
#define XEN_Pin GPIO_PIN_14
#define XEN_GPIO_Port GPIOB
#define E0EN_Pin GPIO_PIN_2
#define E0EN_GPIO_Port GPIOD
#define E0STP_Pin GPIO_PIN_3
#define E0STP_GPIO_Port GPIOB
#define E0DIR_Pin GPIO_PIN_4
#define E0DIR_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
