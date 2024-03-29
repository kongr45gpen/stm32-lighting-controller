/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include <event_groups.h>
#include <semphr.h>
#include <stream_buffer.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// HAL peripheral definitions
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern RNG_HandleTypeDef hrng;
extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi3;

// FreeRTOS task handles
extern xTaskHandle pwmTaskHandle;
extern xTaskHandle dmxTaskHandle;

// FreeRTOS resource handles
extern EventGroupHandle_t xPwmEventGroupHandle;
extern EventGroupHandle_t xButtonEventGroupHandle;
extern EventGroupHandle_t xSerialEventGroupHandle;
extern StreamBufferHandle_t xSerialReceiveBufferHandle;
extern QueueHandle_t xErrorQueueHandle;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define ERROR_MESSAGE_SIZE 256
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
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define USER_Btn_EXTI_IRQn EXTI15_10_IRQn
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define OLED_2_Pin GPIO_PIN_12
#define OLED_2_GPIO_Port GPIOF
#define IGNORE_Pin GPIO_PIN_12
#define IGNORE_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define USB_TX_Pin GPIO_PIN_8
#define USB_TX_GPIO_Port GPIOD
#define USB_RX_Pin GPIO_PIN_9
#define USB_RX_GPIO_Port GPIOD
#define OLED_1_Pin GPIO_PIN_15
#define OLED_1_GPIO_Port GPIOD
#define NRF24_CE_Pin GPIO_PIN_3
#define NRF24_CE_GPIO_Port GPIOG
#define NRF24_CSN_Pin GPIO_PIN_2
#define NRF24_CSN_GPIO_Port GPIOD
#define DMX_OUT_Pin GPIO_PIN_5
#define DMX_OUT_GPIO_Port GPIOD
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
