/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pwmTask.h"
#include "dmxTask.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_usart.h"
#include "displayTask.h"
#include <serialTask.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_tim15_ch1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim16;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (LL_DMA_IsActiveFlag_TC0(DMA1)) {
        // The DMA has been triggered. That means that the DMX packet has been sent completely.
        // Notify the corresponding task
        xTaskNotifyFromISR(dmxTaskHandle, DMXTASK_TXCOMPLETE_BIT, eSetBits, &xHigherPriorityTaskWoken);

        // Clear the TC flag (since we didn't do any HAL shenanigans, the DMA IRQ Handler will not do that)
        LL_DMA_ClearFlag_TC0(DMA1);
    }
  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim15_ch1);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
    // DMX finished event
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(xPwmEventGroupHandle, PWMTASK_TIM_BIT, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
    uint32_t isrflags   = READ_REG(USART3->ISR);
    uint32_t errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE | USART_ISR_RTOF));

    if (0 != errorflags) {
        // An error occurred.
        if (LL_USART_IsActiveFlag_ORE(USART3)) {
            // Overrun occurred
            USART3_OERHandler();
        } else {
            // Ignore other errors
        }

        // Clear all flags
        USART3->ICR = 0xff;
    } else {
        USART3_RXHandler();
    }
  /* USER CODE END USART3_IRQn 0 */
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
  // Whether the button was pressed instead of de-pressed
  uint8_t buttonWasPressed = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (buttonWasPressed) {
      xEventGroupSetBitsFromISR(xButtonEventGroupHandle, DISPLAYTASK_PRESSED_BIT, &xHigherPriorityTaskWoken);
  } else {
      xEventGroupSetBitsFromISR(xButtonEventGroupHandle, DISPLAYTASK_RELEASED_BIT, &xHigherPriorityTaskWoken);
  }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */
  // A variable that shows the state of the counter
    static enum state {
        afterBreak, afterMAB
    } state = afterBreak;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Update event received
    if (__HAL_TIM_GET_IT_SOURCE(&htim16, TIM_IT_UPDATE) != RESET) {
        // The timer was called. This means we stopped counting
        if (state == afterBreak) {
            // The break pulse is over. Now we can send the MAB pulse
            __HAL_TIM_SET_AUTORELOAD(&htim16, 12 - 1); // pulse duration is 12 us
            // Set the pin to HIGH
            LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_5);
            // Set the correct state
            state = afterMAB;
            // Start counting again (the timer is one-shot)
            HAL_TIM_Base_Start_IT(&htim16);
        } else if (state == afterMAB) {
            // The MAB pulse is over. Reset the timer to the duration of the break pulse
            __HAL_TIM_SET_AUTORELOAD(&htim16, 100 - 1); // pulse duration is 100 us
            // Set the pin to its alternate function
            LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
            // Set the correct state, so that we're ready for the next transition
            state = afterBreak;

            // Send a notification to the DMX task so it can do its thing
            xTaskNotifyFromISR(dmxTaskHandle, DMXTASK_BREAK_BIT, eSetBits, &xHigherPriorityTaskWoken);
        }
    }

  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);


  /* USER CODE END TIM16_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
