#include <main.h>
#include <limits.h>
#include <stdlib.h>
#include <universe.h>
#include <string.h>
#include "dmxTask.h"

#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_dma.h"

#include "FreeRTOS.h"
#include "task.h"

/**
 * A task responsible for the DMX UART output based on the universe
 * @param pvParameters
 */
void dmxTask(void *pvParameters) {
    // The dmx data to be transmitted. This is a different value from universe, to account for the status code and
    // any other modifications to the universe data
    static uint8_t __attribute__((section (".sram"))) dmxData[DMX_MAX + 1];

    // Enable the DMA Transmission Complete interrupt
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0);

    // We are ready for the next DMX sequence. Set the output pin to LOW for the break pulse
    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_5);

    // Set some global DMA parameters
    LL_USART_EnableDMAReq_TX(USART2); // Tell the USART2 peripheral to enable the DMA request
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, DMX_MAX + 1); // Set the amount of bytes to be transferred
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_0, (uint32_t) ((uint8_t*) dmxData),
                           LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_TRANSMIT),
                           LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // Set the To and From memory addresses66

    // Start the timer counting for DMX pulses
    HAL_TIM_Base_Start_IT(&htim16);

    while(1) {
        // Wait for a notification
        uint32_t pulNotificationValue = 0;
        xTaskNotifyWait(0, ULONG_MAX, &pulNotificationValue, portMAX_DELAY);

        if ((pulNotificationValue & DMXTASK_BREAK_BIT) != 0U) {
            // Break is complete. Start transmitting DMX data

            // First, copy the universe to the dmxdata, performing any necessary changes
            dmxData[0] = 0; // DMX packet type is 0, according to the spec
            memcpy(dmxData + 1, universe, DMX_MAX + 1); // Copy the rest 512 channels

            // Start the DMA transaction
            LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
        } else if ((pulNotificationValue & DMXTASK_TXCOMPLETE_BIT) != 0U) {
            // DMX transmission is complete

            // Stop the transaction, so we can start over again
            LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);

            // We are ready for the next DMX sequence. Set the output pin to LOW for the break pulse
            LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
            LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_5);

            // Start the timer counting for DMX pulses
            HAL_TIM_Base_Start_IT(&htim16);
        }
    }
}
