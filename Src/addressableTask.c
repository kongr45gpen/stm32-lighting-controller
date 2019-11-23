#include "FreeRTOS.h"
#include <task.h>
#include "addressableTask.h"
#include "stdint.h"

#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_tim.h"


static uint16_t ledCount = 2;
static uint8_t ledColours = 4;
static uint8_t bitDepth = 8;

#define ADDRESSABLE_LEDS_MAX 1000
static uint8_t __attribute__((section (".sram"))) addressableValues[ADDRESSABLE_LEDS_MAX] = { 10, 35, 99, 13, 53, 12, 128, 190, 150, 20 };

/**
 * A task responsible for addressable LED strips
 * @param pvParameters
 */
void addressableTask(void *pvParameters) {
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_1, (uint32_t) addressableValues, // Memory address of the buffer
                           (uint32_t) (&(TIM15->CCR1)),
                           LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // Send message from memory to the USART Data Register
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, ledCount * ledColours); // Set amount of copied bits for DMA

    LL_TIM_EnableDMAReq_CC1(TIM15);              /* Enable DMA requests on channel 1 */
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
    LL_TIM_CC_EnableChannel(TIM15, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(TIM15);

    while(1) {
        vTaskDelay(2);
    }
}
