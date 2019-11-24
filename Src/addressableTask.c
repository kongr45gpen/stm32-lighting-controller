#include "FreeRTOS.h"
#include <task.h>
#include "addressableTask.h"
#include "stdint.h"
#include "string.h"

#include "stm32h7xx_ll_mdma.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_tim.h"


static uint16_t ledCount = 2;
static uint8_t ledColours = 4;
static uint8_t bitDepth = 8;

#define ADDRESSABLE_LEDS_MAX 1000
#define EMPTY_SIZE 80 // Number of low pulses to be sent
#define DATA_SIZE (EMPTY_SIZE + ADDRESSABLE_LEDS_MAX)
static uint16_t __attribute__((section (".sram"))) addressableValues[DATA_SIZE] = { 0 };

/**
 * A task responsible for addressable LED strips based on the WS2812B chip
 * @param pvParameters
 */
void addressableTask(void *pvParameters) {
    memset(addressableValues, 0, DATA_SIZE * sizeof(uint16_t));

    TIM15->CCR1 = 0;
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_1, (uint32_t) addressableValues, // Memory address of the buffer
                           ((uint32_t) (&(TIM15->CCR1))),
                           LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // Send message from memory to the USART Data Register
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, DATA_SIZE); // Set amount of copied bits for DMA

    LL_TIM_EnableDMAReq_CC1(TIM15);              /* Enable DMA requests on channel 1 */
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
    LL_TIM_CC_EnableChannel(TIM15, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(TIM15);
    LL_TIM_EnableAllOutputs(TIM15);

    addressableValues[EMPTY_SIZE + 0] = 1;
    addressableValues[EMPTY_SIZE + 1] = 1;
    addressableValues[EMPTY_SIZE + 2] = 1;
    addressableValues[EMPTY_SIZE + 3] = 1;
    addressableValues[EMPTY_SIZE + 4] = 2;
    addressableValues[EMPTY_SIZE + 5] = 2;
    addressableValues[EMPTY_SIZE + 6] = 2;
    addressableValues[EMPTY_SIZE + 7] = 2;

    addressableValues[EMPTY_SIZE + 8 + 0] = 2;
    addressableValues[EMPTY_SIZE + 8 + 1] = 2;
    addressableValues[EMPTY_SIZE + 8 + 2] = 2;
    addressableValues[EMPTY_SIZE + 8 + 3] = 2;
    addressableValues[EMPTY_SIZE + 8 + 4] = 2;
    addressableValues[EMPTY_SIZE + 8 + 5] = 2;
    addressableValues[EMPTY_SIZE + 8 + 6] = 2;
    addressableValues[EMPTY_SIZE + 8 + 7] = 2;

    addressableValues[EMPTY_SIZE + 16 + 0] = 1;
    addressableValues[EMPTY_SIZE + 16 + 1] = 1;
    addressableValues[EMPTY_SIZE + 16 + 2] = 1;
    addressableValues[EMPTY_SIZE + 16 + 3] = 1;
    addressableValues[EMPTY_SIZE + 16 + 4] = 1;
    addressableValues[EMPTY_SIZE + 16 + 5] = 1;
    addressableValues[EMPTY_SIZE + 16 + 6] = 1;
    addressableValues[EMPTY_SIZE + 16 + 7] = 2;

    addressableValues[EMPTY_SIZE + 24 + 0] = 2;
    addressableValues[EMPTY_SIZE + 24 + 1] = 1;
    addressableValues[EMPTY_SIZE + 24 + 2] = 1;
    addressableValues[EMPTY_SIZE + 24 + 3] = 1;
    addressableValues[EMPTY_SIZE + 24 + 4] = 1;
    addressableValues[EMPTY_SIZE + 24 + 5] = 1;
    addressableValues[EMPTY_SIZE + 24 + 6] = 1;
    addressableValues[EMPTY_SIZE + 24 + 7] = 1;

    addressableValues[EMPTY_SIZE + 32 + 0] = 2;
    addressableValues[EMPTY_SIZE + 32 + 1] = 1;
    addressableValues[EMPTY_SIZE + 32 + 2] = 1;
    addressableValues[EMPTY_SIZE + 32 + 3] = 1;
    addressableValues[EMPTY_SIZE + 32 + 4] = 1;
    addressableValues[EMPTY_SIZE + 32 + 5] = 1;
    addressableValues[EMPTY_SIZE + 32 + 6] = 1;
    addressableValues[EMPTY_SIZE + 32 + 7] = 1;

    while(1) {
        vTaskDelay(2);
    }
}
