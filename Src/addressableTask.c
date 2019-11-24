#include "FreeRTOS.h"
#include <task.h>
#include <universe.h>
#include "addressableTask.h"
#include "stdint.h"
#include "string.h"

#include "stm32h7xx_ll_mdma.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_tim.h"


static uint16_t ledCount = 3;
static uint8_t ledColours = 4;
static uint8_t bitDepth = 8;

#define ADDRESSABLE_LEDS_MAX 1000
#define EMPTY_SIZE 80 // Number of low pulses to be sent
#define DATA_SIZE (EMPTY_SIZE + ADDRESSABLE_LEDS_MAX)
static uint16_t __attribute__((section (".sram"))) addressableValues[DATA_SIZE] = { 0 };

void parseUniverse() {
    // We assume that the first dmaData registers are left untouched as zero
    for (size_t i = 0; i < ledCount * ledColours; i++) {
        // The index of the current LED in the dmaValues array
        uint32_t index = EMPTY_SIZE + bitDepth * i;
        // Red and green are switched, take care of that
        if (i % ledColours == 0) {
            // Red
            index += bitDepth;
        } else if (i % ledColours == 1) {
            // Green
            index -= bitDepth;
        }

        uint8_t datum = universe[i]; // The data as declared in the universe

        for (uint8_t bit = 0; bit < bitDepth; bit++) {
            // Get the bit'th bit, and store 1 or 2 according to whether it's 0 or 1
            addressableValues[index + bit] = ((datum >> (bitDepth - bit - 1U)) & 0x01U) ? 2 : 1;
        }
    }
}


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


    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000 / 45)); // 45 fps refresh rate
        parseUniverse();

    }
}
