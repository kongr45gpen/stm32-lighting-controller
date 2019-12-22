#include "FreeRTOS.h"
#include <task.h>
#include <universe.h>
#include "addressableTask.h"
#include "stdint.h"
#include "string.h"
#include "stdbool.h"

#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_tim.h"

// Some variables for led strip details that are modifiable by end users
static uint16_t ledCount = 8;
static uint16_t blockSize = 1;
static uint8_t ledColours = 3;
static const uint8_t bitDepth = 8;

#define ADDRESSABLE_LEDS_MAX 5500 // The maximum number of values in the array. Keep flexible for manual assignment.
#define EMPTY_SIZE 80 // Number of low pulses to be sent
#define DATA_SIZE (EMPTY_SIZE + ADDRESSABLE_LEDS_MAX)

// The DMA values need to be stored on the SRAM memory, as the default DTCM memory is not connected to the DMA peripheral
static uint16_t __attribute__((section (".sram"))) addressableValues[DATA_SIZE] = { 0 };

/**
 * A function that gets the values from the light universe, and converts them to DMA-acceptable PWM
 */
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

        uint8_t datum; // The data as declared in the universe
        if (blockSize <= 1) {
            // If we have individual access to the LEDS, just use the universe value
            datum = universe[i];
        } else {
            uint16_t colour = i % ledColours;
            uint16_t block = i / ledColours / blockSize;
            datum = universe[colour + block * ledColours];
        }

        for (uint8_t bit = 0; bit < bitDepth; bit++) {
            // Get the bit'th bit, and store 1 or 2 according to whether it's 0 or 1
            addressableValues[index + bit] = ((datum >> (bitDepth - bit - 1U)) & 0x01U) ? 2 : 1;
        }
    }
}


/**
 * A task responsible for addressable LED strips based on the WS2812B or SK6812 chip
 * @param pvParameters
 */
void addressableTask(void *pvParameters) {
    // Reset all the values to 0. The first empty bytes should be 0, and we don't want any random leftovers.
    memset(addressableValues, 0, DATA_SIZE * sizeof(uint16_t));

    // Reset the DMX register of the timer
    TIM15->CCR1 = 0;

    // Initial DMA configuration
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_1, (uint32_t) addressableValues, // Memory address of the buffer
                           ((uint32_t) (&(TIM15->CCR1))),
                           LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // Send message from memory to the USART Data Register
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, DATA_SIZE); // Set amount of copied bits for DMA

    // Enable the DMA request. Here, we can just leave the DMA request running, as a circular buffer is used. No need
    // to handle interrupts and Transaction Copmlete events.
    LL_TIM_EnableDMAReq_CC1(TIM15);              /* Enable DMA requests on channel 1 */
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);

    // Enable the timer counter, PWM and output. The pin now outputs the requested PWM.
    LL_TIM_CC_EnableChannel(TIM15, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(TIM15);
    LL_TIM_EnableAllOutputs(TIM15);

    // Some time for the data to settle
    vTaskDelay(5);

    // Store the last wake time for the delay
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1) {
        // Delay until a refresh is needed
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / 45)); // 45 fps refresh rate

        // Update the values and store them in memory. The DMA will access those values on its own, no need to do
        // anything on our part - just keep them stored!
        parseUniverse();
    }
}
