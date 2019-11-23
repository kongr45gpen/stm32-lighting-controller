#include <main.h>
#include <limits.h>
#include <stdlib.h>
#include "dmxTask.h"

#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_gpio.h"

#include "FreeRTOS.h"
#include "task.h"

/**
 * A task responsible for the DMX UART output based on the universe
 * @param pvParameters
 */
void dmxTask(void *pvParameters) {
    while(1) {
        // Start the timer counting for DMX pulses
        HAL_TIM_Base_Start_IT(&htim16);

        LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
        LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_5);

        xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY);

        while(!LL_USART_IsActiveFlag_TC(USART2)) {}
        LL_USART_TransmitData8(USART2, 0x00);

        uint32_t ra;
        HAL_RNG_GenerateRandomNumber(&hrng, &ra);
        while(!LL_USART_IsActiveFlag_TC(USART2)) {}
        LL_USART_TransmitData8(USART2, ra % 255);

        while(!LL_USART_IsActiveFlag_TC(USART2)) {}
        LL_USART_TransmitData8(USART2, 0x00);

        while(!LL_USART_IsActiveFlag_TC(USART2)) {}
        LL_USART_TransmitData8(USART2, 0xff);

        for (int i = 0; i < 32; i ++) {
            while(!LL_USART_IsActiveFlag_TC(USART2)) {}
            LL_USART_TransmitData8(USART2, 0x00);
        }

        while(!LL_USART_IsActiveFlag_TC(USART2)) {}

        vTaskDelay(1);
    }
}
