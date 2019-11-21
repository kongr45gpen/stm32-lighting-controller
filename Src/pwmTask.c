#include "FreeRTOS.h"
#include <task.h>
#include <stdlib.h>
#include "pwmTask.h"
#include "main.h"
#include "stm32h7xx.h"

TIM_TypeDef * pwmTimers[16] = {
    TIM1, TIM1, TIM1, TIM1,
    TIM2, TIM2, TIM2, TIM2,
    TIM3, TIM3, TIM3, TIM3,
    TIM4, TIM4, TIM4, TIM4
};

uint32_t pwmChannels[16] = {
        TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
        TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
        TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
        TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
};

/**
 * A task responsible for all PWM outputs of the board
 */
void pwmTask( void *pvParameters ) {
    static TIM_HandleTypeDef * usedTimers[4] = {
            &htim1, &htim2, &htim3, &htim4
    };

    const uint16_t period = 4096;
    const uint16_t prescaler = 50;

    for (int i = 0; i < 4; i++) {
        // Set the settings of the timer based on the global settings
        usedTimers[i]->Init.Prescaler = prescaler;
        usedTimers[i]->Init.Period = period;

        // Apply the new settings
        HAL_TIM_PWM_Init(usedTimers[i]);

        // Start the timer itself
        HAL_TIM_Base_Start(usedTimers[i]);

        // Start all the PWM channels
        HAL_TIM_PWM_Start(usedTimers[i], TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(usedTimers[i], TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(usedTimers[i], TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(usedTimers[i], TIM_CHANNEL_4);
    }

    TIM2->CCR1 = 50;
    TIM2->CCR3 = 50;
    TIM5->CCR1 = 500;
//    TIM5->CCER |= TIM_CCER_CC1E;


    while (1) {
        TIM2->CCR3 = (TIM2->CCR3 + 1) % period;
        vTaskDelay(1);
    }
}
