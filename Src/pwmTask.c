#include "FreeRTOS.h"
#include <task.h>
#include <stdlib.h>
#include <universe.h>
#include "pwmTask.h"
#include "main.h"
#include "stm32h7xx.h"

TIM_TypeDef * pwmTimers[16] = {
    TIM1, TIM1, TIM1, TIM1,
    TIM2, TIM2, TIM2, TIM2,
    TIM3, TIM3, TIM3, TIM3,
    TIM4, TIM4, TIM4, TIM4
};

volatile uint32_t * pwmChannels[16] = {
        &(TIM1->CCR1), &(TIM1->CCR2), &(TIM1->CCR3), &(TIM1->CCR4),
        &(TIM2->CCR1), &(TIM2->CCR2), &(TIM2->CCR3), &(TIM2->CCR4),
        &(TIM3->CCR1), &(TIM3->CCR2), &(TIM3->CCR3), &(TIM3->CCR4),
        &(TIM4->CCR1), &(TIM4->CCR2), &(TIM4->CCR3), &(TIM4->CCR4),
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

    while (1) {
        // Set all the values from the registers
        for (int i = 0; i < 16; i++) {
            (*(pwmChannels[i])) = universe[i] * 16UL;
        }

        // Wait until the next timer interrupt
        ulTaskNotifyTake(pdTRUE, 10);
    }
}
