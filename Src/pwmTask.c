#include "FreeRTOS.h"
#include <task.h>
#include <stdlib.h>
#include <universe.h>
#include "pwmTask.h"
#include "main.h"
#include "stm32h7xx.h"

// The exponential gamma 2factor that defines how non-linear the lighting curve is.
// A value of 0 means linear change
const float gam = 3.0f;

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

    const uint16_t period = 4096; // 12 bits of resolution have proven to look good enough
    const uint16_t prescaler = 500;

    for (int i = 0; i < 4; i++) {
        // Set the settings of the timer based on the global settings
        usedTimers[i]->Init.Prescaler = prescaler;
        usedTimers[i]->Init.Period = period;

        // Apply the new settings
        HAL_TIM_PWM_Init(usedTimers[i]);

        // Start the timer itself
        HAL_TIM_Base_Start_IT(usedTimers[i]);

        // Start all the PWM channels
        HAL_TIM_PWM_Start(usedTimers[i], TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(usedTimers[i], TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(usedTimers[i], TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(usedTimers[i], TIM_CHANNEL_4);
    }

    while (1) {
        // Precalculate (e^gam - 1) for later
        float exp_gam = expf(gam) - 1;

        // Set all the values from the registers
        for (int i = 0; i < 16; i++) {
            // Get the original 8-bit value, in a 0-1 range
            float initialValue = universe[i] / 255.0;

            // Normalise the value by means of an exponential curve
            float normalised = (gam == 0) ? initialValue
                    : ( expf(gam * initialValue) - 1) / exp_gam;

            // Convert the [0,1] value into a 12-bit integer
            (*(pwmChannels[i])) = normalised * 4095;
        }

        // Wait until the next timer interrupt
        xEventGroupWaitBits(xPwmEventGroupHandle, PWMTASK_TIM_BIT | PWMTASK_UPDATE_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    }
}
