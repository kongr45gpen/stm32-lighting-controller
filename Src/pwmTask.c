#include "FreeRTOS.h"
#include <task.h>
#include "pwmTask.h"
#include "main.h"
#include "stm32h7xx.h"

/**
 * A task responsible for all PWM outputs of the board
 */
void pwmTask( void *pvParameters ) {
    const uint16_t period = 4096;
    const uint16_t prescaler = 50;

    // Set the prescaler
    htim2.Init.Prescaler = prescaler;
    htim3.Init.Prescaler = prescaler;
    htim4.Init.Prescaler = prescaler;
    htim5.Init.Prescaler = prescaler;
    htim2.Init.Period = period;
    htim3.Init.Period = period;
    htim4.Init.Period = period;
    htim5.Init.Period = period;

    // Initialize the prescaler values
    HAL_TIM_PWM_Init(&htim2);
    HAL_TIM_PWM_Init(&htim3);
    HAL_TIM_PWM_Init(&htim4);
    HAL_TIM_PWM_Init(&htim5);

    // Start all the timers
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_Base_Start(&htim5);

    // Start all the PWMs
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

    TIM2->CCR2 = 50;
    TIM2->CCR3 = 50;
    TIM5->CCR1 = 500;
//    TIM5->CCER |= TIM_CCER_CC1E;

    while (1) {
        TIM5->CCR1 = (TIM5->CCR1 + 1) % period;
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
//        vTaskDelay(1);
    }
}
