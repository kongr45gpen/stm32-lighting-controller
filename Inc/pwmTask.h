#ifndef STM32_LIGHTING_CONTROLLER_PWMTASK_H
#define STM32_LIGHTING_CONTROLLER_PWMTASK_H

#define PWMTASK_TIM_BIT     0x01U
#define PWMTASK_UPDATE_BIT  0x02U

void pwmTask( void *pvParameters );

#endif //STM32_LIGHTING_CONTROLLER_PWMTASK_H
