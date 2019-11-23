#ifndef STM32_LIGHTING_CONTROLLER_DMXTASK_H
#define STM32_LIGHTING_CONTROLLER_DMXTASK_H

#define DMXTASK_BREAK_BIT 0x01U
#define DMXTASK_TXCOMPLETE_BIT 0x02U

void dmxTask( void *pvParameters );

#endif //STM32_LIGHTING_CONTROLLER_DMXTASK_H
