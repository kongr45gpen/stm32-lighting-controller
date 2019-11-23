#ifndef STM32_LIGHTING_CONTROLLER_DISPLAYTASK_H
#define STM32_LIGHTING_CONTROLLER_DISPLAYTASK_H

#define DISPLAYTASK_PRESSED_BIT  0x01U
#define DISPLAYTASK_RELEASED_BIT 0x02U

enum ModeDisplay {
    eModeReady = 0,
    eModeCheck = 1,
    eModeSerial,
    eModeWireless,
    eModeBlackout,
    eModeFlash
};

extern enum ModeDisplay modeDisplay;

void displayTask(void * pvParameters);

#endif //STM32_LIGHTING_CONTROLLER_DISPLAYTASK_H
