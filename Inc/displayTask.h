#ifndef STM32_LIGHTING_CONTROLLER_DISPLAYTASK_H
#define STM32_LIGHTING_CONTROLLER_DISPLAYTASK_H

#define DISPLAYTASK_PRESSED_BIT  0x01U
#define DISPLAYTASK_RELEASED_BIT 0x02U
#define DISPLAYTASK_PRESSED_RELEASED_BITS (DISPLAYTASK_PRESSED_BIT | DISPLAYTASK_RELEASED_BIT)

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

/**
 * Reset the currently displayed mode on the screen
 */
static inline void displayModeReset() {
    modeDisplay = eModeReady;
}

/**
 * Set the currently displayed mode on the screen to a new value
 * @param newDisplayMode
 */
static inline void displayModeSet(enum ModeDisplay newDisplayMode) {
    modeDisplay = newDisplayMode;
}

#endif //STM32_LIGHTING_CONTROLLER_DISPLAYTASK_H
