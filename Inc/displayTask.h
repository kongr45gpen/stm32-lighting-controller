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

/**
 * Show a new error message to the screen
 * @param message A string that contains the error description to be displayed. It MUST be ERROR_MESSAGE_SIZE bytes
 * long. The null byte \0 is parsed as per C convention.
 * @param xTicksToWait The number of ticks to wait before the queue is empty, as per FreeRTOS conventions. Default to
 * 0 for no delay
 * @return Whether there was enough space to include this message instead of the message being discarded
 */
BaseType_t addErrorMessage(const char* message, TickType_t xTicksToWait);

#endif //STM32_LIGHTING_CONTROLLER_DISPLAYTASK_H
