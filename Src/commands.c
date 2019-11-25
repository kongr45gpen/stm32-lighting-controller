/**
 * Commands & their implementations that can be ran from the screen manually, or by serial RX
 */

#include <stdbool.h>
#include "FreeRTOS.h"
#include <task.h>
#include <universe.h>
#include <displayTask.h>
#include "commands.h"

// Assume booleans are atomic
static bool testRunning = false;

void testTask(void * pvParameters) {
    // Initialisation
    testRunning = true;
    universeIsWritable = false;

    // First, black out all channels
    temporaryBlackout();

    int i;
    for (i = 0; i < 100; i++) {
        // Reset previous values
        if (i >= 2) universe[i - 2] = 0;
        if (i >= 1) universe[i - 1] = 0;

        // Make current values full
        universe[i] = 255;
        if (i + 1 < DMX_MAX) {
            universe[i + 1] = 255;
        }

        vTaskDelay(pdMS_TO_TICKS(1000 / 30)); // Try to get close to the DMX refresh rate (we have 30 fps)
    }

    // Reset the two final values
    if (i - 1 < DMX_MAX) universe[i - 1] = 0;
    if (i < DMX_MAX) universe[i] = 0;

    // Cleanup
    testRunning = false;
    universeIsWritable = true;
    displayModeReset(); // Reset the mode shown on the screen
    vTaskDelete(NULL); // Delete myself
}

/**
 * Test all DMX outputs
 */
void CommandTest() {
    if (!testRunning) { // Only create a test task if one is not already running
        xTaskCreate(testTask, "test", 200, NULL, 1, NULL);
    }
}

/**
 * Permanently black out the universe of lights. A restart will be required to un-blackout
 */
void CommandBlackout() {
    temporaryBlackout();
    universeIsWritable = false;
}

/**
 * Reset the microcontroller via software
 */
void CommandReset() {
    HAL_NVIC_SystemReset();
}
