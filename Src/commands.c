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

    int i;
    for (i = 0; i < 100; i++) {
        // Reset previous values
        if (i >= 2) universe[i - 2] = 0;
        if (i >= 1) universe[i - 1] = 0;

        // Make current values full
        universe[i] = 255;
        if (i + 1 < 512) {
            universe[i + 1] = 255;
        }

        vTaskDelay(pdMS_TO_TICKS(1000 / 30)); // Try to get close to the DMX refresh rate (we have 30 fps)
    }

    // Reset the two final values
    if (i - 1 < 512) universe[i - 1] = 0;
    if (i < 512) universe[i] = 0;

    // Cleanup
    testRunning = false;
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
