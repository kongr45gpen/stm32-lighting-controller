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

    static const int wings = 11;
    int i, j;
    for (i = 0; i < DMX_MAX / wings; i++) { // For each wing

       for (j = 0; j < wings; j++) { // For each light in the wing
           // Calculate the index of the current light
           int index = j * DMX_MAX / wings + i;

           if (index - 1 >= 0) {
               // Black out the previous light, if it exists
               universe[index - 1] = 0;
           }

           if (index < DMX_MAX) {
               // Highlight the current light, if it exists
               universe[index] = 255;
           }
       }

        notifyUniverseUpdate();

        vTaskDelay(pdMS_TO_TICKS(1000 / 10)); // Refresh 10 times per second
    }

    temporaryBlackout();

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
