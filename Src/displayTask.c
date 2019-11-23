#include "FreeRTOS.h"
#include <task.h>
#include <string.h>
#include <main.h>
#include "displayTask.h"
#include "ssd1306.h"

#define RECT_0_X 0
#define RECT_0_Y 16
#define RECT_1_X 80
#define RECT_1_Y 63

char *functionNames[] = {
        "Check",
        "Reset",
        "B/O",
};
const uint8_t functionCount = sizeof(functionNames) / sizeof(functionNames[0]);

/**
 * A function that draws all the structural elements of the screen
 */
static void redraw() {
    ssd1306_Fill(Black);

    // Screen structure initialisation
    ssd1306_SetCursor(1, 0);
    ssd1306_WriteString("silight 0.1", Font_7x10, White);

    // Draw the rectangle around the DMX values
    for (int i = RECT_0_X; i <= RECT_1_X; i++) {
        // Vertical lines
        ssd1306_DrawPixel(i, RECT_0_Y, White);
        ssd1306_DrawPixel(i, RECT_1_Y, White);
    }
    for (int i = RECT_0_Y; i <= RECT_1_Y; i++) {
        // Horizontal lines
        ssd1306_DrawPixel(RECT_0_X, i, White);
        ssd1306_DrawPixel(RECT_1_X, i, White);
    }
}

void displayTask(void *pvParameters) {
    ssd1306_Init();

    redraw();

    static uint8_t currentFunction = 0;

    while (1) {
        // First, check if the button was pressed
        if (0 != (DISPLAYTASK_PRESSED_BIT & xEventGroupWaitBits(xButtonEventGroupHandle, DISPLAYTASK_PRESSED_BIT, 0x0, pdFALSE, 0))) {
            // The button was pressed. Wait some time for debouncing
            vTaskDelay(10);

            // Reset the event bits in case of spurious press/depress
            xEventGroupClearBits(xButtonEventGroupHandle, 0xff);

            // Wait until the button is released
            if (0 != (DISPLAYTASK_RELEASED_BIT &
                      xEventGroupWaitBits(xButtonEventGroupHandle, DISPLAYTASK_RELEASED_BIT, 0x0, pdFALSE,
                                          pdMS_TO_TICKS(200)))) {
                // The button was released within a short amount of time, just move to the next function
                currentFunction = (currentFunction + 1) % functionCount;
            } else {
                // The button was not released. The user is pressing it for a longer amount of time.
                if (0 != (DISPLAYTASK_RELEASED_BIT &
                          xEventGroupWaitBits(xButtonEventGroupHandle, DISPLAYTASK_RELEASED_BIT, 0x0, pdFALSE,
                                              pdMS_TO_TICKS(2000)))) {
                    // The user has pressed the button for a long time. Perform the function
                    currentFunction = (currentFunction - 1 + functionCount) % functionCount;
                } else {
                    // The user pressed the button for too long. Assume an error and do nothing.
                }
            }

            // Wait for debouncing to complete
            vTaskDelay(10);
            // Reset the event bits
            xEventGroupClearBits(xButtonEventGroupHandle, 0xff);
        }

        // Status string
        ssd1306_SetCursor(92, 0); // top right corner
        ssd1306_WriteString("RDY", Font_11x18, White);

        ssd1306_SetCursor(2, 50);
        if (xTaskGetTickCount() % 3 < 1) {
            ssd1306_WriteChar('>', Font_7x10, White);
        } else {
            ssd1306_WriteChar(' ', Font_7x10, White);
        }

        // Draw the 3 current function names
        uint16_t currentY = 20;
        const uint16_t currentX = 88;
        const uint16_t width = 40;
        const uint16_t height = 10;
        size_t functionLength;
        for (int i = -1; i <= 1; i++) { // For each function
            uint8_t iAbsolute = (i + currentFunction + functionCount) % functionCount;
            functionLength = strlen(functionNames[iAbsolute]);

            ssd1306_SetCursor(currentX, currentY);

            // Draw the selection background
            for (int j = currentX; j < currentX + width; j++) {
                for (int k = currentY - 1; k < currentY + height; k++) {
                    if (i == 0) {
                        // The current option should be highlighted
                        ssd1306_DrawPixel(j, k, White);
                    } else {
                        // Paint the other options black, to amount for any remaining text
                        ssd1306_DrawPixel(j, k, Black);
                    }
                }
            }

            ssd1306_SetCursor(currentX + (width - 7 * functionLength) / 2, currentY);

            // Draw the name of the function
            ssd1306_WriteString(functionNames[iAbsolute], Font_7x10, i == 0 ? Black : White);

            currentY += 14;
        }

        ssd1306_UpdateScreen();

        vTaskDelay(10);
    }
}


