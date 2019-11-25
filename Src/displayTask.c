#include "FreeRTOS.h"
#include <task.h>
#include <string.h>
#include <main.h>
#include <universe.h>
#include <commands.h>
#include "displayTask.h"
#include "ssd1306.h"

#define RECT_0_X 0
#define RECT_0_Y 16
#define RECT_1_X 80
#define RECT_1_Y 63
#define CHAN_X_PAD 2
#define CHAN_Y_PAD 5
#define CHAN_WIDTH 4
#define CHAN_HEIGHT 6

enum ModeDisplay modeDisplay = eModeReady;

char *functionNames[] = {
        "Check",
        "Reset",
        "B/O",
        "Flash"
};
const uint8_t functionCount = sizeof(functionNames) / sizeof(functionNames[0]);

/**
 * A function that draws all the structural elements of the screen
 */
static void redraw() {
    ssd1306_Fill(Black);

    // Screen structure initialisation
    ssd1306_SetCursor(1, 0);
    ssd1306_WriteString("artlight 0.1", Font_7x10, White);

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

    // Draw numbers for DMX channels
    ssd1306_SetCursor(RECT_0_X + 1, RECT_0_Y + 4);
    ssd1306_WriteString("01", Font_7x10, White);
    ssd1306_SetCursor(RECT_0_X + 1, RECT_0_Y + 4 + CHAN_HEIGHT + CHAN_Y_PAD);
    ssd1306_WriteString("11", Font_7x10, White);
    ssd1306_SetCursor(RECT_0_X + 1, RECT_0_Y + 4 + 2 * (CHAN_HEIGHT + CHAN_Y_PAD));
    ssd1306_WriteString("21", Font_7x10, White);
    ssd1306_SetCursor(RECT_0_X + 1, RECT_0_Y + 4 + 3 * (CHAN_HEIGHT + CHAN_Y_PAD));
    ssd1306_WriteString("31", Font_7x10, White);
}

/**
 * Run one of the functions available on the menu
 * @param function
 */
static void runFunction(uint8_t function) {
    if (function == 3) { // Flash
        modeDisplay = eModeFlash;
    } else if (function == 1) { // Reset
        modeDisplay = eModeReady;
        CommandReset();
    } else if (function == 2) { // Blackout
        modeDisplay = eModeBlackout;
        CommandBlackout();
    } else if (function == 0) { // Check
        modeDisplay = eModeCheck;
        CommandTest();
    }
}

void displayTask(void *pvParameters) {
    // The currently selected function on the menu
    static uint8_t currentFunction = 0;

    // Initialise the OLED display
    ssd1306_Init();
    redraw();

    while (1) {
        // First, check if the button was pressed
        // The ticks to wait here define the update rate of the screen
        if (0 != (DISPLAYTASK_PRESSED_BIT & xEventGroupWaitBits(xButtonEventGroupHandle, DISPLAYTASK_PRESSED_BIT, 0x0, pdFALSE, 50))) {
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
                    runFunction(currentFunction);
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
        char modeText[4] = { '\0' };
        switch(modeDisplay) {
            case eModeReady:
                memcpy(modeText, "RDY", 3);
                break;
            case eModeCheck:
                memcpy(modeText, "CHK", 3);
                break;
            case eModeSerial:
                memcpy(modeText, "SR ", 3);
                break;
            case eModeWireless:
                memcpy(modeText, "RX ", 3);
                break;
            case eModeBlackout:
                memcpy(modeText, "B/O", 3);
                break;
            case eModeFlash:
                memcpy(modeText, "FLS", 3);
                break;
            default:
                memcpy(modeText, "???", 3);
        }
        ssd1306_SetCursor(92, 0); // top right corner
        ssd1306_WriteString(modeText, Font_11x18, White);

        // Show a refresh rate indicator to let the user know if the screen is being updated
        ssd1306_SetCursor(85, 0); // Draw next to the title
        static uint8_t refreshRateIndicator = 0; // A boolean that is toggled every time
        ssd1306_WriteChar(refreshRateIndicator ? '\'' : ' ', Font_7x10, White); // Write a dot for the update
        refreshRateIndicator = !refreshRateIndicator; // Reset the value for the next iteration

        // Draw the universe channel values as boxes with different heights according to the values
        uint16_t c = 0; // The current channel
        for (int y = 0; y < 4; y++) { // The current row #
            for (int x = RECT_0_X + 18; x < RECT_1_X - CHAN_WIDTH; x += CHAN_WIDTH + CHAN_X_PAD) { // The current X pixel
                // Normalize the level of the channel so we can find how tall it should be
                uint8_t level = universe[c] / (255 / CHAN_HEIGHT);

                // The bottom line of the box
                uint16_t originY = RECT_0_Y + 2 + CHAN_HEIGHT + y * (CHAN_HEIGHT + CHAN_Y_PAD);

                // Draw the box itself
                for (int z = 0; z < CHAN_HEIGHT; z++) {
                    for (int j = 0; j < CHAN_WIDTH; j++) {
                        // White when the level is high enough, black to show nothing
                        ssd1306_DrawPixel(x + j, originY - z, level > z ? White : Black);
                    }
                }

                // We have processed this channel. Let's go to the next channel.
                c++;
            }

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
    }
}


