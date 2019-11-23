#include "FreeRTOS.h"
#include <task.h>
#include "displayTask.h"
#include "ssd1306.h"

void displayTask(void *pvParameters) {
    vTaskDelay(400);

    ssd1306_Init();
    ssd1306_Fill(Black);

    while (1) {
        ssd1306_SetCursor(2, 0);
        ssd1306_WriteString("Welcome", Font_11x18, White);

        ssd1306_SetCursor(2,50);
        if (xTaskGetTickCount() % 2 < 1) {
            ssd1306_WriteChar('>', Font_7x10, White);
        } else {
            ssd1306_WriteChar(' ', Font_7x10, White);
        }

        ssd1306_UpdateScreen();

        vTaskDelay(10);
    }
}
