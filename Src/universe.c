#include "FreeRTOS.h"
#include <event_groups.h>
#include <main.h>
#include "stdint.h"
#include "string.h"
#include "universe.h"
#include "pwmTask.h"

volatile uint8_t universe[DMX_MAX];

bool universeIsWritable = true;

void temporaryBlackout() {
    memset(universe, 0, DMX_MAX);
}

void notifyUniverseUpdate() {
    xEventGroupSetBits(xPwmEventGroupHandle, PWMTASK_UPDATE_BIT);
}
