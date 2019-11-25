#include "stdint.h"
#include "string.h"
#include "universe.h"

uint8_t universe[512];

bool universeIsWritable = true;

void temporaryBlackout() {
    memset(universe, 0, 512);
}
