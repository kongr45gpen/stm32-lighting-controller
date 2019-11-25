#include "stdint.h"
#include "string.h"
#include "universe.h"

uint8_t universe[DMX_MAX];

bool universeIsWritable = true;

void temporaryBlackout() {
    memset(universe, 0, DMX_MAX);
}
