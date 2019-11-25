#ifndef STM32_LIGHTING_CONTROLLER_UNIVERSE_H
#define STM32_LIGHTING_CONTROLLER_UNIVERSE_H

#include <stdbool.h>
#include "stdint.h"

// The maximum number of DMX channels per DMX universe. Also used for the internal universe representation
#define DMX_MAX 512

/**
 * Whether the universe is editable (through serial or wireless) and not disabled by an internal reason
 */
extern bool universeIsWritable;

/**
 * DMX values for all DMX channels, 1-512
 */
extern uint8_t universe[DMX_MAX];

/**
 * Get a DMX value
 * @param channel The DMX channel
 * @return The 8-bit DMX value of the channel
 */
inline uint8_t getValue(uint16_t channel) {
    return universe[channel];
}

/**
 * Set a DMX channel's value
 * @param channel The DMX channel
 * @param value The DMX channel value
 */
inline void setValue(uint16_t channel, uint8_t value) {
    universe[channel] = value;
}

/**
 * Notify watching tasks that the universe has changed
 */
void notifyUniverseUpdate();

/**
 * Black out (set to 0) all the channels of the universe for an instant
 */
void temporaryBlackout();

#endif //STM32_LIGHTING_CONTROLLER_UNIVERSE_H
