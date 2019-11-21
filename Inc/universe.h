#ifndef STM32_LIGHTING_CONTROLLER_UNIVERSE_H
#define STM32_LIGHTING_CONTROLLER_UNIVERSE_H

#include "stdint.h"

/**
 * DMX values for all DMX channels, 1-512
 */
extern uint8_t universe[512];

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

#endif //STM32_LIGHTING_CONTROLLER_UNIVERSE_H
