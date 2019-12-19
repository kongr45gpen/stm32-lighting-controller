#include <stdint.h>
#include <string.h>
#include <nrf24.h>
#include <displayTask.h>
#include "wirelessTask.h"

static const uint8_t nrf24Address[] = "kL";

/**
 * A task to wirelessly transmit DMX data
 */
void wirelessTask(void *pvParameters) {
    vTaskDelay(500);

    nRF24_GPIO_Init();
    nRF24_Init();

//    while(1) { nRF24_Check(); }

    if (!nRF24_Check() || 1) {
        addErrorMessage("NRF24L01 not found", 100);

        // NRF24 not found, bail out
        vTaskDelete(NULL);

        // Just for safety
        return;
    } else {
        addErrorMessage("NRF24L01 YES found", 100);
    }

    nRF24_SetRFChannel(99); // Set RF channel
    nRF24_SetDataRate(nRF24_DR_1Mbps); // Set data rate high enough
    nRF24_SetCRCScheme(nRF24_CRC_2byte); // Set error correction CRC
    nRF24_SetAddrWidth(2); // Set the length of the address
    nRF24_SetAddr(nRF24_PIPETX, nrf24Address); // Program TX address
    nRF24_SetAddr(nRF24_PIPE0, nrf24Address); // Program TX address for pipe#0 (auto-ack)
    nRF24_SetTXPower(nRF24_TXPWR_0dBm); // Set RF power (maximum)
    nRF24_DisableAA(nRF24_PIPE0); // Disable Auto-ACK
    nRF24_SetOperationalMode(nRF24_MODE_TX); // Set transmit mode
    nRF24_SetPowerMode(nRF24_PWR_UP); // Wake up the transceiver

    while (1) {
        uint8_t payload[] = "Hello world to the wondrous world of music! What more could one ask?";

        nRF24_ClearIRQFlags(); // Clear any pending IRQ flags
        nRF24_TransmitPacket(payload, strlen(payload));

        vTaskDelay(400);
    }
}
