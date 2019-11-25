#include <main.h>
#include <stdbool.h>
#include <universe.h>
#include <displayTask.h>
#include <stm32h7xx_ll_gpio.h>
#include "serialTask.h"
#include "stm32h7xx_ll_usart.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "event_groups.h"

#define SERIAL_COM_OP 125           // The command header
#define SERIAL_COM_DMX 'f'          // A full DMX package
#define SERIAL_COM_RESET_ERRORS 'e' // Command to reset all errors

/**
 * A function that gets called whenever USART3 receives data. This is set here explicitly and is called automatically
 * by HAL_UART_IRQHandler, since that function automatically performs all the checks needed to see if we have received
 * data without errors.
 */
void USART3_RXHandler() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify the task that we have new data
    xEventGroupSetBitsFromISR(xSerialEventGroupHandle, SERIALTASK_RX_BITS, &xHigherPriorityTaskWoken);

    // Read the data from the USART3 peripheral to clear the bit and the hardware FIFO, if it is enabled. Unfortunately,
    // we need to do this in the ISR (as such, a software FIFO is required), because not reading from the USART FIFO
    // doesn't clear the interrupt, and would end in an infinite loop
    uint8_t datum = USART3->RDR;
    if (1 != xStreamBufferSendFromISR(xSerialReceiveBufferHandle, &datum, 1, &xHigherPriorityTaskWoken)) {
        // We didn't send the expected number of bytes
        static const char message[256] = "Serial message buffer is full";
        xQueueSendFromISR(xErrorQueueHandle, message, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    // Blink the requested LED
    LL_GPIO_SetOutputPin(LD1_GPIO_Port, LL_GPIO_PIN_0);
}

/**
 * Function to call when a FIFO overrun occurs on reception in USART3
 */
void USART3_OERHandler() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Throw an error message
    static const char message[256] = "RX fifo is full, you are sending data too fast!";
    xQueueSendFromISR(xErrorQueueHandle, message, &xHigherPriorityTaskWoken);

    // Flush the FIFO for good measure
    LL_USART_RequestRxDataFlush(USART3);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// The current channel being edited
static uint16_t currentUniverseChannel = 0;
/**
 * The handler for bytes received in the Universe serial command
 * @param datum The data received from the serial port
 * @return Whether the Universe command can receive more data
 */
bool UniverseCommand(uint8_t datum) {
    // Set the value of the universe channel based on the data provided by the serial port
    if (universeIsWritable) {
        universe[currentUniverseChannel] = datum;
    }

    if (currentUniverseChannel < DMX_MAX - 1) {
        // The next byte we receive will be for the next channel
        currentUniverseChannel++;

        // We can receive more channel data
        return true;
    } else {
        // Reset the counter in case the user provides circular data
        currentUniverseChannel = 0;

        // Notify that the universe was updated
        notifyUniverseUpdate();

        // We cannot receive any further channel data.
        return false;
    }
}

void serialReadTask(void *pvParameters) {
    // Enable USART3 reception and error interrupts
    LL_USART_EnableIT_RXNE_RXFNE(USART3);
    LL_USART_EnableIT_RXFF(USART3);

    // A variable to define the command we are currently receiving
    static enum {
        eIdleCommand, ///< No command currently in progress
        eUniverseCommand, ///< Receiving DMX universe data
        eUnknownCommand, ///< Unknown command received, doing nothing and asking questions
    } currentCommand = eIdleCommand;

    // Whether we just received an operation code
    bool justReceivedOp = false;

    while(1) {
        // A variable to store the data to be received
        uint8_t datum = 0;

        // Read one byte from the stream buffer, as soon as it is available
        if (xStreamBufferReceive(xSerialReceiveBufferHandle, &datum, 1, portMAX_DELAY)) {
            if (!justReceivedOp && datum == SERIAL_COM_OP) {
                // We received a non-repeated OP byte. This means that the next byte will be an operation code.
                justReceivedOp = true;
            } else if (justReceivedOp && datum != SERIAL_COM_OP) {
                // Time for a new command to be received. We can parse it

                if (currentCommand == eUniverseCommand) {
                    // The previous command was the universe command, but it finished. Make sure to send a DMX universe
                    // update
                    notifyUniverseUpdate();
                }

                switch (datum) {
                    case SERIAL_COM_DMX:
                        // Received DMX command, just move the state and let the rest of the data be parsed
                        currentCommand = eUniverseCommand;
                        currentUniverseChannel = 0; // Reset the channel count

                        // Set the displayed mode on the screen
                        displayModeSet(eModeSerial);
                        break;
                    case SERIAL_COM_RESET_ERRORS:
                        // Reset all errors
                        break;
                    default:
                        // Unknown command received
                        currentCommand = eUnknownCommand;

                        char message[ERROR_MESSAGE_SIZE];
                        snprintf(message, ERROR_MESSAGE_SIZE, "Unknown command received: %x (%c)", datum, datum);
                        addErrorMessage(message, 0);
                }

                justReceivedOp = false; // Reset the old value
            } else {
                justReceivedOp = false; // Reset the old value

                // The output of the command ran
                bool commandRequestsMoreBytes = 0;

                // An input to a command was given. Send it to the corresponding function
                switch (currentCommand) {
                    case eUniverseCommand:
                        commandRequestsMoreBytes = UniverseCommand(datum);
                    default:
                        break; // Nothing to be done
                }

                if (!commandRequestsMoreBytes) {
                    // Switch to the idle command so that we don't send further data to the selected command if the
                    // user erroneously provides more bytes
                    currentCommand = eIdleCommand;
                }
            }

            // Unblink the LED
            HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
        }
    }
}


