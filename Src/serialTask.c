#include <main.h>
#include <stdbool.h>
#include <universe.h>
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
void USART3_RXHandler(UART_HandleTypeDef *huart) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify the task that we have new data
    xEventGroupSetBitsFromISR(xSerialEventGroupHandle, SERIALTASK_RX_BITS, &xHigherPriorityTaskWoken);

    // Read the data from the USART3 peripheral to clear the bit and the hardware FIFO, if it is enabled. Unfortunately,
    // we need to do this in the ISR (as such, a software FIFO is required), because not reading from the USART FIFO
    // doesn't clear the interrupt, and would end in an infinite loop
    uint8_t datum = USART3->RDR;
    xStreamBufferSendFromISR(xSerialReceiveBufferHandle, &datum, 1, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    // Blink the requested LED
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
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

    static uint8_t string [256];
    size_t len = snprintf(string, 256, "Writing %d to channel %d\r\n", datum, currentUniverseChannel);
    HAL_UART_Transmit(&huart3,string,len,1000);

    if (currentUniverseChannel < 512 - 1) {
        // The next byte we receive will be for the next channel
        currentUniverseChannel++;

        // We can receive more channel data
        return true;
    } else {
        // Reset the counter in case the user provides circular data
        currentUniverseChannel = 0;

        // We cannot receive any further channel data.
        return false;
    }
}


void serialReadTask(void *pvParameters) {
    // Register the manual handlers of USART3
    huart3.RxISR = USART3_RXHandler;

    // We need to enable the USART3 reception interrupt
    LL_USART_EnableIT_RXNE_RXFNE(USART3);

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
                switch (datum) {
                    case SERIAL_COM_DMX:
                        // Received DMX command, just move the state and let the rest of the data be parsed
                        currentCommand = eUniverseCommand;
                        currentUniverseChannel = 0; // Reset the channel count
                        break;
                    case SERIAL_COM_RESET_ERRORS:
                        // Reset all errors
                        break;
                    default:
                        // Unknown command received
                        currentCommand = eUnknownCommand;
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


