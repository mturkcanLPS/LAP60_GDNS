/*
 * SOFTWARE RIGHTS NOTICE:
 *
 * COPYRIGHT 2023 HONEYWELL INTERNATIONAL INC.  ALL RIGHTS RESERVED.
 * THE INFORMATION CONTAINED HEREIN MY NOT BE REPRODUCED, USED, DISTRIBUTED
 * OR DISCLOSED TO OTHERS WITHOUT THE WRITTEN CONSENT OF HONEYWELL.
 * NOTHING CONTAINED HEREIN SHALL BE CONSTRUED AS CONFERRING BY IMPLICATION,
 * ESTOPPEL, OR OTHERWISE ANY LICENSE TO ANY PATENT, TRADEMARK, COPYRIGHT
 * OR OTHER INTELLECTUAL PROPERTY RIGHT OF HONEYWELL OR ANY THIRD PARTY.
 *
 */

#ifndef GPIO_PIN_ARDUINO_H
#define GPIO_PIN_ARDUINO_H

#include "pal_interfaces/i_gpio_pin.h"
#include "pal_interfaces/i_uart.h"
#include <cstdint>

/**
 * @brief The class implements the GPIO Pin interface that communicates with the Arduino controller.
 *
 */
class GPIOPinArduino : public hrvs_sdk::IGPIOPin
{
private:
    static constexpr uint8_t ARDUINO_SYNC_CHAR = 0xFF;      // every Arduino response start with this character
    static constexpr unsigned INPUT_BUFFER_LENGTH = 64;     // legnth of buffer for receving Arduino responses
    static constexpr unsigned UART_MAX_TRIALS = 50;         // maximum number of trials to read data from Arduino UART
    static constexpr unsigned UART_WAIT = 20;               // how long to wait for next trial of reading data from Arduino UART  in ms
    static constexpr unsigned ARDUINO_RESP_SIZE = 2;        // size of arduino response message (simple Arduino protocol located in examples for customers)



public:
    /**
     * @brief Construct a new GPIOPinArduino object.
     * @param uart Pointer to UART object.
     *
     */
    GPIOPinArduino(hrvs_sdk::IUART *uart);

    /**
     * @brief Destroy the GPIOPinArduino object
     *
     */
    ~GPIOPinArduino();

    /**
     * @brief Sends message to Arduino via UART. The message format is {0xFF, pin, state}.
     *
     * @param state Possible pin states are either ZERO or ONE.
     * @return true Method wrote to UART successfully.
     * @return false Method failed writing to UART.
     */
    bool writePinState(State state) const;

    /**
     * @brief Closes the UART used for communication with Arduino.
     *
     */
    bool closePin();

private:
    hrvs_sdk::IUART* pUart; /**< reference to arduinoUart object. */

    /**
     * @brief Reads response from Arduino UART. Initial received bytes are skipped until ARDUINO_SYNC_CHAR is received.
     * 
     * @param response Buffer to store read data.
     * @param responseLength How many bytes to read.
     * @return true Succesfully read response. 
     * @return false Reading of response failed.
     */
    bool readResponseFromUart(uint8_t *response, int responseLength) const;


};

#endif // ifndef GPIO_PIN_ARDUINO_H
