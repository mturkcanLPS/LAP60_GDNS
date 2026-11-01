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

#ifndef UART_WINDOWS_H
#define UART_WINDOWS_H

#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <conio.h>
#include "pal_interfaces/i_uart.h"

/**
 * @brief The class implements the interface to UART devices for Windows OS.
 *
 */
class UARTWindows : public hrvs_sdk::IUART
{

public:
    /**
     * @brief Enum class with possible return codes.
     */
    enum ReturnCode
    {
        OK = 0,
        INIT_ERR,
        GET_COMM_STATE_ERR,
        SET_COMM_STATE_ERR,
        SET_COMM_TIMEOUTS_ERR
    };

private:
    HANDLE handle;

public:
    /**
     * @brief Method for initialization of communication.
     *
     * @param comPort Comm port name.
     * @return int Return code (0 means success).
     */
    int initUART(const char *comPort);

    /**
     * @brief Method for configuration of communication.
     *
     * @param baudrate Communication baudrate.
     * @param parity Parity setting.
     * @param stopBits Number of stop bits.
     * @param dataBits Number of data bits (default 8).
     * @return int Return code (0 means success).
     */
    int configureUART(int baudrate, Parity parity, StopBits stopBits, int dataBits = 8);

    /**
     * @brief Method for reading data from UART.
     *
     * @param buffer Buffer where to store read bytes.
     * @param bytesToRead Number of bytes to read.
     * @param bytesRead Number of actually read bytes.
     * @return true UART read success.
     * @return false UART read error.
     */
    bool readUART(uint8_t *buffer, int bytesToRead, int *bytesRead) const;

    /**
     * @brief Method for writing data to UART.
     *
     * @param buffer Buffer from which are bytes send.
     * @param bytesToWrite Number of bytes to write.
     * @param bytesWritten Number of actually written bytes.
     * @return true UART write success.
     * @return false UART write error.
     */
    bool writeUART(uint8_t *buffer, int bytesToWrite, int *bytesWritten) const;

    /**
     * @brief Method for releasing resources of UART communication.
     *
     * @return true Releasing UART resources ended with success.
     * @return false Releasing UART resources ended with failure.
     */
    bool closeUART();

};

#endif // #ifndef UART_WINDOWS_H
