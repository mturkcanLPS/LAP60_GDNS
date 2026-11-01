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

#ifndef UART_LINUX_H
#define UART_LINUX_H

#include "pal_interfaces/i_uart.h"

/**
 * @brief The class implements the interface to UART devices for Linux OS.
 *
 */
class UARTLinux : public hrvs_sdk::IUART
{
public:
    /**
     * @brief Return codes.
    */
    enum ReturnCode
    {
        OK = 0,
        OPENING_ERROR,
        INVALID_BAUDRATE,
        INVALID_BYTESIZE,
        INVALID_STOPBITS,
        INVALID_PARITY,
        CONFIGURATION_ERROR
    };

private:
    int handle;

public:
    UARTLinux();
    int initUART(const char *comPort);
    int configureUART(int baudrate, Parity parity, StopBits stopBits, int dataBits = 8);
    bool readUART(uint8_t *buffer, int bytesToRead, int *bytesRead) const;
    bool writeUART(uint8_t *buffer, int bytesToWrite, int *bytesWritten) const;
    bool closeUART();
};

#endif // ifndef UART_LINUX_H
