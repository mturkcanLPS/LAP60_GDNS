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

#ifndef I_UART_H
#define I_UART_H

#include <cstdint>

namespace hrvs_sdk
{
    /**
     * @brief Interface for handling of communication with HRVS via UART interface.
     */
    class IUART
    {
    public:
        /**
         * @brief Enumeration defining the possible parity settings.
         */
        enum Parity
        {
            NO = 0,
            EVEN = 1,
            ODD = 2
        };

        /**
         * @brief Enumeration defining the possible number of stop bits.
         */
        enum StopBits
        {
            ONE = 1,
            TWO = 2
        };

    public:
        /**
         * @brief Reads a data from UART.
         *
         * @param buffer Buffer where to store read bytes.
         * @param bytesToRead Number of bytes to read.
         * @param bytesRead Number of actually read bytes.
         * @return true UART read success.
         * @return false UART read error.
         */
        virtual bool readUART(uint8_t *buffer, int bytesToRead, int *bytesRead) const = 0;

        /**
         * @brief Writes the data to UART.
         *
         * @param buffer Buffer from which are bytes send.
         * @param bytesToWrite Number of bytes to write.
         * @param bytesWritten Number of actually written bytes.
         * @return true UART write success.
         * @return false UART write error.
         */
        virtual bool writeUART(uint8_t *buffer, int bytesToWrite, int *bytesWritten) const = 0;

        /**
         * @brief Method for releasing resources of UART communication.
         *
         * @return true Releasing UART resources ended with success.
         * @return false Releasing UART resources ended with failure.
         */
        virtual bool closeUART() = 0;
    };
}

#endif // #ifndef I_UART_H
