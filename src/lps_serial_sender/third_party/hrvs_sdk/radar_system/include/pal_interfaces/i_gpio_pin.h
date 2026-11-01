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

#ifndef I_GPIO_PIN_H
#define I_GPIO_PIN_H

namespace hrvs_sdk
{
    /**
     * @brief Interface for handling of GPIO pin used for the triggering of the meassurement. GPIO pin should be configured as the output pin.
     * @note Initialization and configuration methods are platform dependent. For this reason these methods are not part of the interface.
     * @note For the triggering of the measurement is sufficient to implement only write method (read method is not required).
     */
    class IGPIOPin
    {
    public:
        /**
         * @brief Enumeration defining discrete pin states.
         */
        enum State : unsigned char
        {
            ZERO = 0x00,
            ONE = 0x01,
            INVALID = 0xFF,
        };

    public:
    /**
     * @brief Writes state on GPIO pin.
     *
     * @param state State to write on pin.
     * @return true Writing state of GPIO pin finished successfully.
     * @return false Writing state of GPIO pin finished with error.
     */
    virtual bool writePinState(State state) const = 0;

        /**
         * @brief Method for releasing resources of GPIO pin.
         * @return true Releasing resources reserved by GPIO bin ended with success.
         * @return false Releasing resources reserved by GPIO bin ended with failure.
         */
        virtual bool closePin() = 0;
    };
}

#endif // #ifndef I_THREAD_H
