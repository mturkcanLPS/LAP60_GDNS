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

#ifndef I_TIME_H
#define I_TIME_H

namespace hrvs_sdk
{
    /**
     * @brief Interface to the time source.
     */
    class ITime
    {
    public:
        /**
         * @brief Method should return time since some constant reference time (e.g. time since system was booted or UTC time) in milliseconds. 
         * 
         * @return long long Time since some reference time in milliseconds.
         */
        virtual long long getTime() = 0;
    };
}

#endif // #ifndef I_TIME_H
