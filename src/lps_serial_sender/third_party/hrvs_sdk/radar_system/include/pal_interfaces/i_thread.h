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

#ifndef I_THREAD_H
#define I_THREAD_H

namespace hrvs_sdk
{
    /**
     * @brief Interface for manipulation of active thread.
     */
    class IThread
    {
    public:
        /**
         * @brief Method should put the active thread to sleep for given count of milliseconds.
         * 
         * @param ms How long should active thread be sleeping.
         */
        virtual void sleep(long long ms) = 0;
    };
}

#endif // #ifndef I_THREAD_H
