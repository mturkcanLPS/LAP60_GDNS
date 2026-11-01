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

#ifndef ASYNC_LOG_WRITER_H
#define ASYNC_LOG_WRITER_H

#ifndef NO_INFO

#include "utils/easylogging++.h"
#include <tuple>
#include <string>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace hrvs_sdk
{   
    /**
     * @brief This class is used to write logs via easylogging library asynchronously.
     */
    class AsyncLogWriter
    {
        private:
            static constexpr unsigned LOG_QUEUE_SIZE = 1000;              /** Max size of the queue with logs. */
        
        public:
            /**
             * @brief Construct a new AsyncLogWriter object.
             */
            AsyncLogWriter();
            
            /**
             * @brief Release resources of the AsyncLogWriter object by calling close() method.
             */
            ~AsyncLogWriter();

            /**
             * @brief Start the worker thread for writing logs from the queue by easylogging library.
             */
            void open();
            
            /**
             * @brief Wait for all logs from temporary queue to be written by worker thread and stop the worker thread.
             */
            void close();

            /**
             * @brief Add a log message with the given level to queue and notify the worker thread.
             * If queue is used by worker thread, the log message will be stored in temporary queue.
             * If temporary queue is full, the log message will be discarded.
             * 
             * @param level Log level.
             * @param logMessage Log message.
             */
            void log(el::Level level, const std::string& logMessage);

            /**
             * @brief Synchronous operation. Wait for all logs from temporary queue to be written by worker thread.
             */
            void flush();

        private:
            /** 
             * @brief Worker thread function that performs logging of messages from queue by easylogging library.
             */
            void writeThread();

        private:
            std::thread worker;                                             // Worker thread to perform logging asynchronously
            std::queue<std::tuple<el::Level, std::string>> logQueue;        // Queue of logs (shared resource)
            std::queue<std::tuple<el::Level, std::string>> tmpLogQueue;     // Temporary queue to store logs if logQueue is used by worker thread
            std::mutex mtx;                                                 // Mutex to protect logQueue
            std::condition_variable cond;                                   // Condition variable to notify worker thread
            std::condition_variable condFlush;                              // Condition variable to notify main thread when worker thread has finished writing logs after flush()
            bool exitFlag;                                                  // Flag to signal worker thread to exit
            bool notifyFlag;                                                // FLag to signal worker thread to notify main thread when it has finished writing logs after flush()
    };
}
#endif // #ifdef NO_INFO

#endif // #ifndef ASYNC_LOG_WRITER_H
