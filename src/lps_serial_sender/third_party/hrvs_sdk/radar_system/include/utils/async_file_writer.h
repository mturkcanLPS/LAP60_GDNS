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

#ifndef ASYNC_FILE_WRITER_H
#define ASYNC_FILE_WRITER_H

#ifdef MSG_LOGGING

#include <string>
#include <sstream>
#include <fstream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace hrvs_sdk
{   
    /**
     * @brief This class is used to write to a file asynchronously.
     */
    class AsyncFileWriter
    {
        private:
            static constexpr unsigned STRING_QUEUE_SIZE = 1000;              /** Max size of the queue with strings to be written to file. */

        public:
            /**
             * @brief Construct a new AsyncFileWriter object.
             */
            AsyncFileWriter();
            
            /**
             * @brief Release resources of the AsyncFileWriter object by calling close() method.
             */
            ~AsyncFileWriter();

            /**
             * @brief Open a file for writing and start the worker thread for writing strings from the queue to the file.
             * 
             * @param[in] filename The name of the file to open.
             */
            void open(const std::string& filename);
            
            /**
             * @brief Wait for all strings from temporary queue to be written by worker thread and stop the worker thread.
             */
            void close();

            /**
             * @brief Converts passed value to a string, then calls enqueueWrite().
             * 
             * @tparam T The type of the value to write.
             * @param value The value to write.
             * @return AsyncFileWriter& A reference to the AsyncFileWriter object, to enable chaining of << operators.
             */
            template <typename T>
            AsyncFileWriter& operator<<(const T& value)
            {
                std::ostringstream oss;
                oss << value;
                enqueueWrite(oss.str());
                return *this;
            };

            /**
             * @brief Check if the file is open.
             * 
             * @return true if the file is open, false otherwise.
             */
            bool isOpen() const;

            /**
             * @brief  Synchronous operation. Wait for all strings from temporary queue to be written by worker thread to the file.
             */
            void flush();

        private:
            /** 
             * @brief Worker thread function that writes strings from the queue to the file.
             */
            void writeThread();
            
            /**
             * @brief Add a string to the write queue and notify the worker thread.
             * If queue is used by worker thread, the string will be stored in temporary queue.
             * If temporary queue is full, the string will be discarded.
             * 
             * @param data String to write.
             */
            void enqueueWrite(const std::string& data);

        private:
            std::ofstream ofs;                      // File stream to write to
            std::thread worker;                     // Worker thread to write to file asynchronously
            std::queue<std::string> writeQueue;     // Queue of strings to write to file (shared resource)
            std::queue<std::string> tmpWriteQueue;  // Temporary queue to store strings if writeQueue is used by worker thread
            std::mutex mtx;                         // Mutex to protect writeQueue
            std::condition_variable cond;           // Condition variable to notify worker thread
            std::condition_variable condFlush;      // Condition variable to notify main thread when worker thread has finished writing after flush()
            bool exitFlag;                          // Flag to signal worker thread to exit
            bool notifyFlag;                        // Flag to signal worker thread to notify main thread when it has finished writing after flush()
    };
}
#endif // #ifdef MSG_LOGGING

#endif // #ifndef ASYNC_FILE_WRITER_H
