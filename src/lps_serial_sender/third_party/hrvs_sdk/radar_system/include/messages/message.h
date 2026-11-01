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

#ifndef MESSAGE_H
#define MESSAGE_H

#include "utils/byte_buffer.h"
#include <cstdint>

#ifdef MSG_LOGGING
#include <ostream>
#include <sstream>
#endif	// #ifdef MSG_LOGGING

namespace hrvs_sdk
{
    namespace BYTE_BUFFER = ECTOS::BYTE_BUFFER;

    /************************   MESSAGE INTERFACES  *************************/

    /**
     * @brief Interface for all radar messages used for accessing the message data and message controlling.
     */
    class IRadarMsg
    {
        /******************************   ENUMS   *******************************/
        
        public:
            /**
             * @brief Enumeration with message type identifiers.
             */
            enum MessageType : uint8_t
            {
                NONE = 0,
                // enumeration values, which are sent in messages received from HRVS:
                MINRANGE = 1,
                OBJECT_INFO = 3,
                FILTEREDPOINT = 4,
                ACK = 5,
                // enumeration values used just by radar system:
                CONFIG_MSG,
                MEASUREMENT_START_MSG,
                CONTROL_MSG
            };

            /**
             * @brief Enumeration with supported versions of radar protocol.
             */
            enum RadarProtocolVersion
            {
                PROTOCOL_VER_1_0 = 10,
                PROTOCOL_VER_1_1 = 11
            }; 

        /****************************   CONSTANTS   *****************************/
        
        public:
            static constexpr unsigned SYNC_WORD_LENGTH = 4;                                    /**< Length of synchronization word in bytes. */
            static constexpr uint8_t SYNC_WORD[SYNC_WORD_LENGTH] = {0xA9, 0xB8, 0xC7, 0xD6};   /**< Synchronization word located at the start of every radars output message. */
            static constexpr unsigned CHSUM_LENGTH = 2;                                         /**< Checksum value size in bytes. */
            static constexpr uint8_t UNDEFINED_MSG_RADAR_ID = 0;                               /**< Used when message does not have recipient/sender radar ID (measurement start). */
            static constexpr uint16_t UNDEFINED_MSG_RADAR_FRAME = -1;                          /**< Used when message does not have radar frame number item in its content. */
            static constexpr char* MESSAGE_NAME[] = {
                                                    (char*)"UNKNOWN MESSAGE TYPE",
                                                    (char*)"MINRANGE MESSAGE",
                                                    (char*)"UNKNOWN MESSAGE TYPE",
                                                    (char*)"OBJECT INFO MESSAGE",
                                                    (char*)"FILTERED POINT MESSAGE",
                                                    (char*)"CONFIG ACKNWOLEDGMENT MESSAGE",
                                                    (char*)"CONFIG MESSAGE",
                                                    (char*)"MEASUREMENT START MESSAGE",
                                                    (char*)"CONTROL MESSAGE"
                                                    };

        /************************   INTERFACE METHODS   *************************/

        public:
            /**
             * @brief Gets ID of the msg (used by HRVS to differentiate between message types).
             * 
             * @return MessageType Message ID.
             */
            virtual MessageType getMsgId() const = 0;

            /**
             * @brief Gets the name of msg (used just for logging in radar system methods).
             * 
             * @return const char* Name of message.
             */
            virtual const char* getMsgName() const = 0;

            /**
             * @brief Gets the length of the message.
             * In case messages with payload, returned value is size of payload (to get header length use method getHeaderLength()).
             * 
             * @return unsigned Total length of message in bytes.
             */
            virtual unsigned getMsgLength() const = 0;

            /**
             * @brief Gets frame number from message item of radar message. 
             * 
             * @return uint16_t Number of radar frame.
             */
            virtual uint16_t getRadarFrame() const = 0;

#ifdef MSG_LOGGING
            /**
             * @brief Overloading of insertion operator.
             *
             * @param out Output stream.
             * @param message Object of message class, which string representation should be inserted into output stream.
             * @return std::ostream& Output stream.
             */
            friend std::ostream &operator<<(std::ostream &out, const IRadarMsg &message)
            {
                message.print(out);
                return out;
            }
        
        protected:
            /**
             * @brief Method for inserting string representation of object to output stream.
             *
             * @param out Output stream.
             */
            virtual void print(std::ostream &out) const = 0;

        public:
            // items always printed before message items when logging message
            unsigned logSystemFrameNumber;
            long long logLastMeasTriggerTime;
#endif	// #ifdef MSG_LOGGING
    };

    /************************************************************************/

    /**
     * @brief Interface for message classes, which are input of HRVS.
     */
    class IRadarInput : public virtual IRadarMsg
    {
        /************************   INTERFACE METHODS   *************************/

        public:
            /**
             * @brief Get recipient radar ID. 
             * 
             * @return uint8_t Recipient radar ID.
             */
            virtual uint8_t getRecipientRadarID() const = 0;

            /**
             * @brief Method decomposes message items from instance variables into series of bytes and stores them in buffer.
             * 
             * @param[out] buffer Output buffer where will be stored bytes.
             * @param[in] bufferSize Fixed size of buffer.
             * @return int Total bytes arised by decomposition.
             * If output buffer is small, then method does not return message length but some smaller value (how many decomposed bytes can be stored in output buffer).
             */
            virtual int packMessage(uint8_t *buffer, int bufferSize) = 0;
    };

    /************************************************************************/

    /**
     * @brief Interface for message classes, which are output of HRVS.
     */
    class IRadarOutput : public virtual IRadarMsg
    {
        /************************   INTERFACE METHODS   *************************/

        public:
            /**
             * @brief Get sender radar ID. 
             * 
             * @return uint8_t Sender radar ID.
             */
            virtual uint8_t getSenderRadarID() const = 0;

            /**
             * @brief  Method composes message items from series of bytes in buffer, stores them in instance variables and checks message checksum.
             *
             * @param[in] buffer  Input buffer where are stored bytes.
             * @param[in] bufferSize  Fixed size of buffer.
             * @return true Received data are valid (computed checksum and received checksum are equal).
             * @return false Received data are not valid (computed checksum and received checksum are not equal).
             */
            virtual bool unpackMessage(uint8_t *buffer, int bufferSize) = 0;

            /**
             * @brief Getter for validity of received data.
             *
             * @return true Received data are valid (computed checksum and received checksum are equal).
             * @return false Received data are not valid (computed checksum and received checksum are not equal).
             */
            bool isValid() const { return validData; }

        /************************   INSTANCE VARIABLES   ************************/

        protected:
            bool validData = false;     /**< Validity of received data. */
    };

    /************************************************************************/

    /**
     * @brief Interface for HRVS ouput messages consisting of header and payload with unknown size.
     * 
     * If number of received payload items is higher, ReturnCode::HRVS_SDK_ERROR will be thrown.
     */
    class IRadarPayloadOutput : public IRadarOutput
    {
        /************************   INTERFACE METHODS   *************************/

        public:
            /**
             * @brief Gets the length of the message header (only this length is known in advance). 
             * 
             * @return unsigned Total length of message header in bytes.
             */
            virtual unsigned getHeaderLength() const = 0;

            /**
             * @brief  Method composes message header items from series of bytes in buffer, stores them in instance variables and checks message checksum.
             * Method is used to find out how many bytes has payload, so then getMsgLength() returns correct size of message payload.
             * 
             * @param[in] buffer Input buffer where are stored bytes.
             * @param[in] bufferSize  Fixed size of buffer.
             * @return true Received data are valid (computed checksum and received checksum are equal).
             * @return false Received data are not valid (computed checksum and received checksum are not equal).
             */
            virtual bool unpackHeader(uint8_t *buffer, int bufferSize) = 0;

            /** 
             * @brief Checks whether header is unpacked.
             * 
             * @return true Header is unpacked.
             * @return false Header is not unpacked.
             */
            bool isHeaderUnpacked() const
            {
                return headerUnpacked;
            }

            /** 
             * @brief Checks whether header is valid.
             * 
             * @return true Header is valid.
             * @return false Header is not valid.
             */
            bool isHeaderValid() const
            {
                return headerValid;
            }

            /**
             * @brief Gets count of payload items.
             * 
             * @return unsigned Payload items count.
             */
            virtual unsigned getPayloadItemsCount() const = 0;

        
        /************************   INSTANCE VARIABLES   ************************/

        protected:
            bool headerUnpacked = false;     /** Header unpacked flag. */
            bool headerValid = false;        /** Header validity flag. */
    };

    /**
     * @brief Abstract class for HRVS ouput messages consisting of header and payload with unknown size.
     * 
     * @tparam PAYLOAD_ITEM_TYPE Type of items included in message payload.
     * @tparam MAX_PAYLOAD_ITEMS Max number of payload items, which will be stored. 
     * If number of received payload items is higher, ReturnCode::HRVS_SDK_ERROR will be thrown.
     */
    template<typename PAYLOAD_ITEM_TYPE, int MAX_PAYLOAD_ITEMS>
    class RadarPayloadOutputAbstract : public IRadarPayloadOutput
    {
        public:

        /*****************************   METHODS   ******************************/

            /**
             * @brief Subscripting operator allows access to message items.
             * 
             * @param i Index of message payload item within array of message payload items.
             * Entered index should be in range <0, getPayloadItemsCount()), otherwise first item of items array is returned.
             * @return PAYLOAD_ITEM_TYPE& Message payload item.
             */
            PAYLOAD_ITEM_TYPE& operator[](unsigned i)
            {
                if(i >= 0 && i < getPayloadItemsCount())
                {
                    return payloadItems[i];
                }
                else
                {
                    // return first item, in case invalid index is entered
                    return payloadItems[0];
                }
            };
            
            /**
             * @brief Constant variant of subscripting operator.
             * 
             * @param i Index of message payload item within array of message payload items.
             * Entered index should be in range <0, getPayloadItemsCount()), otherwise first item of items array is returned.
             * @return PAYLOAD_ITEM_TYPE& Message payload item.
             */
            const PAYLOAD_ITEM_TYPE& operator[](unsigned i) const
            {
                if(i >= 0 && i < getPayloadItemsCount())
                {
                    return payloadItems[i];
                }
                else
                {
                    // return first item, in case invalid index is entered
                    return payloadItems[0];
                }
            };

        /************************   INSTANCE VARIABLES   ************************/

        protected:
            PAYLOAD_ITEM_TYPE payloadItems[MAX_PAYLOAD_ITEMS];      /**< Array with message payload items. */           
    };
    
    /************************************************************************/
}

#endif
