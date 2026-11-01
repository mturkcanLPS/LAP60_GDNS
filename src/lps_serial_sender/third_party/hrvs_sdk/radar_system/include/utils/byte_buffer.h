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

/*-------------------------------------------------------------------
 ByteBuffer.h

 Author: Jathan Manley (jathan.manley@honeywell.com)
 Date: January 18, 2008

 Purpose:  A templated set of classes to pack and unpack binary data buffers.

 The class template parameters allow the user to specify the sizeType
 and the byteType so that you can deal with extremely small or large buffers
 efficiently and also deal with your favorite byte type.  The restrictions are:
 the sizeType must be a signed integer type and the byteType must be an 8-bit
 integer type (int8_t, uint8_t, char, unsigned char).  The compiler will attempt
 to generate code with other types and those may even compile, but I would not
 expect correct results.  The third template argument is the endianess of the buffer.
 This const argument will allow an optimizer to avoid the conditional and generate
 essentially two separate code bases for the swapping case and the non-swapping
 case.

 The main functions are get/put for each class.  These are templated functions
 that allow you to specify the type of the variable to are trying to put/get
 from the buffer.  Only POD (plain old data) types should be specified here.
 If you specify a struct/class the bytes may be put/gotten in reverse order
 which will generally mean corruption.

 These classes are useful for packing/unpacking data from sensors
 in a portable way.

 HANDLING TEMPLATE ERRORS:
 (someday it would be nice to handle these with static assertions)

 To avoid a user specifying a byteType that is not 1-byte in length.  The
 user can specify an action (BYTE_TYPE_SIZE_ERROR_ACTION) that will be
 done when an invalid byteType is specified on construction.  Default action
 is to do nothing.

 The user can also specify a SIZE_TYPE_ERROR_ACTION that will take place
 if the template is declared with non-integer or an unsigned integer.

 Keep in mind, that the actions take place in the context of the class constructor
 , and must be defined before the #include.  This should only happen in one place
 for a given program so it would be best to wrap the include with a custom header
 that defines the actions for your program.  In general, you can leave them blank.

 Here is an example:

 #ifndef MY_INCLUDE
 #define MY_INCLUDE

 //define these functions somewhere in a .cpp file
 void myByteTypeError();
 void mySizeTypeError();

 //setup the MACROS
 #define BYTE_TYPE_SIZE_ERROR_ACTION byteTypeError()
 #define SIZE_TYPE_ERROR_ACTION sizeTypeError()

 #include <byte_buffer.h>

 #endif MY_INCLUDE

 Additionally, you can define BYTE_BUFFER_AVOID_TEMPLATE_CHECKS to remove all checking
 of template arguments.

 */


#ifndef BYTE_BUFFER_H
#define BYTE_BUFFER_H

//#define BYTE_BUFFER_AVOID_TEMPLATE_CHECKS

#ifndef BYTE_BUFFER_AVOID_TEMPLATE_CHECKS
	#include <limits>
#endif

namespace ECTOS
{

#ifndef BYTE_BUFFER_AVOID_TEMPLATE_CHECKS

	#ifndef BYTE_TYPE_SIZE_ERROR_ACTION
		#define BYTE_TYPE_SIZE_ERROR_ACTION do {} while(0)
	#endif

	#ifndef SIZE_TYPE_ERROR_ACTION
		#define SIZE_TYPE_ERROR_ACTION do {} while(0)
	#endif

#endif //BYTE_BUFFER_AVOID_TEMPLATE_CHECKS

namespace BYTE_BUFFER
{

	//---------------------------------------------------------------------------
	// ENDIANNESS Settings and constants
	//---------------------------------------------------------------------------
	const unsigned int endianInt = 0x0102;
	const bool PlatformIsBigEndian = (((char*)&endianInt)[0] == 0x01);

	//constants to allow users to specify the endianess of their buffers
	const bool bigEndian = true;
	const bool littleEndian = false;

	//---------------------------------------------------------------------------
	// ByteInputBuffer class definition
	//---------------------------------------------------------------------------

	template<class sizeType, class byteType, const bool bufferIsBigEndian> class ByteInputBuffer {
		public:
		/**
		 * Constructor
		 * @param inbuf the buffer you are going to extract data from
		 * @param len the length of the buffer
		 * Appropriate swaps will take place if the platfrom and buffer
		 * endian specs don't match
		 */
		ByteInputBuffer(const byteType* inbuf, sizeType len) :
		m_offset(0), m_len(len), m_data(inbuf)  {
#ifndef BYTE_BUFFER_AVOID_TEMPLATE_CHECKS
			if(sizeof(byteType) != 1 || !std::numeric_limits<byteType>::is_integer) {
				BYTE_TYPE_SIZE_ERROR_ACTION;
			}
			if(!(std::numeric_limits<sizeType>::is_integer && std::numeric_limits<sizeType>::is_signed)) {
				SIZE_TYPE_ERROR_ACTION;
			}
#endif //BYTE_BUFFER_AVOID_TEMPLATE_CHECKS
		}

		/**
		 * Gets an item of objectType from the buffer
		 * @param value destination object
		 * @return true if sizeof(value) bytes are available to
		 * convert. false, otherwise
		 */
		template<class objectType> inline bool get(objectType& value) {
			byteType* objectPtr = (byteType*)&value;
			const sizeType numBytes = sizeof(value);
			if (m_offset + numBytes> m_len)
			return false;

			if (PlatformIsBigEndian == bufferIsBigEndian) {
				//copy -- matching endianness
				for (sizeType i = 0; i < numBytes; i++) {
					objectPtr[i] = m_data[m_offset + i];
				}
			} else {
				//swap -- endian mismatch
				for (sizeType i = 0; i < numBytes; i++) {
					objectPtr[i] = m_data[(m_offset + numBytes - 1) - i];
				}
			}
			m_offset += numBytes;

			return true;
		}

		/**
		 * Returns a pointer to the buffer you set in the constructor
		 * @return a pointer to m_data (the same pointer as specified in constructor)
		 */
		inline const byteType* getBuffer() {
			return m_data;
		}

		/**
		 * Gets an item of objectType from the buffer
		 * @param value destination object
		 * @return true if numBytes bytes are available to
		 * convert. false, otherwise
		 */
		template<class objectType> inline objectType get() {
			objectType temp;
			get(temp);
			return temp;
		}

		/**
		 * Gets a byte array from the buffer.
		 * @param outData buffer to put bytes into
		 * @param size the size of outData
		 * @return the number of bytes actually read
		 */
		inline sizeType getByteArray(byteType* outData, sizeType size) {
			if (m_offset + size> m_len) {
				size = getBytesRemaining();
			}

			for (sizeType i = 0; i < size; i++) {
				outData[i] = m_data[m_offset + i];
			}
			m_offset += size;

			return size;
		}

		/**
		 * Gets a null terminated string (byte array) from the buffer
		 * @param outData buffer to put bytes into
		 * @param size the size of outData
		 * @return the number of bytes actually read
		 */
		inline sizeType getNullTerminatedByteArray(byteType* outData, sizeType size) {
			if (m_offset + size> m_len) {
				size = getBytesRemaining();
			}

			sizeType i;
			for (i = 0; i < size && m_offset < m_len; i++) {
				outData[i] = m_data[m_offset + i];
				if (m_data[m_offset + i] == 0) {
					break;
				}
			}
			m_offset += i;

			return i;
		}

		/**
		 * Gets the number of bytes remaining in the buffer
		 *
		 * @return the number of bytes remaining in the buffer
		 */
		inline sizeType getBytesRemaining() {
			return m_len - m_offset;
		}

		/**
		 * Sets the internal offset to an absolute position
		 *
		 * @param offset the absolute offset you would like
		 * @return true if "offset" is a valid offset
		 */
		inline bool setOffset(sizeType offset) {
			if ((offset <= m_len) && (offset >= 0)) {
				m_offset = offset;
			} else {
				return false;
			}
			return true;
		}

		/**
		 * Gets the internal offset
		 *
		 * @return the current offset of the ByteBuffer
		 */
		inline sizeType getOffset() {
			return m_offset;
		}

		/**
		 * Moves the internal position pointer by "increment"
		 *
		 * @param increment number of bytes to move the buffer
		 * @return true if increment does not result in an invalid offset
		 */
		inline bool incOffset(sizeType increment) {
			return setOffset(m_offset + increment);
		}

		private:
		sizeType m_offset;
		sizeType m_len;
		const byteType *m_data;
	};

	//-------------------------------------------------------------------------
	// ByteOutputBuffer class definition
	//-------------------------------------------------------------------------
	template<class sizeType, class byteType, const bool bufferIsBigEndian> class ByteOutputBuffer {
		public:

		/**
		 * Constructor
		 * @param buf the buffer you are going to write data into
		 * @param len the length of buf
		 * Appropriate swaps will take place if the platfrom and buffer
		 * endian specs don't match
		 */
		ByteOutputBuffer(byteType* buf, sizeType len) :
		m_offset(0), m_len(len), m_data(buf) {
#ifndef BYTE_BUFFER_AVOID_TEMPLATE_CHECKS
			if(sizeof(byteType) != 1 ||  !std::numeric_limits<byteType>::is_integer) {
				BYTE_TYPE_SIZE_ERROR_ACTION;
			}
			if(!(std::numeric_limits<sizeType>::is_integer && std::numeric_limits<sizeType>::is_signed)) {
				SIZE_TYPE_ERROR_ACTION;
			}
#endif //BYTE_BUFFER_AVOID_TEMPLATE_CHECKS
		}

		/**
		 * Returns a pointer to the buffer you set in the constructor
		 * @return a pointer to m_data (the same pointer as specified in constructor)
		 */
		inline byteType* getBuffer() {
			return m_data;
		}

		/**
		 * Returns the number of bytes put into the buffer
		 * @return the number of bytes in the buffer (according to the internal offset)
		 */
		inline sizeType getSize() {
			return m_offset;
		}

		/**
		 * Puts an item of objectType into the buffer
		 * @param value destination object
		 * @return true if numBytes bytes where available to
		 * convert. false, otherwise
		 */
		template<class objectType> inline bool put(const objectType& value) {
			const byteType* objectPtr = (const byteType*)&value;
			const sizeType numBytes = sizeof(value);

			if (m_offset + numBytes> m_len)
			return false;

			if (PlatformIsBigEndian == bufferIsBigEndian) {
				//copy -- matching endianness
				for (sizeType i = 0; i < numBytes; i++) {
					m_data[m_offset + i] = objectPtr[i];
				}
			} else {
				//swap -- endian mismatch
				for (sizeType i = 0; i < numBytes; i++) {
					m_data[(m_offset + numBytes - 1) - i] = objectPtr[i];
				}
			}
			m_offset += numBytes;

			return true;
		}

		/**
		 * Puts a byte array into the buffer.
		 *
		 * @param inData a pointer to the data to copy into the buffer
		 * @param size the number of bytes to put in
		 * @return the number of bytes read from the buffer
		 */
		inline sizeType putByteArray(byteType* inData, sizeType size) {
			if (m_offset + size> m_len) {
				size = getBytesRemaining();
			}

			for (sizeType i = 0; i < size; i++) {
				m_data[m_offset + i] = inData[i];
			}
			m_offset += size;

			return size;
		}

		/**
		 * Puts a null terminated string (byte array) into the buffer
		 *
		 * @param inData a pointer to the data to copy into the buffer
		 * @param size the number of bytes to put in
		 * @return the number of bytes read from the buffer
		 */
		inline sizeType putNullTerminatedByteArray(byteType* inData, sizeType size) {
			if (m_offset + size> m_len) {
				size = getBytesRemaining();
			}

			sizeType i;
			for (i = 0; i < size && m_offset < m_len; i++) {
				m_data[m_offset + i] = inData[i];
				if (inData[i] == 0) {
					break;
				}
			}
			m_offset += i;

			return i;
		}

		/**
		 * Gets the number of bytes remaining in the buffer
		 *
		 * @return the number of bytes remaining in the buffer
		 */
		inline sizeType getBytesRemaining() {
			return m_len - m_offset;
		}

		/**
		 * Sets the internal offset to an absolute position
		 *
		 * @param offset the absolute offset you would like
		 * @return true if "offset" is a valid offset
		 */
		inline bool setOffset(sizeType offset) {
			if ((offset <= m_len) && (offset >= 0)) {
				m_offset = offset;
			} else {
				return false;
			}
			return true;
		}

		/**
		 * Gets the internal offset
		 *
		 * @return the current offset of the ByteBuffer
		 */
		inline sizeType getOffset() {
			return m_offset;
		}

		/**
		 * Moves the internal position pointer by "increment"
		 *
		 * @param increment number of bytes to move the buffer forward
		 * @return true if increment does not result in an invalid offset
		 */
		inline bool incOffset(sizeType increment) {
			return setOffset(m_offset + increment);
		}

		private:
		sizeType m_offset;
		sizeType m_len;
		byteType *m_data;
	};

} //end namespace byteBuffer

} // namespace ECTOS

#endif	//BYTE_BUFFER_H
