// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef OMNI_PVD_READER_H
#define OMNI_PVD_READER_H

#include "OmniPvdDefines.h"
#include "OmniPvdCommands.h"
#include "OmniPvdReadStream.h"

#define OMNI_PVD_MAX_STRING_LENGTH   2048


/**
 * \brief Used to read debug information from an OmniPvdReadStream
 *
 * Using the getNextCommand function in a while loop for example one can traverse the stream one command after another. Given the command, different functions below will be available.
 *
 * Using the OmniPvdCommand::Enum one can determine the type of command and like that use the appropriate get functions to extract the payload from the command.
 */

class OmniPvdReader
{
public:
	virtual ~OmniPvdReader()
	{
	}

	/**
	 * \brief Sets the log function to print the internal debug messages of the OmniPVD Reader instance
	 *
	 * \param logFunction The function pointer to receive the log messages
	 */
	virtual void OMNI_PVD_CALL setLogFunction(OmniPvdLogFunction logFunction) = 0;
	
	/**
	 * \brief Sets the read stream that contains the OmniPVD API command stream
	 *
	 * \param stream The OmniPvdReadStream that holds the stream of API calls/notifications
	 */
	virtual void OMNI_PVD_CALL setReadStream(OmniPvdReadStream& stream) = 0;	

	/**
	 * \brief Extracts the versions from the binary file to read and tests if the file is older or equal to that of the reader.
	 *
	 * \param majorVersion The major versions of the stream
	 * \param minorVersion The minor versions of the stream
	 * \param patch The patch number of the stream
	 * \return If the reading was possible to start or not
	 */
	virtual bool OMNI_PVD_CALL startReading(OmniPvdVersionType& majorVersion, OmniPvdVersionType& minorVersion, OmniPvdVersionType& patch) = 0;
	
	/**
	 * \brief The heartbeat function of the reader class. As long as the command that is returned is not equal to OmniPvdCommand::eINVALID, then one can safely extract the data fields from the command.
	 *
	 * \return The command enum type
	 */
	virtual OmniPvdCommand::Enum OMNI_PVD_CALL getNextCommand() = 0;

	/**
	 * \brief Returns the major version of the stream
	 *
	 * \return The major version
	 */
	virtual OmniPvdVersionType OMNI_PVD_CALL getMajorVersion() = 0;
	
	/**
	 * \brief Returns the minor version of the stream
	 *
	 * \return The minor version
	 */
	virtual OmniPvdVersionType OMNI_PVD_CALL getMinorVersion() = 0;
	
	/**
	 * \brief Returns the patch number of the stream
	 *
	 * \return The patch value
	 */
	virtual OmniPvdVersionType OMNI_PVD_CALL getPatch() = 0;

	/**
	 * \brief Returns the context handle of the latest command, if it had one, else OMNI_PVD_INVALID_HANDLE
	 *
	 * \return The context handle of the latest command
	 */
	virtual OmniPvdContextHandle OMNI_PVD_CALL getContextHandle() = 0;
	
	/**
	 * \brief Returns the object handle of the latest command, if it had one, else OMNI_PVD_INVALID_HANDLE
	 *
	 * \return The object handle of the latest command
	 */
	virtual OmniPvdObjectHandle OMNI_PVD_CALL getObjectHandle() = 0;
	
	/**
	 * \brief Returns the class handle of the latest command, if it had one, else OMNI_PVD_INVALID_HANDLE
	 *
	 * \return The class handle of the latest command
	 */
	virtual OmniPvdClassHandle OMNI_PVD_CALL getClassHandle() = 0;
	
	/**
	 * \brief Returns the base class handle of the latest command, if it had one, else OMNI_PVD_INVALID_HANDLE
	 *
	 * \return The base class handle of the latest command
	 */
	virtual OmniPvdClassHandle OMNI_PVD_CALL getBaseClassHandle() = 0;
	
	/**
	 * \brief Returns the attribute handle of the latest command, if it had one, else OMNI_PVD_INVALID_HANDLE
	 *
	 * \return The attribute handle of the latest command
	 */
	virtual OmniPvdAttributeHandle OMNI_PVD_CALL getAttributeHandle() = 0;
	
	/**
	 * \brief Returns the class name of the latest command, if it had one, else a null terminated string of length 0
	 *
	 * \return The string containing the class name
	 */
	virtual const char* OMNI_PVD_CALL getClassName() = 0;
	
	/**
	 * \brief Returns the attribute name of the latest command, if it had one, else a null terminated string of length 0
	 *
	 * \return The string containing the attribute name
	 */
	virtual const char* OMNI_PVD_CALL getAttributeName() = 0;
	
	/**
	 * \brief Returns the object name of the latest command, if it had one, else a null terminated string of length 0
	 *
	 * \return The string containing the object name
	 */
	virtual const char* OMNI_PVD_CALL getObjectName() = 0;

	/**
	 * \brief Returns the attribute data pointer, if it had one, else returns 0
	 *
	 * \return The array containing the attribute data
	 */
	virtual const uint8_t* OMNI_PVD_CALL getAttributeDataPointer() = 0;
	
	/**
	 * \brief Returns the attribute data type, the data type is undefined if the last command did not contain attribute data
	 *
	 * \return The attribute data type
	 */
	virtual OmniPvdDataType::Enum OMNI_PVD_CALL getAttributeDataType() = 0;
	
	/**
	 * \brief Returns the attribute data length, the data length of the last command if it was defined, otherwise returns 0
	 *
	 * \return The attribute data length
	 */
	virtual uint32_t OMNI_PVD_CALL getAttributeDataLength() = 0;
	
	/**
	 * \brief Returns the number of elements contained in the last set operation, if they were defined, otherwise returns 0
	 *
	 * \return The number of elements
	 */
	virtual uint32_t OMNI_PVD_CALL getAttributeNumberElements() = 0;
	
	/**
	 * \brief Returns the numberclass handle of the attribute class, if it was defined, else returns OMNI_PVD_INVALID_HANDLE
	 *
	 * \return The attibute class handle
	 */
	virtual OmniPvdClassHandle OMNI_PVD_CALL getAttributeClassHandle() = 0;

	/**
	 * \brief Returns the frame start value
	 *
	 * \return The frame ID value
	 */
	virtual uint64_t OMNI_PVD_CALL getFrameTimeStart() = 0;
	
	/**
	 * \brief Returns the frame stop value
	 *
	 * \return The frame ID value
	 */
	virtual uint64_t OMNI_PVD_CALL getFrameTimeStop() = 0;

	/**
	 * \brief Returns data for the last message received.
	 *
	 * \param message A handle to the message string is returned.
	 * \param file A handle to the string containing the file name the message originated from is returned.
	 * \param line A handle to the line number in the file where the message originated from is returned.
	 * \param type A handle to the message type is returned.
	 * \param handle A handle to the Omni PVD class handle is returned.
	*/
	virtual bool OMNI_PVD_CALL getMessageData(const char*& message, const char*& file, uint32_t& line, uint32_t& type, OmniPvdClassHandle& handle) = 0;

	/**
	 * \brief Returns the class handle containing the enum values, if it was defined, else returns OMNI_PVD_INVALID_HANDLE
	 *
	 * \return The enum class handle
	 */
	virtual OmniPvdClassHandle OMNI_PVD_CALL getEnumClassHandle() = 0;
	
	/**
	 * \brief Returns the enum value for a specific flag, if it was defined, else returns 0
	 *
	 * \return The enum value
	 */
	virtual OmniPvdEnumValueType OMNI_PVD_CALL getEnumValue() = 0;
};

#endif
