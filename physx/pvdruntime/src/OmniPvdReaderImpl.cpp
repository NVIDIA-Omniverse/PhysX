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

#include "OmniPvdDefinesInternal.h"
#include "OmniPvdReaderImpl.h"

#include <inttypes.h>


OmniPvdReaderImpl::OmniPvdReaderImpl()
{
	mMajorVersion = OMNI_PVD_VERSION_MAJOR;
	mMinorVersion = OMNI_PVD_VERSION_MINOR;
	mPatch = OMNI_PVD_VERSION_PATCH;
	mDataBuffer = 0;
	mDataBuffAllocatedLen = 0;
	mCmdAttributeDataPtr = 0;
	mIsReadingStarted = false;
	mReadBaseClassHandle = 1;
}

OmniPvdReaderImpl::~OmniPvdReaderImpl()
{
	mCmdAttributeDataPtr = 0;
	delete[] mDataBuffer;
	mDataBuffer = 0;
	mDataBuffAllocatedLen = 0;
}

void OMNI_PVD_CALL OmniPvdReaderImpl::setLogFunction(OmniPvdLogFunction logFunction)
{
	mLog.setLogFunction(logFunction);
}

void OMNI_PVD_CALL OmniPvdReaderImpl::setReadStream(OmniPvdReadStream& stream)
{
	mStream = &stream;
	mStream->openStream();
}

bool OMNI_PVD_CALL OmniPvdReaderImpl::startReading(OmniPvdVersionType& majorVersion, OmniPvdVersionType& minorVersion, OmniPvdVersionType& patch)
{
	if (mIsReadingStarted)
	{
		return true;
	}
	if (mStream)
	{
		mStream->readBytes((uint8_t*)&majorVersion, sizeof(OmniPvdVersionType));
		mStream->readBytes((uint8_t*)&minorVersion, sizeof(OmniPvdVersionType));
		mStream->readBytes((uint8_t*)&patch, sizeof(OmniPvdVersionType));

		mCmdMajorVersion = majorVersion;
		mCmdMinorVersion = minorVersion;
		mCmdPatch = patch;

		if ((mCmdMajorVersion == 0) && (mCmdMinorVersion < 3))
		{
			mCmdBaseClassHandle = 0;
			mReadBaseClassHandle = 0;
		}
		else
		{
			mCmdBaseClassHandle = 0;
			mReadBaseClassHandle = 1;
		}

		mLog.outputLine("OmniPvdRuntimeReaderImpl::startReading majorVersion(%lu), minorVersion(%lu), patch(%lu)", static_cast<unsigned long>(majorVersion), static_cast<unsigned long>(minorVersion), static_cast<unsigned long>(patch));
		if (majorVersion > mMajorVersion)
		{
			mLog.outputLine("[parser] major version too new\n");
			return false;
		}
		else if (majorVersion == mMajorVersion)
		{
			if (minorVersion > mMinorVersion)
			{				
				mLog.outputLine("[parser] minor version too new\n");
				return false;
			}
			else if (minorVersion == mMinorVersion)
			{
				if (patch > mPatch)
				{
					mLog.outputLine("[parser] patch too new\n");
					return false;
				}
			}			
		}
		mIsReadingStarted = true;
		return true;
	}
	else {
		return false;
	}
	
}

static OmniPvdDataType::Enum readDataType(OmniPvdReadStream& stream)
{
	OmniPvdDataTypeStorageType dataType;
	stream.readBytes((uint8_t*)&dataType, sizeof(OmniPvdDataTypeStorageType));
	return static_cast<OmniPvdDataType::Enum>(dataType);
}

bool OmniPvdReaderImpl::readStringFromStream(char* string, uint16_t& stringLength)
{
	mStream->readBytes((uint8_t*)&stringLength, sizeof(uint16_t));

	if (stringLength < OMNI_PVD_MAX_STRING_LENGTH)
	{
		mStream->readBytes((uint8_t*)string, stringLength);
		string[stringLength] = 0; // trailing zero

		return true;
	}
	else
	{
		uint16_t readBytes = OMNI_PVD_MAX_STRING_LENGTH - 1;

		mStream->readBytes((uint8_t*)string, readBytes);
		string[readBytes] = 0; // trailing zero
		mStream->skipBytes(stringLength - readBytes);

		mLog.outputLine("[parser] ERROR: string name %s... exceeds max length of %d\n", string, OMNI_PVD_MAX_STRING_LENGTH);
		return false;
	}
}

OmniPvdCommand::Enum OMNI_PVD_CALL OmniPvdReaderImpl::getNextCommand()
{
	OmniPvdCommand::Enum cmdType = OmniPvdCommand::eINVALID;
	resetCommandParams();
	if (!mIsReadingStarted)
	{
		if (!startReading(mCmdMajorVersion, mCmdMinorVersion, mCmdPatch))
		{
			return cmdType;
		}
	}	
	if (mStream) {
		OmniPvdCommandStorageType command;
		if (mStream->readBytes(&command, sizeof(OmniPvdCommandStorageType)))
		{
			mCmdMessageParsed = false;

			switch (command) {
				case OmniPvdCommand::eREGISTER_CLASS:
				{
					cmdType = OmniPvdCommand::eREGISTER_CLASS;
					mStream->readBytes((uint8_t*)&mCmdClassHandle, sizeof(OmniPvdClassHandle));
					////////////////////////////////////////////////////////////////////////////////
					// Skip reading the base class if the stream is older or equal to (0,2,x)
					////////////////////////////////////////////////////////////////////////////////
					if (mReadBaseClassHandle)
					{
						mStream->readBytes((uint8_t*)&mCmdBaseClassHandle, sizeof(OmniPvdClassHandle));
					}

					readStringFromStream(mCmdClassName, mCmdClassNameLen);
					mLog.outputLine("[parser] register class (handle: %d, name: %s)\n", mCmdClassHandle, mCmdClassName);
				}
				break;
				case OmniPvdCommand::eREGISTER_ATTRIBUTE:
				{
					cmdType = OmniPvdCommand::eREGISTER_ATTRIBUTE;
					mStream->readBytes((uint8_t*)&mCmdClassHandle, sizeof(OmniPvdClassHandle));
					mStream->readBytes((uint8_t*)&mCmdAttributeHandle, sizeof(OmniPvdAttributeHandle));
					mCmdAttributeDataType = readDataType(*mStream);
					if (mCmdAttributeDataType == OmniPvdDataType::eENUM_VALUE)
					{
						mStream->readBytes((uint8_t*)&mCmdEnumValue, sizeof(OmniPvdEnumValueType));
					}
					else if (mCmdAttributeDataType == OmniPvdDataType::eFLAGS_WORD)
					{
						mStream->readBytes((uint8_t*)&mCmdEnumClassHandle, sizeof(OmniPvdClassHandle));
					}
					else
					{
						mCmdEnumValue = 0;
						mStream->readBytes((uint8_t*)&mCmdAttributeNbElements, sizeof(uint32_t));
					}

					readStringFromStream(mCmdAttributeName, mCmdAttributeNameLen);
					mLog.outputLine("[parser] register attribute (classHandle: %d, attributeHandle: %d, attributeDataType: %d, nrFields: %d, name: %s)\n", mCmdClassHandle, mCmdAttributeHandle, mCmdAttributeDataType, mCmdAttributeNbElements, mCmdAttributeName);
				}
				break;
				case OmniPvdCommand::eREGISTER_CLASS_ATTRIBUTE:
				{
					cmdType = OmniPvdCommand::eREGISTER_CLASS_ATTRIBUTE;
					mStream->readBytes((uint8_t*)&mCmdClassHandle, sizeof(OmniPvdClassHandle));
					mStream->readBytes((uint8_t*)&mCmdAttributeHandle, sizeof(OmniPvdAttributeHandle));
					mStream->readBytes((uint8_t*)&mCmdAttributeClassHandle, sizeof(OmniPvdClassHandle));

					readStringFromStream(mCmdAttributeName, mCmdAttributeNameLen);
					mLog.outputLine("[parser] register class attribute (classHandle: %d, attributeHandle: %d, attribute classAttributeHandle: %d, name: %s)\n", mCmdClassHandle, mCmdAttributeHandle, mCmdAttributeClassHandle, mCmdAttributeName);
				}
				break;
				case OmniPvdCommand::eREGISTER_UNIQUE_LIST_ATTRIBUTE:
				{
					cmdType = OmniPvdCommand::eREGISTER_UNIQUE_LIST_ATTRIBUTE;
					mStream->readBytes((uint8_t*)&mCmdClassHandle, sizeof(OmniPvdClassHandle));
					mStream->readBytes((uint8_t*)&mCmdAttributeHandle, sizeof(OmniPvdAttributeHandle));
					mCmdAttributeDataType = readDataType(*mStream);

					readStringFromStream(mCmdAttributeName, mCmdAttributeNameLen);
					mLog.outputLine("[parser] register attributeSet (classHandle: %d, attributeHandle: %d, attributeDataType: %d, name: %s)\n", mCmdClassHandle, mCmdAttributeHandle, mCmdAttributeDataType, mCmdAttributeName);
				}
				break;
				case OmniPvdCommand::eSET_ATTRIBUTE:
				{
					cmdType = OmniPvdCommand::eSET_ATTRIBUTE;
					mStream->readBytes((uint8_t*)&mCmdContextHandle, sizeof(OmniPvdContextHandle));
					mStream->readBytes((uint8_t*)&mCmdObjectHandle, sizeof(OmniPvdObjectHandle));
					mStream->readBytes((uint8_t*)&mCmdAttributeHandleDepth, sizeof(uint8_t));
					uint32_t* attributeHandleStack = mCmdAttributeHandleStack;
					for (int i = 0; i < mCmdAttributeHandleDepth; i++) {
						mStream->readBytes((uint8_t*)&mCmdAttributeHandle, sizeof(OmniPvdAttributeHandle));
						attributeHandleStack++;
					}
					mStream->readBytes((uint8_t*)&mCmdAttributeDataLen, sizeof(uint32_t));
					readLongDataFromStream(mCmdAttributeDataLen);
					mLog.outputLine("[parser] set attribute (contextHandle:%d, objectHandle: %d, attributeHandle: %d, dataLen: %d)\n", mCmdContextHandle, mCmdObjectHandle, mCmdAttributeHandle, mCmdAttributeDataLen);
				}
				break;
				case OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE:
				{
					cmdType = OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE;
					mStream->readBytes((uint8_t*)&mCmdContextHandle, sizeof(OmniPvdContextHandle));
					mStream->readBytes((uint8_t*)&mCmdObjectHandle, sizeof(OmniPvdObjectHandle));
					mStream->readBytes((uint8_t*)&mCmdAttributeHandleDepth, sizeof(uint8_t));
					uint32_t* attributeHandleStack = mCmdAttributeHandleStack;
					for (int i = 0; i < mCmdAttributeHandleDepth; i++) {
						mStream->readBytes((uint8_t*)&mCmdAttributeHandle, sizeof(OmniPvdAttributeHandle));
						attributeHandleStack++;
					}
					mStream->readBytes((uint8_t*)&mCmdAttributeDataLen, sizeof(uint32_t));
					readLongDataFromStream(mCmdAttributeDataLen);
					mLog.outputLine("[parser] add to attributeSet (contextHandle:%d, objectHandle: %d, attributeHandle: %d, dataLen: %d)\n", mCmdContextHandle, mCmdObjectHandle, mCmdAttributeHandle, mCmdAttributeDataLen);
				}
				break;
				case OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE:
				{
					cmdType = OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE;
					mStream->readBytes((uint8_t*)&mCmdContextHandle, sizeof(OmniPvdContextHandle));
					mStream->readBytes((uint8_t*)&mCmdObjectHandle, sizeof(OmniPvdObjectHandle));
					mStream->readBytes((uint8_t*)&mCmdAttributeHandleDepth, sizeof(uint8_t));
					uint32_t* attributeHandleStack = mCmdAttributeHandleStack;
					for (int i = 0; i < mCmdAttributeHandleDepth; i++) {
						mStream->readBytes((uint8_t*)&mCmdAttributeHandle, sizeof(OmniPvdAttributeHandle));
						attributeHandleStack++;
					}
					mStream->readBytes((uint8_t*)&mCmdAttributeDataLen, sizeof(uint32_t));
					readLongDataFromStream(mCmdAttributeDataLen);
					mLog.outputLine("[parser] remove from attributeSet (contextHandle:%d, objectHandle: %d, attributeHandle: %d, dataLen: %d)\n", mCmdContextHandle, mCmdObjectHandle, mCmdAttributeHandle, mCmdAttributeDataLen);
				}
				break;
				case OmniPvdCommand::eCREATE_OBJECT:
				{
					cmdType = OmniPvdCommand::eCREATE_OBJECT;
					mStream->readBytes((uint8_t*)&mCmdContextHandle, sizeof(OmniPvdContextHandle));
					mStream->readBytes((uint8_t*)&mCmdClassHandle, sizeof(OmniPvdClassHandle));
					mStream->readBytes((uint8_t*)&mCmdObjectHandle, sizeof(OmniPvdObjectHandle));

					readStringFromStream(mCmdObjectName, mCmdObjectNameLen);
					mLog.outputLine("[parser] create object (contextHandle: %d, classHandle: %d, objectHandle: %d, name: %s)\n", mCmdContextHandle, mCmdClassHandle, mCmdObjectHandle, mCmdObjectName);
				}
				break;
				case OmniPvdCommand::eDESTROY_OBJECT:
				{
					cmdType = OmniPvdCommand::eDESTROY_OBJECT;
					mStream->readBytes((uint8_t*)&mCmdContextHandle, sizeof(OmniPvdContextHandle));
					mStream->readBytes((uint8_t*)&mCmdObjectHandle, sizeof(OmniPvdObjectHandle));
					mLog.outputLine("[parser] destroy object (contextHandle: %d, objectHandle: %d)\n", mCmdContextHandle, mCmdObjectHandle);
				}
				break;
				case OmniPvdCommand::eSTART_FRAME:
				{
					cmdType = OmniPvdCommand::eSTART_FRAME;
					mStream->readBytes((uint8_t*)&mCmdContextHandle, sizeof(OmniPvdContextHandle));
					mStream->readBytes((uint8_t*)&mCmdFrameTimeStart, sizeof(uint64_t));
					mLog.outputLine("[parser] start frame (contextHandle: %d, timeStamp: %d)\n", mCmdContextHandle, mCmdFrameTimeStart);
				}
				break;
				case OmniPvdCommand::eSTOP_FRAME:
				{
					cmdType = OmniPvdCommand::eSTOP_FRAME;
					mStream->readBytes((uint8_t*)&mCmdContextHandle, sizeof(OmniPvdContextHandle));
					mStream->readBytes((uint8_t*)&mCmdFrameTimeStop, sizeof(uint64_t));
					mLog.outputLine("[parser] stop frame (contextHandle: %d, timeStamp: %d)\n", mCmdContextHandle, mCmdFrameTimeStop);
				}
				break;
				case OmniPvdCommand::eRECORD_MESSAGE:
				{
					cmdType = OmniPvdCommand::eRECORD_MESSAGE;
					mStream->readBytes((uint8_t*)&mCmdContextHandle, sizeof(OmniPvdContextHandle));

					// Message.
					readStringFromStream(mCmdMessage, mCmdMessageLength);

					// File name.
					readStringFromStream(mCmdMessageFile, mCmdMessageFileLength);

					mStream->readBytes((uint8_t*)&mCmdMessageLine, sizeof(uint32_t));
					mStream->readBytes((uint8_t*)&mCmdMessageType, sizeof(uint32_t));
					mStream->readBytes((uint8_t*)&mCmdMessageClassHandle, sizeof(OmniPvdClassHandle));

					mCmdMessageParsed = true;

					mLog.outputLine("[parser] message (contextHandle: %d, message: %s, file: %s, line: %d, type: %d)\n", 
						mCmdContextHandle, 
						mCmdMessage, mCmdMessageFile, mCmdMessageLine, mCmdMessageType
					);
				}
				break;
				default:
				{
				}
				break;
				}
			return cmdType;
		} else {
			return cmdType;
		}
	}
	else {
		return cmdType;
	}
}

uint32_t OMNI_PVD_CALL OmniPvdReaderImpl::getMajorVersion()
{
	return mCmdMajorVersion;
}

uint32_t OMNI_PVD_CALL OmniPvdReaderImpl::getMinorVersion()
{
	return mCmdMinorVersion;
}

uint32_t OMNI_PVD_CALL OmniPvdReaderImpl::getPatch()
{
	return mCmdPatch;
}

uint64_t OMNI_PVD_CALL OmniPvdReaderImpl::getContextHandle()
{
	return mCmdContextHandle;
}

OmniPvdClassHandle OMNI_PVD_CALL OmniPvdReaderImpl::getClassHandle()
{
	return mCmdClassHandle;
}

OmniPvdClassHandle OMNI_PVD_CALL OmniPvdReaderImpl::getBaseClassHandle()
{
	return mCmdBaseClassHandle;
}

uint32_t OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeHandle()
{
	return mCmdAttributeHandle;
}

uint64_t OMNI_PVD_CALL OmniPvdReaderImpl::getObjectHandle()
{
	return mCmdObjectHandle;
}

const char* OMNI_PVD_CALL OmniPvdReaderImpl::getClassName()
{
	return mCmdClassName;
}

const char* OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeName()
{
	return mCmdAttributeName;
}

const char* OMNI_PVD_CALL OmniPvdReaderImpl::getObjectName()
{
	return mCmdObjectName;
}

const uint8_t* OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeDataPointer()
{
	return mCmdAttributeDataPtr;
}

OmniPvdDataType::Enum OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeDataType()
{
	return mCmdAttributeDataType;
}

uint32_t OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeDataLength()
{
	return mCmdAttributeDataLen;
}

uint32_t OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeNumberElements()
{
	return mCmdAttributeNbElements;
}

OmniPvdClassHandle OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeClassHandle()
{
	return mCmdAttributeClassHandle;
}

uint8_t OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeNumberHandles()
{
	return mCmdAttributeHandleDepth;
}

uint32_t* OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeHandles()
{
	return mCmdAttributeHandleStack;
}

uint64_t OMNI_PVD_CALL OmniPvdReaderImpl::getFrameTimeStart()
{
	return mCmdFrameTimeStart;
}

uint64_t OMNI_PVD_CALL OmniPvdReaderImpl::getFrameTimeStop()
{
	return mCmdFrameTimeStop;
}

bool OMNI_PVD_CALL OmniPvdReaderImpl::getMessageData(const char*& message, const char*& file, uint32_t& line, uint32_t& type, OmniPvdClassHandle& handle)
{
	message = mCmdMessage;
	file = mCmdMessageFile;
	line = mCmdMessageLine;
	type = mCmdMessageType;
	handle = mCmdMessageClassHandle;

	return mCmdMessageParsed;
}

OmniPvdClassHandle OMNI_PVD_CALL OmniPvdReaderImpl::getEnumClassHandle()
{
	return mCmdEnumClassHandle;
}

uint32_t OMNI_PVD_CALL OmniPvdReaderImpl::getEnumValue()
{
	return mCmdEnumValue;
}

void OmniPvdReaderImpl::readLongDataFromStream(uint32_t streamByteLen)
{
	if (streamByteLen < 1) return;
	if (streamByteLen > mDataBuffAllocatedLen) {
		delete[] mDataBuffer;
		mDataBuffAllocatedLen = (uint32_t)(streamByteLen * 1.3f);
		mDataBuffer = new uint8_t[mDataBuffAllocatedLen];
	}
	mCmdAttributeDataPtr = mDataBuffer;
	mStream->readBytes(mCmdAttributeDataPtr, streamByteLen);
}

////////////////////////////////////////////////////////////////////////////////
// Resets all command parameters before a new read command is executed,
// except for those parameters that are "stateful" which are left commented out.
////////////////////////////////////////////////////////////////////////////////
void OmniPvdReaderImpl::resetCommandParams()
{	
	////////////////////////////////////////////////////////////////////////////////
	// Stateful: Depends on the version of the stream reader
	////////////////////////////////////////////////////////////////////////////////
	// OmniPvdVersionType mMajorVersion;
	// OmniPvdVersionType mMinorVersion;
	// OmniPvdVersionType mPatch;
	////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////////
	// Stateful: Depends on the stream being read and stay the same
	////////////////////////////////////////////////////////////////////////////////
	// OmniPvdVersionType mCmdMajorVersion;
	// OmniPvdVersionType mCmdMinorVersion;
	// OmniPvdVersionType mCmdPatch;
	////////////////////////////////////////////////////////////////////////////////

	mCmdContextHandle = OMNI_PVD_INVALID_HANDLE;
	mCmdObjectHandle = OMNI_PVD_INVALID_HANDLE;

	mCmdClassHandle = OMNI_PVD_INVALID_HANDLE;
	mCmdBaseClassHandle = OMNI_PVD_INVALID_HANDLE;
	mCmdAttributeHandle = OMNI_PVD_INVALID_HANDLE;
		
	mCmdClassName[0] = 0;
	mCmdAttributeName[0] = 0;
	mCmdObjectName[0] = 0;

	mCmdClassNameLen = 0;
	mCmdAttributeNameLen = 0;
	mCmdObjectNameLen = 0;

	mCmdAttributeDataPtr = 0;

	////////////////////////////////////////////////////////////////////////////////
	// Unsure how to handle this, a zero value could still be valid
	////////////////////////////////////////////////////////////////////////////////
	//mCmdAttributeDataType = OmniPvdDataType::eINT8; // int 8 is 0
	////////////////////////////////////////////////////////////////////////////////
	
	mCmdAttributeDataLen = 0;
	mCmdAttributeNbElements = 0;
	mCmdEnumValue = 0;
	mCmdEnumClassHandle = OMNI_PVD_INVALID_HANDLE;
	mCmdAttributeClassHandle = OMNI_PVD_INVALID_HANDLE;

	////////////////////////////////////////////////////////////////////////////////
	// Left untouched/dirty : instead depends on the depth parameter below
	////////////////////////////////////////////////////////////////////////////////
	// OmniPvdAttributeHandle mCmdAttributeHandleStack[32];
	////////////////////////////////////////////////////////////////////////////////
	mCmdAttributeHandleDepth = 0;

	////////////////////////////////////////////////////////////////////////////////
	// Stateful for the duration of a frame
	////////////////////////////////////////////////////////////////////////////////
	// uint64_t mCmdFrameTimeStart;
	// uint64_t mCmdFrameTimeStop;
	////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////////
	// Internal scratch buffer, instead the mCmdAttributeDataPtr is set to 0
	////////////////////////////////////////////////////////////////////////////////
	// uint8_t *mDataBuffer;
	// uint32_t mDataBuffAllocatedLen;
	////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////////
	// Internal message buffer
	////////////////////////////////////////////////////////////////////////////////
	mCmdMessageParsed = false;
	mCmdMessageLength = 0;
	mCmdMessage[0] = 0;
	mCmdMessageFileLength = 0;
	mCmdMessageFile[0] = 0;
	mCmdMessageLine = 0;
	mCmdMessageType = 0;
	mCmdMessageClassHandle = 0;
}