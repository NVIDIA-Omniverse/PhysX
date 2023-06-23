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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
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

OmniPvdCommand::Enum OMNI_PVD_CALL OmniPvdReaderImpl::getNextCommand()
{
	OmniPvdCommand::Enum cmdType = OmniPvdCommand::eINVALID;

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
					mStream->readBytes((uint8_t*)&mCmdClassNameLen, sizeof(uint16_t));
					mStream->readBytes((uint8_t*)mCmdClassName, mCmdClassNameLen);
					mCmdClassName[mCmdClassNameLen] = 0; // trailing zero
					mLog.outputLine("[parser] register class (handle: %llu, name: %s)\n", static_cast<unsigned long long>(mCmdClassHandle), mCmdClassName);
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
						mStream->readBytes((uint8_t*)&mCmdEnumValue, sizeof(uint32_t));
					}
					else if (mCmdAttributeDataType == OmniPvdDataType::eFLAGS_WORD)
					{
						mStream->readBytes((uint8_t*)&mCmdEnumClassHandle, sizeof(uint32_t));
					}
					else
					{
						mCmdEnumValue = 0;
						mStream->readBytes((uint8_t*)&mCmdAttributeNbElements, sizeof(uint32_t));
					}
					mStream->readBytes((uint8_t*)&mCmdAttributeNameLen, sizeof(uint16_t));
					mStream->readBytes((uint8_t*)mCmdAttributeName, mCmdAttributeNameLen);
					mCmdAttributeName[mCmdAttributeNameLen] = 0; // trailing zero
					mLog.outputLine("[parser] register attribute (classHandle: %llu, handle: %llu, dataType: %llu, nrFields: %llu, name: %s)\n", static_cast<unsigned long long>(mCmdClassHandle), static_cast<unsigned long long>(mCmdAttributeHandle), static_cast<unsigned long long>(mCmdAttributeDataType), static_cast<unsigned long long>(mCmdAttributeNbElements), mCmdAttributeName);
				}
				break;
				case OmniPvdCommand::eREGISTER_CLASS_ATTRIBUTE:
				{
					cmdType = OmniPvdCommand::eREGISTER_CLASS_ATTRIBUTE;
					mStream->readBytes((uint8_t*)&mCmdClassHandle, sizeof(OmniPvdClassHandle));
					mStream->readBytes((uint8_t*)&mCmdAttributeHandle, sizeof(OmniPvdAttributeHandle));
					mStream->readBytes((uint8_t*)&mCmdAttributeClassHandle, sizeof(OmniPvdClassHandle));
					mStream->readBytes((uint8_t*)&mCmdAttributeNameLen, sizeof(uint16_t));
					mStream->readBytes((uint8_t*)mCmdAttributeName, mCmdAttributeNameLen);
					mCmdAttributeName[mCmdAttributeNameLen] = 0; // trailing zero
					mLog.outputLine("[parser] register class attribute (classHandle: %llu, handle: %llu, classAttributeHandle: %llu, name: %s)\n", static_cast<unsigned long long>(mCmdClassHandle), static_cast<unsigned long long>(mCmdAttributeHandle), static_cast<unsigned long long>(mCmdAttributeClassHandle), mCmdAttributeName);
				}
				break;
				case OmniPvdCommand::eREGISTER_UNIQUE_LIST_ATTRIBUTE:
				{
					cmdType = OmniPvdCommand::eREGISTER_UNIQUE_LIST_ATTRIBUTE;
					mStream->readBytes((uint8_t*)&mCmdClassHandle, sizeof(OmniPvdClassHandle));
					mStream->readBytes((uint8_t*)&mCmdAttributeHandle, sizeof(OmniPvdAttributeHandle));
					mCmdAttributeDataType = readDataType(*mStream);
					mStream->readBytes((uint8_t*)&mCmdAttributeNameLen, sizeof(uint16_t));
					mStream->readBytes((uint8_t*)mCmdAttributeName, mCmdAttributeNameLen);
					mCmdAttributeName[mCmdAttributeNameLen] = 0; // trailing zero
					mLog.outputLine("[parser] register attributeSet (classHandle: %llu, handle: %llu, dataType: %llu, name: %s)\n", static_cast<unsigned long long>(mCmdClassHandle), static_cast<unsigned long long>(mCmdAttributeHandle), static_cast<unsigned long long>(mCmdAttributeDataType), mCmdAttributeName);
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
					mLog.outputLine("[parser] set attribute (contextHandle:%llu, objectHandle: %llu, handle: %llu, dataLen: %llu)\n", static_cast<unsigned long long>(mCmdContextHandle), static_cast<unsigned long long>(mCmdObjectHandle), static_cast<unsigned long long>(mCmdAttributeHandle), static_cast<unsigned long long>(mCmdAttributeDataLen));
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
					mLog.outputLine("[parser] add to attributeSet (contextHandle:%llu, objectHandle: %llu, attributeHandle: %llu, dataLen: %llu)\n", static_cast<unsigned long long>(mCmdContextHandle), static_cast<unsigned long long>(mCmdObjectHandle), static_cast<unsigned long long>(mCmdAttributeHandle), static_cast<unsigned long long>(mCmdAttributeDataLen));
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
					mLog.outputLine("[parser] remove from attributeSet (contextHandle:%llu, objectHandle: %llu, handle: %llu, dataLen: %llu)\n", static_cast<unsigned long long>(mCmdContextHandle), static_cast<unsigned long long>(mCmdObjectHandle), static_cast<unsigned long long>(mCmdAttributeHandle), static_cast<unsigned long long>(mCmdAttributeDataLen));
				}
				break;
				case OmniPvdCommand::eCREATE_OBJECT:
				{
					cmdType = OmniPvdCommand::eCREATE_OBJECT;
					mStream->readBytes((uint8_t*)&mCmdContextHandle, sizeof(OmniPvdContextHandle));
					mStream->readBytes((uint8_t*)&mCmdClassHandle, sizeof(OmniPvdClassHandle));
					mStream->readBytes((uint8_t*)&mCmdObjectHandle, sizeof(OmniPvdObjectHandle));
					mStream->readBytes((uint8_t*)&mCmdObjectNameLen, sizeof(uint16_t));
					if (mCmdObjectNameLen) {
						mStream->readBytes((uint8_t*)mCmdObjectName, mCmdObjectNameLen);
					}
					mCmdObjectName[mCmdObjectNameLen] = 0; // trailing zero
					mLog.outputLine("[parser] create object (contextHandle: %llu, classHandle: %llu, objectHandle: %llu, name: %s)\n", static_cast<unsigned long long>(mCmdContextHandle), static_cast<unsigned long long>(mCmdClassHandle), static_cast<unsigned long long>(mCmdObjectHandle), mCmdObjectName);
				}
				break;
				case OmniPvdCommand::eDESTROY_OBJECT:
				{
					cmdType = OmniPvdCommand::eDESTROY_OBJECT;
					mStream->readBytes((uint8_t*)&mCmdContextHandle, sizeof(OmniPvdContextHandle));
					mStream->readBytes((uint8_t*)&mCmdObjectHandle, sizeof(OmniPvdObjectHandle));
					mLog.outputLine("[parser] destroy object (contextHandle: %llu, objectHandle: %llu)\n", static_cast<unsigned long long>(mCmdContextHandle), static_cast<unsigned long long>(mCmdObjectHandle));
				}
				break;
				case OmniPvdCommand::eSTART_FRAME:
				{
					cmdType = OmniPvdCommand::eSTART_FRAME;
					mStream->readBytes((uint8_t*)&mCmdContextHandle, sizeof(OmniPvdContextHandle));
					mStream->readBytes((uint8_t*)&mCmdFrameTimeStart, sizeof(uint64_t));
					mLog.outputLine("[parser] start frame (contextHandle: %llu, timeStamp: %llu)\n", static_cast<unsigned long long>(mCmdContextHandle), static_cast<unsigned long long>(mCmdFrameTimeStart));
				}
				break;
				case OmniPvdCommand::eSTOP_FRAME:
				{
					cmdType = OmniPvdCommand::eSTOP_FRAME;
					mStream->readBytes((uint8_t*)&mCmdContextHandle, sizeof(OmniPvdContextHandle));
					mStream->readBytes((uint8_t*)&mCmdFrameTimeStop, sizeof(uint64_t));
					mLog.outputLine("[parser] stop frame (contextHandle: %llu, timeStamp: %llu)\n", static_cast<unsigned long long>(mCmdContextHandle), static_cast<unsigned long long>(mCmdFrameTimeStop));
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

uint32_t OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeHandle() {
	return mCmdAttributeHandle;
}

uint64_t OMNI_PVD_CALL OmniPvdReaderImpl::getObjectHandle() {
	return mCmdObjectHandle;
}

const char* OMNI_PVD_CALL OmniPvdReaderImpl::getClassName() {
	return mCmdClassName;
}

const char* OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeName() {
	return mCmdAttributeName;
}

const char* OMNI_PVD_CALL OmniPvdReaderImpl::getObjectName() {
	return mCmdObjectName;
}

const uint8_t* OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeDataPointer() {
	return mCmdAttributeDataPtr;
}

OmniPvdDataType::Enum OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeDataType() {
	return mCmdAttributeDataType;
}

uint32_t OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeDataLength() {
	return mCmdAttributeDataLen;
}

uint32_t OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeNumberElements() {
	return mCmdAttributeNbElements;
}

OmniPvdClassHandle OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeClassHandle() {
	return mCmdAttributeClassHandle;
}

uint8_t OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeNumberHandles() {
	return mCmdAttributeHandleDepth;
}

uint32_t* OMNI_PVD_CALL OmniPvdReaderImpl::getAttributeHandles() {
	return mCmdAttributeHandleStack;
}

uint64_t OMNI_PVD_CALL OmniPvdReaderImpl::getFrameTimeStart() {
	return mCmdFrameTimeStart;
}

uint64_t OMNI_PVD_CALL OmniPvdReaderImpl::getFrameTimeStop() {
	return mCmdFrameTimeStop;
}

OmniPvdClassHandle OMNI_PVD_CALL OmniPvdReaderImpl::getEnumClassHandle()
{
	return mCmdEnumClassHandle;
}

uint32_t OMNI_PVD_CALL OmniPvdReaderImpl::getEnumValue()
{
	return mCmdEnumValue;
}

void OmniPvdReaderImpl::readLongDataFromStream(uint32_t streamByteLen) {
	if (streamByteLen < 1) return;
	if (streamByteLen > mDataBuffAllocatedLen) {
		delete[] mDataBuffer;
		mDataBuffAllocatedLen = (uint32_t)(streamByteLen * 1.3f);
		mDataBuffer = new uint8_t[mDataBuffAllocatedLen];
		mCmdAttributeDataPtr = mDataBuffer;
	}
	mStream->readBytes(mCmdAttributeDataPtr, streamByteLen);
}
