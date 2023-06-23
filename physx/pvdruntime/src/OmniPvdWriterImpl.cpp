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
#include "OmniPvdWriterImpl.h"
#include "OmniPvdCommands.h"
#include <string.h>

OmniPvdWriterImpl::OmniPvdWriterImpl()
{
	mLastClassHandle = 0;
	mLastAttributeHandle = 0;
	mIsFirstWrite = true;
	mStream = 0;
}

OmniPvdWriterImpl::~OmniPvdWriterImpl()
{
}

void OMNI_PVD_CALL OmniPvdWriterImpl::setLogFunction(OmniPvdLogFunction logFunction)
{
	mLog.setLogFunction(logFunction);
}

void OmniPvdWriterImpl::setVersionHelper()
{
	if (mStream && mIsFirstWrite)
	{
		const OmniPvdVersionType omniPvdVersionMajor = OMNI_PVD_VERSION_MAJOR;
		const OmniPvdVersionType omniPvdVersionMinor = OMNI_PVD_VERSION_MINOR;
		const OmniPvdVersionType omniPvdVersionPatch = OMNI_PVD_VERSION_PATCH;
		setVersion(omniPvdVersionMajor, omniPvdVersionMinor, omniPvdVersionPatch);
	}
}

void OmniPvdWriterImpl::setVersion(OmniPvdVersionType majorVersion, OmniPvdVersionType minorVersion, OmniPvdVersionType patch)
{	
	if (mStream && mIsFirstWrite)
	{ 
		if (!mStream->openStream())
		{
			return;
		}
		mStream->writeBytes((const uint8_t*)&majorVersion, sizeof(OmniPvdVersionType));
		mStream->writeBytes((const uint8_t*)&minorVersion, sizeof(OmniPvdVersionType));
		mStream->writeBytes((const uint8_t*)&patch, sizeof(OmniPvdVersionType));

		mLog.outputLine("OmniPvdRuntimeWriterImpl::setVersion majorVersion(%lu), minorVersion(%lu), patch(%lu)", static_cast<unsigned long>(majorVersion), static_cast<unsigned long>(minorVersion), static_cast<unsigned long>(patch));
		
		mIsFirstWrite = false;
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::setWriteStream(OmniPvdWriteStream& stream)
{
	mLog.outputLine("OmniPvdRuntimeWriterImpl::setWriteStream");
	mStream = &stream;
}

OmniPvdWriteStream* OMNI_PVD_CALL OmniPvdWriterImpl::getWriteStream()
{
	return mStream;
}

static void writeCommand(OmniPvdWriteStream& stream, OmniPvdCommand::Enum command)
{
	const OmniPvdCommandStorageType commandTmp = static_cast<OmniPvdCommandStorageType>(command);
	stream.writeBytes((const uint8_t*)&commandTmp, sizeof(OmniPvdCommandStorageType));
}

OmniPvdClassHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerClass(const char* className, OmniPvdClassHandle baseClass)
{
	setVersionHelper();
	if (mStream)
	{
		mLog.outputLine("OmniPvdWriterImpl::registerClass className(%s)", className);

		int classNameLen = (int)strlen(className);
		writeCommand(*mStream, OmniPvdCommand::eREGISTER_CLASS);
		mLastClassHandle++;
		mStream->writeBytes((const uint8_t*)&mLastClassHandle, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((const uint8_t*)&baseClass, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((const uint8_t*)&classNameLen, sizeof(uint16_t));
		mStream->writeBytes((const uint8_t*)className, classNameLen);
		return mLastClassHandle;
	} else {
		return 0;
	}	
}

static void writeDataType(OmniPvdWriteStream& stream, OmniPvdDataType::Enum attributeDataType)
{
	const OmniPvdDataTypeStorageType dataType = static_cast<OmniPvdDataTypeStorageType>(attributeDataType);
	stream.writeBytes((const uint8_t*)&dataType, sizeof(OmniPvdDataTypeStorageType));
}

OmniPvdAttributeHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdDataType::Enum attributeDataType, uint32_t nbElements)
{
	setVersionHelper();
	if (mStream) {

		mLog.outputLine("OmniPvdWriterImpl::registerAttribute classHandle(%llu), attributeName(%s), attributeDataType(%d), nbrFields(%llu)", static_cast<unsigned long long>(classHandle), attributeName, static_cast<int>(attributeDataType), static_cast<unsigned long long>(nbElements));

		int attribNameLen = (int)strlen(attributeName);
		writeCommand(*mStream, OmniPvdCommand::eREGISTER_ATTRIBUTE);
		mLastAttributeHandle++;
		mStream->writeBytes((const uint8_t*)&classHandle, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((const uint8_t*)&mLastAttributeHandle, sizeof(OmniPvdAttributeHandle));
		writeDataType(*mStream, attributeDataType);
		mStream->writeBytes((const uint8_t*)&nbElements, sizeof(uint32_t));
		mStream->writeBytes((const uint8_t*)&attribNameLen, sizeof(uint16_t));
		mStream->writeBytes((const uint8_t*)attributeName, attribNameLen);
		return mLastAttributeHandle;
	}
	else {
		return 0;
	}
}

OmniPvdAttributeHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerFlagsAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdClassHandle enumClassHandle)
{
	setVersionHelper();
	if (mStream) {

		mLog.outputLine("OmniPvdWriterImpl::registerFlagsAttribute classHandle(%llu), enumClassHandle(%llu), attributeName(%s)", static_cast<unsigned long long>(classHandle), static_cast<unsigned long long>(enumClassHandle), attributeName);

		int attribNameLen = (int)strlen(attributeName);
		writeCommand(*mStream, OmniPvdCommand::eREGISTER_ATTRIBUTE);
		mLastAttributeHandle++;
		mStream->writeBytes((const uint8_t*)&classHandle, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((const uint8_t*)&mLastAttributeHandle, sizeof(OmniPvdAttributeHandle));
		writeDataType(*mStream, OmniPvdDataType::eFLAGS_WORD);
		mStream->writeBytes((const uint8_t*)&enumClassHandle, sizeof(uint32_t));
		mStream->writeBytes((const uint8_t*)&attribNameLen, sizeof(uint16_t));
		mStream->writeBytes((const uint8_t*)attributeName, attribNameLen);
		return mLastAttributeHandle;
	}
	else {
		return 0;
	}
}

OmniPvdAttributeHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerEnumValue(OmniPvdClassHandle classHandle, const char* attributeName, uint32_t value)
{
	setVersionHelper();
	if (mStream) {
		int attribNameLen = (int)strlen(attributeName);
		writeCommand(*mStream, OmniPvdCommand::eREGISTER_ATTRIBUTE);
		mLastAttributeHandle++;
		mStream->writeBytes((const uint8_t*)&classHandle, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((const uint8_t*)&mLastAttributeHandle, sizeof(OmniPvdAttributeHandle));
		writeDataType(*mStream, OmniPvdDataType::eENUM_VALUE);
		mStream->writeBytes((const uint8_t*)&value, sizeof(uint32_t));
		mStream->writeBytes((const uint8_t*)&attribNameLen, sizeof(uint16_t));
		mStream->writeBytes((const uint8_t*)attributeName, attribNameLen);
		return mLastAttributeHandle;
	}
	else {
		return 0;
	}
}

OmniPvdAttributeHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerClassAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdClassHandle classAttributeHandle)
{
	setVersionHelper();
	if (mStream)
	{
		int attribNameLen = (int)strlen(attributeName);
		writeCommand(*mStream, OmniPvdCommand::eREGISTER_CLASS_ATTRIBUTE);
		mLastAttributeHandle++;
		mStream->writeBytes((const uint8_t*)&classHandle, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((const uint8_t*)&mLastAttributeHandle, sizeof(OmniPvdAttributeHandle));
		mStream->writeBytes((const uint8_t*)&classAttributeHandle, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((const uint8_t*)&attribNameLen, sizeof(uint16_t));
		mStream->writeBytes((const uint8_t*)attributeName, attribNameLen);
		return mLastAttributeHandle;
	}
	else {
		return 0;
	}
}

OmniPvdAttributeHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerUniqueListAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdDataType::Enum attributeDataType)
{
	setVersionHelper();
	if (mStream)
	{
		int attribNameLen = (int)strlen(attributeName);
		writeCommand(*mStream, OmniPvdCommand::eREGISTER_UNIQUE_LIST_ATTRIBUTE);
		mLastAttributeHandle++;
		mStream->writeBytes((const uint8_t*)&classHandle, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((const uint8_t*)&mLastAttributeHandle, sizeof(OmniPvdAttributeHandle));
		writeDataType(*mStream, attributeDataType);
		mStream->writeBytes((const uint8_t*)&attribNameLen, sizeof(uint16_t));
		mStream->writeBytes((const uint8_t*)attributeName, attribNameLen);
		return mLastAttributeHandle;
	}
	else
	{
		return 0;
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::setAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle* attributeHandles, uint8_t nbAttributeHandles, const uint8_t* data, uint32_t nbrBytes)
{
	setVersionHelper();
	if (mStream)
	{
		writeCommand(*mStream, OmniPvdCommand::eSET_ATTRIBUTE);
		mStream->writeBytes((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		mStream->writeBytes((const uint8_t*)&objectHandle, sizeof(OmniPvdObjectHandle));
		mStream->writeBytes((const uint8_t*)&nbAttributeHandles, sizeof(uint8_t));
		for (int i = 0; i < nbAttributeHandles; i++)
		{
			mStream->writeBytes((const uint8_t*)attributeHandles, sizeof(OmniPvdAttributeHandle));
			attributeHandles++;
		}
		mStream->writeBytes((const uint8_t*)&nbrBytes, sizeof(uint32_t));
		mStream->writeBytes((const uint8_t*)data, nbrBytes);
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::addToUniqueListAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle* attributeHandles, uint8_t nbAttributeHandles, const uint8_t* data, uint32_t nbrBytes)
{
	setVersionHelper();
	if (mStream)
	{
		writeCommand(*mStream, OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE);
		mStream->writeBytes((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		mStream->writeBytes((const uint8_t*)&objectHandle, sizeof(OmniPvdObjectHandle));
		mStream->writeBytes((const uint8_t*)&nbAttributeHandles, sizeof(uint8_t));
		for (int i = 0; i < nbAttributeHandles; i++)
		{
			mStream->writeBytes((const uint8_t*)attributeHandles, sizeof(OmniPvdAttributeHandle));
			attributeHandles++;
		}
		mStream->writeBytes((const uint8_t*)&nbrBytes, sizeof(uint32_t));
		mStream->writeBytes((const uint8_t*)data, nbrBytes);
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::removeFromUniqueListAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle* attributeHandles, uint8_t nbAttributeHandles, const uint8_t* data, uint32_t nbrBytes)
{
	setVersionHelper();
	if (mStream)
	{
		writeCommand(*mStream, OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE);
		mStream->writeBytes((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		mStream->writeBytes((const uint8_t*)&objectHandle, sizeof(OmniPvdObjectHandle));
		mStream->writeBytes((const uint8_t*)&nbAttributeHandles, sizeof(uint8_t));
		for (int i = 0; i < nbAttributeHandles; i++)
		{
			mStream->writeBytes((const uint8_t*)attributeHandles, sizeof(OmniPvdAttributeHandle));
			attributeHandles++;
		}
		mStream->writeBytes((const uint8_t*)&nbrBytes, sizeof(uint32_t));
		mStream->writeBytes((const uint8_t*)data, nbrBytes);
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::createObject(OmniPvdContextHandle contextHandle, OmniPvdClassHandle classHandle, OmniPvdObjectHandle objectHandle, const char* objectName)
{
	setVersionHelper();
	if (mStream)
	{
		writeCommand(*mStream, OmniPvdCommand::eCREATE_OBJECT);
		mStream->writeBytes((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		mStream->writeBytes((const uint8_t*)&classHandle, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((const uint8_t*)&objectHandle, sizeof(OmniPvdObjectHandle));
		int objectNameLen = 0;
		if (objectName)
		{
			objectNameLen = (int)strlen(objectName);
			mStream->writeBytes((const uint8_t*)&objectNameLen, sizeof(uint16_t));
			mStream->writeBytes((const uint8_t*)objectName, objectNameLen);
		}
		else
		{
			mStream->writeBytes((const uint8_t*)&objectNameLen, sizeof(uint16_t));
		}
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::destroyObject(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle)
{
	setVersionHelper();
	if (mStream)
	{
		writeCommand(*mStream, OmniPvdCommand::eDESTROY_OBJECT);
		mStream->writeBytes((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		mStream->writeBytes((const uint8_t*)&objectHandle, sizeof(OmniPvdObjectHandle));
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::startFrame(OmniPvdContextHandle contextHandle, uint64_t timeStamp)
{
	setVersionHelper();
	if (mStream)
	{
		writeCommand(*mStream, OmniPvdCommand::eSTART_FRAME);
		mStream->writeBytes((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		mStream->writeBytes((const uint8_t*)&timeStamp, sizeof(uint64_t));
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::stopFrame(OmniPvdContextHandle contextHandle, uint64_t timeStamp)
{
	setVersionHelper();
	if (mStream)
	{
		writeCommand(*mStream, OmniPvdCommand::eSTOP_FRAME);
		mStream->writeBytes((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		mStream->writeBytes((const uint8_t*)&timeStamp, sizeof(uint64_t));
	}
}
