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

#include "OmniPvdWriterImpl.h"
#include "OmniPvdDefines.h"
#include <string.h>

OmniPvdWriterImpl::OmniPvdWriterImpl()
{	
	resetParams();
}

OmniPvdWriterImpl::~OmniPvdWriterImpl()
{
}

void OmniPvdWriterImpl::resetParams()
{
	mStream = 0;
	mLastClassHandle = 0;
	mLastAttributeHandle = 0;
	mIsFirstWrite = true;
	mStatusFlags = 0; // That or set all flag bits off
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

		writeWithStatus((const uint8_t*)&majorVersion, sizeof(OmniPvdVersionType));
		writeWithStatus((const uint8_t*)&minorVersion, sizeof(OmniPvdVersionType));
		writeWithStatus((const uint8_t*)&patch, sizeof(OmniPvdVersionType));

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

OmniPvdClassHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerClass(const char* className, OmniPvdClassHandle baseClass)
{
	setVersionHelper();
	if (mStream)
	{
		mLog.outputLine("OmniPvdWriterImpl::registerClass className(%s)", className);

		int classNameLen = (int)strlen(className);
		writeCommand(OmniPvdCommand::eREGISTER_CLASS);
		mLastClassHandle++;
		writeWithStatus((const uint8_t*)&mLastClassHandle, sizeof(OmniPvdClassHandle));
		writeWithStatus((const uint8_t*)&baseClass, sizeof(OmniPvdClassHandle));
		writeWithStatus((const uint8_t*)&classNameLen, sizeof(uint16_t));
		writeWithStatus((const uint8_t*)className, classNameLen);
		return mLastClassHandle;
	} else {
		return OMNI_PVD_INVALID_HANDLE;
	}	
}

OmniPvdAttributeHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdDataType::Enum attributeDataType, uint32_t nbElements)
{
	setVersionHelper();
	if (mStream) {

		mLog.outputLine("OmniPvdWriterImpl::registerAttribute classHandle(%llu), attributeName(%s), attributeDataType(%d), nbrFields(%llu)", static_cast<unsigned long long>(classHandle), attributeName, static_cast<int>(attributeDataType), static_cast<unsigned long long>(nbElements));

		int attribNameLen = (int)strlen(attributeName);
		writeCommand(OmniPvdCommand::eREGISTER_ATTRIBUTE);
		mLastAttributeHandle++;
		writeWithStatus((const uint8_t*)&classHandle, sizeof(OmniPvdClassHandle));
		writeWithStatus((const uint8_t*)&mLastAttributeHandle, sizeof(OmniPvdAttributeHandle));
		writeDataType(attributeDataType);
		writeWithStatus((const uint8_t*)&nbElements, sizeof(uint32_t));
		writeWithStatus((const uint8_t*)&attribNameLen, sizeof(uint16_t));
		writeWithStatus((const uint8_t*)attributeName, attribNameLen);
		return mLastAttributeHandle;
	}
	else {
		return OMNI_PVD_INVALID_HANDLE;
	}
}

OmniPvdAttributeHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerFlagsAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdClassHandle enumClassHandle)
{
	setVersionHelper();
	if (mStream) {

		mLog.outputLine("OmniPvdWriterImpl::registerFlagsAttribute classHandle(%llu), enumClassHandle(%llu), attributeName(%s)", static_cast<unsigned long long>(classHandle), static_cast<unsigned long long>(enumClassHandle), attributeName);

		int attribNameLen = (int)strlen(attributeName);
		writeCommand(OmniPvdCommand::eREGISTER_ATTRIBUTE);
		mLastAttributeHandle++;
		writeWithStatus((const uint8_t*)&classHandle, sizeof(OmniPvdClassHandle));
		writeWithStatus((const uint8_t*)&mLastAttributeHandle, sizeof(OmniPvdAttributeHandle));
		writeDataType(OmniPvdDataType::eFLAGS_WORD);
		writeWithStatus((const uint8_t*)&enumClassHandle, sizeof(OmniPvdClassHandle));
		writeWithStatus((const uint8_t*)&attribNameLen, sizeof(uint16_t));
		writeWithStatus((const uint8_t*)attributeName, attribNameLen);
		return mLastAttributeHandle;
	}
	else {
		return OMNI_PVD_INVALID_HANDLE;
	}
}

OmniPvdAttributeHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerEnumValue(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdEnumValueType value)
{
	setVersionHelper();
	if (mStream) {
		int attribNameLen = (int)strlen(attributeName);
		writeCommand(OmniPvdCommand::eREGISTER_ATTRIBUTE);
		mLastAttributeHandle++;
		writeWithStatus((const uint8_t*)&classHandle, sizeof(OmniPvdClassHandle));
		writeWithStatus((const uint8_t*)&mLastAttributeHandle, sizeof(OmniPvdAttributeHandle));
		writeDataType(OmniPvdDataType::eENUM_VALUE);
		writeWithStatus((const uint8_t*)&value, sizeof(OmniPvdEnumValueType));
		writeWithStatus((const uint8_t*)&attribNameLen, sizeof(uint16_t));
		writeWithStatus((const uint8_t*)attributeName, attribNameLen);
		return mLastAttributeHandle;
	}
	else {
		return OMNI_PVD_INVALID_HANDLE;
	}
}

OmniPvdAttributeHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerClassAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdClassHandle classAttributeHandle)
{
	setVersionHelper();
	if (mStream)
	{
		int attribNameLen = (int)strlen(attributeName);
		writeCommand(OmniPvdCommand::eREGISTER_CLASS_ATTRIBUTE);
		mLastAttributeHandle++;
		writeWithStatus((const uint8_t*)&classHandle, sizeof(OmniPvdClassHandle));
		writeWithStatus((const uint8_t*)&mLastAttributeHandle, sizeof(OmniPvdAttributeHandle));
		writeWithStatus((const uint8_t*)&classAttributeHandle, sizeof(OmniPvdClassHandle));
		writeWithStatus((const uint8_t*)&attribNameLen, sizeof(uint16_t));
		writeWithStatus((const uint8_t*)attributeName, attribNameLen);
		return mLastAttributeHandle;
	}
	else {
		return OMNI_PVD_INVALID_HANDLE;
	}
}

OmniPvdAttributeHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerUniqueListAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdDataType::Enum attributeDataType)
{
	setVersionHelper();
	if (mStream)
	{
		int attribNameLen = (int)strlen(attributeName);
		writeCommand(OmniPvdCommand::eREGISTER_UNIQUE_LIST_ATTRIBUTE);
		mLastAttributeHandle++;
		writeWithStatus((const uint8_t*)&classHandle, sizeof(OmniPvdClassHandle));
		writeWithStatus((const uint8_t*)&mLastAttributeHandle, sizeof(OmniPvdAttributeHandle));
		writeDataType(attributeDataType);
		writeWithStatus((const uint8_t*)&attribNameLen, sizeof(uint16_t));
		writeWithStatus((const uint8_t*)attributeName, attribNameLen);
		return mLastAttributeHandle;
	}
	else
	{
		return OMNI_PVD_INVALID_HANDLE;
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::setAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle* attributeHandles, uint8_t nbAttributeHandles, const uint8_t* data, uint32_t nbrBytes)
{
	setVersionHelper();
	if (mStream)
	{
		writeCommand(OmniPvdCommand::eSET_ATTRIBUTE);
		writeWithStatus((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		writeWithStatus((const uint8_t*)&objectHandle, sizeof(OmniPvdObjectHandle));
		writeWithStatus((const uint8_t*)&nbAttributeHandles, sizeof(uint8_t));
		for (int i = 0; i < nbAttributeHandles; i++)
		{
			writeWithStatus((const uint8_t*)attributeHandles, sizeof(OmniPvdAttributeHandle));
			attributeHandles++;
		}
		writeWithStatus((const uint8_t*)&nbrBytes, sizeof(uint32_t));
		writeWithStatus((const uint8_t*)data, nbrBytes);
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::addToUniqueListAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle* attributeHandles, uint8_t nbAttributeHandles, const uint8_t* data, uint32_t nbrBytes)
{
	setVersionHelper();
	if (mStream)
	{
		writeCommand(OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE);
		writeWithStatus((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		writeWithStatus((const uint8_t*)&objectHandle, sizeof(OmniPvdObjectHandle));
		writeWithStatus((const uint8_t*)&nbAttributeHandles, sizeof(uint8_t));
		for (int i = 0; i < nbAttributeHandles; i++)
		{
			writeWithStatus((const uint8_t*)attributeHandles, sizeof(OmniPvdAttributeHandle));
			attributeHandles++;
		}
		writeWithStatus((const uint8_t*)&nbrBytes, sizeof(uint32_t));
		writeWithStatus((const uint8_t*)data, nbrBytes);
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::removeFromUniqueListAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle* attributeHandles, uint8_t nbAttributeHandles, const uint8_t* data, uint32_t nbrBytes)
{
	setVersionHelper();
	if (mStream)
	{
		writeCommand(OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE);
		writeWithStatus((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		writeWithStatus((const uint8_t*)&objectHandle, sizeof(OmniPvdObjectHandle));
		writeWithStatus((const uint8_t*)&nbAttributeHandles, sizeof(uint8_t));
		for (int i = 0; i < nbAttributeHandles; i++)
		{
			writeWithStatus((const uint8_t*)attributeHandles, sizeof(OmniPvdAttributeHandle));
			attributeHandles++;
		}
		writeWithStatus((const uint8_t*)&nbrBytes, sizeof(uint32_t));
		writeWithStatus((const uint8_t*)data, nbrBytes);
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::createObject(OmniPvdContextHandle contextHandle, OmniPvdClassHandle classHandle, OmniPvdObjectHandle objectHandle, const char* objectName)
{
	setVersionHelper();
	if (mStream)
	{
		writeCommand(OmniPvdCommand::eCREATE_OBJECT);
		writeWithStatus((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		writeWithStatus((const uint8_t*)&classHandle, sizeof(OmniPvdClassHandle));
		writeWithStatus((const uint8_t*)&objectHandle, sizeof(OmniPvdObjectHandle));
		int objectNameLen = 0;
		if (objectName)
		{
			objectNameLen = (int)strlen(objectName);
			writeWithStatus((const uint8_t*)&objectNameLen, sizeof(uint16_t));
			writeWithStatus((const uint8_t*)objectName, objectNameLen);
		}
		else
		{
			writeWithStatus((const uint8_t*)&objectNameLen, sizeof(uint16_t));
		}
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::destroyObject(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle)
{
	setVersionHelper();
	if (mStream)
	{
		writeCommand(OmniPvdCommand::eDESTROY_OBJECT);
		writeWithStatus((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		writeWithStatus((const uint8_t*)&objectHandle, sizeof(OmniPvdObjectHandle));
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::startFrame(OmniPvdContextHandle contextHandle, uint64_t timeStamp)
{
	setVersionHelper();
	if (mStream)
	{
		writeCommand(OmniPvdCommand::eSTART_FRAME);
		writeWithStatus((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		writeWithStatus((const uint8_t*)&timeStamp, sizeof(uint64_t));
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::stopFrame(OmniPvdContextHandle contextHandle, uint64_t timeStamp)
{
	setVersionHelper();
	if (mStream)
	{
		writeCommand(OmniPvdCommand::eSTOP_FRAME);
		writeWithStatus((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		writeWithStatus((const uint8_t*)&timeStamp, sizeof(uint64_t));
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::recordMessage(OmniPvdContextHandle contextHandle, const char* message, const char* file, uint32_t line, uint32_t type, OmniPvdClassHandle handle)
{
	setVersionHelper();
	if (mStream)
	{
		writeCommand(OmniPvdCommand::eRECORD_MESSAGE);
		writeWithStatus((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));

		int messageLength = 0;

		if (message)
		{
			messageLength = (int)strlen(message);
			writeWithStatus((const uint8_t*)&messageLength, sizeof(uint16_t));
			writeWithStatus((const uint8_t*)message, messageLength);
		}
		else
		{
			writeWithStatus((const uint8_t*)&messageLength, sizeof(uint16_t));
		}

		int filenameLength = 0;

		if (file)
		{
			filenameLength = (int)strlen(file);
			writeWithStatus((const uint8_t*)&filenameLength, sizeof(uint16_t));
			writeWithStatus((const uint8_t*)file, filenameLength);
		}
		else
		{
			writeWithStatus((const uint8_t*)&filenameLength, sizeof(uint16_t));
		}

		writeWithStatus((const uint8_t*)&line, sizeof(uint32_t));
		writeWithStatus((const uint8_t*)&type, sizeof(uint32_t));
		writeWithStatus((const uint8_t*)&handle, sizeof(OmniPvdClassHandle));
	}
}

uint32_t OMNI_PVD_CALL OmniPvdWriterImpl::getStatus() 
{
	return mStatusFlags;
}

void OMNI_PVD_CALL OmniPvdWriterImpl::clearStatus()
{
	mStatusFlags = 0;
}