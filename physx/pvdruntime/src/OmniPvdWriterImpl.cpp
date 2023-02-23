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

void OmniPvdWriterImpl::setVersion(const OmniPvdVersionType majorVersion, const OmniPvdVersionType minorVersion, const OmniPvdVersionType patch)
{	
	if (mStream && mIsFirstWrite)
	{ 
		if (!mStream->openStream())
		{
			return;
		}
		mStream->writeBytes((unsigned char*)&majorVersion, sizeof(OmniPvdVersionType));
		mStream->writeBytes((unsigned char*)&minorVersion, sizeof(OmniPvdVersionType));
		mStream->writeBytes((unsigned char*)&patch, sizeof(OmniPvdVersionType));

		mLog.outputLine("OmniPvdRuntimeWriterImpl::setVersion majorVersion(%lu), minorVersion(%lu), patch(%lu)", static_cast<unsigned long>(majorVersion), static_cast<unsigned long>(minorVersion), static_cast<unsigned long>(patch));
		
		mIsFirstWrite = false;
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::setWriteStream(const OmniPvdWriteStream* stream)
{
	mLog.outputLine("OmniPvdRuntimeWriterImpl::setWriteStream");
	mStream = (OmniPvdWriteStream*)stream;
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
		unsigned char command = OmniPvdCommandEnum::eOmniPvdRegisterClass;
		mStream->writeBytes(&command, sizeof(uint8_t));
		mLastClassHandle++;
		mStream->writeBytes((unsigned char*)&mLastClassHandle, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((unsigned char*)&baseClass, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((unsigned char*)&classNameLen, sizeof(uint16_t));
		mStream->writeBytes((unsigned char*)className, classNameLen);
		return mLastClassHandle;
	} else {
		return 0;
	}	
}

OmniPvdAttributeHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerAttribute(const OmniPvdClassHandle classHandle, const char* attributeName, const OmniPvdAttributeDataType attributeDataType, const uint32_t nbrFields)
{
	setVersionHelper();
	if (mStream) {

		mLog.outputLine("OmniPvdWriterImpl::registerAttribute classHandle(%llu), attributeName(%s), attributeDataType(%d), nbrFields(%llu)", static_cast<unsigned long long>(classHandle), attributeName, static_cast<int>(attributeDataType), static_cast<unsigned long long>(nbrFields));

		int attribNameLen = (int)strlen(attributeName);
		unsigned char command = OmniPvdCommandEnum::eOmniPvdRegisterAttribute;
		mStream->writeBytes(&command, sizeof(uint8_t));
		mLastAttributeHandle++;
		mStream->writeBytes((unsigned char*)&classHandle, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((unsigned char*)&mLastAttributeHandle, sizeof(OmniPvdAttributeHandle));
		mStream->writeBytes((unsigned char*)&attributeDataType, sizeof(OmniPvdAttributeDataType));
		mStream->writeBytes((unsigned char*)&nbrFields, sizeof(uint32_t));
		mStream->writeBytes((unsigned char*)&attribNameLen, sizeof(uint16_t));
		mStream->writeBytes((unsigned char*)attributeName, attribNameLen);
		return mLastAttributeHandle;
	}
	else {
		return 0;
	}
}

OmniPvdAttributeHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerFlagsAttribute(const OmniPvdClassHandle classHandle, const OmniPvdClassHandle enumClassHandle, const char* attributeName)
{
	setVersionHelper();
	if (mStream) {

		mLog.outputLine("OmniPvdWriterImpl::registerFlagsAttribute classHandle(%llu), enumClassHandle(%llu), attributeName(%s)", static_cast<unsigned long long>(classHandle), static_cast<unsigned long long>(enumClassHandle), attributeName);

		int attribNameLen = (int)strlen(attributeName);
		unsigned char command = OmniPvdCommandEnum::eOmniPvdRegisterAttribute;
		const OmniPvdAttributeDataType attributeDataType = OmniPvdDataTypeEnum::eFLAGS_WORD;
		mStream->writeBytes(&command, sizeof(uint8_t));
		mLastAttributeHandle++;
		mStream->writeBytes((unsigned char*)&classHandle, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((unsigned char*)&mLastAttributeHandle, sizeof(OmniPvdAttributeHandle));
		mStream->writeBytes((unsigned char*)&attributeDataType, sizeof(OmniPvdAttributeDataType));
		mStream->writeBytes((unsigned char*)&enumClassHandle, sizeof(uint32_t));
		mStream->writeBytes((unsigned char*)&attribNameLen, sizeof(uint16_t));
		mStream->writeBytes((unsigned char*)attributeName, attribNameLen);
		return mLastAttributeHandle;
	}
	else {
		return 0;
	}
}

OmniPvdAttributeHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerEnumValue(const OmniPvdClassHandle classHandle, const char* attributeName, uint32_t value)
{
	setVersionHelper();
	if (mStream) {
		int attribNameLen = (int)strlen(attributeName);
		unsigned char command = OmniPvdCommandEnum::eOmniPvdRegisterAttribute;
		OmniPvdAttributeDataType attributeDataType = OmniPvdDataTypeEnum::eENUM_VALUE;
		mStream->writeBytes(&command, sizeof(uint8_t));
		mLastAttributeHandle++;
		mStream->writeBytes((unsigned char*)&classHandle, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((unsigned char*)&mLastAttributeHandle, sizeof(OmniPvdAttributeHandle));
		mStream->writeBytes((unsigned char*)&attributeDataType, sizeof(OmniPvdAttributeDataType));
		mStream->writeBytes((unsigned char*)&value, sizeof(uint32_t));
		mStream->writeBytes((unsigned char*)&attribNameLen, sizeof(uint16_t));
		mStream->writeBytes((unsigned char*)attributeName, attribNameLen);
		return mLastAttributeHandle;
	}
	else {
		return 0;
	}
}

OmniPvdAttributeHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerClassAttribute(const OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdClassHandle classAttributeHandle)
{
	setVersionHelper();
	if (mStream)
	{
		int attribNameLen = (int)strlen(attributeName);
		unsigned char command = OmniPvdCommandEnum::eOmniPvdRegisterClassAttribute;
		mStream->writeBytes(&command, sizeof(uint8_t));
		mLastAttributeHandle++;
		mStream->writeBytes((unsigned char*)&classHandle, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((unsigned char*)&mLastAttributeHandle, sizeof(OmniPvdAttributeHandle));
		mStream->writeBytes((unsigned char*)&classAttributeHandle, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((unsigned char*)&attribNameLen, sizeof(uint16_t));
		mStream->writeBytes((unsigned char*)attributeName, attribNameLen);
		return mLastAttributeHandle;
	}
	else {
		return 0;
	}
}

OmniPvdAttributeHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerSetAttribute(const OmniPvdClassHandle classHandle, const char* attributeName, const OmniPvdAttributeDataType attributeDataType)
{
	setVersionHelper();
	if (mStream)
	{
		int attribNameLen = (int)strlen(attributeName);
		unsigned char command = OmniPvdCommandEnum::eOmniPvdRegisterSetAttribute;
		mStream->writeBytes(&command, sizeof(uint8_t));
		mLastAttributeHandle++;
		mStream->writeBytes((unsigned char*)&classHandle, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((unsigned char*)&mLastAttributeHandle, sizeof(OmniPvdAttributeHandle));
		mStream->writeBytes((unsigned char*)&attributeDataType, sizeof(OmniPvdAttributeDataType));
		mStream->writeBytes((unsigned char*)&attribNameLen, sizeof(uint16_t));
		mStream->writeBytes((unsigned char*)attributeName, attribNameLen);
		return mLastAttributeHandle;
	}
	else
	{
		return 0;
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::setAttribute(const OmniPvdContextHandle contextHandle, const OmniPvdObjectHandle objectHandle, const uint8_t handleDepth, const OmniPvdAttributeHandle* attributeHandles, const uint8_t* data, const uint32_t nbrBytes)
{
	setVersionHelper();
	if (mStream)
	{
		unsigned char command = OmniPvdCommandEnum::eOmniPvdSetAttribute;
		mStream->writeBytes(&command, sizeof(uint8_t));
		mStream->writeBytes((unsigned char*)&contextHandle, sizeof(OmniPvdContextHandle));
		mStream->writeBytes((unsigned char*)&objectHandle, sizeof(OmniPvdObjectHandle));
		mStream->writeBytes((unsigned char*)&handleDepth, sizeof(uint8_t));
		for (int i = 0; i < handleDepth; i++)
		{
			mStream->writeBytes((unsigned char*)attributeHandles, sizeof(OmniPvdAttributeHandle));
			attributeHandles++;
		}
		mStream->writeBytes((unsigned char*)&nbrBytes, sizeof(uint32_t));
		mStream->writeBytes((unsigned char*)data, nbrBytes);
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::setAttributeShallow(const OmniPvdContextHandle contextHandle, const OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle attributeHandle, const uint8_t *data, const uint32_t nbrBytes)
{
	const uint8_t handleDepth = 1;
	setAttribute(contextHandle, objectHandle, handleDepth, &attributeHandle, data, nbrBytes);
}

void OMNI_PVD_CALL OmniPvdWriterImpl::addToSetAttribute(const OmniPvdContextHandle contextHandle, const OmniPvdObjectHandle objectHandle, const uint8_t handleDepth, const OmniPvdAttributeHandle* attributeHandles, const uint8_t* data, const uint32_t nbrBytes)
{
	setVersionHelper();
	if (mStream)
	{
		unsigned char command = OmniPvdCommandEnum::eOmniPvdAddToSetAttribute;
		mStream->writeBytes(&command, sizeof(uint8_t));
		mStream->writeBytes((unsigned char*)&contextHandle, sizeof(OmniPvdContextHandle));
		mStream->writeBytes((unsigned char*)&objectHandle, sizeof(OmniPvdObjectHandle));
		mStream->writeBytes((unsigned char*)&handleDepth, sizeof(uint8_t));
		for (int i = 0; i < handleDepth; i++)
		{
			mStream->writeBytes((unsigned char*)attributeHandles, sizeof(OmniPvdAttributeHandle));
			attributeHandles++;
		}
		mStream->writeBytes((unsigned char*)&nbrBytes, sizeof(uint32_t));
		mStream->writeBytes((unsigned char*)data, nbrBytes);
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::addToSetAttributeShallow(const OmniPvdContextHandle contextHandle, const OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle attributeHandle, const uint8_t* data, const uint32_t nbrBytes)
{
	const uint8_t handleDepth = 1;
	addToSetAttribute(contextHandle, objectHandle, handleDepth, &attributeHandle, data, nbrBytes);
}

void OMNI_PVD_CALL OmniPvdWriterImpl::removeFromSetAttribute(const OmniPvdContextHandle contextHandle, const OmniPvdObjectHandle objectHandle, const uint8_t handleDepth, const OmniPvdAttributeHandle* attributeHandles, const uint8_t* data, const uint32_t nbrBytes)
{
	setVersionHelper();
	if (mStream)
	{
		unsigned char command = OmniPvdCommandEnum::eOmniPvdRemoveFromSetAttribute;
		mStream->writeBytes(&command, sizeof(uint8_t));
		mStream->writeBytes((unsigned char*)&contextHandle, sizeof(OmniPvdContextHandle));
		mStream->writeBytes((unsigned char*)&objectHandle, sizeof(OmniPvdObjectHandle));
		mStream->writeBytes((unsigned char*)&handleDepth, sizeof(uint8_t));
		for (int i = 0; i < handleDepth; i++)
		{
			mStream->writeBytes((unsigned char*)attributeHandles, sizeof(OmniPvdAttributeHandle));
			attributeHandles++;
		}
		mStream->writeBytes((unsigned char*)&nbrBytes, sizeof(uint32_t));
		mStream->writeBytes((unsigned char*)data, nbrBytes);
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::removeFromSetAttributeShallow(const OmniPvdContextHandle contextHandle, const OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle attributeHandle, const uint8_t* data, const uint32_t nbrBytes)
{
	const uint8_t handleDepth = 1;
	removeFromSetAttribute(contextHandle, objectHandle, handleDepth, &attributeHandle, data, nbrBytes);
}

void OMNI_PVD_CALL OmniPvdWriterImpl::createObject(const OmniPvdContextHandle contextHandle, const OmniPvdClassHandle classHandle, const OmniPvdObjectHandle objectHandle, const char* objectName)
{
	setVersionHelper();
	if (mStream)
	{
		unsigned char command = OmniPvdCommandEnum::eOmniPvdCreateObject;
		mStream->writeBytes(&command, sizeof(uint8_t));
		mStream->writeBytes((unsigned char*)&contextHandle, sizeof(OmniPvdContextHandle));
		mStream->writeBytes((unsigned char*)&classHandle, sizeof(OmniPvdClassHandle));
		mStream->writeBytes((unsigned char*)&objectHandle, sizeof(OmniPvdObjectHandle));
		int objectNameLen = 0;
		if (objectName)
		{
			objectNameLen = (int)strlen(objectName);
			mStream->writeBytes((unsigned char*)&objectNameLen, sizeof(uint16_t));
			mStream->writeBytes((unsigned char*)objectName, objectNameLen);
		}
		else
		{
			mStream->writeBytes((unsigned char*)&objectNameLen, sizeof(uint16_t));
		}
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::destroyObject(const OmniPvdContextHandle contextHandle, const OmniPvdObjectHandle objectHandle)
{
	setVersionHelper();
	if (mStream)
	{
		unsigned char command = OmniPvdCommandEnum::eOmniPvdDestroyObject;
		mStream->writeBytes(&command, sizeof(uint8_t));
		mStream->writeBytes((unsigned char*)&contextHandle, sizeof(OmniPvdContextHandle));
		mStream->writeBytes((unsigned char*)&objectHandle, sizeof(OmniPvdObjectHandle));
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::startFrame(const OmniPvdContextHandle contextHandle, const uint64_t timeStamp)
{
	setVersionHelper();
	if (mStream)
	{
		unsigned char command = OmniPvdCommandEnum::eOmniPvdStartFrame;
		mStream->writeBytes(&command, sizeof(uint8_t));
		mStream->writeBytes((unsigned char*)&contextHandle, sizeof(OmniPvdContextHandle));
		mStream->writeBytes((unsigned char*)&timeStamp, sizeof(uint64_t));
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::stopFrame(const OmniPvdContextHandle contextHandle, const uint64_t timeStamp)
{
	setVersionHelper();
	if (mStream)
	{
		unsigned char command = OmniPvdCommandEnum::eOmniPvdStopFrame;
		mStream->writeBytes(&command, sizeof(uint8_t));
		mStream->writeBytes((unsigned char*)&contextHandle, sizeof(OmniPvdContextHandle));
		mStream->writeBytes((unsigned char*)&timeStamp, sizeof(uint64_t));
	}
}
