// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OmniPvdWriterImpl.h"
#include "OmniPvdDefines.h"
#include "OmniPvdReader.h"
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
	mWriteStreamReady = false;
	mStatusFlags = 0; // That or set all flag bits off
}

void OMNI_PVD_CALL OmniPvdWriterImpl::setLogFunction(OmniPvdLogFunction logFunction)
{
	mLog.setLogFunction(logFunction);
}

void OmniPvdWriterImpl::setVersionHelper()
{
	setVersion(OMNI_PVD_VERSION_MAJOR, OMNI_PVD_VERSION_MINOR, OMNI_PVD_VERSION_PATCH);
}

void OmniPvdWriterImpl::setVersion(OmniPvdVersionType majorVersion, OmniPvdVersionType minorVersion, OmniPvdVersionType patch)
{
	if (mStream && mIsFirstWrite && !isFlagOn(OmniPvdWriterStatusFlag::eSTREAM_WRITE_FAILURE))
	{
		if (!mStream->openStream())
		{
			setFlagOn(OmniPvdWriterStatusFlag::eSTREAM_WRITE_FAILURE);
			return;
		}
		mWriteStreamReady = true;

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
	// Binding a stream resets the writer's per-stream state (this runs on every call, so
	// re-binding the same stream object after a reconnect resets it too):
	//  - reset the first-write flag so the 12-byte version header is written again onto
	//    this stream on the next write (otherwise a second stream, bound after writes
	//    already happened on a previous one, would never receive a header and be
	//    undecodable);
	//  - zero the class/attribute handle counters so re-registering the classes and
	//    attributes reproduces the same handle values (registration is deterministic),
	//    keeping the object data that follows, and refers to those handles, consistent
	//    with the re-registered definitions;
	//  - clear the status flags so a fresh stream does not inherit a stale write failure
	//    (e.g. eSTREAM_WRITE_FAILURE) from a previously bound stream.
	mIsFirstWrite = true;
	mWriteStreamReady = false;
	mLastClassHandle = 0;
	mLastAttributeHandle = 0;
	mStatusFlags = 0;
}

OmniPvdWriteStream* OMNI_PVD_CALL OmniPvdWriterImpl::getWriteStream()
{
	return mStream;
}

OmniPvdClassHandle OMNI_PVD_CALL OmniPvdWriterImpl::registerClass(const char* className, OmniPvdClassHandle baseClass)
{
	setVersionHelper();
	if (mWriteStreamReady)
	{
		mLog.outputLine("OmniPvdWriterImpl::registerClass className(%s)", className);

		int classNameLen = (int)strnlen(className, OMNI_PVD_MAX_STRING_LENGTH);
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
	if (mWriteStreamReady) {

		mLog.outputLine("OmniPvdWriterImpl::registerAttribute classHandle(%llu), attributeName(%s), attributeDataType(%d), nbrFields(%llu)", static_cast<unsigned long long>(classHandle), attributeName, static_cast<int>(attributeDataType), static_cast<unsigned long long>(nbElements));

		int attribNameLen = (int)strnlen(attributeName, OMNI_PVD_MAX_STRING_LENGTH);
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
	if (mWriteStreamReady) {

		mLog.outputLine("OmniPvdWriterImpl::registerFlagsAttribute classHandle(%llu), enumClassHandle(%llu), attributeName(%s)", static_cast<unsigned long long>(classHandle), static_cast<unsigned long long>(enumClassHandle), attributeName);

		int attribNameLen = (int)strnlen(attributeName, OMNI_PVD_MAX_STRING_LENGTH);
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
	if (mWriteStreamReady) {
		int attribNameLen = (int)strnlen(attributeName, OMNI_PVD_MAX_STRING_LENGTH);
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
	if (mWriteStreamReady)
	{
		int attribNameLen = (int)strnlen(attributeName, OMNI_PVD_MAX_STRING_LENGTH);
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
	if (mWriteStreamReady)
	{
		int attribNameLen = (int)strnlen(attributeName, OMNI_PVD_MAX_STRING_LENGTH);
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
	if (mWriteStreamReady)
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
	if (mWriteStreamReady)
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
	if (mWriteStreamReady)
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
	if (mWriteStreamReady)
	{
		writeCommand(OmniPvdCommand::eCREATE_OBJECT);
		writeWithStatus((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		writeWithStatus((const uint8_t*)&classHandle, sizeof(OmniPvdClassHandle));
		writeWithStatus((const uint8_t*)&objectHandle, sizeof(OmniPvdObjectHandle));
		int objectNameLen = 0;
		if (objectName)
		{
			objectNameLen = (int)strnlen(objectName, OMNI_PVD_MAX_STRING_LENGTH);
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
	if (mWriteStreamReady)
	{
		writeCommand(OmniPvdCommand::eDESTROY_OBJECT);
		writeWithStatus((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		writeWithStatus((const uint8_t*)&objectHandle, sizeof(OmniPvdObjectHandle));
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::startFrame(OmniPvdContextHandle contextHandle, uint64_t timeStamp)
{
	setVersionHelper();
	if (mWriteStreamReady)
	{
		writeCommand(OmniPvdCommand::eSTART_FRAME);
		writeWithStatus((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		writeWithStatus((const uint8_t*)&timeStamp, sizeof(uint64_t));
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::stopFrame(OmniPvdContextHandle contextHandle, uint64_t timeStamp)
{
	setVersionHelper();
	if (mWriteStreamReady)
	{
		writeCommand(OmniPvdCommand::eSTOP_FRAME);
		writeWithStatus((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));
		writeWithStatus((const uint8_t*)&timeStamp, sizeof(uint64_t));
	}
}

void OMNI_PVD_CALL OmniPvdWriterImpl::recordMessage(OmniPvdContextHandle contextHandle, const char* message, const char* file, uint32_t line, uint32_t type, OmniPvdClassHandle handle)
{
	setVersionHelper();
	if (mWriteStreamReady)
	{
		writeCommand(OmniPvdCommand::eRECORD_MESSAGE);
		writeWithStatus((const uint8_t*)&contextHandle, sizeof(OmniPvdContextHandle));

		int messageLength = 0;

		if (message)
		{
			messageLength = (int)strnlen(message, OMNI_PVD_MAX_STRING_LENGTH);
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
			filenameLength = (int)strnlen(file, OMNI_PVD_MAX_STRING_LENGTH);
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
