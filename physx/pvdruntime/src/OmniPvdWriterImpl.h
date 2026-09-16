// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_WRITER_IMPL_H
#define OMNI_PVD_WRITER_IMPL_H

#include "OmniPvdWriter.h"
#include "OmniPvdCommands.h"
#include "OmniPvdDefinesInternal.h"
#include "OmniPvdLog.h"

class OmniPvdWriterImpl : public OmniPvdWriter {
public:
	OmniPvdWriterImpl();
	~OmniPvdWriterImpl();
	void OMNI_PVD_CALL setLogFunction(OmniPvdLogFunction logFunction) override;
	void setVersionHelper();
	void setVersion(OmniPvdVersionType majorVersion, OmniPvdVersionType minorVersion, OmniPvdVersionType patch);
	void OMNI_PVD_CALL setWriteStream(OmniPvdWriteStream& stream) override;
	OmniPvdWriteStream* OMNI_PVD_CALL getWriteStream() override;

	OmniPvdClassHandle OMNI_PVD_CALL registerClass(const char* className, OmniPvdClassHandle baseClass) override;
	OmniPvdAttributeHandle OMNI_PVD_CALL registerEnumValue(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdEnumValueType value) override;
	OmniPvdAttributeHandle OMNI_PVD_CALL registerAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdDataType::Enum attributeDataType, uint32_t nbElements) override;
	OmniPvdAttributeHandle OMNI_PVD_CALL registerFlagsAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdClassHandle enumClassHandle) override;
	OmniPvdAttributeHandle OMNI_PVD_CALL registerClassAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdClassHandle classAttributeHandle) override;
	OmniPvdAttributeHandle OMNI_PVD_CALL registerUniqueListAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdDataType::Enum attributeDataType) override;
	void OMNI_PVD_CALL setAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle* attributeHandles, uint8_t nbAttributeHandles, const uint8_t* data, uint32_t nbrBytes) override;
	
	void OMNI_PVD_CALL addToUniqueListAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle* attributeHandles, uint8_t nbAttributeHandles, const uint8_t* data, uint32_t nbrBytes) override;

	void OMNI_PVD_CALL removeFromUniqueListAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle* attributeHandles, uint8_t nbAttributeHandles, const uint8_t* data, uint32_t nbrBytes) override;

	void OMNI_PVD_CALL createObject(OmniPvdContextHandle contextHandle, OmniPvdClassHandle classHandle, OmniPvdObjectHandle objectHandle, const char* objectName) override;
	void OMNI_PVD_CALL destroyObject(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle) override;
	void OMNI_PVD_CALL startFrame(OmniPvdContextHandle contextHandle, uint64_t timeStamp) override;
	void OMNI_PVD_CALL stopFrame(OmniPvdContextHandle contextHandle, uint64_t timeStamp) override;

	void OMNI_PVD_CALL recordMessage(OmniPvdContextHandle contextHandle, const char* message, const char* file, uint32_t line, uint32_t type, OmniPvdClassHandle handle) override;

	uint32_t OMNI_PVD_CALL getStatus() override;
	void OMNI_PVD_CALL clearStatus() override;

	void resetParams();

	bool isFlagOn(OmniPvdWriterStatusFlag::Enum flagBitMask)
	{
		return mStatusFlags & uint32_t(flagBitMask);
	}
	
	void setFlagOn(OmniPvdWriterStatusFlag::Enum flagBitMask)
	{
		mStatusFlags = mStatusFlags | uint32_t(flagBitMask);
	}

	void setFlagOff(OmniPvdWriterStatusFlag::Enum flagBitMask)
	{
		mStatusFlags = mStatusFlags & ~uint32_t(flagBitMask);
	}

	void setFlagVal(OmniPvdWriterStatusFlag::Enum flagBitMask, bool value)
	{
		if (value) 
		{
			setFlagOn(flagBitMask);
		}
		else
		{
			setFlagOff(flagBitMask);
		}
	}

	void writeWithStatus(const uint8_t* writePtr, uint64_t nbrBytesToWrite) 
	{
		if (!(mWriteStreamReady && writePtr && (nbrBytesToWrite > 0))) return;
		uint64_t nbrBytesWritten = mStream->writeBytes(writePtr, nbrBytesToWrite);
		const bool writeFailure = nbrBytesWritten != nbrBytesToWrite;
		if (writeFailure) {
			setFlagOn(OmniPvdWriterStatusFlag::eSTREAM_WRITE_FAILURE);
		}
	}
	
	void writeDataType(OmniPvdDataType::Enum attributeDataType)
	{
		const OmniPvdDataTypeStorageType dataType = static_cast<OmniPvdDataTypeStorageType>(attributeDataType);
		writeWithStatus((const uint8_t*)&dataType, sizeof(OmniPvdDataTypeStorageType));
	}
	
	void writeCommand(OmniPvdCommand::Enum command)
	{
		const OmniPvdCommandStorageType commandTmp = static_cast<OmniPvdCommandStorageType>(command);
		writeWithStatus((const uint8_t*)&commandTmp, sizeof(OmniPvdCommandStorageType));
	}

	bool mIsFirstWrite;
	bool mWriteStreamReady;
	OmniPvdLog mLog;
	OmniPvdWriteStream* mStream;
	int mLastClassHandle;
	int mLastAttributeHandle;

	uint32_t mStatusFlags;
};

#endif
