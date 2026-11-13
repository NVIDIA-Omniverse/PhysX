// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_RUNTIME_READER_IMPL_H
#define OMNI_PVD_RUNTIME_READER_IMPL_H

#include "OmniPvdReader.h"
#include "OmniPvdDefinesInternal.h"
#include "OmniPvdLog.h"
#include "OmniPvdLibraryFunctions.h"


class OmniPvdReaderImpl : public OmniPvdReader {
public:
	OmniPvdReaderImpl(uint32_t maxAttributeBytes = 1u << 30, uint8_t maxPathDepth = 32);
	~OmniPvdReaderImpl();

	void OMNI_PVD_CALL setLogFunction(OmniPvdLogFunction logFunction) override;
	void OMNI_PVD_CALL setReadStream(OmniPvdReadStream& stream) override;
	bool OMNI_PVD_CALL startReading(OmniPvdVersionType& majorVersion, OmniPvdVersionType& minorVersion, OmniPvdVersionType& patch) override;
	OmniPvdCommand::Enum OMNI_PVD_CALL getNextCommand() override;

	OmniPvdVersionType OMNI_PVD_CALL getMajorVersion() override;
	OmniPvdVersionType OMNI_PVD_CALL getMinorVersion() override;
	OmniPvdVersionType OMNI_PVD_CALL getPatch() override;

	OmniPvdContextHandle OMNI_PVD_CALL getContextHandle() override;
	OmniPvdObjectHandle OMNI_PVD_CALL getObjectHandle() override;

	OmniPvdClassHandle OMNI_PVD_CALL getClassHandle() override;
	OmniPvdClassHandle OMNI_PVD_CALL getBaseClassHandle() override;
	OmniPvdAttributeHandle OMNI_PVD_CALL getAttributeHandle() override;

	const char* OMNI_PVD_CALL getClassName() override;
	const char* OMNI_PVD_CALL getAttributeName() override;
	const char* OMNI_PVD_CALL getObjectName() override;

	const uint8_t* OMNI_PVD_CALL getAttributeDataPointer() override;
	OmniPvdDataType::Enum OMNI_PVD_CALL getAttributeDataType() override;
	uint32_t OMNI_PVD_CALL getAttributeDataLength() override;
	uint32_t OMNI_PVD_CALL getAttributeNumberElements() override;
	OmniPvdClassHandle OMNI_PVD_CALL getAttributeClassHandle() override;
	
	uint8_t OMNI_PVD_CALL getAttributeNumberHandles();
	OmniPvdAttributeHandle* OMNI_PVD_CALL getAttributeHandles();

	uint64_t OMNI_PVD_CALL getFrameTimeStart() override;
	uint64_t OMNI_PVD_CALL getFrameTimeStop() override;

	bool OMNI_PVD_CALL getMessageData(const char*& message, const char*& file, uint32_t& line, uint32_t& type, OmniPvdClassHandle& handle) override;

	OmniPvdClassHandle OMNI_PVD_CALL getEnumClassHandle() override;
	uint32_t OMNI_PVD_CALL getEnumValue() override;

	// Internal helper. Returns false if the length is implausibly large (a corrupt/truncated
	// stream, e.g. a live socket FIN mid-command) or the payload read came up short, in which
	// case the caller must treat the command as invalid rather than trusting the buffer.
	bool readLongDataFromStream(OmniPvdAttributeDataLengthStorageType streamByteLen);
	bool readStringFromStream(char* string, OmniPvdStringLengthStorageType& stringLength);
	void resetCommandParams();
	void logCommand(OmniPvdCommand::Enum command);
	bool readExact(uint8_t* destination, uint64_t size, bool allowEmpty = false);
	template<class T> bool readValue(T& value)
	{
		value = T();
		return readExact(reinterpret_cast<uint8_t*>(&value), sizeof(value));
	}
	uint32_t mMaxAttributeBytes;
	uint8_t mMaxPathDepth;
	OmniPvdReaderStatus::Code mStatus;

	OmniPvdLog mLog;
	bool mVerbose;

	// Borrowed from the caller; the reader never closes or destroys it.
	OmniPvdReadStream *mStream;

	OmniPvdVersionType mMajorVersion;
	OmniPvdVersionType mMinorVersion;
	OmniPvdVersionType mPatch;
	
	OmniPvdVersionType mCmdMajorVersion;
	OmniPvdVersionType mCmdMinorVersion;
	OmniPvdVersionType mCmdPatch;
	
	OmniPvdContextHandle mCmdContextHandle;
	OmniPvdObjectHandle mCmdObjectHandle;

	OmniPvdClassHandle mCmdClassHandle;
	OmniPvdClassHandle mCmdBaseClassHandle;
	OmniPvdAttributeHandle mCmdAttributeHandle;
		
	char mCmdClassName[OMNI_PVD_MAX_STRING_LENGTH];
	char mCmdAttributeName[OMNI_PVD_MAX_STRING_LENGTH];
	char mCmdObjectName[OMNI_PVD_MAX_STRING_LENGTH];

	OmniPvdStringLengthStorageType mCmdClassNameLen;
	OmniPvdStringLengthStorageType mCmdAttributeNameLen;
	OmniPvdStringLengthStorageType mCmdObjectNameLen;

	uint8_t* mCmdAttributeDataPtr;
	OmniPvdDataType::Enum mCmdAttributeDataType;
	OmniPvdAttributeDataLengthStorageType mCmdAttributeDataLen;
	OmniPvdAttributeElementCountStorageType mCmdAttributeNbElements;
	OmniPvdEnumValueType mCmdEnumValue;
	OmniPvdClassHandle mCmdEnumClassHandle;
	OmniPvdClassHandle mCmdAttributeClassHandle;

	OmniPvdAttributeHandle mCmdAttributeHandleStack[32];
	OmniPvdAttributeHandleDepthStorageType mCmdAttributeHandleDepth;
	
	OmniPvdFrameTimestampStorageType mCmdFrameTimeStart;
	OmniPvdFrameTimestampStorageType mCmdFrameTimeStop;

	uint8_t *mDataBuffer;
	uint32_t mDataBuffAllocatedLen;

	bool mIsReadingStarted;
	uint8_t mReadBaseClassHandle;

	// Messages
	bool mCmdMessageParsed;
	OmniPvdStringLengthStorageType mCmdMessageLength;
	char mCmdMessage[OMNI_PVD_MAX_STRING_LENGTH];
	OmniPvdStringLengthStorageType mCmdMessageFileLength;
	char mCmdMessageFile[OMNI_PVD_MAX_STRING_LENGTH];
	OmniPvdMessageLineStorageType mCmdMessageLine;
	OmniPvdMessageTypeStorageType mCmdMessageType;
	OmniPvdClassHandle mCmdMessageClassHandle;
};

#endif
