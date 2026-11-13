// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OmniPvdDefinesInternal.h"
#include "OmniPvdReaderImpl.h"

#include <inttypes.h>
#include <new>
#include <stdio.h>

namespace
{
uint32_t getDataBufferCapacity(uint32_t streamByteLen, uint32_t maxAttributeBytes)
{
	const uint64_t grown = uint64_t(streamByteLen) * 13 / 10;
	return uint32_t(grown < maxAttributeBytes ? grown : maxAttributeBytes);
}

const char* commandName(OmniPvdCommand::Enum command)
{
	switch (command)
	{
	case OmniPvdCommand::eREGISTER_CLASS: return "REGISTER_CLASS";
	case OmniPvdCommand::eREGISTER_ATTRIBUTE: return "REGISTER_ATTRIBUTE";
	case OmniPvdCommand::eREGISTER_CLASS_ATTRIBUTE: return "REGISTER_CLASS_ATTRIBUTE";
	case OmniPvdCommand::eREGISTER_UNIQUE_LIST_ATTRIBUTE: return "REGISTER_UNIQUE_LIST_ATTRIBUTE";
	case OmniPvdCommand::eSET_ATTRIBUTE: return "SET_ATTRIBUTE";
	case OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE: return "ADD_TO_UNIQUE_LIST_ATTRIBUTE";
	case OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE: return "REMOVE_FROM_UNIQUE_LIST_ATTRIBUTE";
	case OmniPvdCommand::eCREATE_OBJECT: return "CREATE_OBJECT";
	case OmniPvdCommand::eDESTROY_OBJECT: return "DESTROY_OBJECT";
	case OmniPvdCommand::eSTART_FRAME: return "START_FRAME";
	case OmniPvdCommand::eSTOP_FRAME: return "STOP_FRAME";
	case OmniPvdCommand::eRECORD_MESSAGE: return "RECORD_MESSAGE";
	default: return "UNKNOWN";
	}
}

const char* dataTypeName(OmniPvdDataType::Enum type)
{
	static const char* const names[] = {"INT8", "INT16", "INT32", "INT64", "UINT8", "UINT16", "UINT32", "UINT64",
		"FLOAT32", "FLOAT64", "STRING", "OBJECT_HANDLE", "ENUM_VALUE", "FLAGS_WORD"};
	const unsigned index = static_cast<unsigned>(type);
	return index < sizeof(names) / sizeof(names[0]) ? names[index] : "UNKNOWN";
}

void emitReaderTextChunks(OmniPvdLog& log, const char* command, const char* field, const char* bytes, uint16_t encodedLength)
{
	const unsigned retained = encodedLength < OMNI_PVD_MAX_STRING_LENGTH ? encodedLength : OMNI_PVD_MAX_STRING_LENGTH - 1;
	const unsigned chunks = retained ? (retained + 383) / 384 : 1;
	const char hex[] = "0123456789ABCDEF";
	for (unsigned chunk = 0; chunk < chunks; ++chunk)
	{
		const unsigned offset = chunk * 384;
		const unsigned count = retained - offset < 384 ? retained - offset : 384;
		// At most four ASCII bytes per retained byte. Framing plus NUL is below 256 bytes,
		// so the complete callback stays below OmniPvdLog's 2048-byte output buffer.
		char escaped[4 * 384 + 1];
		unsigned length = 0;
		for (unsigned i = 0; i < count; ++i)
		{
			const unsigned char value = static_cast<unsigned char>(bytes[offset + i]);
			if (value < 0x20 || value >= 0x7f)
			{
				escaped[length++] = '\\'; escaped[length++] = 'x';
				escaped[length++] = hex[value >> 4]; escaped[length++] = hex[value & 15];
			}
			else
			{
				if (value == '\\' || value == '"') escaped[length++] = '\\';
				escaped[length++] = static_cast<char>(value);
			}
		}
		escaped[length] = '\0';
		log.outputLine("[parser] text command=%s field=%s chunk=%u/%u offset=%u bytes=%u encoded=%u retained=%u truncated=%u value=\"%s\"",
			command, field, chunk + 1, chunks, offset, count, static_cast<unsigned>(encodedLength), retained,
			encodedLength > retained ? 1u : 0u, escaped);
	}
}
}

OmniPvdReaderImpl::OmniPvdReaderImpl(uint32_t maxAttributeBytes, uint8_t maxPathDepth)
{
	mVerbose = false;
	mMaxAttributeBytes = maxAttributeBytes;
	mMaxPathDepth = maxPathDepth < 32 ? maxPathDepth : 32;
	mStatus = OmniPvdReaderStatus::eNONE;
	mCmdMajorVersion = mCmdMinorVersion = mCmdPatch = 0;
	mCmdFrameTimeStart = mCmdFrameTimeStop = 0;
	resetCommandParams();
	mMajorVersion = OMNI_PVD_VERSION_MAJOR;
	mMinorVersion = OMNI_PVD_VERSION_MINOR;
	mPatch = OMNI_PVD_VERSION_PATCH;
	mStream = 0;
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
}

bool OMNI_PVD_CALL OmniPvdReaderImpl::startReading(OmniPvdVersionType& majorVersion, OmniPvdVersionType& minorVersion, OmniPvdVersionType& patch)
{
	if (!mStream || !mStream->openStream())
	{
		return false;
	}

	if (mIsReadingStarted)
	{
		return true;
	}
	// Header retries require the caller to rewind after a partial read.
	if (mStatus == OmniPvdReaderStatus::eINVALID || mStatus == OmniPvdReaderStatus::eEND_OF_STREAM)
		mStatus = OmniPvdReaderStatus::eNONE;

	OmniPvdVersionType streamMajor = 0;
	OmniPvdVersionType streamMinor = 0;
	OmniPvdVersionType streamPatch = 0;
	if (!readExact(reinterpret_cast<uint8_t*>(&streamMajor), sizeof(streamMajor), true) ||
		!readValue(streamMinor) || !readValue(streamPatch))
	{
		return false;
	}

	majorVersion = streamMajor;
	minorVersion = streamMinor;
	patch = streamPatch;
	const char* unsupportedReason = nullptr;
	if (streamMajor > mMajorVersion)
		unsupportedReason = "[parser] major version too new\n";
	else if (streamMajor == mMajorVersion && streamMinor > mMinorVersion)
		unsupportedReason = "[parser] minor version too new\n";
	else if (streamMajor == mMajorVersion && streamMinor == mMinorVersion && streamPatch > mPatch)
		unsupportedReason = "[parser] patch too new\n";
	if (mLog.mLogFunction && (mVerbose || unsupportedReason))
		mLog.outputLine("OmniPvdRuntimeReaderImpl::startReading majorVersion(%lu), minorVersion(%lu), patch(%lu)",
			static_cast<unsigned long>(streamMajor), static_cast<unsigned long>(streamMinor),
			static_cast<unsigned long>(streamPatch));
	if (unsupportedReason)
	{
		mStatus = OmniPvdReaderStatus::eUNSUPPORTED;
		mLog.outputLine("%s", unsupportedReason);
		return false;
	}

	mCmdMajorVersion = streamMajor;
	mCmdMinorVersion = streamMinor;
	mCmdPatch = streamPatch;
	mCmdBaseClassHandle = 0;
	mReadBaseClassHandle = (streamMajor == 0 && streamMinor < 3) ? 0 : 1;
	mIsReadingStarted = true;
	return true;
}

bool OmniPvdReaderImpl::readExact(uint8_t* destination, uint64_t size, bool allowEmpty)
{
	// Propagate terminal status without reading or repeating its diagnostic.
	if (mStatus != OmniPvdReaderStatus::eNONE) return false;
	const uint64_t read = size && mStream ? mStream->readBytes(destination, size) : 0;
	if (size && read != size)
	{
		if (allowEmpty && !read) return false;
		// ### DEFENSIVE OMPE-112489: Incomplete fields must not reach command handlers.
		mStatus = OmniPvdReaderStatus::eINVALID;
		mLog.outputLine("[parser] ERROR: failed to read complete field\n");
		return false;
	}
	return true;
}

bool OmniPvdReaderImpl::readStringFromStream(char* string, OmniPvdStringLengthStorageType& stringLength)
{
	string[0] = 0;
	if (!readValue(stringLength)) return false;
	const uint16_t keptLength = stringLength < OMNI_PVD_MAX_STRING_LENGTH ? stringLength : OMNI_PVD_MAX_STRING_LENGTH - 1;
	if (!readExact(reinterpret_cast<uint8_t*>(string), keptLength))
	{
		string[0] = 0;
		return false;
	}
	string[keptLength] = 0;
	uint16_t remaining = stringLength - keptLength;
	while (remaining)
	{
		// Read discarded bytes so truncated tails cannot be hidden by a seek beyond EOF.
		uint8_t discarded[256];
		const uint16_t count = remaining < sizeof(discarded) ? remaining : sizeof(discarded);
		if (!readExact(discarded, count))
		{
			string[0] = 0;
			return false;
		}
		remaining -= count;
	}
	return true;
}

OmniPvdCommand::Enum OMNI_PVD_CALL OmniPvdReaderImpl::getNextCommand()
{
	if (mStatus == OmniPvdReaderStatus::eEND_OF_STREAM) mStatus = OmniPvdReaderStatus::eNONE;
	if (mStatus != OmniPvdReaderStatus::eNONE) return OmniPvdCommand::eINVALID;
	if (!mIsReadingStarted && !startReading(mCmdMajorVersion, mCmdMinorVersion, mCmdPatch))
		return OmniPvdCommand::eINVALID;
	if (mStatus != OmniPvdReaderStatus::eNONE) return OmniPvdCommand::eINVALID;
	resetCommandParams();
	OmniPvdCommandStorageType command = 0;
	if (!mStream->readBytes(&command, sizeof(command)))
	{
		mStatus = OmniPvdReaderStatus::eEND_OF_STREAM;
		return OmniPvdCommand::eINVALID;
	}
	switch (command)
	{
	case OmniPvdCommand::eREGISTER_CLASS:
		readValue(mCmdClassHandle);
		if (mReadBaseClassHandle) readValue(mCmdBaseClassHandle);
		readStringFromStream(mCmdClassName, mCmdClassNameLen);
		break;
	case OmniPvdCommand::eREGISTER_ATTRIBUTE:
	case OmniPvdCommand::eREGISTER_UNIQUE_LIST_ATTRIBUTE:
	{
		readValue(mCmdClassHandle);
		readValue(mCmdAttributeHandle);
		OmniPvdDataTypeStorageType type = 0;
		if (!readValue(type)) break;
		if (type > OmniPvdDataType::eFLAGS_WORD)
		{
			mStatus = OmniPvdReaderStatus::eINVALID;
			mLog.outputLine("[parser] ERROR: invalid attribute data type\n");
			break;
		}
		mCmdAttributeDataType = static_cast<OmniPvdDataType::Enum>(type);
		if (command == OmniPvdCommand::eREGISTER_ATTRIBUTE)
		{
			if (mCmdAttributeDataType == OmniPvdDataType::eENUM_VALUE) readValue(mCmdEnumValue);
			else if (mCmdAttributeDataType == OmniPvdDataType::eFLAGS_WORD) readValue(mCmdEnumClassHandle);
			else readValue(mCmdAttributeNbElements);
		}
		readStringFromStream(mCmdAttributeName, mCmdAttributeNameLen);
		break;
	}
	case OmniPvdCommand::eREGISTER_CLASS_ATTRIBUTE:
		readValue(mCmdClassHandle);
		readValue(mCmdAttributeHandle);
		readValue(mCmdAttributeClassHandle);
		readStringFromStream(mCmdAttributeName, mCmdAttributeNameLen);
		break;
	case OmniPvdCommand::eSET_ATTRIBUTE:
	case OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE:
	case OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE:
		readValue(mCmdContextHandle);
		readValue(mCmdObjectHandle);
		if (!readValue(mCmdAttributeHandleDepth)) break;
		if (mCmdAttributeHandleDepth > mMaxPathDepth)
		{
			mStatus = OmniPvdReaderStatus::eLIMIT;
			mLog.outputLine("[parser] ERROR: attribute path depth %u exceeds limit %u\n",
				static_cast<unsigned>(mCmdAttributeHandleDepth), static_cast<unsigned>(mMaxPathDepth));
			break;
		}
		if (!mCmdAttributeHandleDepth)
		{
			mStatus = OmniPvdReaderStatus::eINVALID;
			mLog.outputLine("[parser] ERROR: zero attribute path depth\n");
			break;
		}
		for (unsigned i = 0; i < mCmdAttributeHandleDepth; ++i)
		{
			if (!readValue(mCmdAttributeHandleStack[i])) break;
			mCmdAttributeHandle = mCmdAttributeHandleStack[i];
		}
		if (readValue(mCmdAttributeDataLen)) readLongDataFromStream(mCmdAttributeDataLen);
		break;
	case OmniPvdCommand::eCREATE_OBJECT:
		readValue(mCmdContextHandle);
		readValue(mCmdClassHandle);
		readValue(mCmdObjectHandle);
		readStringFromStream(mCmdObjectName, mCmdObjectNameLen);
		break;
	case OmniPvdCommand::eDESTROY_OBJECT:
		readValue(mCmdContextHandle);
		readValue(mCmdObjectHandle);
		break;
	case OmniPvdCommand::eSTART_FRAME:
		readValue(mCmdContextHandle);
		readValue(mCmdFrameTimeStart);
		break;
	case OmniPvdCommand::eSTOP_FRAME:
		readValue(mCmdContextHandle);
		readValue(mCmdFrameTimeStop);
		break;
	case OmniPvdCommand::eRECORD_MESSAGE:
		readValue(mCmdContextHandle);
		readStringFromStream(mCmdMessage, mCmdMessageLength);
		readStringFromStream(mCmdMessageFile, mCmdMessageFileLength);
		readValue(mCmdMessageLine);
		readValue(mCmdMessageType);
		readValue(mCmdMessageClassHandle);
		mCmdMessageParsed = mStatus == OmniPvdReaderStatus::eNONE;
		break;
	case OmniPvdCommand::eREGISTER_ENUM:
		mStatus = OmniPvdReaderStatus::eUNSUPPORTED;
		mLog.outputLine("[parser] ERROR: unsupported command eREGISTER_ENUM\n");
		break;
	default:
		mStatus = OmniPvdReaderStatus::eINVALID;
		mLog.outputLine("[parser] ERROR: unknown command %u\n", static_cast<unsigned>(command));
		break;
	}
	if (mStatus == OmniPvdReaderStatus::eNONE && mVerbose && mLog.mLogFunction)
		logCommand(static_cast<OmniPvdCommand::Enum>(command));
	return mStatus == OmniPvdReaderStatus::eNONE ? static_cast<OmniPvdCommand::Enum>(command) : OmniPvdCommand::eINVALID;
}

void OmniPvdReaderImpl::logCommand(OmniPvdCommand::Enum command)
{
	const char* name = commandName(command);
	const unsigned opcode = static_cast<unsigned>(command);
	switch (command)
	{
	case OmniPvdCommand::eREGISTER_CLASS:
		if (mReadBaseClassHandle)
			mLog.outputLine("[parser] trace command=%s opcode=%u class=%" PRIu32 " base=%" PRIu32,
				name, opcode, mCmdClassHandle, mCmdBaseClassHandle);
		else
			mLog.outputLine("[parser] trace command=%s opcode=%u class=%" PRIu32, name, opcode, mCmdClassHandle);
		emitReaderTextChunks(mLog, name, "className", mCmdClassName, mCmdClassNameLen);
		break;
	case OmniPvdCommand::eREGISTER_ATTRIBUTE:
	case OmniPvdCommand::eREGISTER_UNIQUE_LIST_ATTRIBUTE:
		if (command == OmniPvdCommand::eREGISTER_UNIQUE_LIST_ATTRIBUTE)
			mLog.outputLine("[parser] trace command=%s opcode=%u class=%" PRIu32 " attribute=%" PRIu32 " type=%s typeId=%u",
				name, opcode, mCmdClassHandle, mCmdAttributeHandle, dataTypeName(mCmdAttributeDataType), static_cast<unsigned>(mCmdAttributeDataType));
		else if (mCmdAttributeDataType == OmniPvdDataType::eENUM_VALUE)
			mLog.outputLine("[parser] trace command=%s opcode=%u class=%" PRIu32 " attribute=%" PRIu32 " type=%s typeId=%u enumValue=%" PRIu32,
				name, opcode, mCmdClassHandle, mCmdAttributeHandle, dataTypeName(mCmdAttributeDataType), static_cast<unsigned>(mCmdAttributeDataType), mCmdEnumValue);
		else if (mCmdAttributeDataType == OmniPvdDataType::eFLAGS_WORD)
			mLog.outputLine("[parser] trace command=%s opcode=%u class=%" PRIu32 " attribute=%" PRIu32 " type=%s typeId=%u enumClass=%" PRIu32,
				name, opcode, mCmdClassHandle, mCmdAttributeHandle, dataTypeName(mCmdAttributeDataType), static_cast<unsigned>(mCmdAttributeDataType), mCmdEnumClassHandle);
		else
			mLog.outputLine("[parser] trace command=%s opcode=%u class=%" PRIu32 " attribute=%" PRIu32 " type=%s typeId=%u elements=%" PRIu32,
				name, opcode, mCmdClassHandle, mCmdAttributeHandle, dataTypeName(mCmdAttributeDataType), static_cast<unsigned>(mCmdAttributeDataType), mCmdAttributeNbElements);
		emitReaderTextChunks(mLog, name, "attributeName", mCmdAttributeName, mCmdAttributeNameLen);
		break;
	case OmniPvdCommand::eREGISTER_CLASS_ATTRIBUTE:
		mLog.outputLine("[parser] trace command=%s opcode=%u class=%" PRIu32 " attribute=%" PRIu32 " attributeClass=%" PRIu32,
			name, opcode, mCmdClassHandle, mCmdAttributeHandle, mCmdAttributeClassHandle);
		emitReaderTextChunks(mLog, name, "attributeName", mCmdAttributeName, mCmdAttributeNameLen);
		break;
	case OmniPvdCommand::eSET_ATTRIBUTE:
	case OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE:
	case OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE:
	{
		char path[355];
		static_assert(sizeof(path) >=
			(sizeof(mCmdAttributeHandleStack) / sizeof(mCmdAttributeHandleStack[0])) * 11 + 2,
			"Diagnostic paths must fit every 32-bit handle, separators, brackets and the terminator.");
		path[0] = '[';
		size_t length = 1;
		for (unsigned i = 0; i < mCmdAttributeHandleDepth; ++i)
		{
			const int written = snprintf(path + length, sizeof(path) - length,
				i ? ",%" PRIu32 : "%" PRIu32, mCmdAttributeHandleStack[i]);
			length += static_cast<size_t>(written);
		}
		path[length++] = ']';
		path[length] = '\0';
		mLog.outputLine("[parser] trace command=%s opcode=%u context=%" PRIu64 " object=%" PRIu64 " depth=%u path=%s bytes=%" PRIu32,
			name, opcode, mCmdContextHandle, mCmdObjectHandle, static_cast<unsigned>(mCmdAttributeHandleDepth), path, mCmdAttributeDataLen);
		break;
	}
	case OmniPvdCommand::eCREATE_OBJECT:
		mLog.outputLine("[parser] trace command=%s opcode=%u context=%" PRIu64 " object=%" PRIu64 " class=%" PRIu32,
			name, opcode, mCmdContextHandle, mCmdObjectHandle, mCmdClassHandle);
		emitReaderTextChunks(mLog, name, "objectName", mCmdObjectName, mCmdObjectNameLen);
		break;
	case OmniPvdCommand::eDESTROY_OBJECT:
		mLog.outputLine("[parser] trace command=%s opcode=%u context=%" PRIu64 " object=%" PRIu64,
			name, opcode, mCmdContextHandle, mCmdObjectHandle);
		break;
	case OmniPvdCommand::eSTART_FRAME:
	case OmniPvdCommand::eSTOP_FRAME:
		mLog.outputLine("[parser] trace command=%s opcode=%u context=%" PRIu64 " timestamp=%" PRIu64,
			name, opcode, mCmdContextHandle, command == OmniPvdCommand::eSTART_FRAME ? mCmdFrameTimeStart : mCmdFrameTimeStop);
		break;
	case OmniPvdCommand::eRECORD_MESSAGE:
		mLog.outputLine("[parser] trace command=%s opcode=%u context=%" PRIu64 " line=%" PRIu32 " type=%" PRIu32 " class=%" PRIu32,
			name, opcode, mCmdContextHandle, mCmdMessageLine, mCmdMessageType, mCmdMessageClassHandle);
		emitReaderTextChunks(mLog, name, "message", mCmdMessage, mCmdMessageLength);
		emitReaderTextChunks(mLog, name, "file", mCmdMessageFile, mCmdMessageFileLength);
		break;
	default:
		break;
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

bool OmniPvdReaderImpl::readLongDataFromStream(OmniPvdAttributeDataLengthStorageType streamByteLen)
{
	if (mStatus != OmniPvdReaderStatus::eNONE) return false;
	if (streamByteLen > mMaxAttributeBytes)
	{
		mStatus = OmniPvdReaderStatus::eLIMIT;
		mLog.outputLine("[parser] ERROR: attribute payload %u bytes exceeds limit %u\n", streamByteLen, mMaxAttributeBytes);
		return false;
	}
	if (!streamByteLen) return true;
	if (streamByteLen > mDataBuffAllocatedLen)
	{
		const uint32_t allocationSize = ::getDataBufferCapacity(streamByteLen, mMaxAttributeBytes);
		uint8_t* buffer = new (std::nothrow) uint8_t[allocationSize];
		if (!buffer)
		{
			mStatus = OmniPvdReaderStatus::eLIMIT;
			mLog.outputLine("[parser] ERROR: attribute payload allocation failed\n");
			return false;
		}
		delete[] mDataBuffer;
		mDataBuffer = buffer;
		mDataBuffAllocatedLen = allocationSize;
	}
	mCmdAttributeDataPtr = mDataBuffer;
	return readExact(mCmdAttributeDataPtr, streamByteLen);
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
	mCmdAttributeDataType = OmniPvdDataType::eINT8;
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
