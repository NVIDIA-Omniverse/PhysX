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

#ifndef OMNI_PVD_RUNTIME_READER_IMPL_H
#define OMNI_PVD_RUNTIME_READER_IMPL_H

#include "OmniPvdReader.h"
#include "OmniPvdLog.h"

class OmniPvdReaderImpl : public OmniPvdReader {
public:
	OmniPvdReaderImpl();
	~OmniPvdReaderImpl();

	void OMNI_PVD_CALL setLogFunction(OmniPvdLogFunction logFunction);
	void OMNI_PVD_CALL setReadStream(OmniPvdReadStream& stream);
	bool OMNI_PVD_CALL startReading(OmniPvdVersionType& majorVersion, OmniPvdVersionType& minorVersion, OmniPvdVersionType& patch);
	OmniPvdCommand::Enum OMNI_PVD_CALL getNextCommand();

	OmniPvdVersionType OMNI_PVD_CALL getMajorVersion();
	OmniPvdVersionType OMNI_PVD_CALL getMinorVersion();
	OmniPvdVersionType OMNI_PVD_CALL getPatch();

	OmniPvdContextHandle OMNI_PVD_CALL getContextHandle();
	OmniPvdObjectHandle OMNI_PVD_CALL getObjectHandle();

	OmniPvdClassHandle OMNI_PVD_CALL getClassHandle();
	OmniPvdClassHandle OMNI_PVD_CALL getBaseClassHandle();
	OmniPvdAttributeHandle OMNI_PVD_CALL getAttributeHandle();

	const char* OMNI_PVD_CALL getClassName();
	const char* OMNI_PVD_CALL getAttributeName();
	const char* OMNI_PVD_CALL getObjectName();

	const uint8_t* OMNI_PVD_CALL getAttributeDataPointer();
	OmniPvdDataType::Enum OMNI_PVD_CALL getAttributeDataType();
	uint32_t OMNI_PVD_CALL getAttributeDataLength();
	uint32_t OMNI_PVD_CALL getAttributeNumberElements();
	OmniPvdClassHandle OMNI_PVD_CALL getAttributeClassHandle();
	
	uint8_t OMNI_PVD_CALL getAttributeNumberHandles();
	OmniPvdAttributeHandle* OMNI_PVD_CALL getAttributeHandles();

	uint64_t OMNI_PVD_CALL getFrameTimeStart();
	uint64_t OMNI_PVD_CALL getFrameTimeStop();
	
	OmniPvdClassHandle OMNI_PVD_CALL getEnumClassHandle();
	uint32_t OMNI_PVD_CALL getEnumValue();

	// Internal helper
	void readLongDataFromStream(uint32_t streamByteLen);

	OmniPvdLog mLog;

	OmniPvdReadStream *mStream;

	OmniPvdVersionType mMajorVersion;
	OmniPvdVersionType mMinorVersion;
	OmniPvdVersionType mPatch;
	
	OmniPvdVersionType mCmdMajorVersion;
	OmniPvdVersionType mCmdMinorVersion;
	OmniPvdVersionType mCmdPatch;
	
	OmniPvdContextHandle mCmdContextHandle;
	OmniPvdObjectHandle mCmdObjectHandle;

	uint32_t mCmdClassHandle;
	uint32_t mCmdBaseClassHandle;
	uint32_t mCmdAttributeHandle;
		
	////////////////////////////////////////////////////////////////////////////////
	// TODO : take care of buffer length limit at read time!
	////////////////////////////////////////////////////////////////////////////////
	char mCmdClassName[1000];
	char mCmdAttributeName[1000];
	char mCmdObjectName[1000];

	uint16_t mCmdClassNameLen;
	uint16_t mCmdAttributeNameLen;
	uint16_t mCmdObjectNameLen;

	uint8_t* mCmdAttributeDataPtr;
	OmniPvdDataType::Enum mCmdAttributeDataType;
	uint32_t mCmdAttributeDataLen;
	uint32_t mCmdAttributeNbElements;
	uint32_t mCmdEnumValue;
	OmniPvdClassHandle mCmdEnumClassHandle;
	OmniPvdClassHandle mCmdAttributeClassHandle;

	OmniPvdAttributeHandle mCmdAttributeHandleStack[32];
	uint8_t mCmdAttributeHandleDepth;
	
	uint64_t mCmdFrameTimeStart;
	uint64_t mCmdFrameTimeStop;

	uint8_t *mDataBuffer;
	uint32_t mDataBuffAllocatedLen;

	bool mIsReadingStarted;
	uint8_t mReadBaseClassHandle;
};

#endif
