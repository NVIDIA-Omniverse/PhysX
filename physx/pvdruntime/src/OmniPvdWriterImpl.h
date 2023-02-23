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

#ifndef OMNI_PVD_WRITER_IMPL_H
#define OMNI_PVD_WRITER_IMPL_H

#include "OmniPvdWriter.h"
#include "OmniPvdLog.h"

class OmniPvdWriterImpl : public OmniPvdWriter {
public:
	OmniPvdWriterImpl();
	~OmniPvdWriterImpl();
	void OMNI_PVD_CALL setLogFunction(OmniPvdLogFunction logFunction);
	void setVersionHelper();
	void setVersion(const OmniPvdVersionType majorVersion, const OmniPvdVersionType minorVersion, const OmniPvdVersionType patch);
	void OMNI_PVD_CALL setWriteStream(const OmniPvdWriteStream* stream);
	OmniPvdWriteStream* OMNI_PVD_CALL getWriteStream();

	OmniPvdClassHandle OMNI_PVD_CALL registerClass(const char* className, OmniPvdClassHandle baseClass);
	OmniPvdAttributeHandle OMNI_PVD_CALL registerEnumValue(const OmniPvdClassHandle classHandle, const char* attributeName, const uint32_t value);
	OmniPvdAttributeHandle OMNI_PVD_CALL registerAttribute(const OmniPvdClassHandle classHandle, const char* attributeName, const OmniPvdAttributeDataType attributeDataType, const uint32_t nbrFields);
	OmniPvdAttributeHandle OMNI_PVD_CALL registerFlagsAttribute(const OmniPvdClassHandle classHandle, const OmniPvdClassHandle enumClassHandle, const char* attributeName);
	OmniPvdAttributeHandle OMNI_PVD_CALL registerClassAttribute(const OmniPvdClassHandle classHandle, const char* attributeName, const OmniPvdClassHandle classAttributeHandle);
	OmniPvdAttributeHandle OMNI_PVD_CALL registerSetAttribute(const OmniPvdClassHandle classHandle, const char* attributeName, const OmniPvdAttributeDataType attributeDataType);
	void OMNI_PVD_CALL setAttribute(const OmniPvdContextHandle contextHandle, const OmniPvdObjectHandle objectHandle, const uint8_t handleDepth, const OmniPvdAttributeHandle* attributeHandles, const uint8_t* data, const uint32_t nbrBytes);
	void OMNI_PVD_CALL setAttributeShallow(const OmniPvdContextHandle contextHandle, const OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle attributeHandle, const uint8_t *data, const uint32_t nbrBytes);
	
	void OMNI_PVD_CALL addToSetAttribute(const OmniPvdContextHandle contextHandle, const OmniPvdObjectHandle objectHandle, const uint8_t handleDepth, const OmniPvdAttributeHandle* attributeHandles, const uint8_t* data, const uint32_t nbrBytes);
	void OMNI_PVD_CALL addToSetAttributeShallow(const OmniPvdContextHandle contextHandle, const OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle attributeHandle, const uint8_t* data, const uint32_t nbrBytes);

	void OMNI_PVD_CALL removeFromSetAttribute(const OmniPvdContextHandle contextHandle, const OmniPvdObjectHandle objectHandle, const uint8_t handleDepth, const OmniPvdAttributeHandle* attributeHandles, const uint8_t* data, const uint32_t nbrBytes);
	void OMNI_PVD_CALL removeFromSetAttributeShallow(const OmniPvdContextHandle contextHandle, const OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle attributeHandle, const uint8_t* data, const uint32_t nbrBytes);

	void OMNI_PVD_CALL createObject(const OmniPvdContextHandle contextHandle, const OmniPvdClassHandle classHandle, const OmniPvdObjectHandle objectHandle, const char* objectName);
	void OMNI_PVD_CALL destroyObject(const OmniPvdContextHandle contextHandle, const OmniPvdObjectHandle objectHandle);
	void OMNI_PVD_CALL startFrame(const OmniPvdContextHandle contextHandle, const uint64_t timeStamp);
	void OMNI_PVD_CALL stopFrame(const OmniPvdContextHandle contextHandle, const uint64_t timeStamp);
	
	bool mIsFirstWrite;
	OmniPvdLog mLog;
	OmniPvdWriteStream *mStream;
	int mLastClassHandle;
	int mLastAttributeHandle;};

#endif
