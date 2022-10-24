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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef OMNI_PVD_READER_H
#define OMNI_PVD_READER_H

#include "OmniPvdDefines.h"
#include "OmniPvdCommands.h"
#include "OmniPvdReadStream.h"

class OmniPvdReader
{
public:
	virtual ~OmniPvdReader()
	{
	}

	virtual void OMNI_PVD_CALL setLogFunction(OmniPvdLogFunction logFunction) = 0;
	virtual void OMNI_PVD_CALL setReadStream(OmniPvdReadStream* stream) = 0;	
	virtual bool OMNI_PVD_CALL startReading(OmniPvdVersionType* majorVersion, OmniPvdVersionType* minorVersion, OmniPvdVersionType* patch) = 0;
	virtual OmniPvdCommandEnum::Enum OMNI_PVD_CALL getNextCommand() = 0;
	virtual OmniPvdCommandEnum::Enum OMNI_PVD_CALL getCommandType() = 0;

	virtual OmniPvdVersionType OMNI_PVD_CALL getMajorVersion() = 0;
	virtual OmniPvdVersionType OMNI_PVD_CALL getMinorVersion() = 0;
	virtual OmniPvdVersionType OMNI_PVD_CALL getPatch() = 0;

	virtual OmniPvdContextHandle OMNI_PVD_CALL getContextHandle() = 0;
	virtual OmniPvdObjectHandle OMNI_PVD_CALL getObjectHandle() = 0;
	virtual OmniPvdClassHandle OMNI_PVD_CALL getClassHandle() = 0;
	virtual OmniPvdClassHandle OMNI_PVD_CALL getBaseClassHandle() = 0;
	virtual OmniPvdAttributeHandle OMNI_PVD_CALL getAttributeHandle() = 0;
	
	virtual char* OMNI_PVD_CALL getClassName() = 0;
	virtual char* OMNI_PVD_CALL getAttributeName() = 0;
	virtual char* OMNI_PVD_CALL getObjectName() = 0;

	virtual uint8_t* OMNI_PVD_CALL getAttributeDataPointer() = 0;
	virtual OmniPvdAttributeDataType OMNI_PVD_CALL getAttributeDataType() = 0;
	virtual uint32_t OMNI_PVD_CALL getAttributeDataLength() = 0;
	virtual uint32_t OMNI_PVD_CALL getAttributeNumberElements() = 0;
	virtual OmniPvdClassHandle OMNI_PVD_CALL getAttributeClassHandle() = 0;

	virtual uint64_t OMNI_PVD_CALL getFrameTimeStart() = 0;
	virtual uint64_t OMNI_PVD_CALL getFrameTimeStop() = 0;

	virtual OmniPvdClassHandle OMNI_PVD_CALL getEnumClassHandle() = 0;
	virtual uint32_t OMNI_PVD_CALL getEnumValue() = 0;
};

#endif
