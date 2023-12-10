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

#ifndef NP_OMNI_PVD_H
#define NP_OMNI_PVD_H

#include "omnipvd/PxOmniPvd.h"
#include "foundation/PxMutex.h"

class OmniPvdReader;
class OmniPvdWriter;
class OmniPvdLoader;
class OmniPvdFileWriteStream;
class OmniPvdPxSampler;

namespace physx
{

class NpOmniPvd : public PxOmniPvd
{
public:
	NpOmniPvd();
	~NpOmniPvd();
	static void destroyInstance();
	static void incRefCount();
	static void decRefCount();
	void release();

	bool initOmniPvd();
	
	OmniPvdWriter* getWriter();
	
	OmniPvdWriter* blockingWriterLoad();

	OmniPvdWriter* acquireExclusiveWriterAccess();
	void releaseExclusiveWriterAccess();

	OmniPvdFileWriteStream* getFileWriteStream();
	bool startSampling();
	
	OmniPvdLoader* mLoader;
	OmniPvdFileWriteStream* mFileWriteStream;
	OmniPvdWriter* mWriter;
	OmniPvdPxSampler* mPhysXSampler;	
	static PxU32 mRefCount;
	static NpOmniPvd* mInstance;
	PxMutex mMutex;
	PxMutex mWriterLoadMutex;
};

}

#endif
