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

#include "NpOmniPvd.h"

#if PX_SUPPORT_OMNI_PVD
#include "OmniPvdPxSampler.h"
#include "OmniPvdLoader.h"
#include "OmniPvdFileWriteStream.h"
#endif
#include "foundation/PxUserAllocated.h"

physx::PxU32 physx::NpOmniPvd::mRefCount = 0;
physx::NpOmniPvd* physx::NpOmniPvd::mInstance = NULL;

namespace physx
{
	NpOmniPvd::NpOmniPvd() :
		mLoader			(NULL),
		mFileWriteStream(NULL),
		mWriter			(NULL),
		mPhysXSampler	(NULL)
	{
	}

	NpOmniPvd::~NpOmniPvd()
	{
#if PX_SUPPORT_OMNI_PVD
		if (mFileWriteStream)
		{
			mFileWriteStream->closeStream();
			mLoader->mDestroyOmniPvdFileWriteStream(*mFileWriteStream);
			mFileWriteStream = NULL;
		}
		if (mWriter)
		{
			mLoader->mDestroyOmniPvdWriter(*mWriter);
			mWriter = NULL;
		}
		if (mLoader)
		{
			mLoader->~OmniPvdLoader();
			PX_FREE(mLoader);
			mLoader = NULL;
		}
#endif
	}

	void NpOmniPvd::destroyInstance()
	{
		PX_ASSERT(mInstance != NULL);
		if (mInstance->mRefCount == 1)
		{				
			mInstance->~NpOmniPvd();			
			PX_FREE(mInstance);
			mInstance = NULL;			
		}
	}

	// Called once by physx::PxOmniPvd* PxCreateOmniPvd(...)
	// Called once by NpPhysics::NpPhysics(...)
	void NpOmniPvd::incRefCount()
	{
		PX_ASSERT(mInstance != NULL);
		NpOmniPvd::mRefCount++;
	}

	// Called once by the Physics destructor in NpPhysics::~NpPhysics(...)
	void NpOmniPvd::decRefCount()
	{
		PX_ASSERT(mInstance != NULL);
		if (NpOmniPvd::mRefCount > 0)
		{
			NpOmniPvd::mRefCount--;
		}
	}

	void NpOmniPvd::release()
	{
		NpOmniPvd::destroyInstance();
	}

	bool NpOmniPvd::initOmniPvd()
	{
#if PX_SUPPORT_OMNI_PVD
		if (mLoader)
		{
			return true;
		}

		mLoader = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(OmniPvdLoader), "OmniPvdLoader"), OmniPvdLoader)();

		if (!mLoader)
		{
			return false;
		}

		bool success;
#if PX_WIN64
		success = mLoader->loadOmniPvd("PVDRuntime_64.dll");
#else
		success = mLoader->loadOmniPvd("libPVDRuntime_64.so");
#endif
		return success;
#else
		return true;
#endif
	}

	OmniPvdWriter* NpOmniPvd::getWriter()
	{
		return blockingWriterLoad();
	}

	OmniPvdWriter* NpOmniPvd::blockingWriterLoad()
	{
#if PX_SUPPORT_OMNI_PVD
		PxMutex::ScopedLock lock(mWriterLoadMutex);
		if (mWriter)
		{
			return mWriter;
		}
		if (mLoader == NULL)
		{
			return NULL;
		}
		mWriter = mLoader->mCreateOmniPvdWriter();
		return mWriter;
#else
		return NULL;
#endif		
	}

	OmniPvdWriter* NpOmniPvd::acquireExclusiveWriterAccess()
	{
#if PX_SUPPORT_OMNI_PVD
		mMutex.lock();
		return blockingWriterLoad();
#else
		return NULL;
#endif
	}
	
	void NpOmniPvd::releaseExclusiveWriterAccess()
	{
#if PX_SUPPORT_OMNI_PVD
		mMutex.unlock();
#endif
	}


	OmniPvdFileWriteStream* NpOmniPvd::getFileWriteStream()
	{
#if PX_SUPPORT_OMNI_PVD
		if (mFileWriteStream)
		{
			return mFileWriteStream;
		}
		if (mLoader == NULL)
		{
			return NULL;
		}
		mFileWriteStream = mLoader->mCreateOmniPvdFileWriteStream();
		return mFileWriteStream;
#else
		return NULL;
#endif
	}

	bool NpOmniPvd::startSampling()
	{
#if PX_SUPPORT_OMNI_PVD
		if (mPhysXSampler)
		{
			mPhysXSampler->startSampling();
			return true;
		}		
		return false;
#else
		return false;
#endif
	}
}

physx::PxOmniPvd* PxCreateOmniPvd(physx::PxFoundation& foundation)
{
	PX_UNUSED(foundation);
#if PX_SUPPORT_OMNI_PVD
	if (physx::NpOmniPvd::mInstance)
	{
		// No need to call this function again
		//foundation.getErrorCallback()
		return physx::NpOmniPvd::mInstance;
	}	
	physx::NpOmniPvd::mInstance = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(physx::NpOmniPvd), "NpOmniPvd"), physx::NpOmniPvd)();
	
	if (physx::NpOmniPvd::mInstance)
	{
		if (physx::NpOmniPvd::mInstance->initOmniPvd())
		{
			physx::NpOmniPvd::mRefCount = 1; // Sets the reference counter to exactly 1
			return physx::NpOmniPvd::mInstance;
		}
		else
		{
			physx::NpOmniPvd::mInstance->~NpOmniPvd();
			PX_FREE(physx::NpOmniPvd::mInstance);
			physx::NpOmniPvd::mInstance = NULL;
			physx::NpOmniPvd::mRefCount = 0;
			return NULL;
		}
	}
	else
	{
		return NULL;
	}
#else
	return NULL;
#endif
}


