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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "NpOmniPvd.h"

#if PX_SUPPORT_OMNI_PVD
#include "OmniPvdPxSampler.h"
#include "OmniPvdLoader.h"
#include "OmniPvdFileWriteStream.h"
#include "OmniPvdSocketWriteStream.h"
#include "OmniPvdWriter.h"
#endif
#include "foundation/PxUserAllocated.h"
#include "foundation/PxFoundation.h"
#include "foundation/PxErrorCallback.h"
#include "foundation/PxErrors.h"

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

	OmniPvdSocketWriteStream* NpOmniPvd::createSocketWriteStream(const char* address, PxU16 port, PxU32 sendTimeout)
	{
#if PX_SUPPORT_OMNI_PVD
		if (mLoader == NULL || mLoader->mCreateOmniPvdSocketWriteStream == NULL)
		{
			return NULL;
		}
		return mLoader->mCreateOmniPvdSocketWriteStream(address, port, sendTimeout);
#else
		PX_UNUSED(address);
		PX_UNUSED(port);
		PX_UNUSED(sendTimeout);
		return NULL;
#endif
	}

	void NpOmniPvd::releaseSocketWriteStream(OmniPvdSocketWriteStream& stream)
	{
#if PX_SUPPORT_OMNI_PVD
		// The caller owns the stream and its lifetime: keep the write stream alive (and bound)
		// until PhysX is done writing to it. If a recording is still active at teardown, the
		// NpScene / NpPhysics destructors emit object-remove commands to the bound stream, so
		// release the PxScene / PxPhysics that drive the recording before releasing the stream,
		// and keep the stream alive until PxOmniPvd is released. This call just frees the
		// caller-owned stream.
		if (mLoader && mLoader->mDestroyOmniPvdSocketWriteStream)
		{
			mLoader->mDestroyOmniPvdSocketWriteStream(stream);
		}
#else
		PX_UNUSED(stream);
#endif
	}

	bool NpOmniPvd::startSampling()
	{
#if PX_SUPPORT_OMNI_PVD
		if (mPhysXSampler == NULL)
		{
			return false;
		}
		// A recording must be stopped before another is started. Calling startSampling() while
		// already sampling is a usage error: it would not start a fresh recording, so report it and
		// bail rather than silently doing nothing useful. Pair startSampling() with stopSampling().
		if (mPhysXSampler->isSampling())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL,
				"PxOmniPvd::startSampling(): already sampling. Call stopSampling() before starting a new recording; this call did not start one.");
			return false;
		}
		// startSampling() takes a full-state snapshot of the current world onto the bound write
		// stream, then records ongoing changes. It can be called any number of times: bind (or
		// re-bind) the destination stream with OmniPvdWriter::setWriteStream() first, which resets
		// the writer for a self-contained recording, and the same stream may be re-bound to take a
		// fresh snapshot. snapshotAll() turns recording on, registers the schema, then walks the
		// whole world (shared resources, shapes, actors, articulations, aggregates, deformables,
		// particles, per-scene state), always recording an object before anything that refers to
		// it, so objects created before this call are recorded too.
		const bool ok = mPhysXSampler->snapshotAll();
		// Then let each registered callback add its module's objects (extension joints, vehicles,
		// custom geometry) to the same stream. Callbacks run after snapshotAll() so their objects
		// can refer to the core objects it already recorded.
		if (ok)
		{
			for (PxU32 i = 0; i < mEventCallbacks.size(); ++i)
			{
				if (mEventCallbacks[i])
					mEventCallbacks[i]->onStartSampling(*this);
			}
		}
		return ok;
#else
		return false;
#endif
	}

	bool NpOmniPvd::stopSampling()
	{
#if PX_SUPPORT_OMNI_PVD
		// Stop sampling only: does NOT flush or close the stream; the caller owns the stream.
		// Stopping is optional. A later startSampling() takes a fresh snapshot regardless; this
		// just suppresses further per-frame writes and object notifications until then.
		if (mPhysXSampler)
		{
			return mPhysXSampler->stopSampling();
		}
		return false;
#else
		return false;
#endif
	}

	bool NpOmniPvd::isSampling() const
	{
#if PX_SUPPORT_OMNI_PVD
		return mPhysXSampler ? mPhysXSampler->isSampling() : false;
#else
		return false;
#endif
	}

	void NpOmniPvd::addEventCallback(PxOmniPvdEventCallback& callback)
	{
#if PX_SUPPORT_OMNI_PVD
		// Dedup: registering the same callback twice is a no-op.
		for (PxU32 i = 0; i < mEventCallbacks.size(); ++i)
		{
			if (mEventCallbacks[i] == &callback)
				return;
		}
		mEventCallbacks.pushBack(&callback);
#else
		PX_UNUSED(callback);
#endif
	}

	void NpOmniPvd::removeEventCallback(PxOmniPvdEventCallback& callback)
	{
#if PX_SUPPORT_OMNI_PVD
		// No-op if the callback is not registered.
		for (PxU32 i = 0; i < mEventCallbacks.size(); ++i)
		{
			if (mEventCallbacks[i] == &callback)
			{
				mEventCallbacks.remove(i);
				return;
			}
		}
#else
		PX_UNUSED(callback);
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


