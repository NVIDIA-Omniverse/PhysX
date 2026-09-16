// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "NpOmniPvd.h"

#if PX_SUPPORT_OMNI_PVD
#include "OmniPvdPxSampler.h"
#include "OmniPvdLibraryFunctions.h"
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
		mWriter			(NULL),
		mPhysXSampler	(NULL)
	{
	}

	NpOmniPvd::~NpOmniPvd()
	{
#if PX_SUPPORT_OMNI_PVD
		if (mWriter)
		{
			destroyOmniPvdWriter(*mWriter);
			mWriter = NULL;
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

	OmniPvdWriter* NpOmniPvd::getWriter()
	{
		return blockingWriterLoad();
	}

	OmniPvdWriter* NpOmniPvd::blockingWriterLoad()
	{
#if PX_SUPPORT_OMNI_PVD
		PxMutex::ScopedLock lock(mWriterLoadMutex);
		if (!mWriter)
			mWriter = createOmniPvdWriter();
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
		// re-bind) the destination stream with OmniPvdWriter::setWriteStream() first. Rebinding resets
		// the writer session state and emits a new versioned segment without changing the transport
		// position. snapshotAll() turns recording on, registers the schema, then walks the
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
		// Stopping is required before another recording, but optional for final teardown: keeping
		// sampling active through PxPhysics::release() captures final object-remove notifications.
		// PxPhysics teardown destroys the sampler and clears mPhysXSampler, so isSampling() then
		// returns false. A later startSampling() takes a fresh snapshot rather than resuming.
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
		physx::NpOmniPvd::mRefCount = 1; // Sets the reference counter to exactly 1
		return physx::NpOmniPvd::mInstance;
	}
	return NULL;
#else
	return NULL;
#endif
}


