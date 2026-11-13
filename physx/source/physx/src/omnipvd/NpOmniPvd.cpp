// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "NpOmniPvd.h"

#if PX_SUPPORT_OMNI_PVD
#include "OmniPvdPxSampler.h"
#include "OmniPvdLibraryFunctions.h"
#include "OmniPvdWriter.h"
#include "OmniPvdWriteStream.h"
#include "NpOmniPvdSetData.h"
#include "NpPhysics.h"
#include "NpScene.h"
#include "NpActor.h"
#include "extensions/PxJoint.h"
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
		mPhysXSampler	(NULL),
		mCaptureDiscovery (false),
		mDiscoveryAllObjects (true),
		mDiscoveryGeneration (0),
		mDiscoveryStream (0),
		mDiscoveryStreamSession (0),
		mRecordingStream (0),
		mRecordingStreamSession (0),
		mCaptureEpoch    (0),
		mCaptureRequestSession (0),
		mLastCaptureRequestId (0),
		mPendingCaptureResult (OmniPvdCaptureRequestResult::eNONE)
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

	bool NpOmniPvd::startSampling(const PxOmniPvdFilterDesc* filter)
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
			mPhysXSampler->reportAlreadySamplingError();
			return false;
		}
		NpPhysics& physics = NpPhysics::getInstance();
		for (PxU32 i = 0; i < physics.getNbScenes(); ++i)
		{
			PxScene* scene = NULL;
			physics.getScenes(&scene, 1, i);
			if (static_cast<NpScene*>(scene)->isAPIWriteForbidden())
			{
				PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL,
					"PxOmniPvd::startSampling(): every scene must be idle before recording starts.");
				return false;
			}
		}
		PxOmniPvdFilterDesc desc = filter ? *filter : PxOmniPvdFilterDesc();
		PxArray<const PxScene*> discoveryScenes; PxArray<const PxBase*> discoveryObjects;
		if ((mCaptureDiscovery && (mPhysXSampler->validateCaptureDesc(desc) ||
			!restrictCaptureDiscovery(desc, discoveryScenes, discoveryObjects))) || !mPhysXSampler->setCaptureDesc(desc))
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL,
				"PxOmniPvd::startSampling(): invalid filter, unsupported object type, object not owned by this PxPhysics, or filter outside the capture discovery scope.");
			return false;
		}
		// Callbacks receive owned lists even if an earlier callback changes the caller's arrays.
		PxArray<const PxScene*> scenes;
		PxArray<const PxBase*> objects;
		for (PxU32 i = 0; i < desc.nbScenes; ++i) scenes.pushBack(desc.scenes[i]);
		for (PxU32 i = 0; i < desc.nbObjects; ++i) objects.pushBack(desc.objects[i]);
		desc.scenes = scenes.begin();
		desc.objects = objects.begin();
		mMetaData.recordSecondaryData = desc.recordSecondaryData;
		mMetaData.captureAllObjects = !desc.nbScenes && !desc.nbObjects;
		{
			// Discard requests retained by the previous recording before enabling sampling.
			ScopedExclusiveWriter lock(this);
			if (mCaptureDiscovery) mPhysXSampler->resetCaptureObjects(*mWriter);
			mPendingCaptureResult = OmniPvdCaptureRequestResult::eNONE;
			mPendingCaptureRequest = OmniPvdCaptureRequest();
			mPendingCaptureHandles.clear();
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
		mRecordingStream = mWriter ? reinterpret_cast<PxU64>(mWriter->getWriteStream()) : 0;
		mRecordingStreamSession = mRecordingStream ? mWriter->getWriteStream()->getCaptureSessionId() : 0;
		// A snapshot can open a new control connection before a later payload write fails.
		const PxU64 session = mWriter && mWriter->getWriteStream() ? mWriter->getWriteStream()->getCaptureSessionId() : 0;
		if (mCaptureRequestSession != session)
		{
			mCaptureRequestSession = session;
			mLastCaptureRequestId = 0;
		}
		// A new PxPhysics creates a new sampler even if the stream connection remains open.
		mPhysXSampler->clearRetiredCaptureHandles();
		// Then let each registered callback add its module's objects (extension joints, vehicles,
		// custom geometry) to the same stream. Callbacks run after snapshotAll() so their objects
		// can refer to the core objects it already recorded.
		if (ok)
		{
			for (PxU32 i = 0; i < mEventCallbacks.size(); ++i)
			{
				if (mEventCallbacks[i])
					mEventCallbacks[i]->onStartSampling(*this, (filter || mCaptureDiscovery) ? &desc : NULL);
			}
			if (isSampling())
			{
				PxConstraint* constraints[256];
				for (PxU32 offset = 0; offset < physics.getNbConstraints(); offset += 256)
				{
					const PxU32 count = physics.getConstraints(constraints, 256, offset);
					for (PxU32 i = 0; i < count; ++i)
						mPhysXSampler->updateConstraintOmniPvdProperties(*constraints[i]);
				}
			}
			if (!isSampling() || (mWriter->getStatus() & OmniPvdWriterStatusFlag::eSTREAM_WRITE_FAILURE))
			{
				stopSampling();
				return false;
			}
		}
		if (!ok && mCaptureDiscovery) stopSampling();
		return ok;
#else
		PX_UNUSED(filter);
		return false;
#endif
	}

	bool NpOmniPvd::startCaptureDiscovery(const PxOmniPvdFilterDesc* scope)
	{
#if PX_SUPPORT_OMNI_PVD
		const PxOmniPvdFilterDesc desc = scope ? *scope : PxOmniPvdFilterDesc();
		if (isSampling() || isCaptureDiscoveryActive() || !validateFilterDesc(desc) ||
			!mWriter || !mWriter->getWriteStream() || !mWriter->getWriteStream()->openStream()) return false;
		const PxU64 stream = reinterpret_cast<PxU64>(mWriter->getWriteStream());
		const PxU64 session = mWriter->getWriteStream()->getCaptureSessionId();
		if (stream == mRecordingStream && session == mRecordingStreamSession)
		{
			// Module references must end before their core objects on this receiver are retired.
			for (PxU32 i = 0; i < mEventCallbacks.size(); ++i) mEventCallbacks[i]->onCaptureReset(*this, desc);
			ScopedExclusiveWriter lock(this);
			mPhysXSampler->resetCaptureObjects(*mWriter);
		}
		mRecordingStream = 0;
		mDiscoveryAllObjects = !desc.nbScenes && !desc.nbObjects;
		mDiscoveryScenes.clear(); mDiscoveryObjects.clear();
		for (PxU32 i = 0; i < desc.nbScenes; ++i) mDiscoveryScenes.insert(reinterpret_cast<PxU64>(desc.scenes[i]));
		for (PxU32 i = 0; i < desc.nbObjects; ++i) mDiscoveryObjects.insert(reinterpret_cast<PxU64>(desc.objects[i]));
		mCaptureDiscovery = true;
		mPendingCaptureResult = OmniPvdCaptureRequestResult::eNONE;
		mPendingCaptureHandles.clear();
		// Preserve rows and generations while the receiver still holds this connection's catalog.
		if (stream != mDiscoveryStream || session != mDiscoveryStreamSession)
		{
			mDiscoveryStream = stream; mDiscoveryStreamSession = session;
			mDiscoveryGeneration = 0; mDiscoveryRows.clear();
		}
		if (mCaptureRequestSession != session) { mCaptureRequestSession = session; mLastCaptureRequestId = 0; }
		mPhysXSampler->clearRetiredCaptureHandles();
		if (mPhysXSampler->streamCaptureCatalog(true)) return true;
		stopSampling();
#else
		PX_UNUSED(scope);
#endif
		return false;
	}

	bool NpOmniPvd::isCaptureDiscoveryActive() const
	{
		return mPhysXSampler && mCaptureDiscovery;
	}

#if PX_SUPPORT_OMNI_PVD
	bool NpOmniPvd::isCaptureDiscoveryScene(const PxScene& scene) const
	{
		const PxU64 handle = reinterpret_cast<PxU64>(&scene);
		return !mPhysXSampler->isCaptureHandleRetired(handle) &&
			(mDiscoveryAllObjects || mDiscoveryScenes.contains(handle));
	}

	bool NpOmniPvd::isCaptureDiscoveryObject(const PxBase& object) const
	{
		const PxU64 handle = reinterpret_cast<PxU64>(&object);
		if (mPhysXSampler->isCaptureHandleRetired(handle)) return false;
		if (mDiscoveryAllObjects || mDiscoveryObjects.contains(handle)) return true;
		const PxScene* scene; const PxBase* owner; const char* name;
		OmniPvdPxSampler::getCaptureObjectIdentity(object, scene, owner, name);
		return (scene && isCaptureDiscoveryScene(*scene)) || (owner &&
			!mPhysXSampler->isCaptureHandleRetired(reinterpret_cast<PxU64>(owner)) &&
			mDiscoveryObjects.contains(reinterpret_cast<PxU64>(owner)));
	}

	bool NpOmniPvd::restrictCaptureDiscovery(PxOmniPvdFilterDesc& desc,
		PxArray<const PxScene*>& scenes, PxArray<const PxBase*>& objects) const
	{
		if (!mCaptureDiscovery) return true;
		for (PxU32 i = 0; i < desc.nbScenes; ++i) if (!isCaptureDiscoveryScene(*desc.scenes[i])) return false;
		for (PxU32 i = 0; i < desc.nbObjects; ++i) if (!isCaptureDiscoveryObject(*desc.objects[i])) return false;
		if (desc.nbScenes || desc.nbObjects || mDiscoveryAllObjects) return true;
		// Empty requests mean all of the original scope, never all of PxPhysics after a root dies.
		scenes.clear(); objects.clear();
		NpPhysics& physics = NpPhysics::getInstance();
		for (PxU32 i = 0; i < physics.getNbScenes(); ++i)
		{
			PxScene* scene = NULL; physics.getScenes(&scene, 1, i);
			if (isCaptureDiscoveryScene(*scene)) scenes.pushBack(scene);
		}
		PxHashMap<PxU64, const PxBase*> catalog;
		OmniPvdPxSampler::buildCaptureObjectCatalog(catalog);
		for (PxHashMap<PxU64, const PxBase*>::Iterator it = catalog.getIterator(); !it.done(); ++it)
			if (mDiscoveryObjects.contains(it->first) && !mPhysXSampler->isCaptureHandleRetired(it->first)) objects.pushBack(it->second);
		desc.scenes = scenes.begin(); desc.nbScenes = scenes.size();
		desc.objects = objects.begin(); desc.nbObjects = objects.size();
		return desc.nbScenes || desc.nbObjects;
	}
#endif

	bool NpOmniPvd::isObjectRecorded(const PxBase& object) const
	{
#if PX_SUPPORT_OMNI_PVD
		if (!mPhysXSampler || !mPhysXSampler->isSampling()) return false;
		if (const PxJoint* joint = object.is<PxJoint>())
			return joint->getConstraint() && mPhysXSampler->isObjectRecorded(*joint->getConstraint());
		return mPhysXSampler->isObjectRecorded(object);
#else
		PX_UNUSED(object);
		return false;
#endif
	}

	bool NpOmniPvd::validateFilterDesc(const PxOmniPvdFilterDesc& desc) const
	{
#if PX_SUPPORT_OMNI_PVD
		if (!mPhysXSampler) return false;
		NpPhysics& physics = NpPhysics::getInstance();
		for (PxU32 i = 0; i < physics.getNbScenes(); ++i)
		{
			PxScene* scene = NULL;
			physics.getScenes(&scene, 1, i);
			if (static_cast<NpScene*>(scene)->isAPIWriteForbidden()) return false;
		}
		if (mPhysXSampler->validateCaptureDesc(desc)) return false;
		// Apply the same discovery scope as startSampling().
		PxOmniPvdFilterDesc scoped = desc;
		PxArray<const PxScene*> discoveryScenes; PxArray<const PxBase*> discoveryObjects;
		return restrictCaptureDiscovery(scoped, discoveryScenes, discoveryObjects);
#else
		PX_UNUSED(desc);
		return false;
#endif
	}

	void NpOmniPvd::notifyObjectCapture(const PxBase& object)
	{
#if PX_SUPPORT_OMNI_PVD
		for (PxU32 i = 0; i < mEventCallbacks.size(); ++i)
			if (mEventCallbacks[i]) mEventCallbacks[i]->onObjectCapture(*this, object);
		if (const PxConstraint* constraint = object.is<PxConstraint>())
			mPhysXSampler->updateConstraintOmniPvdProperties(*constraint);
#else
		PX_UNUSED(object);
#endif
	}

#if PX_SUPPORT_OMNI_PVD
	template<class Object>
	static bool resolveCaptureHandles(const OmniPvdObjectHandle* handles, PxU32 count,
		const PxHashMap<PxU64, const Object*>& catalog, PxArray<const Object*>& objects)
	{
		objects.reserve(count);
		for (PxU32 i = 0; i < count; ++i)
		{
			if (OmniPvdPxSampler::getInstance()->isCaptureHandleRetired(handles[i])) return false;
			const typename PxHashMap<PxU64, const Object*>::Entry* entry = catalog.find(handles[i]);
			if (!entry) return false;
			objects.pushBack(entry->second);
		}
		return true;
	}
#endif

	bool NpOmniPvd::hasPendingCaptureRequest()
	{
#if PX_SUPPORT_OMNI_PVD
		ScopedExclusiveWriter lock(this);
		if ((!isSampling() && !isCaptureDiscoveryActive()) || !mWriter || !mWriter->getWriteStream()) return false;
		OmniPvdWriteStream& stream = *mWriter->getWriteStream();
		if (mCaptureRequestSession != stream.getCaptureSessionId())
		{
			mCaptureRequestSession = stream.getCaptureSessionId();
			mLastCaptureRequestId = 0;
			mPendingCaptureResult = OmniPvdCaptureRequestResult::eNONE;
			mPendingCaptureHandles.clear();
		}
		mPhysXSampler->clearRetiredCaptureHandles();
		if (mWriter->getStatus() & OmniPvdWriterStatusFlag::eSTREAM_WRITE_FAILURE)
			mPendingCaptureResult = OmniPvdCaptureRequestResult::eERROR;
		if (mPendingCaptureResult != OmniPvdCaptureRequestResult::eNONE) return true;
		mPendingCaptureResult = stream.pollCaptureRequest(mPendingCaptureRequest);
		if (mPendingCaptureResult == OmniPvdCaptureRequestResult::eREQUEST)
		{
			OmniPvdCaptureRequest& request = mPendingCaptureRequest;
			const bool valid = PxU64(request.nbScenes) + request.nbObjects <= 65536 &&
				(!request.nbScenes || request.scenes) && (!request.nbObjects || request.objects);
			mPendingCaptureHandles.clear();
			if (valid)
			{
				mPendingCaptureHandles.reserve(request.nbScenes + request.nbObjects);
				const OmniPvdObjectHandle* lists[] = { request.scenes, request.objects };
				const PxU32 counts[] = { request.nbScenes, request.nbObjects };
				for (PxU32 list = 0; list < 2; ++list)
					for (PxU32 i = 0; i < counts[list]; ++i) mPendingCaptureHandles.pushBack(lists[list][i]);
			}
			request.scenes = valid && request.nbScenes ? mPendingCaptureHandles.begin() : NULL;
			request.objects = valid && request.nbObjects ? mPendingCaptureHandles.begin() + request.nbScenes : NULL;
		}
		return mPendingCaptureResult != OmniPvdCaptureRequestResult::eNONE;
#else
		return false;
#endif
	}

	bool NpOmniPvd::processCaptureRequests(PxOmniPvdCaptureFilter* filter)
	{
#if PX_SUPPORT_OMNI_PVD
		if ((!isSampling() && !isCaptureDiscoveryActive()) || !mWriter || !mWriter->getWriteStream()) return false;
		if (!hasPendingCaptureRequest()) return isSampling() || isCaptureDiscoveryActive();
		OmniPvdCaptureRequest request;
		OmniPvdCaptureRequestResult::Enum result;
		PxArray<OmniPvdObjectHandle> sceneHandles, objectHandles;
		{
			ScopedExclusiveWriter lock(this);
			result = mPendingCaptureResult;
			request = mPendingCaptureRequest;
			mPendingCaptureResult = OmniPvdCaptureRequestResult::eNONE;
			if (result == OmniPvdCaptureRequestResult::eREQUEST &&
				PxU64(request.nbScenes) + request.nbObjects <= 65536 &&
				(!request.nbScenes || request.scenes) && (!request.nbObjects || request.objects))
			{
				for (PxU32 i = 0; i < request.nbScenes; ++i) sceneHandles.pushBack(request.scenes[i]);
				for (PxU32 i = 0; i < request.nbObjects; ++i) objectHandles.pushBack(request.objects[i]);
				request.scenes = sceneHandles.begin();
				request.objects = objectHandles.begin();
			}
		}
		if (result == OmniPvdCaptureRequestResult::eNONE) return true;
		if (result == OmniPvdCaptureRequestResult::eERROR) { stopSampling(); return false; }
		PxU32 status = 0;
		if (!request.requestId || request.requestId <= mLastCaptureRequestId ||
			PxU64(request.nbScenes) + request.nbObjects > 65536 ||
			(request.nbScenes && !request.scenes) || (request.nbObjects && !request.objects) ||
			(request.catalogOnly && (request.nbScenes || request.nbObjects))) status = 2;
		if (request.requestId > mLastCaptureRequestId) mLastCaptureRequestId = request.requestId;
		PxHashMap<PxU64, const PxScene*> sceneCatalog;
		NpPhysics& physics = NpPhysics::getInstance();
		for (PxU32 i = 0; !status && i < physics.getNbScenes(); ++i)
		{
			PxScene* scene = NULL;
			physics.getScenes(&scene, 1, i);
			sceneCatalog.insert(reinterpret_cast<PxU64>(scene), scene);
			if (!status && static_cast<NpScene*>(scene)->isAPIWriteForbidden()) status = 3;
		}
		PxArray<const PxScene*> scenes;
		PxArray<const PxBase*> objects;
		PxOmniPvdFilterDesc desc;
		if (!status && request.catalogOnly)
		{
			if (!isCaptureDiscoveryActive()) status = 4;
			else if (!mPhysXSampler->streamCaptureCatalog(false)) { stopSampling(); return false; }
			else status = 1;
		}
		PxArray<const PxScene*> discoveryScenes; PxArray<const PxBase*> discoveryObjects;
		if (!status)
		{
			PxHashMap<PxU64, const PxBase*> objectCatalog;
			if (request.nbObjects) OmniPvdPxSampler::buildCaptureObjectCatalog(objectCatalog);
			if (!resolveCaptureHandles(request.scenes, request.nbScenes, sceneCatalog, scenes) ||
				!resolveCaptureHandles(request.objects, request.nbObjects, objectCatalog, objects)) status = 2;
			desc.recordSecondaryData = request.recordSecondaryData;
			desc.scenes = scenes.begin(); desc.nbScenes = scenes.size();
			desc.objects = objects.begin(); desc.nbObjects = objects.size();
			if (!status && !desc.isValid()) status = 2;
			if (!status && filter && !filter->filterCaptureRequest(desc)) status = 4;
			if (!status) status = mPhysXSampler->validateCaptureDesc(desc);
			if (!status && !restrictCaptureDiscovery(desc, discoveryScenes, discoveryObjects)) status = 4;
		}
		if (!status)
		{
			if (isSampling())
				for (PxU32 i = 0; i < mEventCallbacks.size(); ++i) mEventCallbacks[i]->onCaptureReset(*this, desc);
			{
				ScopedExclusiveWriter lock(this);
				mPhysXSampler->resetCaptureObjects(*mWriter);
			}
			mPhysXSampler->stopSampling();
			++mCaptureEpoch;
			if (!startSampling(&desc)) { stopSampling(); return false; }
			status = 1;
		}
		{
			ScopedExclusiveWriter lock(this);
			const OmniPvdPxCoreRegistrationData* registrationData = NpOmniPvdGetPxCoreRegistrationData();
			const OmniPvdObjectHandle metadata = reinterpret_cast<OmniPvdObjectHandle>(&mMetaData);
			registrationData->pvdPxOmniPvdMetaData.set_captureRequestId_(*mWriter, OMNI_PVD_CONTEXT_HANDLE, metadata, request.requestId);
			registrationData->pvdPxOmniPvdMetaData.set_captureRequestStatus_(*mWriter, OMNI_PVD_CONTEXT_HANDLE, metadata, status);
			registrationData->pvdPxOmniPvdMetaData.set_captureEpoch_(*mWriter, OMNI_PVD_CONTEXT_HANDLE, metadata, mCaptureEpoch);
		}
		if (!(mWriter->getStatus() & OmniPvdWriterStatusFlag::eSTREAM_WRITE_FAILURE)) return true;
		stopSampling();
		return false;
#else
		PX_UNUSED(filter);
		return false;
#endif
	}

	bool NpOmniPvd::stopSampling()
	{
#if PX_SUPPORT_OMNI_PVD
		mCaptureDiscovery = false;
		mDiscoveryScenes.clear(); mDiscoveryObjects.clear();
		// Stop sampling only: does NOT flush or close the stream; the caller owns the stream.
		// Stopping is required before another recording, but optional for final teardown: keeping
		// sampling active through PxPhysics::release() captures final object-remove notifications.
		// PxPhysics teardown destroys the sampler and clears mPhysXSampler, so isSampling() then
		// returns false. A later startSampling() takes a fresh snapshot rather than resuming.
		if (mPhysXSampler)
		{
			const bool stopped = mPhysXSampler->stopSampling();
			ScopedExclusiveWriter lock(this);
			mPendingCaptureResult = OmniPvdCaptureRequestResult::eNONE;
			mPendingCaptureHandles.clear();
			return stopped;
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


