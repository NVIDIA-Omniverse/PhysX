// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_OMNI_PVD_H
#define NP_OMNI_PVD_H

#include "omnipvd/PxOmniPvd.h"
#include "foundation/PxMutex.h"
#include "foundation/PxArray.h"
#include "foundation/PxHashSet.h"
#include "NpOmniPvdMetaData.h"
#include "OmniPvdCaptureRequest.h"

class OmniPvdReader;
class OmniPvdWriter;
class OmniPvdPxSampler;

namespace physx
{

class PxPhysics;

class NpOmniPvd : public PxOmniPvd
{
public:
	NpOmniPvd();
	~NpOmniPvd();
	static void destroyInstance();
	static void incRefCount();
	static void decRefCount();
	void release() PX_OVERRIDE;

	OmniPvdWriter* getWriter() PX_OVERRIDE;
	
	OmniPvdWriter* blockingWriterLoad();

	OmniPvdWriter* acquireExclusiveWriterAccess() PX_OVERRIDE;
	void releaseExclusiveWriterAccess() PX_OVERRIDE;

	bool startSampling(const PxOmniPvdFilterDesc* filter) PX_OVERRIDE;
	bool startCaptureDiscovery(const PxOmniPvdFilterDesc* scope) PX_OVERRIDE;
	bool isCaptureDiscoveryActive() const PX_OVERRIDE;
	bool isCaptureDiscoveryScene(const PxScene& scene) const;
	bool isCaptureDiscoveryObject(const PxBase& object) const;
	bool restrictCaptureDiscovery(PxOmniPvdFilterDesc& desc, PxArray<const PxScene*>& scenes, PxArray<const PxBase*>& objects) const;
	bool isObjectRecorded(const PxBase& object) const PX_OVERRIDE;
	bool validateFilterDesc(const PxOmniPvdFilterDesc& filter) const PX_OVERRIDE;
	bool processCaptureRequests(PxOmniPvdCaptureFilter* filter = NULL) PX_OVERRIDE;
	bool hasPendingCaptureRequest() PX_OVERRIDE;
	void notifyObjectCapture(const PxBase& object);
	bool stopSampling() PX_OVERRIDE;
	bool isSampling() const PX_OVERRIDE;
	// Callback registry (see PxOmniPvdEventCallback): startSampling() invokes each registered
	// callback's onStartSampling(*this, filter) after the core objects are recorded.
	void addEventCallback(PxOmniPvdEventCallback& callback) PX_OVERRIDE;
	void removeEventCallback(PxOmniPvdEventCallback& callback) PX_OVERRIDE;

	OmniPvdWriter* mWriter;
	OmniPvdPxSampler* mPhysXSampler;
	NpOmniPvdMetaData mMetaData;
	bool mCaptureDiscovery;
	bool mDiscoveryAllObjects;
	PxU64 mDiscoveryGeneration;
	PxArray<PxU8> mDiscoveryRows;
	PxU64 mDiscoveryStream; // address and session of the stream that received the catalog rows
	PxU64 mDiscoveryStreamSession;
	PxU64 mRecordingStream;
	PxU64 mRecordingStreamSession;
	PxHashSet<PxU64> mDiscoveryScenes;
	PxHashSet<PxU64> mDiscoveryObjects;
	PxU64 mCaptureEpoch;
	PxU64 mCaptureRequestSession;
	PxU64 mLastCaptureRequestId;
	OmniPvdCaptureRequestResult::Enum mPendingCaptureResult;
	OmniPvdCaptureRequest mPendingCaptureRequest;
	PxArray<OmniPvdObjectHandle> mPendingCaptureHandles;
	PxArray<PxOmniPvdEventCallback*> mEventCallbacks;
	static PxU32 mRefCount;
	static NpOmniPvd* mInstance;
	PxMutex mMutex;
	PxMutex mWriterLoadMutex;
};

}

#endif
