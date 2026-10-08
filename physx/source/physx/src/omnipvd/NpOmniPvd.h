// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_OMNI_PVD_H
#define NP_OMNI_PVD_H

#include "omnipvd/PxOmniPvd.h"
#include "foundation/PxMutex.h"
#include "foundation/PxArray.h"
#include "NpOmniPvdMetaData.h"

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

	bool startSampling() PX_OVERRIDE;
	bool stopSampling() PX_OVERRIDE;
	bool isSampling() const PX_OVERRIDE;
	// Callback registry (see PxOmniPvdEventCallback): startSampling() invokes each registered
	// callback's onStartSampling(*this) after the core objects are recorded.
	void addEventCallback(PxOmniPvdEventCallback& callback) PX_OVERRIDE;
	void removeEventCallback(PxOmniPvdEventCallback& callback) PX_OVERRIDE;

	OmniPvdWriter* mWriter;
	OmniPvdPxSampler* mPhysXSampler;
	NpOmniPvdMetaData mMetaData;
	PxArray<PxOmniPvdEventCallback*> mEventCallbacks;
	static PxU32 mRefCount;
	static NpOmniPvd* mInstance;
	PxMutex mMutex;
	PxMutex mWriterLoadMutex;
};

}

#endif
