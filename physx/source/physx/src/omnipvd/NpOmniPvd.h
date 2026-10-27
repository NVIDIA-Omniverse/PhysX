// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_OMNI_PVD_H
#define NP_OMNI_PVD_H

#include "omnipvd/PxOmniPvd.h"
#include "foundation/PxMutex.h"
#include "NpOmniPvdMetaData.h"

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
	NpOmniPvdMetaData mMetaData;
	static PxU32 mRefCount;
	static NpOmniPvd* mInstance;
	PxMutex mMutex;
	PxMutex mWriterLoadMutex;
};

}

#endif
