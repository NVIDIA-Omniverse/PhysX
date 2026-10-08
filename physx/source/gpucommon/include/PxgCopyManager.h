// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_COPY_MANAGER_H
#define PXG_COPY_MANAGER_H

#include "CmPinnableArray.h"
#include "cudamanager/PxCudaTypes.h"
#include "PxgCopyDesc.h"

namespace physx
{
	class PxCudaContextManager;
	class PxCudaContext;
	class KernelWrangler;

class PxgCopyManager
{
	PX_NOCOPY(PxgCopyManager)
public:

	PxgCopyManager(Cm::VirtualAllocatorCallback& hostMappedAlloc);
							
	~PxgCopyManager(){}

	void waitAndReset(PxCudaContext* cudaContext);
	void pushDeferredHtoD(const PxgCopyDesc& desc);
	void dispatchCopy(CUstream stream, PxCudaContextManager* cudaContextManager, KernelWrangler* kernelWrangler);
	void createFinishedEvent(PxCudaContext* cudaContext);
	void destroyFinishedEvent(PxCudaContext* cudaContext);

	bool mAllocFailed;

protected:

	void resetUnsafe() { mNumDescriptors = 0; }
	bool hasFinishedCopying(PxCudaContext* cudaContext) const;
	
	Cm::PinnableArray<PxU8>			mDescriptorsQueueMapped; // needs to be device mapped memory
	PxU32							mNumDescriptors;
	CUevent							mFinishedEvent;
	bool							mEventRecorded;
};


}

#endif
