// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_ESSENTIAL_COMMON_H
#define PXG_ESSENTIAL_COMMON_H

#include "cudamanager/PxCudaTypes.h"
#include "PxgAllocatorDesc.h"

namespace physx
{
	class PxCudaContextManager;
	class PxCudaContext;

	class PxgCudaKernelWranglerManager;
	class PxgSimulationController;
	class PxgGpuContext;

	class PxgEssentialCore
	{
	public:
		PxgEssentialCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
			PxgAllocatorDesc& allocDesc, PxgSimulationController* simController, PxgGpuContext* context);

		PxgCudaKernelWranglerManager*	mGpuKernelWranglerManager;
		PxCudaContextManager*			mCudaContextManager;
		PxCudaContext*					mCudaContext;

		PxgSimulationController*		mSimController;
		PxgGpuContext*					mGpuContext;

		PxgAllocatorDesc				mAllocDesc;

		CUstream						mStream;
	};
}

#endif