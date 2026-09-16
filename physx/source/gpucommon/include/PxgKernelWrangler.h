// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#ifndef PXG_KERNEL_WRANGLER_H
#define PXG_KERNEL_WRANGLER_H

#include "foundation/PxPreprocessor.h"
#include "PxsKernelWrangler.h"
#include "foundation/PxArray.h"
#include "CudaKernelWrangler.h"

namespace physx
{
	class PxCudaContextManager;
	class PxErrorCallback;

	class PxgCudaKernelWranglerManager : public PxsKernelWranglerManager, public KernelWrangler
	{
	public:
		PxgCudaKernelWranglerManager(PxCudaContextManager& cudaContextManager, PxErrorCallback& errorCallback);
		~PxgCudaKernelWranglerManager();
	};
}

#endif