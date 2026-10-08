// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxgKernelWrangler.h"
#include "foundation/PxAllocator.h"
#include "cudamanager/PxCudaContext.h"

using namespace physx;

static const char* kernelNames[]
{
#define KERNEL_DEF(id, name) name,
#include "PxgKernelNames.h"
#undef KERNEL_DEF
};

PxgCudaKernelWranglerManager::PxgCudaKernelWranglerManager(PxCudaContextManager& cudaContextManager, PxErrorCallback& errorCallback) :
	KernelWrangler(cudaContextManager, errorCallback, kernelNames, sizeof(kernelNames) / sizeof(char*))
{
}

PxgCudaKernelWranglerManager::~PxgCudaKernelWranglerManager()
{
}
