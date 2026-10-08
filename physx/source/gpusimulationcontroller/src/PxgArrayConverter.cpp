// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxgArrayConverter.h"

#include "foundation/PxUserAllocated.h"

#include "PxPhysXGpu.h"
#include "PxgKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "PxgInterpolation.h"

using namespace physx;

#if ENABLE_KERNEL_LAUNCH_ERROR_CHECK
	#define checkCudaError() { cudaError_t err = cudaDeviceSynchronize(); if (err != 0) printf("Cuda error file: %s, line: %i, error: %i\n", PX_FL, err); }
#else
	#define checkCudaError() { }
#endif

#define THREADS_PER_BLOCK 256

PxgArrayConverter::PxgArrayConverter(PxgKernelLauncher& kernelLauncher)
{
	mKernelLauncher = kernelLauncher;
}

void PxgArrayConverter::interleaveGpuBuffers(const PxVec4* vertices, const PxVec4* normals, PxU32 length, PxVec3* interleavedResultBuffer, CUstream stream)
{
	const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
	const PxU32 numBlocks = (length + numThreadsPerBlock - 1) / numThreadsPerBlock;
	mKernelLauncher.launchKernel(PxgKernelIds::util_InterleaveBuffers, numBlocks, numThreadsPerBlock, 0, stream,
		vertices, normals, length, interleavedResultBuffer);
	checkCudaError();
}
