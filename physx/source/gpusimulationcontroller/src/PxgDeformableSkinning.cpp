// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxgDeformableSkinning.h"

#include "foundation/PxUserAllocated.h"

#include "PxPhysXGpu.h"
#include "PxgKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "GuAABBTree.h"
#include "foundation/PxMathUtils.h"


namespace physx
{
	PxgDeformableSkinning::PxgDeformableSkinning(PxgKernelLauncher& kernelLauncher)
	{
		mKernelLauncher = kernelLauncher;
	}

	void PxgDeformableSkinning::computeNormalVectors(
		PxTrimeshSkinningGpuData* skinningDataArrayD, PxU32 arrayLength,
		CUstream stream, PxU32 numGpuThreads)
	{
		physx::PxScopedCudaLock _lock(*mKernelLauncher.getCudaContextManager());

		const PxU32 numThreadsPerBlock = 256;
		const PxU32 numBlocks = (numGpuThreads + numThreadsPerBlock - 1) / numThreadsPerBlock;

		const PxU32 numThreadsPerBlockSmallKernels = 1024;
		const PxU32 numBlocksSmallKernels = (numGpuThreads + numThreadsPerBlockSmallKernels - 1) / numThreadsPerBlockSmallKernels;

		mKernelLauncher.launchKernelXYZ(PxgKernelIds::util_ZeroNormals, numBlocksSmallKernels, arrayLength, 1, numThreadsPerBlockSmallKernels, 1, 1, 0, stream,
			skinningDataArrayD);

		mKernelLauncher.launchKernelXYZ(PxgKernelIds::util_ComputeNormals, numBlocks, arrayLength, 1, numThreadsPerBlock, 1, 1, 0, stream,
			skinningDataArrayD);

		mKernelLauncher.launchKernelXYZ(PxgKernelIds::util_NormalizeNormals, numBlocksSmallKernels, arrayLength, 1, numThreadsPerBlockSmallKernels, 1, 1, 0, stream,
			skinningDataArrayD);
	}

	void PxgDeformableSkinning::evaluateVerticesEmbeddedIntoSurface(
		PxTrimeshSkinningGpuData* skinningDataArrayD, PxU32 arrayLength,
		CUstream stream, PxU32 numGpuThreads)
	{
		physx::PxScopedCudaLock _lock(*mKernelLauncher.getCudaContextManager());
		const PxU32 numThreadsPerBlock = 256;
		const PxU32 numBlocks = (numGpuThreads + numThreadsPerBlock - 1) / numThreadsPerBlock;
		mKernelLauncher.launchKernelXYZ(PxgKernelIds::util_InterpolateSkinnedClothVertices, numBlocks, arrayLength, 1, numThreadsPerBlock, 1, 1, 0, stream,
			skinningDataArrayD);
	}

	void PxgDeformableSkinning::evaluateVerticesEmbeddedIntoVolume(
		PxTetmeshSkinningGpuData* skinningDataArrayD, PxU32 arrayLength,
		CUstream stream, PxU32 numGpuThreads)
	{
		physx::PxScopedCudaLock _lock(*mKernelLauncher.getCudaContextManager());
		const PxU32 numThreadsPerBlock = 256;
		const PxU32 numBlocks = (numGpuThreads + numThreadsPerBlock - 1) / numThreadsPerBlock;
		mKernelLauncher.launchKernelXYZ(PxgKernelIds::util_InterpolateSkinnedSoftBodyVertices, numBlocks, arrayLength, 1, numThreadsPerBlock, 1, 1, 0, stream,
			skinningDataArrayD);
	}
}
