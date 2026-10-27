// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxgAnisotropy.h"



#include "foundation/PxUserAllocated.h"
#include "foundation/PxHashSet.h"

#include "PxPhysics.h"
#include "PxParticleSystem.h"
#include "PxParticleGpu.h"
#include "PxPhysXGpu.h"
#include "PxvGlobals.h"

#include "PxgParticleNeighborhoodProvider.h"
#include "PxgKernelIndices.h"
#include "PxgAlgorithms.h"
#include "PxgSparseGridStandalone.h"
#include "PxgAnisotropyData.h"
#include "PxgCudaMemoryAllocator.h"

namespace physx
{

#if ENABLE_KERNEL_LAUNCH_ERROR_CHECK
#define checkCudaError() { cudaError_t err = cudaDeviceSynchronize(); if (err != 0) printf("Cuda error file: %s, line: %i, error: %i\n", PX_FL, err); }
#else
#define checkCudaError() { }
#endif
	
	void updateAnisotropy(PxgKernelLauncher& launcher, PxGpuParticleSystem* particleSystems, const PxU32 id, PxAnisotropyData* anisotropyDataPerParticleSystem, PxU32 numParticles,
		CUstream stream, PxU32 numThreadsPerBlock = 256)
	{
		const PxU32 numBlocks = (numParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::calculateAnisotropyLaunch, numBlocks, numThreadsPerBlock, 0, stream,
			particleSystems, id, anisotropyDataPerParticleSystem);
		checkCudaError();
	}

	void anisotropyLaunch(PxgKernelLauncher& launcher, float4* deviceParticlePos, PxU32* sortedToOriginalParticleIndex, PxU32* sortedParticleToSubgrid, PxU32 maxNumSubgrids,
		PxU32* subgridNeighbors, PxU32* subgridEndIndices, int numParticles, PxU32* phases, PxU32 validPhaseMask,
		float4* q1, float4* q2, float4* q3, PxReal anisotropy, PxReal anisotropyMin, PxReal anisotropyMax, PxReal particleContactDistance, CUstream stream, PxU32 numThreadsPerBlock = 256)
	{
		const PxU32 numBlocks = (numParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::anisotropyKernel, numBlocks, numThreadsPerBlock, 0, stream,
			deviceParticlePos, sortedToOriginalParticleIndex, sortedParticleToSubgrid, maxNumSubgrids,
			subgridNeighbors, subgridEndIndices, numParticles, phases, validPhaseMask, q1, q2, q3, anisotropy, anisotropyMin, anisotropyMax, particleContactDistance);
		checkCudaError();
	}

	void PxgAnisotropyGenerator::releaseGPUAnisotropyBuffers()
	{
		if (!mAnisotropyDataHost.mAnisotropy_q1)
			return;

		PX_DEVICE_MEMORY_FREE(*mKernelLauncher.getCudaContextManager(), mAnisotropyDataHost.mAnisotropy_q1);
		PX_DEVICE_MEMORY_FREE(*mKernelLauncher.getCudaContextManager(), mAnisotropyDataHost.mAnisotropy_q2);
		PX_DEVICE_MEMORY_FREE(*mKernelLauncher.getCudaContextManager(), mAnisotropyDataHost.mAnisotropy_q3);
		mOwnsAnisotropyGPUBuffers = false;
	}

	void PxgAnisotropyGenerator::allocateGPUAnisotropyBuffers()
	{
		if (mAnisotropyDataHost.mAnisotropy_q1)
			return;

		mAnisotropyDataHost.mAnisotropy_q1 = PX_DEVICE_MEMORY_ALLOC(PxVec4, *mKernelLauncher.getCudaContextManager(), mNumParticles);
		mAnisotropyDataHost.mAnisotropy_q2 = PX_DEVICE_MEMORY_ALLOC(PxVec4, *mKernelLauncher.getCudaContextManager(), mNumParticles);
		mAnisotropyDataHost.mAnisotropy_q3 = PX_DEVICE_MEMORY_ALLOC(PxVec4, *mKernelLauncher.getCudaContextManager(), mNumParticles);
		mOwnsAnisotropyGPUBuffers = true;
	}

	PxgAnisotropyGenerator::PxgAnisotropyGenerator(PxgKernelLauncher& cudaContextManager, PxU32 maxNumParticles, PxReal anisotropyScale, PxReal minAnisotropy, PxReal maxAnisotropy)
		: mAnisotropy1(NULL), mAnisotropy2(NULL), mAnisotropy3(NULL), mEnabled(true)
	{
		mAnisotropyDataHost.mAnisotropy_q1 = NULL;
		mAnisotropyDataHost.mAnisotropy_q2 = NULL;
		mAnisotropyDataHost.mAnisotropy_q3 = NULL;
		mKernelLauncher = cudaContextManager;
		mNumParticles = maxNumParticles;
		mAnisotropyDataPerParticleSystemDevice = PX_DEVICE_MEMORY_ALLOC(PxAnisotropyData, *mKernelLauncher.getCudaContextManager(), 1);

		mAnisotropyDataHost.mAnisotropy = anisotropyScale;
		mAnisotropyDataHost.mAnisotropyMin = minAnisotropy;
		mAnisotropyDataHost.mAnisotropyMax = maxAnisotropy;
		mDirty = true;
		mOwnsAnisotropyGPUBuffers = false;
	}

	void PxgAnisotropyGenerator::release()
	{
		if (!mAnisotropyDataPerParticleSystemDevice)
			return;

		PX_DEVICE_MEMORY_FREE(*mKernelLauncher.getCudaContextManager(), mAnisotropyDataPerParticleSystemDevice);
		if (mOwnsAnisotropyGPUBuffers)
			releaseGPUAnisotropyBuffers();

		PX_DELETE_THIS;
	}

	void PxgAnisotropyGenerator::setResultBufferHost(PxVec4* anisotropy1, PxVec4* anisotropy2, PxVec4* anisotropy3)
	{
		mAnisotropy1 = anisotropy1;
		mAnisotropy2 = anisotropy2;
		mAnisotropy3 = anisotropy3;
		allocateGPUAnisotropyBuffers();
		mDirty = true;
	}

	void PxgAnisotropyGenerator::setResultBufferDevice(PxVec4* anisotropy1, PxVec4* anisotropy2, PxVec4* anisotropy3)
	{
		if (mOwnsAnisotropyGPUBuffers)
			releaseGPUAnisotropyBuffers();
		mAnisotropyDataHost.mAnisotropy_q1 = anisotropy1;
		mAnisotropyDataHost.mAnisotropy_q2 = anisotropy2;
		mAnisotropyDataHost.mAnisotropy_q3 = anisotropy3;
		mDirty = true;
		mAnisotropy1 = NULL;
		mAnisotropy2 = NULL;
		mAnisotropy3 = NULL;
	}

	void PxgAnisotropyGenerator::generateAnisotropy(PxGpuParticleSystem* gpuParticleSystem, PxU32 numParticles, CUstream stream)
	{
		if (mDirty)
		{
			mDirty = false;
			mKernelLauncher.getCudaContextManager()->getCudaContext()->memcpyHtoDAsync(CUdeviceptr(mAnisotropyDataPerParticleSystemDevice), &mAnisotropyDataHost, sizeof(PxAnisotropyData), stream);
		}

		updateAnisotropy(mKernelLauncher, gpuParticleSystem, 0, mAnisotropyDataPerParticleSystemDevice, numParticles, stream);

		if (mAnisotropy1)
		{
			mKernelLauncher.getCudaContextManager()->getCudaContext()->memcpyDtoHAsync(mAnisotropy1, CUdeviceptr(mAnisotropyDataHost.mAnisotropy_q1), numParticles * sizeof(PxVec4), stream);
			mKernelLauncher.getCudaContextManager()->getCudaContext()->memcpyDtoHAsync(mAnisotropy2, CUdeviceptr(mAnisotropyDataHost.mAnisotropy_q2), numParticles * sizeof(PxVec4), stream);
			mKernelLauncher.getCudaContextManager()->getCudaContext()->memcpyDtoHAsync(mAnisotropy3, CUdeviceptr(mAnisotropyDataHost.mAnisotropy_q3), numParticles * sizeof(PxVec4), stream);
		}
	}

	void PxgAnisotropyGenerator::generateAnisotropy(PxVec4* particlePositionsGpu, PxParticleNeighborhoodProvider& neighborhoodProvider, PxU32 numParticles, PxReal particleContactOffset, CUstream stream)
	{
		PxgParticleNeighborhoodProvider* n = static_cast<PxgParticleNeighborhoodProvider*>(&neighborhoodProvider);

		anisotropyLaunch(mKernelLauncher, reinterpret_cast<float4*>(particlePositionsGpu), n->mSparseGridBuilder.getSortedToOriginalParticleIndex(),
			n->mSparseGridBuilder.getSortedParticleToSubgrid(), n->mSparseGridBuilder.getGridParameters().maxNumSubgrids,
			n->mSparseGridBuilder.getSubgridNeighborLookup(), n->getSubgridEndIndicesBuffer(), numParticles, NULL, 0, reinterpret_cast<float4*>(mAnisotropyDataHost.mAnisotropy_q1),
			reinterpret_cast<float4*>(mAnisotropyDataHost.mAnisotropy_q2), reinterpret_cast<float4*>(mAnisotropyDataHost.mAnisotropy_q3),
			mAnisotropyDataHost.mAnisotropy, mAnisotropyDataHost.mAnisotropyMin, mAnisotropyDataHost.mAnisotropyMax, 2 * particleContactOffset, stream);

		if (mAnisotropy1)
		{
			mKernelLauncher.getCudaContextManager()->getCudaContext()->memcpyDtoHAsync(mAnisotropy1, CUdeviceptr(mAnisotropyDataHost.mAnisotropy_q1), numParticles * sizeof(PxVec4), stream);
			mKernelLauncher.getCudaContextManager()->getCudaContext()->memcpyDtoHAsync(mAnisotropy2, CUdeviceptr(mAnisotropyDataHost.mAnisotropy_q2), numParticles * sizeof(PxVec4), stream);
			mKernelLauncher.getCudaContextManager()->getCudaContext()->memcpyDtoHAsync(mAnisotropy3, CUdeviceptr(mAnisotropyDataHost.mAnisotropy_q3), numParticles * sizeof(PxVec4), stream);
		}
	}


	void PxgAnisotropyGenerator::setMaxParticles(PxU32 maxParticles)
	{
		if (maxParticles == mNumParticles)
			return;

		mNumParticles = maxParticles;

		if (!mOwnsAnisotropyGPUBuffers)
			return;

		releaseGPUAnisotropyBuffers();
		allocateGPUAnisotropyBuffers();
	}
}
