// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "PxgSmoothing.h"
#include "PxgAlgorithms.h"
#include "PxgSparseGridStandalone.h"

#include "PxgAnisotropyData.h"

#include "PxPhysics.h"
#include "PxParticleSystem.h"
#include "foundation/PxUserAllocated.h"
#include "PxgCudaMemoryAllocator.h"

#include "PxParticleGpu.h"
#include "PxgParticleNeighborhoodProvider.h"

#include "PxPhysXGpu.h"
#include "PxvGlobals.h"
#include "PxgKernelIndices.h"
#include "PxgCudaMemoryAllocator.h"

using namespace physx;

#if ENABLE_KERNEL_LAUNCH_ERROR_CHECK
	#define checkCudaError() { cudaError_t err = cudaDeviceSynchronize(); if (err != 0) printf("Cuda error file: %s, line: %i, error: %i\n", PX_FL, err); }	
#else
	#define checkCudaError() { }
#endif

void updateSmoothedPositions(PxgKernelLauncher& launcher, PxGpuParticleSystem* particleSystems, const PxU32 id, PxSmoothedPositionData* smoothingDataPerParticleSystem,
	PxU32 numParticles, CUstream stream, PxU32 numThreadsPerBlock = 256)
{
	const PxU32 numBlocks = (numParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
	launcher.launchKernel(PxgKernelIds::smoothPositionsLaunch, numBlocks, numThreadsPerBlock, 0, stream,
		particleSystems, id, smoothingDataPerParticleSystem);
	checkCudaError();
}
	
void smoothPositionsLaunch(PxgKernelLauncher& launcher, float4* deviceParticlePos, PxU32* sortedToOriginalParticleIndex, PxU32* sortedParticleToSubgrid, PxU32 maxNumSubgrids,
	PxU32* subgridNeighbors, PxU32* subgridEndIndices, int numParticles, PxU32* phases, PxU32 validPhaseMask,
	float4* smoothPos, PxReal smoothing, PxReal particleContactDistance, CUstream stream, PxU32 numThreadsPerBlock = 256)
{
	const PxU32 numBlocks = (numParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
	launcher.launchKernel(PxgKernelIds::smoothPositionsKernel, numBlocks, numThreadsPerBlock, 0, stream,
		deviceParticlePos, sortedToOriginalParticleIndex, sortedParticleToSubgrid, maxNumSubgrids,
		subgridNeighbors, subgridEndIndices, numParticles, phases, validPhaseMask, smoothPos, smoothing, particleContactDistance);
	checkCudaError();
}
	
void PxgSmoothedPositionGenerator::releaseGPUSmoothedPositionBuffers()
{
	if (!mPositionSmoothingDataHost.mPositions)
		return;

	PX_DEVICE_MEMORY_FREE(*mKernelLauncher.getCudaContextManager(), mPositionSmoothingDataHost.mPositions);
	mOwnsSmoothedPositionGPUBuffers = false;
}

void PxgSmoothedPositionGenerator::allocateGPUSmoothedPositionBuffers()
{
	if (mPositionSmoothingDataHost.mPositions)
		return;

	mPositionSmoothingDataHost.mPositions = PX_DEVICE_MEMORY_ALLOC(PxVec4, *mKernelLauncher.getCudaContextManager(), mNumParticles);
	mOwnsSmoothedPositionGPUBuffers = true;
}

PxgSmoothedPositionGenerator::PxgSmoothedPositionGenerator(PxgKernelLauncher& cudaContextManager, PxU32 maxNumParticles, PxReal smoothingStrenght)
	: mSmoothedPositions(NULL), mEnabled(true)
{
	mPositionSmoothingDataHost.mPositions = NULL;
	mKernelLauncher = cudaContextManager;
	mNumParticles = maxNumParticles;
	mPositionSmoothingDataPerParticleSystemDevice = PX_DEVICE_MEMORY_ALLOC(PxSmoothedPositionData, *mKernelLauncher.getCudaContextManager(), 1);

	mPositionSmoothingDataHost.mSmoothing = smoothingStrenght;
	mDirty = true;
	mOwnsSmoothedPositionGPUBuffers = false;
}

void PxgSmoothedPositionGenerator::release()
{
	if (!mPositionSmoothingDataPerParticleSystemDevice)
		return;

	PX_DEVICE_MEMORY_FREE(*mKernelLauncher.getCudaContextManager(), mPositionSmoothingDataPerParticleSystemDevice);
	if (mOwnsSmoothedPositionGPUBuffers)
		releaseGPUSmoothedPositionBuffers();

	PX_DELETE_THIS;
}

void PxgSmoothedPositionGenerator::generateSmoothedPositions(PxGpuParticleSystem* gpuParticleSystem, PxU32 numParticles, CUstream stream)
{
	if (mDirty)
	{
		mDirty = false;
		mKernelLauncher.getCudaContextManager()->getCudaContext()->memcpyHtoDAsync(CUdeviceptr(mPositionSmoothingDataPerParticleSystemDevice), &mPositionSmoothingDataHost, sizeof(PxSmoothedPositionData), stream);
	}

	updateSmoothedPositions(mKernelLauncher, gpuParticleSystem, 0, mPositionSmoothingDataPerParticleSystemDevice, numParticles, stream);

	if (mSmoothedPositions)
	{
		mKernelLauncher.getCudaContextManager()->getCudaContext()->memcpyDtoHAsync(mSmoothedPositions, CUdeviceptr(mPositionSmoothingDataHost.mPositions), numParticles * sizeof(PxVec4), stream);
	}
}

void PxgSmoothedPositionGenerator::generateSmoothedPositions(PxVec4* particlePositionsGpu, PxParticleNeighborhoodProvider& neighborhoodProvider, PxU32 numParticles, PxReal particleContactOffset, CUstream stream)
{
	PxgParticleNeighborhoodProvider* n = static_cast<PxgParticleNeighborhoodProvider*>(&neighborhoodProvider);

	smoothPositionsLaunch(mKernelLauncher, reinterpret_cast<float4*>(particlePositionsGpu/*n->reorderedParticles*/), n->mSparseGridBuilder.getSortedToOriginalParticleIndex(),
		n->mSparseGridBuilder.getSortedParticleToSubgrid(), n->mSparseGridBuilder.getGridParameters().maxNumSubgrids,
		n->mSparseGridBuilder.getSubgridNeighborLookup(), n->getSubgridEndIndicesBuffer(), numParticles, NULL, 0, reinterpret_cast<float4*>(mPositionSmoothingDataHost.mPositions),
		mPositionSmoothingDataHost.mSmoothing, 2 * particleContactOffset, stream);

	if (mSmoothedPositions)
	{
		mKernelLauncher.getCudaContextManager()->getCudaContext()->memcpyDtoHAsync(mSmoothedPositions, CUdeviceptr(mPositionSmoothingDataHost.mPositions), numParticles * sizeof(PxVec4), stream);
	}
}

void PxgSmoothedPositionGenerator::setMaxParticles(PxU32 maxParticles)
{
	if (maxParticles == mNumParticles)
		return;

	mNumParticles = maxParticles;

	if (!mOwnsSmoothedPositionGPUBuffers)
		return;

	releaseGPUSmoothedPositionBuffers();
	allocateGPUSmoothedPositionBuffers();
}
