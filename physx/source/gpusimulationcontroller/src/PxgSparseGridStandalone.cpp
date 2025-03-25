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

#include "PxgSparseGridStandalone.h"
#include "PxSparseGridParams.h"
#include "foundation/PxArray.h"

#include "PxgCudaMemoryAllocator.h"

#include "PxPhysXGpu.h"
#include "PxvGlobals.h"
#include "PxgKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "PxgKernelLauncher.h"
#include "PxgCudaMemoryAllocator.h"
#include "PxgCudaHelpers.h"

namespace physx
{
#if ENABLE_KERNEL_LAUNCH_ERROR_CHECK
#define checkCudaError() { cudaError_t err = cudaDeviceSynchronize(); if (err != 0) printf("Cuda error file: %s, line: %i, error: %i\n", PX_FL, err); }	
#else
#define checkCudaError() { }
#endif

#define THREADS_PER_BLOCK 256

	void sparseGridReuseSubgrids(PxgKernelLauncher& launcher, const PxSparseGridParams& sparseGridParams,
		const PxU32* uniqueHashkeysPerSubgridPreviousUpdate, const PxU32* numActiveSubgridsPreviousUpdate, PxU32* subgridOrderMapPreviousUpdate,
		const PxU32* uniqueHashkeysPerSubgrid, const PxU32* numActiveSubgrids, PxU32* subgridOrderMap,
		CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (sparseGridParams.maxNumSubgrids + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::sg_ReuseSubgrids, numBlocks, numThreadsPerBlock, 0, stream,
			sparseGridParams, uniqueHashkeysPerSubgridPreviousUpdate, numActiveSubgridsPreviousUpdate, subgridOrderMapPreviousUpdate,
			uniqueHashkeysPerSubgrid, numActiveSubgrids, subgridOrderMap);
		checkCudaError();
	}


	void sparseGridAddReleasedSubgridsToUnusedStack(PxgKernelLauncher& launcher, const PxSparseGridParams& sparseGridParams,
		const PxU32* numActiveSubgridsPreviousUpdate, const PxU32* subgridOrderMapPreviousUpdate,
		PxU32* unusedSubgridStackSize, PxU32* unusedSubgridStack, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (sparseGridParams.maxNumSubgrids + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::sg_AddReleasedSubgridsToUnusedStack, numBlocks, numThreadsPerBlock, 0, stream,
			numActiveSubgridsPreviousUpdate, subgridOrderMapPreviousUpdate, unusedSubgridStackSize, unusedSubgridStack);
		checkCudaError();
	}

	void sparseGridAllocateNewSubgrids(PxgKernelLauncher& launcher, const PxSparseGridParams& sparseGridParams, const PxU32* numActiveSubgrids, PxU32* subgridOrderMap,
		PxU32* unusedSubgridStackSize, PxU32* unusedSubgridStack, const PxU32* numActiveSubgridsPreviousUpdate, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (sparseGridParams.maxNumSubgrids + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::sg_AllocateNewSubgrids, numBlocks, numThreadsPerBlock, 0, stream,
			numActiveSubgrids, subgridOrderMap, unusedSubgridStackSize, unusedSubgridStack, numActiveSubgridsPreviousUpdate, sparseGridParams.maxNumSubgrids);
		checkCudaError();
	}


	void sparseGridCalcSubgridHashes(PxgKernelLauncher& launcher, const PxSparseGridParams& sparseGridParams, PxU32* indices,
		PxU32* hashkeyPerParticle, PxVec4* deviceParticlePos, const int numParticles,
		const PxU32* phases, const PxU32 validPhaseMask, CUstream stream, const PxU32* activeIndices = NULL)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (numParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::sg_SparseGridCalcSubgridHashes, numBlocks, numThreadsPerBlock, 0, stream,
			sparseGridParams, indices, hashkeyPerParticle, deviceParticlePos,
			numParticles, phases, validPhaseMask, activeIndices);
		checkCudaError();
	}


	void sparseGridMarkRequiredNeighbors(PxgKernelLauncher& launcher, PxU32* outRequiredNeighborMask, PxU32* uniqueSortedHashkey, const PxSparseGridParams sparseGridParams, PxU32 neighborhoodSize,
		PxVec4* particlePositions, const PxU32 numParticles, const PxU32* phases, const PxU32 validPhaseMask, CUstream stream, const PxU32* activeIndices = NULL)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (numParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::sg_SparseGridMarkRequiredNeighbors, numBlocks, numThreadsPerBlock, 0, stream,
			outRequiredNeighborMask, uniqueSortedHashkey, sparseGridParams, neighborhoodSize, particlePositions,
			numParticles, phases, validPhaseMask, activeIndices);
		checkCudaError();
	}


	void sparseGridSortedArrayToDelta(PxgKernelLauncher& launcher, const PxU32* in, const PxU32* mask, PxU32* out, PxU32 n, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (n + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::sg_SparseGridSortedArrayToDelta, numBlocks, numThreadsPerBlock, 0, stream,
			in, mask, out, n);
		checkCudaError();
	}

	void sparseGridGetUniqueValues(PxgKernelLauncher& launcher, const PxU32* sortedData, const PxU32* indices, PxU32* uniqueValues,
		const PxU32 n, PxU32* subgridNeighborCollector, const PxU32 uniqueValuesSize, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (n + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::sg_SparseGridGetUniqueValues, numBlocks, numThreadsPerBlock, 0, stream,
			sortedData, indices, uniqueValues, n, subgridNeighborCollector, uniqueValuesSize);
		checkCudaError();
	}

	void sparseGridBuildSubgridNeighbors(PxgKernelLauncher& launcher, const PxU32* uniqueSortedHashkey, const PxU32* numActiveSubgrids,
		const PxU32 maxNumSubgrids, PxU32* subgridNeighbors, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (maxNumSubgrids + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::sg_SparseGridBuildSubgridNeighbors, numBlocks, numThreadsPerBlock, 0, stream,
			uniqueSortedHashkey, numActiveSubgrids, maxNumSubgrids, subgridNeighbors);
		checkCudaError();
	}


	void sparseGridMarkSubgridEndIndicesLaunch(PxgKernelLauncher& launcher, const PxU32* sortedParticleToSubgrid, PxU32 numParticles, PxU32* subgridEndIndices, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (numParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::sg_MarkSubgridEndIndices, numBlocks, numThreadsPerBlock, 0, stream,
			sortedParticleToSubgrid, numParticles, subgridEndIndices);
		checkCudaError();
	}

	void PxSparseGridBuilder::initialize(PxgKernelLauncher* kernelLauncher, const PxSparseGridParams& sparseGridParams, PxU32 maxNumParticles, PxU32 neighborhoodSize, bool trackParticleOrder)
	{
		mMaxParticles = maxNumParticles;

		mKernelLauncher = kernelLauncher;
		mSparseGridParams = sparseGridParams;
		mNeighborhoodSize = neighborhoodSize;
		mTrackParticleOrder = trackParticleOrder;

		mScan.initialize(kernelLauncher, maxNumParticles);
		mSort.initialize(kernelLauncher, maxNumParticles);
		mHashkeyPerParticle = PX_DEVICE_MEMORY_ALLOC(PxU32, *kernelLauncher->getCudaContextManager(), maxNumParticles);
		mSortedParticleToSubgrid = PX_DEVICE_MEMORY_ALLOC(PxU32, *kernelLauncher->getCudaContextManager(), maxNumParticles);
		mSortedUniqueHashkeysPerSubgrid = PX_DEVICE_MEMORY_ALLOC(PxU32, *kernelLauncher->getCudaContextManager(), sparseGridParams.maxNumSubgrids);
		mSubgridNeighborLookup = PX_DEVICE_MEMORY_ALLOC(PxU32, *kernelLauncher->getCudaContextManager(), 27 * sparseGridParams.maxNumSubgrids);

		if (trackParticleOrder)
			mSortedToOriginalParticleIndex = PX_DEVICE_MEMORY_ALLOC(PxU32, *kernelLauncher->getCudaContextManager(), maxNumParticles);

		if (mNeighborhoodSize > 0)
		{
			mScanNeighbors.initialize(kernelLauncher, 27 * sparseGridParams.maxNumSubgrids);
			mNeighborSort.initialize(kernelLauncher, 27 * sparseGridParams.maxNumSubgrids);
			mNeighborCollector = PX_DEVICE_MEMORY_ALLOC(PxU32, *kernelLauncher->getCudaContextManager(), 27 * sparseGridParams.maxNumSubgrids);
			mRequiredNeighborMask = PX_DEVICE_MEMORY_ALLOC(PxU32, *kernelLauncher->getCudaContextManager(), 27 * sparseGridParams.maxNumSubgrids);
		}
	}

	void PxSparseGridBuilder::release()
	{
		if (!mHashkeyPerParticle)
			return;

		mScan.release();
		mSort.release();

		PX_DEVICE_MEMORY_FREE(*mKernelLauncher->getCudaContextManager(), mHashkeyPerParticle);
		PX_DEVICE_MEMORY_FREE(*mKernelLauncher->getCudaContextManager(), mSortedParticleToSubgrid);
		PX_DEVICE_MEMORY_FREE(*mKernelLauncher->getCudaContextManager(), mSortedUniqueHashkeysPerSubgrid);
		PX_DEVICE_MEMORY_FREE(*mKernelLauncher->getCudaContextManager(), mSubgridNeighborLookup);
		PX_DEVICE_MEMORY_FREE(*mKernelLauncher->getCudaContextManager(), mSortedToOriginalParticleIndex);

		if (mNeighborhoodSize > 0)
		{
			mScanNeighbors.release();
			mNeighborSort.release();
			PX_DEVICE_MEMORY_FREE(*mKernelLauncher->getCudaContextManager(), mNeighborCollector);
			PX_DEVICE_MEMORY_FREE(*mKernelLauncher->getCudaContextManager(), mRequiredNeighborMask);
		}
	}

	void PxSparseGridBuilder::updateSubgridEndIndices(PxU32 numParticles, CUstream stream)
	{
		//Hijack a buffer that is not accessed after the subgrid update with exactly the right size
		PxU32* subgridEndIndicesBuffer = mSortedUniqueHashkeysPerSubgrid;
		sparseGridMarkSubgridEndIndicesLaunch(*mKernelLauncher, mSortedParticleToSubgrid, numParticles, subgridEndIndicesBuffer, stream);
	}

	PxU32* PxSparseGridBuilder::updateSubgrids(PxVec4* deviceParticlePos, PxU32 numParticles, PxU32* devicePhases, CUstream stream, PxU32 validPhase, const PxU32* activeIndices)
	{
		sparseGridCalcSubgridHashes(*mKernelLauncher, mSparseGridParams, mSortedParticleToSubgrid, mHashkeyPerParticle, deviceParticlePos, numParticles, devicePhases, validPhase, stream, activeIndices);
		mSort.sort(mHashkeyPerParticle, 32, stream, mSortedToOriginalParticleIndex, numParticles);

		sparseGridSortedArrayToDelta(*mKernelLauncher, mHashkeyPerParticle, NULL, mSortedParticleToSubgrid, numParticles, stream);
		mScan.exclusiveScan(mSortedParticleToSubgrid, stream, numParticles);

		PxU32* totalCountPointer;
		if (mNeighborhoodSize > 0)
		{
			totalCountPointer = mScanNeighbors.getSumPointer();

			sparseGridGetUniqueValues(*mKernelLauncher, mHashkeyPerParticle, mSortedParticleToSubgrid, mSortedUniqueHashkeysPerSubgrid, numParticles, mNeighborCollector, mSparseGridParams.maxNumSubgrids, stream);
			mNeighborSort.sort(mNeighborCollector, 32, stream);

			PxgCudaHelpers::memsetAsync(*mKernelLauncher->getCudaContextManager(), mRequiredNeighborMask, PxU32(0), 27 * mSparseGridParams.maxNumSubgrids, stream);
			sparseGridMarkRequiredNeighbors(*mKernelLauncher, mRequiredNeighborMask, mNeighborCollector, mSparseGridParams, mNeighborhoodSize, deviceParticlePos, numParticles, devicePhases, validPhase, stream, activeIndices);

			PxU32* tmpBuffer = mSubgridNeighborLookup; //This memory is only used temporary and populated later. It is always large enough to hold the temporary data.
			sparseGridSortedArrayToDelta(*mKernelLauncher, mNeighborCollector, mRequiredNeighborMask, tmpBuffer, 27 * mSparseGridParams.maxNumSubgrids, stream);

			mScanNeighbors.exclusiveScan(tmpBuffer, stream);

			sparseGridGetUniqueValues(*mKernelLauncher, mNeighborCollector, tmpBuffer, mSortedUniqueHashkeysPerSubgrid, 27 * mSparseGridParams.maxNumSubgrids, NULL, mSparseGridParams.maxNumSubgrids, stream);
		}
		else
		{
			totalCountPointer = mScan.getSumPointer();
			sparseGridGetUniqueValues(*mKernelLauncher, mHashkeyPerParticle, mSortedParticleToSubgrid, mSortedUniqueHashkeysPerSubgrid, numParticles, NULL, mSparseGridParams.maxNumSubgrids, stream);
		}
		return totalCountPointer;
	}

	void PxSparseGridBuilder::updateSubgridNeighbors(PxU32* totalCountPointer, CUstream stream)
	{
		sparseGridBuildSubgridNeighbors(*mKernelLauncher, mSortedUniqueHashkeysPerSubgrid, totalCountPointer, mSparseGridParams.maxNumSubgrids, mSubgridNeighborLookup, stream);
		if (mCopySubgridsInUseToHost)
			mKernelLauncher->getCudaContextManager()->getCudaContext()->memcpyDtoHAsync(&mNumSubgridsInUse, CUdeviceptr(totalCountPointer), sizeof(PxU32), stream);
	}

	void PxSparseGridBuilder::updateSparseGrid(PxVec4* deviceParticlePos, const PxU32 numParticles, PxU32* devicePhases, CUstream stream, PxU32 validPhase, const PxU32* activeIndices)
	{
		PxU32* totalCountPointer = updateSubgrids(deviceParticlePos, numParticles, devicePhases, stream, validPhase, activeIndices);			
		updateSubgridNeighbors(totalCountPointer, stream);
	}	
}
