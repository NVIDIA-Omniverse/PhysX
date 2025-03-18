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

#include "PxgSDFBuilder.h"

#include "PxgKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "PxgCudaMemoryAllocator.h"
#include "PxgCudaHelpers.h"

#include "foundation/PxBounds3.h"
#include "foundation/PxErrors.h"
#include "foundation/PxFoundation.h"
#include "foundation/PxSimpleTypes.h"

#define EXTENDED_DEBUG 0

using namespace physx;

// re-allocation policy of 1.5x 
static PX_FORCE_INLINE PxU32 calculateSlack(PxU32 n)
{
	return n * 3 / 2;
}
	
void PxgLinearBVHBuilderGPU::resizeBVH(PxgBVH& bvh, PxU32 numNodes)
{
	if (numNodes > bvh.mMaxNodes)
	{
		const PxU32 numToAlloc = calculateSlack(numNodes);

		PxCudaContextManager* ccm = mKernelLauncher.getCudaContextManager();

		PX_DEVICE_MEMORY_FREE(*ccm, bvh.mNodeLowers);
		PX_DEVICE_MEMORY_FREE(*ccm, bvh.mNodeUppers);

		bvh.mNodeLowers = PX_DEVICE_MEMORY_ALLOC(PxgPackedNodeHalf, *ccm, numToAlloc);
		bvh.mNodeUppers = PX_DEVICE_MEMORY_ALLOC(PxgPackedNodeHalf, *ccm, numToAlloc);

		bvh.mMaxNodes = numToAlloc;

		if (!bvh.mRootNode)
		{
			bvh.mRootNode = PX_DEVICE_MEMORY_ALLOC(PxU32, *ccm, 2);
		}
	}

	bvh.mNumNodes = numNodes;
}

void PxgLinearBVHBuilderGPU::releaseBVH(PxgBVH& bvh)
{
	PxCudaContextManager* ccm = mKernelLauncher.getCudaContextManager();
	PX_DEVICE_MEMORY_FREE(*ccm, bvh.mNodeLowers);
	PX_DEVICE_MEMORY_FREE(*ccm, bvh.mNodeUppers);
	PX_DEVICE_MEMORY_FREE(*ccm, bvh.mRootNode);
}	

PxgLinearBVHBuilderGPU::PxgLinearBVHBuilderGPU(PxgKernelLauncher& kernelLauncher)
	: mMaxTreeDepth(NULL)
	, mKernelLauncher(kernelLauncher)
	, mIndices(NULL)
	, mKeys(NULL)
	, mDeltas(NULL)
	, mRangeLefts(NULL)
	, mRangeRights(NULL)
	, mNumChildren(NULL)		
	, mTotalLower(NULL)
	, mTotalUpper(NULL)
	, mTotalInvEdges(NULL)
	, mMaxItems(0)
{
	PxCudaContextManager* ccm = kernelLauncher.getCudaContextManager();

	mTotalLower = PX_DEVICE_MEMORY_ALLOC(PxVec3, *ccm, 1);
	mTotalUpper = PX_DEVICE_MEMORY_ALLOC(PxVec3, *ccm, 1);
	mTotalInvEdges = PX_DEVICE_MEMORY_ALLOC(PxVec3, *ccm, 1);
}

void PxgLinearBVHBuilderGPU::allocateOrResize(PxgBVH& bvh, PxU32 numItems)
{
	const PxU32 maxNodes = 2 * numItems;

	resizeBVH(bvh, maxNodes);

	if (numItems > mMaxItems)
	{
		const PxU32 itemsToAlloc = (numItems * 3) / 2;
		const PxU32 nodesToAlloc = (maxNodes * 3) / 2;

		PxCudaContextManager* ccm = mKernelLauncher.getCudaContextManager();

		// reallocate temporary storage if necessary
		PX_DEVICE_MEMORY_FREE(*ccm, mIndices);
		PX_DEVICE_MEMORY_FREE(*ccm, mKeys);
		PX_DEVICE_MEMORY_FREE(*ccm, mDeltas);
		PX_DEVICE_MEMORY_FREE(*ccm, mRangeLefts);
		PX_DEVICE_MEMORY_FREE(*ccm, mRangeRights);
		PX_DEVICE_MEMORY_FREE(*ccm, mNumChildren);

		PX_PINNED_MEMORY_FREE(*ccm, mMaxTreeDepth);

		mIndices = PX_DEVICE_MEMORY_ALLOC(PxU32, *ccm, itemsToAlloc);
		mKeys = PX_DEVICE_MEMORY_ALLOC(PxI32, *ccm, itemsToAlloc);
		mDeltas = PX_DEVICE_MEMORY_ALLOC(PxReal, *ccm, itemsToAlloc);// highest differenting bit between keys for item i and i+1
		mRangeLefts = PX_DEVICE_MEMORY_ALLOC(PxI32, *ccm, nodesToAlloc);
		mRangeRights = PX_DEVICE_MEMORY_ALLOC(PxI32, *ccm, nodesToAlloc);
		mNumChildren = PX_DEVICE_MEMORY_ALLOC(PxI32, *ccm, nodesToAlloc);

		mMaxTreeDepth = PX_PINNED_MEMORY_ALLOC(PxI32, *ccm, 1);

		mMaxItems = itemsToAlloc;

		mSort.release();
		mSort.initialize(&mKernelLauncher, numItems);
	}
}

void PxgLinearBVHBuilderGPU::release()
{
	PxCudaContextManager* ccm = mKernelLauncher.getCudaContextManager();

	mSort.release();

	PX_DEVICE_MEMORY_FREE(*ccm, mIndices);
	PX_DEVICE_MEMORY_FREE(*ccm, mKeys);
	PX_DEVICE_MEMORY_FREE(*ccm, mDeltas);
	PX_DEVICE_MEMORY_FREE(*ccm, mRangeLefts);
	PX_DEVICE_MEMORY_FREE(*ccm, mRangeRights);
	PX_DEVICE_MEMORY_FREE(*ccm, mNumChildren);

	PX_PINNED_MEMORY_FREE(*ccm, mMaxTreeDepth);

	PX_DEVICE_MEMORY_FREE(*ccm, mTotalLower);
	PX_DEVICE_MEMORY_FREE(*ccm, mTotalUpper);
	PX_DEVICE_MEMORY_FREE(*ccm, mTotalInvEdges);

	mMaxItems = 0;
}
	
void PxgLinearBVHBuilderGPU::buildFromTriangles(PxgBVH& bvh, const PxVec3* vertices, const PxU32* triangleIndices, const PxI32* itemPriorities, PxI32 numItems, PxBounds3* totalBounds, CUstream stream, PxReal boxMargin)
{
	allocateOrResize(bvh, numItems);

	//Since maxNodes is 2*numItems, the second half of bvh.mNodeLowers and bvh.mNodeUppers
	//Can be used as scratch memory until the BuildHierarchy kernel gets launched
	PX_COMPILE_TIME_ASSERT(sizeof(PxVec4) == sizeof(PxgPackedNodeHalf));
	PxVec4* itemLowers = reinterpret_cast<PxVec4*>(&bvh.mNodeLowers[numItems]);
	PxVec4* itemUppers = reinterpret_cast<PxVec4*>(&bvh.mNodeUppers[numItems]);

	PxU32 kNumThreadsPerBlock = PxgBVHKernelBlockDim::BUILD_HIERARCHY;
	PxU32 kNumBlocks = (numItems + kNumThreadsPerBlock - 1) / kNumThreadsPerBlock;
	mKernelLauncher.launchKernelPtr(PxgKernelIds::bvh_ComputeTriangleBounds, kNumBlocks, kNumThreadsPerBlock, 0, stream,
		&vertices, &triangleIndices, &numItems, &itemLowers, &itemUppers, &boxMargin);

	prepareHierarchConstruction(bvh, itemLowers, itemUppers, itemPriorities, numItems, totalBounds, stream);

	kNumThreadsPerBlock = PxgBVHKernelBlockDim::BUILD_HIERARCHY;
	kNumBlocks = (numItems + kNumThreadsPerBlock - 1) / kNumThreadsPerBlock;
	mKernelLauncher.launchKernelPtr(PxgKernelIds::bvh_BuildHierarchy, kNumBlocks, kNumThreadsPerBlock, 0, stream,
		&numItems, &bvh.mRootNode, &mMaxTreeDepth, &mDeltas, &mNumChildren, &mRangeLefts, &mRangeRights, &bvh.mNodeLowers, &bvh.mNodeUppers);
}

void PxgLinearBVHBuilderGPU::buildTreeAndWindingClustersFromTriangles(PxgBVH& bvh, PxgWindingClusterApproximation* windingNumberClustersD, const PxVec3* vertices, const PxU32* triangleIndices, const PxI32* itemPriorities,
	PxI32 numItems, PxBounds3* totalBounds, CUstream stream, PxReal boxMargin, bool skipAllocate)
{
	if (!skipAllocate)
		allocateOrResize(bvh, numItems);

	//Since maxNodes is 2*numItems, the second half of bvh.mNodeLowers and bvh.mNodeUppers
	//Can be used as scratch memory until the BuildHierarchy kernel gets launched
	PX_COMPILE_TIME_ASSERT(sizeof(PxVec4) == sizeof(PxgPackedNodeHalf));
	PxVec4* itemLowers = reinterpret_cast<PxVec4*>(&bvh.mNodeLowers[numItems]);
	PxVec4* itemUppers = reinterpret_cast<PxVec4*>(&bvh.mNodeUppers[numItems]);

	const PxU32 kNumThreadsPerBlock = PxgBVHKernelBlockDim::BUILD_HIERARCHY;
	const PxU32 kNumBlocks = (numItems + kNumThreadsPerBlock - 1) / kNumThreadsPerBlock;

	mKernelLauncher.launchKernelPtr(PxgKernelIds::bvh_ComputeTriangleBounds, kNumBlocks, kNumThreadsPerBlock, 0, stream,
		&vertices, &triangleIndices, &numItems, &itemLowers, &itemUppers, &boxMargin);

	prepareHierarchConstruction(bvh, itemLowers, itemUppers, itemPriorities, numItems, totalBounds, stream);

	mKernelLauncher.launchKernelPtr(PxgKernelIds::bvh_BuildHierarchyAndWindingClusters, kNumBlocks, kNumThreadsPerBlock, 0, stream,
		&numItems, &bvh.mRootNode, &mMaxTreeDepth, &mDeltas, &mNumChildren, &mRangeLefts, &mRangeRights, &bvh.mNodeLowers, &bvh.mNodeUppers,
		&windingNumberClustersD, &vertices, &triangleIndices);

#if EXTENDED_DEBUG
	bool debugTree = false;
	if (debugTree)
	{
		PxCudaContextManager* ccm = mKernelLauncher.getCudaContextManager();
		PxCUresult result = ccm->getCudaContext()->streamSynchronize(stream);
		PX_ASSERT(result == CUDA_SUCCESS);

		PxArray<PxVec4> lowerDebug;
		lowerDebug.resize(2 * numItems);
		PxArray<PxVec4> upperDebug;
		upperDebug.resize(2 * numItems);
		PxU32 root = 0xFFFFFFFF;

		result = ccm->getCudaContext()->streamSynchronize(stream);
		PX_ASSERT(result == CUDA_SUCCESS);

		PxgCudaHelpers::copyDToH(*ccm->getCudaContext(), lowerDebug.begin(), reinterpret_cast<PxVec4*>(bvh.mNodeLowers), lowerDebug.size());
		PxgCudaHelpers::copyDToH(*ccm->getCudaContext(), upperDebug.begin(), reinterpret_cast<PxVec4*>(bvh.mNodeUppers), upperDebug.size());
		PxgCudaHelpers::copyDToH(*ccm->getCudaContext(), &root, bvh.mRootNode, 1u);

		result = ccm->getCudaContext()->streamSynchronize(stream);
		PX_ASSERT(result == CUDA_SUCCESS);
	}
#endif
}

void PxgLinearBVHBuilderGPU::buildFromLeaveBounds(PxgBVH& bvh, const PxVec4* itemLowers, const PxVec4* itemUppers, const PxI32* itemPriorities, PxI32 numItems, PxBounds3* totalBounds, CUstream stream, bool skipAllocate)
{
	const PxU32 kNumThreadsPerBlock = PxgBVHKernelBlockDim::BUILD_HIERARCHY;
	const PxU32 kNumBlocks = (numItems + kNumThreadsPerBlock - 1) / kNumThreadsPerBlock;

	if (!skipAllocate)
		allocateOrResize(bvh, numItems);

	prepareHierarchConstruction(bvh, itemLowers, itemUppers, itemPriorities, numItems, totalBounds, stream);

	mKernelLauncher.launchKernelPtr(PxgKernelIds::bvh_BuildHierarchy, kNumBlocks, kNumThreadsPerBlock, 0, stream,
		&numItems, &bvh.mRootNode, &mMaxTreeDepth, &mDeltas, &mNumChildren, &mRangeLefts, &mRangeRights, &bvh.mNodeLowers, &bvh.mNodeUppers);
}

void PxgLinearBVHBuilderGPU::prepareHierarchConstruction(PxgBVH& bvh, const PxVec4* itemLowers, const PxVec4* itemUppers, const PxI32* itemPriorities, PxI32 numItems, PxBounds3* totalBounds, CUstream stream)
{
	const PxU32 maxNodes = 2 * numItems;

	const PxU32 kNumThreadsPerBlock = PxgBVHKernelBlockDim::BUILD_HIERARCHY;
	const PxU32 kNumBlocks = (numItems + kNumThreadsPerBlock - 1) / kNumThreadsPerBlock;

	PxCudaContextManager* ccm = mKernelLauncher.getCudaContextManager();

	// if total bounds supplied by the host then we just 
	// compute our edge length and upload it to the GPU directly
	if (totalBounds)
	{
		// calculate Morton codes
		PxVec3 edges = (*totalBounds).getDimensions();
		edges += PxVec3(0.0001f);

		PxVec3 invEdges = PxVec3(1.0f / edges.x, 1.0f / edges.y, 1.0f / edges.z);

		PxgCudaHelpers::copyHToDAsync(*ccm->getCudaContext(), mTotalLower, &totalBounds->minimum, 1, stream);
		PxgCudaHelpers::copyHToDAsync(*ccm->getCudaContext(), mTotalUpper, &totalBounds->maximum, 1, stream);
		PxgCudaHelpers::copyHToDAsync(*ccm->getCudaContext(), mTotalInvEdges, &invEdges, 1, stream);
	}
	else
	{
		static const PxVec3 upper(-FLT_MAX);
		static const PxVec3 lower(FLT_MAX);

		PxgCudaHelpers::copyHToDAsync(*ccm->getCudaContext(), mTotalLower, &lower, 1, stream);
		PxgCudaHelpers::copyHToDAsync(*ccm->getCudaContext(), mTotalUpper, &upper, 1, stream);

		// compute the bounds union on the GPU
		mKernelLauncher.launchKernelPtr(PxgKernelIds::bvh_ComputeTotalBounds, kNumBlocks, kNumThreadsPerBlock, 0, stream,
			&itemLowers, &itemUppers, &mTotalLower, &mTotalUpper, &numItems);

		// compute the total edge length
		mKernelLauncher.launchKernelPtr(PxgKernelIds::bvh_ComputeTotalInvEdges, 1, 1, 0, stream,
			&mTotalLower, &mTotalUpper, &mTotalInvEdges);
	}

	// assign 30-bit Morton code based on the centroid of each triangle and bounds for each leaf
	mKernelLauncher.launchKernelPtr(PxgKernelIds::bvh_CalculateMortonCodes, kNumBlocks, kNumThreadsPerBlock, 0, stream,
		&itemLowers, &itemUppers, &itemPriorities, &numItems, &mTotalLower, &mTotalInvEdges, &mIndices, &mKeys);

	// sort items based on Morton key (note the 32-bit sort key corresponds to the template parameter to Morton3, i.e. 3x9 bit keys combined)
	mSort.sort(reinterpret_cast<PxU32*>(mKeys), 32, stream, mIndices, numItems);

	// initialize leaf nodes
	mKernelLauncher.launchKernelPtr(PxgKernelIds::bvh_BuildLeaves, kNumBlocks, kNumThreadsPerBlock, 0, stream,
		&itemLowers, &itemUppers, &numItems, &mIndices, &mRangeLefts, &mRangeRights, &bvh.mNodeLowers, &bvh.mNodeUppers);
		
	// calculate deltas between adjacent keys
	/*mKernelLauncher.launchKernelPtr(PxgKernelIds::bvh_CalculateKeyDeltas, kNumBlocks, kNumThreadsPerBlock, 0, stream,
		&mKeys, &mDeltas, &numItems);*/
	mKernelLauncher.launchKernelPtr(PxgKernelIds::bvh_CalculateKeyDeltasSquaredDistance, kNumBlocks, kNumThreadsPerBlock, 0, stream,
		&mKeys, &mDeltas, &numItems, &bvh.mNodeLowers, &bvh.mNodeUppers);

	// reset children count, this is our atomic counter so we know when an internal node is complete, only used during building
	PxgCudaHelpers::memsetAsync(*mKernelLauncher.getCudaContextManager()->getCudaContext(), mNumChildren, 0, maxNodes, stream);
}

void PxgSDFBuilder::computeDenseSDF(const PxgBvhTriangleMesh& mesh, const PxgWindingClusterApproximation* windingNumberClustersD,
	const Gu::GridQueryPointSampler& sampler, PxU32 sizeX, PxU32 sizeY, PxU32 sizeZ, PxReal* sdfDataD, CUstream stream, PxReal* windingNumbersD)
{
	PxCUresult result = CUDA_SUCCESS;

	PxU32 blockDimX = 8;
	PxU32 blockDimY = 8;
	PxU32 blockDimZ = 4;
	PxU32 gridDimX = (sizeX + blockDimX - 1) / blockDimX;
	PxU32 gridDimY = (sizeY + blockDimY - 1) / blockDimY;
	PxU32 gridDimZ = (sizeZ + blockDimZ - 1) / blockDimZ;

	bool useHybrid = true;
	if (useHybrid)
	{
#if EXTENDED_DEBUG
		bool enableStatistics = false;
		if (enableStatistics)
			atomicCounter = PX_DEVICE_MEMORY_ALLOC(PxU32, *mKernelLauncher.getCudaContextManager(), 1);
#endif

		result = mKernelLauncher.launchKernelXYZPtr(PxgKernelIds::sdf_CalculateDenseGridHybrid, gridDimX, gridDimY, gridDimZ, blockDimX, blockDimY, blockDimZ, 0, stream,
			&mesh, &windingNumberClustersD, &sampler, &sizeX, &sizeY, &sizeZ, &sdfDataD);

#if EXTENDED_DEBUG
		if (enableStatistics) 
		{
			result = mKernelLauncher.getCudaContextManager()->getCudaContext()->streamSynchronize(stream);
			PX_ASSERT(result == CUDA_SUCCESS);

			PxU32 counter;
			PxgcudaHelpers::copyDToH(*mKernelLauncher.getCudaContextManager(), &counter, atomicCounter, 1);

			result = mKernelLauncher.getCudaContextManager()->getCudaContext()->streamSynchronize(stream);
			PX_ASSERT(result == CUDA_SUCCESS);

			PX_DEVICE_MEMORY_FREE(*mKernelLauncher.getCudaContextManager(), atomicCounter);

			//printf("problematic: %f\n", PxReal(counter) / (sizeX*sizeY*sizeZ));
		}
#endif
	}
	else
	{
		result = mKernelLauncher.launchKernelXYZPtr(PxgKernelIds::sdf_CalculateDenseGridBlocks, gridDimX, gridDimY, gridDimZ, blockDimX, blockDimY, blockDimZ, 0, stream,
			&mesh, &windingNumberClustersD, &sampler, &sizeX, &sizeY, &sizeZ, &sdfDataD, &windingNumbersD);
	}		

	PX_ASSERT(result == CUDA_SUCCESS);
}

PxgSDFBuilder::PxgSDFBuilder(PxgKernelLauncher& kernelLauncher) : mKernelLauncher(kernelLauncher)
{
}

void PxgSDFBuilder::fixHoles(PxU32 width, PxU32 height, PxU32 depth, PxReal* sdfDataD, const PxVec3& cellSize, const PxVec3& minExtents, const PxVec3& maxExtents,
	Gu::GridQueryPointSampler& sampler, CUstream stream)
{
	PxCudaContextManager* ccm = mKernelLauncher.getCudaContextManager();

	PxBounds3 totalBounds(minExtents, maxExtents);

	//Fix the sdf in case the source triangle mesh has holes		
	const PxU32 numItems = width * height * depth;
	PxU32 kNumThreadsPerBlock = PxgBVHKernelBlockDim::SDF_FIX_HOLES;
	PxU32 kNumBlocks = (numItems + kNumThreadsPerBlock - 1) / kNumThreadsPerBlock;
		
	PxU32* atomicCounter = PX_DEVICE_MEMORY_ALLOC(PxU32, *ccm, 1);
	if (!atomicCounter)
		return;

	PxgCudaHelpers::memsetAsync(*ccm->getCudaContext(), atomicCounter, 0u, 1u, stream);
		
	mKernelLauncher.launchKernelPtr(PxgKernelIds::sdf_CountHoles, kNumBlocks, kNumThreadsPerBlock, 0, stream,
		&sdfDataD, &width, &height, &depth, &cellSize, &atomicCounter);

	PxU32 numPointsInCloud = 0;

	PxCUresult result = ccm->getCudaContext()->streamSynchronize(stream);
	PX_ASSERT(result == CUDA_SUCCESS);

	PxgCudaHelpers::copyDToH(*ccm->getCudaContext(), &numPointsInCloud, atomicCounter, 1);

	result = ccm->getCudaContext()->streamSynchronize(stream);
	PX_ASSERT(result == CUDA_SUCCESS);

	if (numPointsInCloud > 0)
	{
		PxgBVH pointCloudBvh = PxgBVH();
		PxgLinearBVHBuilderGPU treeBuilder(mKernelLauncher);
		treeBuilder.allocateOrResize(pointCloudBvh, numPointsInCloud);

		// abort here if the allocations fail above.
		if (ccm->getCudaContext()->isInAbortMode())
		{
			treeBuilder.releaseBVH(pointCloudBvh);
			treeBuilder.release();
			PX_DEVICE_MEMORY_FREE(*ccm, atomicCounter);

			return;
		}

		//Since maxNodes is 2*numItems, the second half of bvh.mNodeLowers and bvh.mNodeUppers
		//Can be used as scratch memory until the BuildHierarchy kernel gets launched
		PX_COMPILE_TIME_ASSERT(sizeof(PxVec4) == sizeof(PxgPackedNodeHalf));
		PxVec4* pointCloudLowers = reinterpret_cast<PxVec4*>(&pointCloudBvh.mNodeLowers[numPointsInCloud]);
		PxVec4* pointCloudUppers = reinterpret_cast<PxVec4*>(&pointCloudBvh.mNodeUppers[numPointsInCloud]);

		PxgCudaHelpers::memsetAsync(*ccm->getCudaContext(), atomicCounter, 0u, 1u, stream);
		mKernelLauncher.launchKernelPtr(PxgKernelIds::sdf_FindHoles, kNumBlocks, kNumThreadsPerBlock, 0, stream,
			&sdfDataD, &width, &height, &depth, &cellSize, &atomicCounter, &sampler, &pointCloudLowers, &pointCloudUppers, &numPointsInCloud);

		mKernelLauncher.launchKernelPtr(PxgKernelIds::sdf_ApplyHoleCorrections, kNumBlocks, kNumThreadsPerBlock, 0, stream,
			&sdfDataD, &width, &height, &depth, &sampler, &pointCloudUppers, &numPointsInCloud);

		treeBuilder.buildFromLeaveBounds(pointCloudBvh, pointCloudLowers, pointCloudUppers, NULL, numPointsInCloud, &totalBounds, stream, true);

#if EXTENDED_DEBUG
		bool debugTree = false;
		if (debugTree)
		{
			PxArray<PxVec4> lower;
			PxArray<PxVec4> upper;
			lower.resize(numPointsInCloud);
			upper.resize(numPointsInCloud);

			result = ccm->getCudaContext()->streamSynchronize(stream);
			PX_ASSERT(result == CUDA_SUCCESS);

			PxgcudaHelpers::copyDToH(*ccm, lower.begin(), reinterpret_cast<PxVec4*>(&pointCloudBvh.mNodeLowers[0]), numPointsInCloud);
			PxgcudaHelpers::copyDToH(*ccm, upper.begin(), reinterpret_cast<PxVec4*>(&pointCloudBvh.mNodeUppers[0]), numPointsInCloud);

			result = ccm->getCudaContext()->streamSynchronize(stream);
			PX_ASSERT(result == CUDA_SUCCESS);
		}
#endif

		PxU32 blockDimX = 8;
		PxU32 blockDimY = 8;
		PxU32 blockDimZ = 4;
		PxU32 gridDimX = (width + blockDimX - 1) / blockDimX;
		PxU32 gridDimY = (height + blockDimY - 1) / blockDimY;
		PxU32 gridDimZ = (depth + blockDimZ - 1) / blockDimZ;

		mKernelLauncher.launchKernelXYZPtr(PxgKernelIds::sdf_CalculateDenseGridPointCloud, gridDimX, gridDimY, gridDimZ, blockDimX, blockDimY, blockDimZ, 0, stream,
			&pointCloudBvh, &sampler, &width, &height, &depth, &sdfDataD);

		result = ccm->getCudaContext()->streamSynchronize(stream);

		//if (treeBuilder.mMaxTreeDepth[0] > 48)
		//	printf("maxDepth: %f\n", PxF64(treeBuilder.mMaxTreeDepth[0]));

		treeBuilder.releaseBVH(pointCloudBvh);
		treeBuilder.release();
	}

	result = ccm->getCudaContext()->streamSynchronize(stream);
	PX_ASSERT(result == CUDA_SUCCESS);

	PX_DEVICE_MEMORY_FREE(*ccm, atomicCounter);
}

PxReal* PxgSDFBuilder::buildDenseSDF(const PxVec3* vertices, PxU32 numVertices, const PxU32* indicesOrig, PxU32 numTriangleIndices, PxU32 width, PxU32 height, PxU32 depth,
	const PxVec3& minExtents, const PxVec3& maxExtents, bool cellCenteredSamples, CUstream stream)
{
	PxCudaContextManager* ccm = mKernelLauncher.getCudaContextManager();

	// AD: we try to get all the allocations done upfront such that we can fail early.
	PxgLinearBVHBuilderGPU treeBuilder(mKernelLauncher);

	PxgBvhTriangleMesh gpuMesh = PxgBvhTriangleMesh();
	gpuMesh.mVertices = PX_DEVICE_MEMORY_ALLOC(PxVec3, *ccm, numVertices);
	gpuMesh.mTriangles = PX_DEVICE_MEMORY_ALLOC(PxU32, *ccm, numTriangleIndices);
	gpuMesh.mNumVertices = numVertices;
	gpuMesh.mNumTriangles = numTriangleIndices / 3;

	PxgWindingClusterApproximation* windingNumberClustersD = PX_DEVICE_MEMORY_ALLOC(PxgWindingClusterApproximation, *ccm, gpuMesh.mNumTriangles);

	PxU32 numSDFSamples = width * height * depth;
	PxReal* sdfDataD = PX_DEVICE_MEMORY_ALLOC(PxReal, *ccm, numSDFSamples);

	PxReal* windingNumbersD = NULL;

	treeBuilder.allocateOrResize(gpuMesh.mBvh, gpuMesh.mNumTriangles);

	if (ccm->getCudaContext()->isInAbortMode())
	{
		PxGetFoundation().error(PxErrorCode::eABORT, PX_FL, "GPU SDF cooking failed!\n");

		PX_DEVICE_MEMORY_FREE(*ccm, gpuMesh.mVertices);
		PX_DEVICE_MEMORY_FREE(*ccm, gpuMesh.mTriangles);
		PX_DEVICE_MEMORY_FREE(*ccm, windingNumberClustersD);

		treeBuilder.releaseBVH(gpuMesh.mBvh);
		treeBuilder.release();

		PX_DEVICE_MEMORY_FREE(*ccm, sdfDataD);

		return NULL;
	}

	// allocations are done here, let's start computing.

	PxBounds3 totalBounds(minExtents, maxExtents);

	PxgCudaHelpers::copyHToDAsync(*ccm->getCudaContext(), gpuMesh.mVertices, vertices, numVertices, stream);
	PxgCudaHelpers::copyHToDAsync(*ccm->getCudaContext(), gpuMesh.mTriangles, indicesOrig, numTriangleIndices, stream);

	treeBuilder.buildTreeAndWindingClustersFromTriangles(gpuMesh.mBvh, windingNumberClustersD, gpuMesh.mVertices, gpuMesh.mTriangles, NULL, gpuMesh.mNumTriangles, &totalBounds, stream, 1e-5f, true);

	const PxVec3 extents(maxExtents - minExtents);
	const PxVec3 cellSize(extents.x / width, extents.y / height, extents.z / depth);
	Gu::GridQueryPointSampler sampler(minExtents, cellSize, cellCenteredSamples);	

#if EXTENDED_DEBUG
	bool debugWindingNumbers = false;		
	PxArray<PxReal> windingNumbers;
	if (debugWindingNumbers)
	{
		windingNumbersD = PX_DEVICE_MEMORY_ALLOC(PxReal, *ccm, (width * height * depth));
		windingNumbers.resize(width * height * depth);
	}
#endif

	computeDenseSDF(gpuMesh, windingNumberClustersD, sampler, width, height, depth, sdfDataD, stream, windingNumbersD);

#if EXTENDED_DEBUG
	if (debugWindingNumbers)
	{
		PxCUresult result = ccm->getCudaContext()->streamSynchronize(stream);
		PX_ASSERT(result == CUDA_SUCCESS);

		PxgCudaHelpers::copyDToH(*ccm, windingNumbers.begin(), windingNumbersD, width * height * depth);
			
		result = ccm->getCudaContext()->streamSynchronize(stream);
		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);

		PxReal minWinding = FLT_MAX;
		PxReal maxWinding = -FLT_MAX;
		PxReal windingClosestToZeroPointFive = FLT_MAX;

		for (PxU32 i = 0; i < windingNumbers.size(); ++i)
		{
			PxReal w = windingNumbers[i];
			minWinding = PxMin(minWinding, w);
			maxWinding = PxMax(maxWinding, w);

			PxReal diffToZeroPointFive = PxAbs(0.5f - w);
			windingClosestToZeroPointFive = PxMin(windingClosestToZeroPointFive, diffToZeroPointFive);
		}

		//printf("windingInfo: %f %f %f\n", minWinding, maxWinding, windingClosestToZeroPointFive);

		PX_DEVICE_MEMORY_FREE(*ccm, windingNumbersD);
	}
#endif

	fixHoles(width, height, depth, sdfDataD, cellSize, minExtents, maxExtents, sampler, stream);

	ccm->getCudaContext()->streamSynchronize(stream);

	PX_DEVICE_MEMORY_FREE(*ccm, gpuMesh.mVertices);
	PX_DEVICE_MEMORY_FREE(*ccm, gpuMesh.mTriangles);
	PX_DEVICE_MEMORY_FREE(*ccm, windingNumberClustersD);

	treeBuilder.releaseBVH(gpuMesh.mBvh);
	treeBuilder.release();

	if (ccm->getCudaContext()->isInAbortMode())
	{
		PxGetFoundation().error(PxErrorCode::eABORT, PX_FL, "GPU SDF cooking failed!\n");
		PX_DEVICE_MEMORY_FREE(*ccm, sdfDataD);
		return NULL;
	}

	return sdfDataD;
}

bool PxgSDFBuilder::buildSDF(const PxVec3* vertices, PxU32 numVertices, const PxU32* indicesOrig, PxU32 numTriangleIndices, PxU32 width, PxU32 height, PxU32 depth,
	const PxVec3& minExtents, const PxVec3& maxExtents, bool cellCenteredSamples, PxReal* sdf, CUstream stream)
{
	PxCudaContextManager* ccm = mKernelLauncher.getCudaContextManager();
	PxScopedCudaLock lock(*ccm);
		
	bool destroyStream = false;
	if (stream == 0)
	{
		mKernelLauncher.getCudaContextManager()->getCudaContext()->streamCreate(&stream, CU_STREAM_NON_BLOCKING);
		destroyStream = true;
	}		

	PxReal* sdfDataD = buildDenseSDF(vertices, numVertices, indicesOrig, numTriangleIndices, width, height, depth, minExtents, maxExtents, cellCenteredSamples, stream);
		
	// buildDenseSDF returns NULL if gpu cooking failed.
	if (!sdfDataD)
		return false;

	PxU32 numSDFSamples = width * height * depth;
	PxCUresult result = ccm->getCudaContext()->streamSynchronize(stream);
	PX_ASSERT(result == CUDA_SUCCESS);
	PxgCudaHelpers::copyDToH(*ccm->getCudaContext(), sdf, sdfDataD, numSDFSamples);
	result = ccm->getCudaContext()->streamSynchronize(stream);
	PX_ASSERT(result == CUDA_SUCCESS);

	PX_DEVICE_MEMORY_FREE(*ccm, sdfDataD);

	if (destroyStream) 
	{
		mKernelLauncher.getCudaContextManager()->getCudaContext()->streamDestroy(stream);
	}

	return true;
}

void PxgSDFBuilder::release()
{
	PX_DELETE_THIS;
}

bool PxgSDFBuilder::buildSparseSDF(const PxVec3* vertices, PxU32 numVertices, const PxU32* indicesOrig,
	PxU32 numTriangleIndices, PxU32 width, PxU32 height, PxU32 depth,
	const PxVec3& minExtents, const PxVec3& maxExtents, PxReal narrowBandThickness, PxU32 subgridSize, PxSdfBitsPerSubgridPixel::Enum bytesPerSubgridPixel,
	PxArray<PxReal>& sdfCoarse, PxArray<PxU32>& sdfSubgridsStartSlots, PxArray<PxU8>& sdfDataSubgrids,
	PxReal& subgridsMinSdfValue, PxReal& subgridsMaxSdfValue, 
	PxU32& sdfSubgrids3DTexBlockDimX, PxU32& sdfSubgrids3DTexBlockDimY, PxU32& sdfSubgrids3DTexBlockDimZ, CUstream stream)
{
	if (mKernelLauncher.getCudaContextManager()->tryAcquireContext())
	{
		bool success = true;
		bool destroyStream = false;
		if (stream == 0)
		{
			mKernelLauncher.getCudaContextManager()->getCudaContext()->streamCreate(&stream, CU_STREAM_NON_BLOCKING);
			destroyStream = true;
		}
		PX_ASSERT(width % subgridSize == 0);
		PX_ASSERT(height % subgridSize == 0);
		PX_ASSERT(depth % subgridSize == 0);

		const PxVec3 extents(maxExtents - minExtents);
		const PxVec3 delta(extents.x / width, extents.y / height, extents.z / depth);

		PxReal* denseSdfD = buildDenseSDF(vertices, numVertices, indicesOrig, numTriangleIndices, width + 1, height + 1, depth + 1, minExtents, maxExtents + delta, false, stream);

		const PxReal errorThreshold = 1e-6f * extents.magnitude();

		if (denseSdfD)
		{
			compressSDF(denseSdfD, width, height, depth, subgridSize, narrowBandThickness, bytesPerSubgridPixel, errorThreshold,
				subgridsMinSdfValue, subgridsMaxSdfValue, sdfCoarse, sdfSubgridsStartSlots, sdfDataSubgrids,
				sdfSubgrids3DTexBlockDimX, sdfSubgrids3DTexBlockDimY, sdfSubgrids3DTexBlockDimZ, stream);

			PX_DEVICE_MEMORY_FREE(*mKernelLauncher.getCudaContextManager(), denseSdfD);

			if (!sdfCoarse.size())
			{
				PxGetFoundation().error(PxErrorCode::eABORT, PX_FL, "GPU SDF cooking failed!\n");
				success = false;
			}
		}
		else
			success = false;

		if (destroyStream)
		{
			mKernelLauncher.getCudaContextManager()->getCudaContext()->streamDestroy(stream);
		}

		mKernelLauncher.getCudaContextManager()->releaseContext();
		return success;
	}
	return false;
}

bool PxgSDFBuilder::allocateBuffersForCompression(
	PxReal*& backgroundSdfD,
	PxU32 numBackgroundSdfSamples,
	PxU32*& subgridAddressesD,
	PxU8*& subgridActiveD,
	PxU32 numAddressEntries,
	PxReal*& subgridGlobalMinValueD,
	PxReal*& subgridGlobalMaxValueD,
	PxGpuScan& scan)
{
	PxCudaContextManager* ccm = mKernelLauncher.getCudaContextManager();

	backgroundSdfD = PX_DEVICE_MEMORY_ALLOC(PxReal, *ccm, numBackgroundSdfSamples);

	subgridAddressesD = PX_DEVICE_MEMORY_ALLOC(PxU32, *ccm, numAddressEntries);
	subgridActiveD = PX_DEVICE_MEMORY_ALLOC(PxU8, *ccm, numAddressEntries);

	subgridGlobalMinValueD = PX_DEVICE_MEMORY_ALLOC(PxReal, *ccm, 1);
	subgridGlobalMaxValueD = PX_DEVICE_MEMORY_ALLOC(PxReal, *ccm, 1);

	scan.initialize(&mKernelLauncher, numAddressEntries);

	if (ccm->getCudaContext()->isInAbortMode())
	{
		return false;
	}

	return true;
}
	
void PxgSDFBuilder::releaseBuffersForCompression(
	PxReal*& backgroundSdfD,
	PxU32*& subgridAddressesD,
	PxU8*& subgridActiveD,
	PxReal*& subgridGlobalMinValueD,
	PxReal*& subgridGlobalMaxValueD,
	PxGpuScan& scan)
{
	PxCudaContextManager* ccm = mKernelLauncher.getCudaContextManager();

	PX_DEVICE_MEMORY_FREE(*ccm, backgroundSdfD);
	PX_DEVICE_MEMORY_FREE(*ccm, subgridAddressesD);
	PX_DEVICE_MEMORY_FREE(*ccm, subgridActiveD);
	PX_DEVICE_MEMORY_FREE(*ccm, subgridGlobalMinValueD);
	PX_DEVICE_MEMORY_FREE(*ccm, subgridGlobalMaxValueD);
	scan.release();
}

void PxgSDFBuilder::compressSDF(PxReal* denseSdfD, PxU32 width, PxU32 height, PxU32 depth, 
	PxU32 cellsPerSubgrid, PxReal narrowBandThickness, PxU32 bytesPerSubgridPixel, PxReal errorThreshold, 
	PxReal& subgridGlobalMinValue, PxReal& subgridGlobalMaxValue, PxArray<PxReal>& sdfCoarse, 
	PxArray<PxU32>& sdfSubgridsStartSlots, PxArray<PxU8>& sdfDataSubgrids,
	PxU32& sdfSubgrids3DTexBlockDimX, PxU32& sdfSubgrids3DTexBlockDimY, PxU32& sdfSubgrids3DTexBlockDimZ, CUstream stream)
{
	// allocations upfront
	PxReal* backgroundSdfD = NULL;
	PxU32* subgridAddressesD = NULL;
	PxU8* subgridActiveD = NULL;
	PxReal* subgridGlobalMinValueD = NULL;
	PxReal* subgridGlobalMaxValueD = NULL;
	PxGpuScan scan;

	const PxU32 w = width / cellsPerSubgrid;
	const PxU32 h = height / cellsPerSubgrid;
	const PxU32 d = depth / cellsPerSubgrid;

	PX_ASSERT(width % cellsPerSubgrid == 0);
	PX_ASSERT(height % cellsPerSubgrid == 0);
	PX_ASSERT(depth % cellsPerSubgrid == 0);

	const PxU32 backgroundSizeX = w + 1;
	const PxU32 backgroundSizeY = h + 1;
	const PxU32 backgroundSizeZ = d + 1;
	const PxU32 numBackgroundSdfSamples = backgroundSizeX * backgroundSizeY * backgroundSizeZ;

	PxU32 numAddressEntries = w * h * d;

	bool success = allocateBuffersForCompression(backgroundSdfD, numBackgroundSdfSamples, subgridAddressesD, subgridActiveD, numAddressEntries, subgridGlobalMinValueD, subgridGlobalMaxValueD, scan);

	if (!success)
	{
		releaseBuffersForCompression(backgroundSdfD, subgridAddressesD, subgridActiveD, subgridGlobalMinValueD, subgridGlobalMaxValueD, scan);
		sdfCoarse.forceSize_Unsafe(0); // this signals that there is no SDF.
		return;
	}

	// then the actual computation
	PxCudaContextManager* ccm = mKernelLauncher.getCudaContextManager();		

	PxReal val = FLT_MAX;
	PxgCudaHelpers::memsetAsync(*ccm->getCudaContext(), subgridGlobalMinValueD, val, 1, stream);
	val = -FLT_MAX;		
	PxgCudaHelpers::memsetAsync(*ccm->getCudaContext(), subgridGlobalMaxValueD, val, 1, stream);
		
	const PxU32 numItems = backgroundSizeX * backgroundSizeY * backgroundSizeZ;
	const PxU32 kNumThreadsPerBlock = PxgBVHKernelBlockDim::BUILD_SDF;
	const PxU32 kNumBlocks = (numItems + kNumThreadsPerBlock - 1) / kNumThreadsPerBlock;

	mKernelLauncher.launchKernelPtr(PxgKernelIds::sdf_PopulateBackgroundSDF, kNumBlocks, kNumThreadsPerBlock, 0, stream,
		&cellsPerSubgrid, &backgroundSdfD, &backgroundSizeX, &backgroundSizeY, &backgroundSizeZ,
		&denseSdfD, &width, &height, &depth);
		
	mKernelLauncher.launchKernelXYZPtr(PxgKernelIds::sdf_MarkRequiredSdfSubgrids, w, h, d, kNumThreadsPerBlock, 1, 1, 0, stream,
		&backgroundSdfD, &denseSdfD, &subgridAddressesD, &subgridActiveD, &cellsPerSubgrid, &width, &height, &depth, &backgroundSizeX, &backgroundSizeY, &backgroundSizeZ, &narrowBandThickness,
		&subgridGlobalMinValueD, &subgridGlobalMaxValueD, &errorThreshold);
		
	scan.exclusiveScan(subgridAddressesD, stream);

	PxU32 numSubgrids = 0;
	PxCUresult result = ccm->getCudaContext()->streamSynchronize(stream);
	PX_ASSERT(result == CUDA_SUCCESS);
	PxgCudaHelpers::copyDToH(*ccm->getCudaContext(), &numSubgrids, scan.getSumPointer(), 1);
	PxgCudaHelpers::copyDToH(*ccm->getCudaContext(), &subgridGlobalMinValue, subgridGlobalMinValueD, 1);
	PxgCudaHelpers::copyDToH(*ccm->getCudaContext(), &subgridGlobalMaxValue, subgridGlobalMaxValueD, 1);

	//Synchronize the stream because the size of following memory allocations depends on calculations done in previously ran kernels
	result = ccm->getCudaContext()->streamSynchronize(stream);
	PX_ASSERT(result == CUDA_SUCCESS);

	const PxU32 valuesPerSubgrid = (cellsPerSubgrid + 1)*(cellsPerSubgrid + 1)*(cellsPerSubgrid + 1);

	const PxReal cubicRoot = PxPow(PxReal(numSubgrids), 1.0f / 3.0f);
	const PxU32 up = PxMax(1u, PxU32(PxCeil(cubicRoot)));

	//Arrange numSubgrids in a 3d layout
	PxU32 n = numSubgrids;
	sdfSubgrids3DTexBlockDimX = PxMin(up, n);
	n = (n + up - 1) / up;
	sdfSubgrids3DTexBlockDimY = PxMin(up, n);
	n = (n + up - 1) / up;
	sdfSubgrids3DTexBlockDimZ = PxMin(up, n);		
	PxU32 subgridDataSize = valuesPerSubgrid * sdfSubgrids3DTexBlockDimX * sdfSubgrids3DTexBlockDimY * sdfSubgrids3DTexBlockDimZ * bytesPerSubgridPixel;

	//if (sdfSubgrids3DTexBlockDimX*sdfSubgrids3DTexBlockDimY*sdfSubgrids3DTexBlockDimZ < numSubgrids)
	//	printf("3D subgrid texture too small\n");

	//if (subgridDataSize == 0)
	//	printf("subgridDataSize is zero %i %i %i %i %i %i %f %f\n", numSubgrids, valuesPerSubgrid, sdfSubgrids3DTexBlockDimX, sdfSubgrids3DTexBlockDimY, sdfSubgrids3DTexBlockDimZ, PxU32(bytesPerSubgridPixel), subgridGlobalMinValue, subgridGlobalMaxValue);

	// we cannot put that one to the front because it depends on data we calculate just above.
	PxU32* quantizedSparseSDFIn3DTextureFormatD = PX_DEVICE_MEMORY_ALLOC(PxU32, *ccm, (subgridDataSize + 3)/4);
	if (!quantizedSparseSDFIn3DTextureFormatD)
	{
		releaseBuffersForCompression(backgroundSdfD, subgridAddressesD, subgridActiveD, subgridGlobalMinValueD, subgridGlobalMaxValueD, scan);
		sdfCoarse.forceSize_Unsafe(0);
		return;
	}
		
	mKernelLauncher.launchKernelXYZPtr(PxgKernelIds::sdf_PopulateSdfSubgrids, w, h, d, kNumThreadsPerBlock, 1, 1, 0, stream,
		&denseSdfD, &width, &height, &depth, &subgridAddressesD, &subgridActiveD, &cellsPerSubgrid, &w, &h, &d,
		&quantizedSparseSDFIn3DTextureFormatD, &sdfSubgrids3DTexBlockDimX, &sdfSubgrids3DTexBlockDimY, &sdfSubgrids3DTexBlockDimZ,
		&subgridGlobalMinValueD, &subgridGlobalMaxValueD, &bytesPerSubgridPixel, &subgridDataSize);

	sdfCoarse.resize(numBackgroundSdfSamples);
	sdfSubgridsStartSlots.resize(numAddressEntries);
	sdfDataSubgrids.resize(subgridDataSize);
		
	result = ccm->getCudaContext()->streamSynchronize(stream);
	PX_ASSERT(result == CUDA_SUCCESS);
	PxgCudaHelpers::copyDToH(*ccm->getCudaContext(), sdfCoarse.begin(), backgroundSdfD, numBackgroundSdfSamples);
	PxgCudaHelpers::copyDToH(*ccm->getCudaContext(), sdfSubgridsStartSlots.begin(), subgridAddressesD, numAddressEntries);
	PxgCudaHelpers::copyDToH(*ccm->getCudaContext(), sdfDataSubgrids.begin(), reinterpret_cast<PxU8*>(quantizedSparseSDFIn3DTextureFormatD), subgridDataSize);
	result = ccm->getCudaContext()->streamSynchronize(stream);
	PX_ASSERT(result == CUDA_SUCCESS);

	releaseBuffersForCompression(backgroundSdfD, subgridAddressesD, subgridActiveD, subgridGlobalMinValueD, subgridGlobalMaxValueD, scan);
	PX_DEVICE_MEMORY_FREE(*ccm, quantizedSparseSDFIn3DTextureFormatD);

	if (bytesPerSubgridPixel == 4)
	{
		//32bit values are stored as normal floats while 16bit and 8bit values are scaled to 0...1 range and then scaled back to original range
		subgridGlobalMinValue = 0.0f;
		subgridGlobalMaxValue = 1.0f;
	}
}
