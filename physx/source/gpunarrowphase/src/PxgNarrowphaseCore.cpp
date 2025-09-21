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

#include "PxNodeIndex.h"
#include "PxgCudaMemoryAllocator.h"
#include "PxsHeapMemoryAllocator.h"
#include "PxsContactManagerState.h"
#include "common/PxProfileZone.h"
#include "cudaNpCommon.h"
#include "foundation/PxAssert.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxFoundation.h"
#include "foundation/PxMemory.h"
#include "geometry/PxHeightFieldSample.h"
#include "PxSceneDesc.h"			// for PxGpuDynamicsMemoryConfig

#include "foundation/PxSort.h"

#include "GuBV32.h"
#include "convex/GuConvexMesh.h"
#include "mesh/GuTriangleMesh.h"
#include "mesh/GuTetrahedronMesh.h"
#include "hf/GuHeightField.h"
#include "GuSDF.h"
#include "GuConvexGeometry.h"

#include "CmFlushPool.h"
#include "CmTask.h"

#include "CudaKernelWrangler.h"

#include "PxsContactManager.h"
#include "PxsTransformCache.h"
#include "PxsMaterialCore.h"
#include "PxsDeformableSurfaceMaterialCore.h"
#include "PxsDeformableVolumeMaterialCore.h"
#include "PxsPBDMaterialCore.h"
#include "PxsContext.h"
#include "PxsPartitionEdge.h"

#include "PxvDynamics.h"			// for PxsBodyCore
#include "PxvGeometry.h"			// for PxsShapeCore

#include "convexNpCommon.h"
#include "PxgNarrowphaseCore.h"
#include "PxgKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "PxgContactsDebug.h"
#include "PxgCudaUtils.h"
#include "PxgNpKernelIndices.h"
#include "PxgNphaseImplementationContext.h"
#include "PxgSimulationCore.h"
#include "PxgParticleSystemCore.h"
#include "PxgParticleSystemCoreKernelIndices.h"
#include "PxgSoftBodyCoreKernelIndices.h"
#include "PxgCudaBroadPhaseSap.h"
#include "PxgParticleSystem.h"
#include "PxgSoftBodyCore.h"
#include "PxgSoftBody.h"
#include "PxgFEMClothCore.h"
#include "PxgFEMCloth.h"
#include "PxContact.h"
#include "PxgContext.h"

#include "PxgSolverCore.h"
#include "PxgSimulationController.h"

#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"
#include "PxgRadixSortKernelIndices.h"

#define GPU_NP_DEBUG 0
#define GPU_NP_DEBUG_VERBOSE 0
#define GPU_NP_VISUALIZATION 0

using namespace physx;
using namespace Gu;

PxgGpuNarrowphaseCore::PxgGpuNarrowphaseCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager, const PxGpuDynamicsMemoryConfig& gpuDynamicsConfig,
	void* contactStreamBase, void* patchStreamBase, void* forceAndIndiceStreamBase, IG::IslandSim* islandSim, CUstream solverStream, PxgHeapMemoryAllocatorManager* heapMemoryManager,
	PxgNphaseImplementationContext* nphaseImplContext) :
	mPairManagementBuffers(heapMemoryManager),
	mGpuTransformCache(heapMemoryManager, PxsHeapStats::eNARROWPHASE),
	mGpuContactDistance(heapMemoryManager, PxsHeapStats::eNARROWPHASE),
	mLostFoundPairsOutputData(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
	mLostFoundPairsCms(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
	mGpuPairManagementData(heapMemoryManager, PxsHeapStats::eNARROWPHASE),
	mRSDesc(heapMemoryManager->mMappedMemoryAllocators),
	mRadixSortDescBuf(heapMemoryManager, PxsHeapStats::eNARROWPHASE),
	mTempGpuRigidIndiceBuf(heapMemoryManager, PxsHeapStats::eNARROWPHASE),
	mTempGpuShapeIndiceBuf(heapMemoryManager, PxsHeapStats::eNARROWPHASE),
	mRadixCountTotalBuf(heapMemoryManager, PxsHeapStats::eNARROWPHASE),
	mPatchAndContactCountersOnDevice(heapMemoryManager, PxsHeapStats::eNARROWPHASE),
	mPatchAndContactCountersReadback(NULL),
	mGpuShapesManager(heapMemoryManager),
	mGpuMaterialManager(heapMemoryManager),
	mGpuFEMMaterialManager(heapMemoryManager),
	mGpuFEMClothMaterialManager(heapMemoryManager),
	mGpuPBDMaterialManager(heapMemoryManager),
	mIntermStackAlloc(*heapMemoryManager->mDeviceMemoryAllocators, gpuDynamicsConfig.collisionStackSize),
	mSolverStream(solverStream),
	mGpuKernelWranglerManager(gpuKernelWrangler),
	mCudaContextManager(cudaContextManager),
	mCudaContext(cudaContextManager->getCudaContext()),
	mHeapMemoryManager(heapMemoryManager),
	mCopyMan(heapMemoryManager),
	mCopyManBp(heapMemoryManager),
	mGeometryManager(heapMemoryManager),
	mIslandSim(islandSim),
	mNphaseImplContext(nphaseImplContext),
	mGpuMultiManifold(heapMemoryManager, PxsHeapStats::eNARROWPHASE),
	mGpuManifold(heapMemoryManager, PxsHeapStats::eNARROWPHASE)
#if PX_ENABLE_SIM_STATS
	, mGpuDynamicsRigidContactCountStats(0),
	mGpuDynamicsRigidPatchCountStats(0),
	mGpuDynamicsCollisionStackSizeStats(0)
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
{	
	mTotalLostFoundPatches = 0;
	mTotalLostFoundPairs = 0;
	mTotalNumPairs = 0;

	for (PxU32 i = 0; i < GPU_BUCKET_ID::eCount; ++i)
	{
		mRemovedIndices[i] = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(RemovedIndicesArray), "RemovedIndicesArray"), RemovedIndicesArray(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)));

		mContactManagers[i] = PX_NEW(PxgMirroredContactManagerPair)(i, PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators));
		mGpuContactManagers[i] = PX_NEW(PxgGpuContactManagerPair)(i, heapMemoryManager);
	}

	PxgPersistentContactManifold emptyManifold;
	emptyManifold.clear();
	PxgPersistentContactMultiManifold emptyMultiManifold;
	emptyMultiManifold.clear();

	mShapesMap = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(RefcountedRecordsMap), "ShapeMap"), RefcountedRecordsMap());
	mGeometriesMap = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(RefcountedRecordsMap), "GeometriesMap"), RefcountedRecordsMap());
	mMaterialsMap = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(RefcountedRecordsMap), "MaterialsMap"), RefcountedRecordsMap());
	mFEMMaterialsMap = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(RefcountedRecordsMap), "FEMMaterialsMap"), RefcountedRecordsMap());
	mFEMClothMaterialsMap = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(RefcountedRecordsMap), "FEMClothMaterialsMap"), RefcountedRecordsMap());
	mPBDMaterialsMap = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(RefcountedRecordsMap), "PBDMaterialsMap"), RefcountedRecordsMap());
	PX_UNUSED(gpuDynamicsConfig);
	mCudaContextManager->acquireContext();

	mGpuMultiManifold.allocate(sizeof(PxgPersistentContactMultiManifold), PX_FL);
	mGpuManifold.allocate(sizeof(PxgPersistentContactManifold), PX_FL);

	mCudaContext->memcpyHtoD(mGpuMultiManifold.getDevicePtr(), &emptyMultiManifold, sizeof(PxgPersistentContactMultiManifold));
	mCudaContext->memcpyHtoD(mGpuManifold.getDevicePtr(), &emptyManifold, sizeof(PxgPersistentContactManifold));

	mCopyMan.createFinishedEvent(mCudaContext);
	mCopyManBp.createFinishedEvent(mCudaContext);

	mGpuPairManagementData.allocate(sizeof(PxgPairManagementData), PX_FL);

	mCollisionStackSizeBytes = gpuDynamicsConfig.collisionStackSize;

	// allocated in GpuDynamicsContext.
	mContactStream = reinterpret_cast<CUdeviceptr>(contactStreamBase);
	mPatchStream = reinterpret_cast<CUdeviceptr>(patchStreamBase);
	mForceAndIndiceStream = reinterpret_cast<CUdeviceptr>(forceAndIndiceStreamBase);

	PxU32 blockAccumArraySize = PxgNarrowPhaseGridDims::COMPACT_LOST_FOUND_PAIRS;

	for (PxU32 i = GPU_BUCKET_ID::eConvex; i <= GPU_BUCKET_ID::eTriangleTriangle; ++i)
	{
		mGpuContactManagers[GPU_BUCKET_ID::Enum(i)]->allocateBlockAccumulationArray(blockAccumArraySize);
	}

	mPairManagementBuffers.mBlockAccumulationArray.allocate(sizeof(PxU32) * PxgNarrowPhaseGridDims::REMOVE_CONTACT_MANAGERS, PX_FL);

	for (PxU32 i = GPU_BUCKET_ID::eConvex; i < GPU_BUCKET_ID::eCount; ++i)
	{
		mGpuContactManagers[i]->allocateLostAndTotalReportedPairsCount(mHeapMemoryManager->mMappedMemoryAllocators);
		mPairManagementData[i] = reinterpret_cast<PxgPairManagementData*>(mHeapMemoryManager->mMappedMemoryAllocators->allocate(sizeof(PxgPairManagementData), PxsHeapStats::eNARROWPHASE, PX_FL));
	}

	mPatchAndContactCountersOnDevice.allocateElements(1, PX_FL);
	mPatchAndContactCountersReadback = reinterpret_cast<PxgPatchAndContactCounters*>(mHeapMemoryManager->mMappedMemoryAllocators->allocate(sizeof(PxgPatchAndContactCounters), PxsHeapStats::eNARROWPHASE, PX_FL));

	mGeometryManager.addBoxHull();

	mRadixCountTotalBuf.allocate(sizeof(PxU32) * PxgRadixSortKernelGridDim::RADIX_SORT * 16, PX_FL);

	mRSDesc.resize(2u);

	for (PxU32 i = 0; i < 2; ++i)
	{
		mRadixSortDescBuf[i].allocate(sizeof(PxgRadixSortDesc), PX_FL);
	}

	createGpuStreamsAndEvents();

	mGpuShapesManager.initialize(mCudaContext, mStream);

	mMaxConvexMeshTempMemory = reinterpret_cast<PxU32*>(heapMemoryManager->mMappedMemoryAllocators->allocate(sizeof(PxU32), PxsHeapStats::eNARROWPHASE, PX_FL));
	*mMaxConvexMeshTempMemory = 0;

	cudaContextManager->releaseContext();
}

PxgGpuNarrowphaseCore::~PxgGpuNarrowphaseCore()
{
	mCudaContextManager->acquireContext();

	mCopyMan.destroyFinishedEvent(mCudaContext);
	mCopyManBp.destroyFinishedEvent(mCudaContext);

	for (PxU32 i = 0; i < GPU_BUCKET_ID::eCount; ++i)
	{
		if (i >= GPU_BUCKET_ID::eConvex)
		{
			if (mPairManagementData[i])
				mHeapMemoryManager->mMappedMemoryAllocators->deallocate(mPairManagementData[i]);

			if (mGpuContactManagers[i]->mContactManagers.mLostAndTotalReportedPairsCountPinned)
				mHeapMemoryManager->mMappedMemoryAllocators->deallocate(mGpuContactManagers[i]->mContactManagers.mLostAndTotalReportedPairsCountPinned);

			if (mGpuContactManagers[i]->mNewContactManagers.mLostAndTotalReportedPairsCountPinned)
				mHeapMemoryManager->mMappedMemoryAllocators->deallocate(mGpuContactManagers[i]->mNewContactManagers.mLostAndTotalReportedPairsCountPinned);
		}

		mRemovedIndices[i]->~RemovedIndicesArray();
		PX_FREE(mRemovedIndices[i]);

		PX_DELETE(mContactManagers[i]);
		PX_DELETE(mGpuContactManagers[i]);
	}

	mHeapMemoryManager->mMappedMemoryAllocators->deallocate(mMaxConvexMeshTempMemory);
	mHeapMemoryManager->mMappedMemoryAllocators->deallocate(mPatchAndContactCountersReadback);

	mCudaContextManager->releaseContext();

	releaseGpuStreamsAndEvents();
	mShapesMap->~RefcountedRecordsMap();
	mGeometriesMap->~RefcountedRecordsMap();
	mMaterialsMap->~RefcountedRecordsMap();
	mFEMMaterialsMap->~RefcountedRecordsMap();
	mFEMClothMaterialsMap->~RefcountedRecordsMap();
	mPBDMaterialsMap->~RefcountedRecordsMap();

	PX_FREE(mShapesMap);
	PX_FREE(mGeometriesMap);
	PX_FREE(mMaterialsMap);
	PX_FREE(mFEMMaterialsMap);
	PX_FREE(mFEMClothMaterialsMap);
	PX_FREE(mPBDMaterialsMap);
}

void test(PxCudaContextManager* cudaContextManager, KernelWrangler*	kernelWrangler); //DBGFD

void PxgGpuNarrowphaseCore::drawPoint(PxRenderOutput& out, PxVec3 a, const PxU32 color, const PxF32 size)
{
	const PxVec3 up(0.f, size, 0.f);
	const PxVec3 right(size, 0.f, 0.f);
	const PxVec3 forwards(0.f, 0.f, size);

	const PxMat44 m(PxIdentity);

	out << color << m << PxRenderOutput::LINES << a + up << a - up;
	out << color << m << PxRenderOutput::LINES << a + right << a - right;
	out << color << m << PxRenderOutput::LINES << a + forwards << a - forwards;
}

void PxgGpuNarrowphaseCore::drawLine(PxRenderOutput& out, PxVec3 a, const PxVec3 b, const PxU32 color)
{
	const PxMat44 m(PxIdentity);
	out << color << m << PxRenderOutput::LINES << a << b;
}

void PxgGpuNarrowphaseCore::drawManifold(PxgPersistentContactManifold* manifolds, PxgContactManagerInput* cmInput, PxsCachedTransform* cachedTransform, const PxU32 numTests, 
	PxRenderOutput& renderOutput, const PxU32 color, const PxF32 size)
{

	for (PxU32 i = 0; i < numTests; ++i)
	{
		PxgContactManagerInput& input = cmInput[i];
		PxsCachedTransform& transform0 = cachedTransform[input.transformCacheRef0];
		PxsCachedTransform& transform1 = cachedTransform[input.transformCacheRef1];
		PxgPersistentContactManifold& manifold = manifolds[i];

		const PxU32 nbContacts = (manifold.mNbContacts & (~0x80000000));

		PX_ASSERT(nbContacts <= 4);

		for (PxU32 j = 0; j < nbContacts; ++j)
		{
			PxVec3 localA(manifold.mLocalContactA[j].x, manifold.mLocalContactA[j].y, manifold.mLocalContactA[j].z);
			PxVec3 localB(manifold.mLocalContactB[j].x, manifold.mLocalContactB[j].y, manifold.mLocalContactB[j].z);
		
			PxVec3 worldA = transform0.transform.transform(localA);
			PxVec3 worldB = transform1.transform.transform(localB);

			drawPoint(renderOutput, worldA, color, size);
			drawPoint(renderOutput, worldB, color, size);
			drawLine(renderOutput, worldA, worldB, color);
		}
	}
}

void PxgGpuNarrowphaseCore::compactLostFoundPairs(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxU32* touchChangeFlags, PxsContactManagerOutput* cmOutputs)
{
	CUresult result;

	PxU32* tempRunsum = (PxU32*)gpuManagers.mTempRunsumArray2.getDevicePtr();
	PxsContactManagerOutputCounts* lostFoundOutputs = (PxsContactManagerOutputCounts*)gpuManagers.mLostFoundPairsOutputData.getDevicePtr();
	PxsContactManager** lostFoundCms = (PxsContactManager**)gpuManagers.mLostFoundPairsCms.getDevicePtr();
	PxU32* blockAccumArray = (PxU32*)gpuManagers.mBlockAccumulationArray.getDevicePtr();
	uint2* lostAndTotalReportedPairsCount = reinterpret_cast<uint2*>(getMappedDevicePtr(mCudaContext, gpuManagers.mLostAndTotalReportedPairsCountPinned));
	PxsContactManager** cmArray = (PxsContactManager**)gpuManagers.mCpuContactManagerMapping.getDevicePtr();

	{
		CUfunction kernelFunction1 = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::COMPACT_LOST_FOUND_PAIRS_1);
		CUfunction kernelFunction2 = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::COMPACT_LOST_FOUND_PAIRS_2);

		PxCudaKernelParam kernelParams1[] =
		{
			PX_CUDA_KERNEL_PARAM(touchChangeFlags),
			PX_CUDA_KERNEL_PARAM(tempRunsum),
			PX_CUDA_KERNEL_PARAM(blockAccumArray),
			PX_CUDA_KERNEL_PARAM(numTests)
		};

		PxCudaKernelParam kernelParams2[] =
		{
			PX_CUDA_KERNEL_PARAM(touchChangeFlags),
			PX_CUDA_KERNEL_PARAM(tempRunsum),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(lostFoundOutputs),
			PX_CUDA_KERNEL_PARAM(lostFoundCms),
			PX_CUDA_KERNEL_PARAM(lostAndTotalReportedPairsCount),
			PX_CUDA_KERNEL_PARAM(blockAccumArray),
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(cmArray)
		};

		result = mCudaContext->launchKernel(kernelFunction1, PxgNarrowPhaseGridDims::COMPACT_LOST_FOUND_PAIRS, 1, 1,
			WARP_SIZE, PxgNarrowPhaseBlockDims::COMPACT_LOST_FOUND_PAIRS / WARP_SIZE, 1,
			0, mStream, kernelParams1, sizeof(kernelParams1), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU prepareLostFoundPairs_Stage1 fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU prepareLostFoundPairs_Stage1 kernel fail!!!\n");
#endif
		result = mCudaContext->launchKernel(kernelFunction2, PxgNarrowPhaseGridDims::COMPACT_LOST_FOUND_PAIRS, 1, 1,
			WARP_SIZE, PxgNarrowPhaseBlockDims::COMPACT_LOST_FOUND_PAIRS / WARP_SIZE, 1,
			0, mStream, kernelParams2, sizeof(kernelParams2), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU prepareLostFoundPairs_Stage2 fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU prepareLostFoundPairs_Stage2 kernel fail!!!\n");
#endif
	}
}

void PxgGpuNarrowphaseCore::testSDKSphereGpu(
	PxgGpuContactManagers& gpuManagers, 
	const PxU32 numTests,
	PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
	PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit,
	PxRenderOutput* renderOutput)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKSphereGpu", 0);

	PX_UNUSED(renderOutput);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PxsContactManagerOutput* cmOutputs = reinterpret_cast<PxsContactManagerOutput*>(gpuManagers.mContactManagerOutputData.getDevicePtr());

	CUdeviceptr gpuShapes = mGpuShapesManager.mGpuShapesBuffer.getDevicePtr();
	PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	PxReal*	contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	PX_ASSERT(transformCache);
	PxsMaterialData* materials = reinterpret_cast<PxsMaterialData*>(mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());
	PxU32* touchChangeFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr());
	PxU32* patchChangeFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr() + sizeof(PxU32) * numTests);
	CUresult result;

	PxgDevicePointer<PxgPatchAndContactCounters> patchAndContactCountersD = mPatchAndContactCountersOnDevice.getTypedDevicePtr();

	{
		CUfunction sphereKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SPHERE_KERNEL_MAIN);

		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(materials),
			PX_CUDA_KERNEL_PARAM(mContactStream),
			PX_CUDA_KERNEL_PARAM(mPatchStream),
			PX_CUDA_KERNEL_PARAM(patchAndContactCountersD),
			PX_CUDA_KERNEL_PARAM(touchChangeFlags),
			PX_CUDA_KERNEL_PARAM(patchChangeFlags),
			PX_CUDA_KERNEL_PARAM(baseContactPatches),
			PX_CUDA_KERNEL_PARAM(baseContactPoints),
			PX_CUDA_KERNEL_PARAM(baseContactForces),
			PX_CUDA_KERNEL_PARAM(patchBytesLimit),
			PX_CUDA_KERNEL_PARAM(contactBytesLimit),
			PX_CUDA_KERNEL_PARAM(forceBytesLimit)
		};

		const PxU32 numThreadsPerBlock = 64;
		const PxU32 numBlocks = (numTests + (numThreadsPerBlock - 1)) / numThreadsPerBlock;
		//Each thread do one collision detection
		result = mCudaContext->launchKernel(sphereKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sphereNphase_Kernel fail to launch kernel stage 3!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cudaMainGjkEpa error in kernel stage 3!!!\n");
#endif

#if GPU_NP_VISUALIZATION
		result = mCudaContext->streamSynchronize(mStream);
		
		PxArray<PxgContactManagerInput> tCmInput(numTests);
		PxArray<PxsContactManagerOutput> tCmOutput(numTests);
		PxArray<PxsCachedTransform> tCachedTransform(PxU32((mGpuTransformCache.getSize() / sizeof(PxsCachedTransform)) + 1));
		PxArray<PxContactPatch> tPatch(numTests);
		PxArray<PxContact> tContact(numTests);
		mCudaContext->memcpyDtoH(tCmInput.begin(), gpuManagers.mContactManagerInputData.getDevicePtr(), sizeof(PxgContactManagerInput) * numTests);
		mCudaContext->memcpyDtoH(tCachedTransform.begin(), mGpuTransformCache.getDevicePtr(), mGpuTransformCache.getSize());
		mCudaContext->memcpyDtoH(tCmOutput.begin(), gpuManagers.mContactManagerOutputData.getDevicePtr(), sizeof(PxsContactManagerOutput) * numTests);
		mCudaContext->memcpyDtoH(tPatch.begin(), mPatchStream, sizeof(PxContactPatch) * numTests);
		mCudaContext->memcpyDtoH(tContact.begin(), mContactStream, sizeof(PxContact) * numTests);

		int bob = 0;
		PX_UNUSED(bob);
		//drawManifold(tManifold.begin(), tCmInput.begin(), tCachedTransform.begin(), numTests, *renderOutput, 0xffffff, 0.05f);

#endif
	}

	compactLostFoundPairs(gpuManagers, numTests, touchChangeFlags, cmOutputs);
}

void PxgGpuNarrowphaseCore::testSDKBoxBoxGpu(
	PxgGpuContactManagers& gpuManagers,
	const PxU32 numTests,
	PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
	PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit,
	PxRenderOutput* renderOutput)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKBoxBoxGpu", 0);

	PX_UNUSED(renderOutput);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PxsContactManagerOutput* cmOutputs = reinterpret_cast<PxsContactManagerOutput*>(gpuManagers.mContactManagerOutputData.getDevicePtr());

	CUdeviceptr gpuShapes = mGpuShapesManager.mGpuShapesBuffer.getDevicePtr();
	PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	PxReal*	contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	PX_ASSERT(transformCache);
	PxsMaterialData* materials = reinterpret_cast<PxsMaterialData*>(mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());
	PxU32* touchLostFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr());
	PxU32* touchFoundFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr() + sizeof(PxU32) * numTests);
	PxgDevicePointer<PxgPatchAndContactCounters> patchAndContactCountersD = mPatchAndContactCountersOnDevice.getTypedDevicePtr();

	CUresult result;

	{
		CUfunction sphereKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::BOX_BOX_KERNEL_MAIN);

		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(materials),
			PX_CUDA_KERNEL_PARAM(mContactStream),
			PX_CUDA_KERNEL_PARAM(mPatchStream),
			PX_CUDA_KERNEL_PARAM(patchAndContactCountersD),
			PX_CUDA_KERNEL_PARAM((touchLostFlags)),
			PX_CUDA_KERNEL_PARAM((touchFoundFlags)),
			PX_CUDA_KERNEL_PARAM((baseContactPatches)),
			PX_CUDA_KERNEL_PARAM((baseContactPoints)),
			PX_CUDA_KERNEL_PARAM((baseContactForces)),
			PX_CUDA_KERNEL_PARAM((patchBytesLimit)),
			PX_CUDA_KERNEL_PARAM((contactBytesLimit)),
			PX_CUDA_KERNEL_PARAM((forceBytesLimit)),
			PX_CUDA_KERNEL_PARAM(toleranceLength)
		};

		const PxU32 numThreadsPerBlock = 64;
		const PxU32 numPairsPerBlock = numThreadsPerBlock / 4;
		const PxU32 numBlocks = (numTests + (numPairsPerBlock - 1)) / numPairsPerBlock;
		//Each thread do one collision detection
		result = mCudaContext->launchKernel(sphereKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU boxBoxNphase_Kernel fail to launch kernel stage 3!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cudaMainGjkEpa error in kernel stage 3!!!\n");
#endif

#if GPU_NP_VISUALIZATION
		result = mCudaContext->streamSynchronize(mStream);

		PxArray<PxgContactManagerInput> tCmInput(numTests);
		PxArray<PxsContactManagerOutput> tCmOutput(numTests);
		PxArray<PxsCachedTransform> tCachedTransform(PxU32((mGpuTransformCache.getSize() / sizeof(PxsCachedTransform)) + 1));
		PxArray<PxContactPatch> tPatch(numTests);
		PxArray<PxContact> tContact(numTests);
		mCudaContext->memcpyDtoH(tCmInput.begin(), gpuManagers.mContactManagerInputData.getDevicePtr(), sizeof(PxgContactManagerInput) * numTests);
		mCudaContext->memcpyDtoH(tCachedTransform.begin(), mGpuTransformCache.getDevicePtr(), mGpuTransformCache.getSize());
		mCudaContext->memcpyDtoH(tCmOutput.begin(), gpuManagers.mContactManagerOutputData.getDevicePtr(), sizeof(PxsContactManagerOutput) * numTests);
		mCudaContext->memcpyDtoH(tPatch.begin(), mPatchStream, sizeof(PxContactPatch) * numTests);
		mCudaContext->memcpyDtoH(tContact.begin(), mContactStream, sizeof(PxContact) * numTests);

		int bob = 0;
		PX_UNUSED(bob);
		//drawManifold(tManifold.begin(), tCmInput.begin(), tCachedTransform.begin(), numTests, *renderOutput, 0xffffff, 0.05f);

#endif
	}

	compactLostFoundPairs(gpuManagers, numTests, touchLostFlags, cmOutputs);
}

void PxgGpuNarrowphaseCore::testSDKConvexConvexGjkEpaGpu(
							PxgGpuContactManagers& gpuManagers, bool insertAveragePoint,
							const PxU32 numTests,
							PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
							PxU32 patchBytesLimit, PxU32 contactBytesLimit,	PxU32 forceBytesLimit,
							PxRenderOutput* renderOutput	)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKConvexConvexGjkEpaGpu", 0);

	PX_UNUSED(renderOutput);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();
	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>( gpuManagers.mContactManagerInputData.getDevicePtr());
	PxsContactManagerOutput* cmOutputs = reinterpret_cast<PxsContactManagerOutput*>( gpuManagers.mContactManagerOutputData.getDevicePtr());
	PxgPersistentContactManifold* cmPersistentManifolds = reinterpret_cast<PxgPersistentContactManifold*>( gpuManagers.mPersistentContactManifolds.getDevicePtr());
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>( mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());
	PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>( mGpuTransformCache.getDevicePtr());
	PxReal*	contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	PX_ASSERT(transformCache);
	PxsMaterialData* materials = reinterpret_cast<PxsMaterialData*>( mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());

	PxU32* touchLostFlags = reinterpret_cast<PxU32*>( gpuManagers.mTempRunsumArray.getDevicePtr());
	PxU32* touchFoundFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr() + sizeof(PxU32) * numTests);
	CUresult result;

	{
		PxCudaKernelParam kernelParams_stage1[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(cmPersistentManifolds),
			PX_CUDA_KERNEL_PARAM((touchLostFlags))
		};

		PxU32 numBlocks_stage1 = (numTests * 4 + PxgNarrowPhaseBlockDims::EARLY_OUT_KERNEL - 1) / PxgNarrowPhaseBlockDims::EARLY_OUT_KERNEL;
		
		CUfunction kernelFunction_stage1 = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_CONVEX_KERNEL_EARLY_OUT);

		CUfunction kernelFunction_stage2 = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_CONVEX_KERNEL_MAIN);

		result = mCudaContext->launchKernel(kernelFunction_stage1, numBlocks_stage1, 1, 1, WARP_SIZE, PxgNarrowPhaseBlockDims::EARLY_OUT_KERNEL / WARP_SIZE, 1, 0,
			mStream, kernelParams_stage1, sizeof(kernelParams_stage1), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cudaMainGjkEpa fail to launch kernel stage 1!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU cudaMainGjkEpa fail to launch kernel stage 1!!!\n");
#endif

		PxCudaKernelParam kernelParams_stage2[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(touchLostFlags),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(cmPersistentManifolds),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance)
		};

		PxU32 numWarpsPerBlock_stage2 = GJK_EPA_WARPS_PER_BLOCK;
		PxU32 numBlocks_stage2 = (numTests + numWarpsPerBlock_stage2 - 1) / numWarpsPerBlock_stage2;
		

		result = mCudaContext->launchKernel(kernelFunction_stage2, numBlocks_stage2, 1, 1, WARP_SIZE, numWarpsPerBlock_stage2, 1, 0,
			mStream, kernelParams_stage2, sizeof(kernelParams_stage2), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cudaMainGjkEpa fail to launch kernel stage 3!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU cudaMainGjkEpa error in kernel stage 3!!!\n");
#endif

#if GPU_NP_VISUALIZATION
		result = mCudaContext->streamSynchronize(mStream);
		PxArray<PxgPersistentContactManifold> tManifold(numTests);
		PxArray<PxgContactManagerInput> tCmInput(numTests);
		PxArray<PxsCachedTransform> tCachedTransform(PxU32((mGpuTransformCache.getSize() / sizeof(PxsCachedTransform)) + 1));
		mCudaContext->memcpyDtoH(tManifold.begin(), gpuManagers.mPersistentContactManifolds.getDevicePtr(), sizeof(PxgPersistentContactManifold) * numTests);
		mCudaContext->memcpyDtoH(tCmInput.begin(), gpuManagers.mContactManagerInputData.getDevicePtr(), sizeof(PxgContactManagerInput) * numTests);
		mCudaContext->memcpyDtoH(tCachedTransform.begin(), mGpuTransformCache.getDevicePtr(), mGpuTransformCache.getSize());

		drawManifold(tManifold.begin(), tCmInput.begin(), tCachedTransform.begin(), numTests, *renderOutput, 0xffffff, 0.05f);
#endif

		PxgDevicePointer<PxgPatchAndContactCounters> patchAndContactCountersD = mPatchAndContactCountersOnDevice.getTypedDevicePtr();

		CUfunction kernelFunctionFinishContacts = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::FINISH_CONTACTS_KERNEL);

		PxCudaKernelParam kernelParamsFinishContacts[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(cmPersistentManifolds),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(materials),
			PX_CUDA_KERNEL_PARAM(mContactStream),
			PX_CUDA_KERNEL_PARAM(mPatchStream),
			PX_CUDA_KERNEL_PARAM(insertAveragePoint),
			PX_CUDA_KERNEL_PARAM(patchAndContactCountersD),
			PX_CUDA_KERNEL_PARAM((touchLostFlags)),
			PX_CUDA_KERNEL_PARAM((touchFoundFlags)),
			PX_CUDA_KERNEL_PARAM((baseContactPatches)),
			PX_CUDA_KERNEL_PARAM((baseContactPoints)),
			PX_CUDA_KERNEL_PARAM((baseContactForces)),
			PX_CUDA_KERNEL_PARAM((patchBytesLimit)),
			PX_CUDA_KERNEL_PARAM((contactBytesLimit)),
			PX_CUDA_KERNEL_PARAM((forceBytesLimit))
		};

		PxU32 numWarpsPerBlockFinishContacts = PxgNarrowPhaseBlockDims::FINISH_CONTACTS / WARP_SIZE;
		PxU32 numBlocksFinishContacts = (numTests + PxgNarrowPhaseBlockDims::FINISH_CONTACTS - 1) / PxgNarrowPhaseBlockDims::FINISH_CONTACTS;

		result = mCudaContext->launchKernel(kernelFunctionFinishContacts, numBlocksFinishContacts, 1, 1, WARP_SIZE, numWarpsPerBlockFinishContacts,
			1, 0, mStream, kernelParamsFinishContacts, sizeof(kernelParamsFinishContacts), 0, PX_FL);

		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU kernelFunctionFinishContacts fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU kernelFunctionFinishContacts fail to launch kernel stage!!!\n");
#endif
	}

	compactLostFoundPairs(gpuManagers, numTests, touchLostFlags, cmOutputs);
}

void PxgGpuNarrowphaseCore::testSDKConvexPlaneGjkEpaGpu(
	PxgGpuContactManagers& gpuManagers, bool insertAveragePoint,
	const PxU32 numTests,
	PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
	PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit,
	PxRenderOutput* renderOutput)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKConvexPlaneGjkEpaGpu", 0);

	PX_UNUSED(renderOutput);
	PX_UNUSED(insertAveragePoint);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();
	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PxsContactManagerOutput* cmOutputs = reinterpret_cast<PxsContactManagerOutput*>(gpuManagers.mContactManagerOutputData.getDevicePtr());
	PxgPersistentContactManifold* cmPersistentManifolds = reinterpret_cast<PxgPersistentContactManifold*>(gpuManagers.mPersistentContactManifolds.getDevicePtr());
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());
	PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	PxReal*	contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	PX_ASSERT(transformCache);
	PxsMaterialData* materials = reinterpret_cast<PxsMaterialData*>(mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());

	PxU32* touchLostFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr());
	PxU32* touchFoundFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr() + sizeof(PxU32) * numTests);
	PxgDevicePointer<PxgPatchAndContactCounters> patchAndContactCountersD = mPatchAndContactCountersOnDevice.getTypedDevicePtr();

	CUresult result;

	{
		CUfunction convexPlaneKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_PLANE_KERNEL_MAIN);

		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(materials),
			PX_CUDA_KERNEL_PARAM(mContactStream),
			PX_CUDA_KERNEL_PARAM(mPatchStream),
			PX_CUDA_KERNEL_PARAM(cmPersistentManifolds),
			PX_CUDA_KERNEL_PARAM(patchAndContactCountersD),
			PX_CUDA_KERNEL_PARAM(touchLostFlags),
			PX_CUDA_KERNEL_PARAM(touchFoundFlags),
			PX_CUDA_KERNEL_PARAM(baseContactPatches),
			PX_CUDA_KERNEL_PARAM(baseContactPoints),
			PX_CUDA_KERNEL_PARAM(baseContactForces),
			PX_CUDA_KERNEL_PARAM(patchBytesLimit),
			PX_CUDA_KERNEL_PARAM(contactBytesLimit),
			PX_CUDA_KERNEL_PARAM(forceBytesLimit),
			PX_CUDA_KERNEL_PARAM(toleranceLength)
		};

		const PxU32 numThreadsPerWarp = 32;
		const PxU32 numWarpsPerBlock = CONVEX_PLANE_WARPS_PER_BLOCK;
		const PxU32 numBlocks = (numTests + (numWarpsPerBlock - 1)) / numWarpsPerBlock;
		//Each thread do one collision detection
		result = mCudaContext->launchKernel(convexPlaneKernelFunction, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, mStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexPlaneNphase_Kernel fail to launch !!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexPlaneNphase_Kernel error!!!\n");
#endif
	}

	compactLostFoundPairs(gpuManagers, numTests, touchLostFlags, cmOutputs);
}

////////////////

void PxgGpuNarrowphaseCore::testSDKConvexCorePlaneGjkEpaGpu(
	PxgGpuContactManagers& gpuManagers, bool insertAveragePoint,
	const PxU32 numTests,
	PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
	PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit,
	PxRenderOutput* renderOutput)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKConvexCorePlaneGjkEpaGpu", 0);

	PX_UNUSED(renderOutput);
	PX_UNUSED(insertAveragePoint);

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PxsContactManagerOutput* cmOutputs = reinterpret_cast<PxsContactManagerOutput*>(gpuManagers.mContactManagerOutputData.getDevicePtr());
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());
	PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	PxReal* contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	PX_ASSERT(transformCache);
	PxsMaterialData* materials = reinterpret_cast<PxsMaterialData*>(mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());

	PxU32* touchLostFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr());
	PxU32* touchFoundFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr() + sizeof(PxU32) * numTests);
	PxgDevicePointer<PxgPatchAndContactCounters> patchAndContactCountersD = mPatchAndContactCountersOnDevice.getTypedDevicePtr();

	CUresult result;
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEXCORE_PLANE_KERNEL_MAIN);

		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(materials),
			PX_CUDA_KERNEL_PARAM(mContactStream),
			PX_CUDA_KERNEL_PARAM(mPatchStream),
			PX_CUDA_KERNEL_PARAM(patchAndContactCountersD),
			PX_CUDA_KERNEL_PARAM(touchLostFlags),
			PX_CUDA_KERNEL_PARAM(touchFoundFlags),
			PX_CUDA_KERNEL_PARAM(baseContactPatches),
			PX_CUDA_KERNEL_PARAM(baseContactPoints),
			PX_CUDA_KERNEL_PARAM(baseContactForces),
			PX_CUDA_KERNEL_PARAM(patchBytesLimit),
			PX_CUDA_KERNEL_PARAM(contactBytesLimit),
			PX_CUDA_KERNEL_PARAM(forceBytesLimit),
		};

		const PxU32 numThreadsPerBlock = 1024;
		const PxU32 numThreadsPerTest = 1;
		const PxU32 numTestsPerBlock = numThreadsPerBlock / numThreadsPerTest;
		const PxU32 numBlocks = (numTests + (numTestsPerBlock - 1)) / numTestsPerBlock;
		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexCorePlaneNphase_Kernel fail to launch!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexCorePlaneNphase_Kernel error!!!\n");
#endif
	}

	compactLostFoundPairs(gpuManagers, numTests, touchLostFlags, cmOutputs);
}

void PxgGpuNarrowphaseCore::testSDKConvexCoreConvexGjkEpaGpu(
	PxgGpuContactManagers& gpuManagers, bool insertAveragePoint,
	const PxU32 numTests,
	PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
	PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit,
	PxRenderOutput* renderOutput)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKConvexCoreConvexGjkEpaGpu", 0);

	PX_UNUSED(renderOutput);
	PX_UNUSED(insertAveragePoint);

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PxsContactManagerOutput* cmOutputs = reinterpret_cast<PxsContactManagerOutput*>(gpuManagers.mContactManagerOutputData.getDevicePtr());
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());
	PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	PxReal* contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	PX_ASSERT(transformCache);
	PxsMaterialData* materials = reinterpret_cast<PxsMaterialData*>(mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());

	PxU32* touchLostFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr());
	PxU32* touchFoundFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr() + sizeof(PxU32) * numTests);
	PxgDevicePointer<PxgPatchAndContactCounters> patchAndContactCountersD = mPatchAndContactCountersOnDevice.getTypedDevicePtr();

	CUresult result;
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEXCORE_CONVEX_KERNEL_MAIN);

		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(materials),
			PX_CUDA_KERNEL_PARAM(mContactStream),
			PX_CUDA_KERNEL_PARAM(mPatchStream),
			PX_CUDA_KERNEL_PARAM(patchAndContactCountersD),
			PX_CUDA_KERNEL_PARAM(touchLostFlags),
			PX_CUDA_KERNEL_PARAM(touchFoundFlags),
			PX_CUDA_KERNEL_PARAM(baseContactPatches),
			PX_CUDA_KERNEL_PARAM(baseContactPoints),
			PX_CUDA_KERNEL_PARAM(baseContactForces),
			PX_CUDA_KERNEL_PARAM(patchBytesLimit),
			PX_CUDA_KERNEL_PARAM(contactBytesLimit),
			PX_CUDA_KERNEL_PARAM(forceBytesLimit),
		};
		const PxU32 numThreadsPerBlock = 512;
		const PxU32 numThreadsPerTest = 1;
		const PxU32 numTestsPerBlock = numThreadsPerBlock / numThreadsPerTest;
		const PxU32 numBlocks = (numTests + (numTestsPerBlock - 1)) / numTestsPerBlock;
		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexCoreConvexNphase_Kernel fail to launch!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexCoreConvexNphase_Kernel error!!!\n");
#endif
	}

	compactLostFoundPairs(gpuManagers, numTests, touchLostFlags, cmOutputs);
}

void PxgGpuNarrowphaseCore::testSDKConvexCoreTrimeshGjkEpaGpu(
	PxgGpuContactManagers& gpuManagers, bool insertAveragePoint,
	const PxU32 numTests,
	PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
	PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit,
	PxRenderOutput* renderOutput)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKConvexCoreTrimeshGjkEpaGpu", 0);

	PX_UNUSED(renderOutput);
	PX_UNUSED(insertAveragePoint);

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PxsContactManagerOutput* cmOutputs = reinterpret_cast<PxsContactManagerOutput*>(gpuManagers.mContactManagerOutputData.getDevicePtr());
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());
	PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	PxReal* contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	PX_ASSERT(transformCache);
	PxsMaterialData* materials = reinterpret_cast<PxsMaterialData*>(mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());

	PxU32* touchLostFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr());
	PxU32* touchFoundFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr() + sizeof(PxU32) * numTests);
	PxgDevicePointer<PxgPatchAndContactCounters> patchAndContactCountersD = mPatchAndContactCountersOnDevice.getTypedDevicePtr();

	CUresult result;
	{
		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(materials),
			PX_CUDA_KERNEL_PARAM(mContactStream),
			PX_CUDA_KERNEL_PARAM(mPatchStream),
			PX_CUDA_KERNEL_PARAM(patchAndContactCountersD),
			PX_CUDA_KERNEL_PARAM(touchLostFlags),
			PX_CUDA_KERNEL_PARAM(touchFoundFlags),
			PX_CUDA_KERNEL_PARAM(baseContactPatches),
			PX_CUDA_KERNEL_PARAM(baseContactPoints),
			PX_CUDA_KERNEL_PARAM(baseContactForces),
			PX_CUDA_KERNEL_PARAM(patchBytesLimit),
			PX_CUDA_KERNEL_PARAM(contactBytesLimit),
			PX_CUDA_KERNEL_PARAM(forceBytesLimit),
		};
		const PxU32 warpThreadCount = WARP_SIZE;
		const PxU32 blockWarpCount = 4;
		const PxU32 blockTests = blockWarpCount;
		const PxU32 numBlocks = (numTests + (blockTests - 1)) / blockTests;
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEXCORE_TRIMESH_KERNEL32_MAIN);
		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, warpThreadCount, blockWarpCount, 1, 0, mStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexCoreTrimeshNphase_Kernel fail to launch!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexCoreTrimeshNphase_Kernel error!!!\n");
#endif
	}

	compactLostFoundPairs(gpuManagers, numTests, touchLostFlags, cmOutputs);
}

void PxgGpuNarrowphaseCore::testSDKConvexCoreTetmeshGjkEpaGpu(
	PxgGpuContactManagers& gpuManagers,
	const PxU32 numTests,
	PxRenderOutput* renderOutput)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKConvexCoreTetmeshGjkEpaGpu", 0);

	PX_UNUSED(renderOutput);

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());
	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();
	CUdeviceptr softBodiesd = simulationCore->getSoftBodyBuffer().getDevicePtr();
	CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();
	PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxReal* contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());
	PX_ASSERT(transformCache);
	PxgSoftBodyCore* softBodyCore = mGpuContext->mGpuSoftBodyCore;
	PxgFEMContactWriter writer(softBodyCore);

	CUstream softbodyStream = softBodyCore->getStream();

	CUdeviceptr totalNumCountsd = softBodyCore->getRigidContactCount().getDevicePtr();
	CUdeviceptr prevNumCountsd = softBodyCore->getPrevRigidContactCount().getDevicePtr();
	mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), softbodyStream);

	CUresult result;
	{
		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(softBodiesd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(writer),
		};
		const PxU32 warpThreadCount = WARP_SIZE;
		const PxU32 blockWarpCount = 4;
		const PxU32 blockTests = blockWarpCount;
		const PxU32 numBlocks = (numTests + (blockTests - 1)) / blockTests;
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEXCORE_TETMESH_KERNEL32_MAIN);
		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, warpThreadCount, blockWarpCount, 1, 0, softbodyStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexCoreTetmeshNphase_Kernel fail to launch!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexCoreTetmeshNphase_Kernel error!!!\n");
#endif
	}
	{
		CUdeviceptr contactsd = softBodyCore->getRigidContacts().getDevicePtr();
		CUdeviceptr barycentricsd = softBodyCore->getRigidBarycentrics().getDevicePtr();
		CUdeviceptr contactInfosd = softBodyCore->getRigidContactInfos().getDevicePtr();

		softbodyOtherContactApplyCollisionToSimMeshMapping(
			contactsd,
			barycentricsd,
			contactInfosd,
			totalNumCountsd,
			prevNumCountsd
		);
	}
}

void PxgGpuNarrowphaseCore::testSDKConvexCoreClothmeshGjkEpaGpu(
	PxgGpuContactManagers& gpuManagers,
	const PxU32 numTests,
	PxRenderOutput* renderOutput)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKConvexCoreClothmeshGjkEpaGpu", 0);

	PX_UNUSED(renderOutput);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PX_ASSERT(cmInputs);
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();
	const PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());
	const PxsCachedTransform* transformCached = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxReal* contactDistanced = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());
	PX_ASSERT(transformCached);

	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();

	PxgFEMClothCore* femClothCore = mGpuContext->mGpuFEMClothCore;

	PxgCudaPagedLinearAllocator<PxgHeapMemoryAllocator>& stackAlloc = femClothCore->getStackAllocator();
	stackAlloc.mMutex.lock();

	CUstream femClothStream = femClothCore->getStream();

	CUdeviceptr gpuMidphasePairsNumOnDevice = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));

	const PxU32 stackSizeBytes = mCollisionStackSizeBytes;
	CUdeviceptr gpuIntermStack = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(256, stackSizeBytes));
	CUdeviceptr stackSizeNeededOnDevice = femClothCore->mStackSizeNeededOnDevice.getDevicePtr();

	//initialize gpu variables
	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, femClothStream);

	CUdeviceptr femClothesd = simulationCore->getFEMClothBuffer().getDevicePtr();

	CUresult result;

	// fem cloth midphase kernel
	{

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(boundsd),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(femClothesd),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes)
		};

		//each warp deals with one test
		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = (numTests + numWarpsPerBlock - 1) / numWarpsPerBlock;
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_MIDPHASE_PRIMITIVES);
		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_midphaseGeneratePairsLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(femClothStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_midphaseGeneratePairsLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

		PxU32 numPairs;
		mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));
#endif
	}

	CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();
	CUdeviceptr filterPairs = simulationCore->getRigidClothFilters();
	const PxU32 nbFilterPairs = simulationCore->getNbRigidClothFilters();
	PxgFEMContactWriter writer(femClothCore);

	// convex core contact generation
	{
		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(femClothesd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(filterPairs),
			PX_CUDA_KERNEL_PARAM(nbFilterPairs),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(stackSizeNeededOnDevice),
			PX_CUDA_KERNEL_PARAM(writer),
		};

		//each thread deals with one test
		const PxU32 numBlocks = 32;
		const PxU32 blockThreadCount = 512;
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEXCORE_CLOTHMESH_KERNEL32_MAIN);
		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, blockThreadCount, 1, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexCoreClothmeshNphase_Kernel fail to launch!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexCoreClothmeshNphase_Kernel error!!!\n");
#endif
	}

	stackAlloc.reset();
	stackAlloc.mMutex.unlock();
}

void PxgGpuNarrowphaseCore::testSDKTriMeshPlaneGpu(PxgGpuContactManagers& gpuManagers, const PxU32 numTests,
	PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
	PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKTriMeshPlaneGpu", 0);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PxsContactManagerOutput* cmOutputs = reinterpret_cast<PxsContactManagerOutput*>(gpuManagers.mContactManagerOutputData.getDevicePtr());
	PxgPersistentContactMultiManifold* cmPersistentMultiManifolds = reinterpret_cast<PxgPersistentContactMultiManifold*>(gpuManagers.mPersistentContactManifolds.getDevicePtr());

	CUdeviceptr gpuShapes = mGpuShapesManager.mGpuShapesBuffer.getDevicePtr();
	PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	PxReal*	contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	PX_ASSERT(transformCache);
	PxsMaterialData* materials = reinterpret_cast<PxsMaterialData*>(mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());
	PxU32* touchLostFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr());
	PxU32* touchFoundFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr() + sizeof(PxU32) * numTests);
	PxgDevicePointer<PxgPatchAndContactCounters> patchAndContactCountersD = mPatchAndContactCountersOnDevice.getTypedDevicePtr();

	CUresult result;

	const PxReal clusterBias = 1e-3f*toleranceLength;

	{
		CUfunction triMeshPlaneKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::TRIMESH_PLANE_CORE);

		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(materials),
			PX_CUDA_KERNEL_PARAM(cmPersistentMultiManifolds),
			PX_CUDA_KERNEL_PARAM(mContactStream),
			PX_CUDA_KERNEL_PARAM(mPatchStream),
			PX_CUDA_KERNEL_PARAM(patchAndContactCountersD),
			PX_CUDA_KERNEL_PARAM(touchLostFlags),
			PX_CUDA_KERNEL_PARAM(touchFoundFlags),
			PX_CUDA_KERNEL_PARAM(baseContactPatches),
			PX_CUDA_KERNEL_PARAM(baseContactPoints),
			PX_CUDA_KERNEL_PARAM(baseContactForces),
			PX_CUDA_KERNEL_PARAM(patchBytesLimit),
			PX_CUDA_KERNEL_PARAM(contactBytesLimit),
			PX_CUDA_KERNEL_PARAM(forceBytesLimit),
			PX_CUDA_KERNEL_PARAM(clusterBias)
		};

		const PxU32 numThreadsPerBlock = 1024;
		const PxU32 numBlocks = numTests;
		//Each thread do one collision detection
		result = mCudaContext->launchKernel(triMeshPlaneKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU trimeshPlaneNarrowphase fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU trimeshPlaneNarrowphase error in kernel!!!\n");
#endif

#if GPU_NP_VISUALIZATION
		result = mCudaContext->streamSynchronize(mStream);

		PxArray<PxgContactManagerInput> tCmInput(numTests);
		PxArray<PxsContactManagerOutput> tCmOutput(numTests);
		PxArray<PxsCachedTransform> tCachedTransform(PxU32((mGpuTransformCache.getSize() / sizeof(PxsCachedTransform)) + 1));
		PxArray<PxContactPatch> tPatch(numTests);
		PxArray<PxContact> tContact(numTests);
		mCudaContext->memcpyDtoH(tCmInput.begin(), gpuManagers.mContactManagerInputData.getDevicePtr(), sizeof(PxgContactManagerInput) * numTests);
		mCudaContext->memcpyDtoH(tCachedTransform.begin(), mGpuTransformCache.getDevicePtr(), mGpuTransformCache.getSize());
		mCudaContext->memcpyDtoH(tCmOutput.begin(), gpuManagers.mContactManagerOutputData.getDevicePtr(), sizeof(PxsContactManagerOutput) * numTests);
		mCudaContext->memcpyDtoH(tPatch.begin(), mPatchStream, sizeof(PxContactPatch) * numTests);
		mCudaContext->memcpyDtoH(tContact.begin(), mContactStream, sizeof(PxContact) * numTests);

		int bob = 0;
		PX_UNUSED(bob);
		//drawManifold(tManifold.begin(), tCmInput.begin(), tCachedTransform.begin(), numTests, *renderOutput, 0xffffff, 0.05f);
#endif
	}

	compactLostFoundPairs(gpuManagers, numTests, touchLostFlags, cmOutputs);
}

void PxgGpuNarrowphaseCore::testSDKTriMeshHeightfieldGpu(
	PxgGpuContactManagers& gpuManagers, const PxU32 numTests,
	PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
	PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKTriMeshHeightfieldGpu", 0);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();
	const PxReal clusterBias = 1e-5f*toleranceLength;
	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PxsContactManagerOutput* cmOutputs = reinterpret_cast<PxsContactManagerOutput*>(gpuManagers.mContactManagerOutputData.getDevicePtr());
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());
	PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	PxReal*	contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	PX_ASSERT(transformCache);
	PxsMaterialData* materials = reinterpret_cast<PxsMaterialData*>(mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());

	PxU32* touchLostFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr());
	PxU32* touchFoundFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr() + sizeof(PxU32) * numTests);
	PxgDevicePointer<PxgPatchAndContactCounters> patchAndContactCountersD = mPatchAndContactCountersOnDevice.getTypedDevicePtr();

	CUresult result;

	{
		CUfunction trimeshKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::TRIMESH_HEIGHTFIELD_CORE);

		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(materials),
			PX_CUDA_KERNEL_PARAM(mContactStream),
			PX_CUDA_KERNEL_PARAM(mPatchStream),
			PX_CUDA_KERNEL_PARAM(patchAndContactCountersD),
			PX_CUDA_KERNEL_PARAM(touchLostFlags),
			PX_CUDA_KERNEL_PARAM(touchFoundFlags),
			PX_CUDA_KERNEL_PARAM(baseContactPatches),
			PX_CUDA_KERNEL_PARAM(baseContactPoints),
			PX_CUDA_KERNEL_PARAM(baseContactForces),
			PX_CUDA_KERNEL_PARAM(patchBytesLimit),
			PX_CUDA_KERNEL_PARAM(contactBytesLimit),
			PX_CUDA_KERNEL_PARAM(forceBytesLimit),
			PX_CUDA_KERNEL_PARAM(clusterBias)
		};

		const PxU32 numBlocks = numTests;
		//Each thread do one collision detection
		result = mCudaContext->launchKernel(trimeshKernelFunction, numBlocks, 1, 1, 1024, 1, 1, 0, mStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU triangleTriangleCollision fail to launch !!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU triangleTriangleCollision error!!!\n");
#endif
	}

	compactLostFoundPairs(gpuManagers, numTests, touchLostFlags, cmOutputs);
}

void PxgGpuNarrowphaseCore::testSDKTriMeshTriMeshGpu(PxgGpuContactManagers& gpuManagers, const PxU32 numTests,
	PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
	PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit)
{

	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKTriMeshTriMeshGpu", 0);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();
	const PxReal clusterBias = 1e-5f*toleranceLength;
	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PxsContactManagerOutput* cmOutputs = reinterpret_cast<PxsContactManagerOutput*>(gpuManagers.mContactManagerOutputData.getDevicePtr());
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());
	PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	PxReal*	contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	PX_ASSERT(transformCache);
	PxsMaterialData* materials = reinterpret_cast<PxsMaterialData*>(mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());

	PxU32* touchLostFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr());
	PxU32* touchFoundFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr() + sizeof(PxU32) * numTests);
	CUresult result;

	mIntermStackAlloc.mMutex.lock();

	CUdeviceptr gpuIntermOverlap = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, numTests * sizeof(PxU8)));

	{
		CUfunction tritriOverlapKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::TRIMESH_TRIMESH_OVERLAP);

		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(gpuIntermOverlap)
		};

		const PxU32 numWarpPerBlocks = NP_TRIMESH_WARPS_PER_BLOCK;
		const PxU32 numThreadsPerWarp = 32;

		const PxU32 numBlocks = (numTests + numWarpPerBlocks -1) / numWarpPerBlocks;
		//Each thread do one collision detection
		result = mCudaContext->launchKernel(tritriOverlapKernelFunction, numBlocks, 1, 1, numThreadsPerWarp, numWarpPerBlocks, 1, 0, mStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU triangleTriangleOverlaps fail to launch !!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU triangleTriangleOverlaps error!!!\n");
#endif
	}

	{
		PxgDevicePointer<PxgPatchAndContactCounters> patchAndContactCountersD = mPatchAndContactCountersOnDevice.getTypedDevicePtr();
		CUfunction tritriKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::TRIMESH_TRIMESH_CORE);

		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(gpuIntermOverlap),
			PX_CUDA_KERNEL_PARAM(materials),
			PX_CUDA_KERNEL_PARAM(mContactStream),
			PX_CUDA_KERNEL_PARAM(mPatchStream),
			PX_CUDA_KERNEL_PARAM(patchAndContactCountersD),
			PX_CUDA_KERNEL_PARAM(touchLostFlags),
			PX_CUDA_KERNEL_PARAM(touchFoundFlags),
			PX_CUDA_KERNEL_PARAM(baseContactPatches),
			PX_CUDA_KERNEL_PARAM(baseContactPoints),
			PX_CUDA_KERNEL_PARAM(baseContactForces),
			PX_CUDA_KERNEL_PARAM(patchBytesLimit),
			PX_CUDA_KERNEL_PARAM(contactBytesLimit),
			PX_CUDA_KERNEL_PARAM(forceBytesLimit),
			PX_CUDA_KERNEL_PARAM(clusterBias)
		};
	
		const PxU32 numBlocks = numTests;
		//Each thread do one collision detection
		result = mCudaContext->launchKernel(tritriKernelFunction, numBlocks, 1, 1, 1024, 1, 1, 0, mStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU triangleTriangleCollision fail to launch !!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU triangleTriangleCollision error!!!\n");
#endif
	}

	compactLostFoundPairs(gpuManagers, numTests, touchLostFlags, cmOutputs);

	mIntermStackAlloc.reset();

	mIntermStackAlloc.mMutex.unlock();
}

static void fetchLostFoundPatchData(PxgGpuContactManagers& gpuContactManagers, PxPinnedArray<PxsContactManagerOutputCounts>& lostFoundPairsOutputData, 
	PxPinnedArray<PxsContactManager*>& lostFoundPairsCms, PxCudaContext* cudaContext, CUstream stream, PxU32& touchChangeOffset, PxU32& patchChangeOffset)
{
	if (gpuContactManagers.mLostAndTotalReportedPairsCountPinned->x)
	{
		const PxU32 count = gpuContactManagers.mLostAndTotalReportedPairsCountPinned->x;

		PX_ASSERT(lostFoundPairsOutputData.size() >= (touchChangeOffset + count));
		PX_ASSERT(lostFoundPairsCms.size() >= touchChangeOffset + count);

		PxsContactManagerOutputCounts* p = &lostFoundPairsOutputData[touchChangeOffset];
		PxsContactManager** p2 = &lostFoundPairsCms[touchChangeOffset];
		
		// AD: DtoH memcopy marker, needs to be safe in case we skip!
		cudaContext->memcpyDtoHAsync(p, 
			gpuContactManagers.mLostFoundPairsOutputData.getDevicePtr(),
			sizeof(PxsContactManagerOutputCounts) * count, stream);

		// AD: DtoH memcopy marker, needs to be safe in case we skip!
		cudaContext->memcpyDtoHAsync(p2, 
			gpuContactManagers.mLostFoundPairsCms.getDevicePtr(),
			sizeof(PxsContactManager*) * count, stream);

		touchChangeOffset += count;
	}

	PxU32 count = gpuContactManagers.mLostAndTotalReportedPairsCountPinned->y - gpuContactManagers.mLostAndTotalReportedPairsCountPinned->x;
	if (count)
	{
		PX_ASSERT(lostFoundPairsOutputData.size() >= patchChangeOffset);
		PX_ASSERT(lostFoundPairsCms.size() >= patchChangeOffset);

		PxsContactManagerOutputCounts* p = &lostFoundPairsOutputData[patchChangeOffset];
		PxsContactManager** p2 = &lostFoundPairsCms[patchChangeOffset];
		
		// AD: DtoH memcopy marker, needs to be safe in case we skip!
		cudaContext->memcpyDtoHAsync(p, 
			gpuContactManagers.mLostFoundPairsOutputData.getDevicePtr() + gpuContactManagers.mLostAndTotalReportedPairsCountPinned->x * sizeof(PxsContactManagerOutputCounts),
			sizeof(PxsContactManagerOutputCounts) * count, stream);


		// AD: DtoH memcopy marker, needs to be safe in case we skip!
		cudaContext->memcpyDtoHAsync(p2, 
			gpuContactManagers.mLostFoundPairsCms.getDevicePtr() + gpuContactManagers.mLostAndTotalReportedPairsCountPinned->x * sizeof(PxsContactManager*),
			sizeof(PxsContactManager*) * count, stream);

		patchChangeOffset += count;
	}
}

CUstream PxgGpuNarrowphaseCore::getStream()
{
	return mStream;
}

void PxgGpuNarrowphaseCore::fetchNarrowPhaseResults(
	PxcDataStreamPool*		contactStreamPool,
	PxcDataStreamPool*		patchStreamPool,
	PxcDataStreamPool*		forceStreamPool,
	PxsContactManagerOutput*	contactManagerOutputs,
	// PT: these 'GPU' buffers are the ones from PxsContactManagers
	const Sc::ShapeInteraction*const*	shapeInteractionsGPU,
	const PxReal*						restDistancesGPU,
	const PxsTorsionalFrictionData*		torsionalDataGPU,
	PxU32 nbFallbackPairs,
	const PxsContactManagerOutputCounts* foundPatchCountsFallback,
	const PxsContactManager*const* foundPatchManagersFallback,
	PxU32 nbFoundPatchManagersFallback
	)
{
	PX_PROFILE_ZONE("GpuNarrowPhase.fetchGpuNarrowPhaseResults", 0);

	PxU32 numTests = 0;
	for (PxU32 i = GPU_BUCKET_ID::eConvex; i < GPU_BUCKET_ID::eCount; ++i)
	{
		numTests += mContactManagers[i]->getNbPassTests();
	}

	if (numTests)
	{
		// AD: some explanation:
		//
		// Each interaction type has two lists of contact managers, existing & new ones.
		// The existing ones are processed in the first pass. The new ones are processed
		// in the second pass (we first need to get the results from the broadphase to 
		// create them.)
		//
		// The following loop copies the contactManagerOutput for each contact manager 
		// from GPU to CPU into a flat list. It is laid out as follows:
		//
		// - first, we have all the outputs fron the fallback pairs. These are the ones that 
		//   had contact gen fall back to CPU.
		// - then we have the outputs for each bucket, always first the existing managers,
		// - followed by the new managers.
		//
		// the resulting list is then used to for places that need indexing into the contact and
		// patch streams on CPU.

		PxU32 offset = nbFallbackPairs;
		for (PxU32 i = GPU_BUCKET_ID::eConvex; i < GPU_BUCKET_ID::eCount; ++i)
		{
			PxU32 nbFirstPassTests = mContactManagers[i]->getNbFirstPassTests();
			if (nbFirstPassTests)
			{
				// AD: DtoH memcpy marker, needs to be safe in case we skip!
				mCudaContext->memcpyDtoHAsync(contactManagerOutputs + offset,
					mGpuContactManagers[i]->mContactManagers.mContactManagerOutputData.getDevicePtr(),
					sizeof(PxsContactManagerOutput) * nbFirstPassTests, mStream);

				offset += nbFirstPassTests;
			}

			PxU32 nbSecondPassTests = mContactManagers[i]->getNbSecondPassTests();
			if (nbSecondPassTests)
			{
				// AD: DtoH memcopy marker, needs to be safe in case we skip!
				mCudaContext->memcpyDtoHAsync(contactManagerOutputs + offset,
					mGpuContactManagers[i]->mNewContactManagers.mContactManagerOutputData.getDevicePtr(),
					sizeof(PxsContactManagerOutput) * nbSecondPassTests, mStream);

				offset += nbSecondPassTests;
			}
		}

		// AD: we need to be careful for the two copies above - they contain pointers into the patch/contact stream
		if (mCudaContext->isInAbortMode())
		{
			PxMemSet(contactManagerOutputs, 0, (numTests * sizeof(PxsContactManagerOutput)));
		}
	}

	mTotalLostFoundPairs = 0;
	mTotalLostFoundPatches = 0;
	mTotalNumPairs = numTests;

	if (numTests != 0)
	{
		// This copies back the sizes and overflow status of the contact stream.
		// After this, we know all the sizes.

		// AD: DtoH memcopy marker, needs to be safe in case we skip!
		mCudaContext->memcpyDtoHAsync(mPatchAndContactCountersReadback, mPatchAndContactCountersOnDevice.getDevicePtr(), sizeof(PxgPatchAndContactCounters), mStream);

		// set to 0 to be safe.
		if (mCudaContext->isInAbortMode())
		{
			mPatchAndContactCountersReadback->contactsBytes = 0;
			mPatchAndContactCountersReadback->patchesBytes = 0;
			mPatchAndContactCountersReadback->forceAndIndiceBytes = 0;
			mPatchAndContactCountersReadback->overflowError = 0;
		}

		{
			PX_PROFILE_ZONE("GpuNarrowPhase.Synchronize", 0);
			
			// it looks like we're only syncing here because of the overflow messages? is it ok to skip this sync if we don't run this block because there are no pairs?
			CUresult result = mCudaContext->streamSynchronize(mStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Synchronizing GPU Narrowphase failed! %d\n", result);
		}	

		PxU32 err = mPatchAndContactCountersReadback->getOverflowError();
		if (err)
		{
			if (err & PxgPatchAndContactCounters::CONTACT_BUFFER_OVERFLOW)
			{
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Contact buffer overflow detected, please increase its size to at least %i in the scene desc!\n", mPatchAndContactCountersReadback->contactsBytes/sizeof(PxContact));
			}

			if (err & PxgPatchAndContactCounters::PATCH_BUFFER_OVERFLOW)
			{
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Patch buffer overflow detected, please increase its size to at least %i in the scene desc!\n", mPatchAndContactCountersReadback->patchesBytes/sizeof(PxContactPatch));
			}
		}

		// AD: todo verify that we catch all the overflows with this.
		if (*mMaxConvexMeshTempMemory > mCollisionStackSizeBytes)
		{
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "PxGpuDynamicsMemoryConfig::collisionStackSize buffer overflow detected, please increase its size to at least %i in the scene desc! Contacts have been dropped.\n", *mMaxConvexMeshTempMemory);
		}

#if PX_ENABLE_SIM_STATS
		mGpuDynamicsRigidContactCountStats = PxMax(PxU32(mPatchAndContactCountersReadback->contactsBytes / sizeof(PxContact)), mGpuDynamicsRigidContactCountStats);
		mGpuDynamicsRigidPatchCountStats = PxMax(PxU32(mPatchAndContactCountersReadback->patchesBytes / sizeof(PxContactPatch)), mGpuDynamicsRigidPatchCountStats);
		mGpuDynamicsCollisionStackSizeStats = PxMax(*mMaxConvexMeshTempMemory, mGpuDynamicsCollisionStackSizeStats); // AD: this does not include the non-rigid stack usages yet!

		// update simulation statistics:
		mNphaseImplContext->getContext().getSimStats().mGpuDynamicsRigidContactCount = mGpuDynamicsRigidContactCountStats;
		mNphaseImplContext->getContext().getSimStats().mGpuDynamicsRigidPatchCount = mGpuDynamicsRigidPatchCountStats;

		// we need to max this because it could already have been set by the non-rigid parts.
		mNphaseImplContext->getContext().getSimStats().mGpuDynamicsCollisionStackSize = PxMax(mGpuDynamicsCollisionStackSizeStats, mNphaseImplContext->getContext().getSimStats().mGpuDynamicsCollisionStackSize);
#else
		PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

		*mMaxConvexMeshTempMemory = 0;
	}

	{
		//We need to group together all the touch found pairs and all the patch found pairs together. To do that, we need to keep track of 2 sets of offsets...
		// AD: We keep track of two sets of change flags per pair: 1) the touchChange, meaning whether we transitioned from touch to no touch or
		// vice-versa. 2) the patchChange, which signals whether the number of contact patches for that pair changed.

		// patchChangeOffset gets the offset into this data where we start having lost/found patches.
		PxU32 touchChangeOffset = 0;
		PxU32 patchChangeOffset = 0;

		// totalLostFoundPairs is the total sum of the touchChange and patchChange events.
		PxU32 totalLostFoundPairs = nbFoundPatchManagersFallback;

		for (PxU32 i = GPU_BUCKET_ID::eConvex; i < GPU_BUCKET_ID::eCount; ++i)
		{
			totalLostFoundPairs += mGpuContactManagers[i]->getTotalLostFoundPairs();
			patchChangeOffset += mGpuContactManagers[i]->getTotalLostFoundPatches(); // this actually gets the touchchange counts to figure out where we should start wit the patch change counts.
		}

		mLostFoundPairsOutputData.forceSize_Unsafe(0);
		mLostFoundPairsOutputData.reserve(totalLostFoundPairs);
		mLostFoundPairsOutputData.forceSize_Unsafe(totalLostFoundPairs);

		mLostFoundPairsCms.forceSize_Unsafe(0);
		mLostFoundPairsCms.reserve(totalLostFoundPairs);
		mLostFoundPairsCms.forceSize_Unsafe(totalLostFoundPairs);

		// we are doing DtoH copies in here.
		for (PxU32 i = GPU_BUCKET_ID::eConvex; i < GPU_BUCKET_ID::eCount; ++i)
		{
			fetchLostFoundPatchData(mGpuContactManagers[i]->mContactManagers, mLostFoundPairsOutputData, mLostFoundPairsCms, mCudaContext, mStream, touchChangeOffset, patchChangeOffset);
			fetchLostFoundPatchData(mGpuContactManagers[i]->mNewContactManagers, mLostFoundPairsOutputData, mLostFoundPairsCms, mCudaContext, mStream, touchChangeOffset, patchChangeOffset);
		}

		// now we have touchChangeOffset holding the number of lost/found changes,
		// and patchChangeOffset hoding the total number of touch + patch changes. We already knew that from the scan..?
		// except the part with the fallbacks.

		// we have copied the outputData plus the pointers into mLostFoundPairsOutputdata and mLostFoundPairsOutputCms.

		mTotalLostFoundPairs = touchChangeOffset;
		mTotalLostFoundPatches = patchChangeOffset - touchChangeOffset + nbFoundPatchManagersFallback;

		// AD: safety for fetchLostFoundPatchData + the corresponding counts.
		// let's just make those 0 along with their counts.
		if (mCudaContext->isInAbortMode())
		{
			mLostFoundPairsCms.forceSize_Unsafe(0);
			mLostFoundPairsOutputData.forceSize_Unsafe(0);
			mTotalLostFoundPairs = 0;
			mTotalLostFoundPatches = 0;

			nbFoundPatchManagersFallback = 0; // for memset below.			
		}

		if(nbFoundPatchManagersFallback)
		{
			// this adds the fallback output data + CMs to the same list.
			PxMemCopy(mLostFoundPairsOutputData.begin() + patchChangeOffset, foundPatchCountsFallback, sizeof(PxsContactManagerOutputCounts) * nbFoundPatchManagersFallback);
			PxMemCopy(mLostFoundPairsCms.begin() + patchChangeOffset, foundPatchManagersFallback, sizeof(PxsContactManager*) * nbFoundPatchManagersFallback);
		}

		//PX_ASSERT(mTotalLostFoundPairs <= mTotalLostFoundPatches);

		// AD: this streamsynchronize is placed quite randomly. Doesn't this one make the sync above unnecessary?
		// we could just report the overflows after this?
		{
			PX_PROFILE_ZONE("GpuNarrowPhase.Synchronize", 0);

			CUresult result =  mCudaContext->streamSynchronize(mStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Fetching GPU Narrowphase failed! %d\n", result);
		}
		
		//KS - no need for atomics - we now fetch all results at once!
		//FD: if there is an overflow, the counter value may exceed the limit, though the contacts\patches should be dropped
		mPatchAndContactCountersReadback->contactsBytes = PxMin(mPatchAndContactCountersReadback->contactsBytes, contactStreamPool->mDataStreamSize);
		mPatchAndContactCountersReadback->patchesBytes = PxMin(mPatchAndContactCountersReadback->patchesBytes, patchStreamPool->mDataStreamSize);
		mPatchAndContactCountersReadback->forceAndIndiceBytes = PxMin(mPatchAndContactCountersReadback->forceAndIndiceBytes, forceStreamPool->mDataStreamSize);

		contactStreamPool->mSharedDataIndexGPU = mPatchAndContactCountersReadback->contactsBytes;
		patchStreamPool->mSharedDataIndexGPU = mPatchAndContactCountersReadback->patchesBytes;
		forceStreamPool->mSharedDataIndexGPU = mPatchAndContactCountersReadback->forceAndIndiceBytes;

		mGpuContext->getFrictionPatchStreamPool().mSharedDataIndexGPU = patchStreamPool->mSharedDataIndexGPU * sizeof(PxFrictionPatch) / sizeof(PxContactPatch);
	}

	// This function actually does not take these inputs. Just for fun. Maybe something to do with the virtual interface?
	// anyway, this is the place where we append the new contact managers to the existing lists.
	appendContactManagers(contactManagerOutputs, nbFallbackPairs);

	//Copy force buffers up from CPU data (includes triangle indices if present). Eventually avoidable. Expected to be small so not important!
	size_t cpuForceOffset = (forceStreamPool->mDataStreamSize - forceStreamPool->mSharedDataIndex);

	mCudaContext->memcpyHtoDAsync(mForceAndIndiceStream + cpuForceOffset,
		forceStreamPool->mDataStream + cpuForceOffset,
		size_t(forceStreamPool->mSharedDataIndex),
		mSolverStream);

	{
		// AD: so this resizes the GPU data structures for the convex pairs to hold numTests + fallback pairs.
		// Apparently we compact everything into one list here: all the contact managers are part of the convex bucket at that point.
		// so we resize the lists of the convex bucket to hold all the data, and DtoD copy everything into that bucket. Seems weird,
		// but all the follow-up kernels rely on it so I guess we need to live with it for the moment.

		//KS - TODO - consider only copying the current memory requirement for the convex contacts!
		mGpuContactManagers[GPU_BUCKET_ID::eConvex]->mContactManagers.mContactManagerInputData.allocateCopyOldDataAsync((numTests + nbFallbackPairs) * sizeof(PxgContactManagerInput), mCudaContext, mSolverStream, PX_FL);
		mGpuContactManagers[GPU_BUCKET_ID::eConvex]->mContactManagers.mContactManagerOutputData.allocateCopyOldDataAsync((numTests + nbFallbackPairs)* sizeof(PxsContactManagerOutput), mCudaContext, mSolverStream, PX_FL);
		mGpuContactManagers[GPU_BUCKET_ID::eConvex]->mContactManagers.mShapeInteractions.allocateCopyOldDataAsync((numTests + nbFallbackPairs) * sizeof(Sc::ShapeInteraction*), mCudaContext, mSolverStream, PX_FL);
		mGpuContactManagers[GPU_BUCKET_ID::eConvex]->mContactManagers.mRestDistances.allocateCopyOldDataAsync((numTests + nbFallbackPairs) * sizeof(PxReal), mCudaContext, mSolverStream, PX_FL);
		mGpuContactManagers[GPU_BUCKET_ID::eConvex]->mContactManagers.mTorsionalProperties.allocateCopyOldDataAsync((numTests + nbFallbackPairs) * sizeof(PxsTorsionalFrictionData), mCudaContext, mSolverStream, PX_FL);

		CUdeviceptr cvxInputDeviceptr = mGpuContactManagers[GPU_BUCKET_ID::eConvex]->mContactManagers.mContactManagerInputData.getDevicePtr();
		CUdeviceptr cvxDeviceptr = mGpuContactManagers[GPU_BUCKET_ID::eConvex]->mContactManagers.mContactManagerOutputData.getDevicePtr();
		CUdeviceptr cvxShapePtr = mGpuContactManagers[GPU_BUCKET_ID::eConvex]->mContactManagers.mShapeInteractions.getDevicePtr();
		CUdeviceptr cvxRestPtr = mGpuContactManagers[GPU_BUCKET_ID::eConvex]->mContactManagers.mRestDistances.getDevicePtr();
		CUdeviceptr cvxTorsionalPtr = mGpuContactManagers[GPU_BUCKET_ID::eConvex]->mContactManagers.mTorsionalProperties.getDevicePtr();

		//KS - we need to synchronize here with mSolverStream because we may have just allocated more memory and triggered a D2D memcpy.
		//This data needs to be read on mStream using a DtoH copy
		synchronizeStreams(mCudaContext, mSolverStream, mStream);

		//Copy CPU contacts after all others, we don't need to move convex buffer
		if(nbFallbackPairs > 0)
		{
			// this adds the fallback pairs to the end of the list.
			mCudaContext->memcpyHtoDAsync(cvxDeviceptr + numTests * sizeof(PxsContactManagerOutput), contactManagerOutputs,
				sizeof(PxsContactManagerOutput) * nbFallbackPairs, mSolverStream);

			mCudaContext->memcpyHtoDAsync(cvxShapePtr + numTests * sizeof(Sc::ShapeInteraction*), shapeInteractionsGPU,
				sizeof(Sc::ShapeInteraction*) * nbFallbackPairs, mSolverStream);

			mCudaContext->memcpyHtoDAsync(cvxRestPtr + numTests * sizeof(PxReal), restDistancesGPU,
				sizeof(PxReal) * nbFallbackPairs, mSolverStream);

			mCudaContext->memcpyHtoDAsync(cvxTorsionalPtr + numTests * sizeof(PxsTorsionalFrictionData), torsionalDataGPU,
				sizeof(PxsTorsionalFrictionData) * nbFallbackPairs, mSolverStream);
		}
	
		// and now we append all the other data from the fallbacks.
		if (1)
		{
			//append other primitive types contacts after convex - convex, the order is following BUCKET_ID other than fallback pairs
			PxU32 appendOffset = mContactManagers[GPU_BUCKET_ID::eConvex]->getNbPassTests();

			for (PxU32 i = GPU_BUCKET_ID::eConvexPlane; i < GPU_BUCKET_ID::eCount; ++i)
			{
				const PxU32 numPassTests = mContactManagers[i]->getNbPassTests();
				if (numPassTests > 0)
				{
					// and now we copy everything into the convex list? what happens to the data that is lying here?
					mCudaContext->memcpyDtoDAsync(cvxInputDeviceptr + appendOffset * sizeof(PxgContactManagerInput),
						mGpuContactManagers[i]->mContactManagers.mContactManagerInputData.getDevicePtr(), sizeof(PxgContactManagerInput) * numPassTests, mSolverStream);
				
					mCudaContext->memcpyDtoDAsync(cvxDeviceptr + appendOffset * sizeof(PxsContactManagerOutput),
						mGpuContactManagers[i]->mContactManagers.mContactManagerOutputData.getDevicePtr(), sizeof(PxsContactManagerOutput) * numPassTests, mSolverStream);

					mCudaContext->memcpyDtoDAsync(cvxShapePtr + appendOffset * sizeof(Sc::ShapeInteraction*),
						mGpuContactManagers[i]->mContactManagers.mShapeInteractions.getDevicePtr(), sizeof(Sc::ShapeInteraction*) * numPassTests, mSolverStream);

					mCudaContext->memcpyDtoDAsync(cvxRestPtr + appendOffset * sizeof(PxReal),
						mGpuContactManagers[i]->mContactManagers.mRestDistances.getDevicePtr(), sizeof(PxReal) * numPassTests, mSolverStream);

					mCudaContext->memcpyDtoDAsync(cvxTorsionalPtr + appendOffset * sizeof(PxsTorsionalFrictionData),
						mGpuContactManagers[i]->mContactManagers.mTorsionalProperties.getDevicePtr(), sizeof(PxsTorsionalFrictionData) * numPassTests, mSolverStream);

					appendOffset += numPassTests;
				}
			}
		}
	}
	
	// finally we copy the GPU contact stream data to the CPU.
	if (!mGpuContext->getEnableDirectGPUAPI() || mGpuContext->getSimulationController()->getEnableOVDCollisionReadback())
	{
		PxU32 numNewContactBytes = mPatchAndContactCountersReadback->contactsBytes;
		PxU32 numNewPatchBytes = mPatchAndContactCountersReadback->patchesBytes;

		////Copy contact/patch stream. TODO - defer this so that we don't sync on it
		if (numNewContactBytes)
		{
			// AD: DtoH memcopy marker, needs to be safe in case we skip!
			mCudaContext->memcpyDtoHAsync(contactStreamPool->mDataStream,
				mContactStream,
				numNewContactBytes, mStream);
		}

		if (numNewPatchBytes)
		{
			// AD: DtoH memcopy marker, needs to be safe in case we skip!
			mCudaContext->memcpyDtoHAsync(patchStreamPool->mDataStream,
				mPatchStream,
				numNewPatchBytes, mStream);
		}

		// AD: I don't think we need to safeguard the contact stream because we don't do any pointer stuff
		// in there. We will just read random garbage.
	}

	// AD: generally, this whole thing is a huge mess. I don't know why we have to keep copying in all directions.
	// fetchNarrowphaseResults suggests that this is a pull-based operation: we unify all the data in a common place
	// to hand it over to the solver. Instead what's happening is that we copy bits around into various places, do some
	// push-based stuff inserting some solver data structures, partially maintain host mirrors etc. 

#if PXG_CONTACT_VALIDATION
	result = mCudaContext->streamSynchronize(mStream);
	PX_ASSERT(result == CUDA_SUCCESS);

#endif
	mCudaContext->streamFlush(mSolverStream);
	mCudaContext->streamFlush(mStream);
}

void PxgGpuNarrowphaseCore::pushBuffer()
{
	PxScopedCudaLock lock(*mCudaContextManager);
	mCudaContext->streamFlush(mStream);
}


void PxgGpuNarrowphaseCore::testSDKSphereTriMeshSATGpu(PxgGpuContactManagers& gpuManagers, bool insertAveragePoint, const PxU32 numTests,
	PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
	PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKSphereTriMeshSATGpu", 0);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();
	const PxReal clusterBias = 1e-5f*toleranceLength;

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PxsContactManagerOutput* cmOutputs = reinterpret_cast<PxsContactManagerOutput*>(gpuManagers.mContactManagerOutputData.getDevicePtr());
	PxgPersistentContactMultiManifold* cmPersistentMultiManifolds = reinterpret_cast<PxgPersistentContactMultiManifold*>(gpuManagers.mPersistentContactManifolds.getDevicePtr());
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());
	
	PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();

	const PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxBounds3* bounds = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());
	const PxReal* contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	PX_ASSERT(transformCache);

	CUresult result;

	mIntermStackAlloc.mMutex.lock();

	CUdeviceptr gpuIntermSphereMeshPair = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, numTests * sizeof(ConvexMeshPair)));
	CUdeviceptr gpuMidphasePairsNumOnDevice = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));
	CUdeviceptr gpuMidphasePairsNumOnDevicePadded = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));
	CUdeviceptr gpuStackShift = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(PxU32)));


	// Core arrays pointers
	CUdeviceptr sphereTriNIGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	CUdeviceptr sphereTriContactsGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	CUdeviceptr sphereTriMaxDepthGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	//This is for post processing
	CUdeviceptr sphereTriIntermDataGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	CUdeviceptr orderedSphereTriIntermDataGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	CUdeviceptr sphereTriSecondPassPairsGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void*)));
	CUdeviceptr gpuSecondPassPairsNum = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));
	
	const PxU32 stackSizeBytes = mCollisionStackSizeBytes;
	CUdeviceptr gpuIntermStack = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, stackSizeBytes));


	//initialize gpu variables
	mCudaContext->memsetD32Async(gpuIntermSphereMeshPair, 0, numTests * sizeof(ConvexMeshPair) / sizeof(PxU32), mStream);
	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, mStream);
	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevicePadded, 0, 1, mStream);
	mCudaContext->memsetD32Async(gpuStackShift, 0, 1, mStream);
	mCudaContext->memsetD32Async(gpuSecondPassPairsNum, 0, 1, mStream);

	//Temp contact buffers
	CUdeviceptr gpuTempContactStack = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, stackSizeBytes));
	CUdeviceptr gpuTempContactIndex = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(PxU32)));
	mCudaContext->memsetD32Async(gpuTempContactIndex, 0, 1, mStream);

	// Convex-Trimesh Midphase kernel
	{
		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = (numTests + numWarpsPerBlock - 1) / numWarpsPerBlock;

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_TRIMESH_MIDPHASE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(bounds),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(gpuShapes),

			PX_CUDA_KERNEL_PARAM(gpuIntermSphereMeshPair),
			PX_CUDA_KERNEL_PARAM(cmPersistentMultiManifolds),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuStackShift),

			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevicePadded),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexTrimeshMidphase fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convex/sphereTrimeshMidphase kernel fail!!!\n");
#endif


	}

	// Sphere-Trimesh Core kernel
	{
		const PxU32 numThreadsPerBlock = 64;
		PxU32 numBlocks = (numTests + numThreadsPerBlock - 1) / numThreadsPerBlock;

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SPHERE_TRIMESH_CORE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(gpuShapes),

			PX_CUDA_KERNEL_PARAM(gpuIntermSphereMeshPair),

			PX_CUDA_KERNEL_PARAM(sphereTriNIGPU),
			PX_CUDA_KERNEL_PARAM(sphereTriContactsGPU),
			PX_CUDA_KERNEL_PARAM(sphereTriMaxDepthGPU),
			PX_CUDA_KERNEL_PARAM(sphereTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(orderedSphereTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(sphereTriSecondPassPairsGPU),

			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevicePadded),

			PX_CUDA_KERNEL_PARAM(gpuSecondPassPairsNum),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),

			PX_CUDA_KERNEL_PARAM(gpuTempContactStack),
			PX_CUDA_KERNEL_PARAM(gpuTempContactIndex),
			PX_CUDA_KERNEL_PARAM(mMaxConvexMeshTempMemory),
			PX_CUDA_KERNEL_PARAM(gpuStackShift),
			PX_CUDA_KERNEL_PARAM(numTests)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sphereTrimeshCore fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sphereTrimeshCore kernel fail!!!\n");
#endif
	}
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_TRIMESH_SORT_TRIANGLES);
		PxU32 numWarpsPerBlock = NP_TRIMESH_WARPS_PER_BLOCK;
		PxU32 numBlocks = (numTests + numWarpsPerBlock - 1) / numWarpsPerBlock;

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(gpuIntermSphereMeshPair),
			PX_CUDA_KERNEL_PARAM(orderedSphereTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(numTests)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE, NP_TRIMESH_WARPS_PER_BLOCK, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortTriangles fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortTriangles kernel fail!!!\n");
#endif
	}

	//Sphere-Trimesh Post Process Kernel
	{
		const PxU32 coreGridMultiplier = 8;	// Number which defines how we expand grid to try to cover all pairs without repretitions

		PxU32 numWarpsPerBlock = NP_TRIMESH_WARPS_PER_BLOCK;
		PxU32 numBlocks = (numTests*coreGridMultiplier + numWarpsPerBlock - 1) / numWarpsPerBlock;

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_TRIMESH_POST_PROCESS);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(gpuIntermSphereMeshPair),
			PX_CUDA_KERNEL_PARAM(sphereTriNIGPU),
			PX_CUDA_KERNEL_PARAM(sphereTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(orderedSphereTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(sphereTriSecondPassPairsGPU),

			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuSecondPassPairsNum),
			PX_CUDA_KERNEL_PARAM(gpuShapes)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE*numWarpsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sphereTrimeshPostProcess fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sphereTrimeshPostProcess kernel fail!!!\n");
#endif
	}

	// Sphere-Trimesh Correlate Kernel
	{
		PxU32 numWarpsPerBlock = CORRELATE_WARPS_PER_BLOCK;
		PxU32 numThreadGroupsPerBlock = numWarpsPerBlock;
		PxU32 numBlocks = (numTests + numThreadGroupsPerBlock - 1) / numThreadGroupsPerBlock;

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_TRIMESH_CORRELATE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(gpuIntermSphereMeshPair),
			PX_CUDA_KERNEL_PARAM(sphereTriMaxDepthGPU),
			PX_CUDA_KERNEL_PARAM(sphereTriNIGPU),
			PX_CUDA_KERNEL_PARAM(sphereTriContactsGPU),
			PX_CUDA_KERNEL_PARAM(cmPersistentMultiManifolds),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(clusterBias),
			PX_CUDA_KERNEL_PARAM(gpuTempContactStack),
			PX_CUDA_KERNEL_PARAM(gpuTempContactIndex)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sphereTrimeshCorrelate fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sphereTrimeshCorrelate kernel fail!!!\n");
#endif
	}


	PxU32* touchChangeFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr());
	PxU32* patchChangeFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr() + sizeof(PxU32)* numTests);

	{

		PxU32 numWarpsPerBlock = CORRELATE_WARPS_PER_BLOCK;
		PxU32 numThreadGroupsPerBlock = numWarpsPerBlock;
		PxU32 numBlocks = (numTests + numThreadGroupsPerBlock - 1) / numThreadGroupsPerBlock;

		PxsMaterialData* materials = reinterpret_cast<PxsMaterialData*>(mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());
		PxgDevicePointer<PxgPatchAndContactCounters> patchAndContactCountersD = mPatchAndContactCountersOnDevice.getTypedDevicePtr();

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_TRIMESH_FINISHCONTACTS);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(gpuIntermSphereMeshPair),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(cmPersistentMultiManifolds),
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(materials),
			PX_CUDA_KERNEL_PARAM(mContactStream),
			PX_CUDA_KERNEL_PARAM(mPatchStream),
			PX_CUDA_KERNEL_PARAM(mForceAndIndiceStream),
			PX_CUDA_KERNEL_PARAM(insertAveragePoint),
			PX_CUDA_KERNEL_PARAM(patchAndContactCountersD),
			PX_CUDA_KERNEL_PARAM(touchChangeFlags),
			PX_CUDA_KERNEL_PARAM(patchChangeFlags),
			PX_CUDA_KERNEL_PARAM(baseContactPatches),
			PX_CUDA_KERNEL_PARAM(baseContactPoints),
			PX_CUDA_KERNEL_PARAM(baseContactForces),
			PX_CUDA_KERNEL_PARAM(patchBytesLimit),
			PX_CUDA_KERNEL_PARAM(contactBytesLimit),
			PX_CUDA_KERNEL_PARAM(forceBytesLimit)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE*numWarpsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexTrimesh finishContacts fail to launch kernel!!\n");
	
#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexTrimesh finishContacts kernel fail!!!\n");
#endif
	}

	mIntermStackAlloc.reset();
	mIntermStackAlloc.mMutex.unlock();

	compactLostFoundPairs(gpuManagers, numTests, touchChangeFlags, cmOutputs);


	//PxCudaStreamFlush(mStream);
}

void PxgGpuNarrowphaseCore::testSDKSphereHeightfieldGpu(PxgGpuContactManagers& gpuManagers, bool insertAveragePoint, const PxU32 numTests,
	PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
	PxU32 patchBytesLimit, PxU32 contactBytesLimit, PxU32 forceBytesLimit)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKSphereHeightfieldGpu", 0);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();
	const PxReal clusterBias = 1e-5f*toleranceLength;

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PxsContactManagerOutput* cmOutputs = reinterpret_cast<PxsContactManagerOutput*>(gpuManagers.mContactManagerOutputData.getDevicePtr());
	PxgPersistentContactMultiManifold* cmPersistentMultiManifolds =
		reinterpret_cast<PxgPersistentContactMultiManifold*>(gpuManagers.mPersistentContactManifolds.getDevicePtr());
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());
	const PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());

	PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();


	const PxBounds3* bounds = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());
	const PxReal* contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	PX_ASSERT(transformCache);

	CUresult result;

	mIntermStackAlloc.mMutex.lock();

	// TODO avoroshilov: manage the stack mem
	CUdeviceptr gpuIntermCvxMeshPair = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, numTests * sizeof(ConvexMeshPair)));
	CUdeviceptr gpuMidphasePairsNumOnDevice = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));
	CUdeviceptr gpuMidphasePairsNumOnDevicePadded = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));
	CUdeviceptr gpuStackShift = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(PxU32)));

	// Core arrays pointers
	CUdeviceptr sphereTriNIGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	CUdeviceptr sphereTriContactsGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	CUdeviceptr sphereTriMaxDepthGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));

	//This is for post processing
	CUdeviceptr sphereTriIntermDataGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	CUdeviceptr orderedSphereTriIntermDataGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	CUdeviceptr sphereTriSecondPassPairsGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void*)));
	CUdeviceptr gpuSecondPassPairsNum = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));

	const PxU32 stackSizeBytes = mCollisionStackSizeBytes;
	CUdeviceptr gpuIntermStack = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, stackSizeBytes));

	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, mStream);
	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevicePadded, 0, 1, mStream);
	mCudaContext->memsetD32Async(gpuStackShift, 0, 1, mStream);
	mCudaContext->memsetD32Async(gpuSecondPassPairsNum, 0, 1, mStream);

	//Temp contact buffers
	CUdeviceptr gpuTempContactStack = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, stackSizeBytes));
	CUdeviceptr gpuTempContactIndex = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(PxU32)));
	mCudaContext->memsetD32Async(gpuTempContactIndex, 0, 1, mStream);


	// Sphere-Hf Midphase kernel
	{
		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = (numTests + numWarpsPerBlock - 1) / numWarpsPerBlock;

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_HEIGHTFIELD_MIDPHASE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(bounds),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(gpuShapes),


			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),
			PX_CUDA_KERNEL_PARAM(cmPersistentMultiManifolds),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuStackShift),

			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevicePadded),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes)

		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sphere(convex)HeightFieldMidphase fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sphere(convex)HeightFieldMidphase kernel fail!!!\n");
#endif
	}

	// Sphere-Hf Core kernel
	{
		const PxU32 numThreadsPerBlock = 64;
		PxU32 numBlocks = (numTests + numThreadsPerBlock - 1) / numThreadsPerBlock;

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SPHERE_HEIGHTFIELD_CORE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),

			PX_CUDA_KERNEL_PARAM(sphereTriNIGPU),
			PX_CUDA_KERNEL_PARAM(sphereTriContactsGPU),
			PX_CUDA_KERNEL_PARAM(sphereTriMaxDepthGPU),
			PX_CUDA_KERNEL_PARAM(sphereTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(orderedSphereTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(sphereTriSecondPassPairsGPU),

			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevicePadded),
			PX_CUDA_KERNEL_PARAM(gpuSecondPassPairsNum),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(gpuTempContactStack),
			PX_CUDA_KERNEL_PARAM(gpuTempContactIndex),
			PX_CUDA_KERNEL_PARAM(mMaxConvexMeshTempMemory),
			PX_CUDA_KERNEL_PARAM(gpuStackShift),
			PX_CUDA_KERNEL_PARAM(numTests)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexHeightfieldCore fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexHeightfieldCore kernel fail!!!\n");
#endif
	}

	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_TRIMESH_SORT_TRIANGLES);
		PxU32 numWarpsPerBlock = NP_TRIMESH_WARPS_PER_BLOCK;
		PxU32 numBlocks = (numTests + numWarpsPerBlock - 1) / numWarpsPerBlock;

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),
			PX_CUDA_KERNEL_PARAM(orderedSphereTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(numTests)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE, NP_TRIMESH_WARPS_PER_BLOCK, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortTriangles fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortTriangles kernel fail!!!\n");
#endif
	}

	//Sphere-Hf Post Process Kernel
	{
		const PxU32 coreGridMultiplier = 8;	// Number which defines how we expand grid to try to cover all pairs without repretitions

		PxU32 numWarpsPerBlock = NP_TRIMESH_WARPS_PER_BLOCK;
		PxU32 numBlocks = (numTests*coreGridMultiplier + numWarpsPerBlock - 1) / numWarpsPerBlock;

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_HEIGHTFIELD_POST_PROCESS);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),
			PX_CUDA_KERNEL_PARAM(sphereTriNIGPU),
			PX_CUDA_KERNEL_PARAM(sphereTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(orderedSphereTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(sphereTriSecondPassPairsGPU),

			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuSecondPassPairsNum),
			PX_CUDA_KERNEL_PARAM(gpuShapes)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE*numWarpsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexTrimeshPostProcess fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexTrimeshPostProcess kernel fail!!!\n");
#endif
	}
	// Sphere-Hf Correlate kernel
	{
		PxU32 numWarpsPerBlock = CORRELATE_WARPS_PER_BLOCK;
		PxU32 numThreadGroupsPerBlock = numWarpsPerBlock;
		PxU32 numBlocks = (numTests + numThreadGroupsPerBlock - 1) / numThreadGroupsPerBlock;

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_TRIMESH_CORRELATE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),
			PX_CUDA_KERNEL_PARAM(sphereTriMaxDepthGPU),
			PX_CUDA_KERNEL_PARAM(sphereTriNIGPU),
			PX_CUDA_KERNEL_PARAM(sphereTriContactsGPU),
			PX_CUDA_KERNEL_PARAM(cmPersistentMultiManifolds),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(clusterBias),
			PX_CUDA_KERNEL_PARAM(gpuTempContactStack),
			PX_CUDA_KERNEL_PARAM(gpuTempContactIndex),
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexHeightfieldCorrelate fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexHeightfieldCorrelate kernel fail!!!\n");
#endif
	}


	PxU32* touchChangeFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr());
	PxU32* patchChangeFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr() + sizeof(PxU32)* numTests);

	{

		PxU32 numWarpsPerBlock = CORRELATE_WARPS_PER_BLOCK;
		PxU32 numThreadGroupsPerBlock = numWarpsPerBlock;
		PxU32 numBlocks = (numTests + numThreadGroupsPerBlock - 1) / numThreadGroupsPerBlock;

		PxsMaterialData* materials = reinterpret_cast<PxsMaterialData*>(mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());
		PxgDevicePointer<PxgPatchAndContactCounters> patchAndContactCountersD = mPatchAndContactCountersOnDevice.getTypedDevicePtr();

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_TRIMESH_FINISHCONTACTS);

		PxCudaKernelParam kernelParams[] =
		{
			/*PX_CUDA_KERNEL_PARAM(gpuConvexShapes),
			PX_CUDA_KERNEL_PARAM(gpuTriMeshShapes),*/
			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(cmPersistentMultiManifolds),
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(materials),
			PX_CUDA_KERNEL_PARAM(mContactStream),
			PX_CUDA_KERNEL_PARAM(mPatchStream),
			PX_CUDA_KERNEL_PARAM(mForceAndIndiceStream),
			PX_CUDA_KERNEL_PARAM(insertAveragePoint),
			PX_CUDA_KERNEL_PARAM(patchAndContactCountersD),
			PX_CUDA_KERNEL_PARAM(touchChangeFlags),
			PX_CUDA_KERNEL_PARAM(patchChangeFlags),
			PX_CUDA_KERNEL_PARAM(baseContactPatches),
			PX_CUDA_KERNEL_PARAM(baseContactPoints),
			PX_CUDA_KERNEL_PARAM(baseContactForces),
			PX_CUDA_KERNEL_PARAM(patchBytesLimit),
			PX_CUDA_KERNEL_PARAM(contactBytesLimit),
			PX_CUDA_KERNEL_PARAM(forceBytesLimit)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE*numWarpsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexHeightfield finishContacts fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexHeightfield finishContacts kernel fail!!!\n");
#endif


	}

	mIntermStackAlloc.reset();
	mIntermStackAlloc.mMutex.unlock();

	compactLostFoundPairs(gpuManagers, numTests, touchChangeFlags, cmOutputs);

	//PxCudaStreamFlush(mStream);
}


void PxgGpuNarrowphaseCore::testSDKConvexTriMeshSATGpu(PxgGpuContactManagers& gpuManagers, bool insertAveragePoint, const PxU32 numTests,
	PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
	PxU32 patchBytesLimit, PxU32 contactBytesLimit,	PxU32 forceBytesLimit)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKConvexTriMeshSATGpu", 0);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();
	const PxReal clusterBias = 1e-5f*toleranceLength;

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>( gpuManagers.mContactManagerInputData.getDevicePtr() );
	PxsContactManagerOutput* cmOutputs = reinterpret_cast<PxsContactManagerOutput*>( gpuManagers.mContactManagerOutputData.getDevicePtr());
	PxgPersistentContactMultiManifold* cmPersistentMultiManifolds = reinterpret_cast<PxgPersistentContactMultiManifold*>( gpuManagers.mPersistentContactManifolds.getDevicePtr() );
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>( mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	const PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>( mGpuTransformCache.getDevicePtr());

	PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();

	const PxBounds3* bounds = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());
	const PxReal* contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	PX_ASSERT(transformCache);

	CUresult result;

	mIntermStackAlloc.mMutex.lock();

	CUdeviceptr gpuIntermCvxMeshPair = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, numTests * sizeof(ConvexMeshPair)));
	CUdeviceptr gpuMidphasePairsNumOnDevice = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));
	CUdeviceptr gpuMidphasePairsNumOnDevicePadded = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));
	CUdeviceptr gpuStackShift = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(PxU32)));
	

	// Core arrays pointers
	CUdeviceptr cvxTriNIGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	CUdeviceptr cvxTriContactsGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	CUdeviceptr cvxTriMaxDepthGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	//This is for post processing
	CUdeviceptr cvxTriIntermDataGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	CUdeviceptr orderedCvxTriIntermDataGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	CUdeviceptr cvxTriSecondPassPairsGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void*)));
	CUdeviceptr gpuSecondPassPairsNum = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));

	const PxU32 stackSizeBytes = mCollisionStackSizeBytes;
	CUdeviceptr gpuIntermStack = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, stackSizeBytes));

	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, mStream);
	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevicePadded, 0, 1, mStream);
	mCudaContext->memsetD32Async(gpuStackShift, 0, 1, mStream);
	mCudaContext->memsetD32Async(gpuSecondPassPairsNum, 0, 1, mStream);

	//Temp contact buffers
	CUdeviceptr gpuTempContactStack = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, stackSizeBytes));
	CUdeviceptr gpuTempContactIndex = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(PxU32)));
	mCudaContext->memsetD32Async(gpuTempContactIndex, 0, 1, mStream);

	// Convex-Trimesh Midphase kernel
	{
		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = (numTests + numWarpsPerBlock - 1) / numWarpsPerBlock;

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_TRIMESH_MIDPHASE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(bounds),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(gpuShapes),

			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),
			PX_CUDA_KERNEL_PARAM(cmPersistentMultiManifolds),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuStackShift),

			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevicePadded),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU convexTrimeshMidphase fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU convexTrimeshMidphase kernel fail!!!\n");
#endif


	}

	// Convex-Trimesh Core kernel
	{

		const PxU32 coreGridMultiplier = 32;	// Number which defines how we expand grid to try to cover all pairs without repretitions

		PxU32 numWarpsPerBlock = NP_TRIMESH_WARPS_PER_BLOCK;
		PxU32 numBlocks = PxMax(2048u, (numTests*coreGridMultiplier + numWarpsPerBlock - 1) / numWarpsPerBlock);
			
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_TRIMESH_CORE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(gpuShapes),

			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),

			PX_CUDA_KERNEL_PARAM(cvxTriNIGPU),
			PX_CUDA_KERNEL_PARAM(cvxTriContactsGPU),
			PX_CUDA_KERNEL_PARAM(cvxTriMaxDepthGPU),
			PX_CUDA_KERNEL_PARAM(cvxTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(orderedCvxTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(cvxTriSecondPassPairsGPU),

			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevicePadded),
			
			PX_CUDA_KERNEL_PARAM(gpuSecondPassPairsNum),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(gpuTempContactStack),
			PX_CUDA_KERNEL_PARAM(gpuTempContactIndex),
			PX_CUDA_KERNEL_PARAM(mMaxConvexMeshTempMemory),
			PX_CUDA_KERNEL_PARAM(gpuStackShift),
			PX_CUDA_KERNEL_PARAM(numTests)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU convexTrimeshCore fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU convexTrimeshCore kernel fail!!!\n");
#endif
	}

	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_TRIMESH_SORT_TRIANGLES);
		PxU32 numWarpsPerBlock = NP_TRIMESH_WARPS_PER_BLOCK;
		PxU32 numBlocks = (numTests + numWarpsPerBlock - 1) / numWarpsPerBlock;

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),
			PX_CUDA_KERNEL_PARAM(orderedCvxTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(numTests)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE, NP_TRIMESH_WARPS_PER_BLOCK, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortTriangles fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortTriangles kernel fail!!!\n");
#endif
	}

	//Convex-Trimesh Post Process Kernel
	{
		const PxU32 coreGridMultiplier = 8;	// Number which defines how we expand grid to try to cover all pairs without repretitions

		PxU32 numWarpsPerBlock = NP_TRIMESH_WARPS_PER_BLOCK;
		PxU32 numBlocks = PxMax(8192u, (numTests*coreGridMultiplier + numWarpsPerBlock - 1) / numWarpsPerBlock);

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_TRIMESH_POST_PROCESS);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),
			PX_CUDA_KERNEL_PARAM(cvxTriNIGPU),
			PX_CUDA_KERNEL_PARAM(cvxTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(orderedCvxTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(cvxTriSecondPassPairsGPU),

			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuSecondPassPairsNum),
			PX_CUDA_KERNEL_PARAM(gpuShapes)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE*numWarpsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexTrimeshPostProcess fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexTrimeshPostProcess kernel fail!!!\n");
#endif
	}

	// Convex-Trimesh Correlate Kernel
	{
		PxU32 numWarpsPerBlock = CORRELATE_WARPS_PER_BLOCK;
		PxU32 numThreadGroupsPerBlock = numWarpsPerBlock;
		PxU32 numBlocks = (numTests + numThreadGroupsPerBlock - 1) / numThreadGroupsPerBlock;

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_TRIMESH_CORRELATE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),
			PX_CUDA_KERNEL_PARAM(cvxTriMaxDepthGPU),
			PX_CUDA_KERNEL_PARAM(cvxTriNIGPU),
			PX_CUDA_KERNEL_PARAM(cvxTriContactsGPU),
			PX_CUDA_KERNEL_PARAM(cmPersistentMultiManifolds),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(clusterBias),
			PX_CUDA_KERNEL_PARAM(gpuTempContactStack),
			PX_CUDA_KERNEL_PARAM(gpuTempContactIndex)
		};  

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU convexTrimeshCorrelate fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU convexTrimeshCorrelate kernel fail!!!\n");
#endif
	}


	PxU32* touchChangeFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr());
	PxU32* patchChangeFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr() + sizeof(PxU32)* numTests);

	{

		PxU32 numWarpsPerBlock = CORRELATE_WARPS_PER_BLOCK;
		PxU32 numThreadGroupsPerBlock = numWarpsPerBlock;
		PxU32 numBlocks = (numTests + numThreadGroupsPerBlock - 1) / numThreadGroupsPerBlock;

		PxsMaterialData* materials = reinterpret_cast<PxsMaterialData*>( mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());
		PxgDevicePointer<PxgPatchAndContactCounters> patchAndContactCountersD = mPatchAndContactCountersOnDevice.getTypedDevicePtr();

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_TRIMESH_FINISHCONTACTS);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(cmPersistentMultiManifolds),
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(materials),
			PX_CUDA_KERNEL_PARAM(mContactStream),
			PX_CUDA_KERNEL_PARAM(mPatchStream),
			PX_CUDA_KERNEL_PARAM(mForceAndIndiceStream),
			PX_CUDA_KERNEL_PARAM(insertAveragePoint),
			PX_CUDA_KERNEL_PARAM(patchAndContactCountersD),
			PX_CUDA_KERNEL_PARAM(touchChangeFlags),
			PX_CUDA_KERNEL_PARAM(patchChangeFlags),
			PX_CUDA_KERNEL_PARAM(baseContactPatches),
			PX_CUDA_KERNEL_PARAM(baseContactPoints),
			PX_CUDA_KERNEL_PARAM(baseContactForces),
			PX_CUDA_KERNEL_PARAM(patchBytesLimit),
			PX_CUDA_KERNEL_PARAM(contactBytesLimit),
			PX_CUDA_KERNEL_PARAM(forceBytesLimit)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE*numWarpsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if(result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU convexTrimesh finishContacts fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU convexTrimesh finishContacts kernel fail!!!\n");
#endif
	}

	mIntermStackAlloc.reset();
	mIntermStackAlloc.mMutex.unlock();

	compactLostFoundPairs(gpuManagers, numTests, touchChangeFlags, cmOutputs);


	//PxCudaStreamFlush(mStream);
}

void PxgGpuNarrowphaseCore::testSDKConvexHeightfieldGpu(PxgGpuContactManagers& gpuManagers, bool insertAveragePoint, const PxU32 numTests,
	PxU8* baseContactPatches, PxU8* baseContactPoints, PxU8* baseContactForces,
	PxU32 patchBytesLimit, PxU32 contactBytesLimit,	PxU32 forceBytesLimit)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKConvexHeightfieldGpu", 0);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();
	const PxReal clusterBias = 1e-5f*toleranceLength;

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PxsContactManagerOutput* cmOutputs = reinterpret_cast<PxsContactManagerOutput*>(gpuManagers.mContactManagerOutputData.getDevicePtr());
	PxgPersistentContactMultiManifold* cmPersistentMultiManifolds =
		reinterpret_cast<PxgPersistentContactMultiManifold*>(gpuManagers.mPersistentContactManifolds.getDevicePtr());
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());
	const PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());

	PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();


	const PxBounds3* bounds = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());
	const PxReal* contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	PX_ASSERT(transformCache);

	CUresult result;

	mIntermStackAlloc.mMutex.lock();

	// TODO avoroshilov: manage the stack mem
	CUdeviceptr gpuIntermCvxMeshPair = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, numTests * sizeof(ConvexMeshPair)));
	CUdeviceptr gpuMidphasePairsNumOnDevice = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));
	CUdeviceptr gpuMidphasePairsNumOnDevicePadded = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));
	CUdeviceptr gpuStackShift = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(PxU32)));

	// Core arrays pointers
	CUdeviceptr cvxTriNIGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	CUdeviceptr cvxTriContactsGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	CUdeviceptr cvxTriMaxDepthGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));

	//This is for post processing
	CUdeviceptr cvxTriIntermDataGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	CUdeviceptr orderedCvxTriIntermDataGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void *)));
	CUdeviceptr cvxTriSecondPassPairsGPU = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(void*)));
	CUdeviceptr gpuSecondPassPairsNum = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));

	const PxU32 stackSizeBytes = mCollisionStackSizeBytes;
	CUdeviceptr gpuIntermStack = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, stackSizeBytes));

	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, mStream);
	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevicePadded, 0, 1, mStream);
	mCudaContext->memsetD32Async(gpuStackShift, 0, 1, mStream);
	mCudaContext->memsetD32Async(gpuSecondPassPairsNum, 0, 1, mStream);

	CUdeviceptr gpuTempContactStack = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, stackSizeBytes));
	CUdeviceptr gpuTempContactIndex = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, sizeof(PxU32)));
	mCudaContext->memsetD32Async(gpuTempContactIndex, 0, 1, mStream);


	// Convex-Height Midphase kernel
	{
		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = (numTests + numWarpsPerBlock - 1) / numWarpsPerBlock;

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_HEIGHTFIELD_MIDPHASE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(bounds),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(gpuShapes),


			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),
			PX_CUDA_KERNEL_PARAM(cmPersistentMultiManifolds),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuStackShift),

			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevicePadded),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes)
		
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU convexHeightFieldMidphase fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU convexHeightFieldMidphase kernel fail!!!\n");
#endif
	}

	// Convex-Hightfield Core kernel
	{
		const PxU32 coreGridMultiplier = 32;	// Number which defines how we expand grid to try to cover all pairs without repretitions

		PxU32 numWarpsPerBlock = NP_TRIMESH_WARPS_PER_BLOCK;
		PxU32 numBlocks = (numTests*coreGridMultiplier + numWarpsPerBlock - 1) / numWarpsPerBlock;

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_HEIGHTFIELD_CORE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),

			PX_CUDA_KERNEL_PARAM(cvxTriNIGPU),
			PX_CUDA_KERNEL_PARAM(cvxTriContactsGPU),
			PX_CUDA_KERNEL_PARAM(cvxTriMaxDepthGPU),
			PX_CUDA_KERNEL_PARAM(cvxTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(orderedCvxTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(cvxTriSecondPassPairsGPU),

			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevicePadded),
			PX_CUDA_KERNEL_PARAM(gpuSecondPassPairsNum),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(gpuTempContactStack),
			PX_CUDA_KERNEL_PARAM(gpuTempContactIndex),
			PX_CUDA_KERNEL_PARAM(mMaxConvexMeshTempMemory),
			PX_CUDA_KERNEL_PARAM(gpuStackShift),
			PX_CUDA_KERNEL_PARAM(numTests)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU convexHeightfieldCore fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU convexHeightfieldCore kernel fail!!!\n");
#endif
	}

	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_TRIMESH_SORT_TRIANGLES);
		PxU32 numWarpsPerBlock = NP_TRIMESH_WARPS_PER_BLOCK;
		PxU32 numBlocks = (numTests + numWarpsPerBlock - 1) / numWarpsPerBlock;

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),
			PX_CUDA_KERNEL_PARAM(orderedCvxTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(numTests)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE, NP_TRIMESH_WARPS_PER_BLOCK, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortTriangles fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortTriangles kernel fail!!!\n");
#endif
	}

	//Convex-Trimesh Post Process Kernel
	{
		const PxU32 coreGridMultiplier = 8;	// Number which defines how we expand grid to try to cover all pairs without repretitions

		PxU32 numWarpsPerBlock = NP_TRIMESH_WARPS_PER_BLOCK;
		PxU32 numBlocks = (numTests*coreGridMultiplier + numWarpsPerBlock - 1) / numWarpsPerBlock;

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_HEIGHTFIELD_POST_PROCESS);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),
			PX_CUDA_KERNEL_PARAM(cvxTriNIGPU),
			PX_CUDA_KERNEL_PARAM(cvxTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(orderedCvxTriIntermDataGPU),
			PX_CUDA_KERNEL_PARAM(cvxTriSecondPassPairsGPU),

			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuSecondPassPairsNum),
			PX_CUDA_KERNEL_PARAM(gpuShapes)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE*numWarpsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexTrimeshPostProcess fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU convexTrimeshPostProcess kernel fail!!!\n");
#endif
	}
	// Convex-Trimesh Correlate kernel
	{
		PxU32 numWarpsPerBlock = CORRELATE_WARPS_PER_BLOCK;
		PxU32 numThreadGroupsPerBlock = numWarpsPerBlock;
		PxU32 numBlocks = (numTests + numThreadGroupsPerBlock - 1) / numThreadGroupsPerBlock;

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_TRIMESH_CORRELATE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),
			PX_CUDA_KERNEL_PARAM(cvxTriMaxDepthGPU),
			PX_CUDA_KERNEL_PARAM(cvxTriNIGPU),
			PX_CUDA_KERNEL_PARAM(cvxTriContactsGPU),
			PX_CUDA_KERNEL_PARAM(cmPersistentMultiManifolds),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(clusterBias),
			PX_CUDA_KERNEL_PARAM(gpuTempContactStack),
			PX_CUDA_KERNEL_PARAM(gpuTempContactIndex)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU convexHeightfieldCorrelate fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU convexHeightfieldCorrelate kernel fail!!!\n");
#endif
	}


	PxU32* touchChangeFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr());
	PxU32* patchChangeFlags = reinterpret_cast<PxU32*>(gpuManagers.mTempRunsumArray.getDevicePtr() + sizeof(PxU32)* numTests);

	{

		PxU32 numWarpsPerBlock = CORRELATE_WARPS_PER_BLOCK;
		PxU32 numThreadGroupsPerBlock = numWarpsPerBlock;
		PxU32 numBlocks = (numTests + numThreadGroupsPerBlock - 1) / numThreadGroupsPerBlock;

		PxsMaterialData* materials = reinterpret_cast<PxsMaterialData*>(mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());
		PxgDevicePointer<PxgPatchAndContactCounters> patchAndContactCountersD = mPatchAndContactCountersOnDevice.getTypedDevicePtr();

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONVEX_TRIMESH_FINISHCONTACTS);

		PxCudaKernelParam kernelParams[] =
		{
			/*PX_CUDA_KERNEL_PARAM(gpuConvexShapes),
			PX_CUDA_KERNEL_PARAM(gpuTriMeshShapes),*/
			PX_CUDA_KERNEL_PARAM(gpuIntermCvxMeshPair),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
			PX_CUDA_KERNEL_PARAM(cmPersistentMultiManifolds),
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(materials),
			PX_CUDA_KERNEL_PARAM(mContactStream),
			PX_CUDA_KERNEL_PARAM(mPatchStream),
			PX_CUDA_KERNEL_PARAM(mForceAndIndiceStream),
			PX_CUDA_KERNEL_PARAM(insertAveragePoint),
			PX_CUDA_KERNEL_PARAM(patchAndContactCountersD),
			PX_CUDA_KERNEL_PARAM(touchChangeFlags),
			PX_CUDA_KERNEL_PARAM(patchChangeFlags),
			PX_CUDA_KERNEL_PARAM(baseContactPatches),
			PX_CUDA_KERNEL_PARAM(baseContactPoints),
			PX_CUDA_KERNEL_PARAM(baseContactForces),
			PX_CUDA_KERNEL_PARAM(patchBytesLimit),
			PX_CUDA_KERNEL_PARAM(contactBytesLimit),
			PX_CUDA_KERNEL_PARAM(forceBytesLimit)
		};

		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, WARP_SIZE*numWarpsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU convexHeightfield finishContacts fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU convexHeightfield finishContacts kernel fail!!!\n");
#endif


	}

	mIntermStackAlloc.reset();
	mIntermStackAlloc.mMutex.unlock();

	compactLostFoundPairs(gpuManagers, numTests, touchChangeFlags, cmOutputs);

	//PxCudaStreamFlush(mStream);
}


void initParticleContactWriter(PxgParticleContactWriter& writer, PxgParticleSystemCore* particleCore)
{
	writer.particleContacts = particleCore->getParticleContacts().getTypedPtr();
	writer.numTotalContacts = particleCore->getParticleContactCount().getTypedPtr();

	writer.contactSortedByParticle = particleCore->getContactSortedByParticle().getTypedPtr();
	writer.tempContactByParticle = particleCore->getTempContactByParticle().getTypedPtr();
	writer.contactIndexSortedByParticle = particleCore->getContactRemapSortedByParticle().getTypedPtr();

	writer.contactByRigid = particleCore->getContactByRigid().getTypedPtr();
	writer.tempContactByRigid = particleCore->getTempContactByRigid().getTypedPtr();
	writer.contactIndexSortedByRigid = particleCore->getContactRemapSortedByRigid().getTypedPtr();

	writer.maxContacts = particleCore->mMaxContacts;
}


void PxgGpuNarrowphaseCore::testSDKParticleSystemGpu(PxgGpuContactManagers& gpuManagers, const PxU32 numTests)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKParticleSystemGpu", 0);
	PxgParticleSystemCore* particleCore = mGpuContext->getGpuParticleSystemCore();
	const bool isTGS = mGpuContext->isTGS();
	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();
	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());
	PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	PxReal*	contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());

	PX_ASSERT(transformCache);
	PxsMaterialData* materials = reinterpret_cast<PxsMaterialData*>(mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());

	//PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();
	PxBounds3 * boundsd = reinterpret_cast<PxBounds3*>(mGpuContext->mGpuBp->getBoundsBuffer().getDevicePtr());
	

	PxgParticleSystem* particleSystemsd = reinterpret_cast<PxgParticleSystem*>(particleCore->getParticleSystemBuffer().getDevicePtr());
	PxU32* tempCellsHistogramd = reinterpret_cast<PxU32*>(particleCore->getTempCellsHistogram().getDevicePtr());
	PxU32* tempBlockCellsHistogramd = reinterpret_cast<PxU32*>(particleCore->getTempBlockCellsHistogram().getDevicePtr());
	PxU32* totalPairsd = reinterpret_cast<PxU32*>(particleCore->getTempHistogramCount().getDevicePtr());

	//Get the particle stream!
	CUstream particleStream = particleCore->getStream();

	// Simulation particles
	{
		CUresult result;
		{
			CUfunction firstPassFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_PRIMITIVES_BOUND_FIRST);
			PxCudaKernelParam kernelParams_stage[] =
			{
				PX_CUDA_KERNEL_PARAM(numTests),
				PX_CUDA_KERNEL_PARAM(cmInputs),
				PX_CUDA_KERNEL_PARAM(gpuShapes),
				PX_CUDA_KERNEL_PARAM(transformCache),
				PX_CUDA_KERNEL_PARAM(boundsd),
				PX_CUDA_KERNEL_PARAM(contactDistance),
				PX_CUDA_KERNEL_PARAM(particleSystemsd),
				PX_CUDA_KERNEL_PARAM(tempBlockCellsHistogramd),	// output
				PX_CUDA_KERNEL_PARAM(tempCellsHistogramd)		// output
			};

			result = mCudaContext->launchKernel(firstPassFunction, PxgParticleSystemKernelGridDim::BOUNDCELLUPDATE, 1, 1, PxgParticleSystemKernelBlockDim::BOUNDCELLUPDATE, 1, 1, 0, /*mStream*/particleStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_primitivesBoundFirstPassLaunch fail to launch!!\n");

			PX_ASSERT(result == CUDA_SUCCESS);
#if GPU_NP_DEBUG
			result = mCudaContext->streamSynchronize(particleStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_primitivesBoundFirstPassLaunch fail!!!\n");

			//Dma back the bound
			PxArray<PxU32> offsets;
			offsets.reserve(5000);
			offsets.forceSize_Unsafe(5000);
			mCudaContext->memcpyDtoH(offsets.begin(), reinterpret_cast<CUdeviceptr>(tempCellsHistogramd), sizeof(PxU32) * 2625);

			PxU32 blockOffset[32];
			mCudaContext->memcpyDtoH(blockOffset, reinterpret_cast<CUdeviceptr>(tempBlockCellsHistogramd), sizeof(PxU32) * 32);

			int bob = 0;
			PX_UNUSED(bob);
#endif
		}

		{
			CUfunction secondPassFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_PRIMITIVES_BOUND_SECOND);
			PxCudaKernelParam kernelParams_stage[] =
			{
				PX_CUDA_KERNEL_PARAM(numTests),
				PX_CUDA_KERNEL_PARAM(tempBlockCellsHistogramd),
				PX_CUDA_KERNEL_PARAM(tempCellsHistogramd),
				PX_CUDA_KERNEL_PARAM(totalPairsd)
			};

			result = mCudaContext->launchKernel(secondPassFunction, PxgParticleSystemKernelGridDim::BOUNDCELLUPDATE, 1, 1, PxgParticleSystemKernelBlockDim::BOUNDCELLUPDATE, 1, 1, 0, /*mStream*/particleStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_primitivesBoundSecondPassLaunch fail to launch!!\n");
#if GPU_NP_DEBUG
			result = mCudaContext->streamSynchronize(particleStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_primitivesBoundSecondPassLaunch fail!!!\n");

			//Dma back the bound
			PxArray<PxU32> offsets;
			offsets.reserve(5000);
			offsets.forceSize_Unsafe(5000);
			mCudaContext->memcpyDtoH(offsets.begin(), reinterpret_cast<CUdeviceptr>(tempCellsHistogramd), sizeof(PxU32) * 2625);


			PxU32 totalNumPairs;
			mCudaContext->memcpyDtoH(&totalNumPairs, reinterpret_cast<CUdeviceptr>(totalPairsd), sizeof(PxU32));

			int bob = 0;
			PX_UNUSED(bob);
#endif
		}

		{
			CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();

			CUfunction psCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_PRIMITIVES_COLLISION);

			PxgParticleContactWriter writer;
			initParticleContactWriter(writer, particleCore);

			PxCudaKernelParam kernelParams_stage[] =
			{
				PX_CUDA_KERNEL_PARAM(isTGS),
				PX_CUDA_KERNEL_PARAM(numTests),
				PX_CUDA_KERNEL_PARAM(toleranceLength),
				PX_CUDA_KERNEL_PARAM(cmInputs),
				PX_CUDA_KERNEL_PARAM(gpuShapes),
				PX_CUDA_KERNEL_PARAM(transformCache),
				PX_CUDA_KERNEL_PARAM(boundsd),
				PX_CUDA_KERNEL_PARAM(contactDistance),
				PX_CUDA_KERNEL_PARAM(restDistanced),
				PX_CUDA_KERNEL_PARAM(materials),
				PX_CUDA_KERNEL_PARAM(tempCellsHistogramd),
				PX_CUDA_KERNEL_PARAM(totalPairsd),
				PX_CUDA_KERNEL_PARAM(particleSystemsd),
				PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
				PX_CUDA_KERNEL_PARAM(writer)
			};

			//Each thread do one cell collision detection
			// blockIdx.z == 0 for regular particles, 1 for diffuse particles
			result = mCudaContext->launchKernel(psCollisionKernelFunction, PxgParticleSystemKernelGridDim::PS_COLLISION, 1, 1, PxgParticleSystemKernelBlockDim::PS_COLLISION, 1, 1, 0, /*mStream*/particleStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU particlePrimitivesCollisionLaunch fail to launch!!\n");

#if GPU_NP_DEBUG
			result = mCudaContext->streamSynchronize(particleStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU particlePrimitivesCollisionLaunch fail!!!\n");

#if GPU_NP_DEBUG_VERBOSE
			PxU32 numContacts;
			mCudaContext->memcpyDtoH(&numContacts, reinterpret_cast<CUdeviceptr>(writer.numTotalContacts), sizeof(PxU32));
			if (numContacts > 0)
			{
				printf("Particle numContacts %i\n", numContacts);

				PxgParticlePrimitiveContact temp[1];
				mCudaContext->memcpyDtoH(temp, reinterpret_cast<CUdeviceptr>(writer.particleContacts), sizeof(PxgParticlePrimitiveContact) * 1);
				printf("Particle first contact: particleId: %llu, rigidId: %u\n", temp[0].particleId, PxU32(temp[0].rigidId >> 32));
			}
#endif
#endif
		}

		{
			CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();
		
			CUfunction psCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_PRIMITIVES_DIFFUSE_COLLISION);

			PxCudaKernelParam kernelParams_stage[] =
			{
				PX_CUDA_KERNEL_PARAM(isTGS),
				PX_CUDA_KERNEL_PARAM(numTests),
				PX_CUDA_KERNEL_PARAM(toleranceLength),
				PX_CUDA_KERNEL_PARAM(cmInputs),
				PX_CUDA_KERNEL_PARAM(gpuShapes),
				PX_CUDA_KERNEL_PARAM(transformCache),
				PX_CUDA_KERNEL_PARAM(boundsd),
				PX_CUDA_KERNEL_PARAM(contactDistance),
				PX_CUDA_KERNEL_PARAM(restDistanced),
				PX_CUDA_KERNEL_PARAM(materials),
				PX_CUDA_KERNEL_PARAM(tempCellsHistogramd),
				PX_CUDA_KERNEL_PARAM(totalPairsd),
				PX_CUDA_KERNEL_PARAM(particleSystemsd),
				PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled)
			};

			//Each thread do one cell collision detection
			// blockIdx.z == 0 for regular particles, 1 for diffuse particles
			result = mCudaContext->launchKernel(psCollisionKernelFunction, PxgParticleSystemKernelGridDim::PS_COLLISION, 1, 1, PxgParticleSystemKernelBlockDim::PS_COLLISION, 1, 1, 0, /*mStream*/particleStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU particlePrimitivesDiffuseCollisionLaunch fail to launch!!\n");

#if GPU_NP_DEBUG
			result = mCudaContext->streamSynchronize(particleStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU particlePrimitivesCollisionLaunch fail!!!\n");
#endif
		}
	}
}


//all contact gen run in soft body stream
void PxgGpuNarrowphaseCore::testSDKParticleSoftbody(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput)
{
	PX_UNUSED(renderOutput);

	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKParticleSoftbody", 0);
	PxgParticleSystemCore* particleCore = mGpuContext->getGpuParticleSystemCore();
	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());

	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());
	PxsCachedTransform* transformCached = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	PxReal*	contactDistanced = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());
	PX_ASSERT(transformCached);	

	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();
	
	PxgSoftBodyCore* softBodyCore = mGpuContext->mGpuSoftBodyCore;
	PxBounds3 * boundsd = reinterpret_cast<PxBounds3*>(mGpuContext->mGpuBp->getBoundsBuffer().getDevicePtr());

	//contact gen is done at soft body stream
	CUstream softbodyStream = softBodyCore->getStream();

	PxgCudaPagedLinearAllocator<PxgHeapMemoryAllocator>& stackAlloc = softBodyCore->getStackAllocator();
	stackAlloc.mMutex.lock();

	const PxU32 stackSizeBytes = mCollisionStackSizeBytes;
	CUdeviceptr gpuMidphasePairsNumOnDevice = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));
	CUdeviceptr gpuIntermStack = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(256, stackSizeBytes));

	CUdeviceptr stackSizeNeededOnDevice = softBodyCore->mStackSizeNeededOnDevice.getDevicePtr();

	//OK. If we sync the soft bodies and particles here, we don't get crashes!
	synchronizeStreams(mCudaContext, particleCore->getStream(), softbodyStream);

	PxgParticleSystem* particleSystemsd = reinterpret_cast<PxgParticleSystem*>(particleCore->getParticleSystemBuffer().getDevicePtr());

	CUresult result;
		
	CUdeviceptr pairs = simulationCore->getSoftBodyParticleFilters();
	const PxU32 nbPairs = simulationCore->getNbSoftBodyParticleFilters();

	//initialize gpu variables
	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, softbodyStream);

	CUdeviceptr softBodiesd = simulationCore->getSoftBodyBuffer().getDevicePtr();

	{
		PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKParticleSoftbody.midphase", 0);
		CUfunction sbMidphaseKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_PS_MIDPHASE); //sb_psMidphaseGeneratePairsLaunch

		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(boundsd),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(softBodiesd),
			PX_CUDA_KERNEL_PARAM(particleSystemsd),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice)
		};

		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;

		//each warp deal with one test. 
		result = mCudaContext->launchKernel(sbMidphaseKernelFunction, 1024, numTests, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, softbodyStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_psMidphaseGeneratePairsLaunch fail to launch!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(softbodyStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU particlePrimitivesCollisionLaunch fail!!!\n");

		PxU32 numPairs;
		mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));

		if (numPairs > 0)
		{
			int bob = 0;
			PX_UNUSED(bob);
		}

#endif
	}

	PxgDevicePointer<float4> contactsd = softBodyCore->getParticleContacts().getTypedDevicePtr();
	PxgDevicePointer<float4> barycentricsd = softBodyCore->getParticleBarycentrics().getTypedDevicePtr();
	PxgDevicePointer<PxgFemOtherContactInfo> contactInfosd = softBodyCore->getParticleContactInfos().getTypedDevicePtr();

	PxgDevicePointer<PxU32> totalNumCountsd = softBodyCore->getParticleContactCount().getTypedDevicePtr();
	PxgDevicePointer<PxU32> prevNumCountsd = softBodyCore->getPrevParticleContactCount().getTypedDevicePtr();

	{
		PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKParticleSoftbody.contactGen", 0);
		mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), softbodyStream);

		PxgFEMContactWriter writer(softBodyCore, true);

		CUfunction sbCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_PS_CG); //sb_psContactGenLaunch

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(particleSystemsd),
			PX_CUDA_KERNEL_PARAM(softBodiesd),
			PX_CUDA_KERNEL_PARAM(pairs),
			PX_CUDA_KERNEL_PARAM(nbPairs),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(stackSizeNeededOnDevice),
			PX_CUDA_KERNEL_PARAM(writer)
		};

		PxU32 numBlocks = 512;
		//each thread deal with one test. 
		result = mCudaContext->launchKernel(sbCollisionKernelFunction, numBlocks, 1, 1, 256, 1, 1, 0, softbodyStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_psContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG

		result = mCudaContext->streamSynchronize(softbodyStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_psContactGenLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

		PxU32 numContacts;
		mCudaContext->memcpyDtoH(&numContacts, totalNumCountsd, sizeof(PxU32));

		if (numContacts > 0)
		{
			PxArray<float4> points(numContacts);
			PxArray<float4> normalPen(numContacts);
			PxArray<float4> barcentric(numContacts);

			CUdeviceptr normalPensd = softBodyCore->getParticleNormalPens().getDevicePtr();

			mCudaContext->memcpyDtoH(points.begin(), contactsd, sizeof(float4) * numContacts);
			mCudaContext->memcpyDtoH(normalPen.begin(), normalPensd, sizeof(float4) * numContacts);
			mCudaContext->memcpyDtoH(barcentric.begin(), barycentricsd, sizeof(float4) * numContacts);

			//const PxReal size = 0.1f;
			//RenderOutput& out = *renderOutput;

			//const PxU32 color = 0xff00ffff;
			//for (PxU32 j = 0; j < numContacts; ++j)
			//{
			//	PxVec3 a(points[j].x, points[j].y, points[j].z);
			//	PxVec3 n(normalPen[j].x, normalPen[j].y, normalPen[j].z);
			//	const PxReal pen = normalPen[j].w;

			//	PxVec3 b = a + n * pen;

			//	const PxVec3 up(0.f, size, 0.f);
			//	const PxVec3 right(size, 0.f, 0.f);
			//	const PxVec3 forwards(0.f, 0.f, size);

			//	const PxMat44(PxIdentity);

			//	out << color << m << RenderOutput::LINES << a + up << a - up;
			//	out << color << m << RenderOutput::LINES << a + right << a - right;
			//	out << color << m << RenderOutput::LINES << a + forwards << a - forwards;

			//	/*out << color << m << RenderOutput::LINES << b + up << b - up;
			//	out << color << m << RenderOutput::LINES << b + right << b - right;
			//	out << color << m << RenderOutput::LINES << b + forwards << b - forwards;*/

			//	//out << color << m << RenderOutput::LINES << a << b;
			//}
		}
#endif
	}

	{
		PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKParticleSoftbody.remap", 0);
        
        softbodyOtherContactApplyCollisionToSimMeshMapping(
			contactsd,
			barycentricsd,
			contactInfosd,
			totalNumCountsd,
			prevNumCountsd
		);
	}

	stackAlloc.reset();
	stackAlloc.mMutex.unlock();
}

//all contact gen run in cloth stream
void PxgGpuNarrowphaseCore::testSDKParticleFemCloth(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput)
{
	PX_UNUSED(renderOutput);

	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKParticleFemCloth", 0);
	PxgParticleSystemCore* particleCore = mGpuContext->getGpuParticleSystemCore();
	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());

	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());
	PxsCachedTransform* transformCached = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	PxReal*	contactDistanced = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());

	PX_ASSERT(transformCached);

	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();

	PxgFEMClothCore* clothCore = mGpuContext->mGpuFEMClothCore;

	PxBounds3 * boundsd = reinterpret_cast<PxBounds3*>(mGpuContext->mGpuBp->getBoundsBuffer().getDevicePtr());

	
	CUstream clothStream = clothCore->getStream();

	PxgCudaPagedLinearAllocator<PxgHeapMemoryAllocator>& stackAlloc = clothCore->getStackAllocator();
	stackAlloc.mMutex.lock();

	CUdeviceptr gpuMidphasePairsNumOnDevice = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));

	const PxU32 stackSizeBytes = mCollisionStackSizeBytes;
	CUdeviceptr gpuIntermStack = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(256, stackSizeBytes));

	CUdeviceptr stackSizeNeededOnDevice = clothCore->mStackSizeNeededOnDevice.getDevicePtr();
	
	CUdeviceptr clothesd = simulationCore->getFEMClothBuffer().getDevicePtr();


	PxgParticleSystem* particleSystemsd = reinterpret_cast<PxgParticleSystem*>(particleCore->getParticleSystemBuffer().getDevicePtr());

	//mCudaContext->streamWaitEvent(softbodyStream, ev);
	//OK. If we sync the cloth and particles here, we don't get crashes!
	synchronizeStreams(mCudaContext, particleCore->getStream(), clothStream);

	CUresult result;

	//initialize gpu variables
	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, clothStream);

		
	{
		CUfunction clothMidphaseKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_PS_MIDPHASE);

		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(boundsd),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(clothesd),
			PX_CUDA_KERNEL_PARAM(particleSystemsd),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice)
		};

		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;

		//each warp deal with one test. 
		result = mCudaContext->launchKernel(clothMidphaseKernelFunction, 1024, numTests, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, clothStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_psMidphaseGeneratePairsLaunch fail to launch!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(clothStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_psMidphaseGeneratePairsLaunch fail!!!\n");

		PxU32 numPairs;
		mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));

		if (numPairs > 0)
		{
			int bob = 0;
			PX_UNUSED(bob);
		}

#endif
	}

	CUdeviceptr totalNumCountsd = clothCore->getParticleContactCount().getDevicePtr();
	CUdeviceptr prevNumCountsd = clothCore->getPrevParticleContactCount().getDevicePtr();

	{
		mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), clothStream);

		PxgFEMContactWriter writer(clothCore, true);

		CUfunction clothCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_PS_CG);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(particleSystemsd),
			PX_CUDA_KERNEL_PARAM(clothesd),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(stackSizeNeededOnDevice),
			PX_CUDA_KERNEL_PARAM(writer)
		};

		PxU32 numBlocks = 512;
		//each thread deal with one test. 
		result = mCudaContext->launchKernel(clothCollisionKernelFunction, numBlocks, 1, 1, 256, 1, 1, 0, clothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_psContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG

		result = mCudaContext->streamSynchronize(clothStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_psContactGenLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

		PxU32 numContacts;
		mCudaContext->memcpyDtoH(&numContacts, totalNumCountsd, sizeof(PxU32));

		if (numContacts > 0)
		{
			CUdeviceptr contactsd = clothCore->getParticleContacts().getDevicePtr();
			CUdeviceptr normalPensd = clothCore->getParticleNormalPens().getDevicePtr();

			drawContacts(*renderOutput, contactsd, normalPensd, numContacts);
		}
#endif
	}
	
	stackAlloc.reset();

	stackAlloc.mMutex.unlock();
}


void PxgGpuNarrowphaseCore::testSDKConvexParticle(PxgGpuContactManagers& gpuManagers, const PxU32 numTests)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKConvexParticle", 0);
	PxgParticleSystemCore* particleCore = mGpuContext->getGpuParticleSystemCore();
	const bool isTGS = mGpuContext->isTGS();
	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();
	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());

	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());
	PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	//PxBounds3* boundArray = reinterpret_cast<PxBounds3*>(mGpuAABBBounds.getDevicePtr());
	PxReal* contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());

	PX_ASSERT(transformCache);
	PxsMaterialData* materials = reinterpret_cast<PxsMaterialData*>(mGpuMaterialManager.mGpuMaterialBuffer.getDevicePtr());

	PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(mGpuContext->mGpuBp->getBoundsBuffer().getDevicePtr());

	PxgParticleSystem* particleSystemsd = reinterpret_cast<PxgParticleSystem*>(particleCore->getParticleSystemBuffer().getDevicePtr());
	PxU32* tempCellsHistogramd = reinterpret_cast<PxU32*>(particleCore->getTempCellsHistogram().getDevicePtr());
	PxU32* tempBlockCellsHistogramd = reinterpret_cast<PxU32*>(particleCore->getTempBlockCellsHistogram().getDevicePtr());
	PxU32* totalPairsd = reinterpret_cast<PxU32*>(particleCore->getTempHistogramCount().getDevicePtr());

	CUstream particleStream = particleCore->getStream();

	//mCudaContext->memsetD32Async(totalContactCountsd, 0, 1, mStream);

	const PxU32 maxContacts = particleCore->mMaxContacts;


	CUresult result;
	{

		CUfunction firstPassFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_PRIMITIVES_BOUND_FIRST);
		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(boundsd),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(particleSystemsd),
			PX_CUDA_KERNEL_PARAM(tempBlockCellsHistogramd),
			PX_CUDA_KERNEL_PARAM(tempCellsHistogramd)
		};

		result = mCudaContext->launchKernel(firstPassFunction, PxgParticleSystemKernelGridDim::BOUNDCELLUPDATE, 1, 1, PxgParticleSystemKernelBlockDim::BOUNDCELLUPDATE, 1, 1, 0, /*mStream*/particleStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_primitivesBoundFirstPassLaunch fail to launch!!\n");
#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(particleStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_primitivesBoundFirstPassLaunch fail!!!\n");
#endif
	}

	{
		CUfunction secondPassFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_PRIMITIVES_BOUND_SECOND);
		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(tempBlockCellsHistogramd),
			PX_CUDA_KERNEL_PARAM(tempCellsHistogramd),
			PX_CUDA_KERNEL_PARAM(totalPairsd)
		};

		result = mCudaContext->launchKernel(secondPassFunction, PxgParticleSystemKernelGridDim::BOUNDCELLUPDATE, 1, 1, PxgParticleSystemKernelBlockDim::BOUNDCELLUPDATE, 1, 1, 0, /*mStream*/particleStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_primitivesBoundSecondPassLaunch fail to launch!!\n");
#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(particleStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_primitivesBoundSecondPassLaunch fail!!!\n");
#endif
	}

	{
		PxgParticlePrimitiveContact* contactsd = reinterpret_cast<PxgParticlePrimitiveContact*>(particleCore->getParticleContacts().getDevicePtr());
		CUdeviceptr totalContactCountsd = particleCore->getParticleContactCount().getDevicePtr();
		CUdeviceptr contactSortedByParticled = particleCore->getContactSortedByParticle().getDevicePtr();
		CUdeviceptr tempContactByParticleBitd = particleCore->getTempContactByParticle().getDevicePtr();
		CUdeviceptr contactRemapSortedByParticled = particleCore->getContactRemapSortedByParticle().getDevicePtr();
		CUdeviceptr contactByRigidd = particleCore->getContactByRigid().getDevicePtr();
		CUdeviceptr tempContactByRigidBitd = particleCore->getTempContactByRigid().getDevicePtr();
		CUdeviceptr contactRemapSortedByRigidd = particleCore->getContactRemapSortedByRigid().getDevicePtr();
		CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();

		//CUdeviceptr pairs = simulationCore->getRigidParticleFilters();
		//const PxU32 nbPairs = simulationCore->getNbRigidParticleFilters();

		CUfunction psCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_CONVEX_COLLISION);

		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(isTGS),
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(boundsd),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(materials),
			PX_CUDA_KERNEL_PARAM(tempCellsHistogramd),
			PX_CUDA_KERNEL_PARAM(totalPairsd),
			PX_CUDA_KERNEL_PARAM(particleSystemsd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
			PX_CUDA_KERNEL_PARAM(contactsd),
			PX_CUDA_KERNEL_PARAM(totalContactCountsd),
			PX_CUDA_KERNEL_PARAM(contactSortedByParticled),
			PX_CUDA_KERNEL_PARAM(tempContactByParticleBitd),
			PX_CUDA_KERNEL_PARAM(contactRemapSortedByParticled),
			PX_CUDA_KERNEL_PARAM(contactByRigidd),
			PX_CUDA_KERNEL_PARAM(tempContactByRigidBitd),
			PX_CUDA_KERNEL_PARAM(contactRemapSortedByRigidd),
			//PX_CUDA_KERNEL_PARAM(pairs),
			//PX_CUDA_KERNEL_PARAM(nbPairs),
			PX_CUDA_KERNEL_PARAM(maxContacts)
		};

		PxU32 numWarpPerBlock = PxgParticleSystemKernelBlockDim::PS_COLLISION / 32;
		//Each warp do one cell collision detection
		result = mCudaContext->launchKernel(psCollisionKernelFunction, PxgParticleSystemKernelGridDim::PS_COLLISION, 1, 1, 32, numWarpPerBlock, 1, 0, /*mStream*/particleStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_convexCollisionLaunch fail to launch!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(particleStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_convexCollisionLaunch fail!!!\n");

#endif
	}

	{
		CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();
		CUfunction psCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_CONVEX_DIFFUSE_COLLISION);

		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(isTGS),
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(boundsd),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(materials),
			PX_CUDA_KERNEL_PARAM(tempCellsHistogramd),
			PX_CUDA_KERNEL_PARAM(totalPairsd),
			PX_CUDA_KERNEL_PARAM(particleSystemsd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled)
		};

		PxU32 numWarpPerBlock = PxgParticleSystemKernelBlockDim::PS_COLLISION / 32;
		//Each warp do one cell collision detection
		result = mCudaContext->launchKernel(psCollisionKernelFunction, PxgParticleSystemKernelGridDim::PS_COLLISION, 1, 1, 32, numWarpPerBlock, 1, 0, /*mStream*/particleStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_convexDiffuseCollisionLaunch fail to launch!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(particleStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_convexDiffuseCollisionLaunch fail!!!\n");

#endif
	}
}

void PxgGpuNarrowphaseCore::testSDKParticleSdfTriMesh(PxgGpuContactManagers& gpuManagers, const PxU32 numTests)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKParticleSdfTriMesh", 0);
	PxgParticleSystemCore* particleCore = mGpuContext->getGpuParticleSystemCore();
	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());

	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	const PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxReal* contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());
	PX_ASSERT(transformCache);

	CUdeviceptr particleSystemsd = particleCore->getParticleSystemBuffer().getDevicePtr();
	CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();

	CUstream particleStream = particleCore->getStream();

	CUresult result;

	PxgParticleContactWriter writer;
	initParticleContactWriter(writer, particleCore);

	// Particle-Sdf-Trimesh Midphase and contact gen kernel
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_SDF_TRIMESH_COLLISION);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(particleSystemsd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
			PX_CUDA_KERNEL_PARAM(writer)
		};

		//each 32 blocks deal with one test. Each block has PS_MIDPHASE_COLLISION_WAPRS_PER_BLOCK warps 
		result = mCudaContext->launchKernel(kernelFunction, 
			numTests, 2, 1, 1024, 1, 1, 0, /*mStream*/particleStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_sdfMeshCollisonLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG

		result = mCudaContext->streamSynchronize(particleStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_sdfMeshCollisonLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);
#endif

	}
}

void PxgGpuNarrowphaseCore::testSDKParticleTriMesh(PxgGpuContactManagers& gpuManagers, const PxU32 numTests)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKParticleTriMesh", 0);
	PxgParticleSystemCore* particleCore = mGpuContext->getGpuParticleSystemCore();
	const bool isTGS = mGpuContext->isTGS();
	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	const PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxReal* contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());
	PX_ASSERT(transformCache);

	CUdeviceptr particleSystemsd = particleCore->getParticleSystemBuffer().getDevicePtr();
	CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();

	CUstream particleStream = particleCore->getStream();

	CUresult result;

	PxgParticleContactWriter writer;
	initParticleContactWriter(writer, particleCore);

	// Particle-Trimesh Midphase and contact gen kernel
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_TRIMESH_COLLISION);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(isTGS),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(particleSystemsd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
			PX_CUDA_KERNEL_PARAM(writer)
		};

		//each 32 blocks deal with one test. Each block has PS_MIDPHASE_COLLISION_WAPRS_PER_BLOCK warps 
		result = mCudaContext->launchKernel(kernelFunction, PxgParticleSystemKernelGridDim::PS_MESH_COLLISION, numTests, 2, WARP_SIZE, PS_MIDPHASE_COLLISION_WAPRS_PER_BLOCK, 1, 0, /*mStream*/particleStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_meshCollisonLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG

		result = mCudaContext->streamSynchronize(particleStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_meshCollisonLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);
#endif

	}
	

	//PxCudaStreamFlush(mStream);
}

void PxgGpuNarrowphaseCore::syncNotRigidWithNp()
{
	PxScopedCudaLock lock(*mCudaContextManager);

	PxgParticleSystemCore** particleCores = mGpuContext->getGpuParticleSystemCores();
	const PxU32 numParticleCores = mGpuContext->getNbGpuParticleSystemCores();
	PxgSoftBodyCore* softbodyCore = mGpuContext->mGpuSoftBodyCore;
	PxgFEMClothCore* femClothCore = mGpuContext->mGpuFEMClothCore;

	for(PxU32 i = 0; i < numParticleCores; ++i)
	{
		PxgParticleSystemCore* particleCore = particleCores[i];
		CUstream particleStream = particleCore->getStream();
		synchronizeStreams(mCudaContext, mStream, particleStream, mParticleEvent);
	}
	
	if (softbodyCore)
	{
		CUstream softbodyStream = softbodyCore->getStream();
		synchronizeStreams(mCudaContext, mStream, softbodyStream, mSoftbodyEvent);
	}

	if (femClothCore)
	{
		CUstream femClothStream = femClothCore->getStream();
		synchronizeStreams(mCudaContext, mStream, femClothStream, mFemClothEvent);
	}
}

void PxgGpuNarrowphaseCore::testSDKParticleHeightfield(PxgGpuContactManagers& gpuManagers, const PxU32 numTests)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKParticleHeightfield", 0);
	PxgParticleSystemCore* particleCore = mGpuContext->getGpuParticleSystemCore();

	const bool isTGS = mGpuContext->isTGS();
	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());

	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	const PxsCachedTransform* transformCache = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxReal* contactDistance = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());
	PX_ASSERT(transformCache);

	CUdeviceptr particleSystemsd = particleCore->getParticleSystemBuffer().getDevicePtr();
	CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();

	CUstream particleStream = particleCore->getStream();

	CUresult result;

	PxgParticleContactWriter writer;
	initParticleContactWriter(writer, particleCore);

	// Particle-Trimesh Midphase and contact gen kernel
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_HEIGHTFIELD_COLLISION);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(isTGS),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCache),
			PX_CUDA_KERNEL_PARAM(contactDistance),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(particleSystemsd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
			PX_CUDA_KERNEL_PARAM(writer)
		};

		//each PS_HEIGHTFIELD_COLLISION blocks deal with one test. 
		// blockIdx.z == 0 for regular particles, 1 for diffuse particles
		result = mCudaContext->launchKernel(kernelFunction, PxgParticleSystemKernelGridDim::PS_HEIGHTFIELD_COLLISION, numTests, 2, PxgParticleSystemKernelBlockDim::PS_HEIGHTFIELD_COLLISION, 1, 1, 0, /*mStream*/particleStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_heightfieldCollisonLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG

		result = mCudaContext->streamSynchronize(particleStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_heightfieldCollisonLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

#endif
	}
}

void PxgGpuNarrowphaseCore::softbodyOtherContactApplyCollisionToSimMeshMapping(
	PxgDevicePointer<float4> contactsd,
	PxgDevicePointer<float4> barycentricsd,
	PxgDevicePointer<PxgFemOtherContactInfo> contactInfosd,
	PxgDevicePointer<PxU32> totalNumCountsd,
	PxgDevicePointer<PxU32> prevNumCountsd
)
{
	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();
	PxgSoftBodyCore* softBodyCore = mGpuContext->mGpuSoftBodyCore;
	CUdeviceptr softBodiesd = simulationCore->getSoftBodyBuffer().getDevicePtr();
	const PxU32 maxNumContacts = softBodyCore->mMaxContacts;
	
	CUstream softbodyStream = softBodyCore->getStream();
	CUresult result;

	CUfunction sbContactRemapKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_OTHER_CONTACT_REMAP_TO_SIM);

	PxCudaKernelParam kernelParams[] =
	{

		PX_CUDA_KERNEL_PARAM(softBodiesd),
		PX_CUDA_KERNEL_PARAM(maxNumContacts),
		PX_CUDA_KERNEL_PARAM(contactsd),
		PX_CUDA_KERNEL_PARAM(barycentricsd),
		PX_CUDA_KERNEL_PARAM(contactInfosd),
		PX_CUDA_KERNEL_PARAM(totalNumCountsd),
		PX_CUDA_KERNEL_PARAM(prevNumCountsd)
	};

	PxU32 numWarpsPerBlock = 16;
	PxU32 numBlocks = 1024;
	//each warp deal with one test. 
	result = mCudaContext->launchKernel(sbContactRemapKernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, softbodyStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_other_contact_remap_to_simLaunch fail to launch kernel!!\n");
}

void PxgGpuNarrowphaseCore::softbodyFemContactApplyCollisionToSimMeshMapping(
	PxgDevicePointer<float4> barycentrics0d, PxgDevicePointer<float4> barycentrics1d,
	PxgDevicePointer<PxgFemFemContactInfo> contactInfosd, PxgDevicePointer<PxU32> totalNumCountsd, PxgDevicePointer<PxU32> prevNumCountsd,
	bool isSelfCollision, bool isCloth)
{
	PX_ASSERT(!isSelfCollision || !isCloth);

	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();
	PxgSoftBodyCore* softBodyCore = mGpuContext->mGpuSoftBodyCore;
	CUdeviceptr softBodiesd = simulationCore->getSoftBodyBuffer().getDevicePtr();
	const PxU32 maxNumContacts = softBodyCore->mMaxContacts;

	CUstream softbodyStream = softBodyCore->getStream();
	CUresult result;

	CUfunction sbContactRemapKernelFunction =
		mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_FEM_CONTACT_REMAP_TO_SIM);

	PxCudaKernelParam kernelParams[] = {
		PX_CUDA_KERNEL_PARAM(softBodiesd),
		PX_CUDA_KERNEL_PARAM(barycentrics0d),
		PX_CUDA_KERNEL_PARAM(barycentrics1d),
		PX_CUDA_KERNEL_PARAM(contactInfosd),
		PX_CUDA_KERNEL_PARAM(totalNumCountsd),
		PX_CUDA_KERNEL_PARAM(prevNumCountsd),
		PX_CUDA_KERNEL_PARAM(maxNumContacts)
	};

	PxU32 gridYDim = 2;
	PxU32 numWarpsPerBlock = 4;
	PxU32 numBlocks = 8192;
	
	if(isCloth)
	{
		gridYDim = 1;
		numWarpsPerBlock = 16;
		numBlocks = 1024;
	}
	else if(isSelfCollision)
	{
		numWarpsPerBlock = 16;
		numBlocks = 1024;
	}

	// each warp deals with one test.
	result = mCudaContext->launchKernel(sbContactRemapKernelFunction, numBlocks, gridYDim, 1, WARP_SIZE, numWarpsPerBlock, 1, 0,
										softbodyStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

	if(result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_fem_contact_remap_to_simLaunch fail to launch kernel!!\n");
}

//soft body with sphere/plane/capsule/box/convex
void PxgGpuNarrowphaseCore::testSDKSoftbody(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKSoftbody", 0);

	PX_UNUSED(renderOutput);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PX_ASSERT(cmInputs);
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();

	const PxsCachedTransform* transformCached = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());
	const PxReal* contactDistanced = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());
	PX_ASSERT(transformCached);

	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();
	PxgSoftBodyCore* softBodyCore = mGpuContext->mGpuSoftBodyCore;

	PxgCudaPagedLinearAllocator<PxgHeapMemoryAllocator>& stackAlloc = softBodyCore->getStackAllocator();
	stackAlloc.mMutex.lock();

	CUstream softbodyStream = softBodyCore->getStream();

	CUdeviceptr gpuMidphasePairsNumOnDevice = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));
	
	const PxU32 stackSizeBytes = mCollisionStackSizeBytes;
	CUdeviceptr gpuIntermStack = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(256, stackSizeBytes));

	CUdeviceptr stackSizeNeededOnDevice = softBodyCore->mStackSizeNeededOnDevice.getDevicePtr();

	//initialize gpu variables
	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, softbodyStream);


	CUdeviceptr softBodiesd = simulationCore->getSoftBodyBuffer().getDevicePtr();

	CUresult result;

	// soft body midphasekernel
	{
		CUfunction sbMidphasekernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_MIDPHASE_PRIMITIVES);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(boundsd),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(softBodiesd),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice)
		};

		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = (numTests + numWarpsPerBlock - 1) / numWarpsPerBlock;
		//each warp deal with one test. 
		result = mCudaContext->launchKernel(sbMidphasekernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, softbodyStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_midphaseGeneratePairsLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG

		result = mCudaContext->streamSynchronize(softbodyStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_midphaseGeneratePairsLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

#if GPU_NP_VISUALIZATION

		PxU32 numPairs;
		mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));

		if (numPairs > 0)
		{
			PxArray<uint4> pairs(numPairs);

			mCudaContext->memcpyDtoH(pairs.begin(), gpuIntermStack, sizeof(uint4) * numPairs);

			int bob = 0;
			PX_UNUSED(bob);
		}
#endif //GPU_NP_VISUALIZATION

#endif //GPU_NP_DEBUG
	}	

	{
		CUdeviceptr totalNumCountsd = softBodyCore->getRigidContactCount().getDevicePtr();
		CUdeviceptr prevNumCountsd = softBodyCore->getPrevRigidContactCount().getDevicePtr();
		mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), softbodyStream);
		CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();

		//PxU32 preNumContacts;
		//mCudaContext->memcpyDtoH(&preNumContacts, totalNumCountsd, sizeof(PxU32));

		CUdeviceptr pairs = simulationCore->getRigidSoftBodyFilters();
		const PxU32 nbPairs = simulationCore->getNbRigidSoftBodyFilters();

		PxgFEMContactWriter writer(softBodyCore);

		CUfunction sbCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_PRIMITIVES_CG);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(softBodiesd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(pairs),
			PX_CUDA_KERNEL_PARAM(nbPairs),
			PX_CUDA_KERNEL_PARAM(stackSizeNeededOnDevice),
			PX_CUDA_KERNEL_PARAM(writer)		
		};

		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = 512;
		//each warp deal with one test. 
		result = mCudaContext->launchKernel(sbCollisionKernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, softbodyStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_primitiveContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG

		result = mCudaContext->streamSynchronize(softbodyStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_primitiveContactGenLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

#if GPU_NP_VISUALIZATION
		PxU32 numContacts;
		mCudaContext->memcpyDtoH(&numContacts, totalNumCountsd, sizeof(PxU32));

		if (numContacts > 0)
		{
			PxArray<float4> points(numContacts);
			PxArray<float4> normalPen(numContacts);

			mCudaContext->memcpyDtoH(points.begin(), contactsd, sizeof(float4) * numContacts);
			mCudaContext->memcpyDtoH(normalPen.begin(), normalPensd, sizeof(float4) * numContacts);


			const PxU32 color = 0xff00ffff;
			const PxReal size = 0.1f;
			PxRenderOutput& out = *renderOutput;
		
			for (PxU32 i = 0; i < numContacts; ++i)
			{
				PxVec3 a(points[i].x, points[i].y, points[i].z);
				PxVec3 n(normalPen[i].x, normalPen[i].y, normalPen[i].z);
				const PxReal pen = normalPen[i].w;

				PxVec3 b = a + n * pen;

				const PxVec3 up(0.f, size, 0.f);
				const PxVec3 right(size, 0.f, 0.f);
				const PxVec3 forwards(0.f, 0.f, size);

				const PxMat44 m(PxIdentity);

				out << color << m << PxRenderOutput::LINES << a + up << a - up;
				out << color << m << PxRenderOutput::LINES << a + right << a - right;
				out << color << m << PxRenderOutput::LINES << a + forwards << a - forwards;


				out << color << m << PxRenderOutput::LINES << b + up << b - up;
				out << color << m << PxRenderOutput::LINES << b + right << b - right;
				out << color << m << PxRenderOutput::LINES << b + forwards << b - forwards;

				out << color << m << PxRenderOutput::LINES << a << b;
			}
		}

		int bob = 0;
		PX_UNUSED(bob);
#endif //GPU_NP_VISUALIZATION

#endif //GPU_NP_DEBUG
	}

	{
		CUdeviceptr contactsd = softBodyCore->getRigidContacts().getDevicePtr();
		CUdeviceptr barycentricsd = softBodyCore->getRigidBarycentrics().getDevicePtr();
		CUdeviceptr contactInfosd = softBodyCore->getRigidContactInfos().getDevicePtr();
		CUdeviceptr totalNumCountsd = softBodyCore->getRigidContactCount().getDevicePtr();
		CUdeviceptr prevNumCountsd = softBodyCore->getPrevRigidContactCount().getDevicePtr();

		softbodyOtherContactApplyCollisionToSimMeshMapping(
			contactsd,
			barycentricsd,
			contactInfosd,
			totalNumCountsd,
			prevNumCountsd
		);

		//PxU32 preNumContacts;
		//mCudaContext->memcpyDtoH(&preNumContacts, totalNumCountsd, sizeof(PxU32));
	}

	stackAlloc.reset();
	stackAlloc.mMutex.unlock();
}


void PxgGpuNarrowphaseCore::testSDKSoftbodies(PxgGpuContactManagers& gpuManagers, const PxU32 numTests,
	PxRenderOutput* renderOutput)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKSoftbodies", 0);

	PX_UNUSED(renderOutput);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PX_ASSERT(cmInputs);
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();

	const PxsCachedTransform* transformCached = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());
	const PxReal* contactDistanced = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	PX_ASSERT(transformCached);

	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();
	PxgSoftBodyCore* softBodyCore = mGpuContext->mGpuSoftBodyCore; 
	CUstream softbodyStream = softBodyCore->getStream();

	PxgDevicePointer<PxgSoftBody> softBodiesd = simulationCore->getSoftBodyBuffer().getTypedDevicePtr();

	PxgDevicePointer<float4> barycentrics0d = softBodyCore->getFemBarycentrics0().getTypedDevicePtr();
	PxgDevicePointer<float4> barycentrics1d = softBodyCore->getFemBarycentrics1().getTypedDevicePtr();
	PxgDevicePointer<PxgFemFemContactInfo> contactInfosd = softBodyCore->getVolumeContactOrVTContactInfos().getTypedDevicePtr();
	PxgDevicePointer<PxU32> totalNumCountsd = softBodyCore->getVolumeContactOrVTContactCount().getTypedDevicePtr();
	PxgDevicePointer<PxU32> prevNumCountsd = softBodyCore->getPrevFemContactCount().getTypedDevicePtr();

	mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), softbodyStream);

	PxgSoftBodyContactWriter writer(softBodyCore);

	CUresult result;

	// soft body first midphase kernel(each output is the index to a bound in one tree and tetrahedron index in other tree)
	{
		CUfunction sbsbMidphaseFirstkernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SB_MIDPHASE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(boundsd),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(softBodiesd),
			PX_CUDA_KERNEL_PARAM(writer)
		};

		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_SBMIDPHASE;
		//blockIdx.y deal with one test. 
		result = mCudaContext->launchKernel(sbsbMidphaseFirstkernelFunction, numBlocks, numTests, 2, WARP_SIZE, numWarpsPerBlock, 1, 0, softbodyStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_sbMidphaseGeneratePairsLaunch fail to launch kernel!!\n");
#if GPU_NP_DEBUG

		result = mCudaContext->streamSynchronize(softbodyStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_sbMidphaseGeneratePairsLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

#if GPU_NP_VISUALIZATION
		PxU32 numContacts;
		mCudaContext->memcpyDtoH(&numContacts, totalNumCountsd, sizeof(PxU32));

		if (numContacts > 0)
		{
			PxArray<float4> points(numContacts);
			PxArray<float4> normalPen(numContacts);

			mCudaContext->memcpyDtoH(points.begin(), contactsd, sizeof(float4) * numContacts);
			mCudaContext->memcpyDtoH(normalPen.begin(), normalPensd, sizeof(float4) * numContacts);

			const PxReal size = 0.1f;
			PxRenderOutput& out = *renderOutput;

			const PxU32 color = 0xff00ffff;
			for (PxU32 j = 0; j < numContacts; ++j)
			{
				PxVec3 a(points[j].x, points[j].y, points[j].z);
				PxVec3 n(normalPen[j].x, normalPen[j].y, normalPen[j].z);
				const PxReal pen = normalPen[j].w;

				PxVec3 b = a + n * pen;

				const PxVec3 up(0.f, size, 0.f);
				const PxVec3 right(size, 0.f, 0.f);
				const PxVec3 forwards(0.f, 0.f, size);

				const PxMat44 m(PxIdentity);

				out << color << m << PxRenderOutput::LINES << a + up << a - up;
				out << color << m << PxRenderOutput::LINES << a + right << a - right;
				out << color << m << PxRenderOutput::LINES << a + forwards << a - forwards;

				out << color << m << PxRenderOutput::LINES << b + up << b - up;
				out << color << m << PxRenderOutput::LINES << b + right << b - right;
				out << color << m << PxRenderOutput::LINES << b + forwards << b - forwards;

				//out << color << m << RenderOutput::LINES << a << b;
			}
		}

#endif //GPU_NP_VISUALIZATION

#endif //GPU_NP_DEBUG
	}

	softbodyFemContactApplyCollisionToSimMeshMapping(barycentrics0d, barycentrics1d, 
		contactInfosd, totalNumCountsd, prevNumCountsd, false, false);
}

void PxgGpuNarrowphaseCore::testSDKSoftbodyCloth(PxgGpuContactManagers& gpuManagers,
	const PxU32 numTests, PxRenderOutput* renderOutput)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKSoftbodyCloth", 0);

	PX_UNUSED(renderOutput);

	bool supportClothVertexCollision = true;
	bool supportClothTriangleCollision = false;

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PX_ASSERT(cmInputs);
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();

	const PxsCachedTransform* transformCached = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());
	const PxReal* contactDistanced = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	PX_ASSERT(transformCached);

	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();
	PxgSoftBodyCore* softBodyCore = mGpuContext->mGpuSoftBodyCore;
	PxgFEMClothCore* femClothCore = mGpuContext->mGpuFEMClothCore;


	CUstream softbodyStream = softBodyCore->getStream();
	//OK. If we sync the soft bodies and particles here, we don't get crashes!
	synchronizeStreams(mCudaContext, femClothCore->getStream(), softbodyStream);

	PxgCudaPagedLinearAllocator<PxgHeapMemoryAllocator>& stackAlloc = softBodyCore->getStackAllocator();
	stackAlloc.mMutex.lock();

	CUdeviceptr gpuMidphasePairsNumOnDevice = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));

	const PxU32 stackSizeBytes = mCollisionStackSizeBytes;
	CUdeviceptr gpuIntermStack = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(256, stackSizeBytes));
	CUdeviceptr stackSizeNeededOnDevice = softBodyCore->mStackSizeNeededOnDevice.getDevicePtr();

	//initialize gpu variables
	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, softbodyStream);

	PxgDevicePointer<PxgSoftBody> softBodiesd = simulationCore->getSoftBodyBuffer().getTypedDevicePtr();
	PxgDevicePointer<PxgFEMCloth> femClothesd = simulationCore->getFEMClothBuffer().getTypedDevicePtr();

	PxgDevicePointer<float4> contactsd = softBodyCore->getClothVsSoftBodyContacts().getTypedDevicePtr();
	PxgDevicePointer<float4> barycentricsd1 = softBodyCore->getClothVsSoftBodyBarycentrics1().getTypedDevicePtr();
	PxgDevicePointer<PxgFemFemContactInfo> contactInfosd = softBodyCore->getClothVsSoftBodyContactInfos().getTypedDevicePtr();
	PxgDevicePointer<PxU32> totalNumCountsd = softBodyCore->getClothVsSoftBodyContactCount().getTypedDevicePtr();
	PxgDevicePointer<PxU32> prevNumCountsd = softBodyCore->getPrevClothVsSoftBodyContactCount().getTypedDevicePtr();

	CUdeviceptr pairs = simulationCore->getClothSoftBodyFilters();
	const PxU32 nbPairs = simulationCore->getNbClothSoftBodyFilters();

	mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), softbodyStream);

	const PxReal nbCollisionPairUpdatesPerTimestep = static_cast<PxReal>(simulationCore->getMaxNbCollisionPairUpdatesPerTimestep());

	CUresult result;

	PxgSoftBodyContactWriter writer(softBodyCore, femClothCore);

	//!
	//! softbody tet - cloth triangle collision
	//! 

	if (supportClothTriangleCollision)
	{
		// soft body midphase kernel(each output is the index to a bound in one tree and tetrahedron index in other tree)
		{
			CUfunction sbmeshMidphaseFirstkernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_CLOTH_MIDPHASE);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(toleranceLength),
				PX_CUDA_KERNEL_PARAM(cmInputs),
				PX_CUDA_KERNEL_PARAM(transformCached),
				PX_CUDA_KERNEL_PARAM(boundsd),
				PX_CUDA_KERNEL_PARAM(contactDistanced),
				PX_CUDA_KERNEL_PARAM(gpuShapes),
				PX_CUDA_KERNEL_PARAM(softBodiesd),
				PX_CUDA_KERNEL_PARAM(femClothesd),
				PX_CUDA_KERNEL_PARAM(stackSizeBytes),
				PX_CUDA_KERNEL_PARAM(gpuIntermStack),
				PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice)
				
			};

			PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
			PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_SBMIDPHASE;
			//each warp deal with one test. 
			result = mCudaContext->launchKernel(sbmeshMidphaseFirstkernelFunction, numBlocks, numTests, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, softbodyStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_clothMidphaseGeneratePairsLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG

			result = mCudaContext->streamSynchronize(softbodyStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_clothMidphaseGeneratePairsLaunch kernel fail!!!\n");

			PX_ASSERT(result == CUDA_SUCCESS);

			PxU32 numPairs;
			mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));

			if (numPairs > 0)
			{
				PxArray<uint4> midphasePairs(numPairs);

				mCudaContext->memcpyDtoH(midphasePairs.begin(), gpuIntermStack, sizeof(uint4) * numPairs);

				//Dma back the bound
				PxBounds3 bounds[3];
				mCudaContext->memcpyDtoH(bounds, (CUdeviceptr)boundsd, sizeof(PxBounds3) * 3);

				const PxVec3 min = bounds[1].minimum;
				const PxVec3 max = bounds[1].maximum;

				PxRenderOutput& out = *renderOutput;

				{
					const PxVec3 extents = bounds[1].getExtents();

					const PxVec3 center = bounds[1].getCenter();
					PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

					out << absPose << PxDebugBox(extents);
				}

				{
					const PxVec3 extents = bounds[2].getExtents();

					const PxVec3 center = bounds[2].getCenter();
					PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

					out << absPose << PxDebugBox(extents);
				}

				/*const PxVec3 dif = max - min;
				PxVec3 v[8];
				v[0] = min;
				v[1] = PxVec3(min.x, min.y, min.z + dif.z);
				v[2] = PxVec3(min.x, min.y + dif.y, min.z);
				v[3] = PxVec3(min.x + dif.y, min.y, min.z);

				v[]*/

				int bob = 0;
				PX_UNUSED(bob);
			}
#endif
		}

		{
			CUfunction sbClothContactGenkernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_CLOTH_CG);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(toleranceLength),
				PX_CUDA_KERNEL_PARAM(cmInputs),
				PX_CUDA_KERNEL_PARAM(transformCached),
				PX_CUDA_KERNEL_PARAM(contactDistanced),
				PX_CUDA_KERNEL_PARAM(gpuShapes),
				PX_CUDA_KERNEL_PARAM(softBodiesd),
				PX_CUDA_KERNEL_PARAM(femClothesd),
				PX_CUDA_KERNEL_PARAM(gpuIntermStack),
				PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
				PX_CUDA_KERNEL_PARAM(stackSizeBytes),
				PX_CUDA_KERNEL_PARAM(pairs),
				PX_CUDA_KERNEL_PARAM(nbPairs),
				PX_CUDA_KERNEL_PARAM(stackSizeNeededOnDevice),
				PX_CUDA_KERNEL_PARAM(writer)
			};

			PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
			PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_MESHCG;
			//each warp deal with one test. 
			result = mCudaContext->launchKernel(sbClothContactGenkernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, softbodyStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_clothContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG

			result = mCudaContext->streamSynchronize(softbodyStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_clothContactGenLaunch kernel fail!!!\n");

			PX_ASSERT(result == CUDA_SUCCESS);

			if (0)
			{
				PxU32 numContacts;
				mCudaContext->memcpyDtoH(&numContacts, totalNumCountsd, sizeof(PxU32));
				CUdeviceptr normalPensd = softBodyCore->getClothVsSoftBodyNormalPens().getDevicePtr();
				drawContacts(*renderOutput, contactsd, normalPensd, numContacts);
			}
#endif

		}
	}
	
	//!
	//! softbody tet - cloth vertex collision
	//! 

	if (supportClothVertexCollision)
	{
		//initialize gpu variables
		mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, softbodyStream);

		//generate vert contacts
		{
			CUfunction sbClothMidphaseFirstkernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_CLOTH_VERT_MIDPHASE);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(toleranceLength),
				PX_CUDA_KERNEL_PARAM(cmInputs),
				PX_CUDA_KERNEL_PARAM(transformCached),
				PX_CUDA_KERNEL_PARAM(boundsd),
				PX_CUDA_KERNEL_PARAM(contactDistanced),
				PX_CUDA_KERNEL_PARAM(gpuShapes),
				PX_CUDA_KERNEL_PARAM(softBodiesd),
				PX_CUDA_KERNEL_PARAM(femClothesd),
				PX_CUDA_KERNEL_PARAM(stackSizeBytes),
				PX_CUDA_KERNEL_PARAM(nbCollisionPairUpdatesPerTimestep),
				PX_CUDA_KERNEL_PARAM(gpuIntermStack),
				PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice)
			};

			PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
			PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_SBMIDPHASE;
			//each warp deal with one test. 
			result = mCudaContext->launchKernel(sbClothMidphaseFirstkernelFunction, numBlocks, numTests, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, softbodyStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_clothVertMidphaseGeneratePairsLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
			result = mCudaContext->streamSynchronize(softbodyStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_clothVertMidphaseGeneratePairsLaunch kernel fail!!!\n");

			PxU32 numPairs;
			mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));

			if (numPairs > 0)
			{
				int bob = 0;
				PX_UNUSED(bob);
			}
#endif
		}

		{
			CUfunction sbClothVertContactGenKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_CLOTH_VERT_CG);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(toleranceLength),
				PX_CUDA_KERNEL_PARAM(cmInputs),
				PX_CUDA_KERNEL_PARAM(transformCached),
				PX_CUDA_KERNEL_PARAM(contactDistanced),
				PX_CUDA_KERNEL_PARAM(gpuShapes),
				PX_CUDA_KERNEL_PARAM(softBodiesd),
				PX_CUDA_KERNEL_PARAM(femClothesd),
				PX_CUDA_KERNEL_PARAM(stackSizeBytes),
				PX_CUDA_KERNEL_PARAM(gpuIntermStack),
				PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
				PX_CUDA_KERNEL_PARAM(pairs),
				PX_CUDA_KERNEL_PARAM(nbPairs),
				PX_CUDA_KERNEL_PARAM(stackSizeNeededOnDevice),
				PX_CUDA_KERNEL_PARAM(writer)
			};

			PxU32 numBlocks = 512;
			//each thread deal with one test. 
			result = mCudaContext->launchKernel(sbClothVertContactGenKernelFunction, numBlocks, 1, 1, 256, 1, 1, 0, softbodyStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_clothVertContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
			result = mCudaContext->streamSynchronize(softbodyStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_clothVertContactGenLaunch kernel fail!!!\n");

			PX_ASSERT(result == CUDA_SUCCESS);

			if (0)
			{
				PxU32 numContacts;
				mCudaContext->memcpyDtoH(&numContacts, totalNumCountsd, sizeof(PxU32));

				CUdeviceptr normalPensd = softBodyCore->getClothVsSoftBodyNormalPens().getDevicePtr();
				drawContacts(*renderOutput, contactsd, normalPensd, numContacts);
			}

#endif
		}
	}

	softbodyFemContactApplyCollisionToSimMeshMapping(contactsd, barycentricsd1, contactInfosd,
		totalNumCountsd, prevNumCountsd, false, true);

	stackAlloc.reset();
	stackAlloc.mMutex.unlock();
}

void PxgGpuNarrowphaseCore::testSDKSoftbodySdfTrimesh(PxgGpuContactManagers& gpuManagers,
	const PxU32 numTests, PxRenderOutput* renderOutput)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKSoftbodySdfTrimesh", 0);

	PX_UNUSED(renderOutput);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PX_ASSERT(cmInputs);
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();

	const PxsCachedTransform* transformCached = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());
	const PxReal* contactDistanced = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());
	PX_ASSERT(transformCached);

	CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();

	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();
	PxgSoftBodyCore* softBodyCore = mGpuContext->mGpuSoftBodyCore;

	CUstream softbodyStream = softBodyCore->getStream();

	//initialize gpu variables
	CUdeviceptr softBodiesd = simulationCore->getSoftBodyBuffer().getDevicePtr();

	PxgFEMContactWriter writer(softBodyCore);

	CUresult result;


	{
		CUdeviceptr totalNumCountsd = softBodyCore->getRigidContactCount().getDevicePtr();
		CUdeviceptr prevNumCountsd = softBodyCore->getPrevRigidContactCount().getDevicePtr();
		mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), softbodyStream);
	}

	CUdeviceptr filterPairs = simulationCore->getRigidSoftBodyFilters();
	const PxU32 nbFilterPairs = simulationCore->getNbRigidSoftBodyFilters();

	// soft body first midphase kernel(each output is the index to a bound in one tree and tetrahedron index in other tree)
	{
		CUfunction sbmeshMidphaseFirstkernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_SDF_MESH_MIDPHASE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(boundsd),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(softBodiesd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),

			PX_CUDA_KERNEL_PARAM(filterPairs),
			PX_CUDA_KERNEL_PARAM(nbFilterPairs),

			PX_CUDA_KERNEL_PARAM(writer)
		};

		PxU32 numBlocks = numTests;
		//each warp deal with one test. 
		result = mCudaContext->launchKernel(sbmeshMidphaseFirstkernelFunction, numBlocks, 1, 1, 1024, 1, 1, 0, softbodyStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_sdfMeshMidphaseGeneratePairsLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG && 0

		result = mCudaContext->streamSynchronize(softbodyStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_midphaseGeneratePairsLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

		PxU32 numPairs;
		mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));

		if (numPairs > 0)
		{
			PxArray<uint4> pairs(numPairs);

			mCudaContext->memcpyDtoH(pairs.begin(), gpuIntermStack, sizeof(uint4) * numPairs);

			//Dma back the bound
			PxBounds3 bounds[3];
			mCudaContext->memcpyDtoH(bounds, (CUdeviceptr)boundsd, sizeof(PxBounds3) * 3);

			const PxVec3 min = bounds[1].minimum;
			const PxVec3 max = bounds[1].maximum;

			PxRenderOutput& out = *renderOutput;

			{
				const PxVec3 extents = bounds[1].getExtents();

				const PxVec3 center = bounds[1].getCenter();
				PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

				out << absPose << PxDebugBox(extents);
			}

			{
				const PxVec3 extents = bounds[2].getExtents();

				const PxVec3 center = bounds[2].getCenter();
				PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

				out << absPose << PxDebugBox(extents);
			}

			/*const PxVec3 dif = max - min;
			PxVec3 v[8];
			v[0] = min;
			v[1] = PxVec3(min.x, min.y, min.z + dif.z);
			v[2] = PxVec3(min.x, min.y + dif.y, min.z);
			v[3] = PxVec3(min.x + dif.y, min.y, min.z);
			v[]*/

			int bob = 0;
			PX_UNUSED(bob);
		}
#endif
	}

	{
		PxgDevicePointer<float4> contactsd = softBodyCore->getRigidContacts().getTypedDevicePtr();
		PxgDevicePointer<float4> barycentricsd = softBodyCore->getRigidBarycentrics().getTypedDevicePtr();
		PxgDevicePointer<PxgFemOtherContactInfo> contactInfosd = softBodyCore->getRigidContactInfos().getTypedDevicePtr();
		PxgDevicePointer<PxU32> totalNumCountsd = softBodyCore->getRigidContactCount().getTypedDevicePtr();
		PxgDevicePointer<PxU32> prevNumCountsd = softBodyCore->getPrevRigidContactCount().getTypedDevicePtr();

		softbodyOtherContactApplyCollisionToSimMeshMapping(
			contactsd,
			barycentricsd,
			contactInfosd,
			totalNumCountsd,
			prevNumCountsd
		);
	}
}

void PxgGpuNarrowphaseCore::testSDKSoftbodyTrimesh(PxgGpuContactManagers& gpuManagers, 
	const PxU32 numTests, PxRenderOutput* renderOutput)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.testSDKSoftbodyTrimesh", 0);

	PX_UNUSED(renderOutput);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PX_ASSERT(cmInputs);
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();

	const PxsCachedTransform* transformCached = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());
	const PxReal* contactDistanced = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());
	PX_ASSERT(transformCached);

	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();
	PxgSoftBodyCore* softBodyCore = mGpuContext->mGpuSoftBodyCore; 

	PxgCudaPagedLinearAllocator<PxgHeapMemoryAllocator>& stackAlloc = softBodyCore->getStackAllocator();
	stackAlloc.mMutex.lock();

	CUstream softbodyStream = softBodyCore->getStream();

	CUdeviceptr gpuMidphasePairsNumOnDevice = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));

	const PxU32 stackSizeBytes = mCollisionStackSizeBytes;
	CUdeviceptr gpuIntermStack = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(256, stackSizeBytes));
	CUdeviceptr stackSizeNeededOnDevice = softBodyCore->mStackSizeNeededOnDevice.getDevicePtr();

	//initialize gpu variables
	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, softbodyStream);
	
	CUdeviceptr softBodiesd = simulationCore->getSoftBodyBuffer().getDevicePtr();
	

	CUresult result;

	// soft body first midphase kernel(each output is the index to a bound in one tree and tetrahedron index in other tree)
	{
		CUfunction sbmeshMidphaseFirstkernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_MESH_MIDPHASE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(boundsd),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(softBodiesd),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice)
		};

		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_SBMIDPHASE;
		//each warp deal with one test. 
		result = mCudaContext->launchKernel(sbmeshMidphaseFirstkernelFunction, numBlocks, numTests, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, softbodyStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_meshMidphaseGeneratePairsLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG

		result = mCudaContext->streamSynchronize(softbodyStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_midphaseGeneratePairsLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

		PxU32 numPairs;
		mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));

		if (numPairs > 0)
		{
			PxArray<uint4> pairs(numPairs);

			mCudaContext->memcpyDtoH(pairs.begin(), gpuIntermStack, sizeof(uint4) * numPairs);

			//Dma back the bound
			PxBounds3 bounds[3];
			mCudaContext->memcpyDtoH(bounds, (CUdeviceptr)boundsd, sizeof(PxBounds3) * 3);

			const PxVec3 min = bounds[1].minimum;
			const PxVec3 max = bounds[1].maximum;

			PxRenderOutput& out = *renderOutput;

			{
				const PxVec3 extents = bounds[1].getExtents();

				const PxVec3 center = bounds[1].getCenter();
				PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

				out << absPose << PxDebugBox(extents);
			}

			{
				const PxVec3 extents = bounds[2].getExtents();

				const PxVec3 center = bounds[2].getCenter();
				PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

				out << absPose << PxDebugBox(extents);
			}

			/*const PxVec3 dif = max - min;
			PxVec3 v[8];
			v[0] = min;
			v[1] = PxVec3(min.x, min.y, min.z + dif.z);
			v[2] = PxVec3(min.x, min.y + dif.y, min.z);
			v[3] = PxVec3(min.x + dif.y, min.y, min.z);
			v[]*/

			int bob = 0;
			PX_UNUSED(bob);
		}
#endif
	}

	{
		CUdeviceptr totalNumCountsd = softBodyCore->getRigidContactCount().getDevicePtr();
		CUdeviceptr prevNumCountsd = softBodyCore->getPrevRigidContactCount().getDevicePtr();
		mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), softbodyStream);
		CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();

		CUdeviceptr pairs = simulationCore->getRigidSoftBodyFilters();
		const PxU32 nbPairs = simulationCore->getNbRigidSoftBodyFilters();

		PxgFEMContactWriter writer(softBodyCore);

		CUfunction sbmeshContactGenkernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_MESH_CG);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(softBodiesd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(pairs),
			PX_CUDA_KERNEL_PARAM(nbPairs),
			PX_CUDA_KERNEL_PARAM(stackSizeNeededOnDevice),
			PX_CUDA_KERNEL_PARAM(writer)
		};

		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_MESHCG;
		//each warp deal with one test. 
		result = mCudaContext->launchKernel(sbmeshContactGenkernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, softbodyStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_meshContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG

		result = mCudaContext->streamSynchronize(softbodyStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_meshContactGenLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

		PxU32 numContacts;
		mCudaContext->memcpyDtoH(&numContacts, totalNumCountsd, sizeof(PxU32));

		if (numContacts > 0)
		{
			PxU32 prevNumContacts;
			mCudaContext->memcpyDtoH(&prevNumContacts, prevNumCountsd, sizeof(PxU32));

			PxArray<float4> points(numContacts);
			PxArray<float4> normalPen(numContacts);

			CUdeviceptr contactsd = softBodyCore->getRigidContacts().getDevicePtr();
			CUdeviceptr normalPensd = softBodyCore->getRigidNormalPens().getDevicePtr();

			mCudaContext->memcpyDtoH(points.begin(), contactsd, sizeof(float4) * numContacts);
			mCudaContext->memcpyDtoH(normalPen.begin(), normalPensd, sizeof(float4) * numContacts);

			const PxReal size = 0.1f;
			PxRenderOutput& out = *renderOutput;

			PxgSoftBody tSoftBody;
			mCudaContext->memcpyDtoH(&tSoftBody, softBodiesd, sizeof(PxgSoftBody));

			const PxU32 softBodyByteSize = 37632;
			PxU8 meshData[softBodyByteSize];
			mCudaContext->memcpyDtoH(meshData, (CUdeviceptr)tSoftBody.mTetMeshData, sizeof(PxU8) * softBodyByteSize);

			float4 * tetmeshVerts;
			uint4 * tetmeshTetIndices;

			PxU8 * tetmeshGeomPtr = reinterpret_cast<PxU8*>(meshData);

			const uint4 nbVerts_nbTets_maxDepth_nbBv32TreeNodes = *reinterpret_cast<const uint4 *>(tetmeshGeomPtr);
			tetmeshGeomPtr += sizeof(uint4);

			//Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(tetmeshGeomPtr);
			tetmeshGeomPtr += sizeof(const BV32DataPacked)* nbVerts_nbTets_maxDepth_nbBv32TreeNodes.w;

			tetmeshVerts = reinterpret_cast<float4 *>(tetmeshGeomPtr);
			tetmeshGeomPtr += sizeof(float4) * nbVerts_nbTets_maxDepth_nbBv32TreeNodes.x;

			tetmeshTetIndices = reinterpret_cast<uint4 *>(tetmeshGeomPtr);
			


			PX_UNUSED(tetmeshVerts);
			PX_UNUSED(tetmeshTetIndices);

			/*const PxU32 tetId = 829;
			uint4 Ind = tetmeshTetIndices[tetId];
			float4 worldV0 = tetmeshVerts[Ind.x];
			float4 worldV1 = tetmeshVerts[Ind.y];
			float4 worldV2 = tetmeshVerts[Ind.z];
			float4 worldV3 = tetmeshVerts[Ind.w];

			{
				const PxU32 color = 0xff00ff00;

				const PxVec3 a(worldV0.x, worldV0.y, worldV0.z);
				const PxVec3 b(worldV1.x, worldV1.y, worldV1.z);
				const PxVec3 c(worldV2.x, worldV2.y, worldV2.z);
				const PxVec3 d(worldV3.x, worldV3.y, worldV3.z);

				const PxVec3 up(0.f, size, 0.f);
				const PxVec3 right(size, 0.f, 0.f);
				const PxVec3 forwards(0.f, 0.f, size);

				const PxMat44 m(PxIdentity);

				out << color << m << PxRenderOutput::LINES << a << b;
				out << color << m << PxRenderOutput::LINES << a << c;
				out << color << m << PxRenderOutput::LINES << a << d;
				out << color << m << PxRenderOutput::LINES << b << c;
				out << color << m << PxRenderOutput::LINES << b << d;
				out << color << m << PxRenderOutput::LINES << c << d;			
			}*/


			const PxU32 color = 0xff00ffff;
			const PxU32 color2 = 0xff0000ff;

			//PxReal maxY = 0.f;
			for (PxU32 i = prevNumContacts; i < numContacts; ++i)
			{
				PxVec3 a(points[i].x, points[i].y, points[i].z);
				PxVec3 n(normalPen[i].x, normalPen[i].y, normalPen[i].z);
				const PxReal pen = normalPen[i].w;

				PxVec3 b = a + n * pen;

				const PxVec3 up(0.f, size, 0.f);
				const PxVec3 right(size, 0.f, 0.f);
				const PxVec3 forwards(0.f, 0.f, size);

				const PxMat44 m(PxIdentity);

				out << color << m << PxRenderOutput::LINES << a + up << a - up;
				out << color << m << PxRenderOutput::LINES << a + right << a - right;
				out << color << m << PxRenderOutput::LINES << a + forwards << a - forwards;


				out << color2 << m << PxRenderOutput::LINES << b + up << b - up;
				out << color2 << m << PxRenderOutput::LINES << b + right << b - right;
				out << color2 << m << PxRenderOutput::LINES << b + forwards << b - forwards;

				//out << color << m << PxRenderOutput::LINES << a << b;
			}
		}

		int bob = 0;
		PX_UNUSED(bob);
#endif
	}

	{
		CUdeviceptr contactsd = softBodyCore->getRigidContacts().getDevicePtr();
		CUdeviceptr barycentricsd = softBodyCore->getRigidBarycentrics().getDevicePtr();
		CUdeviceptr contactInfosd = softBodyCore->getRigidContactInfos().getDevicePtr();
		CUdeviceptr totalNumCountsd = softBodyCore->getRigidContactCount().getDevicePtr();
		CUdeviceptr prevNumCountsd = softBodyCore->getPrevRigidContactCount().getDevicePtr();

		softbodyOtherContactApplyCollisionToSimMeshMapping(
			contactsd,
			barycentricsd,
			contactInfosd,
			totalNumCountsd,
			prevNumCountsd
		);
	}

	stackAlloc.reset();
	stackAlloc.mMutex.unlock();
}

void PxgGpuNarrowphaseCore::testSDKSoftbodyHeightfield(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, 
	PxRenderOutput* renderOutput)
{
	PX_UNUSED(renderOutput);
	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PX_ASSERT(cmInputs);
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	const PxsCachedTransform* transformCached = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxReal* contactDistanced = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());
	PX_ASSERT(transformCached);

	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();

	PxgSoftBodyCore* softbodyCore = mGpuContext->mGpuSoftBodyCore;

	PxgCudaPagedLinearAllocator<PxgHeapMemoryAllocator>& stackAlloc = softbodyCore->getStackAllocator();
	stackAlloc.mMutex.lock();

	CUstream softbodyStream = softbodyCore->getStream();

	CUdeviceptr gpuMidphasePairsNumOnDevice = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));

	const PxU32 stackSizeBytes = mCollisionStackSizeBytes;
	CUdeviceptr gpuIntermStack = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(256, stackSizeBytes));
	CUdeviceptr stackSizeNeededOnDevice = softbodyCore->mStackSizeNeededOnDevice.getDevicePtr();

	//initialize gpu variables
	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, softbodyStream);

	CUdeviceptr softBodiesd = simulationCore->getSoftBodyBuffer().getDevicePtr();
	

	CUresult result;

	// soft body first midphase kernel(each output is the index to a bound in one tree and tetrahedron index in other tree)
	{
		CUfunction sbHFMidphaseFirstkernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_HF_MIDPHASE);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(softBodiesd),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice)
		};

		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_SBMIDPHASE;
		//each warp deal with one test. 
		result = mCudaContext->launchKernel(sbHFMidphaseFirstkernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, softbodyStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_meshMidphaseGeneratePairsLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG

		result = mCudaContext->streamSynchronize(softbodyStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_midphaseGeneratePairsLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

		PxU32 numPairs;
		mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));

		if (numPairs > 0)
		{
			PxArray<uint4> pairs(numPairs);

			mCudaContext->memcpyDtoH(pairs.begin(), gpuIntermStack, sizeof(uint4) * numPairs);

			PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();
			const PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());

			//Dma back the bound
			PxBounds3 bounds[3];
			mCudaContext->memcpyDtoH(bounds, (CUdeviceptr)boundsd, sizeof(PxBounds3) * 3);

			const PxVec3 min = bounds[1].minimum;
			const PxVec3 max = bounds[1].maximum;

			PxRenderOutput& out = *renderOutput;

			{
				const PxVec3 extents = bounds[1].getExtents();

				const PxVec3 center = bounds[1].getCenter();
				PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

				out << absPose << PxDebugBox(extents);
			}

			{
				const PxVec3 extents = bounds[2].getExtents();

				const PxVec3 center = bounds[2].getCenter();
				PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

				out << absPose << PxDebugBox(extents);
			}

			/*const PxVec3 dif = max - min;
			PxVec3 v[8];
			v[0] = min;
			v[1] = PxVec3(min.x, min.y, min.z + dif.z);
			v[2] = PxVec3(min.x, min.y + dif.y, min.z);
			v[3] = PxVec3(min.x + dif.y, min.y, min.z);

			v[]*/

			int bob = 0;
			PX_UNUSED(bob);
		}
#endif
	}

	PxgSoftBodyCore* softBodyCore = mGpuContext->mGpuSoftBodyCore;
	CUdeviceptr contactsd = softBodyCore->getRigidContacts().getDevicePtr();
	CUdeviceptr contactInfosd = softBodyCore->getRigidContactInfos().getDevicePtr();
	CUdeviceptr barycentricsd = softBodyCore->getRigidBarycentrics().getDevicePtr();
	CUdeviceptr totalNumCountsd = softBodyCore->getRigidContactCount().getDevicePtr();
	CUdeviceptr prevNumCountsd = softBodyCore->getPrevRigidContactCount().getDevicePtr();
	mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), softbodyStream);

	{
		CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();
		
		PxgFEMContactWriter writer(softBodyCore);

		CUfunction sbHFContactGenkernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SB_HF_CG);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(softBodiesd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(stackSizeNeededOnDevice),
			PX_CUDA_KERNEL_PARAM(writer)
		};

		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_HFCG;
		//each 64 warps deal with one cm
		result = mCudaContext->launchKernel(sbHFContactGenkernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, softbodyStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_heightfieldContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG

		result = mCudaContext->streamSynchronize(softbodyStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sb_heightfieldContactGenLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

		PxU32 numContacts;
		mCudaContext->memcpyDtoH(&numContacts, totalNumCountsd, sizeof(PxU32));

		if (numContacts > 0)
		{
			PxArray<float4> points(numContacts);
			PxArray<float4> normalPen(numContacts);

			CUdeviceptr normalPensd = softBodyCore->getRigidNormalPens().getDevicePtr();

			mCudaContext->memcpyDtoH(points.begin(), contactsd, sizeof(float4) * numContacts);
			mCudaContext->memcpyDtoH(normalPen.begin(), normalPensd, sizeof(float4) * numContacts);

			const PxReal size = 0.1f;
			PxRenderOutput& out = *renderOutput;

			PxgSoftBody tSoftBody;
			mCudaContext->memcpyDtoH(&tSoftBody, softBodiesd, sizeof(PxgSoftBody));

			const PxU32 softBodyByteSize = 37632;
			PxU8 meshData[softBodyByteSize];
			mCudaContext->memcpyDtoH(meshData, (CUdeviceptr)tSoftBody.mTetMeshData, sizeof(PxU8) * softBodyByteSize);

			float4 * tetmeshVerts;
			uint4 * tetmeshTetIndices;

			PxU8 * tetmeshGeomPtr = reinterpret_cast<PxU8*>(meshData);

			const uint4 nbVerts_nbTets_maxDepth_nbBv32TreeNodes = *reinterpret_cast<const uint4 *>(tetmeshGeomPtr);
			tetmeshGeomPtr += sizeof(uint4);

			//Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(tetmeshGeomPtr);
			tetmeshGeomPtr += sizeof(const BV32DataPacked)* nbVerts_nbTets_maxDepth_nbBv32TreeNodes.w;

			tetmeshVerts = reinterpret_cast<float4 *>(tetmeshGeomPtr);
			tetmeshGeomPtr += sizeof(float4) * nbVerts_nbTets_maxDepth_nbBv32TreeNodes.x;

			tetmeshTetIndices = reinterpret_cast<uint4 *>(tetmeshGeomPtr);



			PX_UNUSED(tetmeshVerts);
			PX_UNUSED(tetmeshTetIndices);

			/*const PxU32 tetId = 829;
			uint4 Ind = tetmeshTetIndices[tetId];
			float4 worldV0 = tetmeshVerts[Ind.x];
			float4 worldV1 = tetmeshVerts[Ind.y];
			float4 worldV2 = tetmeshVerts[Ind.z];
			float4 worldV3 = tetmeshVerts[Ind.w];

			{
			const PxU32 color = 0xff00ff00;

			const PxVec3 a(worldV0.x, worldV0.y, worldV0.z);
			const PxVec3 b(worldV1.x, worldV1.y, worldV1.z);
			const PxVec3 c(worldV2.x, worldV2.y, worldV2.z);
			const PxVec3 d(worldV3.x, worldV3.y, worldV3.z);

			const PxVec3 up(0.f, size, 0.f);
			const PxVec3 right(size, 0.f, 0.f);
			const PxVec3 forwards(0.f, 0.f, size);

			const PxMat44 m(PxIdentity);

			out << color << m << RenderOutput::LINES << a << b;
			out << color << m << RenderOutput::LINES << a << c;
			out << color << m << RenderOutput::LINES << a << d;
			out << color << m << RenderOutput::LINES << b << c;
			out << color << m << RenderOutput::LINES << b << d;
			out << color << m << RenderOutput::LINES << c << d;
			}*/


			const PxU32 color = 0xff00ffff;

			//PxReal maxY = 0.f;
			for (PxU32 i = 0; i < numContacts; ++i)
			{
				PxVec3 a(points[i].x, points[i].y, points[i].z);
				PxVec3 n(normalPen[i].x, normalPen[i].y, normalPen[i].z);

				const PxVec3 up(0.f, size, 0.f);
				const PxVec3 right(size, 0.f, 0.f);
				const PxVec3 forwards(0.f, 0.f, size);

				const PxMat44 m(PxIdentity);

				out << color << m << PxRenderOutput::LINES << a + up << a - up;
				out << color << m << PxRenderOutput::LINES << a + right << a - right;
				out << color << m << PxRenderOutput::LINES << a + forwards << a - forwards;


				//const PxReal pen = normalPen[i].w;
				//PxVec3 b = a + n * pen;
				//out << color << m << RenderOutput::LINES << b + up << b - up;
				//out << color << m << RenderOutput::LINES << b + right << b - right;
				//out << color << m << RenderOutput::LINES << b + forwards << b - forwards;

				//out << color << m << RenderOutput::LINES << a << b;
			}
		}

		int bob = 0;
		PX_UNUSED(bob);
#endif
	}

	{
        softbodyOtherContactApplyCollisionToSimMeshMapping(
			contactsd,
			barycentricsd,
			contactInfosd,
			totalNumCountsd,
			prevNumCountsd
		);
	}

	stackAlloc.reset();
	stackAlloc.mMutex.unlock();
}

void PxgGpuNarrowphaseCore::testSDKFemClothSphere(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput)
{
	PX_UNUSED(renderOutput);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PX_ASSERT(cmInputs);
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();
	const PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());
	const PxsCachedTransform* transformCached = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxReal* contactDistanced = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());
	PX_ASSERT(transformCached);

	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();

	PxgFEMClothCore* femClothCore = mGpuContext->mGpuFEMClothCore;

	PxgCudaPagedLinearAllocator<PxgHeapMemoryAllocator>& stackAlloc = femClothCore->getStackAllocator();
	stackAlloc.mMutex.lock();

	CUstream femClothStream = femClothCore->getStream();

	CUdeviceptr gpuMidphasePairsNumOnDevice = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));

	const PxU32 stackSizeBytes = mCollisionStackSizeBytes;
	CUdeviceptr gpuIntermStack = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(256, stackSizeBytes));
	CUdeviceptr stackSizeNeededOnDevice = femClothCore->mStackSizeNeededOnDevice.getDevicePtr();

	//initialize gpu variables
	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, femClothStream);

	CUdeviceptr femClothesd = simulationCore->getFEMClothBuffer().getDevicePtr();

	CUresult result;
	if (1)	// fem cloth first midphase kernel(each output is the index to a bound in one tree and tetrahedron index in other tree)
	{
		CUfunction femClothMidphaseFirstkernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_MIDPHASE_PRIMITIVES);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(boundsd),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(femClothesd),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes)
		};

		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = (numTests + numWarpsPerBlock - 1) / numWarpsPerBlock;
		//each warp deal with one test. 
		result = mCudaContext->launchKernel(femClothMidphaseFirstkernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_midphaseGeneratePairsLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(femClothStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_midphaseGeneratePairsLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

		PxU32 numPairs;
		mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));

		if (numPairs > 0)
		{
			PxArray<uint4> pairs(numPairs);

			mCudaContext->memcpyDtoH(pairs.begin(), gpuIntermStack, sizeof(uint4) * numPairs);

			//PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();
			//const PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());

			//Dma back the bound
			PxBounds3 bounds[3];
			mCudaContext->memcpyDtoH(bounds, (CUdeviceptr)boundsd, sizeof(PxBounds3) * 3);

			const PxVec3 min = bounds[1].minimum;
			const PxVec3 max = bounds[1].maximum;

			PxRenderOutput& out = *renderOutput;

			{
				const PxVec3 extents = bounds[0].getExtents();

				const PxVec3 center = bounds[0].getCenter();
				PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

				out << absPose << PxDebugBox(extents);
			}

			{
				const PxVec3 extents = bounds[1].getExtents();

				const PxVec3 center = bounds[1].getCenter();
				PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

				out << absPose << PxDebugBox(extents);
			}

			/*const PxVec3 dif = max - min;
			PxVec3 v[8];
			v[0] = min;
			v[1] = PxVec3(min.x, min.y, min.z + dif.z);
			v[2] = PxVec3(min.x, min.y + dif.y, min.z);
			v[3] = PxVec3(min.x + dif.y, min.y, min.z);

			v[]*/

			int bob = 0;
			PX_UNUSED(bob);
		}
#endif
	}

	CUdeviceptr totalNumCountsd = femClothCore->getRigidContactCount().getDevicePtr();
	CUdeviceptr prevNumCountsd = femClothCore->getPrevRigidContactCount().getDevicePtr();
	mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), femClothStream);

	CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();

	CUdeviceptr pairs = simulationCore->getRigidClothFilters();
	const PxU32 nbPairs = simulationCore->getNbRigidClothFilters();

	PxgFEMContactWriter writer(femClothCore);

	if (1)	//contact gen
	{
		CUfunction fcTriangleCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_SPHERE_CG);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(femClothesd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(pairs),
			PX_CUDA_KERNEL_PARAM(nbPairs),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(stackSizeNeededOnDevice),
			PX_CUDA_KERNEL_PARAM(writer)
		};

		PxU32 numThreadsPerBlock = 128;
		PxU32 numBlocks = 1024;
		//each warp deal with one test. 
		result = mCudaContext->launchKernel(fcTriangleCollisionKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_SphereContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(femClothStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_SphereContactGenLaunch kernel fail!!!\n");
#endif
	}

	if (0) // cloth vertex vs. sphere contact gen.
	{
		CUfunction fcVertCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_VERT_SPHERE_CG);

		PxCudaKernelParam vertKernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(femClothesd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
			PX_CUDA_KERNEL_PARAM(pairs),
			PX_CUDA_KERNEL_PARAM(nbPairs),

			PX_CUDA_KERNEL_PARAM(writer)
		};

		const PxU32 numVertBlocks = numTests;
		//const PxU32 numWarpPerBlock = 16;
		const PxU32 numWarpPerBlock = 1;
		const PxU32 numThreadsPerWarp = 32;

		//each thread deal with a vert
		result = mCudaContext->launchKernel(fcVertCollisionKernelFunction, numVertBlocks, 1, 1, numThreadsPerWarp, numWarpPerBlock, 1, 0, femClothStream, vertKernelParams, sizeof(vertKernelParams), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_SphereContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(femClothStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_SphereContactGenLaunch kernel fail!!!\n");

		femClothCore->drawContacts(*renderOutput);
#endif
	}

	stackAlloc.reset();
	stackAlloc.mMutex.unlock();
}

void PxgGpuNarrowphaseCore::testSDKFemClothPlane(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput)
{
	PX_UNUSED(renderOutput);

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PX_ASSERT(cmInputs);
	CUdeviceptr gpuShapes = mGpuShapesManager.mGpuShapesBuffer.getDevicePtr();
	CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();

	const PxsCachedTransform* transformCached = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxReal* contactDistanced = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());
	PX_ASSERT(transformCached);

	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();

	PxgFEMClothCore* femClothCore = mGpuContext->mGpuFEMClothCore;

	CUstream femClothStream = femClothCore->getStream();

	CUdeviceptr femClothesd = simulationCore->getFEMClothBuffer().getDevicePtr();

	CUresult result;

	{
		CUdeviceptr totalNumCountsd = femClothCore->getRigidContactCount().getDevicePtr();
		CUdeviceptr prevNumCountsd = femClothCore->getPrevRigidContactCount().getDevicePtr();
		mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), femClothStream);

		CUdeviceptr pairs = simulationCore->getRigidClothFilters();
		const PxU32 nbPairs = simulationCore->getNbRigidClothFilters();

		PxgFEMContactWriter writer(femClothCore);

		CUfunction fcVertCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_PLANE_VERTEX_CG);
		PxCudaKernelParam vertKernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(femClothesd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
			PX_CUDA_KERNEL_PARAM(pairs),
			PX_CUDA_KERNEL_PARAM(nbPairs),

			PX_CUDA_KERNEL_PARAM(writer)
		};

		const PxU32 numVertBlocks = numTests;
		const PxU32 numWarpPerBlock = 16;
		const PxU32 numThreadsPerWarp = 32;

		//each thread deal with a vert
		result = mCudaContext->launchKernel(fcVertCollisionKernelFunction, numVertBlocks, 1, 1, numThreadsPerWarp, numWarpPerBlock, 1, 0, femClothStream, vertKernelParams, sizeof(vertKernelParams), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_planeVertContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(femClothStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_planeVertContactGenLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

		//PxU32 numContacts;
		//mCudaContext->memcpyDtoH(&numContacts, totalNumCountsd, sizeof(PxU32));

		//if (numContacts > 0)
		//{
		//	drawContacts(*renderOutput, contactsd, normalPensd, numContacts);
		//}

		//femClothCore->drawContacts(*renderOutput);
#endif
	}
}


void PxgGpuNarrowphaseCore::testSDKFemClothBox(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput)
{
	PX_UNUSED(renderOutput);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PX_ASSERT(cmInputs);
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();
	const PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());
	const PxsCachedTransform* transformCached = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxReal* contactDistanced = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());
	PX_ASSERT(transformCached);

	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();

	PxgFEMClothCore* femClothCore = mGpuContext->mGpuFEMClothCore;

	PxgCudaPagedLinearAllocator<PxgHeapMemoryAllocator>& stackAlloc = femClothCore->getStackAllocator();
	stackAlloc.mMutex.lock();

	CUstream femClothStream = femClothCore->getStream();

	CUdeviceptr gpuMidphasePairsNumOnDevice = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));

	const PxU32 stackSizeBytes = mCollisionStackSizeBytes;
	CUdeviceptr gpuIntermStack = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(256, stackSizeBytes));
	CUdeviceptr stackSizeNeededOnDevice = femClothCore->mStackSizeNeededOnDevice.getDevicePtr();

	//initialize gpu variables
	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, femClothStream);

	CUdeviceptr femClothesd = simulationCore->getFEMClothBuffer().getDevicePtr();

	CUresult result;

	// fem cloth first midphase kernel(each output is the index to a bound in one tree and tetrahedron index in other tree)
	{
		CUfunction femClothMidphaseFirstkernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_MIDPHASE_PRIMITIVES);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(boundsd),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(femClothesd),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes)
		};

		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = (numTests + numWarpsPerBlock - 1) / numWarpsPerBlock;
		//each warp deal with one test. 
		result = mCudaContext->launchKernel(femClothMidphaseFirstkernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_midphaseGeneratePairsLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(femClothStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_midphaseGeneratePairsLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

		PxU32 numPairs;
		mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));
#endif
	}

	//contact gen
	if (1)
	{
		CUdeviceptr totalNumCountsd = femClothCore->getRigidContactCount().getDevicePtr();
		CUdeviceptr prevNumCountsd = femClothCore->getPrevRigidContactCount().getDevicePtr();
		mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), femClothStream);

		CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();

		CUdeviceptr pairs = simulationCore->getRigidClothFilters();
		const PxU32 nbPairs = simulationCore->getNbRigidClothFilters();
		CUfunction fcCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_BOX_TRIANGLE_CG);

		PxgFEMContactWriter writer(femClothCore);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(femClothesd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(pairs),
			PX_CUDA_KERNEL_PARAM(nbPairs),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(stackSizeNeededOnDevice),
			PX_CUDA_KERNEL_PARAM(writer)
		};

		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = 4096;
		//each warp deal with one test. 
		result = mCudaContext->launchKernel(fcCollisionKernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_boxTriangleContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(femClothStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_boxTriangleContactGenLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

		PxU32 numContacts;
		mCudaContext->memcpyDtoH(&numContacts, totalNumCountsd, sizeof(PxU32));

		PxRenderOutput& out = *renderOutput;
		const PxReal size = 0.01f;

		const PxMat44 m(PxIdentity);
		const PxVec3 up(0.f, size, 0.f);
		const PxVec3 right(size, 0.f, 0.f);
		const PxVec3 forwards(0.f, 0.f, size);

		const PxU32 color0 = 0xffffff00;
		//const PxU32 color1 = 0xffff0000;

		if (numContacts > 0)
		{
			//printf("numContacts %i MaxContacts %i\n", numContacts, femClothCore->getMaxContacts());
			PxArray<float4> points(numContacts);
			PxArray<float4> normalPen(numContacts);

			CUdeviceptr contactsd = femClothCore->getRigidContacts().getDevicePtr();
			CUdeviceptr normalPensd = femClothCore->getRigidNormalPens().getDevicePtr();

			mCudaContext->memcpyDtoH(points.begin(), contactsd, sizeof(float4) * numContacts);
			mCudaContext->memcpyDtoH(normalPen.begin(), normalPensd, sizeof(float4) * numContacts);

			const PxU32 color = 0xff00ffff;
			for (PxU32 i = 0; i < numContacts; ++i)
			{
				PxVec3 a(points[i].x, points[i].y, points[i].z);
				PxVec3 n(normalPen[i].x, normalPen[i].y, normalPen[i].z);
				const PxReal pen = normalPen[i].w;
				PxVec3 b = a - n * pen;

				out << color << m << PxRenderOutput::LINES << a + up << a - up;
				out << color << m << PxRenderOutput::LINES << a + right << a - right;
				out << color << m << PxRenderOutput::LINES << a + forwards << a - forwards;

				out << color0 << m << PxRenderOutput::LINES << b + up << b - up;
				out << color0 << m << PxRenderOutput::LINES << b + right << b - right;
				out << color0 << m << PxRenderOutput::LINES << b + forwards << b - forwards;

				out << color << m << PxRenderOutput::LINES << a << b;
			}
		}
#endif
	}

	if (1)
	{
		CUdeviceptr totalNumCountsd = femClothCore->getRigidContactCount().getDevicePtr();
		CUdeviceptr prevNumCountsd = femClothCore->getPrevRigidContactCount().getDevicePtr();
		mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), femClothStream);

		CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();

		CUdeviceptr pairs = simulationCore->getRigidClothFilters();
		const PxU32 nbPairs = simulationCore->getNbRigidClothFilters();

		PxgFEMContactWriter writer(femClothCore);

		CUfunction fcBoxVertCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_BOX_VERTEX_COLLISION);
		PxCudaKernelParam vertKernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(femClothesd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
			PX_CUDA_KERNEL_PARAM(pairs),
			PX_CUDA_KERNEL_PARAM(nbPairs),

			PX_CUDA_KERNEL_PARAM(writer)
		};

		const PxU32 numVertBlocks = numTests;
		const PxU32 numWarpPerBlock = 16;
		const PxU32 numThreadsPerWarp = 32;

		//each thread deal with a vert
		result = mCudaContext->launchKernel(fcBoxVertCollisionKernelFunction, numVertBlocks, 1, 1, numThreadsPerWarp, numWarpPerBlock, 1, 0, femClothStream, vertKernelParams, sizeof(vertKernelParams), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_boxVertexContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(femClothStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_boxVertexContactGenLaunch kernel fail!!!\n");

		PxU32 numContacts;
		mCudaContext->memcpyDtoH(&numContacts, totalNumCountsd, sizeof(PxU32));
		if (numContacts > 0)
		{
			int bob = 0;
			PX_UNUSED(bob);
		}
#endif
	}

	stackAlloc.reset();
	stackAlloc.mMutex.unlock();
}


void PxgGpuNarrowphaseCore::testSDKFemClothConvexes(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput)
{
	PX_UNUSED(renderOutput);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PX_ASSERT(cmInputs);
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();
	const PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());
	const PxsCachedTransform* transformCached = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxReal* contactDistanced = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());
	PX_ASSERT(transformCached);


	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();

	PxgFEMClothCore* femClothCore = mGpuContext->mGpuFEMClothCore;

	PxgCudaPagedLinearAllocator<PxgHeapMemoryAllocator>& stackAlloc = femClothCore->getStackAllocator();
	stackAlloc.mMutex.lock();

	CUstream femClothStream = femClothCore->getStream();

	CUdeviceptr gpuMidphasePairsNumOnDevice = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));

	const PxU32 stackSizeBytes = mCollisionStackSizeBytes;
	CUdeviceptr gpuIntermStack = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(256, stackSizeBytes));
	CUdeviceptr stackSizeNeededOnDevice = femClothCore->mStackSizeNeededOnDevice.getDevicePtr();

	// initialize gpu variables
	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, femClothStream);

	CUdeviceptr femClothesd = simulationCore->getFEMClothBuffer().getDevicePtr();
	CUresult result;

#if 1 // cloth triangle vs. convex rigid

	{
		CUfunction femClothMidphaseFirstkernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_MIDPHASE_PRIMITIVES);
		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(boundsd),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(femClothesd),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes)
		};

		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = (numTests + numWarpsPerBlock - 1) / numWarpsPerBlock;
		//each warp deal with one test. 
		result = mCudaContext->launchKernel(femClothMidphaseFirstkernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_midphaseGeneratePairsLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(femClothStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_midphaseGeneratePairsLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

		PxU32 numPairs;
		mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));

		if (0)
		{
			PxArray<uint4> pairs(numPairs);
			mCudaContext->memcpyDtoH(pairs.begin(), gpuIntermStack, sizeof(uint4) * numPairs);

			// Dma back the bound
			PxBounds3 bounds[3];
			mCudaContext->memcpyDtoH(bounds, (CUdeviceptr)boundsd, sizeof(PxBounds3) * 3);

			const PxVec3 min = bounds[1].minimum;
			const PxVec3 max = bounds[1].maximum;

			PxRenderOutput& out = *renderOutput;

			{
				const PxVec3 extents = bounds[1].getExtents();
				const PxVec3 center = bounds[1].getCenter();
				PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));
				out << absPose << PxDebugBox(extents);
			}

			{
				const PxVec3 extents = bounds[2].getExtents();
				const PxVec3 center = bounds[2].getCenter();
				PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

				out << absPose << PxDebugBox(extents);
			}

			int bob = 0;
			PX_UNUSED(bob);
		}
#endif
	}

	//! contact gen
	{
		CUdeviceptr totalNumCountsd = femClothCore->getRigidContactCount().getDevicePtr();
		CUdeviceptr prevNumCountsd = femClothCore->getPrevRigidContactCount().getDevicePtr();
		mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), femClothStream);

		CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();

		CUdeviceptr pairs = simulationCore->getRigidClothFilters();
		const PxU32 nbPairs = simulationCore->getNbRigidClothFilters();

		PxgFEMContactWriter writer(femClothCore);

		CUfunction fcCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_CONVEX_CG);
		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(femClothesd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(pairs),
			PX_CUDA_KERNEL_PARAM(nbPairs),
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(stackSizeNeededOnDevice),
			PX_CUDA_KERNEL_PARAM(writer)
		};

		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = 4096;
		//each warp deal with one test. 
		result = mCudaContext->launchKernel(fcCollisionKernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_convexContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(femClothStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_convexContactGenLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

		PxU32 numContacts;
		mCudaContext->memcpyDtoH(&numContacts, totalNumCountsd, sizeof(PxU32));

		if (numContacts)
		{
			CUdeviceptr contactsd = femClothCore->getRigidContacts().getDevicePtr();
			CUdeviceptr normalPensd = femClothCore->getRigidNormalPens().getDevicePtr();

			drawContacts(*renderOutput, contactsd, normalPensd, numContacts);
		}
#endif
	}

#endif // cloth triangle vs. convex rigid

#if 1 // cloth vertex vs. convex rigid

	{
		mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, femClothStream);

		CUfunction femClothMidphaseFirstkernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_MIDPHASE_VERTEX_PRIMS);
		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(numTests),
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(boundsd),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(femClothesd),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice)
		};

		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = 32;
		//each warp deal with one test. 
		result = mCudaContext->launchKernel(femClothMidphaseFirstkernelFunction, numBlocks, numTests, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_midphaseGenerateVertexPairsLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(femClothStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_midphaseGeneratePairsLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

		PxU32 numPairs;
		mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));

		if (0)//numPairs > 0)
		{
			PxArray<uint4> pairs(numPairs);

			mCudaContext->memcpyDtoH(pairs.begin(), gpuIntermStack, sizeof(uint4) * numPairs);

			//PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();
			//const PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());

			//Dma back the bound
			PxBounds3 bounds[3];
			mCudaContext->memcpyDtoH(bounds, (CUdeviceptr)boundsd, sizeof(PxBounds3) * 3);

			const PxVec3 min = bounds[1].minimum;
			const PxVec3 max = bounds[1].maximum;

			PxRenderOutput& out = *renderOutput;

			{
				const PxVec3 extents = bounds[1].getExtents();

				const PxVec3 center = bounds[1].getCenter();
				PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

				out << absPose << PxDebugBox(extents);
			}

			{
				const PxVec3 extents = bounds[2].getExtents();

				const PxVec3 center = bounds[2].getCenter();
				PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

				out << absPose << PxDebugBox(extents);
			}

			/*const PxVec3 dif = max - min;
			PxVec3 v[8];
			v[0] = min;
			v[1] = PxVec3(min.x, min.y, min.z + dif.z);
			v[2] = PxVec3(min.x, min.y + dif.y, min.z);
			v[3] = PxVec3(min.x + dif.y, min.y, min.z);

			v[]*/

			int bob = 0;
			PX_UNUSED(bob);
		}
#endif
	}
	
	//! contact gen		
	{
		CUdeviceptr totalNumCountsd = femClothCore->getRigidContactCount().getDevicePtr();
		CUdeviceptr prevNumCountsd = femClothCore->getPrevRigidContactCount().getDevicePtr();
		mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), femClothStream);

		CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();

		CUdeviceptr pairs = simulationCore->getRigidClothFilters();
		const PxU32 nbPairs = simulationCore->getNbRigidClothFilters();

		PxgFEMContactWriter writer(femClothCore);

		CUfunction fcCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_CONVEX_VERTEX_COLLISION);
		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(toleranceLength),
			PX_CUDA_KERNEL_PARAM(cmInputs),
			PX_CUDA_KERNEL_PARAM(transformCached),
			PX_CUDA_KERNEL_PARAM(contactDistanced),
			PX_CUDA_KERNEL_PARAM(restDistanced),
			PX_CUDA_KERNEL_PARAM(gpuShapes),
			PX_CUDA_KERNEL_PARAM(femClothesd),
			PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
			PX_CUDA_KERNEL_PARAM(gpuIntermStack),
			PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
			PX_CUDA_KERNEL_PARAM(pairs),
			PX_CUDA_KERNEL_PARAM(nbPairs),			
			PX_CUDA_KERNEL_PARAM(stackSizeBytes),
			PX_CUDA_KERNEL_PARAM(stackSizeNeededOnDevice),
			PX_CUDA_KERNEL_PARAM(writer)
		};

		PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
		PxU32 numBlocks = 4096;
		//each warp deal with one test. 
		result = mCudaContext->launchKernel(fcCollisionKernelFunction, numBlocks, 1, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_convexVertexContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(femClothStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_convexVertexContactGenLaunch kernel fail!!!\n");

		PX_ASSERT(result == CUDA_SUCCESS);

		/*PxU32 numContacts;
		mCudaContext->memcpyDtoH(&numContacts, totalNumCountsd, sizeof(PxU32));

		if (numContacts > 0)
		{
			drawContacts(*renderOutput, contactsd, normalPensd, numContacts);
		}*/
		//femClothCore->drawContacts(*renderOutput);
#endif
	}

#endif // cloth vertex vs. convex rigid

	stackAlloc.reset();
	stackAlloc.mMutex.unlock();
}

void PxgGpuNarrowphaseCore::testSDKFemClothCloth(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput)
{
	// FEMCloth-FEMCloth collision handled in PxgFEMClothCore.cpp
	PX_UNUSED(renderOutput);
	PX_UNUSED(gpuManagers);
	PX_UNUSED(numTests);
}

void PxgGpuNarrowphaseCore::testSDKFemClothSdfTrimesh(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput)
{
	PX_UNUSED(renderOutput);

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PX_ASSERT(cmInputs);
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	const PxsCachedTransform* transformCached = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxReal* contactDistanced = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());


	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();
	PxgFEMClothCore* femClothCore = mGpuContext->mGpuFEMClothCore;	
	CUstream femClothStream = femClothCore->getStream();

	CUdeviceptr femClothesd = simulationCore->getFEMClothBuffer().getDevicePtr();

	CUdeviceptr totalNumCountsd = femClothCore->getRigidContactCount().getDevicePtr();
	CUdeviceptr prevNumCountsd = femClothCore->getPrevRigidContactCount().getDevicePtr();

	mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), femClothStream);

	CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();

	CUdeviceptr pairs = simulationCore->getRigidClothFilters();
	const PxU32 nbPairs = simulationCore->getNbRigidClothFilters();

	PxgFEMContactWriter writer(femClothCore);

	CUfunction fcCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_SDF_MESH_CG);
	PxCudaKernelParam kernelParams[] =
	{
		PX_CUDA_KERNEL_PARAM(numTests),
		PX_CUDA_KERNEL_PARAM(cmInputs),
		PX_CUDA_KERNEL_PARAM(transformCached),
		PX_CUDA_KERNEL_PARAM(contactDistanced),
		PX_CUDA_KERNEL_PARAM(restDistanced),
		PX_CUDA_KERNEL_PARAM(gpuShapes),
		PX_CUDA_KERNEL_PARAM(femClothesd),
		PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),

		PX_CUDA_KERNEL_PARAM(pairs),
		PX_CUDA_KERNEL_PARAM(nbPairs),

		PX_CUDA_KERNEL_PARAM(writer)
	};

	const PxU32 numBlocks = numTests;
	
	//each warp deal with one test. 
	PxCUresult result = mCudaContext->launchKernel(fcCollisionKernelFunction, numBlocks, 1, 1, 1024, 1, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_sdfMeshContactGenLaunch fail to launch kernel!!\n");	
}

void PxgGpuNarrowphaseCore::testSDKFemClothTrimesh(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput)
{
	PX_UNUSED(renderOutput);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PX_ASSERT(cmInputs);
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();
	const PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());
	const PxsCachedTransform* transformCached = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxReal* contactDistanced = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());
	PX_ASSERT(transformCached);

	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();
	
	PxgFEMClothCore* femClothCore = mGpuContext->mGpuFEMClothCore;
	
	PxgCudaPagedLinearAllocator<PxgHeapMemoryAllocator>& stackAlloc = femClothCore->getStackAllocator();
	stackAlloc.mMutex.lock();

	CUstream femClothStream = femClothCore->getStream();

	CUdeviceptr gpuMidphasePairsNumOnDevice = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));

	const PxU32 stackSizeBytes = mCollisionStackSizeBytes;
	CUdeviceptr gpuIntermStack = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(256, stackSizeBytes));
	CUdeviceptr stackSizeNeededOnDevice = femClothCore->mStackSizeNeededOnDevice.getDevicePtr();

	//initialize gpu variables
	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, femClothStream);

	CUdeviceptr femClothesd = simulationCore->getFEMClothBuffer().getDevicePtr();

	CUresult result;

	// fem cloth first midphase kernel(each output is the index to a bound in one tree and tetrahedron index in other tree)
	if (1)
	{
		{
			CUfunction femClothMidphaseFirstkernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_MESH_MIDPHASE);
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(toleranceLength),
				PX_CUDA_KERNEL_PARAM(cmInputs),
				PX_CUDA_KERNEL_PARAM(transformCached),
				PX_CUDA_KERNEL_PARAM(boundsd),
				PX_CUDA_KERNEL_PARAM(contactDistanced),
				PX_CUDA_KERNEL_PARAM(gpuShapes),
				PX_CUDA_KERNEL_PARAM(femClothesd),
				PX_CUDA_KERNEL_PARAM(stackSizeBytes),
				PX_CUDA_KERNEL_PARAM(gpuIntermStack),
				PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice)
			};

			PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
			PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_SBMIDPHASE;
			//each warp deal with one test. 
			result = mCudaContext->launchKernel(femClothMidphaseFirstkernelFunction, numBlocks, numTests, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_meshMidphaseGeneratePairsLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
			result = mCudaContext->streamSynchronize(femClothStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_midphaseGeneratePairsLaunch kernel fail!!!\n");

			PX_ASSERT(result == CUDA_SUCCESS);

			PxU32 numPairs;
			mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));

			if (0)//numPairs > 0)
			{
				PxArray<uint4> pairs(numPairs);

				mCudaContext->memcpyDtoH(pairs.begin(), gpuIntermStack, sizeof(uint4) * numPairs);

				//PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();
				//const PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());

				//Dma back the bound
				PxBounds3 bounds[3];
				mCudaContext->memcpyDtoH(bounds, (CUdeviceptr)boundsd, sizeof(PxBounds3) * 3);

				const PxVec3 min = bounds[1].minimum;
				const PxVec3 max = bounds[1].maximum;

				PxRenderOutput& out = *renderOutput;

				{
					const PxVec3 extents = bounds[0].getExtents();

					const PxVec3 center = bounds[0].getCenter();
					PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

					out << absPose << PxDebugBox(extents);
				}

				{
					const PxVec3 extents = bounds[1].getExtents();

					const PxVec3 center = bounds[1].getCenter();
					PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

					out << absPose << PxDebugBox(extents);
				}

				int bob = 0;
				PX_UNUSED(bob);
			}
#endif
		}

		//contact gen
		if (1)
		{
			CUdeviceptr totalNumCountsd = femClothCore->getRigidContactCount().getDevicePtr();
			CUdeviceptr prevNumCountsd = femClothCore->getPrevRigidContactCount().getDevicePtr();

			mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), femClothStream);

			CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();
			
			CUdeviceptr pairs = simulationCore->getRigidClothFilters();
			const PxU32 nbPairs = simulationCore->getNbRigidClothFilters();

			PxgFEMContactWriter writer(femClothCore);

			CUfunction fcCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_MESH_CG);
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(toleranceLength),
				PX_CUDA_KERNEL_PARAM(cmInputs),
				PX_CUDA_KERNEL_PARAM(transformCached),
				PX_CUDA_KERNEL_PARAM(contactDistanced),
				PX_CUDA_KERNEL_PARAM(restDistanced),
				PX_CUDA_KERNEL_PARAM(gpuShapes),
				PX_CUDA_KERNEL_PARAM(femClothesd),
				PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
				PX_CUDA_KERNEL_PARAM(stackSizeBytes),
				PX_CUDA_KERNEL_PARAM(gpuIntermStack),
				PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),

				PX_CUDA_KERNEL_PARAM(pairs),
				PX_CUDA_KERNEL_PARAM(nbPairs),

				PX_CUDA_KERNEL_PARAM(stackSizeNeededOnDevice),
				PX_CUDA_KERNEL_PARAM(writer)
			};

			PxU32 numBlocks = 2048;
			result = mCudaContext->launchKernel(fcCollisionKernelFunction, numBlocks, 1, 1, 128, 1, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_meshContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
			result = mCudaContext->streamSynchronize(femClothStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_meshContactGenLaunch kernel fail!!!\n");

			PX_ASSERT(result == CUDA_SUCCESS);

			/*PxU32 numContacts;
			mCudaContext->memcpyDtoH(&numContacts, totalNumCountsd, sizeof(PxU32));


			if (numContacts > 0)
			{

				drawContacts(*renderOutput, contactsd, normalPensd, numContacts);
			}*/
#endif
		}
	}

	if (1)
	{
		//initialize gpu variables
		mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, femClothStream);

		//vertex collision
		{
			CUfunction femClothMidphaseFirstkernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_MIDPHASE_VERTEX_MESH);
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(numTests),
				PX_CUDA_KERNEL_PARAM(toleranceLength),
				PX_CUDA_KERNEL_PARAM(cmInputs),
				PX_CUDA_KERNEL_PARAM(transformCached),
				PX_CUDA_KERNEL_PARAM(boundsd),
				PX_CUDA_KERNEL_PARAM(contactDistanced),
				PX_CUDA_KERNEL_PARAM(gpuShapes),
				PX_CUDA_KERNEL_PARAM(femClothesd),
				PX_CUDA_KERNEL_PARAM(stackSizeBytes),
				PX_CUDA_KERNEL_PARAM(gpuIntermStack),
				PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice)
			};

			PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
			PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_SBMIDPHASE;
			//each warp deal with one test. 
			result = mCudaContext->launchKernel(femClothMidphaseFirstkernelFunction, numBlocks, numTests, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_midphaseVertexMeshLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
			result = mCudaContext->streamSynchronize(femClothStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_midphaseVertexMeshLaunch kernel fail!!!\n");

			PX_ASSERT(result == CUDA_SUCCESS);

			PxU32 numPairs;
			mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));

			if (numPairs > 0)
			{
				PxArray<uint4> pairs(numPairs);

				mCudaContext->memcpyDtoH(pairs.begin(), gpuIntermStack, sizeof(uint4) * numPairs);

				//PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();
				//const PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());

				//Dma back the bound
				PxBounds3 bounds[3];
				mCudaContext->memcpyDtoH(bounds, (CUdeviceptr)boundsd, sizeof(PxBounds3) * 3);

				const PxVec3 min = bounds[1].minimum;
				const PxVec3 max = bounds[1].maximum;

				PxRenderOutput& out = *renderOutput;

				{
					const PxVec3 extents = bounds[0].getExtents();

					const PxVec3 center = bounds[0].getCenter();
					PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

					out << absPose << PxDebugBox(extents);
				}

				{
					const PxVec3 extents = bounds[1].getExtents();

					const PxVec3 center = bounds[1].getCenter();
					PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

					out << absPose << PxDebugBox(extents);
				}

				int bob = 0;
				PX_UNUSED(bob);
			}
#endif
		}

		//contact gen
		{
			CUdeviceptr totalNumCountsd = femClothCore->getRigidContactCount().getDevicePtr();
			CUdeviceptr prevNumCountsd = femClothCore->getPrevRigidContactCount().getDevicePtr();

			mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), femClothStream);

			CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();
			
			PxgFEMContactWriter writer(femClothCore);

			CUfunction fcCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_MESH_VERTEX_CG);
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(toleranceLength),
				PX_CUDA_KERNEL_PARAM(cmInputs),
				PX_CUDA_KERNEL_PARAM(transformCached),
				PX_CUDA_KERNEL_PARAM(contactDistanced),
				PX_CUDA_KERNEL_PARAM(restDistanced),
				PX_CUDA_KERNEL_PARAM(gpuShapes),
				PX_CUDA_KERNEL_PARAM(femClothesd),
				PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
				PX_CUDA_KERNEL_PARAM(stackSizeBytes),
				PX_CUDA_KERNEL_PARAM(gpuIntermStack),
				PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
				PX_CUDA_KERNEL_PARAM(stackSizeNeededOnDevice),

				PX_CUDA_KERNEL_PARAM(writer)
			};

			PxU32 numBlocks = 512;
			//each thread deal with one test. 
			result = mCudaContext->launchKernel(fcCollisionKernelFunction, numBlocks, 1, 1, 256, 1, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_meshVertexContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG
			result = mCudaContext->streamSynchronize(femClothStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_meshVertexContactGenLaunch kernel fail!!!\n");

			PX_ASSERT(result == CUDA_SUCCESS);

			femClothCore->drawContacts(*renderOutput);

			PxU32 numContacts;
			mCudaContext->memcpyDtoH(&numContacts, totalNumCountsd, sizeof(PxU32));

			if (numContacts > 0)
			{
				CUdeviceptr contactsd = femClothCore->getRigidContacts().getDevicePtr();
				CUdeviceptr normalPensd = femClothCore->getRigidNormalPens().getDevicePtr();

				drawContacts(*renderOutput, contactsd, normalPensd, numContacts);
			}
#endif
		}
	}

	stackAlloc.reset();
	stackAlloc.mMutex.unlock();
}


void PxgGpuNarrowphaseCore::testSDKFemClothHeightfield(PxgGpuContactManagers& gpuManagers, const PxU32 numTests, PxRenderOutput* renderOutput)
{
	PX_UNUSED(renderOutput);

	const PxReal toleranceLength = mNphaseImplContext->getToleranceLength();

	PxScopedCudaLock lock(*mCudaContextManager);

	const PxgContactManagerInput* cmInputs = reinterpret_cast<const PxgContactManagerInput*>(gpuManagers.mContactManagerInputData.getDevicePtr());
	PX_ASSERT(cmInputs);
	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	const PxsCachedTransform* transformCached = reinterpret_cast<PxsCachedTransform*>(mGpuTransformCache.getDevicePtr());
	const PxReal* contactDistanced = reinterpret_cast<PxReal*>(mGpuContactDistance.getDevicePtr());
	const PxReal* restDistanced = reinterpret_cast<PxReal*>(gpuManagers.mRestDistances.getDevicePtr());
	PX_ASSERT(transformCached);

	PxgSimulationCore* simulationCore = mNphaseImplContext->getSimulationCore();

	PxgFEMClothCore* femClothCore = mGpuContext->mGpuFEMClothCore;

	PxgCudaPagedLinearAllocator<PxgHeapMemoryAllocator>& stackAlloc = femClothCore->getStackAllocator();
	stackAlloc.mMutex.lock();

	CUstream femClothStream = femClothCore->getStream();

	CUdeviceptr gpuMidphasePairsNumOnDevice = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(sizeof(PxU32), sizeof(PxU32)));

	const PxU32 stackSize = mCollisionStackSizeBytes;
	CUdeviceptr gpuIntermStack = reinterpret_cast<CUdeviceptr>(stackAlloc.allocateAligned(256, stackSize));
	CUdeviceptr stackSizeNeededOnDevice = femClothCore->mStackSizeNeededOnDevice.getDevicePtr();

	//initialize gpu variables
	mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, femClothStream);

	CUdeviceptr femClothesd = simulationCore->getFEMClothBuffer().getDevicePtr();


	CUresult result;

	if (1)
	{
		// fem cloth first midphase kernel(each output is the index to a bound in one tree and tetrahedron index in other tree)
		{
			CUfunction femClothMidphaseFirstkernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_HF_MIDPHASE);
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(numTests),
				PX_CUDA_KERNEL_PARAM(toleranceLength),
				PX_CUDA_KERNEL_PARAM(cmInputs),
				PX_CUDA_KERNEL_PARAM(transformCached),
				PX_CUDA_KERNEL_PARAM(contactDistanced),
				PX_CUDA_KERNEL_PARAM(gpuShapes),
				PX_CUDA_KERNEL_PARAM(femClothesd),
				PX_CUDA_KERNEL_PARAM(gpuIntermStack),
				PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
				PX_CUDA_KERNEL_PARAM(stackSize)
			};

			PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
			PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_SBMIDPHASE;
			//each warp deal with one test. 
			result = mCudaContext->launchKernel(femClothMidphaseFirstkernelFunction, numBlocks, numTests, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_heightfieldMidphaseGeneratePairsLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG

			result = mCudaContext->streamSynchronize(femClothStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_heightfieldMidphaseGeneratePairsLaunch kernel fail!!!\n");

			PX_ASSERT(result == CUDA_SUCCESS);

			PxU32 numPairs;
			mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));

			if (numPairs > 0)
			{
				PxArray<uint4> pairs(numPairs);

				mCudaContext->memcpyDtoH(pairs.begin(), gpuIntermStack, sizeof(uint4) * numPairs);

				PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();
				const PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());

				//Dma back the bound
				PxBounds3 bounds[3];
				mCudaContext->memcpyDtoH(bounds, (CUdeviceptr)boundsd, sizeof(PxBounds3) * 3);

				const PxVec3 min = bounds[1].minimum;
				const PxVec3 max = bounds[1].maximum;

				PxRenderOutput& out = *renderOutput;

				{
					const PxVec3 extents = bounds[0].getExtents();

					const PxVec3 center = bounds[0].getCenter();
					PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

					out << absPose << PxDebugBox(extents);
				}

				{
					const PxVec3 extents = bounds[1].getExtents();

					const PxVec3 center = bounds[1].getCenter();
					PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

					out << absPose << PxDebugBox(extents);
				}

				/*const PxVec3 dif = max - min;
				PxVec3 v[8];
				v[0] = min;
				v[1] = PxVec3(min.x, min.y, min.z + dif.z);
				v[2] = PxVec3(min.x, min.y + dif.y, min.z);
				v[3] = PxVec3(min.x + dif.y, min.y, min.z);

				v[]*/

				int bob = 0;
				PX_UNUSED(bob);
			}
#endif
		}

		//contact gen
		{
			CUdeviceptr totalNumCountsd = femClothCore->getRigidContactCount().getDevicePtr();
			CUdeviceptr prevNumCountsd = femClothCore->getPrevRigidContactCount().getDevicePtr();

			mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), femClothStream);

			CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();
			
			PxgFEMContactWriter writer(femClothCore);

			CUfunction fcCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_HF_CG);
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(toleranceLength),
				PX_CUDA_KERNEL_PARAM(cmInputs),
				PX_CUDA_KERNEL_PARAM(transformCached),
				PX_CUDA_KERNEL_PARAM(contactDistanced),
				PX_CUDA_KERNEL_PARAM(restDistanced),
				PX_CUDA_KERNEL_PARAM(gpuShapes),
				PX_CUDA_KERNEL_PARAM(femClothesd),
				PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
				PX_CUDA_KERNEL_PARAM(gpuIntermStack),
				PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
				PX_CUDA_KERNEL_PARAM(stackSize),
				PX_CUDA_KERNEL_PARAM(stackSizeNeededOnDevice),
				PX_CUDA_KERNEL_PARAM(writer)				
			};
		
			PxU32 numBlocks = 2048; 
			result = mCudaContext->launchKernel(fcCollisionKernelFunction, numBlocks, 1, 1, 128, 1, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_heightfieldContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG

			result = mCudaContext->streamSynchronize(femClothStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_heightfieldContactGenLaunch kernel fail!!!\n");

			PX_ASSERT(result == CUDA_SUCCESS);

			PxU32 numContacts;
			mCudaContext->memcpyDtoH(&numContacts, totalNumCountsd, sizeof(PxU32));

			if (0)
			{
				CUdeviceptr contactsd = femClothCore->getRigidContacts().getDevicePtr();
				CUdeviceptr normalPensd = femClothCore->getRigidNormalPens().getDevicePtr();

				drawContacts(*renderOutput, contactsd, normalPensd, numContacts);
			}

#endif
		}
	}

	if (1)
	{
		// fem cloth first midphase kernel(each output is the index to a bound in one tree and tetrahedron index in other tree)
		{

			//initialize gpu variables
			mCudaContext->memsetD32Async(gpuMidphasePairsNumOnDevice, 0, 1, femClothStream);

			CUfunction femClothMidphaseFirstkernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_MIDPHASE_VERTEX_HF);
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(numTests),
				PX_CUDA_KERNEL_PARAM(toleranceLength),
				PX_CUDA_KERNEL_PARAM(cmInputs),
				PX_CUDA_KERNEL_PARAM(transformCached),
				PX_CUDA_KERNEL_PARAM(contactDistanced),
				PX_CUDA_KERNEL_PARAM(gpuShapes),
				PX_CUDA_KERNEL_PARAM(femClothesd),
				PX_CUDA_KERNEL_PARAM(gpuIntermStack),
				PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
				PX_CUDA_KERNEL_PARAM(stackSize)
			};

			PxU32 numWarpsPerBlock = MIDPHASE_WARPS_PER_BLOCK;
			PxU32 numBlocks = PxgSoftBodyKernelGridDim::SB_SBMIDPHASE;
			//each warp deal with one test. 
			result = mCudaContext->launchKernel(femClothMidphaseFirstkernelFunction, numBlocks, numTests, 1, WARP_SIZE, numWarpsPerBlock, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_midphaseVertexHeightfieldLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG

			result = mCudaContext->streamSynchronize(femClothStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_midphaseGeneratePairsLaunch kernel fail!!!\n");

			PX_ASSERT(result == CUDA_SUCCESS);

			PxU32 numPairs;
			mCudaContext->memcpyDtoH(&numPairs, gpuMidphasePairsNumOnDevice, sizeof(PxU32));

			if (numPairs > 0)
			{
				PxArray<uint4> pairs(numPairs);

				mCudaContext->memcpyDtoH(pairs.begin(), gpuIntermStack, sizeof(uint4) * numPairs);

				PxgTypedCudaBuffer<PxBounds3>* aabbBounds = mNphaseImplContext->getSimulationCore()->getBoundArrayBuffer();
				const PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(aabbBounds->getDevicePtr());

				//Dma back the bound
				PxBounds3 bounds[3];
				mCudaContext->memcpyDtoH(bounds, (CUdeviceptr)boundsd, sizeof(PxBounds3) * 3);

				const PxVec3 min = bounds[1].minimum;
				const PxVec3 max = bounds[1].maximum;

				PxRenderOutput& out = *renderOutput;

				{
					const PxVec3 extents = bounds[0].getExtents();

					const PxVec3 center = bounds[0].getCenter();
					PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

					out << absPose << PxDebugBox(extents);
				}

				{
					const PxVec3 extents = bounds[1].getExtents();

					const PxVec3 center = bounds[1].getCenter();
					PxTransform absPose = PxTransform(center, PxQuat(PxIdentity));

					out << absPose << PxDebugBox(extents);
				}

				/*const PxVec3 dif = max - min;
				PxVec3 v[8];
				v[0] = min;
				v[1] = PxVec3(min.x, min.y, min.z + dif.z);
				v[2] = PxVec3(min.x, min.y + dif.y, min.z);
				v[3] = PxVec3(min.x + dif.y, min.y, min.z);

				v[]*/

				int bob = 0;
				PX_UNUSED(bob);
			}
#endif
		}

		//contact gen
		{
			CUdeviceptr totalNumCountsd = femClothCore->getRigidContactCount().getDevicePtr();
			CUdeviceptr prevNumCountsd = femClothCore->getPrevRigidContactCount().getDevicePtr();

			mCudaContext->memcpyDtoDAsync(prevNumCountsd, totalNumCountsd, sizeof(PxU32), femClothStream);

			CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();
			
			PxgFEMContactWriter writer(femClothCore);

			CUfunction fcCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLOTH_HF_VERTEX_CG);
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(toleranceLength),
				PX_CUDA_KERNEL_PARAM(cmInputs),
				PX_CUDA_KERNEL_PARAM(transformCached),
				PX_CUDA_KERNEL_PARAM(contactDistanced),
				PX_CUDA_KERNEL_PARAM(restDistanced),
				PX_CUDA_KERNEL_PARAM(gpuShapes),
				PX_CUDA_KERNEL_PARAM(femClothesd),
				PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
				PX_CUDA_KERNEL_PARAM(gpuIntermStack),
				PX_CUDA_KERNEL_PARAM(gpuMidphasePairsNumOnDevice),
				PX_CUDA_KERNEL_PARAM(writer),
				PX_CUDA_KERNEL_PARAM(stackSize),
				PX_CUDA_KERNEL_PARAM(stackSizeNeededOnDevice)
			};

			PxU32 numBlocks = 512;
			//each thread deal with one test. 
			result = mCudaContext->launchKernel(fcCollisionKernelFunction, numBlocks, 1, 1, 256, 1, 1, 0, femClothStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_heightfieldVertexContactGenLaunch fail to launch kernel!!\n");

#if GPU_NP_DEBUG

			result = mCudaContext->streamSynchronize(femClothStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_heightfieldVertexContactGenLaunch kernel fail!!!\n");

			PX_ASSERT(result == CUDA_SUCCESS);

			PxU32 numContacts;
			mCudaContext->memcpyDtoH(&numContacts, totalNumCountsd, sizeof(PxU32));

			femClothCore->drawContacts(*renderOutput);

			if (numContacts > 0)
			{
				CUdeviceptr contactsd = femClothCore->getRigidContacts().getDevicePtr();
				CUdeviceptr normalPensd = femClothCore->getRigidNormalPens().getDevicePtr();

				drawContacts(*renderOutput, contactsd, normalPensd, numContacts);
			}

#endif
		}
	}

	stackAlloc.reset();
	stackAlloc.mMutex.unlock();
}

void PxgGpuNarrowphaseCore::updateFrictionPatches(PxgGpuContactManagers& gpuManagers, PxU32 count, PxU8* baseContactPatches, PxU8* baseFrictionPatches)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.updateFrictionPatches", 0);

	PxScopedCudaLock lock(*mCudaContextManager);

	PxsContactManagerOutput* cmOutputs = reinterpret_cast<PxsContactManagerOutput*>(gpuManagers.mContactManagerOutputData.getDevicePtr());

	CUresult result;
	{
		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::UPDATE_FRICTION_PATCHES);

		PxCudaKernelParam kernelParams_stage[] =
		{
			PX_CUDA_KERNEL_PARAM(count),
			PX_CUDA_KERNEL_PARAM((baseContactPatches)),
			PX_CUDA_KERNEL_PARAM((baseFrictionPatches)),
			PX_CUDA_KERNEL_PARAM(cmOutputs),
		};

		const PxU32 numThreadsPerBlock = 256;
		const PxU32 numBlocks = (count + (numThreadsPerBlock - 1)) / numThreadsPerBlock;
		//Each thread updates one cm
		result = mCudaContext->launchKernel(kernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams_stage, sizeof(kernelParams_stage), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU updateFrictionPatches fail to launch kernel!!\n", result);

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU updateFrictionPatches fail to launch kernel!!!\n", result);
#endif
	}
}

bool PxgGpuNarrowphaseCore::isMeshGPUCompatible(const PxTriangleMeshGeometryLL& meshLL)
{
	return meshLL.materialsLL.numIndices <= 1 &&
		_getMeshData(meshLL)->mGRB_BV32Tree != NULL &&
		_getMeshData(meshLL)->mAccumulatedTrianglesRef != NULL;
}

bool PxgGpuNarrowphaseCore::isClothMeshGPUCompatible(const PxTriangleMeshGeometryLL& meshLL)
{
	return _getMeshData(meshLL)->mGRB_BV32Tree != NULL &&
	       _getMeshData(meshLL)->mAccumulatedTrianglesRef != NULL;
}

bool PxgGpuNarrowphaseCore::isTetMeshGPUCompatible(const BVTetrahedronMesh* meshData)
{
	return meshData->isTetMeshGPUCompatible();
}

PxU32 PxgGpuNarrowphaseCore::addHull(const ConvexHullData& hull)
{
	PxScopedCudaLock lock(*mCudaContextManager);

	PxU32 idx =  mGeometryManager.addHull(hull);

	return idx;
}

PxU32 PxgGpuNarrowphaseCore::getHullIdxByHostPtr(const ConvexHullData* hull)
{
	PX_ASSERT(mGeometriesMap->find((size_t) hull));
	RefcountedRecord& rec = (*mGeometriesMap)[(size_t) hull];

	return rec.idx;
}

void PxgGpuNarrowphaseCore::removeGeometry(PxU32 idx)
{
	PxScopedCudaLock lock(*mCudaContextManager);
	mGeometryManager.removeGeometry(idx);
}

PxU32 PxgGpuNarrowphaseCore::addTriMesh(const TriangleMesh& triMesh)
{
	PxScopedCudaLock lock(*mCudaContextManager);

	PxU32 idx =  mGeometryManager.addTriMesh(triMesh);
	return idx;
}

PxU32 PxgGpuNarrowphaseCore::getTriMeshIdxByHostPtr(const TriangleMesh* triMesh)
{
	PX_ASSERT(mGeometriesMap->find((size_t) triMesh));
	RefcountedRecord& rec = (*mGeometriesMap)[(size_t)triMesh];
	return rec.idx;
}

PxU32 PxgGpuNarrowphaseCore::addHeightfield(const HeightFieldData& hf)
{
	PxScopedCudaLock lock(*mCudaContextManager);

	PxU32 idx =  mGeometryManager.addHeightfield(hf);
	return idx;
}

PxU32 PxgGpuNarrowphaseCore::getHeightfieldIdxByHostPtr(const HeightFieldData* hf)
{
	PX_ASSERT(mGeometriesMap->find((size_t) hf));
	RefcountedRecord& rec = (*mGeometriesMap)[(size_t)hf];
	return rec.idx;
}

void PxgGpuNarrowphaseCore::uploadDataChunksToGpu()
{
	CUstream stream = mStream;
	mGeometryManager.scheduleCopyHtoD(mCopyMan, *mCudaContext, stream);
	mGpuShapesManager.scheduleCopyHtoD(mCopyMan, mCudaContext, stream);
	mGpuMaterialManager.scheduleCopyHtoD(mCopyMan, mCudaContext, stream, sizeof(PxsMaterialData));
	mGpuFEMClothMaterialManager.scheduleCopyHtoD(mCopyMan, mCudaContext, stream, sizeof(PxsDeformableSurfaceMaterialData));
	mGpuFEMMaterialManager.scheduleCopyHtoD(mCopyMan, mCudaContext, stream, sizeof(PxsDeformableVolumeMaterialData));
	mCopyMan.dispatchCopy(stream, mCudaContextManager, mGpuKernelWranglerManager->getKernelWrangler());
}

void PxgGpuNarrowphaseCore::waitAndResetCopyQueues()
{
	mCopyMan.waitAndReset(mCudaContext);
	mGeometryManager.resetAfterMemcpyCompleted();
	mGpuShapesManager.releaseIDs();
	mGpuMaterialManager.releaseIDs();
	mGpuFEMMaterialManager.releaseIDs();
	mGpuFEMClothMaterialManager.releaseIDs();
}

void PxgGpuNarrowphaseCore::uploadDataChunksToGpuBp()
{
	if(mGpuContext->mGpuBp)
	{
		CUstream stream = mGpuContext->mGpuBp->getBpStream();
		mGpuPBDMaterialManager.scheduleCopyHtoD(mCopyManBp, mCudaContext, stream, sizeof(PxsPBDMaterialData));
		mCopyManBp.dispatchCopy(stream, mCudaContextManager, mGpuKernelWranglerManager->getKernelWrangler());
	}
}

void PxgGpuNarrowphaseCore::waitAndResetCopyQueuesBp()
{
	mCopyManBp.waitAndReset(mCudaContext);
	mGpuPBDMaterialManager.releaseIDs();
}

void PxgGpuNarrowphaseCore::registerShape(const PxNodeIndex& nodeIndex, const PxsShapeCore& shapeCore, const PxU32 transformCacheID, const bool isFemCloth, PxActor* actor)
{
	const PxGeometryType::Enum type = shapeCore.mGeometry.getType();
	const bool isBox = type == PxGeometryType::eBOX;
	const bool isCompatibleConvex = type == PxGeometryType::eCONVEXMESH && shapeCore.mGeometry.get<const PxConvexMeshGeometryLL>().gpuCompatible;
	const bool isCompatibleMesh = type == PxGeometryType::eTRIANGLEMESH && (isFemCloth || isMeshGPUCompatible(shapeCore.mGeometry.get<const PxTriangleMeshGeometryLL>()));
	const bool isCompatibleHF = type == PxGeometryType::eHEIGHTFIELD;
	const bool isCompatibleTetMesh = type == PxGeometryType::eTETRAHEDRONMESH && isTetMeshGPUCompatible(static_cast<const BVTetrahedronMesh*>(_getTetraMeshData(shapeCore.mGeometry.get<const PxTetrahedronMeshGeometryLL>())));
	const bool isCompatibleConvexCore = type == PxGeometryType::eCONVEXCORE && Gu::isGPUCompatible(shapeCore.mGeometry.get<const PxConvexCoreGeometry>());
	PxU32 particleOrSoftbodyIndex = 0xffffffff;

	size_t gpuHullPtr = 0;
	PxMeshScale scale;
	bool compatible = false;

	if (isCompatibleConvex)
	{
		compatible = true;
		const PxConvexMeshGeometryLL& convexGeom = shapeCore.mGeometry.get<const PxConvexMeshGeometryLL>();

		const ConvexHullData* hull = _getHullData(convexGeom);
		const RefcountedRecordsMap::Entry* he = mGeometriesMap->find((size_t)hull);

		PxU32 hullIdx;

		if (!he)
		{
			RefcountedRecord rec;
			rec.refCnt = 1;
			rec.idx = addHull(*hull);
			mGeometriesMap->insert((size_t)hull, rec);

			hullIdx = rec.idx;
		}
		else
		{
			RefcountedRecord& rec = (*mGeometriesMap)[(size_t)hull];
			++rec.refCnt;
			hullIdx = rec.idx;
		}

		gpuHullPtr = mGeometryManager.getGeometryDevPtrByIndex(hullIdx);
		scale = convexGeom.scale;
	}
	else if (isCompatibleMesh)
	{
		compatible = true;
		if (isFemCloth)
		{
			const PxTriangleMeshGeometryLL& triMeshGeom = shapeCore.mGeometry.get<const PxTriangleMeshGeometryLL>();

			scale.scale = PxVec3(0.f);
			scale.rotation = PxQuat(PxIdentity);

			//gpuRemapId is filled in PxgBodySimManager when femCloth register with the body sim manager
			particleOrSoftbodyIndex = triMeshGeom.materialsLL.gpuRemapId;
		}
		else
		{
			compatible = true;
			const PxTriangleMeshGeometryLL& triMeshGeom = shapeCore.mGeometry.get<const PxTriangleMeshGeometryLL>();

			const TriangleMesh* triMesh = _getMeshData(triMeshGeom);
			const RefcountedRecordsMap::Entry* tme = mGeometriesMap->find((size_t)triMesh);

			PxU32 triMeshIdx;

			if (!tme)
			{
				RefcountedRecord rec;
				rec.refCnt = 1;
				rec.idx = addTriMesh(*triMesh);
				mGeometriesMap->insert((size_t)triMesh, rec);

				triMeshIdx = rec.idx;
			}
			else
			{
				RefcountedRecord& rec = (*mGeometriesMap)[(size_t)triMesh];
				++rec.refCnt;
				triMeshIdx = rec.idx;
			}

			gpuHullPtr = mGeometryManager.getGeometryDevPtrByIndex(triMeshIdx);
			scale = triMeshGeom.scale;

			//triangle mesh : this index is invalide
			particleOrSoftbodyIndex = 0xffffffff;
		}
	}
	else if (isCompatibleTetMesh)
	{
		compatible = true;
		const PxTetrahedronMeshGeometryLL& tetMeshGeom = shapeCore.mGeometry.get<const PxTetrahedronMeshGeometryLL>();

		scale.scale = PxVec3(0.f);
		scale.rotation = PxQuat(PxIdentity);

		//gpuRemapId is filled in PxgBodySimManager when soft body register with the body sim manager
		particleOrSoftbodyIndex = tetMeshGeom.materialsLL.gpuRemapId;

	}
	else if (isCompatibleHF)
	{
		compatible = true;
		const PxHeightFieldGeometryLL& hfGeom = shapeCore.mGeometry.get<const PxHeightFieldGeometryLL>();

		const HeightFieldData* hf = _getHFData(hfGeom);
		const RefcountedRecordsMap::Entry* tme = mGeometriesMap->find((size_t)hf);

		PxU32 triMeshIdx;

		if (!tme)
		{
			RefcountedRecord rec;
			rec.refCnt = 1;
			rec.idx = addHeightfield(*hf);
			mGeometriesMap->insert((size_t)hf, rec);

			triMeshIdx = rec.idx;
		}
		else
		{
			RefcountedRecord& rec = (*mGeometriesMap)[(size_t)hf];
			++rec.refCnt;
			triMeshIdx = rec.idx;
		}

		gpuHullPtr = mGeometryManager.getGeometryDevPtrByIndex(triMeshIdx);
		scale = PxMeshScale(PxVec3(hfGeom.rowScale, hfGeom.heightScale, hfGeom.columnScale), PxQuat(PxIdentity));
	}
	else if (isCompatibleConvexCore)
	{
		compatible = true;
		const PxConvexCoreGeometry& convexGeom = shapeCore.mGeometry.get<const PxConvexCoreGeometry>();
		const PxReal* core = static_cast<const PxReal*>(convexGeom.getCoreData());
		scale.scale.x = core[0];
		scale.scale.y = core[1];
		scale.scale.z = core[2];
		scale.rotation.x = core[3];
		scale.rotation.y = core[4];
		scale.rotation.z = core[5];
		scale.rotation.w = convexGeom.getMargin();
		gpuHullPtr = size_t(convexGeom.getCoreType());
	}
	else if (isBox)
	{
		compatible = true;
		gpuHullPtr = mGeometryManager.getBoxHullDevPtr();
		scale.scale = shapeCore.mGeometry.get<const PxBoxGeometry>().halfExtents;
		scale.rotation = PxQuat(PxIdentity);
	}
	else if (type == PxGeometryType::eSPHERE)
	{
		compatible = true;
		const PxSphereGeometry& sphere = shapeCore.mGeometry.get<const PxSphereGeometry>();
		scale.scale = PxVec3(sphere.radius);
		scale.rotation = PxQuat(PxIdentity);
	}
	else if (type == PxGeometryType::eCAPSULE)
	{
		compatible = true;
		const PxCapsuleGeometry& capsule = shapeCore.mGeometry.get<const PxCapsuleGeometry>();
		scale.scale = PxVec3(capsule.halfHeight, capsule.radius, capsule.radius);
		scale.rotation = PxQuat(PxIdentity);
	}
	else if (type == PxGeometryType::ePLANE)
	{
		compatible = true;
		scale.scale = PxVec3(0.f);
		scale.rotation = PxQuat(PxIdentity);
	}
	else if (type == PxGeometryType::ePARTICLESYSTEM)
	{
		compatible = true;
		scale.scale = PxVec3(0.f);
		scale.rotation = PxQuat(PxIdentity);


		//gpuRemapId is filled in PxgBodySimManager when particle system register with the body sim manager
		const PxParticleSystemGeometryLL& particleSystem = shapeCore.mGeometry.get<const PxParticleSystemGeometryLL>();
		particleOrSoftbodyIndex = particleSystem.materialsLL.gpuRemapId;
	}

	if (compatible)
	{
		const RefcountedRecordsMap::Entry* e = mShapesMap->find((size_t)&shapeCore);

		mGpuShapesManager.registerShapeInstance(nodeIndex, transformCacheID, actor);

		if (!e)
		{

			PxgShape newShape;
			newShape.hullOrMeshPtr = gpuHullPtr;
			newShape.materialIndex = mapMaterialIndex<PxsMaterialData>(shapeCore.mMaterialIndex, mMaterialsMap, mGpuMaterialManager);
			newShape.scale = scale;
			newShape.type = type;
			newShape.particleOrSoftbodyId = particleOrSoftbodyIndex;

			RefcountedRecord rec;
			rec.refCnt = 1;

			//this might involve reallocating the shapes array, so we do need the context
			mCudaContextManager->acquireContext();
			rec.idx = mGpuShapesManager.registerShape(newShape);
			mCudaContextManager->releaseContext();

			mShapesMap->insert((size_t)&shapeCore, rec);

		}
		else
		{
			RefcountedRecord& rec = (*mShapesMap)[(size_t)&shapeCore];
			++rec.refCnt;
		}
	}
	
}

void PxgGpuNarrowphaseCore::updateShapeMaterial(const PxsShapeCore& shapeCore)
{
	const PxGeometryType::Enum type = shapeCore.mGeometry.getType();
	
	// Only update for geometries that are supported on GPU:
	// Custom geometries run on CPU, and non-GPU-compatible convexes and meshes will also fall back to CPU contact gen.
	const bool isCustom = type == PxGeometryType::eCUSTOM;
	const bool isIncompatibleConvex = type == PxGeometryType::eCONVEXMESH && !shapeCore.mGeometry.get<const PxConvexMeshGeometryLL>().gpuCompatible;
	const bool isIncompatibleMesh = type == PxGeometryType::eTRIANGLEMESH && !isMeshGPUCompatible(shapeCore.mGeometry.get<const PxTriangleMeshGeometryLL>());

	if (!(isCustom || isIncompatibleConvex || isIncompatibleMesh))
	{
		const PxPair<const size_t, RefcountedRecord>* pair = mShapesMap->find((size_t)&shapeCore);

		if (pair) //KS - this can be null if the shape is not inserted yet, e.g. if it is buffered
		{
			const RefcountedRecord& rec = pair->second;

			mGpuShapesManager.updateShapeMaterial(mapMaterialIndex<PxsMaterialData>(shapeCore.mMaterialIndex, mMaterialsMap, mGpuMaterialManager), rec.idx);
		}
	}
}

PxU32 PxgGpuNarrowphaseCore::getShapeIndex(const PxsShapeCore& shapeCore)
{
	const RefcountedRecordsMap::Entry* e = mShapesMap->find((size_t) &shapeCore);

	if (e)
	{
		return e->second.idx;
	}
	else
	{
		return 0xFFffFFff;
	}
}

void PxgGpuNarrowphaseCore::unregisterShape(const PxsShapeCore& shapeCore, const PxU32 transformCacheID, const bool isFemCloth)
{
	const PxGeometryType::Enum type = shapeCore.mGeometry.getType();

	const bool isCompatibleConvex = type == PxGeometryType::eCONVEXMESH && shapeCore.mGeometry.get<const PxConvexMeshGeometryLL>().gpuCompatible;
	const bool isCompatibleMesh = type == PxGeometryType::eTRIANGLEMESH && isMeshGPUCompatible(shapeCore.mGeometry.get<const PxTriangleMeshGeometryLL>());
	const bool isCompatibleHF = type == PxGeometryType::eHEIGHTFIELD;
	
	if (isCompatibleConvex)
	{
		const PxConvexMeshGeometryLL& convexGeom = shapeCore.mGeometry.get<const PxConvexMeshGeometryLL>();
		const ConvexHullData* hull = _getHullData(convexGeom);
		
		PX_ASSERT(mGeometriesMap->find((size_t) hull));	

		RefcountedRecord& hrec = (*mGeometriesMap)[(size_t) hull];
		hrec.refCnt -= 1;

		if (!(hrec.refCnt))
		{
			removeGeometry(hrec.idx);
			mGeometriesMap->erase((size_t) hull);
		}
	}
	else if(isCompatibleMesh)
	{
		if (!isFemCloth)
		{
			const PxTriangleMeshGeometryLL& triMeshGeom = shapeCore.mGeometry.get<const PxTriangleMeshGeometryLL>();

			const TriangleMesh* triMesh = _getMeshData(triMeshGeom);

			PX_ASSERT(mGeometriesMap->find((size_t)triMesh));

			RefcountedRecord& hrec = (*mGeometriesMap)[(size_t)triMesh];
			hrec.refCnt -= 1;

			if (!(hrec.refCnt))
			{
				removeGeometry(hrec.idx);
				mGeometriesMap->erase((size_t)triMesh);
			}
		}
	}
	else if (isCompatibleHF)
	{
		const PxHeightFieldGeometryLL& triMeshGeom = shapeCore.mGeometry.get<const PxHeightFieldGeometryLL>();
		
		const HeightFieldData* hf = _getHFData(triMeshGeom);
		
		PX_ASSERT(mGeometriesMap->find((size_t) hf));

		RefcountedRecord& hrec = (*mGeometriesMap)[(size_t) hf];
		hrec.refCnt -= 1;

		if (!(hrec.refCnt))
		{
			removeGeometry(hrec.idx);
			mGeometriesMap->erase((size_t) hf);
		}
	}

	const RefcountedRecordsMap::Entry* pair = mShapesMap->find((size_t)&shapeCore);

	if (pair)
	{

		RefcountedRecord& rec = const_cast<RefcountedRecord&>(pair->second);
		rec.refCnt -= 1;

		mGpuShapesManager.unregisterShapeInstance(transformCacheID);

		if (!(rec.refCnt))
		{
			mGpuShapesManager.unregisterShape(rec.idx);
			mShapesMap->erase((size_t)&shapeCore);
		}
	}
	
}

//Aggregate don't have shape so we need to set mHostShapeIdTable[transformCacheID] to be 0xffffffff
//the aggregate also does not have a specific actor so we set the actor to NULL as well.
void PxgGpuNarrowphaseCore::registerAggregate(const PxU32 transformCacheID)
{
	mGpuShapesManager.registerShapeInstance(PxNodeIndex(PX_INVALID_NODE), transformCacheID, NULL, true);
}

template <typename MaterialCore, typename MaterialData>
PxU32 PxgGpuNarrowphaseCore::registerMaterialInternal(const MaterialCore& materialCore, RefcountedRecordsMap* materialsMap, PxgMaterialManager& materialManager)
{
	const PxU16 sdkMaterialIndex = materialCore.mMaterialIndex;
	const RefcountedRecordsMap::Entry* me = materialsMap->find(sdkMaterialIndex);

	const MaterialData& data = materialCore;
	PxU32 shapeId = 0xffffffff;
	if (!me)
	{
		RefcountedRecord rec;
		rec.refCnt = 1;

		//this might involve reallocating the materials array, so we do need the context
		mCudaContextManager->acquireContext();

		shapeId = materialManager.registerMaterial(reinterpret_cast<const PxU8*>(&data), sizeof(MaterialData));
		rec.idx = shapeId;
		
		mCudaContextManager->releaseContext();

		materialsMap->insert(sdkMaterialIndex, rec);
	}
	else
	{
		RefcountedRecord& rec = (*materialsMap)[sdkMaterialIndex];
		shapeId = rec.idx;
		//we support setting materials to shapes before they are added to the scene. Should be done on the same frame.
		if (rec.refCnt == 0)
		{
			materialManager.updateMaterial(reinterpret_cast<const PxU8*>(&data), sizeof(MaterialData), rec.idx);
		}

		++rec.refCnt;
	}

	return shapeId;
}

template <typename MaterialCore, typename MaterialData>
void PxgGpuNarrowphaseCore::updateMaterialInternal(const MaterialCore& materialCore, RefcountedRecordsMap* materialsMap, PxgMaterialManager& materialManager)
{
	const PxU16 sdkMaterialIndex = materialCore.mMaterialIndex;
	PX_ASSERT(materialsMap->find(sdkMaterialIndex));

	RefcountedRecord& rec = (*materialsMap)[sdkMaterialIndex];
	PX_ASSERT(rec.refCnt);

	const MaterialData& data = materialCore;

	materialManager.updateMaterial(reinterpret_cast<const PxU8*>(&data), sizeof(MaterialData), rec.idx);
}

template <typename MaterialCore>
void PxgGpuNarrowphaseCore::unregisterMaterialInternal(const MaterialCore& materialCore, RefcountedRecordsMap* materialsMap, PxgMaterialManager& materialManager)
{
	const PxU16 sdkMaterialIndex = materialCore.mMaterialIndex;
	PX_ASSERT(materialsMap->find(sdkMaterialIndex));

	RefcountedRecord& rec = (*materialsMap)[sdkMaterialIndex];
	PX_ASSERT(rec.refCnt);

	rec.refCnt -= 1;

	if (!(rec.refCnt))
	{
		materialManager.unregisterMaterial(rec.idx);
		materialsMap->erase(sdkMaterialIndex);
	}
}

PxU16 PxgGpuNarrowphaseCore::mapMaterialIndexInternal(PxU16 sdkMaterialIndex, RefcountedRecordsMap* materialsMap,
	PxgMaterialManager& materialManager, PxU32 materialDataByteSize)
{
	const RefcountedRecordsMap::Entry* me = materialsMap->find(sdkMaterialIndex);

	if (me)
	{
		return PxTo16(me->second.idx);
	}
	else
	{
		//add a dummy material and expect it to be added properly on the same frame
		RefcountedRecord rec;
		rec.refCnt = 0;

		//this might involve reallocating the materials array, so we do need the context
		mCudaContextManager->acquireContext();

		PxsMaterialData dummy;
		rec.idx = materialManager.registerMaterial(reinterpret_cast<const PxU8*>(&dummy), materialDataByteSize);
		mCudaContextManager->releaseContext();

		materialsMap->insert(sdkMaterialIndex, rec);

		return PxTo16(rec.idx);
	}
}

//we expect no removals before adds, this should be handled at a higher level
void PxgGpuNarrowphaseCore::registerMaterial(const PxsMaterialCore& materialCore)
{
	registerMaterialInternal<PxsMaterialCore, PxsMaterialData>(materialCore, mMaterialsMap, mGpuMaterialManager);
}

//we expect no updates of non-existant materials, this should be handled at a higher level
void PxgGpuNarrowphaseCore::updateMaterial(const PxsMaterialCore& materialCore)
{
	updateMaterialInternal<PxsMaterialCore, PxsMaterialData>(materialCore, mMaterialsMap, mGpuMaterialManager);
	
}


//we expect no removals before adds, this should be handled at a higher level
void PxgGpuNarrowphaseCore::unregisterMaterial(const PxsMaterialCore& materialCore)
{
	unregisterMaterialInternal<PxsMaterialCore>(materialCore, mMaterialsMap, mGpuMaterialManager);
}


void PxgGpuNarrowphaseCore::registerFEMMaterial(const PxsDeformableSurfaceMaterialCore& materialCore)
{
	registerMaterialInternal<PxsDeformableSurfaceMaterialCore, PxsDeformableSurfaceMaterialData>(materialCore, mFEMClothMaterialsMap, mGpuFEMClothMaterialManager);
}

void PxgGpuNarrowphaseCore::updateFEMMaterial(const PxsDeformableSurfaceMaterialCore& materialCore)
{
	updateMaterialInternal<PxsDeformableSurfaceMaterialCore, PxsDeformableSurfaceMaterialData>(materialCore, mFEMClothMaterialsMap, mGpuFEMClothMaterialManager);
}

void PxgGpuNarrowphaseCore::unregisterFEMMaterial(const PxsDeformableSurfaceMaterialCore& materialCore)
{
	unregisterMaterialInternal<PxsDeformableSurfaceMaterialCore>(materialCore, mFEMClothMaterialsMap, mGpuFEMClothMaterialManager);
}


void PxgGpuNarrowphaseCore::registerFEMMaterial(const PxsDeformableVolumeMaterialCore& materialCore)
{
	registerMaterialInternal<PxsDeformableVolumeMaterialCore, PxsDeformableVolumeMaterialData>(materialCore, mFEMMaterialsMap, mGpuFEMMaterialManager);
}

void PxgGpuNarrowphaseCore::updateFEMMaterial(const PxsDeformableVolumeMaterialCore& materialCore)
{
	updateMaterialInternal<PxsDeformableVolumeMaterialCore, PxsDeformableVolumeMaterialData>(materialCore, mFEMMaterialsMap, mGpuFEMMaterialManager);
}

void PxgGpuNarrowphaseCore::unregisterFEMMaterial(const PxsDeformableVolumeMaterialCore& materialCore)
{
	unregisterMaterialInternal<PxsDeformableVolumeMaterialCore>(materialCore, mFEMMaterialsMap, mGpuFEMMaterialManager);
}


void PxgGpuNarrowphaseCore::registerParticleMaterial(const PxsPBDMaterialCore& materialCore)
{
	registerMaterialInternal<PxsPBDMaterialCore, PxsPBDMaterialData>(materialCore, mPBDMaterialsMap, mGpuPBDMaterialManager);
}

void PxgGpuNarrowphaseCore::updateParticleMaterial(const PxsPBDMaterialCore& materialCore)
{
	updateMaterialInternal<PxsPBDMaterialCore, PxsPBDMaterialData>(materialCore, mPBDMaterialsMap, mGpuPBDMaterialManager);
}

void PxgGpuNarrowphaseCore::unregisterParticleMaterial(const PxsPBDMaterialCore& materialCore)
{
	unregisterMaterialInternal<PxsPBDMaterialCore>(materialCore, mPBDMaterialsMap, mGpuPBDMaterialManager);
}

void PxgGpuNarrowphaseCore::registerContactManagerInternal(PxsContactManager* cm, const Sc::ShapeInteraction* shapeInteraction, PxgContactManagerInput* input, PxsContactManagerOutput& output, PxgNewContactManagers& newContactManagers)
{
	PxPinnedArray<PxgContactManagerInput>& itInputs = newContactManagers.mGpuInputContactManagers;
	PxPinnedArray<PxsContactManagerOutput>& itOutputs = newContactManagers.mGpuOutputContactManagers;
	PxPinnedArray<PxsContactManager*>& itCms = newContactManagers.mCpuContactManagerMapping;
	PxPinnedArray<const Sc::ShapeInteraction*>& itSI = newContactManagers.mShapeInteractions;
	PxFloatArrayPinned& itR = newContactManagers.mRestDistances;
	PxPinnedArray<PxsTorsionalFrictionData>& itTor = newContactManagers.mTorsionalProperties;
	
	PxcNpWorkUnit& workUnit = cm->getWorkUnit();

	if(input)
		itInputs.pushBack(*input);
	else
		itInputs.insert();

	itOutputs.pushBack(output);
	itCms.pushBack(cm);
	itSI.pushBack(shapeInteraction);
	itR.pushBack(cm->getRestDistance());
	itTor.pushBack(PxsTorsionalFrictionData(workUnit.mTorsionalPatchRadius, workUnit.mMinTorsionalPatchRadius));
	
	PxU32 newSz = newContactManagers.mGpuOutputContactManagers.size();

	workUnit.mNpIndex = newContactManagers.computeId(newSz - 1) | PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK;

	if(workUnit.mStatusFlags & PxcNpWorkUnitStatusFlag::eHAS_TOUCH)
		workUnit.mStatusFlags |= PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH;
}

void PxgGpuNarrowphaseCore::unregisterContactManagerInternal(PxsContactManager* cm, PxInt32ArrayPinned& removedIndices, PxgNewContactManagers& newContactManagers)
{
	PxcNpWorkUnit& unit = cm->getWorkUnit();
	PxU32 index = unit.mNpIndex;
	PX_ASSERT(index != 0xFFffFFff);
	
	//Stage 1: we buffer all the removals in the removed array so we can batch remove on the GPU
	if (!(index & PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))
	{
		index = PxgContactManagers::computeIndexFromId(index);
		removedIndices.pushBack(index);
	}
	else
	{
		index &= ~PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK;
		index = PxgContactManagers::computeIndexFromId(index);
			
		PxPinnedArray<PxgContactManagerInput>& itInputs = newContactManagers.mGpuInputContactManagers;
		PxPinnedArray<PxsContactManagerOutput>& itOutputs = newContactManagers.mGpuOutputContactManagers;
		PxPinnedArray<PxsContactManager*>& itCms = newContactManagers.mCpuContactManagerMapping;
		PxPinnedArray<const Sc::ShapeInteraction*>& itSI = newContactManagers.mShapeInteractions;
		PxFloatArrayPinned& itR = newContactManagers.mRestDistances;
		PxPinnedArray<PxsTorsionalFrictionData>& itTor = newContactManagers.mTorsionalProperties;
	
		itInputs.replaceWithLast(index);
		itOutputs.replaceWithLast(index);
		itCms.replaceWithLast(index);
		itSI.replaceWithLast(index);
		itR.replaceWithLast(index);
		itTor.replaceWithLast(index);
		
		PxU32 newSz = newContactManagers.mGpuOutputContactManagers.size();

		if (index < newSz)
		{
			PxcNpWorkUnit& unit_ = itCms[index]->getWorkUnit();
			unit_.mNpIndex = newContactManagers.computeId(index) | PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK;

			if(unit_.mStatusFlags & PxcNpWorkUnitStatusFlag::eHAS_TOUCH)
				processPartitionEdges(mIslandSim->mGpuData, unit_);
		}
	
		cm->getWorkUnit().mNpIndex = 0xFFffFFff;
	}
}


void PxgGpuNarrowphaseCore::refreshContactManagerInternal(PxsContactManager* cm, PxsContactManagerOutput* cmOutputs, const Sc::ShapeInteraction** shapeInteractions, PxgContactManagerInput& input, PxgNewContactManagers& newContactManagers, 
	PxInt32ArrayPinned& removedIndices)
{
	PxcNpWorkUnit& unit = cm->getWorkUnit();
	PxU32 index = unit.mNpIndex;
	PX_ASSERT(index != 0xFFffFFff);
	
	PxScopedCudaLock lock(*mCudaContextManager);

	PxsContactManagerOutput output;
	const Sc::ShapeInteraction* shapeInteraction;

	if (!(index & PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))
	{
		index = PxgContactManagers::computeIndexFromId(index);
		output = cmOutputs[index];
		shapeInteraction = shapeInteractions[index];
		removedIndices.pushBack(index);
	}
	else
	{
		index &= ~PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK;
		index = PxgContactManagers::computeIndexFromId(index);
			
		PxPinnedArray<PxgContactManagerInput>& itInputs = newContactManagers.mGpuInputContactManagers;
		PxPinnedArray<PxsContactManagerOutput>& itOutputs = newContactManagers.mGpuOutputContactManagers;
		PxPinnedArray<PxsContactManager*>& itCms = newContactManagers.mCpuContactManagerMapping;
		PxPinnedArray<const Sc::ShapeInteraction*>& itSI = newContactManagers.mShapeInteractions;
		PxFloatArrayPinned& itR = newContactManagers.mRestDistances;
		PxPinnedArray<PxsTorsionalFrictionData>& itTor = newContactManagers.mTorsionalProperties;

		output = itOutputs[index];

		shapeInteraction = itSI[index];
	
		itInputs.replaceWithLast(index);
		itOutputs.replaceWithLast(index);
		itCms.replaceWithLast(index);
		itSI.replaceWithLast(index);
		itR.replaceWithLast(index);
		itTor.replaceWithLast(index);
		
		PxU32 newSz = newContactManagers.mGpuOutputContactManagers.size();

		if (index < newSz)
			itCms[index]->getWorkUnit().mNpIndex = newContactManagers.computeId(index) | PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK;
	
		cm->getWorkUnit().mNpIndex = 0xFFffFFff;
	}

	registerContactManagerInternal(cm, shapeInteraction, &input, output, newContactManagers);
}

void PxgGpuNarrowphaseCore::registerContactManager(PxsContactManager* cm, const Sc::ShapeInteraction* shapeInteraction, PxsContactManagerOutput& output, const PxU32 bucketId)
{
	registerContactManagerInternal(cm, shapeInteraction, NULL, output, mContactManagers[bucketId]->mNewContactManagers);
}

void PxgGpuNarrowphaseCore::unregisterContactManager(PxsContactManager* cm, const PxU32 bucketId)
{
	unregisterContactManagerInternal(cm, *mRemovedIndices[bucketId], mContactManagers[bucketId]->mNewContactManagers);
}

void PxgGpuNarrowphaseCore::refreshContactManager(PxsContactManager* cm, PxsContactManagerOutput* cmOutputs, PxgContactManagerInput& input, const PxU32 bucketId)
{
	refreshContactManagerInternal(cm, cmOutputs, mContactManagers[bucketId]->mContactManagers.mShapeInteractions.begin(), input, mContactManagers[bucketId]->mNewContactManagers, *mRemovedIndices[bucketId]);
}

void PxgGpuNarrowphaseCore::createGpuStreamsAndEvents()
{
	PxScopedCudaLock lock(*mCudaContextManager);

	int leastPriority, mostPriority;
	cuCtxGetStreamPriorityRange(&leastPriority, &mostPriority);

	//KS - choose midpoint between least/most priority. On Volta/Turing, there are 3 priorities, so this will pick the middle priority.
	//On devices with only 2 priorities, this will choose the lowest priority.
	mCudaContext->streamCreateWithPriority(&mStream, CU_STREAM_NON_BLOCKING, (leastPriority+mostPriority)/2);

	mCudaContext->eventCreate(&mParticleEvent, CU_EVENT_DISABLE_TIMING);
	mCudaContext->eventCreate(&mSoftbodyEvent, CU_EVENT_DISABLE_TIMING);
	mCudaContext->eventCreate(&mFemClothEvent, CU_EVENT_DISABLE_TIMING);
	mCudaContext->eventCreate(&mDirectApiDmaEvent, CU_EVENT_DISABLE_TIMING);
}

void PxgGpuNarrowphaseCore::releaseGpuStreamsAndEvents()
{
	PxScopedCudaLock lock(*mCudaContextManager);

	//destroy stream
	mCudaContext->streamDestroy(mStream);
	mStream = NULL;

	mCudaContext->eventDestroy(mParticleEvent);
	mParticleEvent = NULL;

	mCudaContext->eventDestroy(mSoftbodyEvent);
	mSoftbodyEvent = NULL;

	mCudaContext->eventDestroy(mFemClothEvent);
	mFemClothEvent = NULL;

	mCudaContext->eventDestroy(mDirectApiDmaEvent);
	mDirectApiDmaEvent = NULL;	
}

void PxgGpuNarrowphaseCore::acquireContext()
{
	mCudaContextManager->acquireContext();
}

void PxgGpuNarrowphaseCore::releaseContext()
{
	mCudaContextManager->releaseContext();
}

void PxgGpuNarrowphaseCore::removeLostPairsGpu(const PxU32 bucketID, const PxU16 stage5KernelID, const bool copyManifold)
{
	removeLostPairsGpuInternal<PxgPairManagementData, PxgPersistentContactManifold>(*mPairManagementData[bucketID],
		mGpuPairManagementData.getDevicePtr(), mContactManagers[bucketID]->mContactManagers, mGpuContactManagers[bucketID]->mContactManagers, *mRemovedIndices[bucketID], mPairManagementBuffers,
		stage5KernelID, copyManifold);
}


void PxgGpuNarrowphaseCore::appendContactManagersGpu(PxU32 nbExistingManagers, PxU32 nbNewManagers, PxgGpuContactManagers& gpuContactManagers, PxgGpuContactManagers& newGpuContactManagers, PxU32 manifoldSize)
{
	if (nbNewManagers == 0)
		return;

	PX_PROFILE_ZONE("appendContactManagersGpu", 0);

	const PxU32 oldSize = nbExistingManagers;
	const PxU32 newSize = nbNewManagers + oldSize;

	gpuContactManagers.mLostFoundPairsOutputData.allocate(2 * newSize * sizeof(PxsContactManagerOutputCounts), PX_FL);
	gpuContactManagers.mLostFoundPairsCms.allocate(2 * newSize * sizeof(PxsContactManager*), PX_FL);

	gpuContactManagers.mTempRunsumArray.allocate(2 * newSize * sizeof(PxU32), PX_FL);
	gpuContactManagers.mTempRunsumArray2.allocate(2 * newSize * sizeof(PxU32), PX_FL);

	if (oldSize == 0)
	{
		//Special-case the condition where we didn't have any existing contacts. We can save memory by just assigning the 
		//new pointers to the existing pointers, as the first frame may be the case when we have the most number of new pairs
		gpuContactManagers.mContactManagerInputData.assign(newGpuContactManagers.mContactManagerInputData);
		gpuContactManagers.mContactManagerOutputData.assign(newGpuContactManagers.mContactManagerOutputData);
		gpuContactManagers.mPersistentContactManifolds.assign(newGpuContactManagers.mPersistentContactManifolds);
		gpuContactManagers.mCpuContactManagerMapping.assign(newGpuContactManagers.mCpuContactManagerMapping);
		gpuContactManagers.mShapeInteractions.assign(newGpuContactManagers.mShapeInteractions);
		gpuContactManagers.mRestDistances.assign(newGpuContactManagers.mRestDistances);
		gpuContactManagers.mTorsionalProperties.assign(newGpuContactManagers.mTorsionalProperties);
	}
	else
	{
		// we resize and copy the old data. New managers are appended.
		gpuContactManagers.mContactManagerInputData.allocateCopyOldDataAsync(newSize * sizeof(PxgContactManagerInput), mCudaContext, mSolverStream, PX_FL);
		gpuContactManagers.mContactManagerOutputData.allocateCopyOldDataAsync(newSize * sizeof(PxsContactManagerOutput), mCudaContext, mSolverStream, PX_FL);
		gpuContactManagers.mPersistentContactManifolds.allocateCopyOldDataAsync(newSize* manifoldSize, mCudaContext, mSolverStream, PX_FL);
		gpuContactManagers.mCpuContactManagerMapping.allocateCopyOldDataAsync(newSize * sizeof(PxsContactManager*), mCudaContext, mSolverStream, PX_FL);
		gpuContactManagers.mShapeInteractions.allocateCopyOldDataAsync(newSize * sizeof(Sc::ShapeInteraction*), mCudaContext, mSolverStream, PX_FL);
		gpuContactManagers.mRestDistances.allocateCopyOldDataAsync(newSize * sizeof(PxReal), mCudaContext, mSolverStream, PX_FL);
		gpuContactManagers.mTorsionalProperties.allocateCopyOldDataAsync(newSize * sizeof(PxsTorsionalFrictionData), mCudaContext, mSolverStream, PX_FL);


		// now we copy in the data from the new managers, append it to the exsting data.
		mCudaContext->memcpyDtoDAsync(gpuContactManagers.mContactManagerInputData.getDevicePtr() + sizeof(PxgContactManagerInput) * oldSize, newGpuContactManagers.mContactManagerInputData.getDevicePtr(),
			nbNewManagers * sizeof(PxgContactManagerInput), mSolverStream);
		mCudaContext->memcpyDtoDAsync(gpuContactManagers.mContactManagerOutputData.getDevicePtr() + sizeof(PxsContactManagerOutput) * oldSize, newGpuContactManagers.mContactManagerOutputData.getDevicePtr(),
			nbNewManagers * sizeof(PxsContactManagerOutput), mSolverStream);
		if (manifoldSize)
		{
			mCudaContext->memcpyDtoDAsync(gpuContactManagers.mPersistentContactManifolds.getDevicePtr() + manifoldSize * oldSize, newGpuContactManagers.mPersistentContactManifolds.getDevicePtr(),
				nbNewManagers* manifoldSize, mSolverStream);
		}
		mCudaContext->memcpyDtoDAsync(gpuContactManagers.mCpuContactManagerMapping.getDevicePtr() + sizeof(PxsContactManager*) * oldSize, newGpuContactManagers.mCpuContactManagerMapping.getDevicePtr(),
			nbNewManagers * sizeof(PxsContactManager*), mSolverStream);

		mCudaContext->memcpyDtoDAsync(gpuContactManagers.mShapeInteractions.getDevicePtr() + sizeof(Sc::ShapeInteraction*) * oldSize, newGpuContactManagers.mShapeInteractions.getDevicePtr(),
			nbNewManagers * sizeof(Sc::ShapeInteraction*), mSolverStream);

		mCudaContext->memcpyDtoDAsync(gpuContactManagers.mRestDistances.getDevicePtr() + sizeof(PxReal) * oldSize, newGpuContactManagers.mRestDistances.getDevicePtr(),
			nbNewManagers * sizeof(PxReal), mSolverStream);

		mCudaContext->memcpyDtoDAsync(gpuContactManagers.mTorsionalProperties.getDevicePtr() + sizeof(PxsTorsionalFrictionData) * oldSize, newGpuContactManagers.mTorsionalProperties.getDevicePtr(),
			nbNewManagers * sizeof(PxsTorsionalFrictionData), mSolverStream);
	}

	//cuStreamSynchronize(mSolverStream);
}

void PxgGpuNarrowphaseCore::preallocateNewBuffers(PxU32 nbNewPairs)
{
	for (PxU32 i = GPU_BUCKET_ID::eConvex; i < GPU_BUCKET_ID::eCount; ++i)
	{
		mContactManagers[i]->mNewContactManagers.preallocateNewBuffers(nbNewPairs);
	}

}

template <typename Manifold>
void PxgGpuNarrowphaseCore::prepareTempContactManagers(PxgGpuContactManagers& gpuManagers, PxgNewContactManagers& newManagers,
	Manifold* emptyManifold)
{
	const PxU32 nbNewManagers = newManagers.mCpuContactManagerMapping.size();
	if(nbNewManagers == 0)
		return;
	PX_PROFILE_ZONE("GpuNarrowPhase.prepareTempContactManagers", 0);
	
	PxScopedCudaLock lock(*mCudaContextManager);
	
	gpuManagers.mContactManagerInputData.allocate(sizeof(PxgContactManagerInput) * nbNewManagers, PX_FL);
	gpuManagers.mContactManagerOutputData.allocate(sizeof(PxsContactManagerOutput) * nbNewManagers, PX_FL);
	gpuManagers.mPersistentContactManifolds.allocate(sizeof(Manifold) * nbNewManagers, PX_FL);
	gpuManagers.mCpuContactManagerMapping.allocate(sizeof(PxsContactManager*) * nbNewManagers, PX_FL);
	gpuManagers.mShapeInteractions.allocate(sizeof(Sc::ShapeInteraction*) * nbNewManagers, PX_FL);
	gpuManagers.mRestDistances.allocate(sizeof(PxReal) * nbNewManagers, PX_FL);
	gpuManagers.mTorsionalProperties.allocate(sizeof(PxsTorsionalFrictionData) * nbNewManagers, PX_FL);

	gpuManagers.mLostFoundPairsOutputData.allocate(2 * nbNewManagers * sizeof(PxsContactManagerOutputCounts), PX_FL);
	gpuManagers.mLostFoundPairsCms.allocate(2 * nbNewManagers * sizeof(PxsContactManager*), PX_FL);
	gpuManagers.mTempRunsumArray.allocate(2 * nbNewManagers * sizeof(PxU32), PX_FL);
	gpuManagers.mTempRunsumArray2.allocate(2 * nbNewManagers * sizeof(PxU32), PX_FL);
	
	PxPinnedArray<PxgContactManagerInput>& itInputs = newManagers.mGpuInputContactManagers;
	PxPinnedArray<PxsContactManagerOutput>& itOutputs = newManagers.mGpuOutputContactManagers;
	PxPinnedArray<PxsContactManager*>& itCms = newManagers.mCpuContactManagerMapping;
	PxPinnedArray<const Sc::ShapeInteraction*>& itSI = newManagers.mShapeInteractions;
	PxFloatArrayPinned& itR = newManagers.mRestDistances;
	PxPinnedArray<PxsTorsionalFrictionData>& itTor = newManagers.mTorsionalProperties;
	
	mCudaContext->memcpyHtoDAsync(gpuManagers.mContactManagerInputData.getDevicePtr(), itInputs.begin(), sizeof(PxgContactManagerInput) * nbNewManagers, mStream);
	mCudaContext->memcpyHtoDAsync(gpuManagers.mContactManagerOutputData.getDevicePtr(), itOutputs.begin(), sizeof(PxsContactManagerOutput) * nbNewManagers, mStream);
	mCudaContext->memcpyHtoDAsync(gpuManagers.mCpuContactManagerMapping.getDevicePtr(), itCms.begin(), sizeof(PxsContactManager*) * nbNewManagers, mStream);
	mCudaContext->memcpyHtoDAsync(gpuManagers.mShapeInteractions.getDevicePtr(), itSI.begin(), sizeof(Sc::ShapeInteraction*) * nbNewManagers, mStream);
	mCudaContext->memcpyHtoDAsync(gpuManagers.mRestDistances.getDevicePtr(), itR.begin(), sizeof(PxReal) * nbNewManagers, mStream);
	mCudaContext->memcpyHtoDAsync(gpuManagers.mTorsionalProperties.getDevicePtr(), itTor.begin(), sizeof(PxsTorsionalFrictionData) * nbNewManagers, mStream);

	CUfunction initializeManifolds = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::INITIALIZE_MANIFOLDS);

	CUdeviceptr devicePtr = gpuManagers.mPersistentContactManifolds.getDevicePtr();
	//void* mappedPtr = getMappedDevicePtr(emptyManifold);
	PxU32 size = sizeof(Manifold);

	PxCudaKernelParam kernelParams[] =
	{
		PX_CUDA_KERNEL_PARAM(devicePtr),
		PX_CUDA_KERNEL_PARAM(emptyManifold),
		PX_CUDA_KERNEL_PARAM(size),
		PX_CUDA_KERNEL_PARAM(nbNewManagers)
	};

	CUresult result = mCudaContext->launchKernel(initializeManifolds, PxgNarrowPhaseGridDims::INITIALIZE_MANIFOLDS, 1, 1, PxgNarrowPhaseBlockDims::INITIALIZE_MANIFOLDS, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

	PX_ASSERT(result == CUDA_SUCCESS);

	PX_UNUSED(result);
}

void PxgGpuNarrowphaseCore::prepareTempContactManagers(PxgGpuContactManagers& gpuManagers, PxgNewContactManagers& newManagers)
{
	const PxU32 nbNewManagers = newManagers.mCpuContactManagerMapping.size();
	if (nbNewManagers == 0)
		return;
	PX_PROFILE_ZONE("GpuNarrowPhase.prepareTempContactManagers", 0);

	PxScopedCudaLock lock(*mCudaContextManager);

	gpuManagers.mContactManagerInputData.allocate(sizeof(PxgContactManagerInput) * nbNewManagers, PX_FL);
	gpuManagers.mContactManagerOutputData.allocate(sizeof(PxsContactManagerOutput) * nbNewManagers, PX_FL);
	gpuManagers.mCpuContactManagerMapping.allocate(sizeof(PxsContactManager*) * nbNewManagers, PX_FL);
	gpuManagers.mShapeInteractions.allocate(sizeof(Sc::ShapeInteraction*) * nbNewManagers, PX_FL);
	gpuManagers.mRestDistances.allocate(sizeof(PxReal) * nbNewManagers, PX_FL);
	gpuManagers.mTorsionalProperties.allocate(sizeof(PxsTorsionalFrictionData) * nbNewManagers, PX_FL);

	//we don't need all these for particle but just keep it first
	gpuManagers.mLostFoundPairsOutputData.allocate(2 * nbNewManagers * sizeof(PxsContactManagerOutputCounts), PX_FL);
	gpuManagers.mLostFoundPairsCms.allocate(2 * nbNewManagers * sizeof(PxsContactManager*), PX_FL);
	gpuManagers.mTempRunsumArray.allocate(2 * nbNewManagers * sizeof(PxU32), PX_FL);
	gpuManagers.mTempRunsumArray2.allocate(2 * nbNewManagers * sizeof(PxU32), PX_FL);

	PxPinnedArray<PxgContactManagerInput>& itInputs = newManagers.mGpuInputContactManagers;
	PxPinnedArray<PxsContactManagerOutput>& itOutputs = newManagers.mGpuOutputContactManagers;
	PxPinnedArray<PxsContactManager*>& itCms = newManagers.mCpuContactManagerMapping;
	PxPinnedArray<const Sc::ShapeInteraction*>& itSI = newManagers.mShapeInteractions;
	PxFloatArrayPinned& itR = newManagers.mRestDistances;
	PxPinnedArray<PxsTorsionalFrictionData>& itTor = newManagers.mTorsionalProperties;

	mCudaContext->memcpyHtoDAsync(gpuManagers.mContactManagerInputData.getDevicePtr(), itInputs.begin(), sizeof(PxgContactManagerInput) * nbNewManagers, mStream);
	mCudaContext->memcpyHtoDAsync(gpuManagers.mContactManagerOutputData.getDevicePtr(), itOutputs.begin(), sizeof(PxsContactManagerOutput) * nbNewManagers, mStream);
	mCudaContext->memcpyHtoDAsync(gpuManagers.mCpuContactManagerMapping.getDevicePtr(), itCms.begin(), sizeof(PxsContactManager*) * nbNewManagers, mStream);
	mCudaContext->memcpyHtoDAsync(gpuManagers.mShapeInteractions.getDevicePtr(), itSI.begin(), sizeof(Sc::ShapeInteraction*) * nbNewManagers, mStream);
	mCudaContext->memcpyHtoDAsync(gpuManagers.mRestDistances.getDevicePtr(), itR.begin(), sizeof(PxReal) * nbNewManagers, mStream);
	mCudaContext->memcpyHtoDAsync(gpuManagers.mTorsionalProperties.getDevicePtr(), itTor.begin(), sizeof(PxsTorsionalFrictionData) * nbNewManagers, mStream);
}

void PxgGpuNarrowphaseCore::prepareTempContactManagers()
{
	prepareTempContactManagers<PxgPersistentContactManifold>(mGpuContactManagers[GPU_BUCKET_ID::eConvex]->mNewContactManagers, mContactManagers[GPU_BUCKET_ID::eConvex]->mNewContactManagers, reinterpret_cast<PxgPersistentContactManifold*>(mGpuManifold.getDevicePtr()));
	prepareTempContactManagers<PxgPersistentContactManifold>(mGpuContactManagers[GPU_BUCKET_ID::eConvexPlane]->mNewContactManagers, mContactManagers[GPU_BUCKET_ID::eConvexPlane]->mNewContactManagers, reinterpret_cast<PxgPersistentContactManifold*>(mGpuManifold.getDevicePtr()));
	
	for (PxU32 i = GPU_BUCKET_ID::eConvexTrimesh; i <= GPU_BUCKET_ID::eTrianglePlane; ++i)
	{
		prepareTempContactManagers<PxgPersistentContactMultiManifold>(mGpuContactManagers[i]->mNewContactManagers, mContactManagers[i]->mNewContactManagers, reinterpret_cast<PxgPersistentContactMultiManifold*>(mGpuMultiManifold.getDevicePtr()));
	}
	
	for(PxU32 i  = GPU_BUCKET_ID::eSphere; i < GPU_BUCKET_ID::eCount; ++i)
	{ 
		prepareTempContactManagers(mGpuContactManagers[i]->mNewContactManagers, mContactManagers[i]->mNewContactManagers);
	}
}

class PrepareInputTask : public Cm::Task
{
	PxgContactManagerInput* mInputs;
	PxsContactManager** mCms;
	const PxU32 mNbToProcess;
	PxgGpuNarrowphaseCore& mCore;
public:
	PrepareInputTask(PxgContactManagerInput* inputs, PxsContactManager** cms, PxU32 nbToProcess, PxgGpuNarrowphaseCore& core) : Cm::Task(0), mInputs(inputs), mCms(cms), mNbToProcess(nbToProcess), mCore(core)
	{
	}

	virtual const char* getName() const { return "PrepareInputTask"; }

	virtual void runInternal()
	{
		for (PxU32 a = 0; a < mNbToProcess; ++a)
		{
			PxsContactManager* cm = mCms[a];
			PxcNpWorkUnit& workUnit = cm->getWorkUnit();

			PxgContactManagerInput& input = mInputs[a];

			input.shapeRef0 = mCore.getShapeIndex(*workUnit.getShapeCore0());
			input.shapeRef1 = mCore.getShapeIndex(*workUnit.getShapeCore1());
			input.transformCacheRef0 = workUnit.mTransformCache0;
			input.transformCacheRef1 = workUnit.mTransformCache1;
		}
	}

private:
	PX_NOCOPY(PrepareInputTask)
};


void PxgGpuNarrowphaseCore::prepareTempContactManagersInternal(PxgNewContactManagers& newManagers, Cm::FlushPool& flushPool, PxBaseTask* continuation)
{
	PX_UNUSED(continuation);
	PX_UNUSED(flushPool);
	PX_PROFILE_ZONE("GpuNarrowPhase.prepareInputs", 0);

	PxPinnedArray<PxgContactManagerInput>& itInputs = newManagers.mGpuInputContactManagers;
	PxPinnedArray<PxsContactManager*>& itCms = newManagers.mCpuContactManagerMapping;

	const PxU32 nbPerTask = 256;

	for (PxU32 a = 0, count = itInputs.size(); a < count; a += nbPerTask)
	{

		PrepareInputTask* inputTask = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(PrepareInputTask)), PrepareInputTask)
			(&itInputs[a], &itCms[a], PxMin(nbPerTask, count - a), *this);

		inputTask->setContinuation(continuation);
		inputTask->removeReference();
	}
}

void PxgGpuNarrowphaseCore::prepareTempContactManagersTasks(Cm::FlushPool& flushPool, PxBaseTask* continuation)
{
	for (PxU32 i = GPU_BUCKET_ID::eConvex; i < GPU_BUCKET_ID::eCount; ++i)
	{
		prepareTempContactManagersInternal(mContactManagers[i]->mNewContactManagers, flushPool, continuation);
	}
}

void PxgGpuNarrowphaseCore::removeLostPairsInternal(PxInt32ArrayPinned& removedIndices, PxgContactManagers& contactManagers)
{
	PxPinnedArray<PxgContactManagerInput>& itInputs = contactManagers.mGpuInputContactManagers;
	PxPinnedArray<PxsContactManager*>& itCms = contactManagers.mCpuContactManagerMapping;
	PxPinnedArray<const Sc::ShapeInteraction*>& itSI = contactManagers.mShapeInteractions;
	PxFloatArrayPinned& itR = contactManagers.mRestDistances;
	PxPinnedArray<PxsTorsionalFrictionData>& itTor = contactManagers.mTorsionalProperties;
	
	const PxU32 finalSize = contactManagers.mCpuContactManagerMapping.size() - removedIndices.size();

	mKeepMap.resizeAndClear(removedIndices.size());

	PxU32 removeSize = removedIndices.size();
	{
		PX_PROFILE_ZONE("GpuNarrowPhase.removeLostPairs1", 0);
		for(; removeSize > 0; --removeSize)
		{
			PxU32 removedIndex = removedIndices[removeSize - 1];
			if(removedIndex < finalSize)
				break;
			mKeepMap.set(removedIndex - finalSize);
		}

		PxU32 offset = 0;
		PxU32 writeIndex = finalSize;
		PxU32 totalRemoved = removedIndices.size();
		for(PxU32 a = 0; a < totalRemoved; ++a)
		{
			if(mKeepMap.test(a))
			{
				offset++;
			}
			else
			{
				if(offset)
				{
					itCms[writeIndex] = itCms[writeIndex + offset];
					itInputs[writeIndex] = itInputs[writeIndex + offset];
					itSI[writeIndex] = itSI[writeIndex + offset];
					itR[writeIndex] = itR[writeIndex + offset];
					itTor[writeIndex] = itTor[writeIndex + offset];
				}
				writeIndex++;
			}
		}
		itCms.forceSize_Unsafe(finalSize + removeSize);
		itInputs.forceSize_Unsafe(finalSize + removeSize);
		itSI.forceSize_Unsafe(finalSize + removeSize);
		itR.forceSize_Unsafe(finalSize + removeSize);
		itTor.forceSize_Unsafe(finalSize + removeSize);

	}

	{
		PX_PROFILE_ZONE("GpuNarrowPhase.removeLostPairs2", 0);
		for(PxU32 a = removeSize; a > 0; --a)
		{
			PxU32 removedIndex = removedIndices[a - 1];
			itCms.replaceWithLast(removedIndex);
			itInputs.replaceWithLast(removedIndex);
			itSI.replaceWithLast(removedIndex);
			itR.replaceWithLast(removedIndex);
			itTor.replaceWithLast(removedIndex);
			PxcNpWorkUnit& workUnit = itCms[removedIndex]->getWorkUnit();
			PxU32 npIndex = contactManagers.computeId(removedIndex);
			workUnit.mNpIndex = npIndex;

			if(workUnit.mStatusFlags & PxcNpWorkUnitStatusFlag::eHAS_TOUCH)
				processPartitionEdges(mIslandSim->mGpuData, workUnit);
		}
	}
	
	removedIndices.forceSize_Unsafe(0);
}

#if GPU_NP_DEBUG

bool validateInputPairs(PxgContactManagers& gpuConvexConvexManagers, PxgGpuContactManagers& gpuContactManagers, PxCudaContext* cudaContext)
{
	PxU32 count = gpuConvexConvexManagers.mCpuContactManagerMapping.size();

	PxArray<PxgContactManagerInput> tempInputArray(count);

	cudaContext->memcpyDtoH(tempInputArray.begin(), gpuContactManagers.mContactManagerInputData.getDevicePtr(), sizeof(PxgContactManagerInput)* count);

	PxPinnedArray<PxgContactManagerInput>& iter = gpuConvexConvexManagers.mGpuInputContactManagers;

	for (PxU32 i = 0; i < count; ++i)
	{
		if (tempInputArray[i].shapeRef0 != iter[i].shapeRef0)
			return false;
		if (tempInputArray[i].shapeRef1 != iter[i].shapeRef1)
			return false;
		if (tempInputArray[i].transformCacheRef0 != iter[i].transformCacheRef0)
			return false;
		if (tempInputArray[i].transformCacheRef1 != iter[i].transformCacheRef1)
			return false;
	}
	return true;
}

#endif

void PxgGpuNarrowphaseCore::removeLostPairs()
{
	/*
		This remove algorithm mirrors the behavior of the GPU reduce-and-remove algorithm.
		The approach is as follows:
		(1) Sort the removed indices in increasing order
		(2) Remove all elements that are removed whose indices are beyond the end of the new array size (compact down using shuffles)
		(3) Replace-with-last all elements that are removed whose indices are before the end of the new array size, processing these pairs backwards
	*/

	//Process removed/lost pairs on the GPU..

	PX_PROFILE_ZONE("GpuNarrowPhase.removeLostPairs", 0);

	PxScopedCudaLock lock(*mCudaContextManager);

	PxU32 removedIndSize = mRemovedIndices[GPU_BUCKET_ID::eConvex]->size();
	PxU32 contactCount = mContactManagers[GPU_BUCKET_ID::eConvex]->getNbFirstPassTests();

	for (PxU32 i = GPU_BUCKET_ID::eConvexPlane; i < GPU_BUCKET_ID::eCount; ++i)
	{
		removedIndSize = PxMax(mRemovedIndices[i]->size(), removedIndSize);
		contactCount = PxMax(mContactManagers[i]->getNbFirstPassTests(), contactCount);
	}

	mPairManagementBuffers.mTempRunsumArray.allocate(sizeof(PxU32)* contactCount, PX_FL);
	mPairManagementBuffers.mBlockAccumulationArray.allocate(sizeof(PxU32)* PxgNarrowPhaseGridDims::REMOVE_CONTACT_MANAGERS, PX_FL);
	mPairManagementBuffers.mRemovedIndicesArray.allocate(removedIndSize * sizeof(PxU32), PX_FL);


	//use manifold
	for (PxU32 i = GPU_BUCKET_ID::eConvex; i <= GPU_BUCKET_ID::eConvexPlane; ++i)
	{
		if (mRemovedIndices[i]->size() > 0)
		{
			//KS - must sort the buffer before the DMA to ensure that there isn't a race condition in the DMA/sort algorithm that may be running in parallel with it
			PxSort(mRemovedIndices[i]->begin(), mRemovedIndices[i]->size(), PxLess<PxU32>());

			removeLostPairsGpu(i, PxgKernelIds::REMOVE_CONTACT_MANAGERS_5, true);
			removeLostPairsInternal(*mRemovedIndices[i], mContactManagers[i]->mContactManagers);
			//validateInputPairs(mGpuConvexConvexManagers, mGpuContactManagers, mCudaContext);
		}
	}

	//use multi-manifold
	for (PxU32 i = GPU_BUCKET_ID::eConvexTrimesh; i <= GPU_BUCKET_ID::eTrianglePlane; ++i)
	{
		if (mRemovedIndices[i]->size() > 0)
		{
			//KS - must sort the buffer before the DMA to ensure that there isn't a race condition in the DMA/sort algorithm that may be running in parallel with it
			PxSort(mRemovedIndices[i]->begin(), mRemovedIndices[i]->size(), PxLess<PxU32>());

			removeLostPairsGpu(i, PxgKernelIds::REMOVE_CONTACT_MANAGERS_5_CVXTRI, true);
			removeLostPairsInternal(*mRemovedIndices[i], mContactManagers[i]->mContactManagers);
			//validateInputPairs(mGpuConvexConvexManagers, mGpuContactManagers, mCudaContext);
		}
	}

	//no manifold
	for (PxU32 i = GPU_BUCKET_ID::eSphere; i < GPU_BUCKET_ID::eCount; ++i)
	{
		if (mRemovedIndices[i]->size() > 0)
		{
			//KS - must sort the buffer before the DMA to ensure that there isn't a race condition in the DMA/sort algorithm that may be running in parallel with it
			PxSort(mRemovedIndices[i]->begin(), mRemovedIndices[i]->size(), PxLess<PxU32>());

			removeLostPairsGpu(i, PxgKernelIds::REMOVE_CONTACT_MANAGERS_5, false);
			removeLostPairsInternal(*mRemovedIndices[i], mContactManagers[i]->mContactManagers);
			//validateInputPairs(mGpuConvexConvexManagers, mGpuContactManagers, mCudaContext);
		}
	}

	//validateInputPairs(mGpuConvexConvexManagers, mGpuContactManagers, mCudaContext);
}

template <typename ManagementData, typename Manifold>
void PxgGpuNarrowphaseCore::removeLostPairsGpuInternal(ManagementData& cpuBuffer, CUdeviceptr gpuBuffer,
	PxgContactManagers& contactManagers, PxgGpuContactManagers& gpuContactManagers, PxInt32ArrayPinned& removedIndices, PxgGpuPairManagementBuffers& pairManagementBuffers,
	PxU16 stage5KernelID, const bool copyManifold)
{
	if (removedIndices.size() > 0)
	{
		cpuBuffer.mBlockSharedAccumulator = (PxU32*)pairManagementBuffers.mBlockAccumulationArray.getDevicePtr();
		cpuBuffer.mTempAccumulator = (PxU32*)pairManagementBuffers.mTempRunsumArray.getDevicePtr();
		cpuBuffer.mRemoveIndices = (PxU32*)pairManagementBuffers.mRemovedIndicesArray.getDevicePtr();

		cpuBuffer.mContactManagerInputData = (PxgContactManagerInput*)gpuContactManagers.mContactManagerInputData.getDevicePtr();
		cpuBuffer.mContactManagerOutputData = (PxsContactManagerOutput*)gpuContactManagers.mContactManagerOutputData.getDevicePtr();
		cpuBuffer.mPersistentContactManagers = (Manifold*)gpuContactManagers.mPersistentContactManifolds.getDevicePtr();
		cpuBuffer.mCpuContactManagerMapping = (PxsContactManager**)gpuContactManagers.mCpuContactManagerMapping.getDevicePtr();
		cpuBuffer.mShapeInteractions = (Sc::ShapeInteraction**)gpuContactManagers.mShapeInteractions.getDevicePtr();
		cpuBuffer.mRestDistances = (PxReal*)gpuContactManagers.mRestDistances.getDevicePtr();
		cpuBuffer.mTorsionalData = (PxsTorsionalFrictionData*)gpuContactManagers.mTorsionalProperties.getDevicePtr();
		
		cpuBuffer.mNbPairs = contactManagers.mCpuContactManagerMapping.size();
		cpuBuffer.mNbToRemove = removedIndices.size();

		mCudaContext->memcpyHtoDAsync(gpuBuffer, &cpuBuffer, sizeof(ManagementData), mStream);

		mCudaContext->memcpyHtoDAsync(pairManagementBuffers.mRemovedIndicesArray.getDevicePtr(), removedIndices.begin(), removedIndices.size()*sizeof(PxU32), mStream);

		CUfunction remove1 = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::REMOVE_CONTACT_MANAGERS_1);
		CUfunction remove2 = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::REMOVE_CONTACT_MANAGERS_2);
		CUfunction remove3 = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::REMOVE_CONTACT_MANAGERS_3);
		CUfunction remove4 = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::REMOVE_CONTACT_MANAGERS_4);
		CUfunction remove5 = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(stage5KernelID);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(gpuBuffer),
			PX_CUDA_KERNEL_PARAM(copyManifold)
		};

		CUresult result = mCudaContext->launchKernel(remove1, PxgNarrowPhaseGridDims::REMOVE_CONTACT_MANAGERS, 1, 1, WARP_SIZE, PxgNarrowPhaseBlockDims::REMOVE_CONTACT_MANAGERS / WARP_SIZE, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		result = mCudaContext->launchKernel(remove2, PxgNarrowPhaseGridDims::REMOVE_CONTACT_MANAGERS, 1, 1, WARP_SIZE, PxgNarrowPhaseBlockDims::REMOVE_CONTACT_MANAGERS / WARP_SIZE, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		//result = mCudaContext->streamSynchronize(mStream);
		PX_UNUSED(result);
		PX_ASSERT(result == CUDA_SUCCESS);

		result = mCudaContext->launchKernel(remove3, PxgNarrowPhaseGridDims::REMOVE_CONTACT_MANAGERS, 1, 1, WARP_SIZE, PxgNarrowPhaseBlockDims::REMOVE_CONTACT_MANAGERS / WARP_SIZE, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		
		//result = mCudaContext->streamSynchronize(mStream);

		PX_ASSERT(result == CUDA_SUCCESS);

		result = mCudaContext->launchKernel(remove4, PxgNarrowPhaseGridDims::REMOVE_CONTACT_MANAGERS, 1, 1, WARP_SIZE, PxgNarrowPhaseBlockDims::REMOVE_CONTACT_MANAGERS / WARP_SIZE, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		//result = mCudaContext->streamSynchronize(mStream);

		PX_ASSERT(result == CUDA_SUCCESS);

		result = mCudaContext->launchKernel(remove5, PxgNarrowPhaseGridDims::REMOVE_CONTACT_MANAGERS, 1, 1, WARP_SIZE, PxgNarrowPhaseBlockDims::REMOVE_CONTACT_MANAGERS / WARP_SIZE, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(result == CUDA_SUCCESS);

#if GPU_NP_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"GPU removeContactManagers fail to launch kernel stage 1!!!\n");
#endif
	}
}


void PxgGpuNarrowphaseCore::updateContactDistance(const PxReal* contactDistances, const PxU32 numContactDistance)
{
	if (numContactDistance)
	{
		const PxU32 allocatedSize = numContactDistance * sizeof(PxReal);
		mGpuContactDistance.allocate(allocatedSize, PX_FL);
		mCudaContext->memcpyHtoDAsync(mGpuContactDistance.getDevicePtr(), contactDistances, allocatedSize, mStream);
	}
}

void PxgGpuNarrowphaseCore::adjustNpIndices(PxgNewContactManagers& newContactManagers, PxPinnedArray<PxgContactManagerInput>& itMainInputs,
	PxPinnedArray<PxsContactManager*>& itCms, PxPinnedArray<const Sc::ShapeInteraction*>& itSIs, 
	PxFloatArrayPinned& itR, PxPinnedArray<PxsTorsionalFrictionData>& itTor,
	PxPinnedArray<PxgContactManagerInput>& itNewInputs,
	PxPinnedArray<PxsContactManager*>& itNewCms,
	PxPinnedArray<const Sc::ShapeInteraction*>& itNewSIs, PxFloatArrayPinned& itNewR,
	PxPinnedArray<PxsTorsionalFrictionData>& itNewTor)
{
	for(PxU32 i = 0; i < newContactManagers.mCpuContactManagerMapping.size(); ++i)
	{
		PxU32 index = itCms.size();
		PxsContactManager* cm = itNewCms[i];
		PxcNpWorkUnit& unit = cm->getWorkUnit();
		unit.mNpIndex = newContactManagers.computeId(index);

		if(unit.mStatusFlags & PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH)
		{
			unit.mStatusFlags &= (~PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH);

			processPartitionEdges(mIslandSim->mGpuData, unit);
		}

		itMainInputs.pushBack(itNewInputs[i]);
		itCms.pushBack(cm);
		itSIs.pushBack(itNewSIs[i]);
		itR.pushBack(itNewR[i]);
		itTor.pushBack(itNewTor[i]);

	}
	newContactManagers.clear();
}

void PxgGpuNarrowphaseCore::appendContactManagers(PxsContactManagerOutput* /*cmOutputs*/, PxU32 /*nbFallbackPairs*/)
{
	PX_PROFILE_ZONE("GpuNarrowPhase.appendContactManagers", 0);

	PxScopedCudaLock lock(*mCudaContextManager);

	for (PxU32 i = GPU_BUCKET_ID::eConvex; i < GPU_BUCKET_ID::eCount; ++i)
	{
		mContactManagers[i]->allocateContactManagers();

		appendContactManagersGpu(mContactManagers[i]->getNbFirstPassTests(), mContactManagers[i]->getNbSecondPassTests(),
			mGpuContactManagers[i]->mContactManagers, mGpuContactManagers[i]->mNewContactManagers, BUCKET_ManifoldSize[i]);
	
		PxPinnedArray<PxgContactManagerInput>& itMainInputs = mContactManagers[i]->mContactManagers.mGpuInputContactManagers;
		PxPinnedArray<PxsContactManager*>&  itCms = mContactManagers[i]->mContactManagers.mCpuContactManagerMapping;
		PxPinnedArray<const Sc::ShapeInteraction*>&  itSIs = mContactManagers[i]->mContactManagers.mShapeInteractions;
		PxFloatArrayPinned& itR = mContactManagers[i]->mContactManagers.mRestDistances;
		PxPinnedArray<PxsTorsionalFrictionData>& itTor = mContactManagers[i]->mContactManagers.mTorsionalProperties;

		PxPinnedArray<PxgContactManagerInput>& itNewInputs = mContactManagers[i]->mNewContactManagers.mGpuInputContactManagers;
		PxPinnedArray<PxsContactManager*>& itNewCms = mContactManagers[i]->mNewContactManagers.mCpuContactManagerMapping;
		PxPinnedArray<const Sc::ShapeInteraction*>& itNewSIs = mContactManagers[i]->mNewContactManagers.mShapeInteractions;
		PxFloatArrayPinned& itNewR = mContactManagers[i]->mNewContactManagers.mRestDistances;
		PxPinnedArray<PxsTorsionalFrictionData>& itNewTor = mContactManagers[i]->mNewContactManagers.mTorsionalProperties;

		adjustNpIndices(mContactManagers[i]->mNewContactManagers, itMainInputs, itCms, itSIs, itR, itTor, itNewInputs, itNewCms, itNewSIs, itNewR, itNewTor);
	}
	
	waitAndResetCopyQueues();
	waitAndResetCopyQueuesBp();
}

void PxgGpuNarrowphaseCore::computeRigidsToShapes()
{
	PxgShapeManager& shapeManager = getGpuShapeManager();

	const PxU32 totalNumShapes = shapeManager.mMaxTransformCacheID + 1;

	//we need to allocate enough memory (x4) for the radix sort because each thread read 4 elements
	const PxU32 numOfKeys = (totalNumShapes + 3) &(~3);

	mTempGpuRigidIndiceBuf.allocate(numOfKeys * sizeof(PxNodeIndex), PX_FL);
	mTempGpuShapeIndiceBuf.allocate(numOfKeys * sizeof(PxU32), PX_FL);

	CUdeviceptr rigidIndiced = shapeManager.mGpuRigidIndiceBuffer.getDevicePtr();
	CUdeviceptr tempRigidIndiced = shapeManager.mGpuTempRigidIndiceBuffer.getDevicePtr();
	CUdeviceptr tempRigidIndiceBitsd = shapeManager.mGpuTempRigidBitIndiceBuffer.getDevicePtr();
	CUdeviceptr rankd = shapeManager.mGpuShapeIndiceBuffer.getDevicePtr();

	// always start with the state from CPU which has all the updates, and the indices are still in the right place.
	mCudaContext->memcpyDtoDAsync(rigidIndiced, shapeManager.mGpuShapesRemapTableBuffer.getDevicePtr(), sizeof(PxNodeIndex) * totalNumShapes, mStream);
	mCudaContext->memcpyDtoDAsync(shapeManager.mGpuShapeIndiceBuffer.getDevicePtr(), shapeManager.mGpuUnsortedShapeIndicesBuffer.getDevicePtr(), sizeof(PxU32) * totalNumShapes, mStream);

	//copy the original gpu node index into a temporary buffer
	mCudaContext->memcpyDtoDAsync(tempRigidIndiced, rigidIndiced, sizeof(PxNodeIndex) * totalNumShapes, mStream);

	{
		const bool lower32Bit = true;
		//copy the lower 32 bit to the temp contact rigid index buffer
		CUfunction copyFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_COPY_BITS2);

		PxCudaKernelParam copyKernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(rigidIndiced),
			PX_CUDA_KERNEL_PARAM(tempRigidIndiceBitsd),
			PX_CUDA_KERNEL_PARAM(rankd),
			PX_CUDA_KERNEL_PARAM(totalNumShapes),
			PX_CUDA_KERNEL_PARAM(lower32Bit)
		};

		const PxU32 numThreadsPerBlock = 256;
		const PxU32 numBlocks = (totalNumShapes + numThreadsPerBlock - 1) / numThreadsPerBlock;

		CUresult resultR = mCudaContext->launchKernel(copyFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, copyKernelParams, sizeof(copyKernelParams), 0, PX_FL);
		if (resultR != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopyBits2 fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		resultR = mCudaContext->streamSynchronize(mStream);
		if (resultR != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopyBits2 fail!!\n");
#endif
		}

	//update radix sort desc(keys are the value we want to sort)
	CUdeviceptr inputKeyd = tempRigidIndiceBitsd;
	CUdeviceptr inputRankd = rankd;
	CUdeviceptr outputKeyd = mTempGpuRigidIndiceBuf.getDevicePtr();
	CUdeviceptr outputRankd = mTempGpuShapeIndiceBuf.getDevicePtr();
	CUdeviceptr radixCountd = mRadixCountTotalBuf.getDevicePtr();

	mRSDesc[0].inputKeys = reinterpret_cast<PxU32*>(inputKeyd);
	mRSDesc[0].inputRanks = reinterpret_cast<PxU32*>(inputRankd);
	mRSDesc[0].outputKeys = reinterpret_cast<PxU32*>(outputKeyd);
	mRSDesc[0].outputRanks = reinterpret_cast<PxU32*>(outputRankd);
	mRSDesc[0].radixBlockCounts = reinterpret_cast<PxU32*>(radixCountd);

	mRSDesc[1].outputKeys = reinterpret_cast<PxU32*>(inputKeyd);
	mRSDesc[1].outputRanks = reinterpret_cast<PxU32*>(inputRankd);
	mRSDesc[1].inputKeys = reinterpret_cast<PxU32*>(outputKeyd);
	mRSDesc[1].inputRanks = reinterpret_cast<PxU32*>(outputRankd);
	mRSDesc[1].radixBlockCounts = reinterpret_cast<PxU32*>(radixCountd);

	//dma up the descriptor
	mCudaContext->memcpyHtoDAsync(mRadixSortDescBuf[0].getDevicePtr(), (void*)&mRSDesc[0], sizeof(PxgRadixSortDesc), mStream);
	mCudaContext->memcpyHtoDAsync(mRadixSortDescBuf[1].getDevicePtr(), (void*)&mRSDesc[1], sizeof(PxgRadixSortDesc), mStream);

	//run radix sort
	CUfunction radixFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_MULTIBLOCK_COUNT);
	CUfunction calculateRanksFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_CALCULATERANKS_MULTIBLOCK_COUNT);

	//sort the lower 32 bits
	{
		PxU32 startBit = 0;
		const PxU32 numPass = 8;

		for (PxU32 i = 0; i < numPass; ++i)
		{
			const PxU32 descIndex = i & 1;

			CUdeviceptr rsDesc = mRadixSortDescBuf[descIndex].getDevicePtr();

			PxCudaKernelParam radixSortKernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(rsDesc),
				PX_CUDA_KERNEL_PARAM(totalNumShapes),
				PX_CUDA_KERNEL_PARAM(startBit)
			};

			CUresult  resultR = mCudaContext->launchKernel(radixFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
			if (resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radix sort fail to launch kernel!!\n");

			resultR = mCudaContext->launchKernel(calculateRanksFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
			if (resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radix sort fail to launch kernel!!\n");

			startBit += 4;
		}

#if GPU_NP_DEBUG
		CUresult result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radix sort fail!!\n");

	/*	PxArray<PxNodeIndex> rigidIndiceAfter;
		rigidIndiceAfter.reserve(sizeof(PxNodeIndex) * numOfKeys);
		PxArray<PxU32> shapeIndicesAfter;
		shapeIndicesAfter.reserve(sizeof(PxU32) * numOfKeys);

		mCudaContext->memcpyDtoH(rigidIndiceAfter.begin(), shapeManager.mGpuRigidIndiceBuffer.getDevicePtr(), sizeof(PxNodeIndex) * numOfKeys);
		mCudaContext->memcpyDtoH(shapeIndicesAfter.begin(), shapeManager.mGpuShapeIndiceBuffer.getDevicePtr(), sizeof(PxU32) * numOfKeys);

		int bob;
		PX_UNUSED(bob);*/
#endif
	}

	{
		const bool lower32Bit = false;
		//copy the higher 32 bit to the team contact rigid index buffer
		CUfunction copyFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_COPY_BITS2);

		PxCudaKernelParam copyKernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(rigidIndiced),
			PX_CUDA_KERNEL_PARAM(tempRigidIndiceBitsd),
			PX_CUDA_KERNEL_PARAM(rankd),
			PX_CUDA_KERNEL_PARAM(totalNumShapes),
			PX_CUDA_KERNEL_PARAM(lower32Bit)
		};

		const PxU32 numThreadsPerBlock = 256;
		const PxU32 numBlocks = (totalNumShapes + numThreadsPerBlock - 1) / numThreadsPerBlock;

		CUresult resultR = mCudaContext->launchKernel(copyFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, copyKernelParams, sizeof(copyKernelParams), 0, PX_FL);
		if (resultR != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopyBits2 fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		resultR = mCudaContext->streamSynchronize(mStream);
		if (resultR != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopyBits2 fail!!\n");
#endif
	}

	//sort the higher 32 bits
	{
		PxU32 startBit = 0;
		const PxU32 numPass = 8;

		for (PxU32 i = 0; i < numPass; ++i)
		{
			const PxU32 descIndex = i & 1;

			CUdeviceptr rsDesc = mRadixSortDescBuf[descIndex].getDevicePtr();

			PxCudaKernelParam radixSortKernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(rsDesc),
				PX_CUDA_KERNEL_PARAM(totalNumShapes),
				PX_CUDA_KERNEL_PARAM(startBit)
			};

			CUresult  resultR = mCudaContext->launchKernel(radixFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
			if (resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radix sort fail to launch kernel!!\n");

			resultR = mCudaContext->launchKernel(calculateRanksFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
			if (resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radix sort fail to launch kernel!!\n");

			startBit += 4;
		}

#if GPU_NP_DEBUG
		CUresult result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radix sort fail!!\n");

		/*PxArray<PxNodeIndex> rigidIndiceAfter;
		rigidIndiceAfter.reserve(sizeof(PxNodeIndex) * numOfKeys);
		PxArray<PxU32> shapeIndicesAfter;
		shapeIndicesAfter.reserve(sizeof(PxU32) * numOfKeys);

		mCudaContext->memcpyDtoH(rigidIndiceAfter.begin(), shapeManager.mGpuRigidIndiceBuffer.getDevicePtr(), sizeof(PxNodeIndex) * numOfKeys);
		mCudaContext->memcpyDtoH(shapeIndicesAfter.begin(), shapeManager.mGpuShapeIndiceBuffer.getDevicePtr(), sizeof(PxU32) * numOfKeys);

		int bob;
		PX_UNUSED(bob);*/
#endif
	}

	{
		//copy the original rigidId to the sorted buffer based on mContactRemapSortedByRigidBuf
		CUfunction copyFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_COPY_VALUE2);

		PxCudaKernelParam copyKernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(tempRigidIndiced), //in
			PX_CUDA_KERNEL_PARAM(rigidIndiced), //out
			PX_CUDA_KERNEL_PARAM(rankd),
			PX_CUDA_KERNEL_PARAM(totalNumShapes)
		};

		const PxU32 numThreadsPerBlock = 256;
		const PxU32 numBlocks = (totalNumShapes + numThreadsPerBlock - 1) / numThreadsPerBlock;
		CUresult resultR = mCudaContext->launchKernel(copyFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, copyKernelParams, sizeof(copyKernelParams), 0, PX_FL);
		if (resultR != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopy2 fail to launch kernel!!\n");

#if GPU_NP_DEBUG
		resultR = mCudaContext->streamSynchronize(mStream);
		if (resultR != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopy2 fail!!\n");


	/*	PxArray<PxNodeIndex> rigidIndiceAfter;
		rigidIndiceAfter.reserve(sizeof(PxNodeIndex) * numOfKeys);
		PxArray<PxU32> shapeIndicesAfter;
		shapeIndicesAfter.reserve(sizeof(PxU32) * numOfKeys);

		mCudaContext->memcpyDtoH(rigidIndiceAfter.begin(), shapeManager.mGpuRigidIndiceBuffer.getDevicePtr(), sizeof(PxNodeIndex) * numOfKeys);
		mCudaContext->memcpyDtoH(shapeIndicesAfter.begin(), shapeManager.mGpuShapeIndiceBuffer.getDevicePtr(), sizeof(PxU32) * numOfKeys);

		int bob;
		PX_UNUSED(bob);*/
#endif
	}
}

void PxgGpuNarrowphaseCore::prepareGpuNarrowphase(PxsTransformCache& cache, const PxReal* contactDistances,
	bool hasContactDistanceChanged)
{

	if (hasContactDistanceChanged)
		updateContactDistance(contactDistances, cache.getTotalSize());

	uploadDataChunksToGpu();

	mPatchAndContactCountersReadback->contactsBytes = 0;
	mPatchAndContactCountersReadback->patchesBytes = 0;
	mPatchAndContactCountersReadback->forceAndIndiceBytes = 0;

	CUresult result = mCudaContext->memsetD8Async(mPatchAndContactCountersOnDevice.getDevicePtr(), 0, 
		sizeof(PxgPatchAndContactCounters), mStream);

	if(result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,"prepareGpuNarrowphase GPU error! code %d \n", result);

	mGpuShapesManager.mHasShapeInstanceChanged = false;
}

bool PxgGpuNarrowphaseCore::evaluateSDFDistances(PxVec4* PX_RESTRICT localGradientAndSDFConcatenated, const PxShapeGPUIndex* PX_RESTRICT shapeIndices, const PxVec4* PX_RESTRICT localSamplePointsConcatenated, const PxU32* PX_RESTRICT samplePointCountPerShape, PxU32 nbElements, PxU32 maxPointCount, CUevent startEvent, CUevent finishEvent)
{
	PxScopedCudaLock lock(*mCudaContextManager);
	bool success = true;

	if (startEvent)
	{
		mCudaContext->streamWaitEvent(mStream, startEvent);
	}

	PxgShape* gpuShapes = reinterpret_cast<PxgShape*>(mGpuShapesManager.mGpuShapesBuffer.getDevicePtr());

	CUfunction cuFunc = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::EVALUATE_POINT_DISTANCES_SDF);
	PxCudaKernelParam kernelParams[] =
	{
		PX_CUDA_KERNEL_PARAM(gpuShapes),
		PX_CUDA_KERNEL_PARAM(shapeIndices),
		PX_CUDA_KERNEL_PARAM(localSamplePointsConcatenated),
		PX_CUDA_KERNEL_PARAM(samplePointCountPerShape),
		PX_CUDA_KERNEL_PARAM(maxPointCount),
		PX_CUDA_KERNEL_PARAM(localGradientAndSDFConcatenated)
	};

	const PxU32 numThreadsPerBlock = 1024;
	const PxU32 numBlocks = (maxPointCount + (numThreadsPerBlock - 1)) / numThreadsPerBlock;
	CUresult result = mCudaContext->launchKernel(cuFunc, numBlocks, nbElements, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
	if (result != CUDA_SUCCESS)
	{
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU evaluatePointDistancesSDF fail to launch !!\n");
		success = false;
	}

	if (finishEvent)
	{
		mCudaContext->eventRecord(finishEvent, mStream);
	}
	else
	{
		result = mCudaContext->streamSynchronize(mStream);
		success = (result == CUDA_SUCCESS);
	}

	return success;
}

bool PxgGpuNarrowphaseCore::copyContactData(void* PX_RESTRICT data, PxU32* PX_RESTRICT numContactPairs, const PxU32 maxContactPairs, CUevent startEvent, CUevent finishEvent, PxU8* PX_RESTRICT baseContactPatches, PxU8* PX_RESTRICT baseContactPoints, PxU8* PX_RESTRICT baseContactForces)
{
	PxScopedCudaLock lock(*mCudaContextManager);
	
	bool success = true;

	if (mTotalNumPairs)
	{
		if (startEvent)
		{
			mCudaContext->streamWaitEvent(mStream, startEvent);
		}

		CUdeviceptr numPairsd = reinterpret_cast<CUdeviceptr>(numContactPairs);
		mCudaContext->memsetD32Async(numPairsd, 0, 1, mStream);
		//in fetchNarrowPhaseResults, we append all the contacts inform into a eConvex's list
		CUdeviceptr cvxInputDeviceptr = mGpuContactManagers[GPU_BUCKET_ID::eConvex]->mContactManagers.mContactManagerInputData.getDevicePtr();
		CUdeviceptr cvxDeviceptr = mGpuContactManagers[GPU_BUCKET_ID::eConvex]->mContactManagers.mContactManagerOutputData.getDevicePtr();
		CUdeviceptr shapeToRigidRemapTabled = mGpuShapesManager.mGpuShapesRemapTableBuffer.getDevicePtr();
		CUdeviceptr transformCacheIdToActorTabled = mGpuShapesManager.mGpuTransformCacheIdToActorTableBuffer.getDevicePtr();

		PxMutex::ScopedLock lock2(mIntermStackAlloc.mMutex);

		CUdeviceptr gpuIntermNumPairs = reinterpret_cast<CUdeviceptr>(mIntermStackAlloc.allocateAligned(256, PxgNarrowPhaseGridDims::COMPRESS_CONTACT * sizeof(PxU32)));
		mCudaContext->memsetD32Async(gpuIntermNumPairs, 0, PxgNarrowPhaseGridDims::COMPRESS_CONTACT, mStream);

		CUresult result;

		{
			PxCudaKernelParam kernelParams_stage1[] =
			{
				PX_CUDA_KERNEL_PARAM(cvxDeviceptr),
				PX_CUDA_KERNEL_PARAM(mTotalNumPairs),
				PX_CUDA_KERNEL_PARAM(gpuIntermNumPairs)
			};

			//const PxU32 numWarpsPerBlock = PxgNarrowPhaseBlockDims::COMPRESS_CONTACT / WARP_SIZE;

			CUfunction kernelFunction_stage1 = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::COMPRESS_CONTACT_STAGE_1);

			result = mCudaContext->launchKernel(kernelFunction_stage1, PxgNarrowPhaseGridDims::COMPRESS_CONTACT, 1, 1, PxgNarrowPhaseBlockDims::COMPRESS_CONTACT, 1, 1, 0,
				mStream, kernelParams_stage1, sizeof(kernelParams_stage1), 0, PX_FL);

			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU compressContactStage1 fail to launch kernel stage 1!!\n");

#if GPU_NP_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU compressContactStage1 fail to launch kernel stage 1!!!\n");
#endif
		}

		{
			CUdeviceptr gpuContactPatches = reinterpret_cast<CUdeviceptr>(data);

			CUdeviceptr frictionPatches = mGpuContext->mGpuSolverCore->mFrictionPatches.getDevicePtr();

			PxCudaKernelParam kernelParams_stage2[] =
			{
				PX_CUDA_KERNEL_PARAM(cvxInputDeviceptr),
				PX_CUDA_KERNEL_PARAM(cvxDeviceptr),
				PX_CUDA_KERNEL_PARAM(mTotalNumPairs),
				PX_CUDA_KERNEL_PARAM(shapeToRigidRemapTabled),
				PX_CUDA_KERNEL_PARAM(transformCacheIdToActorTabled),
				PX_CUDA_KERNEL_PARAM(gpuIntermNumPairs),
				PX_CUDA_KERNEL_PARAM(baseContactPatches),
				PX_CUDA_KERNEL_PARAM(baseContactPoints),
				PX_CUDA_KERNEL_PARAM(baseContactForces),
				PX_CUDA_KERNEL_PARAM(mPatchStream),
				PX_CUDA_KERNEL_PARAM(mContactStream),
				PX_CUDA_KERNEL_PARAM(mForceAndIndiceStream),
				PX_CUDA_KERNEL_PARAM(frictionPatches),
				PX_CUDA_KERNEL_PARAM(numPairsd),
				PX_CUDA_KERNEL_PARAM(gpuContactPatches),
				PX_CUDA_KERNEL_PARAM(maxContactPairs)

			};

			CUfunction kernelFunction_stage2 = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::COMPRESS_CONTACT_STAGE_2);

			result = mCudaContext->launchKernel(kernelFunction_stage2, PxgNarrowPhaseGridDims::COMPRESS_CONTACT, 1, 1, PxgNarrowPhaseBlockDims::COMPRESS_CONTACT, 1, 1, 0,
				mStream, kernelParams_stage2, sizeof(kernelParams_stage2), 0, PX_FL);

			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU compressContactStage2 fail to launch kernel stage 1!!\n");

#if GPU_NP_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU compressContactStage2 fail to launch kernel stage 1!!!\n");

			//PxArray<PxGpuContactPair> contactPairs;
			//contactPairs.reserve(maxContactPairs);
			//contactPairs.forceSize_Unsafe(maxContactPairs);

			//mCudaContext->memcpyDtoH(contactPairs.begin(), gpuContactPatches, sizeof(PxGpuContactPair) * maxContactPairs);

			//PxGpuContactPair& pair0 = contactPairs[0];
			//const PxU32 numContactPatches = pair0.nbPatches;

			//PxArray<PxContactPatch> patches;
			//patches.reserve(numContactPatches);
			//patches.forceSize_Unsafe(numContactPatches);

			//mCudaContext->memcpyDtoH(patches.begin(), (CUdeviceptr)pair0.contactPatches, sizeof(PxContactPatch) * numContactPatches);

			//const PxU32 numContacts = pair0.nbContacts;
			//PxArray<PxContact> contacts;
			//contacts.reserve(numContacts);
			//contacts.forceSize_Unsafe(numContacts);
			//mCudaContext->memcpyDtoH(contacts.begin(), (CUdeviceptr)pair0.contactPoints, sizeof(PxContact) * numContacts);

			//PxArray<PxReal> forces;
			//forces.reserve(numContacts);
			//forces.forceSize_Unsafe(numContacts);
			//mCudaContext->memcpyDtoH(forces.begin(), (CUdeviceptr)pair0.contactForces, sizeof(PxReal) * numContacts);

			//PxU32 uNumPairs;
			//mCudaContext->memcpyDtoH(&uNumPairs, numPairsd, sizeof(PxU32));
			//PxU32 bob = 0;
			//PX_UNUSED(bob);
#endif
		}

		if (finishEvent)
		{
			mCudaContext->eventRecord(finishEvent, mStream);
		}
		else
		{
			result = mCudaContext->streamSynchronize(mStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "copyContactData: CUDA error, code %u\n", result);
			success = (result == CUDA_SUCCESS);
		}

		mIntermStackAlloc.reset();
	}

	return success;
}

void PxgGpuNarrowphaseCore::synchronizedStreams(CUstream artiStream)
{
	PX_PROFILE_ZONE("PxgGpuNarrowphaseCore.synchronizedStreams", 0);

	CUresult result = mCudaContext->eventRecord(mDirectApiDmaEvent, mStream);
	PX_ASSERT(result == CUDA_SUCCESS);

	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "SynchronizeStreams cuEventRecord failed\n");

	result = mCudaContext->streamWaitEvent(artiStream, mDirectApiDmaEvent);
	PX_ASSERT(result == CUDA_SUCCESS);

	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "SynchronizeStreams cuStreamWaitEvent failed\n");
}

void PxgGpuNarrowphaseCore::drawContacts(PxRenderOutput& out, CUdeviceptr contactsd, CUdeviceptr normalPensd, const PxU32 numContacts)
{
	PxArray<float4> points(numContacts);
	PxArray<float4> normalPen(numContacts);

	mCudaContext->memcpyDtoH(points.begin(), contactsd, sizeof(float4) * numContacts);
	mCudaContext->memcpyDtoH(normalPen.begin(), normalPensd, sizeof(float4) * numContacts);

	const PxReal size = 0.02f;
	const PxU32 color = 0xff00ffff;
	const PxU32 color2 = 0xffffffff;
	const PxU32 color3 = 0xffffff00;

	//PxReal maxY = 0.f;
	for (PxU32 i = 0; i < numContacts; ++i)
	{
		PxVec3 a(points[i].x, points[i].y, points[i].z);
		PxVec3 n(normalPen[i].x, normalPen[i].y, normalPen[i].z);
		const PxReal pen = normalPen[i].w;

		PxVec3 b = a - n * pen;

		const PxVec3 up(0.f, size, 0.f);
		const PxVec3 right(size, 0.f, 0.f);
		const PxVec3 forwards(0.f, 0.f, size);

		const PxMat44 m(PxIdentity);

		out << color << m << PxRenderOutput::LINES << a + up << a - up;
		out << color << m << PxRenderOutput::LINES << a + right << a - right;
		out << color << m << PxRenderOutput::LINES << a + forwards << a - forwards;

		out << color2 << m << PxRenderOutput::LINES << b + up << b - up;
		out << color2 << m << PxRenderOutput::LINES << b + right << b - right;
		out << color2 << m << PxRenderOutput::LINES << b + forwards << b - forwards;

		out << color3 << m << PxRenderOutput::LINES << a << b;
	}
}
