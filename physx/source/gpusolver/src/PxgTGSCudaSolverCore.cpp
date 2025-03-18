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

#include "PxgTGSCudaSolverCore.h"
#include "PxgSolverConstraintDesc.h"
#include "PxgSolverContext.h"
#include "PxgSolverBody.h"
#include "PxgConstraint.h"
#include "PxgFrictionPatch.h"
#include "foundation/PxVec3.h"
#include "PxgDynamicsContext.h"
#include "common/PxProfileZone.h"
#include "CudaKernelWrangler.h"
#include "cudamanager/PxCudaContextManager.h"
#include "PxgSolverKernelIndices.h"
#include "PxgSolverCoreDesc.h"
#include "PxgRadixSortDesc.h"
#include "PxgPartitionNode.h"
#include "PxSceneDesc.h"
#include "PxgSolverConstraintBlock1D.h"
#include "PxgConstraintBlock.h"
#include "PxgCudaMemoryAllocator.h"
#include "PxgCudaUtils.h"
#include "PxgSimulationController.h"
#include "PxgArticulationCore.h"
#include "PxgSimulationCore.h"
#include "PxgKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "PxgHeapMemAllocator.h"
#include "PxgConstraintWriteBack.h"
#include "PxgSolverConstraint1D.h"
#include "PxgArticulationCoreKernelIndices.h"
#include "PxgSoftBodyCore.h"
#include "PxgParticleSystemCore.h"
#include "PxgFEMClothCore.h"
#include "PxgArticulationCoreDesc.h"
#include "PxgSimulationController.h"
#include "DyConstraintPrep.h"
#include "PxgIslandContext.h"

#include "cudamanager/PxCudaContext.h"

//Turn me on for errors when stuff goes wrong and also to be able to capture PVD captures that indicate timers for individual parts of the GPU solver
//pipeline. This makes overall performance about 5% slower so leave me off if you're not profiling using PVD or trying to track down a crash bug.
#define GPU_DEBUG 0

using namespace physx;

PxgTGSCudaSolverCore::PxgTGSCudaSolverCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager, 
	PxgGpuContext* dynamicContext, PxgHeapMemoryAllocatorManager* heapMemoryManager, const PxGpuDynamicsMemoryConfig& init) :
	PxgSolverCore(gpuKernelWrangler, cudaContextManager, dynamicContext, heapMemoryManager),
	mConstraintContactPrepPool(heapMemoryManager, PxsHeapStats::eSOLVER),
	mContactHeaderStream(heapMemoryManager, PxsHeapStats::eSOLVER),
	mContactStream(heapMemoryManager, PxsHeapStats::eSOLVER),
	mFrictionStream(heapMemoryManager, PxsHeapStats::eSOLVER),
	mSolverEncodedReferenceCount(heapMemoryManager, PxsHeapStats::eSOLVER),
	mArtiConstraintBlockResponse(heapMemoryManager, PxsHeapStats::eSOLVER),
		
	mForceThresholdStream(heapMemoryManager, PxsHeapStats::eSOLVER),
	mTmpForceThresholdStream(heapMemoryManager, PxsHeapStats::eSOLVER),

	mConstraint1DBatchIndices(heapMemoryManager, PxsHeapStats::eSOLVER),
	mContactBatchIndices(heapMemoryManager, PxsHeapStats::eSOLVER),
	mArtiContactBatchIndices(heapMemoryManager, PxsHeapStats::eSOLVER),
	mArtiConstraint1dBatchIndices(heapMemoryManager, PxsHeapStats::eSOLVER),
	mAccumulatedForceObjectPairs(heapMemoryManager, PxsHeapStats::eSOLVER),
	mExceededForceElements(heapMemoryManager, PxsHeapStats::eSOLVER),
	mForceChangeThresholdElements(heapMemoryManager, PxsHeapStats::eSOLVER),
	mThresholdStreamAccumulatedForce(heapMemoryManager, PxsHeapStats::eSOLVER),
	mBlocksThresholdStreamAccumulatedForce(heapMemoryManager, PxsHeapStats::eSOLVER),
	mThresholdStreamWriteIndex(heapMemoryManager, PxsHeapStats::eSOLVER),
	mBlocksThresholdStreamWriteIndex(heapMemoryManager, PxsHeapStats::eSOLVER),
	mThresholdStreamWriteable(heapMemoryManager, PxsHeapStats::eSOLVER),
	mIslandIds(heapMemoryManager, PxsHeapStats::eSOLVER),
	mIslandStaticTouchCount(heapMemoryManager, PxsHeapStats::eSOLVER)
{
	mCudaContextManager->acquireContext();

	mCompressedContacts.allocate(init.maxRigidContactCount * sizeof(PxContact), PX_FL);
	mCompressedPatches.allocate(init.maxRigidPatchCount * sizeof(PxContactPatch), PX_FL);
	mForceBuffer.allocate(init.maxRigidContactCount * sizeof(PxReal) * 2, PX_FL);
	mFrictionPatches.allocate(init.maxRigidPatchCount * sizeof(PxFrictionPatch), PX_FL);

	mCudaContextManager->releaseContext();
}

PxgTGSCudaSolverCore::~PxgTGSCudaSolverCore()
{
}

void PxgTGSCudaSolverCore::createStreams()
{
	//CUstream mStream;
	CUresult result = mCudaContext->streamCreate(&mStream, CU_STREAM_NON_BLOCKING);

	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU Create Stream fail!!\n");

	result = mCudaContext->streamCreate(&mStream2, CU_STREAM_NON_BLOCKING);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU Create Stream fail!!\n");

	mCudaContext->eventCreate(&mEventDmaBack, CU_EVENT_DISABLE_TIMING);

	mCudaContext->eventCreate(&mIntegrateEvent, CU_EVENT_DISABLE_TIMING);

	mPinnedEvent = PX_PINNED_MEMORY_ALLOC(PxU32, *mCudaContextManager, 1);

	//pass mStream to PxgArticulationCore to synchronize data
	//mArticulationCore->setSolverStream(mStream);
	mGpuContext->getArticulationCore()->setSolverStream(mStream);
}

void PxgTGSCudaSolverCore::releaseStreams()
{
	mCudaContext->streamDestroy(mStream);
	mCudaContext->streamDestroy(mStream2);

	mCudaContext->eventDestroy(mEventDmaBack);
	mCudaContext->eventDestroy(mIntegrateEvent);

	PX_PINNED_MEMORY_FREE(*mCudaContextManager, mPinnedEvent);
}

void PxgTGSCudaSolverCore::syncSimulationController()
{
	PxgSimulationCore* core = mGpuContext->getSimulationCore();

	synchronizeStreams(mCudaContext, core->getStream(), mStream);
}

void PxgTGSCudaSolverCore::constructSolverSharedDesc(PxgSolverSharedDesc<IterativeSolveDataTGS>& sharedDesc, const PxgConstantData& cData,
	const PxU32 numIters, const PxReal lengthScale, Cm::UnAlignedSpatialVector* deferredZ, PxU32* articulationDirty,
	uint4* articulationSlabMask)
{
	IterativeSolveDataTGS& iterativeData = sharedDesc.iterativeData;

	iterativeData.solverBodyVelPool = reinterpret_cast<float4*>(mSolverBodyPool.getDevicePtr());
	iterativeData.tempStaticBodyOutputs = reinterpret_cast<float4*>(mTempStaticBodyOutputPool.getDevicePtr());

	iterativeData.solverEncodedReferenceCount = reinterpret_cast<PxU32*>(mSolverEncodedReferenceCount.getDevicePtr()); 

	iterativeData.blockConstraintBatch = reinterpret_cast<PxgBlockConstraintBatch*>(mBlockConstraintBatches.getDevicePtr());

	iterativeData.blockJointConstraintHeaders = reinterpret_cast<PxgTGSBlockSolverConstraint1DHeader*>(mJointHeaderBlockStream.getDevicePtr());
	iterativeData.blockJointConstraintRowsCon = reinterpret_cast<PxgTGSBlockSolverConstraint1DCon*>(mJointRowBlockStreamCon.getDevicePtr());
	iterativeData.blockJointConstraintRowsMod = reinterpret_cast<PxgBlockSolverConstraint1DMod*>(mJointRowBlockStreamMod.getDevicePtr());
	iterativeData.blockContactHeaders = reinterpret_cast<PxgTGSBlockSolverContactHeader*>(mContactHeaderBlockStream.getDevicePtr());
	iterativeData.blockFrictionHeaders = reinterpret_cast<PxgTGSBlockSolverFrictionHeader*>(mFrictionHeaderBlockStream.getDevicePtr());
	iterativeData.blockContactPoints = reinterpret_cast<PxgTGSBlockSolverContactPoint*>(mContactBlockStream.getDevicePtr());
	iterativeData.blockFrictions = reinterpret_cast<PxgTGSBlockSolverContactFriction*>(mFrictionBlockStream.getDevicePtr());

	iterativeData.contactHeaders = reinterpret_cast<PxgTGSSolverContactHeader*>(mContactHeaderStream.getDevicePtr());
	iterativeData.frictionHeaders = NULL;//reinterpret_cast<PxgSolverFrictionHeader*>(mFrictionHeaderStream.getDevicePtr());
	iterativeData.contactPoints = reinterpret_cast<PxgTGSSolverContactPointExt*>(mContactStream.getDevicePtr());
	iterativeData.frictions = reinterpret_cast<PxgTGSSolverFrictionExt*>(mFrictionStream.getDevicePtr());

	iterativeData.artiResponse = reinterpret_cast<PxgArticulationBlockResponse*>(mArtiConstraintBlockResponse.getDevicePtr());
		
	sharedDesc.stepDt = cData.dt / PxReal(numIters);
	sharedDesc.stepInvDtF32 = cData.invDtF32 * PxReal(numIters);
	sharedDesc.lengthScale = lengthScale;

	sharedDesc.invDtF32 = cData.invDtF32;

	constructSolverSharedDescCommon(sharedDesc, cData, deferredZ, articulationDirty, articulationSlabMask);
}

void PxgTGSCudaSolverCore::constructConstraitPrepareDesc(PxgConstraintPrepareDesc& prepareDesc, const PxU32 numDynamicConstraintBatchHeader,
	const PxU32 numStaticConstraintBatchHeaders, const PxU32 numDynamic1dConstraintBatches, const PxU32 numStatic1dConstraintBatches,
	const PxU32 numDynamicContactBatches, const PxU32 numStaticContactBatches,
	const PxU32 numArti1dConstraintBatches, const PxU32 numArtiContactBatches,
	const PxU32 numArtiStatic1dConstraintBatches, const PxU32 numArtiStaticContactBatches, 
	const PxU32 numArtiSelf1dConstraintBatches, const PxU32 numArtiSelfContactBatches, const PxgConstantData& cData,
	PxU32 totalCurrentEdges, PxU32 totalPreviousEdges, PxU32 totalBodies)
{
	prepareDesc.contactConstraintBatchIndices = reinterpret_cast<PxU32*>(mContactBatchIndices.getDevicePtr());
	prepareDesc.jointConstraintBatchIndices = reinterpret_cast<PxU32*>(mConstraint1DBatchIndices.getDevicePtr());
	prepareDesc.artiContactConstraintBatchIndices = reinterpret_cast<PxU32*>(mArtiContactBatchIndices.getDevicePtr());
	prepareDesc.artiJointConstraintBatchIndices = reinterpret_cast<PxU32*>(mArtiConstraint1dBatchIndices.getDevicePtr());

	prepareDesc.blockContactCurrentPrepPool = reinterpret_cast<PxgBlockContactData*>(mConstraintContactPrepBlockPool.getDevicePtr());

	prepareDesc.blockCurrentAnchorPatches = reinterpret_cast<PxgBlockFrictionAnchorPatch*>(mFrictionAnchorPatchBlockStream[mCurrentIndex].getDevicePtr());
	prepareDesc.blockPreviousAnchorPatches = reinterpret_cast<PxgBlockFrictionAnchorPatch*>(mFrictionAnchorPatchBlockStream[1 - mCurrentIndex].getDevicePtr());

	prepareDesc.blockCurrentFrictionIndices = reinterpret_cast<PxgBlockFrictionIndex*>(mFrictionIndexStream[mCurrentIndex].getDevicePtr());
	prepareDesc.blockPreviousFrictionIndices = reinterpret_cast<PxgBlockFrictionIndex*>(mFrictionIndexStream[1 - mCurrentIndex].getDevicePtr());

	prepareDesc.solverConstantData = reinterpret_cast<PxgSolverConstraintManagerConstants*>(mSolverConstantData.getDevicePtr());
	prepareDesc.blockJointPrepPool = reinterpret_cast<PxgBlockConstraint1DData*>(mConstraint1DPrepBlockPool.getDevicePtr());
	prepareDesc.blockJointPrepPool0 = reinterpret_cast<PxgBlockConstraint1DVelocities*>(mConstraint1DPrepBlockPoolVel.getDevicePtr());
	prepareDesc.blockJointPrepPool1 = reinterpret_cast<PxgBlockConstraint1DParameters*>(mConstraint1DPrepBlockPoolPar.getDevicePtr());
	prepareDesc.solverBodyDataPool = reinterpret_cast<PxgSolverBodyData*>(mSolverBodyDataPool.getDevicePtr());
	prepareDesc.solverBodyTxIDataPool = reinterpret_cast<PxgSolverTxIData*>(mSolverTxIDataPool.getDevicePtr());

	prepareDesc.blockWorkUnit = reinterpret_cast<PxgBlockWorkUnit*>(mBlockWorkUnits.getDevicePtr());
	prepareDesc.blockContactPoints = reinterpret_cast<PxgBlockContactPoint*>(mGpuContactBlockBuffer.getDevicePtr());
	/*prepareDesc.jointPrepPool = reinterpret_cast<PxgConstraint1DData*>(mConstraint1DPrepPool.getDevicePtr());
	prepareDesc.jointPrepPool0 = reinterpret_cast<PxgConstraint1DVelocities*>(mConstraint1DPrepPoolVel.getDevicePtr());
	prepareDesc.jointPrepPool1 = reinterpret_cast<PxgConstraint1DParameters*>(mConstraint1DPrepPoolPar.getDevicePtr());*/

	prepareDesc.contactManagerOutputBase = reinterpret_cast<PxsContactManagerOutput*>(mGpuContactManagerOutputBase);

	prepareDesc.constraintUniqueIndices = reinterpret_cast<PxU32*>(mConstraintUniqueIndices);
	prepareDesc.artiConstraintUniqueIndices = reinterpret_cast<PxU32*>(mArtiConstraintUniqueIndices);
	prepareDesc.artiContactUniqueIndices = reinterpret_cast<PxU32*>(mArtiContactUniqueIndices);

	prepareDesc.currentAnchorPatches = reinterpret_cast<PxgFrictionAnchorPatch*>(mFrictionAnchorPatchStream[mCurrentIndex].getDevicePtr());
	prepareDesc.previousAnchorPatches = reinterpret_cast<PxgFrictionAnchorPatch*>(mFrictionAnchorPatchStream[1 - mCurrentIndex].getDevicePtr());

	prepareDesc.body2WorldPool = reinterpret_cast<PxAlignedTransform*>(mOutBody2WorldPool.getDevicePtr());

	prepareDesc.numBatches = numDynamicConstraintBatchHeader;
	prepareDesc.numStaticBatches = numStaticConstraintBatchHeaders;
	prepareDesc.num1dConstraintBatches = numDynamic1dConstraintBatches;
	prepareDesc.numContactBatches = numDynamicContactBatches;
	prepareDesc.numStatic1dConstraintBatches = numStatic1dConstraintBatches;
	prepareDesc.numStaticContactBatches = numStaticContactBatches;
	prepareDesc.numArtiStatic1dConstraintBatches = numArtiStatic1dConstraintBatches;
	prepareDesc.numArtiStaticContactBatches = numArtiStaticContactBatches;
	prepareDesc.numArtiSelf1dConstraintBatches = numArtiSelf1dConstraintBatches;
	prepareDesc.numArtiSelfContactBatches = numArtiSelfContactBatches;
	prepareDesc.numArtiContactBatches = numArtiContactBatches;
	prepareDesc.numArti1dConstraintBatches = numArti1dConstraintBatches;

	prepareDesc.bounceThresholdF32 = cData.bounceThresholdF32;
	prepareDesc.frictionOffsetThreshold = cData.frictionOffsetThreshold;
	prepareDesc.correlationDistance = cData.correlationDistance;
	prepareDesc.ccdMaxSeparation = cData.ccdMaxSeparation;
	prepareDesc.biasCoefficient = cData.biasCoefficient;
	prepareDesc.totalPreviousEdges = totalPreviousEdges;
	prepareDesc.totalCurrentEdges = totalCurrentEdges;

	prepareDesc.articContactIndex = 0;
	prepareDesc.articJointIndex = 0;
	prepareDesc.totalBodyCount = totalBodies;
	prepareDesc.nbElementsPerBody = 3;
}

// PT: TODO: refactor with PxgCudaSolverCore::constructSolverDesc
void PxgTGSCudaSolverCore::constructSolverDesc(PxgSolverCoreDesc& scDesc, PxU32 numIslands, PxU32 numSolverBodies, PxU32 numConstraintBatchHeader,
	PxU32 numArticConstraints, PxU32 numSlabs, bool enableStabilization)
{
	// PT: TODO: move all these remaining class members to base class?

	scDesc.thresholdStreamAccumulatedForce = reinterpret_cast<PxReal*>(mThresholdStreamAccumulatedForce.getDevicePtr());
	scDesc.thresholdStreamAccumulatedForceBetweenBlocks = reinterpret_cast<PxReal*>(mBlocksThresholdStreamAccumulatedForce.getDevicePtr());

	scDesc.thresholdStreamWriteIndex = reinterpret_cast<PxU32*>(mThresholdStreamWriteIndex.getDevicePtr());
	scDesc.thresholdStreamWriteIndexBetweenBlocks = reinterpret_cast<PxU32*>(mBlocksThresholdStreamWriteIndex.getDevicePtr());
	scDesc.thresholdStreamWriteable = reinterpret_cast<bool*>(mThresholdStreamWriteable.getDevicePtr());

	scDesc.thresholdStream = reinterpret_cast<Dy::ThresholdStreamElement*>(mForceThresholdStream.getDevicePtr());
	scDesc.tmpThresholdStream = reinterpret_cast<Dy::ThresholdStreamElement*>(mTmpForceThresholdStream.getDevicePtr());

	scDesc.accumulatedForceObjectPairs = reinterpret_cast<PxReal*>(mAccumulatedForceObjectPairs.getDevicePtr());

	scDesc.exceededForceElements = reinterpret_cast<Dy::ThresholdStreamElement*>(mExceededForceElements[mCurrentIndex].getDevicePtr());
	scDesc.prevExceededForceElements = reinterpret_cast<Dy::ThresholdStreamElement*>(mExceededForceElements[1 - mCurrentIndex].getDevicePtr());

	scDesc.forceChangeThresholdElements = reinterpret_cast<Dy::ThresholdStreamElement*>(mForceChangeThresholdElements.getDevicePtr());

	PxgSolverCore::constructSolverDesc(scDesc, numIslands, numSolverBodies, numConstraintBatchHeader, numArticConstraints, numSlabs, enableStabilization);
}

void PxgTGSCudaSolverCore::gpuMemDMAUpContactData(PxgPinnedHostLinearMemoryAllocator* compressedContactsHostMemoryAllocator,
	PxU32 compressedContactStreamUpperPartSize,
	PxU32 compressedContactStreamLowerPartSize,
	PxgPinnedHostLinearMemoryAllocator* compressedPatchesHostMemoryAllocator,
	PxU32 compressedPatchStreamUpperPartSize,
	PxU32 compressedPatchStreamLowerPartSize,
	PxU32 totalContactManagers,
	const PartitionIndexData* partitionIndexData,
	const PartitionNodeData* partitionNodeData,
	const PxgSolverConstraintManagerConstants* constantData,
	PxU32 constantDataCount,
	PxU32 partitionIndexDataCount,
	const PxU32* partitionConstraintBatchStartIndices,
	const PxU32* partitionArticConstraintBatchStartIndices,
	const PxU32* partitionJointBatchCounts,
	const PxU32* partitionArtiJointBatchCounts,
	PxU32 nbPartitions, const PxU32* destroyedEdges, PxU32 nbDestroyedEdges,
	const PxU32* npIndexArray, PxU32 npIndexArraySize,
	PxU32 totalNumJoints,
	const PxU32* islandIds, const PxU32* nodeInteractionCounts, PxU32 nbNodes, const PxU32* islandStaticTouchCount, PxU32 nbIslands)
{
	PX_PROFILE_ZONE("PxgTGSCudaSolverCore.gpuMemDMAUpContactData", 0);
	PX_UNUSED(compressedPatchStreamLowerPartSize);
	PX_UNUSED(compressedContactStreamLowerPartSize);
	CUdeviceptr compressedContactsd = mCompressedContacts.getDevicePtr();//.allocate((PxU32) compressedContactsHostMemoryAllocator->mTotalSize);
	CUdeviceptr compressedPatchesd = mCompressedPatches.getDevicePtr();//allocate((PxU32) compressedPatchesHostMemoryAllocator->mTotalSize);

	mDestroyedEdgeIndices.allocate(sizeof(PxU32) * nbDestroyedEdges, PX_FL);

	//allocate device memory for constraint write back buffer, including active and inactive
	mConstraintWriteBackBuffer.allocate(sizeof(PxgConstraintWriteback) * totalNumJoints, PX_FL);

	mForceThresholdStream.allocate(sizeof(Dy::ThresholdStreamElement) * totalContactManagers, PX_FL);
	mTmpForceThresholdStream.allocate(sizeof(Dy::ThresholdStreamElement) * totalContactManagers, PX_FL);

	mPartitionIndexData.allocate(sizeof(PartitionIndexData) * partitionIndexDataCount, PX_FL);
	mPartitionNodeData.allocate(sizeof(PartitionNodeData) * partitionIndexDataCount, PX_FL);
	mSolverConstantData.allocate(sizeof(PxgSolverConstraintManagerConstants) * constantDataCount, PX_FL);
	mPartitionStartBatchIndices.allocate(sizeof(PxU32) * nbPartitions, PX_FL);
	mPartitionArticulationStartBatchIndices.allocate(sizeof(PxU32) * nbPartitions, PX_FL);
	mPartitionJointBatchCounts.allocate(sizeof(PxU32) * nbPartitions, PX_FL);
	mPartitionArtiJointBatchCounts.allocate(sizeof(PxU32) * nbPartitions, PX_FL);

	mNpIndexArray.allocate(sizeof(PxU32) * npIndexArraySize, PX_FL);

	mIslandIds.allocate(nbNodes * sizeof(PxU32), PX_FL);
	mIslandStaticTouchCount.allocate(nbIslands * sizeof(PxU32), PX_FL);
	allocateNodeInteractionCounts(nbNodes);

	mTotalContactManagers = totalContactManagers;

	PX_ASSERT((size_t)compressedContactsHostMemoryAllocator->mTotalSize >= compressedContactStreamUpperPartSize + compressedContactStreamLowerPartSize);
	PX_ASSERT((size_t)compressedPatchesHostMemoryAllocator->mTotalSize >= compressedPatchStreamUpperPartSize + compressedPatchStreamLowerPartSize);

	mCudaContext->memcpyHtoDAsync(compressedContactsd + compressedContactsHostMemoryAllocator->mTotalSize
		- (size_t)compressedContactStreamUpperPartSize,
		compressedContactsHostMemoryAllocator->mStart
		+ compressedContactsHostMemoryAllocator->mTotalSize
		- (size_t)compressedContactStreamUpperPartSize,
		(size_t)compressedContactStreamUpperPartSize, mStream);
	mCudaContext->memcpyHtoDAsync(compressedPatchesd + compressedPatchesHostMemoryAllocator->mTotalSize
		- (size_t)compressedPatchStreamUpperPartSize,
		compressedPatchesHostMemoryAllocator->mStart
		+ compressedPatchesHostMemoryAllocator->mTotalSize
		- (size_t)compressedPatchStreamUpperPartSize,
		(size_t)compressedPatchStreamUpperPartSize, mStream);

	mCudaContext->memcpyHtoDAsync(mPartitionIndexData.getDevicePtr(), partitionIndexData, sizeof(PartitionIndexData) * partitionIndexDataCount, mStream);
	mCudaContext->memcpyHtoDAsync(mPartitionNodeData.getDevicePtr(), partitionNodeData, sizeof(PartitionNodeData) * partitionIndexDataCount, mStream);
	mCudaContext->memcpyHtoDAsync(mSolverConstantData.getDevicePtr(), constantData, sizeof(PxgSolverConstraintManagerConstants) * constantDataCount, mStream);
	mCudaContext->memcpyHtoDAsync(mPartitionStartBatchIndices.getDevicePtr(), partitionConstraintBatchStartIndices, sizeof(PxU32) * nbPartitions, mStream);
	mCudaContext->memcpyHtoDAsync(mPartitionArticulationStartBatchIndices.getDevicePtr(), partitionArticConstraintBatchStartIndices, sizeof(PxU32) * nbPartitions, mStream);
	mCudaContext->memcpyHtoDAsync(mPartitionJointBatchCounts.getDevicePtr(), partitionJointBatchCounts, sizeof(PxU32) * nbPartitions, mStream);
	mCudaContext->memcpyHtoDAsync(mPartitionArtiJointBatchCounts.getDevicePtr(), partitionArtiJointBatchCounts, sizeof(PxU32) * nbPartitions, mStream);
	mCudaContext->memcpyHtoDAsync(mNpIndexArray.getDevicePtr(), npIndexArray, npIndexArraySize * sizeof(PxU32), mStream);
	mCudaContext->memcpyHtoDAsync(mIslandIds.getDevicePtr(), islandIds, nbNodes * sizeof(PxU32), mStream);
	mCudaContext->memcpyHtoDAsync(mIslandStaticTouchCount.getDevicePtr(), islandStaticTouchCount, sizeof(PxU32) * nbIslands, mStream);
	uploadNodeInteractionCounts(nodeInteractionCounts, nbNodes);

	mCudaContext->memcpyHtoDAsync(mDestroyedEdgeIndices.getDevicePtr(), destroyedEdges, nbDestroyedEdges * sizeof(PxU32), mStream);

	const PxU32 nbBlocksRequired = (nbDestroyedEdges + PxgKernelBlockDim::CLEAR_FRICTION_PATCH_COUNTS - 1) / PxgKernelBlockDim::CLEAR_FRICTION_PATCH_COUNTS;

	if (nbBlocksRequired > 0)
	{
		//Launch zero friction patch kernel
		const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLEAR_FRICTION_PATCH_COUNTS);

		CUdeviceptr frictionPatchPtr = mFrictionPatchCounts[1 - mCurrentIndex].getDevicePtr();
		CUdeviceptr destroyedIndicesPtr = mDestroyedEdgeIndices.getDevicePtr();
		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(frictionPatchPtr),
			PX_CUDA_KERNEL_PARAM(destroyedIndicesPtr),
			PX_CUDA_KERNEL_PARAM(nbDestroyedEdges)
		};

		CUresult result = mCudaContext->launchKernel(kernelFunction, nbBlocksRequired, 1, 1, PxgKernelBlockDim::CLEAR_FRICTION_PATCH_COUNTS, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);
	}

	//mCudaContext->streamFlush(mStream);	// PT: TODO: why is it NOT commented out in PxgCudaSolverCore::gpuMemDMAUpContactData?
}

//solverBodyIndices and islandNodeIndices will include rigid bodies and articulations
void PxgTGSCudaSolverCore::gpuMemDmaUpBodyData(PxPinnedArray<PxgSolverBodyData>& solverBodyDataPool,
	PxPinnedArray<PxgSolverTxIData>& solverTxIDataPool,
	const PxU32 numSolverBodies,
	const PxU32 totalNumRigidBatches, const PxU32 totalNumArticBatches,
	const PxU32 nbSlabs, const PxU32 nbStaticSlabs, const PxU32 maxNumStaticPartitions)
{
	PX_PROFILE_ZONE("GpuDynamics.gpuMemDmaUpBodyData", 0);
	
	const PxU32 nbStaticKinematic = solverBodyDataPool.size();

	mCudaContext->memcpyHtoDAsync(mSolverBodyDataPool.getDevicePtr(), solverBodyDataPool.begin(), sizeof(PxgSolverBodyData) * nbStaticKinematic, mStream);
		
	mCudaContext->memcpyHtoDAsync(mSolverTxIDataPool.getDevicePtr(), solverTxIDataPool.begin(), sizeof(PxgSolverTxIData) * nbStaticKinematic, mStream);

	//Allocate space for 3 float4s per-body referenced by a batch. That's 3 * 2 * 32 * sizeof(float4). 
	const PxU32 numRigidBodyMirrorBodies = totalNumRigidBatches * 3 * 2 * 32;
	//Also allocate space for a single rigid body referenced by each articulation constraint. That's 3* sizeof(float4)
	const PxU32 numArticConstraintMirrorBodies = totalNumArticBatches * 3 * 2 * 32;
	//In addition, allocate space for velocity averaging results. This is 3 * nbSlabs * numSolverBodies *sizeof(float4).
	//We need to round the number of bodies up to a multiple of 32 to optimize memory accesses
	const PxU32 numAccumulationBodies = (((nbSlabs * numSolverBodies + 31)&(~31)) * 3);

	const PxU32 numOutputBodies = ((numSolverBodies + 31)&(~31)) * 3;

	mSolverBodyOutputVelocityOffset = numRigidBodyMirrorBodies + numArticConstraintMirrorBodies + numAccumulationBodies;

	const PxU32 totalBodiesSize = (numRigidBodyMirrorBodies + numArticConstraintMirrorBodies + numAccumulationBodies + numOutputBodies);
	//Allocate solver body pool
	mSolverBodyPool.allocate(sizeof(float4) * totalBodiesSize, PX_FL);

	const PxU32 numEncodedReferenceCount = numSolverBodies * ((PxMax(1u, nbSlabs) + 31) / 32);
	mSolverEncodedReferenceCount.allocate(sizeof(PxU32) * numEncodedReferenceCount, PX_FL);
	mCudaContext->memsetD32Async(mSolverEncodedReferenceCount.getDevicePtr(), 0, numEncodedReferenceCount * (sizeof(PxU32) / sizeof(PxU32)),
								 mStream);

	mTempStaticBodyOutputPool.allocate(sizeof(float4)*numSolverBodies*nbStaticSlabs * 2, PX_FL);

	mNbStaticRigidSlabs = nbStaticSlabs;

	mMaxNumStaticPartitions = maxNumStaticPartitions;

	//mCudaContext->streamFlush(mStream);

#if GPU_DEBUG
	CUresult result = mCudaContext->streamSynchronize(mStream);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU DMA up fail!!\n");
#endif
}

void PxgTGSCudaSolverCore::allocateSolverBodyBuffers(const PxU32 numSolverBodies,
	PxPinnedArray<PxNodeIndex>& islandNodeIndices,
	const PxU32 numActiveActiculations, const PxU32 maxArticulationLinks)
{
	allocateSolverBodyBuffersCommon(numSolverBodies, islandNodeIndices);
	mOutArtiVelocityPool.allocate(sizeof(float4)*numActiveActiculations*maxArticulationLinks * 4, PX_FL);
}

PxU32 PxgTGSCudaSolverCore::getDescriptorsAllocationSize()
{
	PxU32 size = 0;
	const PxU32 alignment = 16;

	const PxU32 sharedDescSize = sizeof(PxgSolverSharedDesc<IterativeSolveDataTGS>) + alignment;
	const PxU32 solverCoreDescSize = sizeof(PxgSolverCoreDesc) + alignment;
	const PxU32 prepareDescSize = sizeof(PxgConstraintPrepareDesc) + alignment;
	const PxU32 prePrepDescSize = sizeof(PxgPrePrepDesc) + alignment;
	const PxU32 rsDescSize = (sizeof(PxgRadixSortDesc)*2) + alignment;

	size = sharedDescSize + solverCoreDescSize + prepareDescSize + prePrepDescSize + rsDescSize;

	return size;
}

void PxgTGSCudaSolverCore::allocatePinnedDescriptors(PxgPinnedHostLinearMemoryAllocator& hostAllocator)
{
	mSharedDesc = reinterpret_cast<PxgSolverSharedDesc<IterativeSolveDataTGS>*>(hostAllocator.allocate(sizeof(PxgSolverSharedDesc<IterativeSolveDataTGS>), 16));
	mSolverCoreDesc = reinterpret_cast<PxgSolverCoreDesc*>(hostAllocator.allocate(sizeof(PxgSolverCoreDesc), 16));
	mPrepareDesc = reinterpret_cast<PxgConstraintPrepareDesc*>(hostAllocator.allocate(sizeof(PxgConstraintPrepareDesc), 16));
	mPrePrepDesc = reinterpret_cast<PxgPrePrepDesc*>(hostAllocator.allocate(sizeof(PxgPrePrepDesc), 16));
	mRsDesc = reinterpret_cast<PxgRadixSortDesc*>(hostAllocator.allocate(sizeof(PxgRadixSortDesc)*2, 16));
}

void PxgTGSCudaSolverCore::gpuMemDMAUp(PxgPinnedHostLinearMemoryAllocator& hostAllocator,
	const PxgConstraintPrePrepData& data,
	const PxU32 numSolverBodies, PxgConstraintBatchHeader* constraintBatchHeaders,
	PxgIslandContext* islandContextPool, const PxU32 numIslands, const PxgPartitionData& pData,
	const PxU32 numConstraintBatchHeader, const PxU32 numStaticConstraintBatchHeader,
	const PxU32 numArticConstraintBatchHeader, 
	const PxU32 numArtiStaticConstraintBatchHeader, const PxU32 numArtiSelfConstraintBatchHeader, const PxgConstantData& cData,
	const PxU32 numContactBlockes, const PxU32 numFrictionBlockes,
	const PxU32 numArtiContactBlocks, const PxU32 numArtiFrictionBlocks,
	const PxU32 totalCurrentEdges, const PxU32 totalPreviousEdges, const PxU32 numSlabs, const PxU32 maxNbPartitions,
	const bool enableStabilization, PxU8* cpuContactPatchStreamBase, PxU8* cpuContactStreamBase, PxU8* cpuForceStreamBase, PxsContactManagerOutputIterator& outputIterator,
	const PxU32 totalActiveBodyCount, const PxU32 activeBodyStartIndex, const PxU32 nbArticulations, Cm::UnAlignedSpatialVector* deferredZ,
	PxU32* articulationDirty, uint4* articulationSlabMask, Sc::ShapeInteraction** shapeInteractions, PxReal* restDistances,
	PxsTorsionalFrictionData* torsionalData,
	PxU32* artiStaticContactIndices, const PxU32 artiStaticContactIndSize, PxU32* artiStaticJointIndices, PxU32 artiStaticJointSize,
	PxU32* artiStaticContactCounts, PxU32* artiStaticJointCounts,
	PxU32* artiSelfContactIndices, const PxU32 artiSelfContactIndSize, PxU32* artiSelfJointIndices, PxU32 artiSelfJointSize,
	PxU32* artiSelfContactCounts, PxU32* artiSelfJointCounts,
	PxU32* rigidStaticContactIndices, const PxU32 rigidStaticContactIndSize, PxU32* rigidStaticJointIndices, const PxU32 rigidStaticJointSize,
	PxU32* rigidStaticContactCounts, PxU32* rigidStaticJointCounts, const PxReal lengthScale, bool hasForceThresholds)
{
	PX_PROFILE_ZONE("GpuDynamics.DMAUp", 0);

	const PxU32 totalContactBlocks = numContactBlockes + numArtiContactBlocks;
	const PxU32 totalFrictionBlocks = numFrictionBlockes + numArtiFrictionBlocks;

	mDataBuffer.allocate((PxU32)hostAllocator.mCurrentSize, PX_FL);
	CUdeviceptr dataBufferd = mDataBuffer.getDevicePtr();
	mContactUniqueIndices = dataBufferd + (reinterpret_cast<PxU8*>(data.contactUniqueIndices) - hostAllocator.mStart);
	mConstraintUniqueIndices = dataBufferd + (reinterpret_cast<PxU8*>(data.constraintUniqueIndices) - hostAllocator.mStart);
	mArtiConstraintUniqueIndices = dataBufferd + (reinterpret_cast<PxU8*>(data.artiConstraintUniqueindices) - hostAllocator.mStart);
	mArtiContactUniqueIndices = dataBufferd + (reinterpret_cast<PxU8*>(data.artiContactUniqueIndices) - hostAllocator.mStart);
	mConstraintBatchHeaders = dataBufferd + (reinterpret_cast<PxU8*>(constraintBatchHeaders) - hostAllocator.mStart);

	mArtiStaticConstraintUniqueIndices = dataBufferd + (reinterpret_cast<PxU8*>(data.artiStaticConstraintUniqueIndices) - hostAllocator.mStart);
	mArtiStaticContactUniqueIndices = dataBufferd + (reinterpret_cast<PxU8*>(data.artiStaticContactUniqueIndices) - hostAllocator.mStart);

	mArtiStaticConstraintStartIndex = dataBufferd + (reinterpret_cast<PxU8*>(data.artiStaticConstraintStartIndex) - hostAllocator.mStart);
	mArtiStaticConstraintCount = dataBufferd + (reinterpret_cast<PxU8*>(data.artiStaticConstraintCount) - hostAllocator.mStart);
	mArtiStaticContactStartIndex = dataBufferd + (reinterpret_cast<PxU8*>(data.artiStaticContactStartIndex) - hostAllocator.mStart);
	mArtiStaticContactCount = dataBufferd + (reinterpret_cast<PxU8*>(data.artiStaticContactCount) - hostAllocator.mStart);

	const PxU32 numDynamicContactBatches = data.numContactBatches;
	const PxU32 numTotalRigidContactBatches = numDynamicContactBatches + data.numStaticContactBatches;
	const PxU32 numDynamic1DConstraintBatches = data.num1dConstraintBatches;
	const PxU32 numTotalRigid1DConstraintBatches = numDynamic1DConstraintBatches + data.numStatic1dConstraintBatches;
	const PxU32 numArtiContactBatches = data.numArtiContactsBatches + data.numArtiStaticContactsBatches + data.numArtiSelfContactsBatches;
	const PxU32 numArti1dConstraintBatches = data.numArti1dConstraintBatches + data.numArtiStatic1dConstraintBatches + data.numArtiSelf1dConstraintBatches;
		
	const PxU32 totalNum1DConstraintBatches = numTotalRigid1DConstraintBatches + numArti1dConstraintBatches;

	const PxU32 totalContactBatches = numTotalRigidContactBatches + numArtiContactBatches;

	mArtiOrderedStaticContacts.allocate(sizeof(PxU32)*data.numArtiStaticContactsBatches, PX_FL);
	mArtiOrderedStaticConstraints.allocate(sizeof(PxU32)*data.numArtiStatic1dConstraintBatches, PX_FL);

	mBlockConstraintBatches.allocate(sizeof(PxgBlockConstraintBatch) * (totalContactBatches + totalNum1DConstraintBatches), PX_FL);

	mIslandContextPool = dataBufferd + (reinterpret_cast<PxU8*>(islandContextPool) - hostAllocator.mStart);
	mSolverCoreDescd = dataBufferd + (reinterpret_cast<PxU8*>(mSolverCoreDesc) - hostAllocator.mStart);
	mSharedDescd = dataBufferd + (reinterpret_cast<PxU8*>(mSharedDesc) - hostAllocator.mStart);
	mPrepareDescd = dataBufferd + (reinterpret_cast<PxU8*>(mPrepareDesc) - hostAllocator.mStart);
	mPrePrepDescd = dataBufferd + (reinterpret_cast<PxU8*>(mPrePrepDesc) - hostAllocator.mStart);
	mRadixSortDescd[0] = dataBufferd + (reinterpret_cast<PxU8*>(&mRsDesc[0]) - hostAllocator.mStart);
	mRadixSortDescd[1] = dataBufferd + (reinterpret_cast<PxU8*>(&mRsDesc[1]) - hostAllocator.mStart);

	mConstraint1DBatchIndices.allocate(numTotalRigid1DConstraintBatches*sizeof(PxU32), PX_FL);
	mContactBatchIndices.allocate(numTotalRigidContactBatches*sizeof(PxU32), PX_FL);
	mArtiContactBatchIndices.allocate(numArtiContactBatches * sizeof(PxU32), PX_FL);
	mArtiConstraint1dBatchIndices.allocate(numArti1dConstraintBatches * sizeof(PxU32), PX_FL);

	mSolverBodyReferences.allocate(totalActiveBodyCount * numSlabs * sizeof(PxgSolverReferences), PX_FL);
	//CUdeviceptr constraint1DBatchIndicesd = mConstraint1DBatchIndices.getDevicePtr();
	//CUdeviceptr contactBatchIndicesd = mContactBatchIndices.getDevicePtr();
	CUdeviceptr solverBodyReferencesd = mSolverBodyReferences.getDevicePtr();

	mBlockWorkUnits.allocate(sizeof(PxgBlockWorkUnit) * totalContactBatches, PX_FL);

	mConstraintsPerPartition.allocate(sizeof(PxU32) * pData.numConstraintsPerPartition, PX_FL);
	mArtiConstraintsPerPartition.allocate(sizeof(PxU32) * pData.numArtiConstraintsPerPartition, PX_FL);

	mConstraintContactPrepBlockPool.allocate(sizeof(PxgBlockContactData) * totalContactBatches, PX_FL);
	mConstraint1DPrepBlockPool.allocate(sizeof(PxgBlockConstraint1DData) * totalNum1DConstraintBatches, PX_FL);
	mConstraint1DPrepBlockPoolVel.allocate(Dy::MAX_CONSTRAINT_ROWS* totalNum1DConstraintBatches * sizeof(PxgBlockConstraint1DVelocities), PX_FL);
	mConstraint1DPrepBlockPoolPar.allocate(Dy::MAX_CONSTRAINT_ROWS* totalNum1DConstraintBatches * sizeof(PxgBlockConstraint1DParameters), PX_FL);

	//allocate enough for cpu and gpu joints
	mConstraintDataPool.allocate(data.nbTotalRigidJoints * sizeof(PxgConstraintData), PX_FL);
	mConstraintRowPool.allocate(data.nbTotalRigidJoints* sizeof(Px1DConstraint) * Dy::MAX_CONSTRAINT_ROWS, PX_FL);

	mArtiConstraintDataPool.allocate(data.nbTotalArtiJoints * sizeof(PxgConstraintData), PX_FL);
	mArtiConstraintRowPool.allocate(data.nbTotalArtiJoints * sizeof(Px1DConstraint) * Dy::MAX_CONSTRAINT_ROWS, PX_FL);

	mJointHeaderBlockStream.allocate(totalNum1DConstraintBatches * sizeof(PxgTGSBlockSolverConstraint1DHeader), PX_FL);

	mJointRowBlockStreamCon.allocate(Dy::MAX_CONSTRAINT_ROWS* totalNum1DConstraintBatches * sizeof(PxgTGSBlockSolverConstraint1DCon), PX_FL);
	//mJointRowBlockStreamMod.allocate(Dy::MAX_CONSTRAINT_ROWS* totalNum1DConstraintBatches * sizeof(PxgBlockSolverConstraint1DMod));

	mContactHeaderBlockStream.allocate(totalContactBatches * sizeof(PxgTGSBlockSolverContactHeader), PX_FL);
	mFrictionHeaderBlockStream.allocate(totalContactBatches * sizeof(PxgTGSBlockSolverFrictionHeader), PX_FL);
	mContactBlockStream.allocate(totalContactBlocks * sizeof(PxgTGSBlockSolverContactPoint), PX_FL);
	mFrictionBlockStream.allocate(totalFrictionBlocks * sizeof(PxgTGSBlockSolverContactFriction), PX_FL);

	mGpuContactBlockBuffer.allocate(totalContactBlocks * sizeof(PxgBlockContactPoint), PX_FL);

	mConstraintContactPrepPool.allocate(numArtiContactBatches * sizeof(PxgContactData), PX_FL);
	mContactHeaderStream.allocate(numArtiContactBatches * sizeof(PxgTGSSolverContactHeader), PX_FL);
	mContactStream.allocate(numArtiContactBlocks * sizeof(PxgTGSSolverContactPointExt), PX_FL);
	mFrictionStream.allocate(numArtiFrictionBlocks * sizeof(PxgTGSSolverFrictionExt), PX_FL);

	//KS - we should not need to allocate block response vectors for non-arti constraints!
	mArtiConstraintBlockResponse.allocate((numArtiContactBlocks + numArtiFrictionBlocks + Dy::MAX_CONSTRAINT_ROWS*(numArti1dConstraintBatches)) * sizeof(PxgArticulationBlockResponse), PX_FL);


	// AD: We already skip the computation if we have no pair requesting force threshold reports, so we also
	// don't allocate the memory.
	if (hasForceThresholds)
	{
		mThresholdStreamAccumulatedForce.allocate(sizeof(PxReal) * totalContactBatches * 32, PX_FL);
		mBlocksThresholdStreamAccumulatedForce.allocate(PxgKernelGridDim::COMPUTE_ACCUMULATED_THRESHOLDSTREAM * sizeof(PxReal), PX_FL);
		mAccumulatedForceObjectPairs.allocate(sizeof(PxReal) * totalContactBatches * 32, PX_FL);
		mExceededForceElements[mCurrentIndex].allocate(sizeof(Dy::ThresholdStreamElement) * totalContactBatches * 32, PX_FL);

		//make sure we have enough space for the both previous exceeded force pairs and the current exceeded force pairs, persistent force pairs
		mThresholdStreamWriteIndex.allocate(sizeof(PxU32) * (totalContactBatches * 32 + mNbPrevExceededForceElements * 2), PX_FL);
		mBlocksThresholdStreamWriteIndex.allocate(PxgKernelGridDim::COMPUTE_ACCUMULATED_THRESHOLDSTREAM * sizeof(PxU32), PX_FL);
		mThresholdStreamWriteable.allocate(sizeof(bool) * (totalContactBatches * 32 + mNbPrevExceededForceElements * 2), PX_FL);
		mForceChangeThresholdElements.allocate(sizeof(Dy::ThresholdStreamElement) * (totalContactBatches * 32 + mNbPrevExceededForceElements * 2), PX_FL);

		mRadixSort.allocate(totalContactBatches);
	}

	mRigidStaticContactIndices.allocate(rigidStaticContactIndSize * sizeof(PxU32), PX_FL);
	mRigidStaticJointIndices.allocate(rigidStaticJointSize * sizeof(PxU32), PX_FL);
	mRigidStaticContactCounts.allocate(islandContextPool->mBodyCount * sizeof(PxU32), PX_FL);
	mRigidStaticJointCounts.allocate(islandContextPool->mBodyCount * sizeof(PxU32), PX_FL);
	mRigidStaticContactStartIndices.allocate(islandContextPool->mBodyCount * sizeof(PxU32), PX_FL);
	mRigidStaticJointStartIndices.allocate(islandContextPool->mBodyCount * sizeof(PxU32), PX_FL);

	const PxU32 numBlocks = 32;

	mTempContactUniqueIndicesBlockBuffer.allocate(numBlocks * sizeof(PxU32), PX_FL);
	mTempConstraintUniqueIndicesBlockBuffer.allocate(numBlocks * sizeof(PxU32), PX_FL);
	mTempContactHeaderBlockBuffer.allocate(numBlocks * sizeof(PxU32), PX_FL);
	mTempConstraintHeaderBlockBuffer.allocate(numBlocks * sizeof(PxU32), PX_FL);

	mCudaContext->memcpyHtoDAsync(mRigidStaticContactIndices.getDevicePtr(), rigidStaticContactIndices, sizeof(PxU32)*rigidStaticContactIndSize, mStream);
	mCudaContext->memcpyHtoDAsync(mRigidStaticJointIndices.getDevicePtr(), rigidStaticJointIndices, sizeof(PxU32)*rigidStaticJointSize, mStream);
	mCudaContext->memcpyHtoDAsync(mRigidStaticContactCounts.getDevicePtr(), rigidStaticContactCounts, sizeof(PxU32)*islandContextPool->mBodyCount, mStream);
	mCudaContext->memcpyHtoDAsync(mRigidStaticJointCounts.getDevicePtr(), rigidStaticJointCounts, sizeof(PxU32)*islandContextPool->mBodyCount, mStream);

	mArtiStaticContactIndices.allocate(artiStaticContactIndSize * sizeof(PxU32), PX_FL);
	mArtiStaticJointIndices.allocate(artiStaticJointSize * sizeof(PxU32), PX_FL);
	mArtiStaticContactCounts.allocate(nbArticulations * sizeof(PxU32), PX_FL);
	mArtiStaticJointCounts.allocate(nbArticulations * sizeof(PxU32), PX_FL);

	mArtiSelfContactIndices.allocate(artiSelfContactIndSize * sizeof(PxU32), PX_FL);
	mArtiSelfJointIndices.allocate(artiSelfJointSize * sizeof(PxU32), PX_FL);
	mArtiSelfContactCounts.allocate(nbArticulations * sizeof(PxU32), PX_FL);
	mArtiSelfJointCounts.allocate(nbArticulations * sizeof(PxU32), PX_FL);

	mCudaContext->memcpyHtoDAsync(mArtiStaticContactIndices.getDevicePtr(), artiStaticContactIndices, sizeof(PxU32)*artiStaticContactIndSize, mStream);
	mCudaContext->memcpyHtoDAsync(mArtiStaticJointIndices.getDevicePtr(), artiStaticJointIndices, sizeof(PxU32)*artiStaticJointSize, mStream);
	mCudaContext->memcpyHtoDAsync(mArtiStaticContactCounts.getDevicePtr(), artiStaticContactCounts, sizeof(PxU32)*nbArticulations, mStream);
	mCudaContext->memcpyHtoDAsync(mArtiStaticJointCounts.getDevicePtr(), artiStaticJointCounts, sizeof(PxU32)*nbArticulations, mStream);

	mCudaContext->memcpyHtoDAsync(mArtiSelfContactIndices.getDevicePtr(), artiSelfContactIndices, sizeof(PxU32)*artiSelfContactIndSize, mStream);
	mCudaContext->memcpyHtoDAsync(mArtiSelfJointIndices.getDevicePtr(), artiSelfJointIndices, sizeof(PxU32)*artiSelfJointSize, mStream);
	mCudaContext->memcpyHtoDAsync(mArtiSelfContactCounts.getDevicePtr(), artiSelfContactCounts, sizeof(PxU32)*nbArticulations, mStream);
	mCudaContext->memcpyHtoDAsync(mArtiSelfJointCounts.getDevicePtr(), artiSelfJointCounts, sizeof(PxU32)*nbArticulations, mStream);

	constructConstraintPrePrepDesc(*mPrePrepDesc, numConstraintBatchHeader, numStaticConstraintBatchHeader, numArticConstraintBatchHeader, numArtiStaticConstraintBatchHeader,
		numArtiSelfConstraintBatchHeader, pData, reinterpret_cast<PxContact*>(cpuContactStreamBase), reinterpret_cast<PxContactPatch*>(cpuContactPatchStreamBase),
		reinterpret_cast<PxReal*>(cpuForceStreamBase), data.nbGpuRigidJoints, data.nbGpuArtiJoints, data.nbTotalArtiJoints, outputIterator, maxNbPartitions, totalActiveBodyCount, nbArticulations, 
		activeBodyStartIndex, shapeInteractions, restDistances, torsionalData, 3, numSlabs);

	constructSolverSharedDesc(*mSharedDesc, cData, islandContextPool->mNumPositionIterations, lengthScale, deferredZ, articulationDirty,
		articulationSlabMask);

	constructConstraitPrepareDesc(*mPrepareDesc, numConstraintBatchHeader, numStaticConstraintBatchHeader,
		numDynamic1DConstraintBatches, data.numStatic1dConstraintBatches, numDynamicContactBatches, data.numStaticContactBatches,
		data.numArti1dConstraintBatches, data.numArtiContactsBatches, data.numArtiStatic1dConstraintBatches, data.numArtiStaticContactsBatches,
		data.numArtiSelf1dConstraintBatches, data.numArtiSelfContactsBatches, cData, totalCurrentEdges, totalPreviousEdges, numSolverBodies);

	constructSolverDesc(*mSolverCoreDesc, numIslands, numSolverBodies, numConstraintBatchHeader,
		numArticConstraintBatchHeader, numSlabs, enableStabilization);

	mRadixSort.constructRadixSortDesc(mRsDesc);

	mCudaContext->memcpyHtoDAsync(mConstraintsPerPartition.getDevicePtr(), pData.constraintsPerPartition, sizeof(PxU32) * pData.numConstraintsPerPartition, mStream);
	mCudaContext->memcpyHtoDAsync(mArtiConstraintsPerPartition.getDevicePtr(), pData.artiConstraintsPerPartition, sizeof(PxU32) * pData.numArtiConstraintsPerPartition, mStream);
	mCudaContext->memcpyHtoDAsync(mConstraint1DBatchIndices.getDevicePtr(), data.constraint1DBatchIndices, sizeof(PxU32) * numDynamic1DConstraintBatches, mStream);
	mCudaContext->memcpyHtoDAsync(mContactBatchIndices.getDevicePtr(), data.constraintContactBatchIndices, sizeof(PxU32) * numDynamicContactBatches, mStream);
	mCudaContext->memcpyHtoDAsync(mArtiContactBatchIndices.getDevicePtr(), data.artiConstraintContactBatchIndices, sizeof(PxU32) * numArtiContactBatches, mStream);
	mCudaContext->memcpyHtoDAsync(mArtiConstraint1dBatchIndices.getDevicePtr(), data.artiConstraint1dBatchindices, sizeof(PxU32) * numArti1dConstraintBatches, mStream);

	mCudaContext->memcpyHtoDAsync(dataBufferd, hostAllocator.mStart, (size_t)hostAllocator.mCurrentSize, mStream);

	mCudaContext->memsetD32Async(solverBodyReferencesd, 0xFFFFFFFF, totalActiveBodyCount * numSlabs, mStream);

	//mCudaContext->streamFlush(mStream);

	mNbArticSlabs = numSlabs;

#if GPU_DEBUG
	CUresult result = mCudaContext->streamSynchronize(mStream);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU DMA up fail!!\n");
#endif
}

void PxgTGSCudaSolverCore::gpuMemDMAbackSolverData(PxU8* forceBufferPool, PxU32 forceBufferOffset, PxU32 forceBufferUpperPartSize,
	PxU32 forceBufferLowerPartSize, Dy::ThresholdStreamElement* changedElems, bool hasForceThresholds, Dy::ConstraintWriteback* constraintWriteBack,
	const PxU32 writeBackSize, bool copyAllToHost, Dy::ErrorAccumulator*& contactError)
{
	PX_PROFILE_ZONE("GpuDynamics.DMABackSolverData", 0);

	//Make mStream2 wait for mStream to finish its work before continuing
	synchronizeStreams(mCudaContext, mStream, mStream2, mEventDmaBack);

	mCudaContext->memcpyDtoHAsync(mSolverCoreDesc, mSolverCoreDescd, sizeof(PxgSolverCoreDesc), mStream2);
		
	contactError = &mSolverCoreDesc->contactErrorAccumulator;

	if (copyAllToHost)
	{
		if (writeBackSize)
		{
			//dma back constraint writeback
			mCudaContext->memcpyDtoHAsync(reinterpret_cast<void*>(constraintWriteBack), mConstraintWriteBackBuffer.getDevicePtr(), writeBackSize * sizeof(Dy::ConstraintWriteback), mStream2);
		}

		//ML : upper part is the cpu force buffer, which the cpu narrow phase fill in the contact face index. Then solver fill in the force in the force buffer 
		if (forceBufferUpperPartSize)
			mCudaContext->memcpyDtoHAsync(forceBufferPool + forceBufferOffset, mForceBuffer.getDevicePtr() + forceBufferOffset, forceBufferUpperPartSize, mStream2);

		//ML : lower part is the gpu force buffer, which the gpu narrow phase fill in contact face index. The solver fill in the force in the force buffer
		if (forceBufferLowerPartSize)
			mCudaContext->memcpyDtoHAsync(forceBufferPool, mForceBuffer.getDevicePtr(), forceBufferLowerPartSize, mStream2);
	}

	if (hasForceThresholds)
	{
		// AD: we copy back this whole thing because of 2 PxU32s: nbExceededThresholdElements and nbForceChangeElements.
		mCudaContext->memcpyDtoHAsync(mSolverCoreDesc, mSolverCoreDescd, sizeof(PxgSolverCoreDesc), mStream2);

		CUdeviceptr pDeviceAddress = 0;
		CUresult result = mCudaContext->memHostGetDevicePointer(&pDeviceAddress, changedElems, 0);
		PX_ASSERT(result == CUDA_SUCCESS);
		//Dispatch kernel to copy from force changed event buffer to changed elems buffer!

		CUfunction function = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::DMA_CHANGED_ELEMS);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(mSolverCoreDescd),
			PX_CUDA_KERNEL_PARAM(pDeviceAddress)
		};

		result = mCudaContext->launchKernel(function, PxgKernelGridDim::DMA_CHANGED_ELEMS, 1, 1, PxgKernelBlockDim::DMA_CHANGED_ELEMS, 1, 1, 0, mStream2, kernelParams, sizeof(kernelParams), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU constraintPartition fail to launch kernel!!\n");
	}

	if (copyAllToHost)
	{
		PxcDataStreamPool& frictionPatchesStreamPool = mGpuContext->getFrictionPatchStreamPool();
		if (frictionPatchesStreamPool.mSharedDataIndexGPU > 0)
			mCudaContext->memcpyDtoHAsync(frictionPatchesStreamPool.mDataStream, mFrictionPatches.getDevicePtr(),
				frictionPatchesStreamPool.mSharedDataIndexGPU, mStream2);
		if (frictionPatchesStreamPool.mSharedDataIndex > 0)
			mCudaContext->memcpyDtoHAsync(frictionPatchesStreamPool.mDataStream + frictionPatchesStreamPool.mDataStreamSize - frictionPatchesStreamPool.mSharedDataIndex,
				mFrictionPatches.getDevicePtr() + frictionPatchesStreamPool.mDataStreamSize - frictionPatchesStreamPool.mSharedDataIndex,
				frictionPatchesStreamPool.mSharedDataIndex, mStream2);
	}

	//mCudaContext->streamFlush(mStream2);

#if GPU_DEBUG
	CUresult result = mCudaContext->streamSynchronize(mStream2);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU pre integration kernel fail!\n");
#endif
}

void PxgTGSCudaSolverCore::syncDmaBack(PxU32& nbChangedThresholdElements)
{
	PX_PROFILE_ZONE("GpuDynamics.DMABackBodies.Sync", 0);
	mCudaContextManager->acquireContext();
	//Wait for mStream to have completed
	/*CUresult result = mCudaContext->streamSynchronize(mStream);
	PX_UNUSED(result);*/
	volatile PxU32* pEvent = mPinnedEvent;
	if (!spinWait(*pEvent, 0.1f))
		mCudaContext->streamSynchronize(mStream);

	PX_ASSERT(PxU32(mSolverCoreDesc->sharedThresholdStreamIndex) >= mSolverCoreDesc->nbExceededThresholdElements);

	nbChangedThresholdElements = mSolverCoreDesc->nbForceChangeElements;
	mNbPrevExceededForceElements = mSolverCoreDesc->nbExceededThresholdElements;

	// AD: safety in case we are in skip mode.
	if (mCudaContext->isInAbortMode())
	{
		nbChangedThresholdElements = 0;
		mNbPrevExceededForceElements = 0;
	}

	/*if(result != CUDA_SUCCESS)
	PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "gpuMemDMAbackSolverBodies fail in sync stream!!\n");*/
	mCudaContextManager->releaseContext();
}

void PxgTGSCudaSolverCore::acquireContext()
{
	mCudaContextManager->acquireContext();
}

void PxgTGSCudaSolverCore::releaseContext()
{
	mCudaContextManager->releaseContext();
}

void PxgTGSCudaSolverCore::preIntegration(const PxU32 offset, const PxU32 nbSolverBodies, const PxReal dt, const PxVec3& gravity)
{
	PX_PROFILE_ZONE("GpuDynamics.ConstraintPrePrepParallel", 0);

	CUdeviceptr islandNodeIndices = mIslandNodeIndices2.getDevicePtr();
	CUdeviceptr solverBodyIndices = mSolverBodyIndices.getDevicePtr();

	// PT: TODO: merge PRE_INTEGRATION and TGS_PRE_INTEGRATION kernels, they call the same code, then refactor the CPU-side launch code between PGS / TGS.
	const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::TGS_PRE_INTEGRATION);
		
	const PxU32 nbBlocks = (nbSolverBodies - offset + PxgKernelBlockDim::PRE_INTEGRATION - 1) / PxgKernelBlockDim::PRE_INTEGRATION;

	if (nbBlocks)
	{
		const bool skipGravityApplication = mGpuContext->isExternalForcesEveryTgsIterationEnabled();

		PxgSimulationCore* simulationCore = mGpuContext->getSimulationCore();
		CUdeviceptr solverBodyDatad = mSolverBodyDataPool.getDevicePtr();
		CUdeviceptr solverBodySleepDatad = mSolverBodySleepDataPool.getDevicePtr();
		PxgDevicePointer<PxgBodySim> bodySimd = simulationCore->getBodySimBufferDevicePtr();
		CUdeviceptr outTransforms = mOutBody2WorldPool.getDevicePtr();
		CUdeviceptr solverTxIDatad = mSolverTxIDataPool.getDevicePtr();
		CUdeviceptr velocityoutd = mOutVelocityPool.getDevicePtr();
			
		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(offset),
			PX_CUDA_KERNEL_PARAM(nbSolverBodies),
			PX_CUDA_KERNEL_PARAM(dt),
			PX_CUDA_KERNEL_PARAM(gravity),
			PX_CUDA_KERNEL_PARAM(solverBodyDatad),
			PX_CUDA_KERNEL_PARAM(solverBodySleepDatad),
			PX_CUDA_KERNEL_PARAM(solverTxIDatad),
			PX_CUDA_KERNEL_PARAM(bodySimd),
			PX_CUDA_KERNEL_PARAM(islandNodeIndices),
			PX_CUDA_KERNEL_PARAM(outTransforms),
			PX_CUDA_KERNEL_PARAM(velocityoutd),
			PX_CUDA_KERNEL_PARAM(solverBodyIndices),
			PX_CUDA_KERNEL_PARAM(skipGravityApplication)
		};

		CUresult launchResult = mCudaContext->launchKernel(kernelFunction, nbBlocks, 1, 1, PxgKernelBlockDim::PRE_INTEGRATION, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(launchResult == CUDA_SUCCESS);
		PX_UNUSED(launchResult);

#if GPU_DEBUG
		CUresult result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU pre integration kernel fail!\n");
#endif
	}

	// PT: TODO: refactor with similar code in PGS
	const CUfunction staticInitFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::INIT_STATIC_KINEMATICS);
	const PxU32 nbStaticBlocks = (offset + PxgKernelBlockDim::PRE_INTEGRATION - 1) / PxgKernelBlockDim::PRE_INTEGRATION;

	if (nbStaticBlocks)
	{
		CUdeviceptr solverBodyDatad = mSolverBodyDataPool.getDevicePtr();
		CUdeviceptr outTransforms = mOutBody2WorldPool.getDevicePtr();
		CUdeviceptr solverTxIDatad = mSolverTxIDataPool.getDevicePtr();
		CUdeviceptr outVelocities = mOutVelocityPool.getDevicePtr();

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(offset),
			PX_CUDA_KERNEL_PARAM(nbSolverBodies),
			PX_CUDA_KERNEL_PARAM(solverBodyDatad),
			PX_CUDA_KERNEL_PARAM(solverTxIDatad),
			PX_CUDA_KERNEL_PARAM(outTransforms),
			PX_CUDA_KERNEL_PARAM(outVelocities),
			PX_CUDA_KERNEL_PARAM(islandNodeIndices),
			PX_CUDA_KERNEL_PARAM(solverBodyIndices)
		};

		CUresult launchResult = mCudaContext->launchKernel(staticInitFunction, nbStaticBlocks, 1, 1, PxgKernelBlockDim::PRE_INTEGRATION, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(launchResult == CUDA_SUCCESS);
		PX_UNUSED(launchResult);

#if GPU_DEBUG
		CUresult result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU static init kernel fail!\n");
#endif
	}
}

void PxgTGSCudaSolverCore::jointConstraintBlockPrePrepParallel(PxU32 nbConstraintBatches)
{
	PX_PROFILE_ZONE("GpuDynamics.ConstraintPrePrepParallel", 0);

	CUdeviceptr descd = mPrePrepDescd;
	PxCudaKernelParam kernelParams[] =
	{
		PX_CUDA_KERNEL_PARAM(descd),
		PX_CUDA_KERNEL_PARAM(mSharedDescd)
	};

	//create block version of joint constraint
	const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::JOINT_CONSTRAINT_PREPREP_BLOCK);

	const PxU32 nbBlocksRequired = (nbConstraintBatches*PXG_BATCH_SIZE + PxgKernelBlockDim::CONSTRAINT_PREPREP_BLOCK - 1) / PxgKernelBlockDim::CONSTRAINT_PREPREP_BLOCK;

	if (nbBlocksRequired)
	{
		CUresult result = mCudaContext->launchKernel(kernelFunction, nbBlocksRequired, 1, 1, PxgKernelBlockDim::CONSTRAINT_PREPREP_BLOCK, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);
	}

#if GPU_DEBUG
	CUresult result = mCudaContext->streamSynchronize(mStream);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU constraintPrePrepare kernel fail!\n");
#endif
}

void PxgTGSCudaSolverCore::jointConstraintPrepareParallel(PxU32 nbJointBatches)
{
	PX_PROFILE_ZONE("GpuDynamics.Constraint1DPrepParallel", 0);

	const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::TGS_JOINT_CONSTRAINT_PREPARE_BLOCK_PARALLEL);

	PxCudaKernelParam kernelParams[] =
	{
		PX_CUDA_KERNEL_PARAM(mPrepareDescd),
		PX_CUDA_KERNEL_PARAM(mSharedDescd)
	};

	const PxU32 nbBlocksRequired = (nbJointBatches*PXG_BATCH_SIZE + PxgKernelBlockDim::CONSTRAINT_PREPARE_BLOCK_PARALLEL - 1) / PxgKernelBlockDim::CONSTRAINT_PREPARE_BLOCK_PARALLEL;

	if (nbBlocksRequired)
	{

		CUresult result = mCudaContext->launchKernel(kernelFunction, nbBlocksRequired, 1, 1, PxgKernelBlockDim::CONSTRAINT_PREPARE_BLOCK_PARALLEL, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU jointConstraintPrepare fail to launch kernel!!\n");
	}

#if GPU_DEBUG
	CUresult result = mCudaContext->streamSynchronize(mStream);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU jointConstraintPrepare fail!!\n");
#endif
}

void PxgTGSCudaSolverCore::contactConstraintPrepareParallel(PxU32 nbContactBatches)
{
	using namespace physx;

	PX_PROFILE_ZONE("GpuDynamics.ConstraintPrepParallel", 0);

	const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::TGS_CONTACT_CONSTRAINT_PREPARE_BLOCK_PARALLEL);

	PxCudaKernelParam kernelParams[] =
	{
		PX_CUDA_KERNEL_PARAM(mPrepareDescd),
		PX_CUDA_KERNEL_PARAM(mSharedDescd)
	};

	PxU32 nbWarpsPerBlock = PxgKernelBlockDim::CONSTRAINT_PREPARE_BLOCK_PARALLEL / 32;

	const PxU32 nbBlocks = (nbContactBatches + nbWarpsPerBlock - 1) / nbWarpsPerBlock;

	if (nbBlocks > 0)
	{
		CUresult result = mCudaContext->launchKernel(kernelFunction, nbBlocks, 1, 1, PxgKernelBlockDim::CONSTRAINT_PREPARE_BLOCK_PARALLEL, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU contactConstraintPrepare fail to launch kernel!!\n");

#if GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU contactConstraintPrepare fail!!\n");
#endif
	}
}

void PxgTGSCudaSolverCore::artiJointConstraintPrepare(PxU32 nbArtiJointBatches)
{
	PX_PROFILE_ZONE("GpuDynamics.artiJointConstraintPrepareParallel", 0);

	const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_JOINT_PREP_TGS);

	PxCudaKernelParam kernelParams[] =
	{
		PX_CUDA_KERNEL_PARAM(mPrepareDescd),
		PX_CUDA_KERNEL_PARAM(mSharedDescd)
	};

	const PxU32 numThreadsPerWarp = 32;

	PxU32 nbWarpsPerBlock = PxgKernelBlockDim::ARTI_CONSTRAINT_PREPARE / numThreadsPerWarp;

	const PxU32 nbBlocksRequired = (nbArtiJointBatches + nbWarpsPerBlock - 1) / nbWarpsPerBlock;

	if (nbBlocksRequired)
	{
		CUresult result = mCudaContext->launchKernel(kernelFunction, nbBlocksRequired, 1, 1, numThreadsPerWarp, nbWarpsPerBlock, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU artiJointConstraintPrepare fail to launch kernel!!\n");

#if GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU artiJointConstraintPrepare fail!!\n");
#endif
	}
}

void PxgTGSCudaSolverCore::artiContactConstraintPrepare(PxU32 nbArtiContactBatches)
{
	using namespace physx;

	PX_PROFILE_ZONE("GpuDynamics.artiContactConstraintPrepParallel", 0);
		
	const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ARTI_CONTACT_PREP_TGS);
			
	PxCudaKernelParam kernelParams[] =
	{
		PX_CUDA_KERNEL_PARAM(mPrepareDescd),
		PX_CUDA_KERNEL_PARAM(mSharedDescd)
	};

	PxU32 nbWarpsPerBlock = PxgKernelBlockDim::ARTI_CONSTRAINT_PREPARE / 32;

	const PxU32 nbBlocks = (nbArtiContactBatches + nbWarpsPerBlock - 1) / nbWarpsPerBlock;
	const PxU32 numThreadsPerWarp = 32;
	if (nbBlocks > 0)
	{
		CUresult result = mCudaContext->launchKernel(kernelFunction, nbBlocks, 1, 1, numThreadsPerWarp, nbWarpsPerBlock, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU artiContactConstraintPrepare fail to launch kernel!!\n");

#if GPU_DEBUG
		result = cuStreamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU artiContactConstraintPrepare fail!!\n");
#endif
	}
}

void PxgTGSCudaSolverCore::nonRigidConstraintPrepare(PxU32 numArticulations)
{
	PxgSoftBodyCore* softBodyCore = mGpuContext->getGpuSoftBodyCore();
	if (softBodyCore)
	{
		softBodyCore->constraintPrep(mPrePrepDescd, mPrepareDescd, mSharedDesc->stepInvDtF32, mSharedDescd, mStream, true, mGpuContext->mSolverBodyPool.size(), numArticulations);
	}

	PxgFEMClothCore* femClothCore = mGpuContext->getGpuFEMClothCore();
	if (femClothCore)
	{
		femClothCore->constraintPrep(mPrePrepDescd, mPrepareDescd, mSharedDesc->stepInvDtF32, mSharedDescd, mStream, mGpuContext->mSolverBodyPool.size(), numArticulations);
	}

	PxgParticleSystemCore** particleSystemCores = mGpuContext->getGpuParticleSystemCores();
	const PxU32 numParticleCores = mGpuContext->getNbGpuParticleSystemCores();

	for (PxU32 i = 0; i < numParticleCores; ++i)
	{
		PxgParticleSystemCore* particleSystemCore = particleSystemCores[i];

		particleSystemCore->constraintPrep(mPrePrepDescd, mPrepareDescd, mSolverCoreDescd, mSharedDescd, mSharedDesc->dt, mStream, true, mGpuContext->mSolverBodyPool.size());
	}
}

void PxgTGSCudaSolverCore::writeBackBlock(PxU32 a, PxgIslandContext& context)
{
	const CUfunction writebackBlockFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::WRITEBACK_BLOCKS_TGS);

	PxCudaKernelParam kernelParams[] =
	{
		PX_CUDA_KERNEL_PARAM(mSolverCoreDescd),
		PX_CUDA_KERNEL_PARAM(mSharedDescd),
		PX_CUDA_KERNEL_PARAM(a),
	};

	PxU32 nbBlocksRequired = ((context.mArtiBatchCount + context.mStaticArtiBatchCount + context.mBatchCount + context.mSelfArtiBatchCount + context.mStaticRigidBatchCount)*PXG_BATCH_SIZE + PxgKernelBlockDim::WRITEBACK_BLOCKS - 1) / PxgKernelBlockDim::WRITEBACK_BLOCKS;
	if (nbBlocksRequired)
	{

		CUresult result = mCudaContext->launchKernel(writebackBlockFunction, nbBlocksRequired, 1, 1, PxgKernelBlockDim::WRITEBACK_BLOCKS, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU mWritebackBlockFunction fail to launch kernel!!\n");

#if GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU mWritebackBlockFunction kernel fail!\n");
#endif	
	}
}

void PxgTGSCudaSolverCore::solvePartitions(PxgIslandContext* islandContexts, PxInt32ArrayPinned& constraintsPerPartition,
	PxInt32ArrayPinned& artiConstraintsPerPartition, PxU32 islandIndex, bool doFriction, PxReal accumulatedDt, 
	PxReal minPen, bool anyArticulationConstraints, bool isVelocityIteration)
{
	PxgIslandContext& context = islandContexts[islandIndex];

	const PxU32 numThreadsPerWarp = WARP_SIZE;
	PxU32 numWarpsPerBlock = PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES / numThreadsPerWarp;
	CUdeviceptr artiDescd = mGpuContext->getArticulationCore()->getArticulationCoreDescd();

	const CUfunction solveWholeIslandFunction =
	    mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RIGID_SOLVE_WHOLE_ISLAND_TGS);
	const CUfunction solveUnifiedBlock =
	    mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SOLVE_UNIFIED);
	const CUfunction computeBodiesAverageVelocitiesFunction =
	    mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::COMPUTE_AVERAGE_VELOCITY_TGS);

	PxU32 bodyCount = (context.mBodyCount + context.mBodyStartIndex) * mSolverCoreDesc->numSlabs;

	const PxU32 maxBodies = 944; 

	PxU32 startIndex = 0;
	PxU32 startArtic = 0;

	PxCudaKernelParam defaultKernelParams[] = { PX_CUDA_KERNEL_PARAM(mSolverCoreDescd), PX_CUDA_KERNEL_PARAM(mSharedDescd) };

	{
		if(!isVelocityIteration && context.mArticulationCount == 0 && bodyCount <= maxBodies)
		{
			PX_PROFILE_ZONE("GpuDynamics.Solve.solveWholeIsland", 0);

			// Update reference count every sub-timestep or iteration.
			// Though not perfect, this provides sufficiently accurate reference counts.
			// Note that the reference count is updated after the first call of "averageDeltaV" to keep the articulation reference count in sync.
			const bool isTGS = true;
			precomputeReferenceCount(islandContexts, islandIndex, constraintsPerPartition, artiConstraintsPerPartition, isTGS, minPen,
									 accumulatedDt);

			PxCudaKernelParam kernelParams[] = {
				PX_CUDA_KERNEL_PARAM(mSolverCoreDescd), PX_CUDA_KERNEL_PARAM(mSharedDescd),
				PX_CUDA_KERNEL_PARAM(islandIndex),      PX_CUDA_KERNEL_PARAM(doFriction),
				PX_CUDA_KERNEL_PARAM(accumulatedDt),    PX_CUDA_KERNEL_PARAM(minPen)
			};

			CUresult result = mCudaContext->launchKernel(solveWholeIslandFunction, 1, 1, 1, 256, 1, 1, 0, mStream, kernelParams,
														 sizeof(kernelParams), 0, PX_FL);
			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU solveWholdIslandTGS fail to launch kernel!!\n");
		}
		else
		{
			PX_PROFILE_ZONE("GpuDynamics.Solve.solvePartitions", 0);

			for(PxU32 c = 0; c < context.mNumPartitions; ++c)
			{
				PxCudaKernelParam kernelParams[] = { PX_CUDA_KERNEL_PARAM(mSolverCoreDescd), PX_CUDA_KERNEL_PARAM(mSharedDescd),
													 PX_CUDA_KERNEL_PARAM(islandIndex),		 PX_CUDA_KERNEL_PARAM(c),
													 PX_CUDA_KERNEL_PARAM(doFriction),		 PX_CUDA_KERNEL_PARAM(accumulatedDt),
													 PX_CUDA_KERNEL_PARAM(minPen),			 PX_CUDA_KERNEL_PARAM(artiDescd) };

				{
					PxU32 endIndex = constraintsPerPartition[c];

					const PxU32 nbBlocks = ((endIndex - startIndex) * PXG_BATCH_SIZE + PxgKernelBlockDim::SOLVE_BLOCK_PARTITION - 1) /
										   (PxgKernelBlockDim::SOLVE_BLOCK_PARTITION);

					startIndex = endIndex;

					endIndex = artiConstraintsPerPartition[c];

					PxU32 nbArtiBlocks = ((endIndex - startArtic) * PXG_BATCH_SIZE + PxgKernelBlockDim::SOLVE_BLOCK_PARTITION - 1) /
										 (PxgKernelBlockDim::SOLVE_BLOCK_PARTITION);

					startArtic = endIndex;

					const PxU32 maxBlocks = PxMax(nbBlocks, nbArtiBlocks);

					// In case of per-substep force application, we need to force-run the articulation averageDeltaV
					// function in partition 0 even if no articulation constraints are present in that partition.
					if(nbArtiBlocks > 0 || (anyArticulationConstraints && c == 0))
					{
						mGpuContext->getArticulationCore()->averageDeltaV(
							mNbArticSlabs, mStream, reinterpret_cast<float4*>(mSolverBodyPool.getDevicePtr()), c, true, mSharedDescd);
					}

					// Update reference count every sub-timestep or iteration.
					// Though not perfect, this provides sufficiently accurate reference counts.
					// Note that the reference count is updated after the first call of "averageDeltaV" to keep the articulation reference
					// count in sync.
					if(c == 0)
					{
						const bool isTGS = true;
						precomputeReferenceCount(islandContexts, islandIndex, constraintsPerPartition, artiConstraintsPerPartition, isTGS,
												 minPen, accumulatedDt);
					}

					if(maxBlocks)
					{
						const PxU32 blockY = nbArtiBlocks > 0 ? 2 : 1;
						CUresult result =
							mCudaContext->launchKernel(solveUnifiedBlock, maxBlocks, blockY, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0,
													   mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
						if(result != CUDA_SUCCESS)
							PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU solveBlockUnified fail to launch kernel!!\n");

#if GPU_DEBUG
						result = mCudaContext->streamSynchronize(mStream);
						if(result != CUDA_SUCCESS)
							PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU solveBlockUnified kernel fail!\n");
#endif
					}
				}
			}
		}
	}

	// we need to kick off a kernel to average velocity for the same body but in different partitions
	{
		const PxU32 nbThreadsRequired = 32 * context.mBodyCount;
		const PxU32 nbBlocksRequired = (nbThreadsRequired + PxgKernelBlockDim::COMPUTE_BODIES_AVERAGE_VELOCITY - 1) /
									   PxgKernelBlockDim::COMPUTE_BODIES_AVERAGE_VELOCITY;

		if(nbBlocksRequired > 0)
		{
			CUresult result = mCudaContext->launchKernel(computeBodiesAverageVelocitiesFunction, nbBlocksRequired, 1, 1, 32,
														 PxgKernelBlockDim::COMPUTE_BODIES_AVERAGE_VELOCITY / 32, 1, 0, mStream,
														 defaultKernelParams, sizeof(defaultKernelParams), 0, PX_FL);
			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
										"GPU compute solver bodies average velocities fail to launch kernel!!\n");

#if GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			if(result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU compute solver bodies average velocities kernel fail!\n");
#endif
		}
	}
}

void PxgTGSCudaSolverCore::solveContactMultiBlockParallel(PxgIslandContext* islandContexts, const PxU32 numIslands, const PxU32 /*maxPartitions*/,
	PxInt32ArrayPinned& constraintsPerPartition, PxInt32ArrayPinned& artiConstraintsPerPartition, const PxVec3& gravity, 
	PxReal* posIterResidualSharedMem, PxU32 posIterResidualSharedMemSize, Dy::ErrorAccumulator* posIterError, PxPinnedArray<Dy::ErrorAccumulator>& artiContactPosIterError,
	PxPinnedArray<Dy::ErrorAccumulator>& perArticulationInternalError)
{
	PX_PROFILE_ZONE("GpuDynamics.Solve", 0);

	PxgSoftBodyCore* softbodyCore = mGpuContext->getGpuSoftBodyCore(); 
		
	PxgParticleSystemCore** particleCores = mGpuContext->getGpuParticleSystemCores();

	const PxU32 numParticleCores = mGpuContext->getNbGpuParticleSystemCores();

	PxgFEMClothCore* femClothCore = mGpuContext->getGpuFEMClothCore(); 

	const CUfunction initBodiesFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::TGS_INIT_SOLVER_VELS);		
	const CUfunction concludeBlockFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONCLUDE_BLOCKS_TGS);
	const CUfunction writebackBodiesFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::WRITE_BACK_BODIES_TGS);		
	const CUfunction propagateBodiesVelocityFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PROPAGATE_AVERAGE_VELOCITY_TGS);

	const CUfunction solveRigidStaticconstraintsFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RIGID_SOLVE_STATIC_CONSTRAINTS_TGS);
	const CUfunction propagateRigidStaticconstraintsFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PROPAGATE_STATIC_SOLVER_VELOCITIES_TGS);

	const PxReal stepDt = mSharedDesc->dt/PxReal(islandContexts->mNumPositionIterations);

	const PxReal invStepDt = 1.f / stepDt;
	const PxReal invTotalDt = 1.f/ mSharedDesc->dt;

	CUdeviceptr artiDescd = mGpuContext->getArticulationCore()->getArticulationCoreDescd();

	const PxReal biasCoefficient = islandContexts->mBiasCoefficient;

	// resetVelocities (ZeroBodies) is called in "doConstraintPrepGPU"

	for (PxU32 a = 0; a < numIslands; ++a)
	{
		PxgIslandContext& context = islandContexts[a];
			
		PxReal accumulatedDt = 0.f;

		{
			const PxU32 nbThreadsRequired = 32 * (context.mBodyCount + context.mBodyStartIndex);
			const PxU32 nbBlocksRequired = (nbThreadsRequired + PxgKernelBlockDim::COMPUTE_BODIES_AVERAGE_VELOCITY - 1) / PxgKernelBlockDim::COMPUTE_BODIES_AVERAGE_VELOCITY;

			if (nbBlocksRequired > 0)
			{
				PxCudaKernelParam initBodiesParams[] =
				{
					PX_CUDA_KERNEL_PARAM(mSolverCoreDescd),
					PX_CUDA_KERNEL_PARAM(mSharedDescd)
				};

				CUresult result = mCudaContext->launchKernel(initBodiesFunction, nbBlocksRequired, 1, 1, 32, PxgKernelBlockDim::COMPUTE_BODIES_AVERAGE_VELOCITY / 32, 1, 0, mStream, initBodiesParams, sizeof(initBodiesParams), 0, PX_FL);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU initializeSolverVelocitiesTGS fail to launch kernel!! %i\n", result);

#if GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU initializeSolverVelocitiesTGS kernel fail!\n");
#endif
			}
		}

		const bool isTGS = true;
		PxReal minPen = -PX_MAX_F32;
		bool doFriction = true;
		bool isVelocityIteration = false;
		bool isFinalIteration = false;
		const bool externalForcesEveryTgsIterationEnabled = mGpuContext->isExternalForcesEveryTgsIterationEnabled();

		//Zero contact error accumulators
		PxReal* zeroA = ((PxReal*)mSolverCoreDescd) + offsetof(PxgSolverCoreDesc, contactErrorAccumulator) / sizeof(PxReal);
		PxReal* zeroB = ((PxReal*)artiDescd) + offsetof(PxgArticulationCoreDesc, mContactErrorAccumulator) / sizeof(PxReal);
		const bool residualReportingEnabled = mGpuContext->isResidualReportingEnabled();
		const PxU32 clearValue = residualReportingEnabled ? 0u : 0xFFFFFFFFu;
		//Clear the residual accumulation values at least once even if residual accumulation is not enabled because depending on the value used
		//for clearing, residuals will get computed or not.
		mCudaContext->memsetD32Async(CUdeviceptr(zeroA), clearValue, sizeof(Dy::ErrorAccumulator) / sizeof(PxU32), mStream);
		mCudaContext->memsetD32Async(CUdeviceptr(zeroB), clearValue, sizeof(Dy::ErrorAccumulator) / sizeof(PxU32), mStream);

		for (PxI32 b = 0; b < context.mNumPositionIterations; ++b)
		{
			PX_PROFILE_ZONE("GpuDynamics.Solve.PosIteration", 0);

			if (residualReportingEnabled) 
			{
				//Zero contact error accumulators
				mCudaContext->memsetD32Async(CUdeviceptr(zeroA), clearValue, sizeof(Dy::ErrorAccumulator) / sizeof(PxU32), mStream);
				mCudaContext->memsetD32Async(CUdeviceptr(zeroB), clearValue, sizeof(Dy::ErrorAccumulator) / sizeof(PxU32), mStream);
			}

			//KS - this works because PX_CUDA_KERNEL_PARAM is a pointer to the data
			if (b == (context.mNumPositionIterations - 1))
				isFinalIteration = true;

			if (externalForcesEveryTgsIterationEnabled)
			{
				// Apply substep gravity to rigids
				const PxU32 nbThreadsRequired = 32 * (context.mBodyCount + context.mBodyStartIndex);
				const PxU32 nbBlocksRequired = (nbThreadsRequired + PxgKernelBlockDim::COMPUTE_BODIES_AVERAGE_VELOCITY - 1) / PxgKernelBlockDim::COMPUTE_BODIES_AVERAGE_VELOCITY;

				if (nbBlocksRequired > 0)
				{
					PxCudaKernelParam kernelParams[] =
					{
						PX_CUDA_KERNEL_PARAM(mSolverCoreDescd),
						PX_CUDA_KERNEL_PARAM(mSharedDescd),
						PX_CUDA_KERNEL_PARAM(gravity)
					};

					CUfunction applyTGSSubstepGravity = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::APPLY_TGS_SUBSTEP_GRAVITY);
					CUresult result = mCudaContext->launchKernel(applyTGSSubstepGravity, nbBlocksRequired, 1, 1, 32, PxgKernelBlockDim::COMPUTE_BODIES_AVERAGE_VELOCITY / 32, 1, 0,
						mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU applyTGSSubstepGravity fail to launch kernel!! %i\n", result);
				}

				// Apply substep gravity to articulations
				mGpuContext->getArticulationCore()->applyTgsSubstepForces(stepDt, mStream);
			}
			
			solvePartitions(islandContexts, constraintsPerPartition, artiConstraintsPerPartition, a, doFriction, accumulatedDt, minPen, externalForcesEveryTgsIterationEnabled, isVelocityIteration);

			// Before solving internal constraints, ensure to propagate remaining impulses from solvePartitions to the
			// articulation root via "averageLinkImpulsesAndPropagate"
			// This ensures that future impulse propagations do not originate from rigid body or articulation impulses,
			// and that there are no lingering impulses from the rigid body or articulation solver (i.e.,
			// solvePartitions).
			mGpuContext->getArticulationCore()->propagateRigidBodyImpulsesAndSolveInternalConstraints(
			    stepDt, invStepDt, isVelocityIteration, accumulatedDt, biasCoefficient,
			    reinterpret_cast<PxU32*>(mArtiOrderedStaticContacts.getDevicePtr()),
			    reinterpret_cast<PxU32*>(mArtiOrderedStaticConstraints.getDevicePtr()), mSharedDescd,
			    doFriction, isTGS, residualReportingEnabled, externalForcesEveryTgsIterationEnabled);

			if (softbodyCore || numParticleCores > 0 || femClothCore)
			{
				mGpuContext->getArticulationCore()->outputVelocity(mSolverCoreDescd, mStream, true);
			}

			for (PxU32 i = 0; i < numParticleCores; ++i)
			{
				particleCores[i]->solveTGS(mPrePrepDescd, mSolverCoreDescd, mSharedDescd, artiDescd, stepDt, invTotalDt, mStream, false, b, context.mNumPositionIterations, islandContexts->mBiasCoefficient);
			}

			if (femClothCore)
			{
				femClothCore->solve(mPrePrepDescd, mSolverCoreDescd, mSharedDescd, artiDescd, stepDt, mStream, b,
									context.mNumPositionIterations, false, gravity);
			}

			if (softbodyCore)
			{
				softbodyCore->solveTGS(mPrePrepDescd, mPrepareDescd, mSolverCoreDescd, mSharedDescd, artiDescd, stepDt, mStream,
					false, islandContexts->mBiasCoefficient*invStepDt, b == 0, gravity);
			}

			for (PxU32 i = 0; i < numParticleCores; ++i)
			{
				particleCores[i]->updateParticles(stepDt);
			}

			{
				const PxU32 nbBlocksRequired = ((context.mBodyCount + PxgKernelBlockDim::SOLVE_BLOCK_PARTITION - 1) / PxgKernelBlockDim::SOLVE_BLOCK_PARTITION)*mNbStaticRigidSlabs;

				if (nbBlocksRequired)
				{
					PxCudaKernelParam staticKernelParams[] =
					{
						PX_CUDA_KERNEL_PARAM(mSolverCoreDescd),
						PX_CUDA_KERNEL_PARAM(mSharedDescd),
						PX_CUDA_KERNEL_PARAM(a),
						PX_CUDA_KERNEL_PARAM(mNbStaticRigidSlabs),
						PX_CUDA_KERNEL_PARAM(mMaxNumStaticPartitions),
						PX_CUDA_KERNEL_PARAM(accumulatedDt),
						PX_CUDA_KERNEL_PARAM(minPen)
					};

					CUresult result = mCudaContext->launchKernel(solveRigidStaticconstraintsFunction, nbBlocksRequired, 1, 1, PxgKernelBlockDim::SOLVE_BLOCK_PARTITION, 1, 1, 0, mStream, staticKernelParams, sizeof(staticKernelParams), 0, PX_FL);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU solveStaticBlock fail to launch kernel!!\n");
						
#if GPU_DEBUG
					result = mCudaContext->streamSynchronize(mStream);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU solveStaticBlock kernel fail!\n");
#endif			

					result = mCudaContext->launchKernel(propagateRigidStaticconstraintsFunction, nbBlocksRequired, 1, 1, PxgKernelBlockDim::SOLVE_BLOCK_PARTITION, 1, 1, 0, mStream, staticKernelParams, sizeof(staticKernelParams), 0, PX_FL);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU solveStaticBlock fail to launch kernel!!\n");
				}
			}

			{
				const PxU32 nbThreadsRequired = 32 * (context.mBodyCount + context.mBodyStartIndex);
				const PxU32 nbBlocksRequired = (nbThreadsRequired + PxgKernelBlockDim::COMPUTE_BODIES_AVERAGE_VELOCITY - 1) / PxgKernelBlockDim::COMPUTE_BODIES_AVERAGE_VELOCITY;

				if (nbBlocksRequired > 0)
				{
					PxCudaKernelParam propagateBodiesVelocityParams[] =
					{
						PX_CUDA_KERNEL_PARAM(mSolverCoreDescd),
						PX_CUDA_KERNEL_PARAM(mSharedDescd),
						PX_CUDA_KERNEL_PARAM(isVelocityIteration),
						PX_CUDA_KERNEL_PARAM(isFinalIteration),
						PX_CUDA_KERNEL_PARAM(biasCoefficient)
					};

					CUresult result = mCudaContext->launchKernel(propagateBodiesVelocityFunction, nbBlocksRequired, 1, 1, 32, PxgKernelBlockDim::COMPUTE_BODIES_AVERAGE_VELOCITY / 32, 1, 0, mStream, propagateBodiesVelocityParams, sizeof(propagateBodiesVelocityParams), 0, PX_FL);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU propagate solver bodies average velocities fail to launch kernel!!\n");

#if GPU_DEBUG
					result = mCudaContext->streamSynchronize(mStream);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU compute solver bodies average velocities kernel fail!\n");
#endif
				}
			}

			mGpuContext->getArticulationCore()->stepArticulation(stepDt);

			accumulatedDt += stepDt;
		}//end of mNumPositionIterations

		{
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(mSolverCoreDescd),
				PX_CUDA_KERNEL_PARAM(mSharedDescd),
				PX_CUDA_KERNEL_PARAM(a)
			};

			const PxU32 nbBlocksRequired = ((context.mArtiBatchCount + context.mStaticArtiBatchCount + context.mSelfArtiBatchCount + context.mBatchCount + context.mStaticRigidBatchCount)*PXG_BATCH_SIZE + PxgKernelBlockDim::CONCLUDE_BLOCKS - 1) / PxgKernelBlockDim::CONCLUDE_BLOCKS;

			if (nbBlocksRequired)
			{

				CUresult result = mCudaContext->launchKernel(concludeBlockFunction, nbBlocksRequired, 1, 1, PxgKernelBlockDim::CONCLUDE_BLOCKS, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU concludeBlockFunction fail to launch kernel!!\n");

#if GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU concludeBlockFunction kernel fail!\n");
#endif
			}
		}

		{
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(mSolverCoreDescd),
				PX_CUDA_KERNEL_PARAM(mSharedDescd),
				PX_CUDA_KERNEL_PARAM(a),
			};

			const PxU32 nbBlocksRequired = (context.mBodyCount + PxgKernelBlockDim::WRITE_BACK_BODIES - 1) / PxgKernelBlockDim::WRITE_BACK_BODIES;

			if (nbBlocksRequired)
			{
				CUresult result = mCudaContext->launchKernel(writebackBodiesFunction, nbBlocksRequired, 1, 1, PxgKernelBlockDim::WRITE_BACK_BODIES, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU solveContactParallel fail to launch kernel!!\n");

#if GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU solveContactParallel kernel fail!\n");
#endif
			}

			//mArticulationCore->saveVelocities();
		}

		doFriction = true;
		//KS - do not zero this. Velocity iterations cause a bunch of problems for the TGS solver because they
		//effectively move the goal post when the solver was already converging on a solution. The solver itself
		//supports implicit gentle de-penetration and we support bias clamping in order to limit energy gain
		//from penetration.
		minPen = 0.f; 
		isVelocityIteration = true;
		isFinalIteration = false;
			
		if (residualReportingEnabled)
		{
			writeBackBlock(a, context);

			if (posIterResidualSharedMemSize > 0)
			{
				CUfunction function = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::DMA_CONSTRAINT_RESIDUAL);

				CUdeviceptr ptr = mConstraintWriteBackBuffer.getDevicePtr();
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(ptr),
					PX_CUDA_KERNEL_PARAM(posIterResidualSharedMem),
					PX_CUDA_KERNEL_PARAM(posIterResidualSharedMemSize)
				};

				PxU32 threadBlockSize = 256;
				PxU32 gridSize = (posIterResidualSharedMemSize + threadBlockSize - 1) / threadBlockSize;

				PxCUresult result = mCudaContext->launchKernel(function, gridSize, 1, 1, threadBlockSize, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU dmaConstraintResidual fail to launch kernel!!\n");
			}

			if (mGpuContext->getArticulationCore()->getArticulationCoreDesc()->nbArticulations > 0)
			{
				//perArticulationInternalError.resize(mGpuContext->getArticulationCore()->getArticulationCoreDesc()->nbArticulations);

				CUfunction function = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::DMA_ARTICULATION_RESIDUAL);

				CUdeviceptr ptr = (CUdeviceptr)perArticulationInternalError.begin();
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(artiDescd),
					PX_CUDA_KERNEL_PARAM(ptr)
				};

				PxU32 threadBlockSize = 256;
				PxU32 gridSize = (mGpuContext->getArticulationCore()->getArticulationCoreDesc()->nbArticulations + threadBlockSize - 1) / threadBlockSize;

				PxCUresult result = mCudaContext->launchKernel(function, gridSize, 1, 1, threadBlockSize, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU dmaArticulationResidual fail to launch kernel!!\n");
			}

			PX_UNUSED(perArticulationInternalError);

			{
				PxgSolverCoreDesc* gpuPtr = (PxgSolverCoreDesc*)mSolverCoreDescd;
				mCudaContext->memcpyDtoHAsync(posIterError, (CUdeviceptr)&gpuPtr->contactErrorAccumulator, sizeof(Dy::ErrorAccumulator), mStream); //Should posIterError be shared memory?	
			}
			{
				artiContactPosIterError.resize(1);
				PxgArticulationCoreDesc* gpuPtr = (PxgArticulationCoreDesc*)mGpuContext->getArticulationCore()->getArticulationCoreDescd();
				mCudaContext->memcpyDtoHAsync(artiContactPosIterError.begin(), (CUdeviceptr)&gpuPtr->mContactErrorAccumulator, sizeof(Dy::ErrorAccumulator), mStream);
			}
		}

		const bool anyArticulationConstraints = (context.mArtiBatchCount + context.mStaticArtiBatchCount + context.mSelfArtiBatchCount) > 0;
		for (PxI32 b = 0; b < context.mNumVelocityIterations; ++b)
		{
			PX_PROFILE_ZONE("GpuDynamics.Solve.VelIteration", 0);

			if (residualReportingEnabled)
			{
				//Zero contact error accumulators
				mCudaContext->memsetD32Async(CUdeviceptr(zeroA), clearValue, sizeof(Dy::ErrorAccumulator) / sizeof(PxU32), mStream);
				mCudaContext->memsetD32Async(CUdeviceptr(zeroB), clearValue, sizeof(Dy::ErrorAccumulator) / sizeof(PxU32), mStream);
			}
			
			solvePartitions(islandContexts, constraintsPerPartition, artiConstraintsPerPartition, a, doFriction, accumulatedDt, minPen, anyArticulationConstraints, isVelocityIteration);

			// Before solving internal constraints, ensure to propagate remaining impulses from solvePartitions to the
			// articulation root via "averageLinkImpulsesAndPropagate"
			// This ensures that future impulse propagations do not originate from rigid body or articulation impulses,
			// and that there are no lingering impulses from the rigid body or articulation solver (i.e.,
			// solvePartitions).
			mGpuContext->getArticulationCore()->propagateRigidBodyImpulsesAndSolveInternalConstraints(
			    stepDt, invStepDt, isVelocityIteration, accumulatedDt, biasCoefficient,
			    reinterpret_cast<PxU32*>(mArtiOrderedStaticContacts.getDevicePtr()),
			    reinterpret_cast<PxU32*>(mArtiOrderedStaticConstraints.getDevicePtr()), mSharedDescd,
			    doFriction, isTGS, residualReportingEnabled, externalForcesEveryTgsIterationEnabled);

			if (softbodyCore || numParticleCores > 0 || femClothCore)
			{
				mGpuContext->getArticulationCore()->outputVelocity(mSolverCoreDescd, mStream, true);
			}

			for (PxU32 i = 0; i < numParticleCores; ++i)
			{
				particleCores[i]->solveTGS(mPrePrepDescd, mSolverCoreDescd, mSharedDescd, artiDescd, stepDt, invTotalDt, mStream, true, -1, -1, islandContexts->mBiasCoefficient);
			}

			//! no velocity iteration support for FEM cloth
				
			if (softbodyCore)
			{
				softbodyCore->solveTGS(mPrePrepDescd, mPrepareDescd, mSolverCoreDescd, mSharedDescd, artiDescd, stepDt, mStream,
					true, islandContexts->mBiasCoefficient*invStepDt, false, gravity);
			}

			for (PxU32 i = 0; i < numParticleCores; ++i)
			{
				particleCores[i]->updateParticles(stepDt);
			}

			{
				const PxU32 nbBlocksRequired = ((context.mBodyCount + PxgKernelBlockDim::SOLVE_BLOCK_PARTITION - 1) / PxgKernelBlockDim::SOLVE_BLOCK_PARTITION)*mNbStaticRigidSlabs;

				if (nbBlocksRequired)
				{
					PxCudaKernelParam staticKernelParams[] =
					{
						PX_CUDA_KERNEL_PARAM(mSolverCoreDescd),
						PX_CUDA_KERNEL_PARAM(mSharedDescd),
						PX_CUDA_KERNEL_PARAM(a),
						PX_CUDA_KERNEL_PARAM(mNbStaticRigidSlabs),
						PX_CUDA_KERNEL_PARAM(mMaxNumStaticPartitions),
						PX_CUDA_KERNEL_PARAM(accumulatedDt),
						PX_CUDA_KERNEL_PARAM(minPen)
					};

					CUresult result = mCudaContext->launchKernel(solveRigidStaticconstraintsFunction, nbBlocksRequired, 1, 1, PxgKernelBlockDim::SOLVE_BLOCK_PARTITION, 1, 1, 0, mStream, staticKernelParams, sizeof(staticKernelParams), 0, PX_FL);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU solveStaticBlock fail to launch kernel!!\n");
#if GPU_DEBUG
					result = mCudaContext->streamSynchronize(mStream);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU solveStaticBlock kernel fail!\n");
#endif			
						
					result = mCudaContext->launchKernel(propagateRigidStaticconstraintsFunction, nbBlocksRequired, 1, 1, PxgKernelBlockDim::SOLVE_BLOCK_PARTITION, 1, 1, 0, mStream, staticKernelParams, sizeof(staticKernelParams), 0, PX_FL);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU solveStaticBlock fail to launch kernel!!\n");
				}
			}

			{
				const PxU32 nbThreadsRequired = 32 * ((context.mBodyCount + 9) / 10);
				const PxU32 nbBlocksRequired = (nbThreadsRequired + PxgKernelBlockDim::COMPUTE_BODIES_AVERAGE_VELOCITY - 1) / PxgKernelBlockDim::COMPUTE_BODIES_AVERAGE_VELOCITY;

				if (nbBlocksRequired > 0)
				{
					PxCudaKernelParam propagateBodiesVelocityParams[] =
					{
						PX_CUDA_KERNEL_PARAM(mSolverCoreDescd),
						PX_CUDA_KERNEL_PARAM(mSharedDescd),
						PX_CUDA_KERNEL_PARAM(isVelocityIteration),
						PX_CUDA_KERNEL_PARAM(isFinalIteration),
						PX_CUDA_KERNEL_PARAM(biasCoefficient)
					};

					CUresult result = mCudaContext->launchKernel(propagateBodiesVelocityFunction, nbBlocksRequired, 1, 1, 32, PxgKernelBlockDim::COMPUTE_BODIES_AVERAGE_VELOCITY / 32, 1, 0, mStream, propagateBodiesVelocityParams, sizeof(propagateBodiesVelocityParams), 0, PX_FL);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU compute solver bodies average velocities fail to launch kernel!!\n");

#if GPU_DEBUG
					result = mCudaContext->streamSynchronize(mStream);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU compute solver bodies average velocities kernel fail!\n");
#endif
				}
			}

		}//end of mNumVelocityIterations

		writeBackBlock(a, context);

		if (softbodyCore)
		{
			softbodyCore->copyContactCountsToHost();
			softbodyCore->finalizeVelocities(mSharedDesc->dt, biasCoefficient, true);
		}

		for (PxU32 i = 0; i < numParticleCores; ++i)
		{
			particleCores[i]->finalizeVelocities(mSharedDesc->dt, biasCoefficient);
		}

		if (femClothCore)
		{
			femClothCore->copyContactCountsToHost();
			femClothCore->finalizeVelocities(mSharedDesc->dt);
		}
	}

#if GPU_DEBUG
	CUresult result = mCudaContext->streamSynchronize(mStream);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU DMA back fail 6!!\n");
#endif

}

void PxgTGSCudaSolverCore::radixSort(const PxU32 nbPasses)
{
	CUdeviceptr solverDescd = mSolverCoreDescd;

	CUfunction radixFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RADIXSORT_SINGLEBLOCK);
	CUfunction calculateRanksFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RADIXSORT_CALCULATERANKS);

	PxU32 startBit = 0;

	for (PxU32 i = 0; i<nbPasses; ++i)
	{
		const PxU32 descIndex = (i & 1);

		CUdeviceptr rsDesc = mRadixSortDescd[descIndex];

		PxCudaKernelParam radixSortKernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(solverDescd),
			PX_CUDA_KERNEL_PARAM(rsDesc),
			PX_CUDA_KERNEL_PARAM(startBit)
		};

		CUresult  resultR = mCudaContext->launchKernel(radixFunction, PxgKernelGridDim::RADIXSORT, 1, 1, PxgKernelBlockDim::RADIXSORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);

		PX_ASSERT(resultR == CUDA_SUCCESS);

		resultR = mCudaContext->launchKernel(calculateRanksFunction, PxgKernelGridDim::RADIXSORT, 1, 1, PxgKernelBlockDim::RADIXSORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);

		PX_UNUSED(resultR);
		PX_ASSERT(resultR == CUDA_SUCCESS);

		startBit += 4;
	}
}

void PxgTGSCudaSolverCore::accumulatedForceThresholdStream(PxU32 maxNodes)
{
	using namespace physx;

	PX_PROFILE_ZONE("GpuDynamics.AccumulatedForceThresholdStream", 0);

	PxU32 highestBit = PxHighestSetBit(maxNodes) + 1;

	PxU32 nbPasses = (highestBit + 3) / 4;
	if (nbPasses & 1)
		nbPasses++;

	/*PX_UNUSED(maxNodes);
	PxU32 nbPasses = 8;*/

	//copy thresholdstream to tmpThresholdStream
	CUresult result = mCudaContext->memcpyDtoDAsync(reinterpret_cast<CUdeviceptr>(mSolverCoreDesc->tmpThresholdStream), reinterpret_cast<CUdeviceptr>(mSolverCoreDesc->thresholdStream), sizeof(Dy::ThresholdStreamElement) * mTotalContactManagers, mStream);

	PX_UNUSED(result);
	PX_ASSERT(result == CUDA_SUCCESS);

	CUdeviceptr rsDescd = mRadixSortDescd[0];

	PxCudaKernelParam kernelParams0[] =
	{
		PX_CUDA_KERNEL_PARAM(mSolverCoreDescd),
		PX_CUDA_KERNEL_PARAM(rsDescd)
	};

	CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::INITIALIZE_INPUT_AND_RANKS_B);

	result = mCudaContext->launchKernel(kernelFunction, PxgKernelGridDim::INITIALIZE_INPUT_AND_RANKS, 1, 1, PxgKernelBlockDim::INITIALIZE_INPUT_AND_RANKS, 1, 1, 0, mStream, kernelParams0, sizeof(kernelParams0), 0, PX_FL);

	PX_ASSERT(result == CUDA_SUCCESS);

	//radix sort for the bodyIndexB
	{
		radixSort(nbPasses);
	}

	kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::INITIALIZE_INPUT_AND_RANKS_A);
	result = mCudaContext->launchKernel(kernelFunction, PxgKernelGridDim::INITIALIZE_INPUT_AND_RANKS, 1, 1, PxgKernelBlockDim::INITIALIZE_INPUT_AND_RANKS, 1, 1, 0, mStream, kernelParams0, sizeof(kernelParams0), 0, PX_FL);

	PX_ASSERT(result == CUDA_SUCCESS);

	{
		//radix sort for the bodyIndexA
		radixSort(nbPasses);
	}

	//we need to reorganize the threshold stream and put the result in tmpThresholdStream
	kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::REORGANIZE_THRESHOLDSTREAM);
	result = mCudaContext->launchKernel(kernelFunction, PxgKernelGridDim::REORGANIZE_THRESHOLDSTREAM, 1, 1, PxgKernelBlockDim::REORGANIZE_THRESHOLDSTREAM, 1, 1, 0, mStream, kernelParams0, sizeof(kernelParams0), 0, PX_FL);

	PX_ASSERT(result == CUDA_SUCCESS);

	PxCudaKernelParam kernelParams[] =
	{
		PX_CUDA_KERNEL_PARAM(mSolverCoreDescd),
	};

	kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::COMPUTE_ACCUMULATED_THRESHOLDSTREAM);

	result = mCudaContext->launchKernel(kernelFunction, PxgKernelGridDim::COMPUTE_ACCUMULATED_THRESHOLDSTREAM, 1, 1, PxgKernelBlockDim::COMPUTE_ACCUMULATED_THRESHOLDSTREAM, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
	PX_ASSERT(result == CUDA_SUCCESS);

	kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::OUTPUT_ACCUMULATED_THRESHOLDSTREAM);

	result = mCudaContext->launchKernel(kernelFunction, PxgKernelGridDim::OUTPUT_ACCUMULATED_THRESHOLDSTREAM, 1, 1, PxgKernelBlockDim::OUTPUT_ACCUMULATED_THRESHOLDSTREAM, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
	PX_ASSERT(result == CUDA_SUCCESS);

	kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::WRITEOUT_ACCUMULATEDFORCEPEROBJECT);

	result = mCudaContext->launchKernel(kernelFunction, PxgKernelGridDim::WRITEOUT_ACCUMULATEDFORCEPEROBJECT, 1, 1, PxgKernelBlockDim::WRITEOUT_ACCUMULATEDFORCEPEROBJECT, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

	PX_ASSERT(result == CUDA_SUCCESS);

	////copy thresholdstream to tmpThresholdStream
	//result = mCudaContext->memcpyDtoDAsync(reinterpret_cast<CUdeviceptr>(desc->tmpAccumulatedForceObjectPairs), reinterpret_cast<CUdeviceptr>(desc->accumulatedForceObjectPairs), sizeof(PxReal) * desc->numContactBatches * 32, mStream);

	//PX_ASSERT(result == CUDA_SUCCESS);

	//kernelFunction = mKernelWrangler->getCuFunction(PxgKernelIds::CALCULATE_ACCUMULATEDFORCEPEROBJECTPAIRS);

	//result = mCudaContext->launchKernel(kernelFunction, PxgKernelGridDim::CALCULATE_ACCUMULATEDFORCEPEROBJECTPAIRS, 1, 1, PxgKernelBlockDim::CALCULATE_ACCUMULATEDFORCEPEROBJECTPAIRS, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

	//PX_ASSERT(result == CUDA_SUCCESS);

	/*kernelFunction = mKernelWrangler->getCuFunction(PxgKernelIds::WRITEBACK_ACCUMULATEDFORCE);

	result = mCudaContext->launchKernel(kernelFunction, PxgKernelGridDim::WRITEBACK_ACCUMULATEDFORCE, 1, 1, PxgKernelBlockDim::WRITEBACK_ACCUMULATEDFORCE, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

	PX_ASSERT(result == CUDA_SUCCESS);*/

	{
		PxCudaKernelParam exceededForceKernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(mSolverCoreDescd),
			PX_CUDA_KERNEL_PARAM(mSharedDescd),
		};

		kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::COMPUTE_EXCEEDEDFORCE_THRESHOLDELEMENT_INDICE);

		result = mCudaContext->launchKernel(kernelFunction, PxgKernelGridDim::COMPUTE_EXCEEDEDFORCE_THRESHOLDELEMENT_INDICE, 1, 1, PxgKernelBlockDim::COMPUTE_EXCEEDEDFORCE_THRESHOLDELEMENT_INDICE, 1, 1, 0, mStream, exceededForceKernelParams, sizeof(exceededForceKernelParams), 0, PX_FL);

		PX_ASSERT(result == CUDA_SUCCESS);
	}

	kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::OUTPUT_EXCEEDEDFORCE_THRESHOLDELEMENT_INDICE);

	result = mCudaContext->launchKernel(kernelFunction, PxgKernelGridDim::OUTPUT_EXCEEDEDFORCE_THRESHOLDELEMENT_INDICE, 1, 1, PxgKernelBlockDim::OUTPUT_EXCEEDEDFORCE_THRESHOLDELEMENT_INDICE, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

	PX_ASSERT(result == CUDA_SUCCESS);

	////createExceededForceThresholdELEMENT
	//kernelFunction = mKernelWrangler->getCuFunction(PxgKernelIds::CREATE_EXCEEDEDFORCE_THRESHOLDELEMENT);

	//result = mCudaContext->launchKernel(kernelFunction, PxgKernelGridDim::CREATE_EXCEEDEDFORCE_THRESHOLDELEMENT, 1, 1, PxgKernelBlockDim::CREATE_EXCEEDEDFORCE_THRESHOLDELEMENT, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

	//PX_ASSERT(result == CUDA_SUCCESS);

	//initialize all masks to be 1 
	result = mCudaContext->memsetD32Async(mThresholdStreamWriteIndex.getDevicePtr(), 1, (mNbPrevExceededForceElements * 2 + mPrepareDesc->numContactBatches * 32), mStream);

	PX_ASSERT(result == CUDA_SUCCESS);

	if (mNbPrevExceededForceElements > 0)
	{
		//setThresholdPairsMask
		kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::SET_THRESHOLDELEMENT_MASK);
		result = mCudaContext->launchKernel(kernelFunction, PxgKernelGridDim::SET_THRESHOLDELEMENT_MASK, 1, 1, PxgKernelBlockDim::SET_THRESHOLDELEMENT_MASK, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(result == CUDA_SUCCESS);
	}

	//computeThresholdPairsMaskIndices
	kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::COMPUTE_THRESHOLDELEMENT_MASK_INDICES);
	result = mCudaContext->launchKernel(kernelFunction, PxgKernelGridDim::COMPUTE_THRESHOLDELEMENT_MASK_INDICES, 1, 1, PxgKernelBlockDim::COMPUTE_THRESHOLDELEMENT_MASK_INDICES, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
	PX_ASSERT(result == CUDA_SUCCESS);

	//outputThresholdPairsMaskIndices
	kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::OUTPUT_THRESHOLDELEMENT_MASK_INDICES);
	result = mCudaContext->launchKernel(kernelFunction, PxgKernelGridDim::OUTPUT_THRESHOLDELEMENT_MASK_INDICES, 1, 1, PxgKernelBlockDim::OUTPUT_THRESHOLDELEMENT_MASK_INDICES, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
	PX_ASSERT(result == CUDA_SUCCESS);
	//createForceChangeThresholdPairs
	kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CREATE_FORCECHANGE_THRESHOLDELEMENTS);
	result = mCudaContext->launchKernel(kernelFunction, PxgKernelGridDim::CREATE_FORCECHANGE_THRESHOLDELEMENTS, 1, 1, PxgKernelBlockDim::CREATE_FORCECHANGE_THRESHOLDELEMENTS, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
	PX_ASSERT(result == CUDA_SUCCESS);
#if GPU_DEBUG
	result = mCudaContext->streamSynchronize(mStream);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU createForceChangeThresholdElement kernel fail!!\n");
#endif
}

void PxgTGSCudaSolverCore::integrateCoreParallel(const PxU32 offset, const PxU32 nbSolverBodies)
{
	using namespace physx;

	PX_PROFILE_ZONE("GpuDynamics.Integrate", 0);

	const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::INTEGRATE_CORE_PARALLEL_TGS);
	CUdeviceptr islandIds = mIslandIds.getDevicePtr();
	CUdeviceptr islandStaticTouchCounts = mIslandStaticTouchCount.getDevicePtr();
	CUdeviceptr nodeIteractionCounts = mNodeInteractionCounts.getDevicePtr();

	PxCudaKernelParam kernelParams[] =
	{
		PX_CUDA_KERNEL_PARAM(offset),
		PX_CUDA_KERNEL_PARAM(mSolverCoreDescd),
		PX_CUDA_KERNEL_PARAM(mSharedDescd),
		PX_CUDA_KERNEL_PARAM(islandIds),
		PX_CUDA_KERNEL_PARAM(islandStaticTouchCounts),
		PX_CUDA_KERNEL_PARAM(nodeIteractionCounts),
	};

	const PxU32 nbBlocks = (nbSolverBodies - offset + PxgKernelBlockDim::INTEGRATE_CORE_PARALLEL - 1) / PxgKernelBlockDim::INTEGRATE_CORE_PARALLEL;

	if (nbBlocks)
	{
		CUresult result = mCudaContext->launchKernel(kernelFunction, nbBlocks, 1, 1, PxgKernelBlockDim::INTEGRATE_CORE_PARALLEL, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU integrateCoreParallel fail to launch kernel!!\n");

#if GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU DMA back fail 7!!\n");
#endif
	}

	//mCudaContext->streamFlush(mStream);
}

void PxgTGSCudaSolverCore::getDataStreamBase(void*& contactStreamBase, void*& patchStreamBase, void*& forceAndIndexStreamBase)
{
	contactStreamBase = reinterpret_cast<void*>(mCompressedContacts.getDevicePtr());
	patchStreamBase = reinterpret_cast<void*>(mCompressedPatches.getDevicePtr());
	forceAndIndexStreamBase = reinterpret_cast<void*>(mForceBuffer.getDevicePtr());
}

