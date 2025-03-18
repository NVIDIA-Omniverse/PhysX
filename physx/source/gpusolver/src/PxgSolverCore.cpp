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

#include "PxgSolverCore.h"
#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"
#include "PxgSimulationController.h"
#include "PxgSimulationCore.h"
#include "common/PxProfileZone.h"
#include "PxgCudaUtils.h"
#include "PxgKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "CudaKernelWrangler.h"
#include "PxgContext.h"
#include "PxgArticulationCore.h"
#include "PxgSolverKernelIndices.h"
#include "PxgDynamicsConfiguration.h"
#include "PxgFrictionPatch.h"
#include "PxgDynamicsContext.h"
#include "PxgArticulationCoreKernelIndices.h"
#include "DyConstraintPrep.h"
#include "PxgIslandContext.h"

#define GPU_CORE_DEBUG 0

using namespace physx;

PxgRadixSortBuffers::PxgRadixSortBuffers(PxgHeapMemoryAllocatorManager* heapMemoryManager) :
	mInputKeys(heapMemoryManager, PxsHeapStats::eSOLVER),
	mInputRanks(heapMemoryManager, PxsHeapStats::eSOLVER),
	mOutputKeys(heapMemoryManager, PxsHeapStats::eSOLVER),
	mOutputRanks(heapMemoryManager, PxsHeapStats::eSOLVER),
	mRadixCounts(heapMemoryManager, PxsHeapStats::eSOLVER)
{
}

void PxgRadixSortBuffers::constructRadixSortDesc(PxgRadixSortDesc* rsDesc) const
{
	rsDesc[0].inputKeys = reinterpret_cast<PxU32*>(mInputKeys.getDevicePtr());
	rsDesc[0].inputRanks = reinterpret_cast<PxU32*>(mInputRanks.getDevicePtr());
	rsDesc[0].outputKeys = reinterpret_cast<PxU32*>(mOutputKeys.getDevicePtr());
	rsDesc[0].outputRanks = reinterpret_cast<PxU32*>(mOutputRanks.getDevicePtr());
	rsDesc[0].radixBlockCounts = reinterpret_cast<PxU32*>(mRadixCounts.getDevicePtr());

	rsDesc[1].inputKeys = reinterpret_cast<PxU32*>(mOutputKeys.getDevicePtr());
	rsDesc[1].inputRanks = reinterpret_cast<PxU32*>(mOutputRanks.getDevicePtr());
	rsDesc[1].outputKeys = reinterpret_cast<PxU32*>(mInputKeys.getDevicePtr());
	rsDesc[1].outputRanks = reinterpret_cast<PxU32*>(mInputRanks.getDevicePtr());
	rsDesc[1].radixBlockCounts = reinterpret_cast<PxU32*>(mRadixCounts.getDevicePtr());
}

void PxgRadixSortBuffers::allocate(PxU32 totalContactBatches)
{
	mInputKeys.allocate(sizeof(PxU32)*totalContactBatches * 32, PX_FL);
	mInputRanks.allocate(sizeof(PxU32)*totalContactBatches * 32, PX_FL);
	mOutputKeys.allocate(sizeof(PxU32)*totalContactBatches * 32, PX_FL);
	mOutputRanks.allocate(sizeof(PxU32)*totalContactBatches * 32, PX_FL);
	mRadixCounts.allocate(sizeof(PxU32)*32*16, PX_FL);
}

PxgSolverCore::PxgSolverCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager, PxgGpuContext* dynamicContext, PxgHeapMemoryAllocatorManager* heapMemoryManager) :
	mGpuKernelWranglerManager(gpuKernelWrangler),
	mCudaContextManager(cudaContextManager),
	mCudaContext(cudaContextManager->getCudaContext()),
	mGpuContext(dynamicContext), 
	mHeapMemoryManager(heapMemoryManager),
	mSolverCoreDesc(NULL),
	mPrepareDesc(NULL),
	mPrePrepDesc(NULL),
	mRsDesc(NULL),
	mNbStaticRigidSlabs(0),
	mMaxNumStaticPartitions(0),
	mTotalContactManagers(0),
	mNbPrevExceededForceElements(0),
	mNbArticSlabs(0),
	mContactHeaderBlockStream(heapMemoryManager, PxsHeapStats::eSOLVER),
	mFrictionHeaderBlockStream(heapMemoryManager, PxsHeapStats::eSOLVER),
	mContactBlockStream(heapMemoryManager, PxsHeapStats::eSOLVER),
	mFrictionBlockStream(heapMemoryManager, PxsHeapStats::eSOLVER),
	mJointHeaderBlockStream(heapMemoryManager, PxsHeapStats::eSOLVER),
	mJointRowBlockStreamCon(heapMemoryManager, PxsHeapStats::eSOLVER),
	mJointRowBlockStreamMod(heapMemoryManager, PxsHeapStats::eSOLVER),
	mConstraintContactPrepBlockPool(heapMemoryManager, PxsHeapStats::eSOLVER),
	mConstraint1DPrepBlockPool(heapMemoryManager, PxsHeapStats::eSOLVER),
	mConstraint1DPrepBlockPoolVel(heapMemoryManager, PxsHeapStats::eSOLVER),
	mConstraint1DPrepBlockPoolPar(heapMemoryManager, PxsHeapStats::eSOLVER),
	mConstraintDataPool(heapMemoryManager, PxsHeapStats::eSOLVER),
	mConstraintRowPool(heapMemoryManager, PxsHeapStats::eSOLVER),
	mArtiConstraintDataPool(heapMemoryManager, PxsHeapStats::eSOLVER),
	mArtiConstraintRowPool(heapMemoryManager, PxsHeapStats::eSOLVER),
	mSolverBodyPool(heapMemoryManager, PxsHeapStats::eSOLVER),
	mTempStaticBodyOutputPool(heapMemoryManager, PxsHeapStats::eSOLVER),
	mIslandNodeIndices2(heapMemoryManager, PxsHeapStats::eSOLVER),
	mSolverBodyIndices(heapMemoryManager, PxsHeapStats::eSOLVER),
	mOutVelocityPool(heapMemoryManager, PxsHeapStats::eSOLVER),
	mOutBody2WorldPool(heapMemoryManager, PxsHeapStats::eSOLVER),
	mSolverBodyDataPool(heapMemoryManager, PxsHeapStats::eSOLVER),
	mSolverBodySleepDataPool(heapMemoryManager, PxsHeapStats::eSOLVER),
	mOutArtiVelocityPool(heapMemoryManager, PxsHeapStats::eSOLVER),
	mSolverTxIDataPool(heapMemoryManager, PxsHeapStats::eSOLVER),
	mConstraintsPerPartition(heapMemoryManager, PxsHeapStats::eSOLVER),
	mArtiConstraintsPerPartition(heapMemoryManager, PxsHeapStats::eSOLVER),
	mMotionVelocityArray(heapMemoryManager, PxsHeapStats::eSOLVER),
	mBlockConstraintBatches(heapMemoryManager, PxsHeapStats::eSOLVER),
	mArtiOrderedStaticConstraints(heapMemoryManager, PxsHeapStats::eSOLVER),
	mArtiOrderedStaticContacts(heapMemoryManager, PxsHeapStats::eSOLVER),
	mSolverBodyReferences(heapMemoryManager, PxsHeapStats::eSOLVER),
	mBlockWorkUnits(heapMemoryManager, PxsHeapStats::eSOLVER),
	mPartitionIndexData(heapMemoryManager, PxsHeapStats::eSOLVER),
	mPartitionNodeData(heapMemoryManager, PxsHeapStats::eSOLVER),
	mSolverConstantData(heapMemoryManager, PxsHeapStats::eSOLVER),
	mPartitionStartBatchIndices(heapMemoryManager, PxsHeapStats::eSOLVER),
	mPartitionArticulationStartBatchIndices(heapMemoryManager, PxsHeapStats::eSOLVER),
	mPartitionJointBatchCounts(heapMemoryManager, PxsHeapStats::eSOLVER),
	mPartitionArtiJointBatchCounts(heapMemoryManager, PxsHeapStats::eSOLVER),
	mDestroyedEdgeIndices(heapMemoryManager, PxsHeapStats::eSOLVER),
	mNpIndexArray(heapMemoryManager, PxsHeapStats::eSOLVER),
	mGpuContactBlockBuffer(heapMemoryManager, PxsHeapStats::eSOLVER),
	mDataBuffer(heapMemoryManager, PxsHeapStats::eSOLVER),
	mCompressedContacts(heapMemoryManager, PxsHeapStats::eSOLVER),
	mCompressedPatches(heapMemoryManager, PxsHeapStats::eSOLVER),
	mConstraintWriteBackBuffer(heapMemoryManager, PxsHeapStats::eSOLVER),
	mForceBuffer(heapMemoryManager, PxsHeapStats::eSOLVER),
	mFrictionPatches(heapMemoryManager, PxsHeapStats::eSOLVER),
	mArtiStaticContactIndices(heapMemoryManager, PxsHeapStats::eSOLVER),
	mArtiStaticJointIndices(heapMemoryManager, PxsHeapStats::eSOLVER),
	mArtiStaticContactCounts(heapMemoryManager, PxsHeapStats::eSOLVER),
	mArtiStaticJointCounts(heapMemoryManager, PxsHeapStats::eSOLVER),
	mRigidStaticContactIndices(heapMemoryManager, PxsHeapStats::eSOLVER),
	mRigidStaticJointIndices(heapMemoryManager, PxsHeapStats::eSOLVER),
	mRigidStaticContactCounts(heapMemoryManager, PxsHeapStats::eSOLVER),
	mRigidStaticJointCounts(heapMemoryManager, PxsHeapStats::eSOLVER),
	mRigidStaticContactStartIndices(heapMemoryManager, PxsHeapStats::eSOLVER),
	mRigidStaticJointStartIndices(heapMemoryManager, PxsHeapStats::eSOLVER),
	mTempContactUniqueIndicesBlockBuffer(heapMemoryManager, PxsHeapStats::eSOLVER),
	mTempConstraintUniqueIndicesBlockBuffer(heapMemoryManager, PxsHeapStats::eSOLVER),
	mTempContactHeaderBlockBuffer(heapMemoryManager, PxsHeapStats::eSOLVER),
	mTempConstraintHeaderBlockBuffer(heapMemoryManager, PxsHeapStats::eSOLVER),
	mArtiSelfContactIndices(heapMemoryManager, PxsHeapStats::eSOLVER),
	mArtiSelfJointIndices(heapMemoryManager, PxsHeapStats::eSOLVER),
	mArtiSelfContactCounts(heapMemoryManager, PxsHeapStats::eSOLVER),
	mArtiSelfJointCounts(heapMemoryManager, PxsHeapStats::eSOLVER),
	mNodeInteractionCounts(heapMemoryManager, PxsHeapStats::eSOLVER),
	mFrictionPatchBlockStream(heapMemoryManager, PxsHeapStats::eSOLVER),
	mFrictionAnchorPatchBlockStream(heapMemoryManager, PxsHeapStats::eSOLVER),
	mFrictionIndexStream(heapMemoryManager, PxsHeapStats::eSOLVER),
	mFrictionPatchCounts(heapMemoryManager, PxsHeapStats::eSOLVER),
	mFrictionPatchStream(heapMemoryManager, PxsHeapStats::eSOLVER),
	mFrictionAnchorPatchStream(heapMemoryManager, PxsHeapStats::eSOLVER),
	mCurrentIndex(0),
	mPinnedEvent(NULL),
	mCpuIslandNodeIndices(NULL),
	mSolverBodyOutputVelocityOffset(0),
	mRadixSort(heapMemoryManager)
{}

// These two structures must have the same layout
PX_COMPILE_TIME_ASSERT(sizeof(PxgFrictionPatchGPU) == sizeof(PxFrictionPatch));

void PxgSolverCore::allocateFrictionPatchStream(PxI32 numContactBatches, PxI32 numArtiContactBatches)
{
	mFrictionPatchBlockStream[mCurrentIndex].allocate(sizeof(PxgBlockFrictionPatch) * (numContactBatches + numArtiContactBatches), PX_FL);
	mFrictionAnchorPatchBlockStream[mCurrentIndex].allocate(sizeof(PxgBlockFrictionAnchorPatch) * (numContactBatches + numArtiContactBatches), PX_FL);

	/*frictionPatchStream[currentIndex].allocate(sizeof(PxgFrictionPatch) * numArtiContactBatches);
	frictionAnchorPatchStream[currentIndex].allocate(sizeof(PxgFrictionAnchorPatch) * numArtiContactBatches);*/
}

void PxgSolverCore::allocateFrictionCounts(PxU32 totalEdges)
{
	mFrictionPatchCounts[1 - mCurrentIndex].allocateCopyOldDataAsync(totalEdges * sizeof(PxU32), mCudaContext, mStream, PX_FL);
	mFrictionPatchCounts[mCurrentIndex].allocate(totalEdges * sizeof(PxU32), PX_FL);
}

PxgBlockFrictionIndex* PxgSolverCore::allocateFrictionPatchIndexStream(PxU32 totalFrictionPatchCount)
{
	mFrictionIndexStream[mCurrentIndex].allocateCopyOldDataAsync(sizeof(PxgBlockFrictionIndex) * totalFrictionPatchCount, mCudaContext, mStream, PX_FL);
	return reinterpret_cast<PxgBlockFrictionIndex*>(mFrictionIndexStream[mCurrentIndex].getDevicePtr());
}

void PxgSolverCore::allocateNodeInteractionCounts(PxU32 nbNodes)
{
	mNodeInteractionCounts.allocate(nbNodes * sizeof(PxU32), PX_FL);
}

void PxgSolverCore::uploadNodeInteractionCounts(const PxU32* nodeInteractionCounts, PxU32 nbNodes)
{
	mCudaContext->memcpyHtoDAsync(mNodeInteractionCounts.getDevicePtr(), nodeInteractionCounts, sizeof(PxU32) * nbNodes, mStream);
}

void PxgSolverCore::gpuMemDMAbackSolverBodies(float4* solverBodyPool, PxU32 nbSolverBodies,
	PxPinnedArray<PxAlignedTransform>& body2WorldPool,
	PxPinnedArray<PxgSolverBodySleepData>& solverBodySleepDataPool,
	const bool enableDirectGPUAPI)
{
	PX_PROFILE_ZONE("GpuDynamics.DMABackBodies", 0);

	if (!enableDirectGPUAPI)
	{
		mCudaContext->memcpyDtoHAsync(solverBodyPool, mOutVelocityPool.getDevicePtr(), sizeof(PxgSolverBody) * nbSolverBodies, mStream);
		mCudaContext->memcpyDtoHAsync(body2WorldPool.begin(), mOutBody2WorldPool.getDevicePtr(), sizeof(PxAlignedTransform) * nbSolverBodies, mStream);
		mCudaContext->memcpyDtoHAsync(solverBodySleepDataPool.begin(), mSolverBodySleepDataPool.getDevicePtr(), sizeof(PxgSolverBodySleepData) * nbSolverBodies, mStream);
	}

	synchronizeStreams(mCudaContext, mStream2, mStream, mIntegrateEvent);

	CUfunction signalFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::BP_SIGNAL_COMPLETE);

	*mPinnedEvent = 0;

	void* devicePtr = getMappedDevicePtr(mCudaContext, mPinnedEvent);
	PxCudaKernelParam signalParams[] =
	{
		PX_CUDA_KERNEL_PARAM(devicePtr)
	};

	mCudaContext->launchKernel(signalFunction, 1, 1, 1, 1, 1, 1, 0, mStream, signalParams, sizeof(signalParams), 0, PX_FL);
}

void PxgSolverCore::allocateSolverBodyBuffersCommon(PxU32 numSolverBodies, PxPinnedArray<PxNodeIndex>& islandNodeIndices)
{
	mMotionVelocityArray.allocate(sizeof(float4) * numSolverBodies * 2, PX_FL);

	mSolverBodyIndices.allocate(sizeof(PxU32) * numSolverBodies, PX_FL);

	//allocate enough solver body data space(static + kinematic + dynamic), but we just need to dma static and kinematic solver body and preIntegration kernel will
	//fill in dynamic solver body data
	mSolverBodyDataPool.allocate(sizeof(PxgSolverBodyData) * numSolverBodies, PX_FL);
	mSolverBodySleepDataPool.allocate(sizeof(PxgSolverBodySleepData) * numSolverBodies, PX_FL);
	mSolverTxIDataPool.allocate(sizeof(PxgSolverTxIData) * numSolverBodies, PX_FL);
	mOutVelocityPool.allocate(sizeof(float4)*numSolverBodies * 2, PX_FL); //Output buffer to read back solver body velocities
	mOutBody2WorldPool.allocate(sizeof(PxAlignedTransform)*numSolverBodies, PX_FL); //output buffer to read back solver body transform

	//allocate enough memory for numArticulations * maxLinks
	//mOutArtiVelocityPool.allocate(sizeof(float4)*numActiveActiculations*maxArticulationLinks * 2);
	
	mIslandNodeIndices2.allocate(sizeof(PxNodeIndex) * islandNodeIndices.size(), PX_FL);

	mCudaContext->memsetD32Async(mSolverBodyIndices.getDevicePtr(), 0xFFffFFff, numSolverBodies, mStream);
	mCudaContext->memcpyHtoDAsync(mIslandNodeIndices2.getDevicePtr(), islandNodeIndices.begin(), sizeof(PxNodeIndex) *islandNodeIndices.size(), mStream);

	synchronizeStreams(mCudaContext, mStream, mGpuContext->getArticulationCore()->getStream());

	mCpuIslandNodeIndices = islandNodeIndices.begin();
}

void PxgSolverCore::constructConstraintPrePrepDesc(PxgPrePrepDesc& preDesc, PxU32 numBatches, PxU32 numStaticBatches, PxU32 numArtiBatches, PxU32 numArtiStaticBatches,
	PxU32 numArtiSelfBatches, const PxgPartitionData& pData, PxContact* cpuCompressedContactsBase, PxContactPatch* cpuCompressedPatchesBase, PxReal* cpuForceBufferBase,
	PxU32 nbD6RigidJoint, PxU32 nbD6ArtiJoint, PxU32 nbTotalArtiJoints,
	PxsContactManagerOutputIterator& outputIterator, PxU32 maxConstraintPartitions, PxU32 totalActiveBodies, PxU32 nbArticulations,
	PxU32 activeBodyStartOffset, Sc::ShapeInteraction** shapeInteractions, PxReal* restDistances, PxsTorsionalFrictionData* torsionalData,
	PxU32 nbElementsPerBody, PxU32 numSlabs)
{
	preDesc.blockBatches = reinterpret_cast<PxgBlockConstraintBatch*>(mBlockConstraintBatches.getDevicePtr());
	preDesc.numBatches = numBatches;
	preDesc.numStaticBatches = numStaticBatches;
	preDesc.numArtiBatches = numArtiBatches;
	preDesc.numArtiStaticBatches = numArtiStaticBatches; //this is just estimation. we write the actually numArticStaticBatches in artiSumInternalContactAndJointBatches2
	preDesc.numArtiSelfBatches = numArtiSelfBatches; //this is also just an estimation.
	preDesc.blockWorkUnit = reinterpret_cast<PxgBlockWorkUnit*>(mBlockWorkUnits.getDevicePtr());

	preDesc.numTotalContacts = pData.numTotalContacts;
	preDesc.numTotalConstraints = pData.numTotalConstraints;
	preDesc.numTotalStaticConstraints = pData.numTotalStaticConstraints;
	preDesc.numTotalStaticContacts = pData.numTotalStaticContacts;

	preDesc.numTotalArtiContacts = pData.numTotalArtiContacts;
	preDesc.numTotalArtiConstraints = pData.numTotalArtiConstraints;
	preDesc.numTotalStaticArtiContacts = pData.numTotalArtiStaticContacts;
	preDesc.numTotalStaticArtiConstraints = pData.numTotalArtiStaticConstraints;
	preDesc.numTotalSelfArtiContacts = pData.numTotalArtiSelfContacts;
	preDesc.numTotalSelfArtiConstraints = pData.numTotalArtiSelfConstraints;

	preDesc.artiStaticConstraintBatchOffset = pData.artiStaticConstraintBatchOffset;
	preDesc.artiStaticContactBatchOffset = pData.artiStaticContactBatchOffset;

	preDesc.blockContactData = reinterpret_cast<PxgBlockContactData*>(mConstraintContactPrepBlockPool.getDevicePtr());
	preDesc.blockContactPoints = reinterpret_cast<PxgBlockContactPoint*>(mGpuContactBlockBuffer.getDevicePtr());
	preDesc.compressedContacts = reinterpret_cast<PxContact*>(mCompressedContacts.getDevicePtr());
	preDesc.compressedPatches = reinterpret_cast<PxContactPatch*>(mCompressedPatches.getDevicePtr());
	preDesc.forceBuffer = reinterpret_cast<PxU8*>(mForceBuffer.getDevicePtr());

	preDesc.sharedJointRowIndex = 0;
	preDesc.nbD6RigidJoints = nbD6RigidJoint;
	preDesc.nbD6ArtiJoints = nbD6ArtiJoint;
	preDesc.nbTotalArtiJoints = nbTotalArtiJoints;

	preDesc.blockPrepData = reinterpret_cast<PxgBlockConstraint1DData*>(mConstraint1DPrepBlockPool.getDevicePtr());
	preDesc.blockPrepVelocityData = reinterpret_cast<PxgBlockConstraint1DVelocities*>(mConstraint1DPrepBlockPoolVel.getDevicePtr());
	preDesc.blockPrepParameterData = reinterpret_cast<PxgBlockConstraint1DParameters*>(mConstraint1DPrepBlockPoolPar.getDevicePtr());

	//this is the first pass of constraint 1D data which filled in by the GPU for D6 joint. After that, if we have other joint type, which is filled in by CPU, we need to append
	//the CPU result in this buffer and do the second pass of data filling for the block format in GPU
	preDesc.constraintData = reinterpret_cast<PxgConstraintData*>(mConstraintDataPool.getDevicePtr());
	preDesc.constraintRows = reinterpret_cast<Px1DConstraint*>(mConstraintRowPool.getDevicePtr());

	preDesc.artiConstraintData = reinterpret_cast<PxgConstraintData*>(mArtiConstraintDataPool.getDevicePtr());
	preDesc.artiConstraintRows = reinterpret_cast<Px1DConstraint*>(mArtiConstraintRowPool.getDevicePtr());

	PxgSimulationCore* simCore = mGpuContext->getSimulationCore();
	preDesc.rigidJointData = reinterpret_cast<PxgD6JointData*>(simCore->getD6RigidJointBuffer().getDevicePtr());//reinterpret_cast<PxgD6JointData*>(mD6JointDataPool.getDevicePtr(0)); 
	preDesc.rigidConstraintPrePrep = reinterpret_cast<PxgConstraintPrePrep*>(simCore->getD6RigidJointPrePreBuffer().getDevicePtr());//reinterpret_cast<PxgConstraintPrePrep*>(mD6JointPrePrepPool.getDevicePtr(0));

	preDesc.artiJointData = reinterpret_cast<PxgD6JointData*>(simCore->getD6ArtiJointBuffer().getDevicePtr());
	preDesc.artiConstraintPrePrep = reinterpret_cast<PxgConstraintPrePrep*>(simCore->getD6ArtiJointPrePreBuffer().getDevicePtr());

	preDesc.cpuCompressedContactsBase = cpuCompressedContactsBase;
	preDesc.cpuCompressedPatchesBase = cpuCompressedPatchesBase;
	preDesc.cpuForceBufferBase = cpuForceBufferBase;

	preDesc.contactManagerOutputBase = reinterpret_cast<PxsContactManagerOutput*>(mGpuContactManagerOutputBase);
	preDesc.sharedFrictionConstraintIndex = 0;
	preDesc.sharedContactConstraintIndex = 0;	
	preDesc.sharedArticulationResponseIndex = 0;
	preDesc.solverBodyIndices = reinterpret_cast<PxU32*>(mSolverBodyIndices.getDevicePtr());

	preDesc.mPartitionIndices = reinterpret_cast<PartitionIndexData*>(mPartitionIndexData.getDevicePtr());
	preDesc.mPartitionstartBatchIndices = reinterpret_cast<PxU32*>(mPartitionStartBatchIndices.getDevicePtr());
	preDesc.mPartitionArtiStartBatchIndices = reinterpret_cast<PxU32*>(mPartitionArticulationStartBatchIndices.getDevicePtr());
	preDesc.mPartitionJointCounts = reinterpret_cast<PxU32*>(mPartitionJointBatchCounts.getDevicePtr());
	preDesc.mPartitionArtiJointCounts = reinterpret_cast<PxU32*>(mPartitionArtiJointBatchCounts.getDevicePtr());
	preDesc.currFrictionPatchCount = reinterpret_cast<PxU32*>(mFrictionPatchCounts[mCurrentIndex].getDevicePtr());
	preDesc.prevFrictionPatchCount = reinterpret_cast<PxU32*>(mFrictionPatchCounts[1 - mCurrentIndex].getDevicePtr());

	preDesc.mNpOutputIndices = reinterpret_cast<PxU32*>(mNpIndexArray.getDevicePtr());

	for (PxU32 i = 0; i < GPU_BUCKET_ID::eCount; ++i)
		preDesc.mCmOutputOffsets[i] = outputIterator.getIndex(i + GPU_BUCKET_ID::eCount);

	preDesc.mSolverBodyData = reinterpret_cast<PxgSolverBodyData*>(mSolverBodyDataPool.getDevicePtr());
	preDesc.mPartitionNodeData = reinterpret_cast<PartitionNodeData*>(mPartitionNodeData.getDevicePtr());

	preDesc.mContactConstantData = reinterpret_cast<PxgSolverConstraintManagerConstants*>(mSolverConstantData.getDevicePtr());

	preDesc.mBatchHeaders = reinterpret_cast<PxgConstraintBatchHeader*>(mConstraintBatchHeaders);
	preDesc.mContactUniqueIndices = reinterpret_cast<PxU32*>(mContactUniqueIndices);
	preDesc.mConstraintUniqueIndices = reinterpret_cast<PxU32*>(mConstraintUniqueIndices);
	preDesc.mArtiConstraintUniqueIndices = reinterpret_cast<PxU32*>(mArtiConstraintUniqueIndices);
	preDesc.mArtiContactUniqueIndices = reinterpret_cast<PxU32*>(mArtiContactUniqueIndices);

	preDesc.mSolverBodyReferences = reinterpret_cast<PxgSolverReferences*>(mSolverBodyReferences.getDevicePtr());
	preDesc.mMaxConstraintPartitions = maxConstraintPartitions;
	preDesc.mTotalSlabs = numSlabs;
	preDesc.mTotalActiveBodies = totalActiveBodies;
	preDesc.mTotalActiveArticulations = nbArticulations;
	preDesc.mActiveBodyStartOffset = activeBodyStartOffset;
	preDesc.nbElementsPerBody = nbElementsPerBody;
	preDesc.mRestDistances = restDistances;
	preDesc.mTorsionalFrictionData = torsionalData;
	preDesc.mShapeInteractions = shapeInteractions;

	preDesc.mArtiStaticContactIndices = reinterpret_cast<PxU32*>(mArtiStaticContactIndices.getDevicePtr());
	preDesc.mArtiStaticConstraintIndices = reinterpret_cast<PxU32*>(mArtiStaticJointIndices.getDevicePtr());
	preDesc.mArtiStaticContactCounts = reinterpret_cast<PxU32*>(mArtiStaticContactCounts.getDevicePtr());
	preDesc.mArtiStaticConstraintCounts = reinterpret_cast<PxU32*>(mArtiStaticJointCounts.getDevicePtr());

	preDesc.mArtiSelfContactIndices = reinterpret_cast<PxU32*>(mArtiSelfContactIndices.getDevicePtr());
	preDesc.mArtiSelfConstraintIndices = reinterpret_cast<PxU32*>(mArtiSelfJointIndices.getDevicePtr());
	preDesc.mArtiSelfContactCounts = reinterpret_cast<PxU32*>(mArtiSelfContactCounts.getDevicePtr());
	preDesc.mArtiSelfConstraintCounts = reinterpret_cast<PxU32*>(mArtiSelfJointCounts.getDevicePtr());

	preDesc.mRigidStaticContactIndices = reinterpret_cast<PxU32*>(mRigidStaticContactIndices.getDevicePtr());
	preDesc.mRigidStaticConstraintIndices = reinterpret_cast<PxU32*>(mRigidStaticJointIndices.getDevicePtr());
	preDesc.mRigidStaticContactCounts = reinterpret_cast<PxU32*>(mRigidStaticContactCounts.getDevicePtr());
	preDesc.mRigidStaticConstraintCounts = reinterpret_cast<PxU32*>(mRigidStaticJointCounts.getDevicePtr());

	preDesc.mRigidStaticContactStartIndices = reinterpret_cast<PxU32*>(mRigidStaticContactStartIndices.getDevicePtr());
	preDesc.mRigidStaticConstraintStartIndices = reinterpret_cast<PxU32*>(mRigidStaticJointStartIndices.getDevicePtr());

	preDesc.mTempContactUniqueIndices = reinterpret_cast<PxU32*>(mTempContactUniqueIndicesBlockBuffer.getDevicePtr());
	preDesc.mTempConstraintUniqueIndices = reinterpret_cast<PxU32*>(mTempConstraintUniqueIndicesBlockBuffer.getDevicePtr());
	preDesc.mTempContactBlockHeader = reinterpret_cast<PxU32*>(mTempContactHeaderBlockBuffer.getDevicePtr());
	preDesc.mTempConstraintBlockHeader = reinterpret_cast<PxU32*>(mTempConstraintHeaderBlockBuffer.getDevicePtr());
}

void PxgSolverCore::constructSolverSharedDescCommon(PxgSolverSharedDescBase& sharedDesc, const PxgConstantData& cData,
	Cm::UnAlignedSpatialVector* deferredZ, PxU32* articulationDirty, uint4* articulationSlabMask)
{
	sharedDesc.dt = cData.dt;
	sharedDesc.invDtF32 = cData.invDtF32;

	sharedDesc.blockCurrentFrictionPatches = reinterpret_cast<PxgBlockFrictionPatch*>(mFrictionPatchBlockStream[mCurrentIndex].getDevicePtr());
	sharedDesc.blockPreviousFrictionPatches = reinterpret_cast<PxgBlockFrictionPatch*>(mFrictionPatchBlockStream[1 - mCurrentIndex].getDevicePtr());

	sharedDesc.currentFrictionPatches = reinterpret_cast<PxgFrictionPatch*>(mFrictionPatchStream[mCurrentIndex].getDevicePtr());
	sharedDesc.previousFrictionPatches = reinterpret_cast<PxgFrictionPatch*>(mFrictionPatchStream[1 - mCurrentIndex].getDevicePtr());

	PxgSimulationCore* core = mGpuContext->getSimulationCore();

	sharedDesc.mBodySimBufferDeviceData = core->getBodySimBufferDeviceData().getPointer();
	sharedDesc.articulations = reinterpret_cast<PxgArticulation*>(core->getArticulationBuffer().getDevicePtr());
	sharedDesc.articulationDeferredZ = deferredZ;
	sharedDesc.articulationDirty = articulationDirty;
	sharedDesc.articulationSlabMask = articulationSlabMask;

	sharedDesc.deltaOutOffset = mSolverBodyOutputVelocityOffset;
}

// PT: I don't understand the existing code. We already have a constructSolverSharedDescCommon function above, working on a
// PxgSolverSharedDescBase structure. But there is still plenty of "constructSolverDesc" code that could be shared between
// PGS and TGS when we initialize PxgSolverCoreDesc (which doesn't inherit from PxgSolverSharedDescBase). I just started moving
// that shared code here, without touching the other bits.
void PxgSolverCore::constructSolverDesc(PxgSolverCoreDesc& scDesc, PxU32 numIslands, PxU32 numSolverBodies, PxU32 numConstraintBatchHeader, PxU32 numArticConstraints, PxU32 numSlabs, bool enableStabilization)
{
	CUdeviceptr islandContextPoold = mIslandContextPool;//mIslandContextPool.getDevicePtr(0);
	CUdeviceptr motionVelocityArrayd = mMotionVelocityArray.getDevicePtr();
	CUdeviceptr constraintsPerPartitiond = mConstraintsPerPartition.getDevicePtr();

	scDesc.outSolverVelocity = reinterpret_cast<float4*>(mOutVelocityPool.getDevicePtr());
	scDesc.outBody2World = reinterpret_cast<PxAlignedTransform*>(mOutBody2WorldPool.getDevicePtr());
	scDesc.solverBodyDataPool = reinterpret_cast<PxgSolverBodyData*>(mSolverBodyDataPool.getDevicePtr());
	scDesc.solverBodyTxIDataPool = reinterpret_cast<PxgSolverTxIData*>(mSolverTxIDataPool.getDevicePtr());
	scDesc.solverBodySleepDataPool = reinterpret_cast<PxgSolverBodySleepData*>(mSolverBodySleepDataPool.getDevicePtr());

	scDesc.outArtiVelocity = reinterpret_cast<float4*>(mOutArtiVelocityPool.getDevicePtr());
		
	scDesc.constraintWriteBack = reinterpret_cast<PxgConstraintWriteback*>(mConstraintWriteBackBuffer.getDevicePtr());
	scDesc.forceBuffer = reinterpret_cast<PxF32*>(mForceBuffer.getDevicePtr());
	scDesc.frictionPatches = reinterpret_cast<PxFrictionPatch*>(mFrictionPatches.getDevicePtr());

	scDesc.solverBodyReferences = reinterpret_cast<PxgSolverReferences*>(mSolverBodyReferences.getDevicePtr());

	scDesc.contactManagerOutputBase = reinterpret_cast<PxsContactManagerOutput*>(mGpuContactManagerOutputBase);

	scDesc.islandContextPool = reinterpret_cast<PxgIslandContext*>(islandContextPoold);
	scDesc.motionVelocityArray = reinterpret_cast<float4*>(motionVelocityArrayd);
	scDesc.constraintsPerPartition = reinterpret_cast<PxU32*>(constraintsPerPartitiond);
	scDesc.artiConstraintsPerPartition = reinterpret_cast<PxU32*>(mArtiConstraintsPerPartition.getDevicePtr());
	PxgSimulationCore* simulationCore = mGpuContext->getSimulationCore(); 
	scDesc.mBodySimBufferDeviceData = simulationCore->getBodySimBufferDeviceData().getPointer();
	scDesc.mBodySimPrevVelocitiesBufferDeviceData = simulationCore->getBodySimPrevVelocitiesBufferDeviceData().getPointer();

	scDesc.mRigidStaticContactCounts = reinterpret_cast<PxU32*>(mRigidStaticContactCounts.getDevicePtr());
	scDesc.mRigidStaticContactStartIndices = reinterpret_cast<PxU32*>(mRigidStaticContactStartIndices.getDevicePtr());

	scDesc.mRigidStaticJointCounts = reinterpret_cast<PxU32*>(mRigidStaticJointCounts.getDevicePtr());
	scDesc.mRigidStaticJointStartIndices = reinterpret_cast<PxU32*>(mRigidStaticJointStartIndices.getDevicePtr());

	scDesc.numIslands = numIslands;
	scDesc.numSolverBodies = numSolverBodies;
	scDesc.numBatches = numConstraintBatchHeader;
	scDesc.numArticBatches = numArticConstraints;

	scDesc.accumulatedBodyDeltaVOffset = mSolverBodyOutputVelocityOffset;

	scDesc.numSlabs = numSlabs;
	scDesc.maxLinksPerArticulation = mGpuContext->getSimulationCore()->getMaxArticulationLinks();

	scDesc.sharedThresholdStreamIndex = 0;
	scDesc.nbForceChangeElements = 0;
	scDesc.nbExceededThresholdElements = 0;
	scDesc.nbPrevExceededThresholdElements = mNbPrevExceededForceElements;
	scDesc.enableStabilization = enableStabilization;
}

void PxgSolverCore::gpuMemDMAUpJointData(const PxPinnedArray<PxgConstraintData>& cpuJointDataPool, const PxPinnedArray<Px1DConstraint>& cpuJointRowPool,
	PxU32 nbCpuJoints, PxU32 nbGpuJoints, PxU32 totalCpuRows)
{
	CUdeviceptr startPtr = mConstraintDataPool.getDevicePtr() + nbGpuJoints * sizeof(PxgConstraintData);
	CUdeviceptr startRowPtr = mConstraintRowPool.getDevicePtr() + nbGpuJoints * sizeof(Px1DConstraint)*Dy::MAX_CONSTRAINT_ROWS;
	mCudaContext->memcpyHtoDAsync(startPtr, cpuJointDataPool.begin(), nbCpuJoints * sizeof(PxgConstraintData), mStream);
	mCudaContext->memcpyHtoDAsync(startRowPtr, cpuJointRowPool.begin(), totalCpuRows * sizeof(Px1DConstraint), mStream);

#if GPU_CORE_DEBUG
	CUresult result = mCudaContext->streamSynchronize(mStream);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU DMA up cpu joint data fail!!\n");
#endif
}

void PxgSolverCore::gpuMemDMAUpArtiJointData(const PxPinnedArray<PxgConstraintData>& cpuArtiJointDataPool, const PxPinnedArray<Px1DConstraint>& cpuArtiJointRowPool,
	PxU32 nbCpuArtiJoints, PxU32 nbGpuArtiJoints, PxU32 totalArtiRows)
{
	CUdeviceptr startPtr = mArtiConstraintDataPool.getDevicePtr() + nbGpuArtiJoints * sizeof(PxgConstraintData);
	CUdeviceptr startRowPtr = mArtiConstraintRowPool.getDevicePtr() + nbGpuArtiJoints * sizeof(Px1DConstraint)*Dy::MAX_CONSTRAINT_ROWS;

	mCudaContext->memcpyHtoDAsync(startPtr, cpuArtiJointDataPool.begin(), nbCpuArtiJoints * sizeof(PxgConstraintData), mStream);
	mCudaContext->memcpyHtoDAsync(startRowPtr, cpuArtiJointRowPool.begin(), totalArtiRows * sizeof(Px1DConstraint), mStream);

#if GPU_CORE_DEBUG
	CUresult result = mCudaContext->streamSynchronize(mStream);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU DMA up articulation joint data fail!!\n");
#endif
}

void PxgSolverCore::constraintPrePrepParallel(PxU32 nbConstraintBatches, PxU32 nbD6Joints, PxU32 numBodies)
{
	PX_PROFILE_ZONE("GpuDynamics.ConstraintPrePrepParallel", 0);

	///////////////////////////////////////
	//New step here!!!
	//We need to prep up the static rigid body contact buffers prior to contact pre-prep

	{
		const CUfunction staticKernel1 = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RIGID_SUM_STATIC_CONTACT1);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(mPrePrepDescd),
			PX_CUDA_KERNEL_PARAM(numBodies)
		};

		const PxU32 nbBlocksRequired = 32;

		CUresult launchResult = mCudaContext->launchKernel(staticKernel1, nbBlocksRequired, 1, 1, PxgKernelBlockDim::COMPUTE_STATIC_CONTACT_CONSTRAINT_COUNT, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(launchResult == CUDA_SUCCESS);
		PX_UNUSED(launchResult);

#if GPU_CORE_DEBUG
		CUresult result = mCudaContext->streamSynchronize(mStream);
		PX_ASSERT(result == CUDA_SUCCESS);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU rigidSumInternalContactAndJointBatches1 kernel fail!\n");
#endif
	}

	{
		const CUfunction staticKernel2 = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RIGID_SUM_STATIC_CONTACT2);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(mPrePrepDescd),
			PX_CUDA_KERNEL_PARAM(mSolverCoreDescd),
			PX_CUDA_KERNEL_PARAM(mPrepareDescd),
			PX_CUDA_KERNEL_PARAM(numBodies)
		};

		const PxU32 nbBlocksRequired = 32;

		CUresult launchResult = mCudaContext->launchKernel(staticKernel2, nbBlocksRequired, 1, 1, PxgKernelBlockDim::COMPUTE_STATIC_CONTACT_CONSTRAINT_COUNT, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(launchResult == CUDA_SUCCESS);
		PX_UNUSED(launchResult);

#if GPU_CORE_DEBUG
		CUresult result = mCudaContext->streamSynchronize(mStream);
		PX_ASSERT(result == CUDA_SUCCESS);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU rigidSumInternalContactAndJointBatches1 kernel fail!\n");
#endif
	}

	//////////////////////////////////////

	CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CONTACT_CONSTRAINT_PREPREP_BLOCK);

	CUdeviceptr descd = mPrePrepDescd;
	CUdeviceptr shDescd = mSharedDescd;
	PxCudaKernelParam kernelParams[] =
	{
		PX_CUDA_KERNEL_PARAM(descd),
		PX_CUDA_KERNEL_PARAM(shDescd)
	};

	const PxU32 nbBlocksRequired = (nbConstraintBatches*PXG_BATCH_SIZE + PxgKernelBlockDim::CONSTRAINT_PREPREP_BLOCK - 1) / PxgKernelBlockDim::CONSTRAINT_PREPREP_BLOCK;

	if (nbBlocksRequired > 0)
	{
		CUresult launchResult = mCudaContext->launchKernel(kernelFunction, nbBlocksRequired, 1, 1, PxgKernelBlockDim::CONSTRAINT_PREPREP_BLOCK, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		PX_ASSERT(launchResult == CUDA_SUCCESS);
		PX_UNUSED(launchResult);

#if GPU_CORE_DEBUG
		CUresult result = mCudaContext->streamSynchronize(mStream);
		PX_ASSERT(result == CUDA_SUCCESS);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU constraintContactBlockPrePrepLaunch kernel fail!\n");
#endif
	}

	const PxU32 nbD6JointsBlocks = (nbD6Joints + PxgKernelBlockDim::CONSTRAINT_PREPREP_BLOCK - 1) / PxgKernelBlockDim::CONSTRAINT_PREPREP_BLOCK;
	if (nbD6JointsBlocks > 0)
	{
		//non-block joint constraint pre-prepare
		kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::JOINT_CONSTRAINT_PREPREP);

		CUresult result = mCudaContext->launchKernel(kernelFunction, nbD6JointsBlocks, 1, 1, PxgKernelBlockDim::CONSTRAINT_PREPREP_BLOCK, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);

		PX_ASSERT(result == CUDA_SUCCESS);
		PX_UNUSED(result);

#if GPU_CORE_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU constraintPrePrepare kernel fail!\n");
#endif
	}
}

void PxgSolverCore::resetVelocities(bool isTGS)
{
	PX_PROFILE_ZONE("GpuDynamics.ZeroBodies", 0);

	{
		CUfunction zeroBodiesFunction =
		    isTGS ? mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ZERO_BODIES_TGS)
		          : mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ZERO_BODIES);
		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(mSolverCoreDescd),
			PX_CUDA_KERNEL_PARAM(mSharedDescd)
		};
		CUresult result = mCudaContext->launchKernel(zeroBodiesFunction, PxgKernelGridDim::ZERO_BODIES, 1, 1, PxgKernelBlockDim::ZERO_BODIES, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU zero bodies fail to launch kernel!!\n");

#if GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU zero bodies kernel fail!\n");
#endif
	}
}

void PxgSolverCore::precomputeReferenceCount(PxgIslandContext* islandContext, PxU32 islandIndex, PxInt32ArrayPinned& constraintsPerPartition,
	PxInt32ArrayPinned& artiConstraintsPerPartition, bool isTGS, PxReal minPen, PxReal elapsedTime)
{
	PX_PROFILE_ZONE("GpuDynamics.precomputeReferenceCount", 0);
	{
		PxgIslandContext& context = islandContext[islandIndex];

		if(context.mNumPartitions)
		{
			const PxU32 numThreadsPerWarp = WARP_SIZE;
			PxU32 numWarpsPerBlock =
			    PxgArticulationCoreKernelBlockDim::COMPUTE_UNCONSTRAINED_VELOCITES / numThreadsPerWarp;

			if (isTGS)
			{
				CUfunction markActiveSlabTGS = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
					PxgKernelIds::MARK_ACTIVE_SLAB_TGS);

				// Mark slabs that are active.
				// Loosely following solveBlockUnified launches.
				{
					const PxU32 lastPartition = context.mNumPartitions - 1; // Pass the last partition so that all the
																			// partition iterations can be run in a single
																			// kernel.
					CUdeviceptr artiDescd = mGpuContext->getArticulationCore()->getArticulationCoreDescd();

					PxCudaKernelParam kernelParamsTGS[] = {
						PX_CUDA_KERNEL_PARAM(mSolverCoreDescd), PX_CUDA_KERNEL_PARAM(mSharedDescd),
						PX_CUDA_KERNEL_PARAM(islandIndex),      PX_CUDA_KERNEL_PARAM(lastPartition),
						PX_CUDA_KERNEL_PARAM(minPen),           PX_CUDA_KERNEL_PARAM(elapsedTime),
						PX_CUDA_KERNEL_PARAM(artiDescd)
					};

					PxU32 nbBlocks = (constraintsPerPartition[lastPartition] * PXG_BATCH_SIZE +
						PxgKernelBlockDim::SOLVE_BLOCK_PARTITION - 1) /
						PxgKernelBlockDim::SOLVE_BLOCK_PARTITION;

					PxU32 nbArtiBlocks = (artiConstraintsPerPartition[lastPartition] * PXG_BATCH_SIZE +
						PxgKernelBlockDim::SOLVE_BLOCK_PARTITION - 1) /
						PxgKernelBlockDim::SOLVE_BLOCK_PARTITION;

					const PxU32 maxBlocks = PxMax(nbBlocks, nbArtiBlocks);

					if (maxBlocks)
					{
						const PxU32 blockY = nbArtiBlocks > 0 ? 2 : 1;
						CUresult result = mCudaContext->launchKernel(markActiveSlabTGS, maxBlocks, blockY, 1,
						                                             numThreadsPerWarp, numWarpsPerBlock, 1, 0, mStream,
						                                             kernelParamsTGS, sizeof(kernelParamsTGS), 0, PX_FL);

						if (result != CUDA_SUCCESS)
							PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
								"GPU markActiveSlab fail to launch kernel!!\n");

#if GPU_DEBUG
						result = mCudaContext->streamSynchronize(mStream);
						if (result != CUDA_SUCCESS)
							PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
								"GPU markActiveSlab kernel fail!\n");
#endif
					}
				}
			}
			else
			{
				CUfunction markActiveSlabPGS = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(
					PxgKernelIds::MARK_ACTIVE_SLAB_PGS);

				// Mark slabs that are active.
				// Loosely following solveBlockUnified launches.
				{
					const PxU32 lastPartition = context.mNumPartitions - 1; // Pass the last partition so that all the
																			// partition iterations can be run in a single
																			// kernel.
					CUdeviceptr artiDescd = mGpuContext->getArticulationCore()->getArticulationCoreDescd();

					PxCudaKernelParam kernelParamsPGS[] = {
						PX_CUDA_KERNEL_PARAM(mSolverCoreDescd), PX_CUDA_KERNEL_PARAM(mSharedDescd),
						PX_CUDA_KERNEL_PARAM(islandIndex),      PX_CUDA_KERNEL_PARAM(lastPartition),
						PX_CUDA_KERNEL_PARAM(artiDescd)
					};

					PxU32 nbBlocks = (constraintsPerPartition[lastPartition] * PXG_BATCH_SIZE +
						PxgKernelBlockDim::SOLVE_BLOCK_PARTITION - 1) /
						PxgKernelBlockDim::SOLVE_BLOCK_PARTITION;

					PxU32 nbArtiBlocks = (artiConstraintsPerPartition[lastPartition] * PXG_BATCH_SIZE +
						PxgKernelBlockDim::SOLVE_BLOCK_PARTITION - 1) /
						PxgKernelBlockDim::SOLVE_BLOCK_PARTITION;

					const PxU32 maxBlocks = PxMax(nbBlocks, nbArtiBlocks);

					if (maxBlocks)
					{
						const PxU32 blockY = nbArtiBlocks > 0 ? 2 : 1;
						CUresult result = mCudaContext->launchKernel(markActiveSlabPGS, maxBlocks, blockY, 1,
						                                             numThreadsPerWarp, numWarpsPerBlock, 1, 0, mStream,
						                                             kernelParamsPGS, sizeof(kernelParamsPGS), 0, PX_FL);

						if (result != CUDA_SUCCESS)
							PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
								"GPU markActiveSlab fail to launch kernel!!\n");

#if GPU_DEBUG
						result = mCudaContext->streamSynchronize(mStream);
						if (result != CUDA_SUCCESS)
							PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
								"GPU markActiveSlab kernel fail!\n");
#endif
					}
				}
			}
		}
	}
}
