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

#ifndef PXG_TGS_CUDA_SOLVER_CORE_H
#define PXG_TGS_CUDA_SOLVER_CORE_H

#include "PxgSolverCore.h"

namespace physx
{
	// PT: TODO: rename to just PxgTGSSolverCore ?
	class PxgTGSCudaSolverCore : public PxgSolverCore
	{
		PX_NOCOPY(PxgTGSCudaSolverCore)
	private:

		//this is for articulation
		PxgCudaBuffer		mConstraintContactPrepPool;
		PxgTypedCudaBuffer<PxgTGSSolverContactHeader>		mContactHeaderStream;
		PxgTypedCudaBuffer<PxgTGSSolverContactPointExt>		mContactStream;
		PxgTypedCudaBuffer<PxgTGSSolverFrictionExt>		mFrictionStream;

		// Each bit encodes the activation of a slab (32 bits). When there are more than 32 slabs, use multiple indices.
		// To query the reference count, count the number of active slabs/bits.
		PxgTypedCudaBuffer<PxU32>		mSolverEncodedReferenceCount;

		//This is the new articulation block constraint format!
		//It shares the original rigid body contact/constraint format but adds in
		//an additional buffer for the response vectors
		PxgTypedCudaBuffer<PxgArticulationBlockResponse>		mArtiConstraintBlockResponse;
		
		PxgTypedCudaBuffer<Dy::ThresholdStreamElement>		mForceThresholdStream;
		PxgTypedCudaBuffer<Dy::ThresholdStreamElement>		mTmpForceThresholdStream;

		PxgTypedCudaBuffer<PxU32>		mConstraint1DBatchIndices;
		PxgTypedCudaBuffer<PxU32>		mContactBatchIndices;
		PxgTypedCudaBuffer<PxU32>		mArtiContactBatchIndices;
		PxgTypedCudaBuffer<PxU32>		mArtiConstraint1dBatchIndices;

		PxgTypedCudaBuffer<PxReal>		mAccumulatedForceObjectPairs; //store the accumulated force for a pair of objects
		PxgCudaBufferN<2>	mExceededForceElements;
		PxgTypedCudaBuffer<Dy::ThresholdStreamElement>		mForceChangeThresholdElements;

		PxgTypedCudaBuffer<PxReal>		mThresholdStreamAccumulatedForce;
		PxgTypedCudaBuffer<PxReal>		mBlocksThresholdStreamAccumulatedForce;

		PxgTypedCudaBuffer<PxU32>		mThresholdStreamWriteIndex;
		PxgTypedCudaBuffer<PxU32>		mBlocksThresholdStreamWriteIndex;
		PxgTypedCudaBuffer<bool>		mThresholdStreamWriteable;

		PxgTypedCudaBuffer<PxU32>		mIslandIds;
		PxgTypedCudaBuffer<PxU32>		mIslandStaticTouchCount;

		PxgSolverSharedDesc<IterativeSolveDataTGS>* mSharedDesc;

		void radixSort(const PxU32 nbPasses);

		friend class PxgArticulationCore;

	public:

		PxgTGSCudaSolverCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager, 
			PxgGpuContext* dynamicContext, PxgHeapMemoryAllocatorManager* heapMemoryManager, const PxGpuDynamicsMemoryConfig& init);
		~PxgTGSCudaSolverCore();

		void constructSolverSharedDesc(PxgSolverSharedDesc<IterativeSolveDataTGS>& desc, const PxgConstantData& cData,
			const PxU32 numIters, const PxReal lengthScale, Cm::UnAlignedSpatialVector* deferredZ, PxU32* articulationDirty,
			uint4* articulationSlabMask);

		void constructConstraitPrepareDesc(PxgConstraintPrepareDesc& desc, const PxU32 numDynamicConstraintBatchHeader,
			const PxU32 numStaticConstraintBatchHeaders, const PxU32 numDynamic1dConstraintBatches, const PxU32 numStatic1dConstraintBatches,
			const PxU32 numDynamicContactBatches, const PxU32 numStaticContactBatches,
			const PxU32 numArti1dConstraintBatches, const PxU32 numArtiContactBatches,
			const PxU32 numArtiStatic1dConstraintBatches, const PxU32 numArtiStaticContactBatches, 
			const PxU32 numArtiSelf1dConstraintBatches, const PxU32 numArtiSelfContactBatches,
			const PxgConstantData& cData, PxU32 totalCurrentEdges, PxU32 totalPreviousEdges, PxU32 totalBodies);

		void constructSolverDesc(PxgSolverCoreDesc& desc, PxU32 numIsland, PxU32 numSolverBodies, PxU32 numConstraintBatchHeader,
			PxU32 numArticConstraints, PxU32 numSlabs, bool enableStabilization);

		void syncSimulationController();

		virtual void createStreams();
		virtual void releaseStreams();

		virtual void acquireContext();
		virtual void releaseContext();

		void gpuMemDMAUpContactData(PxgPinnedHostLinearMemoryAllocator* compressedContactsHostMemoryAllocator,
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
			PxU32 nbPartitions,
			const PxU32* destroyedEdges,
			PxU32 nbDestroyedEdges,
			const PxU32* npIndexArray, PxU32 npIndexArraySize,
			PxU32 totalNumJoints,
			const PxU32* islandIds, const PxU32* nodeInteractionCounts, PxU32 nbNodes, const PxU32* islandStaticTouchCount, PxU32 nbIslands);

		void gpuMemDmaUpBodyData(PxPinnedArray<PxgSolverBodyData>& solverBodyDataPool,
			PxPinnedArray<PxgSolverTxIData>& solverTxIDataPool,
			const PxU32 numSolverBodies,
			const PxU32 totalNumRigidBatches, const PxU32 totalNumArticBatches,
			const PxU32 nbSlabs, const PxU32 nbStaticSlabs, const PxU32 maxNumStaticPartitions);

		void allocateSolverBodyBuffers(const PxU32 numSolverBodies,
			PxPinnedArray<PxNodeIndex>& islandNodeIndices,
			const PxU32 numActiveActiculations, const PxU32 maxArticulationLinks);

		PxU32 getDescriptorsAllocationSize();
		void allocatePinnedDescriptors(PxgPinnedHostLinearMemoryAllocator& hostAllocator); 

		void gpuMemDMAUp(PxgPinnedHostLinearMemoryAllocator& hostAllocator, const PxgConstraintPrePrepData& data,
			const PxU32 numSolverBodies, PxgConstraintBatchHeader* constraintBatchHeaders,
			PxgIslandContext* islandContextPool, const PxU32 numIslands, const PxgPartitionData& partitionData,
			const PxU32 numConstraintBatchHeader, const PxU32 numStaticConstraintBatchHeader,
			const PxU32 numArticConstraintBatchHeader, const PxU32 numArticStaticConstraintBatchHeader, 
			const PxU32 numArtiSelfConstraintBatchHeader, const PxgConstantData& cData,
			const PxU32 numContactBlockes, const PxU32 numFrictionBlockes,
			const PxU32 numArtiContacts, const PxU32 numArtiFrictions,
			const PxU32 totalCurrentEdges, const PxU32 totalPreviousEdges, const PxU32 numSlabs, const PxU32 maxNbPartitions,
			const bool enableStabilization, PxU8* cpuContactPatchStreamBase, PxU8* cpuContactStreamBase, PxU8* cpuForceStreamBase, PxsContactManagerOutputIterator& outputIterator,
			const PxU32 totalActiveBodyCount, const PxU32 activeBodyStartIndex, const PxU32 nbArticulations, Cm::UnAlignedSpatialVector* deferredZ,
			PxU32* articulationDirty, uint4* articulationSlabMask, Sc::ShapeInteraction** shapeInteractions, PxReal* restDistances,
			PxsTorsionalFrictionData* torsionalData,
			PxU32* artiStaticContactIndices, const PxU32 artiContactIndSize, PxU32* artiStaticJointIndices, PxU32 artiStaticJointSize,
			PxU32* artiStaticContactCounts, PxU32* artiStaticJointCounts,
			PxU32* artiSelfContactIndices, const PxU32 artiSelfContactIndSize, PxU32* artiSelfJointIndices, PxU32 artiSelfJointSize,
			PxU32* artiSelfContactCounts, PxU32* artiSelfJointCounts, 
			PxU32* rigidStaticContactIndices, const PxU32 rigidContactIndSize, PxU32* rigidStaticJointIndices, const PxU32 rigidStaticJointSize,
			PxU32* rigidStaticContactCounts, PxU32* rigidSaticJointCounts, const PxReal lengthScale, bool hasForceThresholds);

		void gpuMemDMAbackSolverData(PxU8* forceBufferPool, PxU32 forceBufferOffset, PxU32 forceBufferUpperPartSize,
			PxU32 forceBufferLowerPartSize, Dy::ThresholdStreamElement* changedElems, bool hasForceThresholds, Dy::ConstraintWriteback* constraintWriteBack,
			const PxU32 writeBackSize, bool copyAllToHost, Dy::ErrorAccumulator*& contactError);

		void syncDmaBack(PxU32& nbChangedThresholdElements);

		void preIntegration(const PxU32 offset, const PxU32 nbSolverBodies, const PxReal dt, const PxVec3& gravity);
		
		void jointConstraintBlockPrePrepParallel(PxU32 nbConstraintBatches);

		void jointConstraintPrepareParallel(PxU32 nbJointBatches);
		void contactConstraintPrepareParallel(PxU32 nbContactBatches);
		void artiJointConstraintPrepare(PxU32 nbArtiJointBatches);
		void artiContactConstraintPrepare(PxU32 nbArtiContactBatches);
		//soft body/cloth/particle constraint prepare
		void nonRigidConstraintPrepare(PxU32 nbArticulations);

		void solveContactMultiBlockParallel(PxgIslandContext* islandContexts, const PxU32 numIslands, const PxU32 maxPartitions,
			PxInt32ArrayPinned& constraintsPerPartition, PxInt32ArrayPinned& artiConstraintsPerPartition, const PxVec3& gravity,
			PxReal* posIterResidualSharedMem, PxU32 posIterResidualSharedMemSize, Dy::ErrorAccumulator* posIterError, PxPinnedArray<Dy::ErrorAccumulator>& artiContactPosIterError,
			PxPinnedArray<Dy::ErrorAccumulator>& perArticulationInternalError);

		void writeBackBlock(PxU32 a, PxgIslandContext& context);

		void solvePartitions(PxgIslandContext* islandContexts, PxInt32ArrayPinned& constraintsPerPartition, PxInt32ArrayPinned& artiConstraintsPerPartition,
			PxU32 islandIndex, bool doFriction, PxReal accumulatedDt, PxReal minPen, bool anyArticulationConstraints, bool isVelocityIteration);

		void accumulatedForceThresholdStream(PxU32 maxNodes);
		void integrateCoreParallel(const PxU32 offset, const PxU32 nbSolverBodies);

		void getDataStreamBase(void*& contactStreamBase, void*& patchStreamBase, void*& forceAndIndexStreamBase);
	};
}

#endif
