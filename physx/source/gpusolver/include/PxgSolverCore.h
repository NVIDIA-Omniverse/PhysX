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

#ifndef PXG_SOLVER_CORE_H
#define PXG_SOLVER_CORE_H

#include "foundation/PxPinnedArray.h"
#include "foundation/PxUserAllocated.h"
#include "PxgConstraint.h"
#include "PxgSolverCoreDesc.h"
#include "PxvNphaseImplementationContext.h"
#include "PxgCudaBuffer.h"
#include "PxScene.h"

namespace physx
{
	namespace Dy
	{
		struct ConstraintWriteback;
	}

	struct PxgConstraintPrePrepData
	{
	public:
		PxU32 nbGpuRigidJoints;				//gpu preprep joints
		PxU32 nbTotalRigidJoints;			//cpu + gpu preprep joints
		PxU32 nbGpuArtiJoints;				//gpu preprep joints
		PxU32 nbTotalArtiJoints;			//cpu + gpu preprep joint
		PxU32 numContactBatches;
		PxU32 num1dConstraintBatches;
		PxU32 numStaticContactBatches;
		PxU32 numStatic1dConstraintBatches;

		PxU32 numArtiContactsBatches;
		PxU32 numArti1dConstraintBatches;
		PxU32 numArtiStaticContactsBatches;
		PxU32 numArtiStatic1dConstraintBatches;
		PxU32 numArtiSelfContactsBatches;
		PxU32 numArtiSelf1dConstraintBatches;

		PxU32 artiStaticConstraintBatchOffset;
		PxU32 artiStaticContactBatchOffset;

		PxU32* constraintUniqueIndices;
		PxU32* contactUniqueIndices;
		PxU32* constraintStaticUniqueIndices;
		PxU32* contactStaticUniqueIndices;
		PxU32* artiConstraintUniqueindices;
		PxU32* artiContactUniqueIndices;
		PxU32* artiStaticConstraintUniqueIndices;
		PxU32* artiStaticContactUniqueIndices;

		PxU32* artiStaticConstraintStartIndex;
		PxU32* artiStaticConstraintCount;
		PxU32* artiStaticContactStartIndex;
		PxU32* artiStaticContactCount;

		PxU32* rigidStaticConstraintStartIndex;
		PxU32* rigidStaticConstraintCount;

		//mapped memory
		PxU32* constraint1DBatchIndices;
		PxU32* constraintContactBatchIndices;
		PxU32* artiConstraintContactBatchIndices;
		PxU32* artiConstraint1dBatchindices;
	};

	struct PxgConstantData
	{
	public:
		PxReal dt;
		PxReal invDtF32;
		PxReal bounceThresholdF32;
		PxReal frictionOffsetThreshold;
		PxReal correlationDistance;
		PxReal ccdMaxSeparation;
		PxReal biasCoefficient;
		PxVec3 gravity;
	};

	struct PxgPartitionData
	{
	public:
		const PxU32* constraintsPerPartition; //rigid body contact and 1d constraint
		PxU32 numConstraintsPerPartition;

		const PxU32* artiConstraintsPerPartition; // articulation contact and 1d constraint
		PxU32 numArtiConstraintsPerPartition; 

		PxU32 numTotalConstraints;
		PxU32 numTotalContacts;
		PxU32 numTotalStaticConstraints;
		PxU32 numTotalStaticContacts;

		PxU32 numTotalArtiContacts; //dynamic contacts
		PxU32 numTotalArtiConstraints; //external constraints
		PxU32 numTotalArtiStaticContacts; //static contacts
		PxU32 numTotalArtiStaticConstraints; //static constraints
		PxU32 numTotalArtiSelfContacts; //static contacts
		PxU32 numTotalArtiSelfConstraints; //static constraints

		PxU32 artiStaticContactBatchOffset;
		PxU32 artiStaticConstraintBatchOffset;
	};

	class PxgPinnedHostLinearMemoryAllocator;

	class PxgRadixSortBuffers
	{
		public:
						PxgRadixSortBuffers(PxgHeapMemoryAllocatorManager* heapMemoryManager);

		void			constructRadixSortDesc(PxgRadixSortDesc* rsDesc)	const;
		void			allocate(PxU32 totalContactBatches);

		PxgCudaBuffer	mInputKeys;
		PxgCudaBuffer	mInputRanks;
		PxgCudaBuffer	mOutputKeys;
		PxgCudaBuffer	mOutputRanks;
		PxgCudaBuffer	mRadixCounts; 
	};

	class PxgSolverCore : public PxUserAllocated
	{
	protected:

		PxgCudaKernelWranglerManager*	mGpuKernelWranglerManager;
		PxCudaContextManager*			mCudaContextManager;
		PxCudaContext*					mCudaContext;
		PxgGpuContext*					mGpuContext;
		PxgHeapMemoryAllocatorManager*  mHeapMemoryManager;
		
		//PxgSimulationController*		mSimulationController;
		/*PxgArticulationCore*			mArticulationCore;*/

		PxgSolverCoreDesc*				mSolverCoreDesc;
		PxgConstraintPrepareDesc*		mPrepareDesc;
		PxgPrePrepDesc*					mPrePrepDesc;
		PxgRadixSortDesc*				mRsDesc;

		CUdeviceptr						mIslandContextPool;
		CUdeviceptr						mSolverCoreDescd;
		CUdeviceptr						mSharedDescd;
		CUdeviceptr						mPrepareDescd;
		CUdeviceptr						mPrePrepDescd;
		CUdeviceptr						mPartionDescd;
		CUdeviceptr						mRadixSortDescd[2];

		PxU32							mNbStaticRigidSlabs;
		PxU32							mMaxNumStaticPartitions;

		PxU32							mTotalContactManagers;
		PxU32							mNbPrevExceededForceElements;

		PxU32							mNbArticSlabs;
		PxU32							mNbConstraintSlabs; // slabs used for contacts and joints.

		void							allocateNodeInteractionCounts(PxU32 nbNodes);
		void							uploadNodeInteractionCounts(const PxU32* nodeInteractionCounts, PxU32 nbNodes);

	public:

		PxgSolverCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
			PxgGpuContext* dynamicContext, PxgHeapMemoryAllocatorManager* heapMemoryManager);

		virtual ~PxgSolverCore(){}

		/*PX_FORCE_INLINE void setSimulationController(PxgSimulationController* simulationController) { mSimulationController = simulationController; }
		PX_FORCE_INLINE PxgSimulationController* getSimulationController() { return mSimulationController; }*/

		/*PX_FORCE_INLINE void setArticulationCore(PxgArticulationCore* articulationCore) { mArticulationCore = articulationCore; }
		PX_FORCE_INLINE PxgArticulationCore* getArticulationCore() { return mArticulationCore; }
*/
		PX_FORCE_INLINE CUdeviceptr getPrePrepDescDeviceptr() { return mPrePrepDescd; }
		PX_FORCE_INLINE CUdeviceptr getPrepDescDeviceptr() { return mPrepareDescd; }
		PX_FORCE_INLINE CUdeviceptr getSolverCoreDescDeviceptr() { return mSolverCoreDescd; }
		PX_FORCE_INLINE CUdeviceptr getSharedDescDeviceptr() { return mSharedDescd; }

		virtual PxU32 getDescriptorsAllocationSize() = 0;
		virtual void allocatePinnedDescriptors(PxgPinnedHostLinearMemoryAllocator& hostAllocator) = 0;

		virtual void syncSimulationController() = 0;

		virtual void gpuMemDMAUpContactData(PxgPinnedHostLinearMemoryAllocator* compressedContactsHostMemoryAllocator,
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
				const PxU32* islandIds, const PxU32* nodeInteractionCounts, PxU32 nbNodes, const PxU32* islandStaticTouchCount, PxU32 nbIslands) = 0;

		virtual void gpuMemDmaUpBodyData(PxPinnedArray<PxgSolverBodyData>& solverBodyDataPool,
			PxPinnedArray<PxgSolverTxIData>& solverTxIDataPool,
			const PxU32 numSolverBodies,
			const PxU32 totalNumRigidBatches, const PxU32 totalNumArticBatches,
			const PxU32 nbSlabs, const PxU32 nbStaticSlabs, const PxU32 maxNumStaticPartitions) = 0;

		virtual void allocateSolverBodyBuffers(const PxU32 numSolverBodies,
			PxPinnedArray<PxNodeIndex>& islandNodeIndices,
			const PxU32 numActiveActiculations, const PxU32 maxArticulationLinks) = 0;

		virtual void gpuMemDMAUp(PxgPinnedHostLinearMemoryAllocator& hostAllocator, const PxgConstraintPrePrepData& data,
			const PxU32 numSolverBodies, PxgConstraintBatchHeader* constraintBatchHeaders,
			PxgIslandContext* islandContextPool, const PxU32 numIslands, const PxgPartitionData& partitionData ,
			const PxU32 numConstraintBatchHeader, const PxU32 numStaticConstraintBatchHeader,
			const PxU32 numArticConstraintBatchHeader, const PxU32 numArticStaticConstraintBatchHeader,
			const PxU32 numArtiSelfConstraintBatchHeader, const PxgConstantData& cData,
			const PxU32 numContactBlockes, const PxU32 numFrictionBlockes, 
			const PxU32 numArtiContacts, const PxU32 numArtiFrictions,
			const PxU32 totalCurrentEdges, const PxU32 totalPreviousEdges, const PxU32 numSlabs, const PxU32 maxNbPartitions,
			const bool enableStabilization,
			PxU8* cpuContactPatchStreamBase, PxU8* cpuContactStreamBase, PxU8* cpuForceStreamBase, PxsContactManagerOutputIterator& outputIterator,
			const PxU32 totalActiveBodyCount, const PxU32 activeBodyStartIndex, const PxU32 numArticulations, Cm::UnAlignedSpatialVector* deferredZ,
			PxU32* articulationDirty, uint4* articulationSlabMask, Sc::ShapeInteraction** shapeInteractions, PxReal* restDistances,
			PxsTorsionalFrictionData* torsionalData,
			PxU32* artiStaticContactIndices, const PxU32 artiContactIndSize, PxU32* artiStaticJointIndices, PxU32 artiStaticJointSize,
			PxU32* artiStaticContactCounts, PxU32* artiStaticJointCounts,
			PxU32* artiSelfContactIndices, const PxU32 artiSelfContactIndSize, PxU32* artiSelfJointIndices, PxU32 artiSelfJointSize,
			PxU32* artiSelfContactCounts, PxU32* artiSelfJointCounts, 
			PxU32* rigidStaticContactIndices, const PxU32 rigidContactIndSize, PxU32* rigidStaticJointIndices, const PxU32 rigidStaticJointSize,
			PxU32* rigidStaticContactCounts, PxU32* rigidSaticJointCounts, const PxReal lengthScale, bool hasForceThresholds) = 0;

		virtual void gpuMemDMAbackSolverData(PxU8* forceBufferPool, PxU32 forceBufferOffset, PxU32 forceBufferUpperPartSize,
			PxU32 forceBufferLowerPartSize, Dy::ThresholdStreamElement* changedElems, bool hasForceThresholds, Dy::ConstraintWriteback* constraintWriteBack,
			const PxU32 writeBackSize, bool copyAllToHost, Dy::ErrorAccumulator*& contactError) = 0;


		virtual void syncDmaBack(PxU32& nbChangedThresholdElements) = 0;

		virtual void createStreams() = 0;
		virtual void releaseStreams() = 0;

		virtual void acquireContext() = 0;
		virtual void releaseContext() = 0;

		virtual void preIntegration(const PxU32 offset, const PxU32 nbSolverBodies, const PxReal dt, const PxVec3& gravity) = 0;
		
		virtual void jointConstraintBlockPrePrepParallel(PxU32 nbConstraintBatches) = 0;

		virtual void jointConstraintPrepareParallel(PxU32 nbJointBatches) = 0;
		virtual void contactConstraintPrepareParallel(PxU32 nbContactBatches) = 0;
		virtual void artiJointConstraintPrepare(PxU32 nbArtiJointBatches) = 0;
		virtual void artiContactConstraintPrepare(PxU32 nbArtiContactBatches) = 0;
		virtual void nonRigidConstraintPrepare(PxU32 nbParticulations) = 0;

		virtual void solveContactMultiBlockParallel(PxgIslandContext* islandContexts, const PxU32 numIslands, const PxU32 maxPartitions,
			PxInt32ArrayPinned& constraintsPerPartition, PxInt32ArrayPinned& artiConstraintsPerPartition, const PxVec3& gravity,
			PxReal* posIterResidualSharedMem, PxU32 posIterResidualSharedMemSize, Dy::ErrorAccumulator* posIterError, PxPinnedArray<Dy::ErrorAccumulator>& artiContactPosIterError,
			PxPinnedArray<Dy::ErrorAccumulator>& perArticulationInternalError) = 0;

		virtual void accumulatedForceThresholdStream(PxU32 maxNodes) = 0;
		virtual void integrateCoreParallel( const PxU32 offset, const PxU32 nbSolverBodies) = 0;

		virtual void getDataStreamBase(void*& contactStreamBase, void*& patchStreamBase, void*& forceAndIndexStreamBase) = 0;

		PX_FORCE_INLINE PxgDevicePointer<PxNodeIndex> getGpuIslandNodeIndices()  { return mIslandNodeIndices2.getTypedDevicePtr(); }

		PX_FORCE_INLINE void setGpuContactManagerOutputBase(PxsContactManagerOutput* gpuContactManagerOutputBase) { mGpuContactManagerOutputBase = reinterpret_cast<CUdeviceptr>(gpuContactManagerOutputBase); }

		PX_FORCE_INLINE CUstream getStream() { return mStream; } 

		PX_FORCE_INLINE PxgDevicePointer<PxU32> getSolverBodyIndices() { return mSolverBodyIndices.getTypedDevicePtr(); }

		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgSolverBodyData>*	getSolverBodyData() { return &mSolverBodyDataPool; }

		PX_FORCE_INLINE PxgDevicePointer<PxgSolverBodySleepData>	getSolverBodySleepData() { return mSolverBodySleepDataPool.getTypedDevicePtr();}

		PX_FORCE_INLINE PxNodeIndex* getCpuIslandNodeIndices() { return mCpuIslandNodeIndices; }

		PX_FORCE_INLINE PxgDevicePointer<PxgConstraintWriteback> getConstraintWriteBackBufferDevicePtr() const { return mConstraintWriteBackBuffer.getTypedDevicePtr(); }

		void allocateFrictionPatchStream(PxI32 numContactBatches, PxI32 numArtiContactBatches);
		PxgBlockFrictionIndex* allocateFrictionPatchIndexStream(PxU32 totalFrictionPatchCount);
		void allocateFrictionCounts(PxU32 totalEdges);

		void gpuMemDMAbackSolverBodies(float4* solverBodyPool, PxU32 nbSolverBodies,
			PxPinnedArray<PxAlignedTransform>& body2WorldPool,
			PxPinnedArray<PxgSolverBodySleepData>& solverBodySleepDataPool, bool enableDirectGPUAPI);

		void allocateSolverBodyBuffersCommon(PxU32 numSolverBodies, PxPinnedArray<PxNodeIndex>& islandNodeIndices);

		void constructConstraintPrePrepDesc(PxgPrePrepDesc& preDesc, PxU32 numBatches, PxU32 numStaticBatches, PxU32 numArticBatches, PxU32 numArticStaticBatches, PxU32 numArticSelfBatches,
			const PxgPartitionData& pData, PxContact* cpuCompressedcontactsBase, PxContactPatch* cpuCompressedPatchesBase, PxReal* cpuForceBufferBase,
			PxU32 nbD6RigidJoint, PxU32 nbD6ArtiJoint, PxU32 nbTotalArtiJoints,
			PxsContactManagerOutputIterator& outputIterator, PxU32 maxConstraintPartitions, PxU32 totalActiveBodies, PxU32 totalActiveArticulations,
			PxU32 activeBodyStartOffset, Sc::ShapeInteraction** shapeInteractions, PxReal* restDistances, PxsTorsionalFrictionData* torsionalData, PxU32 nbElementsPerBody, PxU32 numSlabs);

		void constructSolverSharedDescCommon(PxgSolverSharedDescBase& desc,
			const PxgConstantData& cData, Cm::UnAlignedSpatialVector* deferredZ, PxU32* articulationDirty, uint4* articulationSlabMask);

		void constructSolverDesc(PxgSolverCoreDesc& scDesc, PxU32 numIslands, PxU32 numSolverBodies, PxU32 numConstraintBatchHeader, PxU32 numArticConstraints, PxU32 numSlabs, bool enableStabilization);

		void gpuMemDMAUpJointData(const PxPinnedArray<PxgConstraintData>& cpuJointDataPool, const PxPinnedArray<Px1DConstraint>& cpuJointRowPool,
			PxU32 nbCpuJoints, PxU32 nbGpuJoints, PxU32 totalCpuRows);

		void gpuMemDMAUpArtiJointData(const PxPinnedArray<PxgConstraintData>& artiJointDataPool, const PxPinnedArray<Px1DConstraint>& artiJointRowPool,
			PxU32 nbCpuArtiJoints, PxU32 nbGpuArtiJoints, PxU32 totalArtiRows);

		void constraintPrePrepParallel(PxU32 nbConstraintBatches, PxU32 nbD6Joints, PxU32 numBodies);
		
		void precomputeReferenceCount(PxgIslandContext* islandContext, PxU32 islandIndex, PxInt32ArrayPinned& constraintsPerPartition,
			PxInt32ArrayPinned& artiConstraintsPerPartition, bool isTGS, PxReal minPen = 0.0f, PxReal elapsedTime = 0.0f);
		
		void resetVelocities(bool isTGS);

		PX_FORCE_INLINE	void	resetMemoryAllocator()
		{
			mCurrentIndex = 1 - mCurrentIndex;
		}

		PxgCudaBuffer		mContactHeaderBlockStream; //Different types for PGS and TGS
		PxgCudaBuffer		mFrictionHeaderBlockStream; //Different types for PGS and TGS
		PxgCudaBuffer		mContactBlockStream; //Different types for PGS and TGS
		PxgCudaBuffer		mFrictionBlockStream; //Different types for PGS and TGS

		PxgCudaBuffer		mJointHeaderBlockStream; //Different types for PGS and TGS
		PxgCudaBuffer		mJointRowBlockStreamCon; //Different types for PGS and TGS
		PxgTypedCudaBuffer<PxgBlockSolverConstraint1DMod>	mJointRowBlockStreamMod;

		PxgTypedCudaBuffer<PxgBlockContactData>				mConstraintContactPrepBlockPool;

		PxgTypedCudaBuffer<PxgBlockConstraint1DData>		mConstraint1DPrepBlockPool;
		PxgTypedCudaBuffer<PxgBlockConstraint1DVelocities>	mConstraint1DPrepBlockPoolVel;
		PxgTypedCudaBuffer<PxgBlockConstraint1DParameters>	mConstraint1DPrepBlockPoolPar;

		PxgTypedCudaBuffer<PxgConstraintData>				mConstraintDataPool;
		PxgTypedCudaBuffer<Px1DConstraint>					mConstraintRowPool;

		PxgTypedCudaBuffer<PxgConstraintData>				mArtiConstraintDataPool;
		PxgTypedCudaBuffer<Px1DConstraint>					mArtiConstraintRowPool;

		PxgTypedCudaBuffer<float4>					mSolverBodyPool;
		PxgTypedCudaBuffer<float4>					mTempStaticBodyOutputPool;
		PxgTypedCudaBuffer<PxNodeIndex>				mIslandNodeIndices2;
		PxgTypedCudaBuffer<PxU32>					mSolverBodyIndices;

		PxgTypedCudaBuffer<float4>					mOutVelocityPool;		//this is the output of linear and angular velocity for the solver body
		PxgTypedCudaBuffer<PxAlignedTransform>		mOutBody2WorldPool;		//this is the output of body to world transform for the solver body
		PxgTypedCudaBuffer<PxgSolverBodyData>		mSolverBodyDataPool;
		PxgTypedCudaBuffer<PxgSolverBodySleepData>	mSolverBodySleepDataPool;

		PxgTypedCudaBuffer<float4>					mOutArtiVelocityPool; //velocity(linear and angular) of the link for the articulations
		
		PxgTypedCudaBuffer<PxgSolverTxIData>		mSolverTxIDataPool;
		PxgTypedCudaBuffer<PxU32>					mConstraintsPerPartition;
		PxgTypedCudaBuffer<PxU32>					mArtiConstraintsPerPartition;
		PxgTypedCudaBuffer<float4>					mMotionVelocityArray;

		PxgTypedCudaBuffer<PxgBlockConstraintBatch>	mBlockConstraintBatches;
		CUdeviceptr									mConstraintBatchHeaders;
		CUdeviceptr									mConstraintUniqueIndices;
		CUdeviceptr									mContactUniqueIndices;

		CUdeviceptr			mArtiConstraintUniqueIndices;
		CUdeviceptr			mArtiContactUniqueIndices;

		CUdeviceptr			mArtiStaticConstraintUniqueIndices;
		CUdeviceptr			mArtiStaticContactUniqueIndices;

		PxgTypedCudaBuffer<PxU32>		mArtiOrderedStaticConstraints;
		PxgTypedCudaBuffer<PxU32>		mArtiOrderedStaticContacts;

		PxgTypedCudaBuffer<PxgSolverReferences>		mSolverBodyReferences;
		PxgTypedCudaBuffer<PxgBlockWorkUnit>		mBlockWorkUnits;

		//Body remapping information
		PxgTypedCudaBuffer<PartitionIndexData>		mPartitionIndexData;
		PxgTypedCudaBuffer<PartitionNodeData>		mPartitionNodeData;
		PxgTypedCudaBuffer<PxgSolverConstraintManagerConstants>		mSolverConstantData;
		PxgTypedCudaBuffer<PxU32>	mPartitionStartBatchIndices;
		PxgTypedCudaBuffer<PxU32>	mPartitionArticulationStartBatchIndices;
		PxgTypedCudaBuffer<PxU32>	mPartitionJointBatchCounts;
		PxgTypedCudaBuffer<PxU32>	mPartitionArtiJointBatchCounts;
									
		PxgTypedCudaBuffer<PxU32>	mDestroyedEdgeIndices;
		PxgTypedCudaBuffer<PxU32>	mNpIndexArray;

		PxgTypedCudaBuffer<PxgBlockContactPoint>	mGpuContactBlockBuffer;
		PxgTypedCudaBuffer<PxU32>					mDataBuffer;
		PxgTypedCudaBuffer<PxContact>				mCompressedContacts;
		PxgTypedCudaBuffer<PxContactPatch>			mCompressedPatches;
		PxgTypedCudaBuffer<PxgConstraintWriteback>	mConstraintWriteBackBuffer; //1d constraint write back buffer
		PxgTypedCudaBuffer<PxReal>					mForceBuffer; // contact write back buffer
		PxgTypedCudaBuffer<PxFrictionPatch>			mFrictionPatches;

		CUdeviceptr						mGpuContactManagerOutputBase;

		PxgTypedCudaBuffer<PxU32>		mArtiStaticContactIndices;
		PxgTypedCudaBuffer<PxU32>		mArtiStaticJointIndices;
		PxgTypedCudaBuffer<PxU32>		mArtiStaticContactCounts;
		PxgTypedCudaBuffer<PxU32>		mArtiStaticJointCounts;

		PxgTypedCudaBuffer<PxU32>		mRigidStaticContactIndices;
		PxgTypedCudaBuffer<PxU32>		mRigidStaticJointIndices;
		PxgTypedCudaBuffer<PxU32>		mRigidStaticContactCounts;
		PxgTypedCudaBuffer<PxU32>		mRigidStaticJointCounts;
		PxgTypedCudaBuffer<PxU32>		mRigidStaticContactStartIndices;
		PxgTypedCudaBuffer<PxU32>		mRigidStaticJointStartIndices;

		PxgTypedCudaBuffer<PxU32>		mTempContactUniqueIndicesBlockBuffer;
		PxgTypedCudaBuffer<PxU32>		mTempConstraintUniqueIndicesBlockBuffer;
		PxgTypedCudaBuffer<PxU32>		mTempContactHeaderBlockBuffer;
		PxgTypedCudaBuffer<PxU32>		mTempConstraintHeaderBlockBuffer;

		PxgTypedCudaBuffer<PxU32>		mArtiSelfContactIndices;
		PxgTypedCudaBuffer<PxU32>		mArtiSelfJointIndices;
		PxgTypedCudaBuffer<PxU32>		mArtiSelfContactCounts;
		PxgTypedCudaBuffer<PxU32>		mArtiSelfJointCounts;

		PxgTypedCudaBuffer<PxU32>		mNodeInteractionCounts;

		PxgCudaBufferN<2>	mFrictionPatchBlockStream;
		PxgCudaBufferN<2>	mFrictionAnchorPatchBlockStream;
		PxgCudaBufferN<2>	mFrictionIndexStream;
		PxgCudaBufferN<2>	mFrictionPatchCounts;

		//Non-block versions for articulation contacts. Remove!
		PxgCudaBufferN<2>	mFrictionPatchStream;
		PxgCudaBufferN<2>	mFrictionAnchorPatchStream;

		PxU32				mCurrentIndex;

		CUdeviceptr			mArtiStaticConstraintStartIndex;
		CUdeviceptr			mArtiStaticConstraintCount;
		CUdeviceptr			mArtiStaticContactStartIndex;
		CUdeviceptr			mArtiStaticContactCount;

		CUstream			mStream;
		CUstream			mStream2;

		PxU32*				mPinnedEvent;

		CUevent				mEventDmaBack;

		CUevent				mIntegrateEvent;

		PxNodeIndex*		mCpuIslandNodeIndices;

		PxU32				mSolverBodyOutputVelocityOffset;

		PxgRadixSortBuffers	mRadixSort;
	};
}

#endif
