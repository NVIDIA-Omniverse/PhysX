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

#ifndef PXG_CONTEXT_H
#define PXG_CONTEXT_H

#include "DyContext.h"
#include "PxSimulationStatistics.h"
#include "PxgConstraintPartition.h"
#include "PxgCudaMemoryAllocator.h"
#include "PxgConstraintPrep.h"
#include "PxvNphaseImplementationContext.h"

namespace physx
{
	class PxgCudaBroadPhaseSap;
	class PxgSolverCore;
	class PxgGpuNarrowphaseCore;
	class PxgArticulationCore;
	class PxgSimulationCore;
	class PxgSoftBodyCore;
	class PxgFEMClothCore;
	class PxgPBDParticleSystemCore;
	class PxgSimulationController;
	struct PxgIslandContext;
	class PxgHeapMemoryAllocatorManager;
	struct PxsTorsionalFrictionData;

	// PT: TODO: all these tasks are missing a proper context ID for the profiler...

	class PxgCpuJointPrePrepTask : public Cm::Task
	{
		PxgSimulationController& mSimController;

		const Dy::Constraint*const* mConstraints;
		PxgConstraintData* mConstraintData;
		Px1DConstraint* mConstraintRows;

		const PxU32 mStartIndex;
		const PxU32 mNbToProcess;
		const PxU32 mGpuJointOffset;

		PxI32* mRowCounts;

		PX_NOCOPY(PxgCpuJointPrePrepTask)

	public:
		PxgCpuJointPrePrepTask(PxgSimulationController& simConstroller, PxU32 startIndex, PxU32 nbToProcess, PxU32 gpuJointOffset,
			const Dy::Constraint*const* constraints, PxgConstraintData* constraintData, Px1DConstraint* constraintRows, PxI32* rowCounts) :
			Cm::Task(0), mSimController(simConstroller), mConstraints(constraints), mConstraintData(constraintData), mConstraintRows(constraintRows),
			mStartIndex(startIndex), mNbToProcess(nbToProcess), mGpuJointOffset(gpuJointOffset), mRowCounts(rowCounts)
		{
		}

		virtual void runInternal() PX_OVERRIDE PX_FINAL;

		virtual const char* getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgTGSCpuJointPrePrepTask";
		}
	};

	class PxgGpuContext;

	class PxgCpuPreIntegrationTask : public Cm::Task
	{
		PxgGpuContext& mContext;

		PX_NOCOPY(PxgCpuPreIntegrationTask)

	public:

		PxgCpuPreIntegrationTask(PxgGpuContext& context) : Cm::Task(0), mContext(context)
		{
		}

		virtual void runInternal() PX_OVERRIDE PX_FINAL;

		virtual const char* getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgCpuPreIntegrationTask";
		}
	};

	class PxgCpuContactPrePrepTask : public Cm::Task
	{
		//From the below, we should be able to iterate over the partitions, process contact pairs
		const PxgIncrementalPartition& mPartition;
		const PxU32 mPartitionIndex;
		const PxU32 mStartIndexWithinPartition;
		const PxU32 mNbToProcess;

		const PxU32* mStartSlabIter;
		const PxU32 mStartSlabOffset;
		const PxU32* mContactStartIndices;

		PxgConstraintBatchHeader* mBatchHeaders;
		const PxU32 mNumBatches;
		const PxU32 mWorkUnitStartIndex;

		PxU32* mPinnedEdgeIds;

		const PxsContactManagerOutputIterator& mOutputIterator;

		const PxU8* mBaseContactPatch;
		const PxU8* mBaseContactPointer;

		PX_NOCOPY(PxgCpuContactPrePrepTask)

	public:
		PxgCpuContactPrePrepTask(const PxgIncrementalPartition& partition, PxU32 partitionIndex, PxU32 startIndexWithinPartition, PxU32 nbToProcess,
			const PxU32* startSlabIter, PxU32 startSlabOffset, const PxU32* contactStartIndices,
			PxgConstraintBatchHeader* batchHeaders, PxU32 nbBatches, PxU32 workUnitStartIndex,
			PxU32* pinnedEdgeIds, PxsContactManagerOutputIterator& outputIter,
			const PxU8* baseContactPatch, const PxU8* baseContactPointer) : Cm::Task(0),
			mPartition(partition), mPartitionIndex(partitionIndex), mStartIndexWithinPartition(startIndexWithinPartition), mNbToProcess(nbToProcess),
			mStartSlabIter(startSlabIter), mStartSlabOffset(startSlabOffset), mContactStartIndices(contactStartIndices),
			mBatchHeaders(batchHeaders), mNumBatches(nbBatches), mWorkUnitStartIndex(workUnitStartIndex),
			mPinnedEdgeIds(pinnedEdgeIds), mOutputIterator(outputIter),
			mBaseContactPatch(baseContactPatch), mBaseContactPointer(baseContactPointer)
		{
		}

		virtual void runInternal() PX_OVERRIDE PX_FINAL;

		virtual const char* getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgCpuContactPrePrepTask";
		}
	};

	class PxgCpuConstraintPrePrepTask : public Cm::Task
	{
		const PartitionIndices& mEdgeIds;
		const PxU32 mStartEdgeIdx;
		const PxU32 mNumEdges;
		PxgConstraintBatchHeader* mBatchHeaders;

		const PxU32 mNumBatches;
		const PxU32 mConstraintBlockStartIndex;
		const PxU32 mUniqueIdStartIndex;

		PxU32* mPinnedEdgeIds;

		const PxgConstraintPrePrep* mConstraintPrePrep;

		PX_NOCOPY(PxgCpuConstraintPrePrepTask)

	public:

		static const PxU32 NbConstraintsPerTaskTGS = 2048u;
		static const PxU32 NbConstraintsPerTaskPGS = 8192u;

		PxgCpuConstraintPrePrepTask(const PartitionIndices& edgeIds, PxU32 startEdgeIdx, PxU32 nbEdges, PxgConstraintBatchHeader* batchHeaders, PxU32 nbBatches,
			PxU32 constraintBlockStartIndex, PxU32 uniqueIdStartIndex, PxU32* pinnedEdgeIds,
			const PxgConstraintPrePrep* constraintPrePrep) : Cm::Task(0),
			mEdgeIds(edgeIds), mStartEdgeIdx(startEdgeIdx), mNumEdges(nbEdges), mBatchHeaders(batchHeaders), mNumBatches(nbBatches),
			mConstraintBlockStartIndex(constraintBlockStartIndex), mUniqueIdStartIndex(uniqueIdStartIndex), mPinnedEdgeIds(pinnedEdgeIds)
			, mConstraintPrePrep(constraintPrePrep)
		{
		}

		virtual void runInternal() PX_OVERRIDE PX_FINAL;

		virtual const char* getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgCpuConstraintPrePrepTask";
		}
	};

	//this include contact and constraint
	class PxgCpuArtiConstraintPrePrepTask : public Cm::Task
	{
		const PartitionIndices&		mEdgeIds;
		const PxU32					mStartEdgeIdx;
		const PxU32					mNumEdges;
		PxgConstraintBatchHeader*	mBatchHeaders;

		const PxU32					mNumBatches;
		const PxU32					mConstraintBlockStartIndex;
		const PxU32					mUniqueIdStartIndex;

		PxU32*						mPinnedEdgeIds;

		const PxgConstraintPrePrep*	mConstraintPrePrep;
		const bool					mIsContact;

		PX_NOCOPY(PxgCpuArtiConstraintPrePrepTask)

	public:

		static const PxU32 NbConstraintsPerTaskPGS = 8192u;
		static const PxU32 NbConstraintsPerTaskTGS = 512u;

		PxgCpuArtiConstraintPrePrepTask(const PartitionIndices& edgeIds, PxU32 startEdgeIdx, PxU32 nbEdges, PxgConstraintBatchHeader* batchHeaders,
			PxU32 nbBatches, PxU32 constraintBlockStartIndex, PxU32 uniqueIdStartIndex, PxU32* pinnedEdgeIds,
			const PxgConstraintPrePrep* constraintPrePrep, bool isContact) : Cm::Task(0),
			mEdgeIds(edgeIds), mStartEdgeIdx(startEdgeIdx), mNumEdges(nbEdges), mBatchHeaders(batchHeaders), mNumBatches(nbBatches),
			mConstraintBlockStartIndex(constraintBlockStartIndex), mUniqueIdStartIndex(uniqueIdStartIndex), mPinnedEdgeIds(pinnedEdgeIds)
			, mConstraintPrePrep(constraintPrePrep), mIsContact(isContact)
		{
		}

		virtual void		runInternal() PX_OVERRIDE PX_FINAL;

		virtual const char* getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgCpuArtiConstraintPrePrepTask";
		}
	};

	class PxgCpuPrepTask : public Cm::Task
	{
		PxgGpuContext& mContext;
		
		PX_NOCOPY(PxgCpuPrepTask)

	public:
							PxgCpuPrepTask(PxgGpuContext& context) : Cm::Task(0), mContext(context)	{}

		virtual void		runInternal() PX_OVERRIDE PX_FINAL;
		virtual const char*	getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgCpuPrepTask";
		}
	};

	class PxgGpuPrePrepTask : public Cm::Task
	{
		PxgGpuContext& mContext;

		PX_NOCOPY(PxgGpuPrePrepTask)

	public:
							PxgGpuPrePrepTask(PxgGpuContext& context) : Cm::Task(0), mContext(context)	{}

		virtual void		runInternal() PX_OVERRIDE PX_FINAL;
		virtual const char* getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgGpuPrePrepTask";
		}
	};

	class PxgPostSolveTask : public Cm::Task
	{
		PxgGpuContext& mContext;
	
		PX_NOCOPY(PxgPostSolveTask)

	public:
							PxgPostSolveTask(PxgGpuContext& context) : Cm::Task(0), mContext(context)	{}

		virtual void		runInternal() PX_OVERRIDE PX_FINAL;
		virtual const char*	getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgPostSolveTask";
		}
	};

	class PxgGpuTask : public Cm::Task
	{
		PxgGpuContext&	mContext;
		PxU32			mMaxNodes;
		PxBitMapPinned*	mChangedHandleMap;//this is for the simulation controller
		
		PX_NOCOPY(PxgGpuTask)

	public:
							PxgGpuTask(PxgGpuContext& context) : Cm::Task(0), mContext(context), mMaxNodes(0), mChangedHandleMap(NULL)	{}

				void		setMaxNodesAndWordCounts(const PxU32 maxNodes, PxBitMapPinned& changedHandleMap) { mMaxNodes = maxNodes; mChangedHandleMap = &changedHandleMap; }

		virtual void		runInternal() PX_OVERRIDE PX_FINAL;
		virtual const char*	getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgGpuTask";
		}
	};

	class PxgGpuIntegrationTask : public Cm::Task
	{
		PxgGpuContext& mContext;

		PX_NOCOPY(PxgGpuIntegrationTask)
	public:
							PxgGpuIntegrationTask(PxgGpuContext& context) : Cm::Task(0), mContext(context)	{}

		virtual void		runInternal() PX_OVERRIDE PX_FINAL;
		virtual const char*	getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgGpuIntegrationTask";
		}
	};

	class PxgGpuContext : public Dy::Context
	{
		PX_NOCOPY(PxgGpuContext)

	public:

		PxgGpuContext(Cm::FlushPool& flushPool, IG::SimpleIslandManager& islandManager, 
			PxU32 maxNumPartitions, PxU32 maxNumStaticPartitions, bool enableStabilization, bool useEnhancedDeterminism, 
			PxReal maxBiasCoefficient, PxvSimStats& simStats, PxgHeapMemoryAllocatorManager* heapMemoryManager,
			PxReal lengthScale, bool enableDirectGPUAPI, PxU64 contextID, bool isResidualReportingEnabled, bool isTGS);

		virtual ~PxgGpuContext();

		PX_FORCE_INLINE PxgSolverCore* getGpuSolverCore() { return mGpuSolverCore;}

		PX_FORCE_INLINE PxgArticulationCore* getArticulationCore() { return mGpuArticulationCore; }

		PX_FORCE_INLINE PxgGpuNarrowphaseCore* getNarrowphaseCore() { return mGpuNpCore; }

		PX_FORCE_INLINE PxgSimulationCore* getSimulationCore() { return mGpuSimulationCore; }

		PX_FORCE_INLINE PxgCudaBroadPhaseSap*	getGpuBroadPhase() { return mGpuBp;  }

		PX_FORCE_INLINE PxgSoftBodyCore*	getGpuSoftBodyCore() { return mGpuSoftBodyCore; }

		PX_FORCE_INLINE PxgFEMClothCore*	getGpuFEMClothCore() { return mGpuFEMClothCore; }

		PX_FORCE_INLINE PxgParticleSystemCore**	getGpuParticleSystemCores() { return mGpuParticleSystemCores.begin(); }
		PX_FORCE_INLINE PxU32	getNbGpuParticleSystemCores() { return mGpuParticleSystemCores.size(); }

		PxgParticleSystemCore* getGpuParticleSystemCore();

		PX_FORCE_INLINE PxU32 getCurrentContactStreamIndex() { return mCurrentContactStream; }

		PX_FORCE_INLINE Cm::FlushPool&	getFlushPool() { return mFlushPool; }

		PX_FORCE_INLINE PxU8* getPatchStream(const PxU32 index) { return mPatchStreamAllocators[index]->mStart; }
		PX_FORCE_INLINE PxU8* getContactStream(const PxU32 index) { return mContactStreamAllocators[index]->mStart; }

		PX_FORCE_INLINE bool enforceConstraintWriteBackToHostCopy() const { return mEnforceConstraintWriteBackToHostCopy; }

		//this method make sure we get PxgSimultionController instead of PxsSimulationController
		PxgSimulationController*			getSimulationController();

		virtual void						setSimulationController(PxsSimulationController* mSimulationController)	PX_OVERRIDE;

		virtual void						mergeResults()	PX_OVERRIDE;
		virtual void						getDataStreamBase(void*& contactStreamBase, void*& patchStreamBase, void*& forceAndIndexStreamBase)	PX_OVERRIDE;
		
		virtual void						updateBodyCore(PxBaseTask* continuation)	PX_OVERRIDE;

		virtual void						update(	Cm::FlushPool& flushPool, PxBaseTask* continuation, PxBaseTask* postPartitioningTask, PxBaseTask* lostTouchTask,
													PxvNphaseImplementationContext* nphase, PxU32 maxPatchesPerCM, PxU32 maxArticulationLinks, PxReal dt,
													const PxVec3& gravity, PxBitMapPinned& changedHandleMap)	PX_OVERRIDE;

		virtual void						updatePostPartitioning(	PxBaseTask* lostTouchTask,
																	PxvNphaseImplementationContext* nphase, PxU32 maxPatchesPerCM, PxU32 maxArticulationLinks, PxReal dt,
																	const PxVec3& gravity, PxBitMapPinned& changedHandleMap)	PX_OVERRIDE;

		virtual void setActiveBreakableConstraintCount(PxU32 activeBreakableConstraintCount) PX_OVERRIDE
		{
			mEnforceConstraintWriteBackToHostCopy = (activeBreakableConstraintCount > 0);
		}

		//this is the pre-prepare code for block format joints loaded from the non-block format joints
		void								doConstraintJointBlockPrePrepGPU();

		void								doStaticArticulationConstraintPrePrep(physx::PxBaseTask* continuation, const PxU32 articulationConstraintBatchIndex, const PxU32 articulationContactBatchIndex);

		void								doStaticRigidConstraintPrePrep(physx::PxBaseTask* continuation);
		
		void								doConstraintSolveGPU(PxU32 maxNodes, PxBitMapPinned& changedHandleMap);

		void								doPostSolveTask(physx::PxBaseTask* continuation);

		virtual void						processPatches(	Cm::FlushPool& flushPool, PxBaseTask* continuation,
															PxsContactManager** lostFoundPatchManagers, PxU32 nbLostFoundPatchManagers, PxsContactManagerOutputCounts* outCounts)	PX_OVERRIDE;

		bool								isTGS() const { return mIsTGS; }
		bool								isExternalForcesEveryTgsIterationEnabled() const { return mIsExternalForcesEveryTgsIterationEnabled; }

		void								doPreIntegrationTaskCommon(physx::PxBaseTask* continuation);

		void 								doConstraintPrePrepCommon(physx::PxBaseTask* continuation);

		void								doConstraintPrePrepGPUCommon(bool hasForceThresholds);

		void								cpuJointPrePrepTask(physx::PxBaseTask* continuation);

		void 								allocateTempPinnedSolverMemoryCommon();

		PX_FORCE_INLINE bool				getEnableDirectGPUAPI() const { return mEnableDirectGPUAPI;	}

		PxvSimStats&			 			getSimStats() { return mSimStats; }

		PxBaseTask*								mLostTouchTask;

		PxU32									mTotalEdges;
		PxU32									mTotalPreviousEdges;

		PxsContactManagerOutputIterator			mOutputIterator;

		PxReal*									mGPURestDistances;
		Sc::ShapeInteraction**					mGPUShapeInteractions;
		PxsTorsionalFrictionData*				mGPUTorsionalData;

		Cm::FlushPool&							mFlushPool;
		bool									mSolvedThisFrame;
		PxgIncrementalPartition					mIncrementalPartition;

		PxPinnedArray<PxNodeIndex>				mActiveNodeIndex;	//this will change everyframe, include rigid bodies and articulation
		
		PxgSolverBody							mWorldSolverBody;
		PxgSolverBodyData						mWorldSolverBodyData;
		PxgSolverBodySleepData					mWorldSolverBodySleepData;
		PxgSolverTxIData						mWorldTxIData;

		PxPinnedArray<PxgSolverBody>			mSolverBodyPool;
		PxPinnedArray<PxAlignedTransform>		mBody2WorldPool;

		//write back from the active articulation
		//each articulation has max 64 links and max 3 * 63 dofsnd 1 wake counter
		//see PxgArticulationLinkJointRootStateData
		PxInt8ArrayPinned						mLinkAndJointAndRootStateDataPool;

		PxPinnedArray<PxgSolverBodySleepData>	mArticulationSleepDataPool;
		PxPinnedArray<Dy::ErrorAccumulator>		mInternalResidualPerArticulationVelIter; //Internal residuals in first half (do not include residuals from external constraints, e. g. contacts or PxConstraints), second half contains residual from contacts
		PxPinnedArray<Dy::ErrorAccumulator>		mInternalResidualPerArticulationPosIter; //Internal residuals in first half (do not include residuals from external constraints, e. g. contacts or PxConstraints), second half contains residual from contacts

		PxInt32ArrayPinned						m1dConstraintBatchIndices;
		PxInt32ArrayPinned						mContactConstraintBatchIndices;
		PxInt32ArrayPinned						mArti1dConstraintBatchIndices;
		PxInt32ArrayPinned						mArtiContactConstraintBatchIndices;

		PxInt32ArrayPinned						mConstraintsPerPartition;
		PxInt32ArrayPinned						mArtiConstraintsPerPartition;

		PxPinnedArray<PxgSolverBodyData>		mSolverBodyDataPool;
		PxPinnedArray<PxgSolverBodySleepData>	mSolverBodySleepDataPool;
		PxPinnedArray<PxgSolverTxIData>			mSolverTxIDataPool;

		PxgPinnedHostLinearMemoryAllocator*		mPinnedMemoryAllocator;

		PxgPinnedHostLinearMemoryAllocator*		mContactStreamAllocators[2];
		PxgPinnedHostLinearMemoryAllocator*		mPatchStreamAllocators[2];
		PxgPinnedHostLinearMemoryAllocator*		mForceStreamAllocator;
		PxgPinnedHostLinearMemoryAllocator*		mFrictionPatchStreamAllocator;

		PxU32									mCurrentContactStream;

		PxgIslandContext*						mIslandContextPool;
		PxU32									mNumIslandContextPool;

		PxU32									mNum1DConstraintBlockPrepPool; //this and mNum1dConstraintBatches is the same, we can get rid of it later

		PxU32									mNumContactManagers;
		PxU32									mNum1DConstraints;

		PxU32									mKinematicCount;
		PxU32									mArticulationCount;
		PxU32									mArticulationStartIndex; //record the start node index in the mActiveNodeIndex for articulation

		PxU32									mBodyCount;

		PxI32									mNumContactBatches;
		PxI32									mNum1dConstraintBatches;
		PxI32									mNumArtiContactBatches;
		PxI32									mNumArti1dConstraintBatches;
		PxI32									mNumStaticArtiContactBatches;
		PxI32									mNumStaticArti1dConstraintBatches;
		PxI32									mNumSelfArtiContactBatches;
		PxI32									mNumSelfArti1dConstraintBatches;

		PxI32									mNumStaticRigidContactBatches;
		PxI32									mNumStaticRigid1dConstraintBatches;

		PxI32									mArtiStaticConstraintBatchOffset;
		PxI32									mArtiStaticContactBatchOffset;

		//KS - we can't know this on CPU because the offset comes after the articulation constraints, which
		//are computed on GPU
		//PxI32									mRigidStaticConstraintBatchOffset;
		//PxI32									mRigidStaticContactBatchOffset;

		PxU32*									mConstraintUniqueIndices;
		PxU32*									mContactUniqueIndices;
		PxU32*									mArtiConstraintUniqueIndices;
		PxU32*									mArtiContactUniqueIndices;
		PxU32*									mArtiStaticConstraintUniqueIndices;
		PxU32*									mArtiStaticContactUniqueIndices;
		PxU32*									mArtiSelfConstraintUniqueIndices;
		PxU32*									mArtiSelfContactUniqueIndices;
		PxU32*									mArtiStaticConstraintStartIndex;

		PxU32*									mRigidStaticConstraintUniqueIndices;
		PxU32*									mRigidStaticContactUniqueIndices;

		PxU32*									mArtiStaticConstraintCount;
		PxU32*									mArtiStaticContactStartIndex;
		PxU32*									mArtiStaticContactCount;

		PxU32*									mRigidStaticConstraintStartIndex;
		PxU32*									mRigidStaticConstraintCount;

		PxI32									mCachedPositionIterations;
		PxI32									mCachedVelocityIterations;

		PxInt32ArrayPinned						mArtiStaticContactCounts;
		PxInt32ArrayPinned						mArtiStaticJointCounts;

		PxInt32ArrayPinned						mArtiStaticContactIndices;
		PxInt32ArrayPinned						mArtiStaticJointIndices;

		PxInt32ArrayPinned						mArtiSelfContactCounts;
		PxInt32ArrayPinned						mArtiSelfJointCounts;

		PxInt32ArrayPinned						mArtiSelfContactIndices;
		PxInt32ArrayPinned						mArtiSelfJointIndices;

		PxInt32ArrayPinned						mRigidStaticContactCounts;
		PxInt32ArrayPinned						mRigidStaticJointCounts;

		PxInt32ArrayPinned						mRigidStaticContactIndices;
		PxInt32ArrayPinned						mRigidStaticJointIndices;

		PxInt32ArrayPinned						mNodeIndicesStagingBuffer;
		PxInt32ArrayPinned						mIslandIds;
		PxInt32ArrayPinned						mIslandStaticTouchCounts;

		//other joint type(not d6) cpu constraints
		PxgConstraintBatchHeader*				mConstraintBatchHeaders;
		PxgConstraintBatchHeader*				mArticConstraintBatchHeaders;

		PxU32									mNumConstraintBatches;

		PxU32									mNumArticConstraintBatches;
		PxU32									mNumArtiStaticConstraintBatches;
		PxU32									mNumArtiSelfConstraintBatches;

		PxU32									mNumRigidStaticConstraintBatches;

		bool									mHasForceThresholds;
		const bool								mIsTGS;
	    bool									mIsExternalForcesEveryTgsIterationEnabled;

		PxgCudaBroadPhaseSap*					mGpuBp;
		PxgGpuNarrowphaseCore*					mGpuNpCore;
		PxgArticulationCore*					mGpuArticulationCore;
		PxgSimulationCore*						mGpuSimulationCore;
		PxgSoftBodyCore*						mGpuSoftBodyCore;
		PxgFEMClothCore*						mGpuFEMClothCore;
		PxArray<PxgParticleSystemCore*>			mGpuParticleSystemCores;
		PxgParticleSystemCore*					mGpuPBDParticleSystemCore;
		
		PxgSolverCore*							mGpuSolverCore;

		PxU32									mMaxNumStaticPartitions;

		const bool								mEnableDirectGPUAPI;
		bool									mRecomputeArticulationBlockFormat;

		// when Direct GPU API is enabled, the constraint writeback data might have to be copied to host to
		// support breakable D6 joints
		bool									mEnforceConstraintWriteBackToHostCopy;

		PxgCpuPreIntegrationTask				mPreIntegrationTask;
		PxgCpuPrepTask							mPrepTask;
		PxgGpuPrePrepTask						mGpuPrePrepTask;
		PxgGpuIntegrationTask					mGpuIntegrationTask;
		PxgGpuTask								mGpuTask; //this task include preprepare constraint, prepare constraint, solve and integration tasks
		PxgPostSolveTask						mPostSolveTask;

		void									doConstraintPrepGPU();
		void									doPreIntegrationGPU(); //this is the pre integration code(copying data from pxgbodysim to solver body data)
		void									doArticulationGPU(); //this is the articulation forward dynamic code
		void									doSoftbodyGPU();//this is the soft body update tetrahedron rotations code
		void									doFEMClothGPU();
		void									doConstraintPrePrepGPU(); //this is the pre-prepare code for block format contacts and non-block format joints
	};
}

#endif


