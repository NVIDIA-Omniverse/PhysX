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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef DY_TGS_DYNAMICS_H
#define DY_TGS_DYNAMICS_H

#include "PxvConfig.h"
#include "CmSpatialVector.h"
#include "CmTask.h"
#include "CmPool.h"
#include "PxcThreadCoherentCache.h"
#include "DyThreadContext.h"
#include "PxcConstraintBlockStream.h"
#include "DySolverBody.h"
#include "DyContext.h"
#include "PxsIslandManagerTypes.h"
#include "PxvNphaseImplementationContext.h"
#include "solver/PxSolverDefs.h"
#include "PxsIslandSim.h"

namespace physx
{
	namespace Cm
	{
		class FlushPool;
	}

	namespace IG
	{
		class SimpleIslandManager;
	}

	class PxsRigidBody;

	struct PxsBodyCore;
	class PxsIslandIndices;
	struct PxsIndexedInteraction;
	struct PxsIndexedContactManager;
	struct PxsExternalAccelerationProvider;

	namespace Dy
	{
		struct SolverContext;

		struct SolverIslandObjectsStep
		{
			PxsRigidBody**				bodies;
			FeatherstoneArticulation**	articulations;
			FeatherstoneArticulation**	articulationOwners;
			PxsIndexedContactManager*	contactManagers;

			const IG::IslandId*			islandIds;
			PxU32						numIslands;
			PxU32*						bodyRemapTable;
			PxU32*						nodeIndexArray;

			PxSolverConstraintDesc*		constraintDescs;
			PxSolverConstraintDesc*		orderedConstraintDescs;
			PxSolverConstraintDesc*		tempConstraintDescs;
			PxConstraintBatchHeader*	constraintBatchHeaders;
			Cm::SpatialVector*			motionVelocities;
			PxsBodyCore**				bodyCoreArray;

			PxU32						solverBodyOffset;
			PxsExternalAccelerationProvider* externalAccelerations;

			SolverIslandObjectsStep() : bodies(NULL), articulations(NULL), articulationOwners(NULL),
				contactManagers(NULL), islandIds(NULL), numIslands(0), nodeIndexArray(NULL), constraintDescs(NULL), motionVelocities(NULL), bodyCoreArray(NULL),
				solverBodyOffset(0), externalAccelerations(NULL)
			{
			}
		};

		struct IslandContextStep
		{
			//The thread context for this island (set in in the island start task, released in the island end task)
			ThreadContext*		mThreadContext;
			PxsIslandIndices	mCounts;
			SolverIslandObjectsStep mObjects;
			PxU32				mPosIters;
			PxU32				mVelIters;
			PxU32				mArticulationOffset;
			PxReal				mStepDt;
			PxReal				mInvStepDt;
			PxReal				mBiasCoefficient;
			PxI32				mSharedSolverIndex;
			PxI32				mSolvedCount;
			PxI32				mSharedRigidBodyIndex;
			PxI32				mRigidBodyIntegratedCount;
			PxI32				mSharedArticulationIndex;
			PxI32				mArticulationIntegratedCount;
			PxI32				mSharedGravityIndex;
			PxI32				mGravityIntegratedCount;
		};

		struct SolverIslandObjectsStep;

		class SolverBodyVelDataPool : public PxArray<PxTGSSolverBodyVel, PxAlignedAllocator<128, PxReflectionAllocator<PxTGSSolverBodyVel> > >
		{
			PX_NOCOPY(SolverBodyVelDataPool)
		public:
			SolverBodyVelDataPool() {}
		};

		class SolverBodyTxInertiaPool : public PxArray<PxTGSSolverBodyTxInertia, PxAlignedAllocator<128, PxReflectionAllocator<PxTGSSolverBodyTxInertia> > >
		{
			PX_NOCOPY(SolverBodyTxInertiaPool)
		public:
			SolverBodyTxInertiaPool() {}
		};

		class SolverBodyDataStepPool : public PxArray<PxTGSSolverBodyData, PxAlignedAllocator<128, PxReflectionAllocator<PxTGSSolverBodyData> > >
		{
			PX_NOCOPY(SolverBodyDataStepPool)
		public:
			SolverBodyDataStepPool() {}
		};

		class SolverStepConstraintDescPool : public PxArray<PxSolverConstraintDesc, PxAlignedAllocator<128, PxReflectionAllocator<PxSolverConstraintDesc> > >
		{
			PX_NOCOPY(SolverStepConstraintDescPool)
		public:
			SolverStepConstraintDescPool() { }
		};

#if PX_VC 
#pragma warning(push)
#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif

class DynamicsTGSContext : public Context
{
	PX_NOCOPY(DynamicsTGSContext)
public:
									DynamicsTGSContext(PxcNpMemBlockPool* memBlockPool,
										PxcScratchAllocator& scratchAllocator,
										Cm::FlushPool& taskPool,
										PxvSimStats& simStats,
										PxTaskManager* taskManager,
										PxVirtualAllocatorCallback* allocator,
										PxsMaterialManager* materialManager,
										IG::SimpleIslandManager* islandManager,
										PxU64 contextID,
										bool enableStabilization,
										bool useEnhancedDeterminism,
										PxReal lengthScale,
										bool isExternalForcesEveryTgsIterationEnabled,
										bool isResidualReportingEnabled
										);

	virtual								~DynamicsTGSContext();

	// Context
	virtual	void						destroy()	PX_OVERRIDE;
	virtual void						update(IG::SimpleIslandManager& simpleIslandManager, PxBaseTask* continuation, PxBaseTask* lostTouchTask,
										PxvNphaseImplementationContext* nphase, PxU32 maxPatchesPerCM, PxU32 maxArticulationLinks, PxReal dt, const PxVec3& gravity, PxBitMapPinned& changedHandleMap)	PX_OVERRIDE;
	virtual void						mergeResults()	PX_OVERRIDE;
	virtual void						setSimulationController(PxsSimulationController* simulationController)	PX_OVERRIDE	{ mSimulationController = simulationController; }
	virtual PxSolverType::Enum			getSolverType()	const	PX_OVERRIDE	{ return PxSolverType::eTGS;	}
		
	//~Context

	/**
	\brief Allocates and returns a thread context object.
	\return A thread context.
	*/
	PX_FORCE_INLINE ThreadContext*		getThreadContext()	{ return mThreadContextPool.get();	}

	/**
	\brief Returns a thread context to the thread context pool.
	\param[in] context The thread context to return to the thread context pool.
	*/
					void				putThreadContext(ThreadContext* context)	{ mThreadContextPool.put(context);	}

	PX_FORCE_INLINE Cm::FlushPool&		getTaskPool()					{ return mTaskPool;			}
	PX_FORCE_INLINE ThresholdStream&	getThresholdStream()			{ return *mThresholdStream; }
	PX_FORCE_INLINE PxvSimStats&		getSimStats()					{ return mSimStats;			}
	PX_FORCE_INLINE	PxU32				getKinematicCount()		const	{ return mKinematicCount;	}

					void				updatePostKinematic(IG::SimpleIslandManager& simpleIslandManager, PxBaseTask* continuation, PxBaseTask* lostTouchTask, PxU32 maxLinks);
protected:

	// PT: TODO: the thread stats are missing for TGS
/*#if PX_ENABLE_SIM_STATS
					void			addThreadStats(const ThreadContext::ThreadSimStats& stats);
#else
					PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif*/

			// Solver helper-methods
			/**
			\brief Computes the unconstrained velocity for a given PxsRigidBody
			\param[in] atom The PxsRigidBody
			*/
			void								computeUnconstrainedVelocity(PxsRigidBody* atom)	const;

			/**
			\brief fills in a PxSolverConstraintDesc from an indexed interaction
			\param[in,out] desc The PxSolverConstraintDesc
			\param[in] constraint The PxsIndexedInteraction
			*/
			void								setDescFromIndices(PxSolverConstraintDesc& desc, const IG::IslandSim& islandSim,
				const PxsIndexedInteraction& constraint, PxU32 solverBodyOffset, PxTGSSolverBodyVel* solverBodies);


			void								setDescFromIndices(PxSolverConstraintDesc& desc, IG::EdgeIndex edgeIndex,
				const IG::SimpleIslandManager& islandManager, PxU32* bodyRemapTable, PxU32 solverBodyOffset, PxTGSSolverBodyVel* solverBodies);


			void solveIsland(const SolverIslandObjectsStep& objects,
				const PxsIslandIndices& counts,
				PxU32 solverBodyOffset,
				IG::SimpleIslandManager& islandManager,
				PxU32* bodyRemapTable, PxsMaterialManager* materialManager,
				PxsContactManagerOutputIterator& iterator,
				PxBaseTask* continuation);

			void prepareBodiesAndConstraints(const SolverIslandObjectsStep& objects,
				IG::SimpleIslandManager& islandManager,
				IslandContextStep& islandContext);

			void setupDescs(IslandContextStep& islandContext, const SolverIslandObjectsStep& objects, PxU32* mBodyRemapTable, PxU32 mSolverBodyOffset,
				PxsContactManagerOutputIterator& outputs);

			void preIntegrateBodies(PxsBodyCore** bodyArray, PxsRigidBody** originalBodyArray,
				PxTGSSolverBodyVel* solverBodyVelPool, PxTGSSolverBodyTxInertia* solverBodyTxInertia, PxTGSSolverBodyData* solverBodyDataPool2,
				PxU32* nodeIndexArray, PxU32 bodyCount, const PxVec3& gravity, PxReal dt, PxU32& posIters, PxU32& velIters, PxU32 iteration);

			void setupArticulations(IslandContextStep& islandContext, const PxVec3& gravity, PxReal dt, PxU32& posIters, PxU32& velIters, PxBaseTask* continuation);

			PxU32 setupArticulationInternalConstraints(IslandContextStep& islandContext, PxReal dt, PxReal invStepDt);

			void createSolverConstraints(PxSolverConstraintDesc* contactDescPtr, PxConstraintBatchHeader* headers, PxU32 nbHeaders,
				PxsContactManagerOutputIterator& outputs, Dy::ThreadContext& islandThreadContext, Dy::ThreadContext& threadContext, PxReal stepDt, PxReal totalDt, 
				PxReal invStepDt, PxReal biasCoefficient);

			void solveConstraintsIteration(const PxSolverConstraintDesc* const contactDescPtr, const PxConstraintBatchHeader* const batchHeaders, PxU32 nbHeaders, PxReal invStepDt,
				const PxTGSSolverBodyTxInertia* const solverTxInertia, PxReal elapsedTime, PxReal minPenetration, SolverContext& cache);

			template <bool TSync>
			void solveConcludeConstraintsIteration(const PxSolverConstraintDesc* const contactDescPtr, const PxConstraintBatchHeader* const batchHeaders, PxU32 nbHeaders,
				PxTGSSolverBodyTxInertia* solverTxInertia, PxReal elapsedTime, SolverContext& cache, PxU32 iterCount);

			template <bool Sync>
			void parallelSolveConstraints(const PxSolverConstraintDesc* const contactDescPtr, const PxConstraintBatchHeader* const batchHeaders, PxU32 nbHeaders, PxTGSSolverBodyTxInertia* solverTxInertia,
				PxReal elapsedTime, PxReal minPenetration, SolverContext& cache, PxU32 iterCount);

			void writebackConstraintsIteration(const PxConstraintBatchHeader* const hdrs, const PxSolverConstraintDesc* const contactDescPtr, PxU32 nbHeaders, SolverContext& cache);

			void parallelWritebackConstraintsIteration(const PxSolverConstraintDesc* const contactDescPtr, const PxConstraintBatchHeader* const batchHeaders, PxU32 nbHeaders, SolverContext& cache);

			void applySubstepGravity(PxsRigidBody** bodies, PxsExternalAccelerationProvider& externalAccelerations,
				PxU32 count, PxTGSSolverBodyVel* vels, PxReal dt, PxTGSSolverBodyTxInertia* PX_RESTRICT txInertias, PxU32* nodeIndexArray);

			void applySubstepGravityParallel(const SolverIslandObjectsStep& objects, PxTGSSolverBodyVel* solverVels, const PxU32 bodyOffset, PxReal stepDt,
				const PxU32 nbBodies, PxU32& startGravityIdx, PxU32& nbGravityRemaining, PxU32& targetGravityProgressCount,
				PxI32* gravityProgressCount, PxI32* gravityCounts, PxU32 unrollSize);

			void applyArticulationSubstepGravityParallel(PxU32& startArticulationIdx, PxU32& targetArticulationProgressCount, PxI32* articulationProgressCount,
				PxI32* articulationIntegrationCounts, ArticulationSolverDesc* articulationDescs, PxU32 nbArticulations, PxReal stepDt, ThreadContext& threadContext);

			void integrateBodies(const SolverIslandObjectsStep& objects,
				PxU32 count, PxTGSSolverBodyVel* vels,
				PxTGSSolverBodyTxInertia* txInertias, const PxTGSSolverBodyData*const bodyDatas, PxReal dt, PxReal invTotalDt, bool averageBodies,
				PxReal ratio);

			void integrateBodiesAndApplyGravity(const SolverIslandObjectsStep& objects,
				PxU32 count, PxTGSSolverBodyVel* vels,
				PxTGSSolverBodyTxInertia* txInertias, const PxTGSSolverBodyData*const bodyDatas, PxReal dt, PxReal invTotalDt, bool averageBodies,
				PxReal ratio, PxU32 posIters);

			void parallelIntegrateBodies(PxTGSSolverBodyVel* vels, PxTGSSolverBodyTxInertia* txInertias,
				const PxTGSSolverBodyData* const bodyDatas, PxU32 count, PxReal dt, PxU32 iteration, PxReal invTotalDt, bool average,
				PxReal ratio);

			void copyBackBodies(const SolverIslandObjectsStep& objects,
				PxTGSSolverBodyVel* vels, PxTGSSolverBodyTxInertia* txInertias,
				PxTGSSolverBodyData* solverBodyData, PxReal invDt,	IG::IslandSim& islandSim,
				PxU32 startIdx, PxU32 endIdx);

			void updateArticulations(Dy::ThreadContext& threadContext, PxU32 startIdx, PxU32 endIdx, PxReal dt);

			void stepArticulations(Dy::ThreadContext& threadContext, const PxsIslandIndices& counts, PxReal dt, PxReal stepInvDt);

			void applyArticulationTgsSubstepForces(Dy::ThreadContext& threadContext, PxU32 numArticulations, PxReal stepDt);

			void iterativeSolveIsland(const SolverIslandObjectsStep& objects, const PxsIslandIndices& counts, ThreadContext& mThreadContext,
				PxReal stepDt, PxReal invStepDt, PxU32 posIters, PxU32 velIters, SolverContext& cache, PxReal ratio,
				PxReal biasCoefficient);

			void iterativeSolveIslandParallel(const SolverIslandObjectsStep& objects, const PxsIslandIndices& counts, ThreadContext& mThreadContext,
				PxReal stepDt, PxU32 posIters, PxU32 velIters, PxI32* solverCounts, PxI32* integrationCounts, PxI32* articulationIntegrationCounts, PxI32* gravityCounts,
				PxI32* solverProgressCount, PxI32* integrationProgressCount, PxI32* articulationProgressCount, PxI32* gravityProgressCount, PxU32 solverUnrollSize, PxU32 integrationUnrollSize,
				PxReal ratio, PxReal biasCoefficient);

			void endIsland(ThreadContext& mThreadContext);

			void finishSolveIsland(ThreadContext& mThreadContext, const SolverIslandObjectsStep& objects,
				const PxsIslandIndices& counts, IG::SimpleIslandManager& islandManager, PxBaseTask* continuation);

			/**
			\brief Resets the thread contexts
			*/
			void									resetThreadContexts();

			/**
			\brief Returns the scratch memory allocator.
			\return The scratch memory allocator.
			*/
			PX_FORCE_INLINE PxcScratchAllocator&	getScratchAllocator() { return mScratchAllocator; }

			//Data

			PxTGSSolverBodyVel						mWorldSolverBodyVel;
			PxTGSSolverBodyTxInertia				mWorldSolverBodyTxInertia;
			PxTGSSolverBodyData						mWorldSolverBodyData2;

			/**
			\brief A thread context pool
			*/
			PxcThreadCoherentCache<ThreadContext, PxcNpMemBlockPool> mThreadContextPool;

			/**
			\brief Solver constraint desc array
			*/
			SolverStepConstraintDescPool	mSolverConstraintDescPool;

			SolverStepConstraintDescPool	mOrderedSolverConstraintDescPool;

			SolverStepConstraintDescPool	mTempSolverConstraintDescPool;

			PxArray<PxConstraintBatchHeader> mContactConstraintBatchHeaders;

			/**
			\brief Array of motion velocities for all bodies in the scene.
			*/
			PxArray<Cm::SpatialVector> mMotionVelocityArray;

			/**
			\brief Array of body core pointers for all bodies in the scene.
			*/
			PxArray<PxsBodyCore*>	mBodyCoreArray;

			/**
			\brief Array of rigid body pointers for all bodies in the scene.
			*/
			PxArray<PxsRigidBody*> mRigidBodyArray;

			/**
			\brief Array of articulationpointers for all articulations in the scene.
			*/
			PxArray<FeatherstoneArticulation*> mArticulationArray;

			SolverBodyVelDataPool				mSolverBodyVelPool;

			SolverBodyTxInertiaPool				mSolverBodyTxInertiaPool;

			SolverBodyDataStepPool				mSolverBodyDataPool2;

			ThresholdStream*					mExceededForceThresholdStream[2]; //this store previous and current exceeded force thresholdStream	

			PxArray<PxU32>						mExceededForceThresholdStreamMask;

			PxArray<PxU32>						mSolverBodyRemapTable;				//Remaps from the "active island" index to the index within a solver island

			PxArray<PxU32>						mNodeIndexArray;					//island node index

			PxArray<PxsIndexedContactManager> mContactList;

			/**
			\brief The total number of kinematic bodies in the scene
			*/
			PxU32						mKinematicCount;

			/**
			\brief Atomic counter for the number of threshold stream elements.
			*/
			PxI32						mThresholdStreamOut;

			PxsMaterialManager*			mMaterialManager;

			PxsContactManagerOutputIterator mOutputIterator;

		private:
			//private:
			PxcScratchAllocator&						mScratchAllocator;
			Cm::FlushPool&								mTaskPool;
			PxTaskManager*								mTaskManager;
			PxU32										mCurrentIndex; // this is the index point to the current exceeded force threshold stream
			bool										mIsExternalForcesEveryTgsIterationEnabled;

			friend class SetupDescsTask;
			friend class PreIntegrateTask;
			friend class SetupArticulationTask;
			friend class SetupArticulationInternalConstraintsTask;
			friend class SetupSolverConstraintsTask;
			friend class SolveIslandTask;
			friend class EndIslandTask;
			friend class SetupSolverConstraintsSubTask;
			friend class ParallelSolveTask;
			friend class PreIntegrateParallelTask;
			friend class CopyBackTask;
			friend class UpdateArticTask;
			friend class FinishSolveIslandTask;
		};

#if PX_VC 
#pragma warning(pop)
#endif

	}
}

#endif

