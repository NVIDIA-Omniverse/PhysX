// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_TGS_DYNAMICS_H
#define DY_TGS_DYNAMICS_H

#include "DyDynamicsBase.h"
#include "PxPhysXConfig.h"
#include "CmSpatialVector.h"
#include "CmTask.h"
#include "CmPool.h"
#include "PxcThreadCoherentCache.h"
#include "PxcConstraintBlockStream.h"
#include "DySolverBody.h"
#include "PxsIslandSim.h"
#include "DyCpuGpuBiasCoefficient.h"

namespace physx
{
	class PxsRigidBody;

	struct PxsBodyCore;
	class PxsIslandIndices;
	struct PxsIndexedInteraction;
	struct PxsIndexedContactManager;
	struct PxsExternalAccelerationProvider;

	namespace Dy
	{
		struct SolverContext;

		// PT: TODO: what is const and what is not?
		struct SolverIslandObjectsStep
		{
			PxsRigidBody**				bodies;
			FeatherstoneArticulation**	articulations;
			FeatherstoneArticulation**	articulationOwners;
			PxsIndexedContactManager*	contactManagers;	// PT: points to DynamicsTGSContext::mContactList

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
			BiasCoefficientCollection mBiasCoefficients;
			//PT: each counter is padded out to its own cache line: they are all incremented atomically
			//and spin-read (WAIT_FOR_PROGRESS) by every solver thread, and previously shared cache lines
			//both with each other and with the read-only island data above (false sharing). Explicit
			//padding rather than PX_ALIGN because the struct is not allocated with 64-byte alignment.
			PxU8				mPad0[64];
			PxI32				mSharedSolverIndex;				PxU8 mPad1[60];
			PxI32				mSolvedCount;					PxU8 mPad2[60];
			PxI32				mSharedRigidBodyIndex;			PxU8 mPad3[60];
			PxI32				mRigidBodyIntegratedCount;		PxU8 mPad4[60];
			PxI32				mSharedArticulationIndex;		PxU8 mPad5[60];
			PxI32				mArticulationIntegratedCount;	PxU8 mPad6[60];
			PxI32				mSharedGravityIndex;			PxU8 mPad7[60];
			PxI32				mGravityIntegratedCount;		PxU8 mPad8[60];
		};

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

class DynamicsTGSContext : public DynamicsContextBase
{
	PX_NOCOPY(DynamicsTGSContext)
public:
										DynamicsTGSContext(	PxcNpMemBlockPool* memBlockPool, Cm::FlushPool& taskPool,
															PxvSimStats& simStats, Cm::VirtualAllocatorCallback& allocator,
															PxsMaterialManager* materialManager, IG::SimpleIslandManager& islandManager,
															PxU64 contextID, PxReal lengthScale, PxSceneFlags sceneFlags);
	virtual								~DynamicsTGSContext();

	// Context
	virtual	void						destroy()	PX_OVERRIDE;
	virtual void						update(	Cm::FlushPool& flushPool, PxBaseTask* continuation, PxBaseTask* postPartitioningTask, PxBaseTask* lostTouchTask,
												PxvNphaseImplementationContext* nphase, PxU32 maxPatchesPerCM, PxU32 maxArticulationLinks,
												PxReal dt, const PxVec3& gravity, Cm::PinnableBitMap& changedHandleMap)	PX_OVERRIDE;
	virtual void						mergeResults()	PX_OVERRIDE;
	virtual void						setSimulationController(PxsSimulationController* simulationController)	PX_OVERRIDE	{ mSimulationController = simulationController; }
	virtual PxSolverType::Enum			getSolverType()	const	PX_OVERRIDE	{ return PxSolverType::eTGS;	}
		
	//~Context

					void				updatePostKinematic(PxBaseTask* continuation, PxBaseTask* lostTouchTask, PxU32 maxLinks);
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

			void								setDescFromIndices_Contacts(PxSolverConstraintDesc& desc, const IG::IslandSim& islandSim,
																			const PxsIndexedInteraction& constraint, PxU32 solverBodyOffset, PxTGSSolverBodyVel* solverBodies);

			void								setDescFromIndices_Constraints(	PxSolverConstraintDesc& desc, const IG::IslandSim& islandSim, IG::EdgeIndex edgeIndex,
																				const PxU32* bodyRemapTable, PxU32 solverBodyOffset, PxTGSSolverBodyVel* solverBodies);

			void solveIsland(const SolverIslandObjectsStep& objects,
				const PxsIslandIndices& counts,
				PxU32 solverBodyOffset,
				PxU32* bodyRemapTable, PxsMaterialManager* materialManager,
				PxsContactManagerOutputIterator& iterator,
				PxBaseTask* continuation);

			void prepareBodiesAndConstraints(const SolverIslandObjectsStep& objects, IslandContextStep& islandContext);

			void setupDescs(IslandContextStep& islandContext, const SolverIslandObjectsStep& objects, PxU32* mBodyRemapTable, PxU32 mSolverBodyOffset,
				PxsContactManagerOutputIterator& outputs);

			void preIntegrateBodies(PxsBodyCore** bodyArray, PxsRigidBody** originalBodyArray,
				PxTGSSolverBodyVel* solverBodyVelPool, PxTGSSolverBodyTxInertia* solverBodyTxInertia, PxTGSSolverBodyData* solverBodyDataPool2,
				PxU32* nodeIndexArray, PxU32 bodyCount, const PxVec3& gravity, PxReal dt, PxU32& posIters, PxU32& velIters, PxU32 iteration);

			void setupArticulations(IslandContextStep& islandContext, const PxVec3& gravity, PxReal dt, PxU32& posIters, PxU32& velIters, PxBaseTask* continuation);

			void setupArticulationInternalConstraints(IslandContextStep& islandContext, PxReal dt, PxReal invStepDt);

			void createSolverConstraints(PxSolverConstraintDesc* contactDescPtr, PxConstraintBatchHeader* headers, PxU32 nbHeaders,
				PxsContactManagerOutputIterator& outputs, Dy::ThreadContext& islandThreadContext, Dy::ThreadContext& threadContext, PxReal stepDt, PxReal totalDt, 
				PxReal invStepDt, PxReal rigidContactBiasCoefficient, PxReal jointBiasCoefficient);

			void writebackConstraintsIteration(const PxConstraintBatchHeader* const hdrs, const PxSolverConstraintDesc* const contactDescPtr, PxU32 nbHeaders);

			void parallelWritebackConstraintsIteration(const PxSolverConstraintDesc* const contactDescPtr, const PxConstraintBatchHeader* const batchHeaders, PxU32 nbHeaders);

			void applySubstepGravity(PxsRigidBody** bodies, PxsExternalAccelerationProvider& externalAccelerations,
				PxU32 count, PxTGSSolverBodyVel* vels, PxReal dt, PxTGSSolverBodyTxInertia* PX_RESTRICT txInertias, PxU32* nodeIndexArray);

			void applySubstepGravityParallel(const SolverIslandObjectsStep& objects, PxTGSSolverBodyVel* solverVels, const PxU32 bodyOffset, PxReal stepDt,
				const PxU32 nbBodies, PxU32& startGravityIdx, PxU32& nbGravityRemaining, PxU32& targetGravityProgressCount,
				PxI32* gravityProgressCount, PxI32* gravityCounts, PxU32 unrollSize);

			void applyArticulationSubstepGravityParallel(PxU32& startArticulationIdx, PxU32& targetArticulationProgressCount, PxI32* articulationProgressCount,
				PxI32* articulationIntegrationCounts, FeatherstoneArticulation** articulationDescs, PxU32 nbArticulations, PxReal stepDt, ThreadContext& threadContext);

			void integrateBodies(PxU32 count, PxTGSSolverBodyVel* vels, PxTGSSolverBodyTxInertia* txInertias, PxReal dt);

			void integrateBodiesAndApplyGravity(const SolverIslandObjectsStep& objects, PxU32 count, PxTGSSolverBodyVel* vels,
				PxTGSSolverBodyTxInertia* txInertias, PxReal dt, PxU32 posIters);

			void copyBackBodies(const SolverIslandObjectsStep& objects,
				PxTGSSolverBodyVel* vels, PxTGSSolverBodyTxInertia* txInertias,
				PxTGSSolverBodyData* solverBodyData, PxReal invDt,	IG::IslandSim& islandSim,
				PxU32 startIdx, PxU32 endIdx);

			void updateArticulations(Dy::ThreadContext& threadContext, PxU32 startIdx, PxU32 endIdx, PxReal dt);

			void stepArticulations(Dy::ThreadContext& threadContext, const PxsIslandIndices& counts, PxReal dt);

			void applyArticulationTgsSubstepForces(Dy::ThreadContext& threadContext, PxU32 numArticulations, PxReal stepDt);

			void iterativeSolveIsland(const SolverIslandObjectsStep& objects, const PxsIslandIndices& counts, ThreadContext& mThreadContext,
				PxReal stepDt, PxReal invStepDt, PxReal totalDt, PxU32 posIters, PxU32 velIters, PxReal articulationBiasCoefficient);

			void iterativeSolveIslandParallel(const SolverIslandObjectsStep& objects, const PxsIslandIndices& counts, ThreadContext& mThreadContext,
				PxReal stepDt, PxReal totalDt, PxU32 posIters, PxU32 velIters, PxReal articulationBiasCoefficient,
				PxI32* solverCounts, PxI32* integrationCounts, PxI32* articulationIntegrationCounts, PxI32* gravityCounts,
				PxI32* solverProgressCount, PxI32* integrationProgressCount, PxI32* articulationProgressCount, PxI32* gravityProgressCount, PxU32 solverUnrollSize, PxU32 integrationUnrollSize);

			void endIsland(ThreadContext& mThreadContext);

			void finishSolveIsland(ThreadContext& mThreadContext, const SolverIslandObjectsStep& objects,
				const PxsIslandIndices& counts, IG::SimpleIslandManager& islandManager, PxBaseTask* continuation);

			PxTGSSolverBodyVel				mWorldSolverBodyVel;
			PxTGSSolverBodyTxInertia		mWorldSolverBodyTxInertia;
			PxTGSSolverBodyData				mWorldSolverBodyData2;

			/**
			\brief Solver constraint desc array
			*/
			SolverStepConstraintDescPool	mSolverConstraintDescPool;

			SolverStepConstraintDescPool	mOrderedSolverConstraintDescPool;

			SolverStepConstraintDescPool	mTempSolverConstraintDescPool;

			SolverBodyVelDataPool			mSolverBodyVelPool;

			SolverBodyTxInertiaPool			mSolverBodyTxInertiaPool;

			SolverBodyDataStepPool			mSolverBodyDataPool2;			

		private:
			bool	mIsExternalForcesEveryTgsIterationEnabled;

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

