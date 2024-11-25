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

#ifndef DY_DYNAMICS_H
#define DY_DYNAMICS_H

#include "PxvConfig.h"
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
struct PxSolverConstraintDesc;

namespace Cm
{
	class SpatialVector;
}

namespace Dy
{
	class SolverCore;
	struct SolverIslandParams;
	class DynamicsContext;

#define SOLVER_PARALLEL_METHOD_ARGS	\
	DynamicsContext&	context,	\
	SolverIslandParams& params,		\
	IG::IslandSim& islandSim

/**
\brief Solver body pool (array) that enforces 128-byte alignment for base address of array.
\note This reduces cache misses on platforms with 128-byte-size cache lines by aligning the start of the array to the beginning of a cache line.
*/
class SolverBodyPool : public PxArray<PxSolverBody, PxAlignedAllocator<128, PxReflectionAllocator<PxSolverBody> > > 
{ 
	PX_NOCOPY(SolverBodyPool)
public:
	SolverBodyPool() {}
};

/**
\brief Solver body data pool (array) that enforces 128-byte alignment for base address of array.
\note This reduces cache misses on platforms with 128-byte-size cache lines by aligning the start of the array to the beginning of a cache line.
*/
class SolverBodyDataPool : public PxArray<PxSolverBodyData, PxAlignedAllocator<128, PxReflectionAllocator<PxSolverBodyData> > >
{
	PX_NOCOPY(SolverBodyDataPool)
public:
	SolverBodyDataPool() {}
};

class SolverConstraintDescPool : public PxArray<PxSolverConstraintDesc, PxAlignedAllocator<128, PxReflectionAllocator<PxSolverConstraintDesc> > >
{
	PX_NOCOPY(SolverConstraintDescPool)
public:
	SolverConstraintDescPool() { }
};

/**
\brief Encapsulates an island's context
*/

struct IslandContext
{
	//The thread context for this island (set in in the island start task, released in the island end task)
	ThreadContext*		mThreadContext;
	PxsIslandIndices	mCounts;
};

/**
\brief Encapsules the data used by the constraint solver.
*/

#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif

class DynamicsContext : public Context
{
	PX_NOCOPY(DynamicsContext)
public:
	
										DynamicsContext(PxcNpMemBlockPool* memBlockPool,
														PxcScratchAllocator& scratchAllocator,
														Cm::FlushPool& taskPool,
														PxvSimStats& simStats,
														PxTaskManager* taskManager,
														PxVirtualAllocatorCallback* allocator,
														PxsMaterialManager* materialManager,
														IG::SimpleIslandManager& islandManager,
														PxU64 contextID,
														bool enableStabilization,
														bool useEnhancedDeterminism,
														PxReal maxBiasCoefficient,
														bool frictionEveryIteration,
														PxReal lengthScale,
														bool isResidualReportingEnabled
														);

	virtual								~DynamicsContext();

	// Context
	virtual	void						destroy()	PX_OVERRIDE;
	virtual void						update(PxBaseTask* continuation, PxBaseTask* lostTouchTask,
										PxvNphaseImplementationContext* nPhase, PxU32 maxPatchesPerCM, PxU32 maxArticulationLinks, PxReal dt, const PxVec3& gravity, PxBitMapPinned& changedHandleMap)	PX_OVERRIDE;
	virtual void						mergeResults()	PX_OVERRIDE;
	virtual void						setSimulationController(PxsSimulationController* simulationController )	PX_OVERRIDE	{ mSimulationController = simulationController; }
	virtual PxSolverType::Enum			getSolverType()	const	PX_OVERRIDE	{ return PxSolverType::ePGS;	}
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
	PX_FORCE_INLINE ThresholdStream&	getThresholdStream()			{ return *mThresholdStream;	}
	PX_FORCE_INLINE PxvSimStats&		getSimStats()					{ return mSimStats;			}
	PX_FORCE_INLINE	PxU32				getKinematicCount()		const	{ return mKinematicCount;	}

					void				updatePostKinematic(IG::SimpleIslandManager& simpleIslandManager, PxBaseTask* continuation, PxBaseTask* lostTouchTask, PxU32 maxLinks);
protected:

#if PX_ENABLE_SIM_STATS
					void				addThreadStats(const ThreadContext::ThreadSimStats& stats);
#else
					PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

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
										const PxsIndexedInteraction& constraint, PxU32 solverBodyOffset);


	void								setDescFromIndices(PxSolverConstraintDesc& desc, IG::EdgeIndex edgeIndex,
											const IG::SimpleIslandManager& islandManager, PxU32* bodyRemapTable, PxU32 solverBodyOffset);

	/**
	\brief Compute the unconstrained velocity for set of bodies in parallel. This function may spawn additional tasks.
	\param[in] dt The timestep
	\param[in] bodyArray The array of body cores
	\param[in] originalBodyArray The array of PxsRigidBody
	\param[in] nodeIndexArray The array of island node index
	\param[in] bodyCount The number of bodies
	\param[out] solverBodyPool The pool of solver bodies. These are synced with the corresponding body in bodyArray.
	\param[out] solverBodyDataPool The pool of solver body data. These are synced with the corresponding body in bodyArray
	\param[out] motionVelocityArray The motion velocities for the bodies
	\param[out] maxSolverPositionIterations The maximum number of position iterations requested by any body in the island
	\param[out] maxSolverVelocityIterations The maximum number of velocity iterations requested by any body in the island
	\param[out] integrateTask The continuation task for any tasks spawned by this function.
	*/
	void								preIntegrationParallel(
											   PxF32 dt,
											   PxsBodyCore*const* bodyArray,					// INOUT: core body attributes
											   PxsRigidBody*const* originalBodyArray,			// IN: original body atom names
											   PxU32 const* nodeIndexArray,						// IN: island node index
											   PxU32 bodyCount,									// IN: body count
											   PxSolverBody* solverBodyPool,					// IN: solver atom pool (space preallocated)
											   PxSolverBodyData* solverBodyDataPool,
											   Cm::SpatialVector* motionVelocityArray,			// OUT: motion velocities
											   PxU32& maxSolverPositionIterations,
											   PxU32& maxSolverVelocityIterations,
											   PxBaseTask& integrateTask
											   );

	/**
	\brief Solves an island in parallel.

	\param[in] params Solver parameter structure
	*/

	void								solveParallel(SolverIslandParams& params, IG::IslandSim& islandSim, Cm::SpatialVectorF* deltaV, Dy::ErrorAccumulatorEx* errorAccumulator);

	void								integrateCoreParallel(SolverIslandParams& params, Cm::SpatialVectorF* deltaV, IG::IslandSim& islandSim);

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

	/**
	\brief Body to represent the world static body.
	*/
	PX_ALIGN(16, PxSolverBody				mWorldSolverBody);
	/**
	\brief Body data to represent the world static body.
	*/
	PX_ALIGN(16, PxSolverBodyData			mWorldSolverBodyData);

	/**
	\brief A thread context pool
	*/
	PxcThreadCoherentCache<ThreadContext, PxcNpMemBlockPool> mThreadContextPool;

	/**
	\brief Solver constraint desc array
	*/
	SolverConstraintDescPool	mSolverConstraintDescPool;

	/**
	\brief Ordered solver constraint desc array (after partitioning)
	*/
	SolverConstraintDescPool	mOrderedSolverConstraintDescPool;

	/**
	\brief A temporary array of constraint descs used for partitioning
	*/
	SolverConstraintDescPool	mTempSolverConstraintDescPool;

	/**
	\brief An array of contact constraint batch headers
	*/
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
	\brief Array of articulation pointers for all articulations in the scene.
	*/
	PxArray<FeatherstoneArticulation*> mArticulationArray;

	/**
	\brief Global pool for solver bodies. Kinematic bodies are at the start, and then dynamic bodies
	*/
	SolverBodyPool			mSolverBodyPool;
	/**
	\brief Global pool for solver body data. Kinematic bodies are at the start, and then dynamic bodies
	*/
	SolverBodyDataPool		mSolverBodyDataPool;

	ThresholdStream*		mExceededForceThresholdStream[2]; //this store previous and current exceeded force thresholdStream	

	PxArray<PxU32>		mExceededForceThresholdStreamMask;

	/**
	\brief Interface to the solver core.
	\note We currently only support PxsSolverCoreSIMD. Other cores may be added in future releases.
	*/
	SolverCore*				mSolverCore[PxFrictionType::eFRICTION_COUNT];

	PxArray<PxU32>		mSolverBodyRemapTable;				//Remaps from the "active island" index to the index within a solver island

	PxArray<PxU32>		mNodeIndexArray;					//island node index

	PxArray<PxsIndexedContactManager>	mContactList;
	
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
	PxcScratchAllocator&			mScratchAllocator;
	Cm::FlushPool&					mTaskPool;
	PxTaskManager*					mTaskManager;
	PxU32							mCurrentIndex; // this is the index point to the current exceeded force threshold stream

	protected:

	friend class PxsSolverStartTask;
	friend class PxsSolverAticulationsTask;
	friend class PxsSolverSetupConstraintsTask;
	friend class PxsSolverCreateFinalizeConstraintsTask;	
	friend class PxsSolverConstraintPartitionTask;
	friend class PxsSolverSetupSolveTask;
	friend class PxsSolverIntegrateTask;
	friend class PxsSolverEndTask;
	friend class PxsSolverConstraintPostProcessTask;
	friend class PxsForceThresholdTask;
	friend class SolverArticulationUpdateTask;

	friend void solveParallel(SOLVER_PARALLEL_METHOD_ARGS);
};

#if PX_VC 
    #pragma warning(pop)
#endif

}
}

#endif
