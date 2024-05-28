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

#ifndef DY_THREAD_CONTEXT_H
#define DY_THREAD_CONTEXT_H

#include "foundation/PxTransform.h"
#include "geomutils/PxContactBuffer.h"

#include "PxvConfig.h"
#include "PxvDynamics.h"
#include "PxcThreadCoherentCache.h"
#include "PxcConstraintBlockStream.h"
#include "foundation/PxBitMap.h"
#include "DyVArticulation.h"
#include "DyFrictionPatchStreamPair.h"
#include "DySolverConstraintDesc.h"
#include "DyCorrelationBuffer.h"
#include "foundation/PxAllocator.h"
#include "DyResidualAccumulator.h"

namespace physx
{
struct PxsIndexedContactManager;
class PxsRigidBody;

namespace Dy
{

/*!
Cache information specific to the software implementation(non common).

See PxcgetThreadContext.

Not thread-safe, so remember to have one object per thread!

TODO! refactor this and rename(it is a general per thread cache). Move transform cache into its own class.
*/
class ThreadContext : public PxcThreadCoherentCache<ThreadContext, PxcNpMemBlockPool>::EntryBase
{
	PX_NOCOPY(ThreadContext)
public:

#if PX_ENABLE_SIM_STATS
	struct ThreadSimStats
	{
		void clear()
		{
			numActiveConstraints = 0;
			numActiveDynamicBodies = 0;
			numActiveKinematicBodies = 0;
			numAxisSolverConstraints = 0;
		}

		PxU32 numActiveConstraints;
		PxU32 numActiveDynamicBodies;
		PxU32 numActiveKinematicBodies;
		PxU32 numAxisSolverConstraints;

		Dy::ErrorAccumulatorEx contactErrorAccumulator;
	};
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

	//TODO: tune cache size based on number of active objects.
	ThreadContext(PxcNpMemBlockPool* memBlockPool);
	void reset();
	void resizeArrays(PxU32 frictionConstraintDescCount, PxU32 articulationCount);

	// PT: TODO: is there a reason why everything is public except mArticulations ?
	PX_FORCE_INLINE	PxArray<ArticulationSolverDesc>&	getArticulations()	{ return mArticulations;	}

#if PX_ENABLE_SIM_STATS
	PX_FORCE_INLINE ThreadSimStats& getSimStats()
	{
		return mThreadSimStats;
	}
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

	PxContactBuffer								mContactBuffer;

		// temporary buffer for correlation
	PX_ALIGN(16, CorrelationBuffer				mCorrelationBuffer); 

	FrictionPatchStreamPair						mFrictionPatchStreamPair;	// patch streams

	PxsConstraintBlockManager					mConstraintBlockManager;	// for when this thread context is "lead" on an island
	PxcConstraintBlockStream 					mConstraintBlockStream;		// constraint block pool

	// this stuff is just used for reformatting the solver data. Hopefully we should have a more
	// sane format for this when the dust settles - so it's just temporary. If we keep this around
	// here we should move these from public to private

	PxU32										mNumDifferentBodyConstraints;
	PxU32										mNumDifferentBodyFrictionConstraints;
	PxU32										mNumSelfConstraints;
	PxU32										mNumStaticConstraints;
	PxU32										mNumSelfFrictionConstraints;
	PxU32										mNumSelfConstraintFrictionBlocks;
	bool										mHasOverflowPartitions;

	PxArray<PxU32>								mConstraintsPerPartition;
	PxArray<PxU32>								mFrictionConstraintsPerPartition;
	//PxArray<PxU32>								mPartitionNormalizationBitmap;	// PT: for PX_NORMALIZE_PARTITIONS
	PxsBodyCore**								mBodyCoreArray;
	PxsRigidBody**								mRigidBodyArray;
	FeatherstoneArticulation**					mArticulationArray;
	Cm::SpatialVector*							motionVelocityArray;
	PxU32*										bodyRemapTable;
	PxU32*										mNodeIndexArray;

	// PT: TODO: unify names around here, some use "m", some don't

	//Constraint info for normal constraint solver
	PxSolverConstraintDesc*						contactConstraintDescArray;
	PxU32										contactDescArraySize;
	PxSolverConstraintDesc*						orderedContactConstraints;
	PxConstraintBatchHeader*					contactConstraintBatchHeaders;
	PxU32										numContactConstraintBatches;

	//Constraint info for partitioning
	PxSolverConstraintDesc*						tempConstraintDescArray;

	//Additional constraint info for 1d/2d friction model
	PxArray<PxSolverConstraintDesc>				frictionConstraintDescArray;
	PxArray<PxConstraintBatchHeader>			frictionConstraintBatchHeaders;

	//Info for tracking compound contact managers (temporary data - could use scratch memory!)
	PxArray<CompoundContactManager>				compoundConstraints;

	//Used for sorting constraints. Temporary, could use scratch memory
	PxArray<const PxsIndexedContactManager*>	orderedContactList;
	PxArray<const PxsIndexedContactManager*>	tempContactList;
	PxArray<PxU32>								sortIndexArray;
	
	PxArray<Cm::SpatialVectorF>					mZVector; // scratch space, used for propagation during constraint prepping
	PxArray<Cm::SpatialVectorF>					mDeltaV; // scratch space, used temporarily for propagating velocities

	PxU32										numDifferentBodyBatchHeaders;
	PxU32										numSelfConstraintBatchHeaders;

	PxU32										mOrderedContactDescCount;
	PxU32										mOrderedFrictionDescCount;

	PxU32										mConstraintSize;

	PxU32										mAxisConstraintCount;

	PxU32										mMaxPartitions;
	PxU32										mMaxFrictionPartitions;
	PxU32										mMaxSolverPositionIterations;
	PxU32										mMaxSolverVelocityIterations;
	PxU32										mMaxArticulationLinks;
	
	PxSolverConstraintDesc*						mContactDescPtr;
	PxSolverConstraintDesc*						mStartContactDescPtr;
	PxSolverConstraintDesc*						mFrictionDescPtr;
private:

	PxArray<ArticulationSolverDesc>				mArticulations;

#if PX_ENABLE_SIM_STATS
	ThreadSimStats								mThreadSimStats;
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
};

}

}

#endif
