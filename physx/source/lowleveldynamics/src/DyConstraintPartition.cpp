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

#include "DyConstraintPartition.h"
#include "foundation/PxHashMap.h"
#include "DyFeatherstoneArticulation.h"

using namespace physx;

// PT: notes:
// - there was a prefetch in one codepath, not in the other. It was completely wrong anyway. Removed.
// - why do we subtract numStaticConstraints in one codepath only? Feature or bug? (see batchConstraints)
// - enhancedDeterminism was not used (commented out). Removed it from the API for now.
//
// Generally speaking the approach used here is like a radix/counting sort:
// - one pass to compute counters (classifyConstraintDesc)
// - compute offsets/histogram from counters (accumulation)
// - reset counters (afterClassification)
// - second pass recomputing the same "radices" (or in this case the partition indices),
//   and this time doing the desc copies/sort (writeConstraintDesc)
//
// This could probably be improved, as it is unclear why we need to re-classify constraints in the
// second pass for example (we could just store & reuse the previous results I think).
//
// More importantly perhaps, why did we put all these progress counters inside the bodies?
// Why not just a temp flat array of these, used just for the partitioning, reducing memory usage & cache misses?
// Or are we really using all these variables later in the solver?
//
// - maxSolverFrictionProgress is used but only for articulation (same name, different variable)
// - maxSolverNormalProgress doesn't look used
// - maxSolverFrictionProgress doesn't look used
//
// - nbStaticInteractions doesn't look used
// - maxDynamicPartition looks used in TGS
// - partitionMask looks used in TGS

PX_COMPILE_TIME_ASSERT(PX_OFFSET_OF(PxSolverBody, maxSolverNormalProgress)==PX_OFFSET_OF(PxTGSSolverBodyVel, maxDynamicPartition));
PX_COMPILE_TIME_ASSERT(PX_OFFSET_OF(PxSolverBody, maxSolverFrictionProgress)==PX_OFFSET_OF(PxTGSSolverBodyVel, nbStaticInteractions));
PX_COMPILE_TIME_ASSERT(PX_OFFSET_OF(PxSolverBody, solverProgress)==PX_OFFSET_OF(PxTGSSolverBodyVel, partitionMask));

namespace physx
{
namespace Dy
{
namespace
{

#define MAX_NUM_PARTITIONS 32u
// PT: for template args but it would be so much easier to use bodies[2] instead of bodyA/bodyB in the structs
#define BODYA	false
#define BODYB	true

class ClassificationBase
{
	PX_NOCOPY(ClassificationBase)
public:
	PxU8* const mBodies;
	const PxU32 mBodySize;
	const PxU32 mBodyStride;
	const PxU32 mBodyCount;

	ClassificationBase(PxU8* bodies, PxU32 bodyCount, PxU32 bodyStride) :
		mBodies		(bodies),
		mBodySize	(bodyCount*bodyStride),
		mBodyStride	(bodyStride),
		mBodyCount	(bodyCount)
	{
	}
};

PX_FORCE_INLINE	void initSolverProgress(PxU32 nbBodies, PxU32 stride, PxU8* bodies)
{
	while(nbBodies--)
	{
		PxSolverBody& body = *reinterpret_cast<PxSolverBody*>(bodies);
		bodies += stride;
		body.solverProgress = 0;
		//We re-use maxSolverFrictionProgress and maxSolverNormalProgress to record the
		//maximum partition used by dynamic constraints and the number of static constraints affecting
		//a body. We use this to make partitioning much cheaper and be able to support an arbitrary number of dynamic partitions.
		body.maxSolverFrictionProgress = 0;
		body.maxSolverNormalProgress = 0;
	}
}

PX_FORCE_INLINE	void resetSolverProgress(PxU32 nbBodies, PxU32 stride, PxU8* bodies)
{
	while(nbBodies--)
	{
		PxSolverBody& body = *reinterpret_cast<PxSolverBody*>(bodies);
		bodies += stride;
		body.solverProgress = 0;
		//Keep the dynamic constraint count but bump the static constraint count back to 0.
		//This allows us to place the static constraints in the appropriate place when we see them
		//because we know the maximum index for the dynamic constraints...
		body.maxSolverFrictionProgress = 0;
	}
}

// PT: TODO: unify all this, there's no need for such duplication. I think we could just use the "extended" version
// all the time, but it could have a small performance impact, so for now I'll keep both.

// PT: regular version without articulations
PX_FORCE_INLINE void reserveSpaceForStaticConstraints_(PxArray<PxU32>& numConstraintsPerPartition, PxU32 bodyCount, PxU32 bodyStride, PxU8* bodies)
{
	while(bodyCount--)
	{
		PxSolverBody& body = *reinterpret_cast<PxSolverBody*>(bodies);
		bodies += bodyStride;

		body.solverProgress = 0;

		const PxU32 requiredSize = PxU32(body.maxSolverNormalProgress + body.maxSolverFrictionProgress);
		if(requiredSize > numConstraintsPerPartition.size())
			numConstraintsPerPartition.resize(requiredSize);

		for(PxU32 b=0; b<body.maxSolverFrictionProgress; b++)
			numConstraintsPerPartition[body.maxSolverNormalProgress + b]++;
	}
}

// PT: "extended" version with articulations
PX_FORCE_INLINE void reserveSpaceForStaticConstraints_(PxArray<PxU32>& numConstraintsPerPartition, PxU32 bodyCount, PxU32 bodyStride, PxU8* bodies,
														PxU32 numArticulations, Dy::FeatherstoneArticulation** articulations)
{
	reserveSpaceForStaticConstraints_(numConstraintsPerPartition, bodyCount, bodyStride, bodies);

	while(numArticulations--)
	{
		FeatherstoneArticulation* articulation = *articulations++;
		articulation->solverProgress = 0;

		const PxU32 requiredSize = PxU32(articulation->maxSolverNormalProgress + articulation->maxSolverFrictionProgress);
		if(requiredSize > numConstraintsPerPartition.size())
			numConstraintsPerPartition.resize(requiredSize);

		for(PxU32 b=0; b<articulation->maxSolverFrictionProgress; b++)
			numConstraintsPerPartition[articulation->maxSolverNormalProgress + b]++;
	}
}

// PT: putting these in functions to ensure both versions do the same thing for rigid bodies

template<const bool a_or_b>
static PX_FORCE_INLINE PxU32 getRigidBodyStaticContactWriteIndex(const PxSolverConstraintDesc& desc)
{
	PxSolverBody* body = a_or_b ? desc.bodyB : desc.bodyA;
	return PxU32(body->maxSolverNormalProgress + body->maxSolverFrictionProgress++);
}

template<const bool a_or_b>
static PX_FORCE_INLINE void storeRigidBodyProgress(const PxSolverConstraintDesc& desc, PxU32 bodyProgress, PxU16 availablePartition)
{
	PxSolverBody* body = a_or_b ? desc.bodyB : desc.bodyA;
	body->solverProgress = bodyProgress;
	body->maxSolverNormalProgress = PxMax(body->maxSolverNormalProgress, availablePartition);
}

// PT: regular version without articulations
class RigidBodyClassification : public ClassificationBase
{
	PX_NOCOPY(RigidBodyClassification)

public:
	RigidBodyClassification(PxU8* bodies, PxU32 bodyCount, PxU32 bodyStride) : ClassificationBase(bodies, bodyCount, bodyStride)
	{
	}

	PX_FORCE_INLINE void clearState()
	{
		for(PxU32 a = 0; a < mBodySize; a+= mBodyStride)
			reinterpret_cast<PxSolverBody*>(mBodies+a)->solverProgress = 0;
	}

	PX_FORCE_INLINE void zeroBodies()
	{
		initSolverProgress(mBodyCount, mBodyStride, mBodies);
	}

	PX_FORCE_INLINE void afterClassification()	const
	{
		resetSolverProgress(mBodyCount, mBodyStride, mBodies);
	}

	PX_FORCE_INLINE void reserveSpaceForStaticConstraints(PxArray<PxU32>& numConstraintsPerPartition)
	{
		reserveSpaceForStaticConstraints_(numConstraintsPerPartition, mBodyCount, mBodyStride, mBodies);
	}

	// Returns true if it is a dynamic-dynamic constraint; false if it is a dynamic-static or dynamic-kinematic constraint
	PX_FORCE_INLINE bool classifyConstraint(const PxSolverConstraintDesc& desc, uintptr_t& indexA, uintptr_t& indexB,
		bool& activeA, bool& activeB, PxU32& bodyAProgress, PxU32& bodyBProgress) const
	{
		// PT: TODO: that divide is a bit clumsy and we could find a better way to get the index
		// *IMPORTANT*:something tricky is happening here for static bodies that all reference a fake solver body class
		// located at a random position in memory (potentially *before* the start of the bodies array). When that happens
		// the index can be very negative, and the code below sees it as a large (unsigned) positive number, so the body
		// is properly seen as inactive but the index is basically a random number at that point.
		indexA = uintptr_t(reinterpret_cast<PxU8*>(desc.bodyA) - mBodies) / mBodyStride;
		indexB = uintptr_t(reinterpret_cast<PxU8*>(desc.bodyB) - mBodies) / mBodyStride;
		activeA = indexA < mBodyCount;
		activeB = indexB < mBodyCount;
		bodyAProgress = desc.bodyA->solverProgress;
		bodyBProgress = desc.bodyB->solverProgress;
		return activeA && activeB;
	}

	PX_FORCE_INLINE PxU32 getStaticContactWriteIndex(const PxSolverConstraintDesc& desc, bool activeA, bool activeB)	const
	{
		if(activeA)
			return getRigidBodyStaticContactWriteIndex<BODYA>(desc);
		else if(activeB)
			return getRigidBodyStaticContactWriteIndex<BODYB>(desc);
	
		return 0xffffffff;
	}

	PX_FORCE_INLINE void recordStaticConstraint(const PxSolverConstraintDesc& desc, bool activeA, bool activeB) const
	{
		if(activeA)
			desc.bodyA->maxSolverFrictionProgress++;

		if(activeB)
			desc.bodyB->maxSolverFrictionProgress++;
	}

	PX_FORCE_INLINE void storeProgress_(const PxSolverConstraintDesc& desc, PxU32 bodyAProgress, PxU32 bodyBProgress)
	{
		desc.bodyA->solverProgress = bodyAProgress;
		desc.bodyB->solverProgress = bodyBProgress;
	}

	PX_FORCE_INLINE void storeProgress(const PxSolverConstraintDesc& desc, PxU32 bodyAProgress, PxU32 bodyBProgress, PxU16 availablePartition)
	{
		storeRigidBodyProgress<BODYA>(desc, bodyAProgress, availablePartition);
		storeRigidBodyProgress<BODYB>(desc, bodyBProgress, availablePartition);
	}
};

template<const bool a_or_b>
static PX_FORCE_INLINE PxU32 getArticulationStaticContactWriteIndex(const PxSolverConstraintDesc& desc, bool forceStaticCollisionsToSolver)
{
	FeatherstoneArticulation* articulation = a_or_b ? getArticulationB(desc) : getArticulationA(desc);
	//Attempt to store static constraints on the articulation (only supported with the reduced coordinate articulations).
	//This acts as an optimization
	if(!forceStaticCollisionsToSolver && articulation->storeStaticConstraint(desc))
		return 0xffffffff;
	return PxU32(articulation->maxSolverNormalProgress + articulation->maxSolverFrictionProgress++);
}

template<const bool a_or_b>
static PX_FORCE_INLINE void recordArticulationStaticConstraint(const PxSolverConstraintDesc& desc, bool forceStaticCollisionsToSolver)
{
	FeatherstoneArticulation* articulation = a_or_b ? getArticulationB(desc) : getArticulationA(desc);
	if(!articulation->willStoreStaticConstraint() || forceStaticCollisionsToSolver)
		articulation->maxSolverFrictionProgress++;
}

template<const bool a_or_b>
static PX_FORCE_INLINE void storeArticulationProgress(const PxSolverConstraintDesc& desc, PxU32 bodyProgress, PxU16 availablePartition)
{
	FeatherstoneArticulation* articulation = a_or_b ? getArticulationB(desc) : getArticulationA(desc);
	articulation->solverProgress = bodyProgress;
	articulation->maxSolverNormalProgress = PxMax(articulation->maxSolverNormalProgress, availablePartition);
}

// PT: "extended" version with articulations
class ExtendedRigidBodyClassification : public ClassificationBase
{
	PX_NOCOPY(ExtendedRigidBodyClassification)

public:
	Dy::FeatherstoneArticulation** mArticulations;
	const PxU32 mNumArticulations;

	// PT: only used for point-friction, which is not available in immediate mode.
	// Immediate mode version should use "true" for this, in order to match the previous imm mode batching code.
	const bool mForceStaticCollisionsToSolver;

	ExtendedRigidBodyClassification(PxU8* bodies, PxU32 numBodies, PxU32 stride, Dy::FeatherstoneArticulation** articulations, PxU32 numArticulations, bool forceStaticCollisionsToSolver) :
		ClassificationBase				(bodies, numBodies, stride),
		mArticulations					(articulations),
		mNumArticulations				(numArticulations),
		mForceStaticCollisionsToSolver	(forceStaticCollisionsToSolver)
	{
		for(PxU32 i=0; i<mNumArticulations; i++)
			mArticulations[i]->mArticulationIndex = PxTo16(i);
	}

	PX_FORCE_INLINE void storeProgress_(const PxSolverConstraintDesc& desc, PxU32 bodyAProgress, PxU32 bodyBProgress)
	{
		if (desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY)
			desc.bodyA->solverProgress = bodyAProgress;
		else
			getArticulationA(desc)->solverProgress = bodyAProgress;

		if (desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY)
			desc.bodyB->solverProgress = bodyBProgress;
		else
			getArticulationB(desc)->solverProgress = bodyBProgress;
	}

	PX_FORCE_INLINE void clearState()
	{
		for(PxU32 a = 0; a < mBodySize; a+= mBodyStride)
			reinterpret_cast<PxSolverBody*>(mBodies+a)->solverProgress = 0;

		for(PxU32 a = 0; a < mNumArticulations; ++a)
			mArticulations[a]->solverProgress = 0;
	}

	PX_FORCE_INLINE void zeroBodies()
	{
		initSolverProgress(mBodyCount, mBodyStride, mBodies);

		for(PxU32 a=0; a<mNumArticulations; ++a)
		{
			Dy::FeatherstoneArticulation* articulation = mArticulations[a];
			articulation->solverProgress = 0;
			articulation->maxSolverFrictionProgress = 0;
			articulation->maxSolverNormalProgress = 0;
		}
	}

	PX_FORCE_INLINE void afterClassification()	const
	{
		resetSolverProgress(mBodyCount, mBodyStride, mBodies);

		for(PxU32 a=0; a<mNumArticulations; ++a)
		{
			Dy::FeatherstoneArticulation* articulation = mArticulations[a];
			articulation->solverProgress = 0;
			articulation->maxSolverFrictionProgress = 0;
		}
	}

	PX_FORCE_INLINE void reserveSpaceForStaticConstraints(PxArray<PxU32>& numConstraintsPerPartition)
	{
		reserveSpaceForStaticConstraints_(numConstraintsPerPartition, mBodyCount, mBodyStride, mBodies, mNumArticulations, mArticulations);
	}

	// PT: this version is slightly different from the immediate mode version, which didn't use mArticulationIndex.
	// Returns true if it is a dynamic-dynamic constraint; false if it is a dynamic-static or dynamic-kinematic constraint
	PX_FORCE_INLINE bool classifyConstraint(const PxSolverConstraintDesc& desc, uintptr_t& indexA, uintptr_t& indexB, 
		bool& activeA, bool& activeB, PxU32& bodyAProgress, PxU32& bodyBProgress) const
	{
		// PT: note that the rigid-body path is slightly different here from the regular version (which is exactly why
		// trying to share the code helps in rediscovering these differences). In this case we have an extra "hasStatic"
		// variable to deal with and the bodyProgress is set to 0 when the body is inactive, which is not the same as
		// what we do in the regular version. I don't know if it's by design or not but I didn't touch that.

		bool hasStatic = false;
		if(desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY)
		{
			indexA = uintptr_t(reinterpret_cast<PxU8*>(desc.bodyA) - mBodies)/mBodyStride;
			activeA = indexA < mBodyCount;
			hasStatic = !activeA;
			bodyAProgress = activeA ? desc.bodyA->solverProgress: 0;
		}
		else
		{
			FeatherstoneArticulation* articulationA = getArticulationA(desc);
			indexA = mBodyCount + articulationA->mArticulationIndex;
			bodyAProgress = articulationA->solverProgress;
			activeA = true;
		}

		if(desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY)
		{
			indexB = uintptr_t(reinterpret_cast<PxU8*>(desc.bodyB) - mBodies)/mBodyStride;
			activeB = indexB < mBodyCount;
			hasStatic = hasStatic || !activeB;
			bodyBProgress = activeB ? desc.bodyB->solverProgress : 0;
		}
		else
		{
			FeatherstoneArticulation* articulationB = getArticulationB(desc);
			indexB = mBodyCount + articulationB->mArticulationIndex;
			activeB = true;
			bodyBProgress = articulationB->solverProgress;
		}
		return !hasStatic;
	}

	PX_FORCE_INLINE void recordStaticConstraint(const PxSolverConstraintDesc& desc, bool activeA, bool activeB)
	{
		if(activeA)
		{
			if(desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY)
				desc.bodyA->maxSolverFrictionProgress++;
			else
				recordArticulationStaticConstraint<BODYA>(desc, mForceStaticCollisionsToSolver);
		}

		if(activeB)
		{
			if(desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY)
				desc.bodyB->maxSolverFrictionProgress++;
			else
				recordArticulationStaticConstraint<BODYB>(desc, mForceStaticCollisionsToSolver);
		}
	}

	PX_FORCE_INLINE PxU32 getStaticContactWriteIndex(const PxSolverConstraintDesc& desc, bool activeA, bool activeB)	const
	{
		if(activeA)
		{
			if(desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY)
				return getRigidBodyStaticContactWriteIndex<BODYA>(desc);
			else
				return getArticulationStaticContactWriteIndex<BODYA>(desc, mForceStaticCollisionsToSolver);
		}
		else if(activeB)
		{
			if(desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY)
				return getRigidBodyStaticContactWriteIndex<BODYB>(desc);
			else
				return getArticulationStaticContactWriteIndex<BODYB>(desc, mForceStaticCollisionsToSolver);
		}
		return 0xffffffff;
	}

	PX_FORCE_INLINE void storeProgress(const PxSolverConstraintDesc& desc, PxU32 bodyAProgress, PxU32 bodyBProgress, PxU16 availablePartition)
	{
		if(desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY)
			storeRigidBodyProgress<BODYA>(desc, bodyAProgress, availablePartition);
		else
			storeArticulationProgress<BODYA>(desc, bodyAProgress, availablePartition);

		if(desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY)
			storeRigidBodyProgress<BODYB>(desc, bodyBProgress, availablePartition);
		else
			storeArticulationProgress<BODYB>(desc, bodyBProgress, availablePartition);
	}
};

static PX_FORCE_INLINE bool computeAvailablePartition(PxU32& availablePartition, PxU32& partitionsA, PxU32& partitionsB, bool activeA, bool activeB)
{
	const PxU32 combinedMask = (~partitionsA & ~partitionsB);
	availablePartition = combinedMask == 0 ? MAX_NUM_PARTITIONS : PxLowestSetBit(combinedMask);
	if(availablePartition == MAX_NUM_PARTITIONS)
		return false;

	const PxU32 partitionBit = (1u << availablePartition);
	if(activeA)
		partitionsA |= partitionBit;
	if(activeB)
		partitionsB |= partitionBit;
	return true;
}

template <typename Classification>
static PxU32 classifyConstraintDesc(const PxSolverConstraintDesc* PX_RESTRICT descs, PxU32 numConstraints, Classification& classification, 
				PxArray<PxU32>& numConstraintsPerPartition, PxSolverConstraintDesc* PX_RESTRICT eaTempConstraintDescriptors, PxU32 maxPartitions)
{
	const PxSolverConstraintDesc* _desc = descs;
	const PxU32 numConstraintsMin1 = numConstraints - 1;

	PxU32 numUnpartitionedConstraints = 0;

	numConstraintsPerPartition.forceSize_Unsafe(MAX_NUM_PARTITIONS);

	PxMemZero(numConstraintsPerPartition.begin(), sizeof(PxU32) * MAX_NUM_PARTITIONS);

	for(PxU32 i = 0; i < numConstraints; ++i, _desc++)
	{
		const PxU32 prefetchOffset = PxMin(numConstraintsMin1 - i, 4u);
		//PxPrefetchLine(_desc[prefetchOffset].constraint);	// PT: removed because we don't actually use constraint?
		PxPrefetchLine(_desc[prefetchOffset].bodyA);
		PxPrefetchLine(_desc[prefetchOffset].bodyB);
		//PxPrefetchLine(_desc + 8);

		uintptr_t indexA, indexB;
		bool activeA, activeB;
		PxU32 partitionsA, partitionsB;
		const bool notContainsStatic = classification.classifyConstraint(*_desc, indexA, indexB, activeA, activeB, partitionsA, partitionsB);
		
		if(notContainsStatic)
		{			
			PxU32 availablePartition;
			if(!computeAvailablePartition(availablePartition, partitionsA, partitionsB, activeA, activeB))
			{
				eaTempConstraintDescriptors[numUnpartitionedConstraints++] = *_desc;
				continue;
			}

			numConstraintsPerPartition[availablePartition]++;
			availablePartition++;
			classification.storeProgress(*_desc, partitionsA, partitionsB, PxU16(availablePartition));
		}
		else
		{
			classification.recordStaticConstraint(*_desc, activeA, activeB);
		}
	}

	// PT: this whole part below was missing in immediate mode

	PxU32 partitionStartIndex = 0;

	while (numUnpartitionedConstraints > 0)
	{
		classification.clearState();

		partitionStartIndex += MAX_NUM_PARTITIONS;

		if(maxPartitions <= partitionStartIndex)
			break;

		//Keep partitioning the un-partitioned constraints and blat the whole thing to 0!
		numConstraintsPerPartition.resize(MAX_NUM_PARTITIONS + numConstraintsPerPartition.size());
		PxMemZero(numConstraintsPerPartition.begin() + partitionStartIndex, sizeof(PxU32) * MAX_NUM_PARTITIONS);

		PxU32 newNumUnpartitionedConstraints = 0;
		PxU32 partitionsA, partitionsB;
		bool activeA, activeB;
		uintptr_t indexA, indexB;
		for (PxU32 i = 0; i < numUnpartitionedConstraints; ++i)
		{
			const PxSolverConstraintDesc& desc = eaTempConstraintDescriptors[i];

			classification.classifyConstraint(desc, indexA, indexB, activeA, activeB, partitionsA, partitionsB);

			PxU32 availablePartition;
			if(!computeAvailablePartition(availablePartition, partitionsA, partitionsB, activeA, activeB))
			{
				//Need to shuffle around unpartitioned constraints...
				eaTempConstraintDescriptors[newNumUnpartitionedConstraints++] = desc;
				continue;
			}

			availablePartition += partitionStartIndex;
			numConstraintsPerPartition[availablePartition]++;
			availablePartition++;

			classification.storeProgress(desc, partitionsA, partitionsB, PxU16(availablePartition));
		}

		numUnpartitionedConstraints = newNumUnpartitionedConstraints;
	}

	classification.reserveSpaceForStaticConstraints(numConstraintsPerPartition);

	return numUnpartitionedConstraints;
}

template <typename Classification>
static PxU32 writeConstraintDesc(	const PxSolverConstraintDesc* PX_RESTRICT descs, PxU32 numConstraints, Classification& classification,
									PxArray<PxU32>& accumulatedConstraintsPerPartition, PxSolverConstraintDesc* PX_RESTRICT eaTempConstraintDescriptors,
									PxSolverConstraintDesc* PX_RESTRICT eaOrderedConstraintDesc, PxU32 maxPartitions, PxU32 numOverflows)
{
	const PxSolverConstraintDesc* _desc = descs;
	const PxU32 numConstraintsMin1 = numConstraints - 1;

	PxU32 numUnpartitionedConstraints = 0;
	PxU32 numStaticConstraints = 0;

	for(PxU32 i = 0; i < numConstraints; ++i, _desc++)
	{
		const PxU32 prefetchOffset = PxMin(numConstraintsMin1 - i, 4u);
		//PxPrefetchLine(_desc[prefetchOffset].constraint);	// PT: removed because we don't actually use constraint?
		PxPrefetchLine(_desc[prefetchOffset].bodyA);
		PxPrefetchLine(_desc[prefetchOffset].bodyB);
		//PxPrefetchLine(_desc + 8);

		uintptr_t indexA, indexB;
		bool activeA, activeB;
		PxU32 partitionsA, partitionsB;
		const bool notContainsStatic = classification.classifyConstraint(*_desc, indexA, indexB, activeA, activeB, partitionsA, partitionsB);

		if(notContainsStatic)
		{
			PxU32 availablePartition;
			if(!computeAvailablePartition(availablePartition, partitionsA, partitionsB, activeA, activeB))
			{
				// PT: TODO: these copies could be costly
				eaTempConstraintDescriptors[numUnpartitionedConstraints++] = *_desc;
				continue;
			}

			classification.storeProgress(*_desc, partitionsA, partitionsB, PxU16(availablePartition + 1));

			// PT: TODO: these copies could be costly
			eaOrderedConstraintDesc[numOverflows + accumulatedConstraintsPerPartition[availablePartition]++] = *_desc;
		}
		else
		{
			//Just count the number of static constraints and store in maxSolverFrictionProgress...
			const PxU32 index = classification.getStaticContactWriteIndex(*_desc, activeA, activeB);
			if(index != 0xffffffff)
				eaOrderedConstraintDesc[numOverflows + accumulatedConstraintsPerPartition[index]++] = *_desc;
			else
				numStaticConstraints++;
		}
	}

	// PT: this whole part below was missing in immediate mode

	PxU32 partitionStartIndex = 0;

	while (numUnpartitionedConstraints > 0)
	{
		classification.clearState();

		partitionStartIndex += MAX_NUM_PARTITIONS;

		if(partitionStartIndex >= maxPartitions)
			break;

		PxU32 newNumUnpartitionedConstraints = 0;

		PxU32 partitionsA, partitionsB;
		bool activeA, activeB;
		uintptr_t indexA, indexB;
		for (PxU32 i = 0; i < numUnpartitionedConstraints; ++i)
		{
			const PxSolverConstraintDesc& desc = eaTempConstraintDescriptors[i];

			classification.classifyConstraint(desc, indexA, indexB, activeA, activeB, partitionsA, partitionsB);

			PxU32 availablePartition;
			if(!computeAvailablePartition(availablePartition, partitionsA, partitionsB, activeA, activeB))
			{
				//Need to shuffle around unpartitioned constraints...
				eaTempConstraintDescriptors[newNumUnpartitionedConstraints++] = desc;
				continue;
			}

			classification.storeProgress_(desc, partitionsA, partitionsB);
			availablePartition += partitionStartIndex;
			eaOrderedConstraintDesc[numOverflows + accumulatedConstraintsPerPartition[availablePartition]++] = desc;
		}

		numUnpartitionedConstraints = newNumUnpartitionedConstraints;
	}

	return numStaticConstraints;
}

static void outputOverflowConstraints(
	PxArray<PxU32>& accumulatedConstraintsPerPartition, PxSolverConstraintDesc* overflowConstraints, PxU32 nbOverflowConstraints,
	PxSolverConstraintDesc* PX_RESTRICT eaOrderedConstraintDesc)
{
	//Firstly, we resize and shuffle accumulatedConstraintsPerPartition

	accumulatedConstraintsPerPartition.resize(accumulatedConstraintsPerPartition.size()+1);
	PxU32 partitionCount = accumulatedConstraintsPerPartition.size();

	while(partitionCount-- > 1)
	{
		accumulatedConstraintsPerPartition[partitionCount] = accumulatedConstraintsPerPartition[partitionCount -1] + nbOverflowConstraints;
	}
	accumulatedConstraintsPerPartition[0] = nbOverflowConstraints;

	//Now fill in the constraints and work out the iter

	for (PxU32 i = 0; i < nbOverflowConstraints; ++i)
	{
		eaOrderedConstraintDesc[i] = overflowConstraints[i];
	}
}

}

#define PX_NORMALIZE_PARTITIONS 1

#if PX_NORMALIZE_PARTITIONS

#ifdef REMOVED_UNUSED
template<typename Classification>
PxU32 normalizePartitions(PxArray<PxU32>& accumulatedConstraintsPerPartition, PxSolverConstraintDesc* PX_RESTRICT eaOrderedConstraintDescriptors, 
	PxU32 numConstraintDescriptors, PxArray<PxU32>& bitField, const Classification& classification, PxU32 numBodies, PxU32 numArticulations)
{
	PxU32 numPartitions = 0;
	
	PxU32 prevAccumulation = 0;
	for(; numPartitions < accumulatedConstraintsPerPartition.size() && accumulatedConstraintsPerPartition[numPartitions] > prevAccumulation; 
		prevAccumulation = accumulatedConstraintsPerPartition[numPartitions++]);

	PxU32 targetSize = (numPartitions == 0 ? 0 : (numConstraintDescriptors)/numPartitions);

	bitField.reserve((numBodies + numArticulations + 31)/32);
	bitField.forceSize_Unsafe((numBodies + numArticulations + 31)/32);

	for(PxU32 i = numPartitions; i > 0; i--)
	{
		PxU32 partitionIndex = i-1;

		//Build the partition mask...

		PxU32 startIndex = partitionIndex == 0 ? 0 : accumulatedConstraintsPerPartition[partitionIndex-1];
		PxU32 endIndex = accumulatedConstraintsPerPartition[partitionIndex];

		//If its greater than target size, there's nothing that will be pulled into it from earlier partitions
		if((endIndex - startIndex) >= targetSize)
			continue;


		PxMemZero(bitField.begin(), sizeof(PxU32)*bitField.size());

		for(PxU32 a = startIndex; a < endIndex; ++a)
		{
			PxSolverConstraintDesc& desc = eaOrderedConstraintDescriptors[a];

			uintptr_t indexA, indexB;
			bool activeA, activeB;
			PxU32 partitionsA, partitionsB;

			classification.classifyConstraint(desc, indexA, indexB, activeA, activeB, partitionsA, partitionsB);

			if (activeA)
				bitField[PxU32(indexA) / 32] |= (1u << (indexA & 31)); 
			if(activeB)
				bitField[PxU32(indexB)/32] |= (1u << (indexB & 31));
		}

		bool bTerm = false;
		for(PxU32 a = partitionIndex; a > 0 && !bTerm; --a)
		{
			PxU32 pInd = a-1;

			PxU32 si = pInd == 0 ? 0 : accumulatedConstraintsPerPartition[pInd-1];
			PxU32 ei = accumulatedConstraintsPerPartition[pInd];

			for(PxU32 b = ei; b > si && !bTerm; --b)
			{
				PxU32 ind = b-1;
				PxSolverConstraintDesc& desc = eaOrderedConstraintDescriptors[ind];

				uintptr_t indexA, indexB;
				bool activeA, activeB;
				PxU32 partitionsA, partitionsB;

				classification.classifyConstraint(desc, indexA, indexB, activeA, activeB, partitionsA, partitionsB);

				bool canAdd = true;

				if(activeA && (bitField[PxU32(indexA)/32] & (1u << (indexA & 31))))
					canAdd = false;
				if(activeB && (bitField[PxU32(indexB)/32] & (1u << (indexB & 31))))
					canAdd = false;

				if(canAdd)
				{
					PxSolverConstraintDesc tmp = eaOrderedConstraintDescriptors[ind];

					if(activeA)
						bitField[PxU32(indexA)/32] |= (1u << (indexA & 31));
					if(activeB)
						bitField[PxU32(indexB)/32] |= (1u << (indexB & 31));

					PxU32 index = ind;
					for(PxU32 c = pInd; c < partitionIndex; ++c)
					{
						PxU32 newIndex = --accumulatedConstraintsPerPartition[c];
						if(index != newIndex)
							eaOrderedConstraintDescriptors[index] = eaOrderedConstraintDescriptors[newIndex];	
						index = newIndex;
					}

					if(index != ind)
						eaOrderedConstraintDescriptors[index] = tmp;

					if((accumulatedConstraintsPerPartition[partitionIndex] - accumulatedConstraintsPerPartition[partitionIndex-1]) >= targetSize)
					{
						bTerm = true;
						break;
					}
				}
			}
		}
	}
		
	PxU32 partitionCount = 0;
	PxU32 lastPartitionCount = 0;
	for (PxU32 a = 0; a < numPartitions; ++a)
	{
		const PxU32 constraintCount = accumulatedConstraintsPerPartition[a];
		accumulatedConstraintsPerPartition[partitionCount] = constraintCount;
		if (constraintCount != lastPartitionCount)
		{
			lastPartitionCount = constraintCount;
			partitionCount++;
		}
	}

	accumulatedConstraintsPerPartition.forceSize_Unsafe(partitionCount);

	return partitionCount;
}
#endif

#endif

template <const bool extended, typename Classification>
static void batchConstraints(
	const PxSolverConstraintDesc* PX_RESTRICT eaConstraintDescriptors, PxU32 numConstraintDescriptors,
	Classification& classification, PxArray<PxU32>& constraintsPerPartition,
	PxSolverConstraintDesc* PX_RESTRICT eaOverflowConstraintDescriptors, PxU32 maxPartitions,
	PxSolverConstraintDesc* PX_RESTRICT eaOrderedConstraintDescriptors,
	PxU32& numOverflows, PxU32& numOrderedConstraints, PxU32& numStaticConstraints)
{
	// PT: "initSolverProgress" replaced with zeroBodies(), now deal with articulations there
	classification.zeroBodies();

	numOverflows = classifyConstraintDesc(	eaConstraintDescriptors, numConstraintDescriptors, classification, constraintsPerPartition,
											eaOverflowConstraintDescriptors, maxPartitions);

	// PT: just the same as computing the offsets in a radix sort
	PxU32 accumulation = 0;
	for(PxU32 a=0; a<constraintsPerPartition.size(); a++)
	{
		const PxU32 count = constraintsPerPartition[a];
		constraintsPerPartition[a] = accumulation;
		accumulation += count;
	}

	// PT: "resetSolverProgress" replaced with afterClassification(), now deal with articulations there
	classification.afterClassification();

	numStaticConstraints = writeConstraintDesc(	eaConstraintDescriptors, numConstraintDescriptors, classification, constraintsPerPartition, 
												eaOverflowConstraintDescriptors, eaOrderedConstraintDescriptors, maxPartitions, numOverflows);

	// PT: TODO: not sure why this was different in the two codepaths
	if(extended)
		numOrderedConstraints = numConstraintDescriptors - numStaticConstraints;
	else
		numOrderedConstraints = numConstraintDescriptors;

	// Next step, let's slot the overflow partitions into the first slot and work out targets for them...
	if(numOverflows)
		outputOverflowConstraints(constraintsPerPartition, eaOverflowConstraintDescriptors, numOverflows, eaOrderedConstraintDescriptors);
}

PxU32 partitionContactConstraints(ConstraintPartitionOut& out, const ConstraintPartitionIn& in)
{
	const PxU32 numBodies = in.mNumBodies;
	const PxU32	numArticulations = in.mNumArticulationPtrs;
	
	PxArray<PxU32>& constraintsPerPartition = *out.mConstraintsPerPartition;
	constraintsPerPartition.forceSize_Unsafe(0);

	const PxU32 stride = in.mStride;

	// PT: "initSolverProgress" moved to batchConstraints

	PxU32 numOrderedConstraints = 0;
	PxU32 numStaticConstraints = 0;
	PxU32 numOverflows = 0;

	if(numArticulations == 0)
	{
		RigidBodyClassification classification(in.mBodies, numBodies, stride);
		batchConstraints<false>(in.mContactConstraintDescriptors, in.mNumContactConstraintDescriptors,
								classification, constraintsPerPartition,
								out.mOverflowConstraintDescriptors, in.mMaxPartitions,
								out.mOrderedContactConstraintDescriptors,
								numOverflows, numOrderedConstraints, numStaticConstraints);
	}
	else
	{
		ExtendedRigidBodyClassification classification(in.mBodies, numBodies, stride, in.mArticulationPtrs, numArticulations, in.mForceStaticConstraintsToSolver);
		batchConstraints<true>(	in.mContactConstraintDescriptors, in.mNumContactConstraintDescriptors,
								classification, constraintsPerPartition,
								out.mOverflowConstraintDescriptors, in.mMaxPartitions,
								out.mOrderedContactConstraintDescriptors,
								numOverflows, numOrderedConstraints, numStaticConstraints);
	}

	const PxU32 numConstraintsDifferentBodies = numOrderedConstraints;

	//PX_ASSERT(numConstraintsDifferentBodies == numConstraintDescriptors);

	//Now handle the articulated self-constraints.
	PxU32 totalConstraintCount = numConstraintsDifferentBodies;	

	out.mNumDifferentBodyConstraints = numConstraintsDifferentBodies;
	out.mNumSelfConstraints = totalConstraintCount - numConstraintsDifferentBodies;

	out.mNumStaticConstraints = numStaticConstraints;
	out.mNumOverflowConstraints = numOverflows;

	PxU32 maxPartition = 0;
	//if (args.enhancedDeterminism)
	{
		PxU32 prevPartitionSize = 0;
		maxPartition = 0;
		for (PxU32 a = 0; a < constraintsPerPartition.size(); ++a, maxPartition++)
		{
			if (constraintsPerPartition[a] == prevPartitionSize)
				break;
			prevPartitionSize = constraintsPerPartition[a];
		}
	}

	return maxPartition;
}

///////////////////////////////////////////////////////////////////////////////

template<const bool a_or_b>
static PX_FORCE_INLINE PxU32 getRigidBodyProgress(const PxSolverConstraintDesc& desc, PxU32 bodyCount, PxU32 bodyStride, PxU8* const bodies)
{
	PxSolverBody* body = a_or_b ? desc.bodyB : desc.bodyA;
	const uintptr_t index = uintptr_t(reinterpret_cast<PxU8*>(body) - bodies) / bodyStride;
	return index < bodyCount ? body->maxSolverFrictionProgress++ : 0;
}

static PX_FORCE_INLINE void getProgressRequirements(const PxSolverConstraintDesc& desc, PxU32& progressA, PxU32& progressB, PxU32 bodyCount, PxU32 bodyStride, PxU8* const bodies)
{
	progressA = getRigidBodyProgress<BODYA>(desc, bodyCount, bodyStride, bodies);
	progressB = getRigidBodyProgress<BODYB>(desc, bodyCount, bodyStride, bodies);
}

// PT: TODO: we could just use that one for both cases tbh
static PX_FORCE_INLINE void getProgressRequirementsExtended(const PxSolverConstraintDesc& desc, PxU32& progressA, PxU32& progressB, PxU32 bodyCount, PxU32 bodyStride, PxU8* const bodies)
{
	if(desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY)
		progressA = getRigidBodyProgress<BODYA>(desc, bodyCount, bodyStride, bodies);
	else
		progressA = getArticulationA(desc)->maxSolverFrictionProgress++;

	if(desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY)
	{
		progressB = getRigidBodyProgress<BODYB>(desc, bodyCount, bodyStride, bodies);
	}
	else 
	{
		if(desc.articulationA != desc.articulationB)
			progressB = getArticulationB(desc)->maxSolverFrictionProgress++;
		else
			progressB = progressA;
	}
}

void processOverflowConstraints(PxU8* bodies, PxU32 bodyStride, PxU32 numBodies, Dy::ArticulationSolverDesc* articulationDescs, PxU32 numArticulations,
	PxSolverConstraintDesc* constraints, PxU32 numConstraints)
{
	// PT: TODO: resetSolverProgress + the articulation reset below is the same as afterClassification()
	// And skipping the articulation reset when numConstraints == 0 seems like a mistake.
	resetSolverProgress(numBodies, bodyStride, bodies);

	if (numConstraints == 0)
		return;

	if (numArticulations == 0)
	{
		for (PxU32 i = 0; i < numConstraints; ++i)
		{
			PxU32 progressA, progressB;
			getProgressRequirements(constraints[i], progressA, progressB, numBodies, bodyStride, bodies);
			constraints[i].progressA = PxTo16(progressA);
			constraints[i].progressB = PxTo16(progressB);
		}
	}
	else
	{
		PX_ALLOCA(_eaArticulations, Dy::FeatherstoneArticulation*, numArticulations);
		Dy::FeatherstoneArticulation** eaArticulations = _eaArticulations;
		for (PxU32 i = 0; i<numArticulations; i++)
		{
			FeatherstoneArticulation* articulation = articulationDescs[i].articulation;
			eaArticulations[i] = articulation;
			articulation->solverProgress = 0;
			articulation->maxSolverFrictionProgress = 0;
			//articulation->maxSolverNormalProgress = 0;
		}

		for (PxU32 i = 0; i < numConstraints; ++i)
		{
			PxU32 progressA, progressB;
			getProgressRequirementsExtended(constraints[i], progressA, progressB, numBodies, bodyStride, bodies);
			constraints[i].progressA = PxTo16(progressA);
			constraints[i].progressB = PxTo16(progressB);
		}
	}
}
}
}
