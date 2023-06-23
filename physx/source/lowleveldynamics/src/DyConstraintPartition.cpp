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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.

#include "DyConstraintPartition.h"
#include "DyArticulationUtils.h"
#include "DySoftBody.h"
#include "foundation/PxHashMap.h"
#include "DyFeatherstoneArticulation.h"

#define INTERLEAVE_SELF_CONSTRAINTS 1

namespace physx
{
namespace Dy
{
namespace
{
class RigidBodyClassification : public RigidBodyClassificationBase
{
public:
	RigidBodyClassification(PxU8* bodies, PxU32 bodyCount, PxU32 bodyStride) : RigidBodyClassificationBase(bodies, bodyCount, bodyStride)
	{
	}

/*	PX_FORCE_INLINE void getProgress(const PxSolverConstraintDesc& desc, PxU32& bodyAProgress, PxU32& bodyBProgress)	const
	{
		bodyAProgress = desc.bodyA->solverProgress;
		bodyBProgress = desc.bodyB->solverProgress;
	}*/

	PX_FORCE_INLINE void getProgressRequirements(const PxSolverConstraintDesc& desc, PxU32& progressA, PxU32& progressB) const
	{
		const uintptr_t indexA = uintptr_t(reinterpret_cast<PxU8*>(desc.bodyA) - mBodies) / mBodyStride;
		const uintptr_t indexB = uintptr_t(reinterpret_cast<PxU8*>(desc.bodyB) - mBodies) / mBodyStride;
		const bool activeA = indexA < mBodyCount;
		const bool activeB = indexB < mBodyCount;

		if (activeA)
			progressA = desc.bodyA->maxSolverFrictionProgress++;
		else
			progressA = 0;
		if (activeB)
			progressB = desc.bodyB->maxSolverFrictionProgress++;
		else
			progressB = 0;
	}

	PX_FORCE_INLINE void clearState()
	{
		for(PxU32 a = 0; a < mBodySize; a+= mBodyStride)
			reinterpret_cast<PxSolverBody*>(mBodies+a)->solverProgress = 0;
	}

	PX_FORCE_INLINE void reserveSpaceForStaticConstraints(PxArray<PxU32>& numConstraintsPerPartition)
	{
		for(PxU32 a = 0; a < mBodySize; a += mBodyStride)
		{
			PxSolverBody& body = *reinterpret_cast<PxSolverBody*>(mBodies+a);
			body.solverProgress = 0;

			PxU32 requiredSize = PxU32(body.maxSolverNormalProgress + body.maxSolverFrictionProgress);
			if(requiredSize > numConstraintsPerPartition.size())
			{
				numConstraintsPerPartition.resize(requiredSize);
			}

			for(PxU32 b = 0; b < body.maxSolverFrictionProgress; ++b)
			{
				numConstraintsPerPartition[body.maxSolverNormalProgress + b]++;
			}
		}
	}
};

class ExtendedRigidBodyClassification : public ExtendedRigidBodyClassificationBase
{
	const bool mForceStaticCollisionsToSolver;	// PT: why is this one not present in the immediate mode version?

public:

	ExtendedRigidBodyClassification(PxU8* bodies, PxU32 numBodies, PxU32 stride, Dy::FeatherstoneArticulation** articulations, PxU32 numArticulations, bool forceStaticCollisionsToSolver) :
		ExtendedRigidBodyClassificationBase	(bodies, numBodies, stride, articulations, numArticulations),
		mForceStaticCollisionsToSolver		(forceStaticCollisionsToSolver)
	{
		// PT: why is this loop not present in the immediate mode version?
		for (PxU32 i = 0; i < mNumArticulations; ++i)
			mArticulations[i]->mArticulationIndex = PxTo16(i);
	}

	// PT: this version is slightly different from the immediate mode version, see mArticulationIndex!
	//Returns true if it is a dynamic-dynamic constraint; false if it is a dynamic-static or dynamic-kinematic constraint
	PX_FORCE_INLINE bool classifyConstraint(const PxSolverConstraintDesc& desc, uintptr_t& indexA, uintptr_t& indexB, 
		bool& activeA, bool& activeB, PxU32& bodyAProgress, PxU32& bodyBProgress) const
	{
		bool hasStatic = false;
		if(desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY)
		{
			indexA=uintptr_t(reinterpret_cast<PxU8*>(desc.bodyA) - mBodies)/mStride;
			activeA = indexA < mBodyCount;
			hasStatic = !activeA;//desc.bodyADataIndex == 0;
			bodyAProgress = activeA ? desc.bodyA->solverProgress: 0;
		}
		else
		{
			FeatherstoneArticulation* articulationA = getArticulationA(desc);
			indexA=mBodyCount+ articulationA->mArticulationIndex;
			//bodyAProgress = articulationA->getFsDataPtr()->solverProgress;
			bodyAProgress = articulationA->solverProgress;
			activeA = true;
		}

		if(desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY)
		{
			indexB=uintptr_t(reinterpret_cast<PxU8*>(desc.bodyB) - mBodies)/mStride;
			activeB = indexB < mBodyCount;
			hasStatic = hasStatic || !activeB;
			bodyBProgress = activeB ? desc.bodyB->solverProgress : 0;
		}
		else
		{
			FeatherstoneArticulation* articulationB = getArticulationB(desc);
			indexB=mBodyCount+ articulationB->mArticulationIndex;
			activeB = true;
			bodyBProgress = articulationB->solverProgress;
		}
		return !hasStatic;
	}

/*	PX_FORCE_INLINE void getProgress(const PxSolverConstraintDesc& desc,
		PxU32& bodyAProgress, PxU32& bodyBProgress) const
	{
		if (desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY)
		{
			bodyAProgress = desc.bodyA->solverProgress;
		}
		else
		{
			FeatherstoneArticulation* articulationA = getArticulationA(desc);
			bodyAProgress = articulationA->solverProgress;
		}

		if (desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY)
		{
			bodyBProgress = desc.bodyB->solverProgress;
		}
		else
		{
			FeatherstoneArticulation* articulationB = getArticulationB(desc);
			bodyBProgress = articulationB->solverProgress;
		}
	}*/

	PX_FORCE_INLINE void storeProgress(const PxSolverConstraintDesc& desc, PxU32 bodyAProgress, PxU32 bodyBProgress, PxU16 availablePartition)
	{
		if (desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY)
		{
			desc.bodyA->solverProgress = bodyAProgress;
			desc.bodyA->maxSolverNormalProgress = PxMax(desc.bodyA->maxSolverNormalProgress, availablePartition);
		}
		else
		{
			FeatherstoneArticulation* articulationA = getArticulationA(desc);
			articulationA->solverProgress = bodyAProgress;
			articulationA->maxSolverNormalProgress = PxMax(articulationA->maxSolverNormalProgress, availablePartition);
		}

		if (desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY)
		{
			desc.bodyB->solverProgress = bodyBProgress;
			desc.bodyB->maxSolverNormalProgress = PxMax(desc.bodyB->maxSolverNormalProgress, availablePartition);
		}
		else
		{
			FeatherstoneArticulation* articulationB = getArticulationB(desc);
			articulationB->solverProgress = bodyBProgress;
			articulationB->maxSolverNormalProgress = PxMax(articulationB->maxSolverNormalProgress, availablePartition);
		}
	}

	PX_FORCE_INLINE void storeProgress(const PxSolverConstraintDesc& desc, PxU32 bodyAProgress, PxU32 bodyBProgress)
	{
		if (desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY)
		{
			desc.bodyA->solverProgress = bodyAProgress;
		}
		else
		{
			FeatherstoneArticulation* articulationA = getArticulationA(desc);
			articulationA->solverProgress = bodyAProgress;
		}

		if (desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY)
		{
			desc.bodyB->solverProgress = bodyBProgress;
		}
		else
		{
			FeatherstoneArticulation* articulationB = getArticulationB(desc);
			articulationB->solverProgress = bodyBProgress;
		}
	}

	PX_FORCE_INLINE void recordStaticConstraint(const PxSolverConstraintDesc& desc, bool& activeA, bool& activeB)
	{
		if (activeA)
		{
			if (desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY)
				desc.bodyA->maxSolverFrictionProgress++;
			else
			{
				FeatherstoneArticulation* articulationA = getArticulationA(desc);
				if(!articulationA->willStoreStaticConstraint() || mForceStaticCollisionsToSolver)
					articulationA->maxSolverFrictionProgress++;
			}
		}

		if (activeB)
		{
			if (desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY)
				desc.bodyB->maxSolverFrictionProgress++;
			else
			{
				FeatherstoneArticulation* articulationB = getArticulationB(desc);
				if (!articulationB->willStoreStaticConstraint() || mForceStaticCollisionsToSolver)
					articulationB->maxSolverFrictionProgress++;
			}
		}
	}

	PX_FORCE_INLINE void getProgressRequirements(const PxSolverConstraintDesc& desc, PxU32& progressA, PxU32& progressB) const
	{
		if (desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY)
		{
			uintptr_t indexA = uintptr_t(reinterpret_cast<PxU8*>(desc.bodyA) - mBodies) / mStride;
			if(indexA < mBodyCount)
				progressA = desc.bodyA->maxSolverFrictionProgress++;
			else
				progressA = 0;
		}
		else
		{
			FeatherstoneArticulation* articulationA = getArticulationA(desc);
			progressA = articulationA->maxSolverFrictionProgress++;
		}

		if (desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY)
		{
			uintptr_t indexB = uintptr_t(reinterpret_cast<PxU8*>(desc.bodyB) - mBodies) / mStride;
			if(indexB < mBodyCount)
				progressB = desc.bodyB->maxSolverFrictionProgress++;
			else
				progressB = 0;
		}
		else 
		{
			if (desc.articulationA != desc.articulationB)
			{
				FeatherstoneArticulation* articulationB = getArticulationB(desc);
				progressB = articulationB->maxSolverFrictionProgress++;
			}
			else
				progressB = progressA;
		}
	}

	PX_FORCE_INLINE PxU32 getStaticContactWriteIndex(const PxSolverConstraintDesc& desc, bool activeA, bool activeB)	const
	{
		if (activeA)
		{
			if (desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY)
			{
				return PxU32(desc.bodyA->maxSolverNormalProgress + desc.bodyA->maxSolverFrictionProgress++);
			}
			else
			{
				FeatherstoneArticulation* articulationA = getArticulationA(desc);
				//Attempt to store static constraints on the articulation (only supported with the reduced coordinate articulations).
				//This acts as an optimization
				if(!mForceStaticCollisionsToSolver && articulationA->storeStaticConstraint(desc))
					return 0xffffffff;
				return PxU32(articulationA->maxSolverNormalProgress + articulationA->maxSolverFrictionProgress++);
			}
		}
		else if (activeB)
		{
			if (desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY)
			{
				return PxU32(desc.bodyB->maxSolverNormalProgress + desc.bodyB->maxSolverFrictionProgress++);
			}
			else
			{
				FeatherstoneArticulation* articulationB = getArticulationB(desc);
				//Attempt to store static constraints on the articulation (only supported with the reduced coordinate articulations).
				//This acts as an optimization
				if (!mForceStaticCollisionsToSolver && articulationB->storeStaticConstraint(desc))
					return 0xffffffff;
				return PxU32(articulationB->maxSolverNormalProgress + articulationB->maxSolverFrictionProgress++);
			}
		}

		return 0xffffffff;
	}

	PX_FORCE_INLINE void clearState()
	{
		for(PxU32 a = 0; a < mBodySize; a+= mStride)
			reinterpret_cast<PxSolverBody*>(mBodies+a)->solverProgress = 0;

		for(PxU32 a = 0; a < mNumArticulations; ++a)
			(reinterpret_cast<FeatherstoneArticulation*>(mArticulations[a]))->solverProgress = 0;
	}

	PX_FORCE_INLINE void reserveSpaceForStaticConstraints(PxArray<PxU32>& numConstraintsPerPartition)
	{
		for(PxU32 a = 0; a < mBodySize; a+= mStride)
		{
			PxSolverBody& body = *reinterpret_cast<PxSolverBody*>(mBodies+a);
			body.solverProgress = 0;

			PxU32 requiredSize = PxU32(body.maxSolverNormalProgress + body.maxSolverFrictionProgress);
			if(requiredSize > numConstraintsPerPartition.size())
			{
				numConstraintsPerPartition.resize(requiredSize);
			}

			for(PxU32 b = 0; b < body.maxSolverFrictionProgress; ++b)
			{
				numConstraintsPerPartition[body.maxSolverNormalProgress + b]++;
			}
		}

		for(PxU32 a = 0; a < mNumArticulations; ++a)
		{
			FeatherstoneArticulation* articulation = reinterpret_cast<FeatherstoneArticulation*>(mArticulations[a]);
			articulation->solverProgress = 0;

			PxU32 requiredSize = PxU32(articulation->maxSolverNormalProgress + articulation->maxSolverFrictionProgress);
			if(requiredSize > numConstraintsPerPartition.size())
			{
				numConstraintsPerPartition.resize(requiredSize);
			}

			for(PxU32 b = 0; b < articulation->maxSolverFrictionProgress; ++b)
			{
				numConstraintsPerPartition[articulation->maxSolverNormalProgress + b]++;
			}
		}
	}

};

template <typename Classification>
static PxU32 classifyConstraintDesc(const PxSolverConstraintDesc* PX_RESTRICT descs, PxU32 numConstraints, Classification& classification, 
							PxArray<PxU32>& numConstraintsPerPartition, PxSolverConstraintDesc* PX_RESTRICT eaTempConstraintDescriptors,
							PxU32 maxPartitions)
{
	const PxSolverConstraintDesc* _desc = descs;
	const PxU32 numConstraintsMin1 = numConstraints - 1;

	PxU32 numUnpartitionedConstraints = 0;

	numConstraintsPerPartition.forceSize_Unsafe(32);

	PxMemZero(numConstraintsPerPartition.begin(), sizeof(PxU32) * 32);

	for(PxU32 i = 0; i < numConstraints; ++i, _desc++)
	{
		const PxU32 prefetchOffset = PxMin(numConstraintsMin1 - i, 4u);
		PxPrefetchLine(_desc[prefetchOffset].constraint);
		PxPrefetchLine(_desc[prefetchOffset].bodyA);
		PxPrefetchLine(_desc[prefetchOffset].bodyB);
		PxPrefetchLine(_desc + 8);

		uintptr_t indexA, indexB;
		bool activeA, activeB;

		PxU32 partitionsA, partitionsB;
		const bool notContainsStatic = classification.classifyConstraint(*_desc, indexA, indexB, activeA, activeB,
			partitionsA, partitionsB);
		
		if(notContainsStatic)
		{			
			PxU32 availablePartition;
			{
				const PxU32 combinedMask = (~partitionsA & ~partitionsB);
				availablePartition = combinedMask == 0 ? MAX_NUM_PARTITIONS : PxLowestSetBit(combinedMask);
				if(availablePartition == MAX_NUM_PARTITIONS)
				{
					eaTempConstraintDescriptors[numUnpartitionedConstraints++] = *_desc;
					continue;
				}

				const PxU32 partitionBit = (1u << availablePartition);
				if (activeA)
					partitionsA |= partitionBit;
				if(activeB)
					partitionsB |= partitionBit;
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

	PxU32 partitionStartIndex = 0;

	while (numUnpartitionedConstraints > 0)
	{
		classification.clearState();

		partitionStartIndex += 32;

		if(maxPartitions <= partitionStartIndex)
			break;

		//Keep partitioning the un-partitioned constraints and blat the whole thing to 0!
		numConstraintsPerPartition.resize(32 + numConstraintsPerPartition.size());
		PxMemZero(numConstraintsPerPartition.begin() + partitionStartIndex, sizeof(PxU32) * 32);

		PxU32 newNumUnpartitionedConstraints = 0;
		PxU32 partitionsA, partitionsB;
		bool activeA, activeB;
		uintptr_t indexA, indexB;
		for (PxU32 i = 0; i < numUnpartitionedConstraints; ++i)
		{
			const PxSolverConstraintDesc& desc = eaTempConstraintDescriptors[i];

			classification.classifyConstraint(desc, indexA, indexB, activeA, activeB,
				partitionsA, partitionsB);

			PxU32 availablePartition;
			{
				const PxU32 combinedMask = (~partitionsA & ~partitionsB);
				availablePartition = combinedMask == 0 ? MAX_NUM_PARTITIONS : PxLowestSetBit(combinedMask);
				if (availablePartition == MAX_NUM_PARTITIONS)
				{
					//Need to shuffle around unpartitioned constraints...
					eaTempConstraintDescriptors[newNumUnpartitionedConstraints++] = desc;
					continue;
				}

				const PxU32 partitionBit = (1u << availablePartition);
				if (activeA)
					partitionsA |= partitionBit;
				if (activeB)
					partitionsB |= partitionBit;
			}


			/*desc.bodyA->solverProgress = partitionsA;
			desc.bodyB->solverProgress = partitionsB;*/
			availablePartition += partitionStartIndex;
			numConstraintsPerPartition[availablePartition]++;
			availablePartition++;

			classification.storeProgress(desc, partitionsA, partitionsB, PxU16(availablePartition));

			/*	desc.bodyA->maxSolverNormalProgress = PxMax(desc.bodyA->maxSolverNormalProgress, PxU16(availablePartition));
				desc.bodyB->maxSolverNormalProgress = PxMax(desc.bodyB->maxSolverNormalProgress, PxU16(availablePartition));*/
		}

		numUnpartitionedConstraints = newNumUnpartitionedConstraints;
	}

	classification.reserveSpaceForStaticConstraints(numConstraintsPerPartition);

	return numUnpartitionedConstraints;
}

template <typename Classification>
static PxU32 writeConstraintDesc(	const PxSolverConstraintDesc* PX_RESTRICT descs, PxU32 numConstraints, Classification& classification,
									PxArray<PxU32>& accumulatedConstraintsPerPartition, PxSolverConstraintDesc* eaTempConstraintDescriptors,
									PxSolverConstraintDesc* PX_RESTRICT eaOrderedConstraintDesc, PxU32 maxPartitions, PxU32 numOverflows)
{
	PX_UNUSED(eaTempConstraintDescriptors);
	const PxSolverConstraintDesc* _desc = descs;
	const PxU32 numConstraintsMin1 = numConstraints - 1;

	PxU32 numUnpartitionedConstraints = 0;
	PxU32 numStaticConstraints = 0;

	for(PxU32 i = 0; i < numConstraints; ++i, _desc++)
	{
		const PxU32 prefetchOffset = PxMin(numConstraintsMin1 - i, 4u);
		PxPrefetchLine(_desc[prefetchOffset].constraint);
		PxPrefetchLine(_desc[prefetchOffset].bodyA);
		PxPrefetchLine(_desc[prefetchOffset].bodyB);
		PxPrefetchLine(_desc + 8);

		uintptr_t indexA, indexB;
		bool activeA, activeB;
		PxU32 partitionsA, partitionsB;
		const bool notContainsStatic = classification.classifyConstraint(*_desc, indexA, indexB, activeA, activeB, partitionsA, partitionsB);

		if(notContainsStatic)
		{
			PxU32 availablePartition;
			{
				const PxU32 combinedMask = (~partitionsA & ~partitionsB);
				availablePartition = combinedMask == 0 ? MAX_NUM_PARTITIONS : PxLowestSetBit(combinedMask);
				if(availablePartition == MAX_NUM_PARTITIONS)
				{
					eaTempConstraintDescriptors[numUnpartitionedConstraints++] = *_desc;
					continue;
				}

				const PxU32 partitionBit = (1u << availablePartition);
				if(activeA)
					partitionsA |= partitionBit;
				if(activeB)
					partitionsB |= partitionBit;
			}

			classification.storeProgress(*_desc, partitionsA, partitionsB, PxU16(availablePartition + 1));

			eaOrderedConstraintDesc[numOverflows + accumulatedConstraintsPerPartition[availablePartition]++] = *_desc;
		}
		else
		{
			//Just count the number of static constraints and store in maxSolverFrictionProgress...
			PxU32 index = classification.getStaticContactWriteIndex(*_desc, activeA, activeB);
			if (index != 0xffffffff)
			{
				eaOrderedConstraintDesc[numOverflows + accumulatedConstraintsPerPartition[index]++] = *_desc;
			}
			else
				numStaticConstraints++;
		}
	}

	PxU32 partitionStartIndex = 0;

	while (numUnpartitionedConstraints > 0)
	{
		classification.clearState();

		partitionStartIndex += 32;

		if(partitionStartIndex >= maxPartitions)
			break;

		PxU32 newNumUnpartitionedConstraints = 0;

		PxU32 partitionsA, partitionsB;
		bool activeA, activeB;
		uintptr_t indexA, indexB;
		for (PxU32 i = 0; i < numUnpartitionedConstraints; ++i)
		{
			const PxSolverConstraintDesc& desc = eaTempConstraintDescriptors[i];

			/*	PxU32 partitionsA=desc.bodyA->solverProgress;
				PxU32 partitionsB=desc.bodyB->solverProgress;*/

			classification.classifyConstraint(desc, indexA, indexB,
				activeA, activeB, partitionsA, partitionsB);

			PxU32 availablePartition;
			{
				const PxU32 combinedMask = (~partitionsA & ~partitionsB);
				availablePartition = combinedMask == 0 ? MAX_NUM_PARTITIONS : PxLowestSetBit(combinedMask);
				if (availablePartition == MAX_NUM_PARTITIONS)
				{
					//Need to shuffle around unpartitioned constraints...
					eaTempConstraintDescriptors[newNumUnpartitionedConstraints++] = desc;
					continue;
				}

				const PxU32 partitionBit = (1u << availablePartition);

				if (activeA)
					partitionsA |= partitionBit;
				if (activeB)
					partitionsB |= partitionBit;
			}

			/*desc.bodyA->solverProgress = partitionsA;
			desc.bodyB->solverProgress = partitionsB;
*/
			classification.storeProgress(desc, partitionsA, partitionsB);
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

PxU32 partitionContactConstraints(ConstraintPartitionArgs& args) 
{
	PxU32 maxPartition = 0;
	//Unpack the input data.
	const PxU32 numBodies = args.mNumBodies;
	const PxU32	numArticulations = args.mNumArticulationPtrs;
	
	const PxU32 numConstraintDescriptors = args.mNumContactConstraintDescriptors;

	const PxSolverConstraintDesc* PX_RESTRICT eaConstraintDescriptors = args.mContactConstraintDescriptors;
	PxSolverConstraintDesc* PX_RESTRICT eaOrderedConstraintDescriptors = args.mOrderedContactConstraintDescriptors;
	PxSolverConstraintDesc* PX_RESTRICT eaOverflowConstraintDescriptors = args.mOverflowConstraintDescriptors;

	PxArray<PxU32>& constraintsPerPartition = *args.mConstraintsPerPartition;
	constraintsPerPartition.forceSize_Unsafe(0);

	const PxU32 stride = args.mStride;

	for(PxU32 a = 0, offset = 0; a < numBodies; ++a, offset += stride)
	{
		PxSolverBody& body = *reinterpret_cast<PxSolverBody*>(args.mBodies + offset);
		body.solverProgress = 0;
		//We re-use maxSolverFrictionProgress and maxSolverNormalProgress to record the
		//maximum partition used by dynamic constraints and the number of static constraints affecting
		//a body. We use this to make partitioning much cheaper and be able to support an arbitrary number of dynamic partitions.
		body.maxSolverFrictionProgress = 0;
		body.maxSolverNormalProgress = 0;
	}

	PxU32 numOrderedConstraints=0;	
	PxU32 numStaticConstraints = 0;
	PxU32 numOverflows = 0;

	if(numArticulations == 0)
	{
		RigidBodyClassification classification(args.mBodies, numBodies, stride);
		numOverflows = classifyConstraintDesc(eaConstraintDescriptors, numConstraintDescriptors, classification, constraintsPerPartition,
			eaOverflowConstraintDescriptors, args.mMaxPartitions);
		
		PxU32 accumulation = 0;
		for(PxU32 a = 0; a < constraintsPerPartition.size(); ++a)
		{
			PxU32 count = constraintsPerPartition[a];
			constraintsPerPartition[a] = accumulation;
			accumulation += count;
		}

		for(PxU32 a = 0, offset = 0; a < numBodies; ++a, offset += stride)
		{
			PxSolverBody& body = *reinterpret_cast<PxSolverBody*>(args.mBodies + offset);
			PxPrefetchLine(&args.mBodies[a], 256);
			body.solverProgress = 0;
			//Keep the dynamic constraint count but bump the static constraint count back to 0.
			//This allows us to place the static constraints in the appropriate place when we see them
			//because we know the maximum index for the dynamic constraints...
			body.maxSolverFrictionProgress = 0;
		}

		writeConstraintDesc(eaConstraintDescriptors, numConstraintDescriptors, classification, constraintsPerPartition, 
			eaOverflowConstraintDescriptors, eaOrderedConstraintDescriptors, args.mMaxPartitions, numOverflows);

		numOrderedConstraints = numConstraintDescriptors;

		//Next step, let's slot the overflow partitions into the first slot and work out targets for them...
		if (numOverflows)
		{
			outputOverflowConstraints(constraintsPerPartition,
				eaOverflowConstraintDescriptors, numOverflows, eaOrderedConstraintDescriptors);
		}
	}
	else
	{
		const ArticulationSolverDesc* articulationDescs=args.mArticulationPtrs;
		PX_ALLOCA(_eaArticulations, Dy::FeatherstoneArticulation*, numArticulations);
		Dy::FeatherstoneArticulation** eaArticulations = _eaArticulations;
		for(PxU32 i=0;i<numArticulations;i++)
		{
			FeatherstoneArticulation* articulation = articulationDescs[i].articulation;
			eaArticulations[i]= articulation;
			articulation->solverProgress = 0;
			articulation->maxSolverFrictionProgress = 0;
			articulation->maxSolverNormalProgress = 0;
		}
		ExtendedRigidBodyClassification classification(args.mBodies, numBodies, stride, eaArticulations, numArticulations,
			args.mForceStaticConstraintsToSolver);

		numOverflows = classifyConstraintDesc(eaConstraintDescriptors, numConstraintDescriptors, classification, 
			constraintsPerPartition, eaOverflowConstraintDescriptors, args.mMaxPartitions);

		PxU32 accumulation = 0;
		for(PxU32 a = 0; a < constraintsPerPartition.size(); ++a)
		{
			PxU32 count = constraintsPerPartition[a];
			constraintsPerPartition[a] = accumulation;
			accumulation += count;
		}

		for(PxU32 a = 0, offset = 0; a < numBodies; ++a, offset += stride)
		{
			PxSolverBody& body = *reinterpret_cast<PxSolverBody*>(args.mBodies+offset);
			body.solverProgress = 0;
			//Keep the dynamic constraint count but bump the static constraint count back to 0.
			//This allows us to place the static constraints in the appropriate place when we see them
			//because we know the maximum index for the dynamic constraints...
			body.maxSolverFrictionProgress = 0;
		}

		for(PxU32 a = 0; a < numArticulations; ++a)
		{
			FeatherstoneArticulation* articulation = eaArticulations[a];
			articulation->solverProgress = 0;
			articulation->maxSolverFrictionProgress = 0;
		}

		numStaticConstraints = writeConstraintDesc(eaConstraintDescriptors, numConstraintDescriptors, classification, constraintsPerPartition, 
			eaOverflowConstraintDescriptors, eaOrderedConstraintDescriptors, args.mMaxPartitions, numOverflows);

		numOrderedConstraints = numConstraintDescriptors - numStaticConstraints;

		if (numOverflows)
		{
			outputOverflowConstraints(constraintsPerPartition,
				eaOverflowConstraintDescriptors, numOverflows, eaOrderedConstraintDescriptors);
		}
	}

	const PxU32 numConstraintsDifferentBodies=numOrderedConstraints;

	//PX_ASSERT(numConstraintsDifferentBodies == numConstraintDescriptors);

	//Now handle the articulated self-constraints.
	PxU32 totalConstraintCount = numConstraintsDifferentBodies;	

	args.mNumDifferentBodyConstraints=numConstraintsDifferentBodies;
	args.mNumSelfConstraints=totalConstraintCount-numConstraintsDifferentBodies;

	args.mNumStaticConstraints = numStaticConstraints;
	args.mNumOverflowConstraints = numOverflows;

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

void processOverflowConstraints(PxU8* bodies, PxU32 bodyStride, PxU32 numBodies, Dy::ArticulationSolverDesc* articulationDescs, PxU32 numArticulations,
	PxSolverConstraintDesc* constraints, PxU32 numConstraints)
{
	for (PxU32 a = 0, offset = 0; a < numBodies; ++a, offset += bodyStride)
	{
		PxSolverBody& body = *reinterpret_cast<PxSolverBody*>(bodies + offset);
		body.solverProgress = 0;
		//Keep the dynamic constraint count but bump the static constraint count back to 0.
		//This allows us to place the static constraints in the appropriate place when we see them
		//because we know the maximum index for the dynamic constraints...
		body.maxSolverFrictionProgress = 0;
	}

	if (numConstraints == 0)
		return;

	if (numArticulations == 0)
	{
		RigidBodyClassification classification(bodies, numBodies, bodyStride);
		for (PxU32 i = 0; i < numConstraints; ++i)
		{
			PxU32 progressA, progressB;
			classification.getProgressRequirements(constraints[i], progressA, progressB);
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

		ExtendedRigidBodyClassification classification(bodies, numBodies, bodyStride, eaArticulations, numArticulations, false);

		for (PxU32 i = 0; i < numConstraints; ++i)
		{
			PxU32 progressA, progressB;
			classification.getProgressRequirements(constraints[i], progressA, progressB);
			constraints[i].progressA = PxTo16(progressA);
			constraints[i].progressB = PxTo16(progressB);
		}
	}
}

}

}
