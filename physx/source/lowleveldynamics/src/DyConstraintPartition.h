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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef DY_CONSTRAINT_PARTITION_H
#define DY_CONSTRAINT_PARTITION_H

#include "DyDynamics.h"

namespace physx
{
namespace Dy
{
#define MAX_NUM_PARTITIONS 32u

// PT: introduced base classes to start sharing code between the SDK's and the immediate mode's versions.

class RigidBodyClassificationBase
{
	PX_NOCOPY(RigidBodyClassificationBase)

	PxU8* const mBodies;
	const PxU32 mBodySize;
	const PxU32 mBodyStride;
	const PxU32 mBodyCount;

public:
	RigidBodyClassificationBase(PxU8* bodies, PxU32 bodyCount, PxU32 bodyStride) :
		mBodies		(bodies),
		mBodySize	(bodyCount*bodyStride),
		mBodyStride	(bodyStride),
		mBodyCount	(bodyCount)
	{
	}

	//Returns true if it is a dynamic-dynamic constraint; false if it is a dynamic-static or dynamic-kinematic constraint
	PX_FORCE_INLINE bool classifyConstraint(const PxSolverConstraintDesc& desc, uintptr_t& indexA, uintptr_t& indexB,
		bool& activeA, bool& activeB, PxU32& bodyAProgress, PxU32& bodyBProgress) const
	{
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
			return PxU32(desc.bodyA->maxSolverNormalProgress + desc.bodyA->maxSolverFrictionProgress++);
		else if(activeB)
			return PxU32(desc.bodyB->maxSolverNormalProgress + desc.bodyB->maxSolverFrictionProgress++);
	
		return 0xffffffff;
	}

	PX_FORCE_INLINE void recordStaticConstraint(const PxSolverConstraintDesc& desc, bool activeA, bool activeB) const
	{
		if(activeA)
			desc.bodyA->maxSolverFrictionProgress++;

		if(activeB)
			desc.bodyB->maxSolverFrictionProgress++;
	}

	PX_FORCE_INLINE void storeProgress(const PxSolverConstraintDesc& desc, PxU32 bodyAProgress, PxU32 bodyBProgress)
	{
		desc.bodyA->solverProgress = bodyAProgress;
		desc.bodyB->solverProgress = bodyBProgress;
	}

	PX_FORCE_INLINE void storeProgress(const PxSolverConstraintDesc& desc, PxU32 bodyAProgress, PxU32 bodyBProgress, PxU16 availablePartition)
	{
		desc.bodyA->solverProgress = bodyAProgress;
		desc.bodyA->maxSolverNormalProgress = PxMax(desc.bodyA->maxSolverNormalProgress, availablePartition);
		desc.bodyB->solverProgress = bodyBProgress;
		desc.bodyB->maxSolverNormalProgress = PxMax(desc.bodyB->maxSolverNormalProgress, availablePartition);
	}
};

class ExtendedRigidBodyClassificationBase
{
	PX_NOCOPY(ExtendedRigidBodyClassificationBase)

	PxU8* const mBodies;
	const PxU32 mBodyCount;
	const PxU32 mBodySize;
	const PxU32 mStride;
	Dy::FeatherstoneArticulation** mArticulations;
	const PxU32 mNumArticulations;

public:

	ExtendedRigidBodyClassificationBase(PxU8* bodies, PxU32 numBodies, PxU32 stride, Dy::FeatherstoneArticulation** articulations, PxU32 numArticulations) :
		mBodies				(bodies),
		mBodyCount			(numBodies),
		mBodySize			(numBodies*stride),
		mStride				(stride),
		mArticulations		(articulations),
		mNumArticulations	(numArticulations)
	{
	}
};

struct ConstraintPartitionArgs
{
	//Input
	PxU8*							mBodies;
	PxU32							mNumBodies;
	PxU32							mStride;
	const ArticulationSolverDesc*	mArticulationPtrs;
	PxU32							mNumArticulationPtrs;
	const PxSolverConstraintDesc*	mContactConstraintDescriptors;
	PxU32							mNumContactConstraintDescriptors;
	//output
	PxSolverConstraintDesc*			mOrderedContactConstraintDescriptors;
	PxSolverConstraintDesc*			mOverflowConstraintDescriptors;
	PxU32							mNumDifferentBodyConstraints;
	PxU32							mNumSelfConstraints;
	PxU32							mNumStaticConstraints;
	PxU32							mNumOverflowConstraints;
	PxArray<PxU32>*					mConstraintsPerPartition;
	//PxArray<PxU32>*					mBitField;	// PT: removed, unused

	PxU32							mMaxPartitions;

	bool							mEnhancedDeterminism;
	bool							mForceStaticConstraintsToSolver;
};

PxU32 partitionContactConstraints(ConstraintPartitionArgs& args);

void processOverflowConstraints(PxU8* bodies, PxU32 bodyStride, PxU32 numBodies, ArticulationSolverDesc* articulations, PxU32 numArticulations,
	PxSolverConstraintDesc* constraints, PxU32 numConstraints);

} // namespace physx

}

#endif
