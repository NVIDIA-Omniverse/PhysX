// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_CONSTRAINT_PARTITION_H
#define DY_CONSTRAINT_PARTITION_H

#include "DyDynamics.h"
#include "DyFeatherstoneArticulation.h"

namespace physx
{
namespace Dy
{
// PT: input of partitionContactConstraints
struct ConstraintPartitionIn
{
	ConstraintPartitionIn(	PxU8* bodies, PxU32 nbBodies, PxU32 stride,
							Dy::FeatherstoneArticulation** articulations, PxU32 nbArticulations,
							const PxSolverConstraintDesc* contactConstraintDescs, PxU32 nbContactConstraintDescs,
							PxU32 maxPartitions, bool forceStaticConstraintsToSolver) :
		mBodies	(bodies), mNumBodies(nbBodies), mStride(stride),
		mArticulationPtrs(articulations), mNumArticulationPtrs(nbArticulations),
		mContactConstraintDescriptors(contactConstraintDescs), mNumContactConstraintDescriptors(nbContactConstraintDescs),
		mMaxPartitions(maxPartitions), mForceStaticConstraintsToSolver(forceStaticConstraintsToSolver)
	{
	}

	PxU8*							mBodies;							// PT: PxSolverBody (PGS) or PxTGSSolverBodyVel (TGS)
	PxU32							mNumBodies;
	PxU32							mStride;
	Dy::FeatherstoneArticulation**	mArticulationPtrs;
	PxU32							mNumArticulationPtrs;
	const PxSolverConstraintDesc*	mContactConstraintDescriptors;
	PxU32							mNumContactConstraintDescriptors;
	PxU32							mMaxPartitions;						// PT: limit the number of "resizes" beyond the initial 32
	bool							mForceStaticConstraintsToSolver;	// PT: only for PGS + point-friction
};

// PT: output of partitionContactConstraints
struct ConstraintPartitionOut
{
	ConstraintPartitionOut(PxSolverConstraintDesc* orderedContactConstraintDescriptors, PxSolverConstraintDesc* overflowConstraintDescriptors, PxArray<PxU32>* constraintsPerPartition) :
		mOrderedContactConstraintDescriptors(orderedContactConstraintDescriptors),
		mOverflowConstraintDescriptors(overflowConstraintDescriptors),
		mConstraintsPerPartition(constraintsPerPartition),
		mNumDifferentBodyConstraints(0),
		mNumStaticConstraints(0),
		mNumOverflowConstraints(0)
	{
	}

	PxSolverConstraintDesc*	mOrderedContactConstraintDescriptors;
	PxSolverConstraintDesc*	mOverflowConstraintDescriptors;
	PxArray<PxU32>*			mConstraintsPerPartition;	// PT: actually accumulated constraints per partition
	PxU32					mNumDifferentBodyConstraints;
	PxU32					mNumStaticConstraints;
	PxU32					mNumOverflowConstraints;
};

PxU32 partitionContactConstraints(ConstraintPartitionOut& out, const ConstraintPartitionIn& in);

// PT: TODO: why is this only called for TGS?
void processOverflowConstraints(PxU8* bodies, PxU32 bodyStride, PxU32 numBodies, FeatherstoneArticulation** articulations, PxU32 numArticulations,
	PxSolverConstraintDesc* constraints, PxU32 numConstraints);

} // namespace physx

}

#endif
