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
		mNumSelfConstraints(0),
		mNumStaticConstraints(0),
		mNumOverflowConstraints(0)
	{
	}

	PxSolverConstraintDesc*	mOrderedContactConstraintDescriptors;
	PxSolverConstraintDesc*	mOverflowConstraintDescriptors;
	PxArray<PxU32>*			mConstraintsPerPartition;	// PT: actually accumulated constraints per partition
	//PxArray<PxU32>*		mBitField;	// PT: removed, unused
	PxU32					mNumDifferentBodyConstraints;
	PxU32					mNumSelfConstraints;
	PxU32					mNumStaticConstraints;
	PxU32					mNumOverflowConstraints;
};

PxU32 partitionContactConstraints(ConstraintPartitionOut& out, const ConstraintPartitionIn& in);

// PT: TODO: why is this only called for TGS?
void processOverflowConstraints(PxU8* bodies, PxU32 bodyStride, PxU32 numBodies, ArticulationSolverDesc* articulations, PxU32 numArticulations,
	PxSolverConstraintDesc* constraints, PxU32 numConstraints);

} // namespace physx

}

#endif
