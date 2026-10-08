// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "DyThreadContext.h"
#include "foundation/PxBitUtils.h"

namespace physx
{
namespace Dy
{

ThreadContext::ThreadContext(PxcNpMemBlockPool* memBlockPool) :
	mFrictionPatchStreamPair		(*memBlockPool),
	mConstraintBlockManager			(*memBlockPool),
	mConstraintBlockStream			(*memBlockPool),
	mNumDifferentBodyConstraints	(0),
	mNumStaticConstraints			(0),
	mHasOverflowPartitions			(false),
	mNbArticulations				(0),
	mConstraintsPerPartition		("ThreadContext::mConstraintsPerPartition"),
	//mPartitionNormalizationBitmap	("ThreadContext::mPartitionNormalizationBitmap"),
	mBodyCoreArray					(NULL),
	mRigidBodyArray					(NULL),
	mArticulationArray				(NULL),
	motionVelocityArray				(NULL),
	bodyRemapTable					(NULL),
	mNodeIndexArray					(NULL),
	contactConstraintDescArray		(NULL),
	contactDescArraySize			(0),
	orderedContactConstraints		(NULL),
	contactConstraintBatchHeaders	(NULL),
	numContactConstraintBatches		(0),
	tempConstraintDescArray			(NULL),
#if PGS_SUPPORT_COMPOUND_CONSTRAINTS
	compoundConstraints				("ThreadContext::compoundConstraints"),
	orderedContactList				("ThreadContext::orderedContactList"),
	tempContactList					("ThreadContext::tempContactList"),
	sortIndexArray					("ThreadContext::sortIndexArray"),
#endif
	mOrderedContactDescCount		(0),
	mOrderedFrictionDescCount		(0),
	mConstraintSize					(0),
	mAxisConstraintCount			(0),
	mMaxPartitions					(0),
	mMaxFrictionPartitions			(0),
	mMaxSolverPositionIterations	(0),
	mMaxSolverVelocityIterations	(0),
	mMaxArticulationLinks			(0),
	mContactDescPtr					(NULL)
{
#if PX_ENABLE_SIM_STATS
	mThreadSimStats.clear();
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
	//Defaulted to have space for 16384 bodies
	//mPartitionNormalizationBitmap.reserve(512); 
	//Defaulted to have space for 128 partitions (should be more-than-enough)
	mConstraintsPerPartition.reserve(128);
}

void ThreadContext::resizeArrays(PxU32 articulationCount)
{
	mNbArticulations = articulationCount;

	mContactDescPtr = contactConstraintDescArray;
}

void ThreadContext::reset()
{
	// TODO: move these to the PxcNpThreadContext
	mFrictionPatchStreamPair.reset();
	mConstraintBlockStream.reset();

	mContactDescPtr = contactConstraintDescArray;

	mAxisConstraintCount = 0;
	mMaxSolverPositionIterations = 0;
	mMaxSolverVelocityIterations = 0;
	mNumDifferentBodyConstraints = 0;
	mNumStaticConstraints = 0;
	mConstraintSize = 0;
}

}
} 
