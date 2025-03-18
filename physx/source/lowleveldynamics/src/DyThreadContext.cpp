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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "DyThreadContext.h"
#include "foundation/PxBitUtils.h"

namespace physx
{
namespace Dy
{

ThreadContext::ThreadContext(PxcNpMemBlockPool* memBlockPool) :
	mFrictionPatchStreamPair				(*memBlockPool),
	mConstraintBlockManager					(*memBlockPool),
	mConstraintBlockStream					(*memBlockPool),
	mNumDifferentBodyConstraints			(0),
	mNumStaticConstraints					(0),
	mHasOverflowPartitions					(false),
	mConstraintsPerPartition				("ThreadContext::mConstraintsPerPartition"),
	//mPartitionNormalizationBitmap			("ThreadContext::mPartitionNormalizationBitmap"),
	mBodyCoreArray							(NULL),
	mRigidBodyArray							(NULL),
	mArticulationArray						(NULL),
	motionVelocityArray						(NULL),
	bodyRemapTable							(NULL),
	mNodeIndexArray							(NULL),
	contactConstraintDescArray				(NULL),
	contactDescArraySize					(0),
	orderedContactConstraints				(NULL),
	contactConstraintBatchHeaders			(NULL),
	numContactConstraintBatches				(0),
	tempConstraintDescArray					(NULL),
#if PGS_SUPPORT_COMPOUND_CONSTRAINTS
	compoundConstraints						("ThreadContext::compoundConstraints"),
	orderedContactList						("ThreadContext::orderedContactList"),
	tempContactList							("ThreadContext::tempContactList"),
	sortIndexArray							("ThreadContext::sortIndexArray"),
#endif
	mOrderedContactDescCount				(0),
	mOrderedFrictionDescCount				(0),
	mConstraintSize							(0),
	mAxisConstraintCount					(0),
	mMaxPartitions							(0),
	mMaxFrictionPartitions					(0),
	mMaxSolverPositionIterations			(0),
	mMaxSolverVelocityIterations			(0),
	mMaxArticulationLinks					(0),
	mContactDescPtr							(NULL),
	mArticulations							("ThreadContext::articulations")
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
	mArticulations.forceSize_Unsafe(0);
	mArticulations.reserve(PxMax<PxU32>(PxNextPowerOfTwo(articulationCount), 16));
	mArticulations.forceSize_Unsafe(articulationCount);

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
