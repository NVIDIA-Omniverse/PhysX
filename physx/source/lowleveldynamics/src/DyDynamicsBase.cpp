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

#include "DyDynamicsBase.h"
#include "PxsIslandSim.h"
#include "common/PxProfileZone.h"

using namespace physx;
using namespace Dy;

DynamicsContextBase::DynamicsContextBase(
	PxcNpMemBlockPool* memBlockPool,
	Cm::FlushPool& taskPool,
	PxvSimStats& simStats,
	PxVirtualAllocatorCallback* allocatorCallback,
	PxsMaterialManager* materialManager,
	IG::SimpleIslandManager& islandManager,
	PxU64 contextID,
	PxReal maxBiasCoefficient,
	PxReal lengthScale,
	bool enableStabilization,
	bool useEnhancedDeterminism,
	bool isResidualReportingEnabled
	) :
	Dy::Context			(islandManager, allocatorCallback, simStats, enableStabilization, useEnhancedDeterminism, maxBiasCoefficient, lengthScale, contextID, isResidualReportingEnabled),
	mThreadContextPool	(memBlockPool),
	mMaterialManager	(materialManager),
	mTaskPool			(taskPool),
	mKinematicCount		(0),
	mThresholdStreamOut	(0),
	mCurrentIndex		(0)
{
}

DynamicsContextBase::~DynamicsContextBase()
{
}

void DynamicsContextBase::resetThreadContexts()
{
	PxcThreadCoherentCacheIterator<ThreadContext, PxcNpMemBlockPool> threadContextIt(mThreadContextPool);
	ThreadContext* threadContext = threadContextIt.getNext();

	while(threadContext != NULL)
	{
		threadContext->reset();
		threadContext = threadContextIt.getNext();
	}
}

PxU32 DynamicsContextBase::reserveSharedSolverConstraintsArrays(const IG::IslandSim& islandSim, PxU32 maxArticulationLinks)
{
	PX_PROFILE_ZONE("reserveSharedSolverConstraintsArrays", mContextID);

	const PxU32 bodyCount = islandSim.getNbActiveNodes(IG::Node::eRIGID_BODY_TYPE);

	const PxU32 numArtics = islandSim.getNbActiveNodes(IG::Node::eARTICULATION_TYPE);

	const PxU32 numArticulationConstraints = numArtics * maxArticulationLinks; //Just allocate enough memory to fit worst-case maximum size articulations...

	const PxU32 nbActiveContactManagers = islandSim.getNbActiveEdges(IG::Edge::eCONTACT_MANAGER);
	const PxU32 nbActiveConstraints = islandSim.getNbActiveEdges(IG::Edge::eCONSTRAINT);

	const PxU32 totalConstraintCount = nbActiveConstraints + nbActiveContactManagers + numArticulationConstraints;

	mContactConstraintBatchHeaders.forceSize_Unsafe(0);
	mContactConstraintBatchHeaders.reserve((totalConstraintCount + 63) & (~63));
	mContactConstraintBatchHeaders.forceSize_Unsafe(totalConstraintCount);

	mContactList.forceSize_Unsafe(0);
	mContactList.reserve((nbActiveContactManagers + 63u) & (~63u));
	mContactList.forceSize_Unsafe(nbActiveContactManagers);

	mMotionVelocityArray.forceSize_Unsafe(0);
	mMotionVelocityArray.reserve((bodyCount + 63u) & (~63u));
	mMotionVelocityArray.forceSize_Unsafe(bodyCount);

	mBodyCoreArray.forceSize_Unsafe(0);
	mBodyCoreArray.reserve((bodyCount + 63u) & (~63u));
	mBodyCoreArray.forceSize_Unsafe(bodyCount);

	mRigidBodyArray.forceSize_Unsafe(0);
	mRigidBodyArray.reserve((bodyCount + 63u) & (~63u));
	mRigidBodyArray.forceSize_Unsafe(bodyCount);

	mArticulationArray.forceSize_Unsafe(0);
	mArticulationArray.reserve((numArtics + 63u) & (~63u));
	mArticulationArray.forceSize_Unsafe(numArtics);

	mNodeIndexArray.forceSize_Unsafe(0);
	mNodeIndexArray.reserve((bodyCount + 63u) & (~63u));
	mNodeIndexArray.forceSize_Unsafe(bodyCount);

	ThresholdStream& stream = getThresholdStream();
	stream.forceSize_Unsafe(0);
	stream.reserve(PxNextPowerOfTwo(nbActiveContactManagers != 0 ? nbActiveContactManagers - 1 : nbActiveContactManagers));

	//flip exceeded force threshold buffer
	mCurrentIndex = 1 - mCurrentIndex;

	return totalConstraintCount;
}
