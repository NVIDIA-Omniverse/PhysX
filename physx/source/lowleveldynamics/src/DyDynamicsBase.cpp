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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "DyDynamicsBase.h"
#include "DyIslandManager.h"
#include "foundation/PxSort.h"
#include "common/PxProfileZone.h"
#include "PxsSimpleIslandManager.h"
#include "PxsContactManager.h"
#include "PxvSimStats.h"

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
	PxSceneFlags sceneFlags) :
	Dy::Context			(islandManager, allocatorCallback, simStats, maxBiasCoefficient, lengthScale, contextID, sceneFlags),
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

bool DynamicsContextBase::updateShared(PxvNphaseImplementationContext* nphase, PxReal dt, const PxVec3& gravity)
{
	PX_PROFILE_ZONE("Dynamics.updateShared", mContextID);

	mOutputIterator = nphase->getContactManagerOutputs();

	mDt = dt;
	mInvDt = dt == 0.0f ? 0.0f : 1.0f / dt;
	mGravity = gravity;

	const IG::IslandSim& islandSim = mIslandManager.getAccurateIslandSim();

	const PxU32 islandCount = islandSim.getNbActiveIslands();

	const PxU32 activatedContactCount = islandSim.getNbActivatedEdges(IG::Edge::eCONTACT_MANAGER);
	const IG::EdgeIndex* const activatingEdges = islandSim.getActivatedEdges(IG::Edge::eCONTACT_MANAGER);

	{
		PX_PROFILE_ZONE("resetFrictionPatchCount", mContextID);

		for (PxU32 a = 0; a < activatedContactCount; ++a)
		{
			PxsContactManager* cm = mIslandManager.getContactManager(activatingEdges[a]);
			if (cm)
				cm->getWorkUnit().mFrictionPatchCount = 0; //KS - zero the friction patch count on any activating edges
		}
	}

#if PX_ENABLE_SIM_STATS
	if (islandCount > 0)
	{
		mSimStats.mNbActiveKinematicBodies = islandSim.getNbActiveKinematics();
		mSimStats.mNbActiveDynamicBodies = islandSim.getNbActiveNodes(IG::Node::eRIGID_BODY_TYPE);
		mSimStats.mNbActiveConstraints = islandSim.getNbActiveEdges(IG::Edge::eCONSTRAINT);
	}
	else
	{
		mSimStats.mNbActiveKinematicBodies = islandSim.getNbActiveKinematics();
		mSimStats.mNbActiveDynamicBodies = 0;
		mSimStats.mNbActiveConstraints = 0;
	}
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

	mThresholdStreamOut = 0;

	// Reset thread contexts
	{
		PxcThreadCoherentCacheIterator<ThreadContext, PxcNpMemBlockPool> threadContextIt(mThreadContextPool);
		ThreadContext* threadContext = threadContextIt.getNext();
		while(threadContext)
		{
			threadContext->reset();
			threadContext = threadContextIt.getNext();
		}
	}

	//If there is no work to do then we can do nothing at all.
	if(!islandCount)
		return false;

	return true;
}

namespace physx
{
namespace Dy
{
struct EnhancedSortPredicate
{
	bool operator()(const PxsIndexedContactManager& left, const PxsIndexedContactManager& right) const
	{
		const PxcNpWorkUnit& unit0 = left.contactManager->getWorkUnit();
		const PxcNpWorkUnit& unit1 = right.contactManager->getWorkUnit();
		return (unit0.mTransformCache0 < unit1.mTransformCache0) ||
			((unit0.mTransformCache0 == unit1.mTransformCache0) && (unit0.mTransformCache1 < unit1.mTransformCache1));
	}
};
}
}

PxU32 DynamicsContextBase::iterateIslandsContactEdges(	PxsIslandIndices& counts, PxU32 nbIslands, const IG::IslandId* PX_RESTRICT const islandIds,
														PxsIndexedContactManager* PX_RESTRICT indexedManagers, const PxU32* PX_RESTRICT bodyRemapTable)
{
	PX_PROFILE_ZONE("IterateIslandsContactEdges", mContextID);
	PX_UNUSED(counts);

	const IG::IslandSim& islandSim = mIslandManager.getAccurateIslandSim();

	PxU32 currentContactIndex = 0;
	for(PxU32 i=0; i<nbIslands; ++i)
	{
		const IG::Island& island = islandSim.getIsland(islandIds[i]);

		IG::EdgeIndex contactEdgeIndex = island.mFirstEdge[IG::Edge::eCONTACT_MANAGER];

		while(contactEdgeIndex != IG_INVALID_EDGE)
		{
			const IG::Edge& edge = islandSim.getEdge(contactEdgeIndex);

			PxsContactManager* contactManager = mIslandManager.getContactManager(contactEdgeIndex);

			if(contactManager)
			{
				const PxNodeIndex nodeIndex1 = islandSim.mCpuData.getNodeIndex1(contactEdgeIndex);
				const PxNodeIndex nodeIndex2 = islandSim.mCpuData.getNodeIndex2(contactEdgeIndex);

				PxsIndexedContactManager& indexedManager = indexedManagers[currentContactIndex++];
				indexedManager.contactManager = contactManager;

				PX_ASSERT(!nodeIndex1.isStaticBody());
				{
					const IG::Node& node1 = islandSim.getNode(nodeIndex1);

					//Is it an articulation or not???
					if(node1.getNodeType() == IG::Node::eARTICULATION_TYPE)
					{
						indexedManager.articulation0 = nodeIndex1.getInd();
						indexedManager.indexType0 = PxsIndexedInteraction::eARTICULATION;
					}
					else
					{
						if(node1.isKinematic())
						{
							indexedManager.indexType0 = PxsIndexedInteraction::eKINEMATIC;
							indexedManager.solverBody0 = islandSim.getActiveNodeIndex(nodeIndex1);
						}
						else
						{
							indexedManager.indexType0 = PxsIndexedInteraction::eBODY;
							indexedManager.solverBody0 = bodyRemapTable[islandSim.getActiveNodeIndex(nodeIndex1)];
						}
						PX_ASSERT(indexedManager.solverBody0 < (counts.bodies + mKinematicCount + 1));
					}
				}

				if(nodeIndex2.isStaticBody())
				{
					indexedManager.indexType1 = PxsIndexedInteraction::eWORLD;
				}
				else
				{
					const IG::Node& node2 = islandSim.getNode(nodeIndex2);

					//Is it an articulation or not???
					if(node2.getNodeType() == IG::Node::eARTICULATION_TYPE)
					{
						indexedManager.articulation1 = nodeIndex2.getInd();
						indexedManager.indexType1 = PxsIndexedInteraction::eARTICULATION;
					}
					else
					{
						if(node2.isKinematic())
						{
							indexedManager.indexType1 = PxsIndexedInteraction::eKINEMATIC;
							indexedManager.solverBody1 = islandSim.getActiveNodeIndex(nodeIndex2);
						}
						else
						{
							indexedManager.indexType1 = PxsIndexedInteraction::eBODY;
							indexedManager.solverBody1 = bodyRemapTable[islandSim.getActiveNodeIndex(nodeIndex2)];
						}
						PX_ASSERT(indexedManager.solverBody1 < (counts.bodies + mKinematicCount + 1));
					}
				}
			}
			contactEdgeIndex = edge.mNextIslandEdge;
		}
	}

	if(mUseEnhancedDeterminism)
		PxSort(indexedManagers, currentContactIndex, EnhancedSortPredicate());

	return currentContactIndex;
}

void DynamicsContextBase::iterateIslandsNodes(	PxsIslandIndices& counts, PxU32 nbIslands, const IG::IslandId* PX_RESTRICT const islandIds,
												PxsBodyCore** PX_RESTRICT bodyArrayPtr, PxsRigidBody** PX_RESTRICT rigidBodyPtr,
												FeatherstoneArticulation** PX_RESTRICT articulationPtr,
												PxU32* PX_RESTRICT bodyRemapTable, PxU32* PX_RESTRICT nodeIndexArray)
{
	PX_PROFILE_ZONE("IterateIslandsNodes", mContextID);
	PX_UNUSED(counts);

	const IG::IslandSim& islandSim = mIslandManager.getAccurateIslandSim();

	PxU32 bodyIndex = 0, articIndex = 0;
	for (PxU32 i = 0; i < nbIslands; ++i)
	{
		const IG::Island& island = islandSim.getIsland(islandIds[i]);

		PxNodeIndex currentIndex = island.mRootNode;

		while (currentIndex.isValid())
		{
			const IG::Node& node = islandSim.getNode(currentIndex);

			if (node.getNodeType() == IG::Node::eARTICULATION_TYPE)
			{
				articulationPtr[articIndex++] = getObjectFromIG<FeatherstoneArticulation>(node);
			}
			else
			{
				PX_ASSERT(bodyIndex < (counts.bodies + mKinematicCount + 1));
				if(!mUseEnhancedDeterminism)
				{
					// PT: we can merge the two loops together without enhanced determinism
					PxsRigidBody* rigid = getObjectFromIG<PxsRigidBody>(node);
					rigidBodyPtr[bodyIndex] = rigid;
					bodyArrayPtr[bodyIndex] = &rigid->getCore();
					bodyRemapTable[islandSim.getActiveNodeIndex(currentIndex)] = bodyIndex;
				}
				nodeIndexArray[bodyIndex++] = currentIndex.index();
			}

			currentIndex = node.mNextNode;
		}
	}

	if(mUseEnhancedDeterminism)
	{
		//Bodies can come in a slightly jumbled order from islandGen. It's deterministic if the scene is 
		//identical but can vary if there are additional bodies in the scene in a different island.
		PxSort(nodeIndexArray, bodyIndex);

		for (PxU32 a = 0; a < bodyIndex; ++a)
		{
			const PxNodeIndex currentIndex(nodeIndexArray[a]);
			PxsRigidBody* rigid = getRigidBodyFromIG(islandSim, currentIndex);
			rigidBodyPtr[a] = rigid;
			bodyArrayPtr[a] = &rigid->getCore();
			nodeIndexArray[a] = currentIndex.index();
			bodyRemapTable[islandSim.getActiveNodeIndex(currentIndex)] = a;
		}
	}
}
