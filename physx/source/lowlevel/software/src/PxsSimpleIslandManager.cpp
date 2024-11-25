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

#include "common/PxProfileZone.h"
#include "PxsSimpleIslandManager.h"
#include "foundation/PxSort.h"
#include "PxsContactManager.h"
#include "CmTask.h"
#include "DyVArticulation.h"

using namespace physx;
using namespace IG;

///////////////////////////////////////////////////////////////////////////////

ThirdPassTask::ThirdPassTask(PxU64 contextID, SimpleIslandManager& islandManager, IslandSim& islandSim) : Cm::Task(contextID), mIslandManager(islandManager), mIslandSim(islandSim)
{
}

void ThirdPassTask::runInternal()
{
	PX_PROFILE_ZONE("Basic.thirdPassIslandGen", getContextId());

	mIslandSim.removeDestroyedEdges();
	mIslandSim.processLostEdges(mIslandManager.mDestroyedNodes, true, true, mIslandManager.mMaxDirtyNodesPerFrame);
}

///////////////////////////////////////////////////////////////////////////////

PostThirdPassTask::PostThirdPassTask(PxU64 contextID, SimpleIslandManager& islandManager) : Cm::Task(contextID), mIslandManager(islandManager)
{
}

void PostThirdPassTask::runInternal()
{
	PX_PROFILE_ZONE("Basic.postThirdPassIslandGen", getContextId());

	for (PxU32 a = 0; a < mIslandManager.mDestroyedNodes.size(); ++a)
		mIslandManager.mNodeHandles.freeHandle(mIslandManager.mDestroyedNodes[a].index());

	mIslandManager.mDestroyedNodes.clear();

	for (PxU32 a = 0; a < mIslandManager.mDestroyedEdges.size(); ++a)
		mIslandManager.mEdgeHandles.freeHandle(mIslandManager.mDestroyedEdges[a]);

	mIslandManager.mDestroyedEdges.clear();

	PX_ASSERT(mIslandManager.validateDeactivations());
}

///////////////////////////////////////////////////////////////////////////////

SimpleIslandManager::SimpleIslandManager(bool useEnhancedDeterminism, bool gpu, PxU64 contextID) : 
	mDestroyedNodes				("mDestroyedNodes"), 
	mDestroyedEdges				("mDestroyedEdges"), 
	mAccurateIslandManager		(mCpuData, gpu ? &mGpuData : NULL, contextID),
	mSpeculativeIslandManager	(mCpuData, NULL, contextID),
	mSpeculativeThirdPassTask	(contextID, *this, mSpeculativeIslandManager),
	mAccurateThirdPassTask		(contextID, *this, mAccurateIslandManager),
	mPostThirdPassTask			(contextID, *this),
	mContextID					(contextID),
	mGPU						(gpu)
{
	if(gpu)
		mGpuData.mFirstPartitionEdges.resize(1024);
	mMaxDirtyNodesPerFrame = useEnhancedDeterminism ? 0xFFFFFFFF : 1000u;
}

SimpleIslandManager::~SimpleIslandManager()
{
}

PxNodeIndex SimpleIslandManager::addNode(bool isActive, bool isKinematic, Node::NodeType type, void* object)
{
	const PxU32 handle = mNodeHandles.getHandle();
	const PxNodeIndex nodeIndex(handle);
	mAccurateIslandManager		.addNode(isActive, isKinematic, type, nodeIndex, object);
	mSpeculativeIslandManager	.addNode(isActive, isKinematic, type, nodeIndex, object);
	return nodeIndex;
}

void SimpleIslandManager::removeNode(const PxNodeIndex index)
{
	PX_ASSERT(mNodeHandles.isValidHandle(index.index()));
	mDestroyedNodes.pushBack(index);
}

EdgeIndex SimpleIslandManager::addEdge(void* edge, PxNodeIndex nodeHandle1, PxNodeIndex nodeHandle2, Sc::Interaction* interaction)
{
	const EdgeIndex handle = mEdgeHandles.getHandle();

	const PxU32 nodeIds = 2 * handle;
	if (mCpuData.mEdgeNodeIndices.size() == nodeIds)
	{
		PX_PROFILE_ZONE("ReserveEdges", getContextId());
		const PxU32 newSize = nodeIds + 2048;
		mCpuData.mEdgeNodeIndices.resize(newSize);
		// PT: TODO: isn't that twice as much as needed for these arrays?
		mAuxCpuData.mConstraintOrCm.resize(newSize);
		mInteractions.resize(newSize);
	}

	mCpuData.mEdgeNodeIndices[nodeIds] = nodeHandle1;
	mCpuData.mEdgeNodeIndices[nodeIds + 1] = nodeHandle2;
	mAuxCpuData.mConstraintOrCm[handle] = edge;
	mInteractions[handle] = interaction;

	return handle;
}

EdgeIndex SimpleIslandManager::resizeEdgeArrays(EdgeIndex handle, bool flag)
{
	if(mConnectedMap.size() == handle)
		mConnectedMap.resize(2 * (handle + 1));

	if(mGPU && mGpuData.mFirstPartitionEdges.capacity() == handle)
		mGpuData.mFirstPartitionEdges.resize(2 * (handle + 1));

	if(flag)
		mConnectedMap.reset(handle);	// PT: for contact manager
	else
		mConnectedMap.set(handle);		// PT: for constraint

	return handle;
}

EdgeIndex SimpleIslandManager::addContactManager(PxsContactManager* manager, PxNodeIndex nodeHandle1, PxNodeIndex nodeHandle2, Sc::Interaction* interaction, Edge::EdgeType edgeType)
{
	const EdgeIndex handle = addEdge(manager, nodeHandle1, nodeHandle2, interaction);

	mSpeculativeIslandManager.addConnection(nodeHandle1, nodeHandle2, edgeType, handle);

	if (manager)
		manager->getWorkUnit().mEdgeIndex = handle;

	return resizeEdgeArrays(handle, true);
}

EdgeIndex SimpleIslandManager::addConstraint(Dy::Constraint* constraint, PxNodeIndex nodeHandle1, PxNodeIndex nodeHandle2, Sc::Interaction* interaction)
{
	const EdgeIndex handle = addEdge(constraint, nodeHandle1, nodeHandle2, interaction);

	mAccurateIslandManager.addConnection(nodeHandle1, nodeHandle2, Edge::eCONSTRAINT, handle);
	mSpeculativeIslandManager.addConnection(nodeHandle1, nodeHandle2, Edge::eCONSTRAINT, handle);

	return resizeEdgeArrays(handle, false);
}

void SimpleIslandManager::activateNode(PxNodeIndex index)
{
	mAccurateIslandManager.activateNode(index);
	mSpeculativeIslandManager.activateNode(index);
}

void SimpleIslandManager::deactivateNode(PxNodeIndex index)
{
	mAccurateIslandManager.deactivateNode(index);
	mSpeculativeIslandManager.deactivateNode(index);
}

void SimpleIslandManager::putNodeToSleep(PxNodeIndex index)
{
	mAccurateIslandManager.putNodeToSleep(index);
	mSpeculativeIslandManager.putNodeToSleep(index);
}

void SimpleIslandManager::removeConnection(EdgeIndex edgeIndex)
{
	if(edgeIndex == IG_INVALID_EDGE)
		return;
	mDestroyedEdges.pushBack(edgeIndex);
	mSpeculativeIslandManager.removeConnection(edgeIndex);
	if(mConnectedMap.test(edgeIndex))
	{
		mAccurateIslandManager.removeConnection(edgeIndex);
		mConnectedMap.reset(edgeIndex);
	}

	mAuxCpuData.mConstraintOrCm[edgeIndex] = NULL;
	mInteractions[edgeIndex] = NULL;
}

void SimpleIslandManager::firstPassIslandGen()
{
	PX_PROFILE_ZONE("Basic.firstPassIslandGen", getContextId());
	mSpeculativeIslandManager.clearDeactivations();
	mSpeculativeIslandManager.wakeIslands();
	mSpeculativeIslandManager.processNewEdges();
	mSpeculativeIslandManager.removeDestroyedEdges();
	mSpeculativeIslandManager.processLostEdges(mDestroyedNodes, false, false, mMaxDirtyNodesPerFrame);
}

void SimpleIslandManager::additionalSpeculativeActivation()
{
	mSpeculativeIslandManager.wakeIslands2();
}

void SimpleIslandManager::secondPassIslandGen()
{
	PX_PROFILE_ZONE("Basic.secondPassIslandGen", getContextId());
	
	mAccurateIslandManager.wakeIslands();
	mAccurateIslandManager.processNewEdges();

	mAccurateIslandManager.removeDestroyedEdges();
	mAccurateIslandManager.processLostEdges(mDestroyedNodes, false, false, mMaxDirtyNodesPerFrame);

	for(PxU32 a = 0; a < mDestroyedNodes.size(); ++a)
		mNodeHandles.freeHandle(mDestroyedNodes[a].index());

	mDestroyedNodes.clear();
	//mDestroyedEdges.clear();
}

bool SimpleIslandManager::validateDeactivations() const
{
	//This method sanity checks the deactivations produced by third-pass island gen. Specifically, it ensures that any bodies that 
	//the speculative IG wants to deactivate are also candidates for deactivation in the accurate island gen. In practice, both should be the case. If this fails, something went wrong...

	const PxNodeIndex* const nodeIndices = mSpeculativeIslandManager.getNodesToDeactivate(Node::eRIGID_BODY_TYPE);
	const PxU32 nbNodesToDeactivate = mSpeculativeIslandManager.getNbNodesToDeactivate(Node::eRIGID_BODY_TYPE);

	for(PxU32 i = 0; i < nbNodesToDeactivate; ++i)
	{
		//Node is active in accurate sim => mismatch between accurate and inaccurate sim!
		const Node& node = mAccurateIslandManager.getNode(nodeIndices[i]);
		const Node& speculativeNode = mSpeculativeIslandManager.getNode(nodeIndices[i]);
		//KS - we need to verify that the bodies in the "deactivating" list are still candidates for deactivation. There are cases where they may not no longer be candidates, e.g. if the application
		//put bodies to sleep and activated them
		if(node.isActive() && !speculativeNode.isActive())
			return false;
	}
	return true;
}

void SimpleIslandManager::thirdPassIslandGen(PxBaseTask* continuation)
{
	mAccurateIslandManager.clearDeactivations();

	mPostThirdPassTask.setContinuation(continuation);
	
	mSpeculativeThirdPassTask.setContinuation(&mPostThirdPassTask);
	mAccurateThirdPassTask.setContinuation(&mPostThirdPassTask);

	mSpeculativeThirdPassTask.removeReference();
	mAccurateThirdPassTask.removeReference();

	mPostThirdPassTask.removeReference();

	//PX_PROFILE_ZONE("Basic.thirdPassIslandGen", getContextId());
	//mSpeculativeIslandManager.removeDestroyedEdges();
	//mSpeculativeIslandManager.processLostEdges(mDestroyedNodes, true, true);

	//mAccurateIslandManager.removeDestroyedEdges();
	//mAccurateIslandManager.processLostEdges(mDestroyedNodes, true, true);
}

bool SimpleIslandManager::checkInternalConsistency()
{
	return mAccurateIslandManager.checkInternalConsistency() && mSpeculativeIslandManager.checkInternalConsistency();
}

void SimpleIslandManager::setEdgeConnected(EdgeIndex edgeIndex, Edge::EdgeType edgeType)
{
	if(!mConnectedMap.test(edgeIndex))
	{
		mAccurateIslandManager.addConnection(mCpuData.mEdgeNodeIndices[edgeIndex * 2], mCpuData.mEdgeNodeIndices[edgeIndex * 2 + 1], edgeType, edgeIndex);
		mConnectedMap.set(edgeIndex);
	}
}

void SimpleIslandManager::deactivateEdge(const EdgeIndex edgeIndex)
{
	if (mGPU && mGpuData.mFirstPartitionEdges[edgeIndex])
	{
		//this is the partition edges created/updated by the gpu solver
		mGpuData.mDestroyedPartitionEdges.pushBack(mGpuData.mFirstPartitionEdges[edgeIndex]);
		mGpuData.mFirstPartitionEdges[edgeIndex] = NULL;
	}
}

void SimpleIslandManager::setEdgeDisconnected(EdgeIndex edgeIndex)
{
	if(mConnectedMap.test(edgeIndex))
	{
		//PX_ASSERT(!mAccurateIslandManager.getEdge(edgeIndex).isInDirtyList());
		mAccurateIslandManager.removeConnection(edgeIndex);
		mConnectedMap.reset(edgeIndex);
	}
}

void SimpleIslandManager::setEdgeRigidCM(const EdgeIndex edgeIndex, PxsContactManager* cm)
{
	mAuxCpuData.mConstraintOrCm[edgeIndex] = cm;
	cm->getWorkUnit().mEdgeIndex = edgeIndex;
}

void SimpleIslandManager::clearEdgeRigidCM(const EdgeIndex edgeIndex)
{
	mAuxCpuData.mConstraintOrCm[edgeIndex] = NULL;
	deactivateEdge(edgeIndex);
}

void SimpleIslandManager::setKinematic(PxNodeIndex nodeIndex) 
{ 
	mAccurateIslandManager.setKinematic(nodeIndex); 
	mSpeculativeIslandManager.setKinematic(nodeIndex);
}

void SimpleIslandManager::setDynamic(PxNodeIndex nodeIndex) 
{ 
	mAccurateIslandManager.setDynamic(nodeIndex); 
	mSpeculativeIslandManager.setDynamic(nodeIndex);
}
