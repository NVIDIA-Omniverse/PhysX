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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXS_SIMPLE_ISLAND_GEN_H
#define PXS_SIMPLE_ISLAND_GEN_H

#include "foundation/PxUserAllocated.h"
#include "PxsIslandSim.h"
#include "CmTask.h"

namespace physx
{

namespace Sc
{
	class Interaction;
}
namespace IG
{

	class SimpleIslandManager;

class ThirdPassTask : public Cm::Task
{
	SimpleIslandManager& mIslandManager;
	IslandSim& mIslandSim;

public:

	ThirdPassTask(PxU64 contextID, SimpleIslandManager& islandManager, IslandSim& islandSim);

	virtual void runInternal();

	virtual const char* getName() const
	{
		return "ThirdPassIslandGenTask";
	}

private:
	PX_NOCOPY(ThirdPassTask)
};

class PostThirdPassTask : public Cm::Task
{
	SimpleIslandManager& mIslandManager;

public:

	PostThirdPassTask(PxU64 contextID, SimpleIslandManager& islandManager);

	virtual void runInternal();

	virtual const char* getName() const
	{
		return "PostThirdPassTask";
	}
private:
	PX_NOCOPY(PostThirdPassTask)
};

class SimpleIslandManager : public PxUserAllocated
{
	HandleManager<PxU32> mNodeHandles;						//! Handle manager for nodes
	HandleManager<EdgeIndex> mEdgeHandles;					//! Handle manager for edges

	//An array of destroyed nodes
	PxArray<PxNodeIndex> mDestroyedNodes;
	Cm::BlockArray<Sc::Interaction*> mInteractions;
	

	//Edges destroyed this frame
	PxArray<EdgeIndex> mDestroyedEdges;
	PxArray<PartitionEdge*> mFirstPartitionEdges;
	PxArray<PartitionEdge*> mDestroyedPartitionEdges;
	//KS - stores node indices for a given edge. Node index 0 is at 2* edgeId and NodeIndex1 is at 2*edgeId + 1
	//can also be used for edgeInstance indexing so there's no need to figure out outboundNode ID either!
	Cm::BlockArray<PxNodeIndex> mEdgeNodeIndices;
	Cm::BlockArray<void*> mConstraintOrCm;	//! Pointers to either the constraint or Cm for this pair

	PxBitMap mConnectedMap;

	IslandSim mIslandManager;
	IslandSim mSpeculativeIslandManager;

	ThirdPassTask mSpeculativeThirdPassTask;
	ThirdPassTask mAccurateThirdPassTask;

	PostThirdPassTask mPostThirdPassTask;
	PxU32 mMaxDirtyNodesPerFrame;

	PxU64	mContextID;
public:

	SimpleIslandManager(bool useEnhancedDeterminism, PxU64 contextID);

	~SimpleIslandManager();

	PxNodeIndex addRigidBody(PxsRigidBody* body, bool isKinematic, bool isActive);

	void removeNode(const PxNodeIndex index);

	PxNodeIndex addArticulation(Sc::ArticulationSim* articulation, Dy::FeatherstoneArticulation* llArtic, bool isActive);

#if PX_SUPPORT_GPU_PHYSX
	PxNodeIndex addSoftBody(Dy::SoftBody* llSoftBody, bool isActive);

	PxNodeIndex addFEMCloth(Dy::FEMCloth* llFEMCloth, bool isActive);

	PxNodeIndex addParticleSystem(Dy::ParticleSystem* llParticleSystem, bool isActive);

	PxNodeIndex addHairSystem(Dy::HairSystem* llHairSystem, bool isActive);
#endif

	EdgeIndex addContactManager(PxsContactManager* manager, PxNodeIndex nodeHandle1, PxNodeIndex nodeHandle2, Sc::Interaction* interaction,
		Edge::EdgeType edgeType);

	EdgeIndex addConstraint(Dy::Constraint* constraint, PxNodeIndex nodeHandle1, PxNodeIndex nodeHandle2, Sc::Interaction* interaction);

	bool isConnected(EdgeIndex edgeIndex) const { return !!mConnectedMap.test(edgeIndex); }

	PX_FORCE_INLINE PxNodeIndex getEdgeIndex(EdgeInstanceIndex edgeIndex) const { return mEdgeNodeIndices[edgeIndex]; }

	void activateNode(PxNodeIndex index);
	void deactivateNode(PxNodeIndex index);
	void putNodeToSleep(PxNodeIndex index);

	void removeConnection(EdgeIndex edgeIndex);
	
	void firstPassIslandGen();
	void additionalSpeculativeActivation();
	void secondPassIslandGen();
	void thirdPassIslandGen(PxBaseTask* continuation);

	PX_INLINE void clearDestroyedEdges()
	{
		mDestroyedPartitionEdges.forceSize_Unsafe(0);
	}

	void setEdgeConnected(EdgeIndex edgeIndex, Edge::EdgeType edgeType);
	void setEdgeDisconnected(EdgeIndex edgeIndex);

	bool getIsEdgeConnected(EdgeIndex edgeIndex);

	void setEdgeRigidCM(const EdgeIndex edgeIndex, PxsContactManager* cm);

	void clearEdgeRigidCM(const EdgeIndex edgeIndex);

	void setKinematic(PxNodeIndex nodeIndex);

	void setDynamic(PxNodeIndex nodeIndex);

	const IslandSim& getSpeculativeIslandSim() const { return mSpeculativeIslandManager; }
	const IslandSim& getAccurateIslandSim() const { return mIslandManager; }

	IslandSim& getAccurateIslandSim() { return mIslandManager; }
	IslandSim& getSpeculativeIslandSim() { return mSpeculativeIslandManager; }

	PX_FORCE_INLINE PxU32 getNbEdgeHandles() const { return mEdgeHandles.getTotalHandles(); }

	PX_FORCE_INLINE PxU32 getNbNodeHandles() const { return mNodeHandles.getTotalHandles(); }

	void deactivateEdge(const EdgeIndex edge);

	PX_FORCE_INLINE PxsContactManager* getContactManager(IG::EdgeIndex edgeId) const { return reinterpret_cast<PxsContactManager*>(mConstraintOrCm[edgeId]); }
	PX_FORCE_INLINE PxsContactManager* getContactManagerUnsafe(IG::EdgeIndex edgeId) const { return reinterpret_cast<PxsContactManager*>(mConstraintOrCm[edgeId]); }
	PX_FORCE_INLINE Dy::Constraint* getConstraint(IG::EdgeIndex edgeId) const { return reinterpret_cast<Dy::Constraint*>(mConstraintOrCm[edgeId]); }
	PX_FORCE_INLINE Dy::Constraint* getConstraintUnsafe(IG::EdgeIndex edgeId) const { return reinterpret_cast<Dy::Constraint*>(mConstraintOrCm[edgeId]); }

	PX_FORCE_INLINE Sc::Interaction* getInteraction(IG::EdgeIndex edgeId) const { return mInteractions[edgeId]; }

	PX_FORCE_INLINE	PxU64			getContextId() const { return mContextID; }

	bool checkInternalConsistency();


private:

	friend class ThirdPassTask;
	friend class PostThirdPassTask;

	bool validateDeactivations() const;

	PX_NOCOPY(SimpleIslandManager)
};



}
}

#endif
