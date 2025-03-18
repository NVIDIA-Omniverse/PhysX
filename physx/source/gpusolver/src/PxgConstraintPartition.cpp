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

#include "common/PxProfileZone.h"

#include "PxgConstraintPartition.h"
#include "PxcNpWorkUnit.h"
#include "PxsContactManager.h"
#include "PxsContactManagerState.h"
#include "PxgConstraintPrep.h"
#include "PxvNphaseImplementationContext.h"
#include "PxgBodySimManager.h"
#include "PxgJointManager.h"
#include "PxvDynamics.h"
#include "CmFlushPool.h"

using namespace physx;

// PT: unfortunately these don't seem to be just an optimization, several UTs fail if we disable them
// ===> which means things will fail when the buffers in the body sim manager are full
#define ARTIC_STATIC_EDGES_INTERNAL_SOLVER	1
#define ARTIC_SELF_CONSTRAINT_SOLVER		1
#define RIGID_STATIC_EDGE_SOLVER			1

// PT: getDestroyedContactEdgeIndices() is only called after updateIncrementalIslands(), which resets
// mDestroyedContactEdgeIndices. So it looks like any recorded data in that array before that point is never consumed.
#define RECORD_DESTROYED_EDGES_IN_FOUND_LOST_PASSES	0

#define USE_FINE_GRAINED_PROFILE_ZONES		0

// PT: sanity check - run default versions
static const bool gRunDefaultVersion = false;

/////

// PT: note that inside PartitionEdgeManager we reuse PartitionEdge::mNextPatch for a linked-list of free
// edges but it has nothing to do with contact patches and we could be using another place in PartitionEdge.
PartitionEdgeManager::PartitionEdgeManager() : mFreeEdges(NULL), mEdgeCount(0)
{
}

PartitionEdgeManager::~PartitionEdgeManager()
{
	const PxU32 size = mMemory.size();
	for(PxU32 a=0; a<size; a++)
	{
		// PT: this is useless, and we didn't call new or placement new on PartitionEdgeSlab either anyway.
		//PartitionEdgeSlab* currentSlab = mPartitionEdgeSlabs[a];
		//currentSlab->~PartitionEdgeSlab();

		void* memory = mMemory[a];
		PX_FREE(memory);
	}
	mPartitionEdgeSlabs.clear();
	mMemory.clear();
}

void PartitionEdgeManager::allocateSlab()
{
	PX_ASSERT(mFreeEdges == NULL);
	PartitionEdgeSlab* newSlab;
	{
		// PT: align slab base address so that an edge doesn't cross a cache line
		void* memory = PX_ALLOC((sizeof(PartitionEdgeSlab)+32), "PartitionEdgeSlab");
		const size_t aligned = (size_t(memory) + 31) & size_t(~31);
		newSlab = reinterpret_cast<PartitionEdgeSlab*>(aligned);

		mPartitionEdgeSlabs.pushBack(newSlab);
		mMemory.pushBack(memory);
	}

	PartitionEdge* edges = &newSlab->mEdges[0];

	PxU32 currentIndex = mEdgeCount;

	edges->mUniqueIndex = currentIndex++;

	for(PxU32 a = 1; a < SLAB_SIZE; ++a)
	{
		edges[a-1].mNextPatch = &edges[a];
		edges[a].mUniqueIndex = currentIndex++;
	}
	edges[SLAB_SIZE-1].mNextPatch = NULL;
	mEdgeCount += SLAB_SIZE;
	mFreeEdges = edges;
}

PX_FORCE_INLINE PartitionEdge* PartitionEdgeManager::getEdge(IG::EdgeIndex index)
{
	if(mFreeEdges == NULL)
		allocateSlab();

	PX_ASSERT(mFreeEdges != NULL);

	PartitionEdge* edge = mFreeEdges;
	mFreeEdges = mFreeEdges->mNextPatch;

	PX_PLACEMENT_NEW(edge, PartitionEdge(index));
	return edge;
}

PX_FORCE_INLINE void PartitionEdgeManager::putEdge(PartitionEdge* edge)
{
	edge->mNextPatch = mFreeEdges;
	mFreeEdges = edge;
}

/////

static PX_FORCE_INLINE void increaseNodeInteractionCounts(PxInt32ArrayPinned& nodeInteractionCountArray, PxNodeIndex nodeIndex1, PxNodeIndex nodeIndex2)
{
	if(nodeIndex1.isValid())
		nodeInteractionCountArray[nodeIndex1.index()]++;
	if(nodeIndex2.isValid())
		nodeInteractionCountArray[nodeIndex2.index()]++;
}

static PX_FORCE_INLINE void increaseNodeInteractionCountsMT(PxInt32ArrayPinned& nodeInteractionCountArray, PxNodeIndex nodeIndex1, PxNodeIndex nodeIndex2)
{
	if(nodeIndex1.isValid())
		PxAtomicIncrement(reinterpret_cast<volatile PxI32*>(&nodeInteractionCountArray[nodeIndex1.index()]));
	if(nodeIndex2.isValid())
		PxAtomicIncrement(reinterpret_cast<volatile PxI32*>(&nodeInteractionCountArray[nodeIndex2.index()]));
}

static PX_FORCE_INLINE void decreaseNodeInteractionCounts(PxInt32ArrayPinned& nodeInteractionCountArray, PxNodeIndex nodeIndex1, PxNodeIndex nodeIndex2)
{
	if(nodeIndex1.isValid())
		nodeInteractionCountArray[nodeIndex1.index()]--;
	if(nodeIndex2.isValid())
		nodeInteractionCountArray[nodeIndex2.index()]--;
}

static PX_FORCE_INLINE void decreaseNodeInteractionCountsMT(PxInt32ArrayPinned& nodeInteractionCountArray, PxNodeIndex nodeIndex1, PxNodeIndex nodeIndex2)
{
	if(nodeIndex1.isValid())
		PxAtomicDecrement(reinterpret_cast<volatile PxI32*>(&nodeInteractionCountArray[nodeIndex1.index()]));
	if(nodeIndex2.isValid())
		PxAtomicDecrement(reinterpret_cast<volatile PxI32*>(&nodeInteractionCountArray[nodeIndex2.index()]));
}

/////

static PX_FORCE_INLINE void increaseForceThresholds(const PxcNpWorkUnit& unit, PartitionEdge* edge, PxU32& nbForceThresholds)
{
	if(unit.mFlags & PxcNpWorkUnitFlag::eFORCE_THRESHOLD)
	{
		edge->setHasThreshold();
		nbForceThresholds++;
	}
}

static PX_FORCE_INLINE void increaseForceThresholdsMT(const PxcNpWorkUnit& unit, PartitionEdge* edge, PxU32* nbForceThresholds)
{
	if(unit.mFlags & PxcNpWorkUnitFlag::eFORCE_THRESHOLD)
	{
		edge->setHasThreshold();
		PxAtomicIncrement(reinterpret_cast<volatile PxI32*>(nbForceThresholds));
	}
}

static PX_FORCE_INLINE void decreaseForceThresholds(const PartitionEdge* edge, PxU32& nbForceThresholds)
{
	if(edge->hasThreshold())
	{
		PX_ASSERT(nbForceThresholds);
		nbForceThresholds--;
	}
}

/////

PxgIncrementalPartition::PxgIncrementalPartition(const PxVirtualAllocator& allocator, PxU32 maxNumPartitions, PxU64 contextID) :
	mNodeCount(0), mNbContactBatches(0), mNbConstraintBatches(0), mNbPartitions(0), mTotalContacts(0), mTotalConstraints(0), 
	mTotalArticulationContacts(0), mTotalArticulationConstraints(0), 
	mMaxSlabCount(0), mNbForceThresholds(0), mPartitionIndexArray(allocator), mPartitionNodeArray(allocator), 
	mSolverConstants(allocator), mNodeInteractionCountArray(allocator),	mDestroyedContactEdgeIndices(allocator), 
	mStartSlabPerPartition(allocator), mArticStartSlabPerPartition(allocator), mNbJointsPerPartition(allocator), 
	mNbArtiJointsPerPartition(allocator), mCSlab(maxNumPartitions), mContextID(contextID)
{
	mPartitionSlabs.pushBack(PX_NEW(PartitionSlab));
}

PxgIncrementalPartition::~PxgIncrementalPartition()
{
	{
		const PxU32 nbBatches = mBatches.size();
		for(PxU32 i=0;i<nbBatches;i++)
		{
			PartitionEdgeBatch* batch = mBatches[i];
			PX_DELETE(batch);
		}
		mBatches.reset();
	}

	const PxU32 nbSlabs = mPartitionSlabs.size();
	for(PxU32 a = 0; a < nbSlabs; ++a)
	{
		PX_DELETE(mPartitionSlabs[a]);
	}
	mPartitionSlabs.clear();
}

void PxgIncrementalPartition::reserveNodes(PxU32 nodeCount)
{
	if(nodeCount > mNodeCount)
	{
		nodeCount = PxMax(nodeCount, 2u * mNodeCount);
		const PxU32 nbSlabs = mPartitionSlabs.size();
		for(PxU32 a = 0; a < nbSlabs; ++a)
		{
			PartitionSlab* slab = mPartitionSlabs[a];

			// PT: this can use up quite a bit of memory:
			// nb slabs * nb objects * (PXG_BATCH_SIZE * sizeof(NodeEntryStorage) + bitmask)
			slab->mNodeBitmap.resize(nodeCount);
			slab->mNodeEntries.resize(nodeCount);
		}

		//mNodeInteractionCountArray.resize(sizeof(PxU32), nodeCount);
		mNodeInteractionCountArray.reserve(nodeCount);
		mNodeInteractionCountArray.forceSize_Unsafe(nodeCount);

		PxMemZero(mNodeInteractionCountArray.begin() + mNodeCount, (nodeCount - mNodeCount)*sizeof(PxU32));
		mNodeCount = nodeCount;
	}
}

void PxgIncrementalPartition::getPreviousAndNextReferencesInSlab(NodeEntryDecoded& prev, NodeEntryDecoded& next, PxU32 index, PxU32 uniqueId, const PartitionSlab* slab, PxU32 slabMask) const
{
	PX_ASSERT(slabMask);

	PxU32 partitionId = mPartitionIndexArray[uniqueId].mPartitionIndex;

#if PX_DEBUG
	{	// PT: checks that the passed data we already had matches the data we previously computed in this function
		const PxU32 slabId = partitionId/PXG_BATCH_SIZE;
		PX_ASSERT(slab == mPartitionSlabs[slabId]);
		PX_ASSERT(slabMask == mPartitionSlabs[slabId]->mNodeBitmap[index]);
	}
#endif

	// PT: I think that AND gets back the id between 0 and 31 that we started from when writing mPartitionIndex in the
	// first place, from "baseId + id". We have to do it this way to reuse that function for both the add & remove cases.
	// In the add case we could directly pass the proper partitionId to the function.
	const PxU32 partitionMask = PXG_BATCH_SIZE - 1;
	partitionId = partitionId & partitionMask;

	// PT: say we are in partition 3.
	// partitionBit = 1 << 3 = 8                  = 00001000
	// maskPrev     = 8 - 1 = 7                   = 00000111
	// maskNext     = ~(16 - 1) = ~15 = ~00001111 = 11110000
	// slabMask marks which partitions are used for this node.
	// Say the mask is 10101010 (pretending we only have 8 partitions).
	// bitMaskPrev = 10101010 & 00000111 = xxxxX010 <= we clear out the bits before the partition X we're in
	// bitMaskNext = 10101010 & 11110000 = 1010Xxxx <= we clear out the bits after the partition X we're in
	// Ignoring the case where the results are 0, we find:
	// idprev = PxHighestSetBit(bitMaskPrev) = 1 (highest because we cleared out the top bits)
	// idnext = PxLowestSetBit(bitMaskNext) = 5 (lowest because we cleared out the bottom bits)
	// Then we fetch the edges corresponding to these indices. They are the previous and next edges
	// that will reference the same node (whose slabMask we used).
	//
	// So the problem if we want to MT this is that at the same time another edge can be processed, that
	// references the same node, fetches the same slabMask, updates the same slabMask, then writes itself
	// to the mEdges array for that node. In one thread the mask was 0 and the edge entry null, while in
	// another thread that mask was updated to 1 and the edge entry written out.

	const PxU32 partitionBit = 1u << partitionId;
	//const PxU32 maskPrev = ((1u<<(partitionId))-1u);
	const PxU32 maskPrev = partitionBit - 1u;
	//const PxU32 maskNext = partitionId == partitionMask ? 0 : ~((1u<<(partitionId+1))-1u);
	const PxU32 maskNext = partitionId == partitionMask ? 0 : ~(partitionBit + partitionBit - 1u);

	PxU32 bitMaskPrev = slabMask & maskPrev;
	PxU32 bitMaskNext = slabMask & maskNext;

	bitMaskPrev = bitMaskPrev == 0 ? slabMask : bitMaskPrev;
	bitMaskNext = bitMaskNext == 0 ? slabMask : bitMaskNext;

	PX_ASSERT(bitMaskPrev != 0);
	PX_ASSERT(bitMaskNext != 0);

	const PxU32 idprev = PxHighestSetBit(bitMaskPrev);
	const PxU32 idnext = PxLowestSetBit(bitMaskNext);
#if STORE_INDICES_IN_NODE_ENTRIES
	#if STORE_EDGE_DATA_IN_NODE_ENTRIES
	const NodeEntryStorage* src = slab->mNodeEntries[index].mEdges;
	prev = src[idprev];
	next = src[idnext];
	#else
	const PxU32* src = slab->mNodeEntries[index].mEdges;
	prev = mEdgeManager.getPartitionEdge(src[idprev]);
	next = mEdgeManager.getPartitionEdge(src[idnext]);
	#endif
#else
	prev = slab->mNodeEntries[index].mEdges[idprev];
	next = slab->mNodeEntries[index].mEdges[idnext];
#endif
}

// PT: this function does the actual edge coloring + maintain a linked list of partition edges in PartitionNodeData
void PxgIncrementalPartition::addEdgeInternal(const PartitionEdge* PX_RESTRICT partitionEdge, PartitionSlab* PX_RESTRICT slab, PxU16 id, PxU16 baseId)
{
	const PxU32 uniqueId = partitionEdge->mUniqueIndex;

	//Insert this edge into this partition!!!!
	mPartitionIndexArray[uniqueId].mPartitionIndex = PxU16(baseId + id);

	slab->mPartitions[id].addToPartition(uniqueId, mPartitionIndexArray[uniqueId]);

	const PxNodeIndex node0 = partitionEdge->mNode0;
	const PxNodeIndex node1 = partitionEdge->mNode1;
	const PxU32 node0Index = node0.index();
	const PxU32 node1Index = node1.index();

	PxU32 slabMask0 = 0;
	if(!partitionEdge->hasInfiniteMass0())
	{
		slabMask0 = slab->mNodeBitmap[node0Index] | (1 << id);
		slab->mNodeBitmap[node0Index] = slabMask0;

#if STORE_INDICES_IN_NODE_ENTRIES
	#if STORE_EDGE_DATA_IN_NODE_ENTRIES
		slab->mNodeEntries[node0Index].mEdges[id].mUniqueIndex = uniqueId;
		slab->mNodeEntries[node0Index].mEdges[id].mNode0Index = node0Index;
	#else
		slab->mNodeEntries[node0Index].mEdges[id] = uniqueId;
	#endif
#else
		slab->mNodeEntries[node0Index].mEdges[id] = partitionEdge;
#endif
	}

	PxU32 slabMask1 = 0;
	if(!partitionEdge->hasInfiniteMass1())
	{
		slabMask1 = slab->mNodeBitmap[node1Index] | (1 << id);
		slab->mNodeBitmap[node1Index] = slabMask1;

#if STORE_INDICES_IN_NODE_ENTRIES
	#if STORE_EDGE_DATA_IN_NODE_ENTRIES
		slab->mNodeEntries[node1Index].mEdges[id].mUniqueIndex = uniqueId;
		slab->mNodeEntries[node1Index].mEdges[id].mNode0Index = node0Index;
	#else
		slab->mNodeEntries[node1Index].mEdges[id] = uniqueId;
	#endif
#else
		slab->mNodeEntries[node1Index].mEdges[id] = partitionEdge;
#endif
	}

	// PT: builds data used in constraintContactBlockPrePrepLaunch / constraint1DBlockPrePrepLaunch

	PartitionNodeData& nodeData = mPartitionNodeArray[uniqueId];

	// PT:
	// The edge was just added to partition P.
	// The edge involves node0 and node1.
	// Each node has a bitmask encoding which partitions the node is involved in.
	// We just updated that bitmask with the bit from partition P. So that bitmask cannot be 0 here.
	//
	// getPreviousAndNextReferencesInSlab() looks for the previous and next bits / partitions
	// involving the same node. Each of these bit was set by / corresponds to another partition edge,
	// that we actually store in mNodeEntries. We have 32 edges in mNodeEntries for each node, so this
	// is effectively a redundant "copy" of the bitmask, except each bit is an explicit partition edge pointer.
	//
	// mPartitionNodeArray stores, for each edge, which nodes are involved in it, and which are the next
	// partition edges involving the same nodes. So this is a linked list of partition edges.

	if(!partitionEdge->hasInfiniteMass0())
	{
		NodeEntryDecoded prevEdge;
		NodeEntryDecoded nextEdge;
		getPreviousAndNextReferencesInSlab(prevEdge, nextEdge, node0Index, uniqueId, slab, slabMask0);

		// PT:
		// Here for example, we retrieved the previous and next edges that contain node0 in this slab.
		// So we know that:
		// - either prevEdge.mNode0Index or prevEdge.mNode1Index will be equal to node0Index
		// - either nextEdge.mNode0Index or nextEdge.mNode1Index will be equal to node0Index
		//
		// We have to setup PartitionNodeData for the current edge. We only have a single LL here so we
		// have to 1) setup the next indices for the current PartitionNodeData, and 2) update the next indices
		// of the previous PartitionNodeData in the LL.
		//
		// 1) We know that the next edge is nextEdge. We are dealing with node0 so we're going to set mNextIndex[0].
		// The only question is whether node0 is the first or second node in nextEdge. We use |1 for the second case.
		//
		// 2) We know that the previous edge is prevEdge. We know that either its node0 or its node1 will
		// be our node (node0). In the first case we must update the previous node's mNextIndex[0] (the LL for
		// the previous node's node0). Otherwise mNextIndex[1]. Either way our own address is uniqueId, and we
		// don't use |1 because node0 is not the second node for us (i.e. for nodeData).
		//
		// Questions remain:
		// - why do we need to build this LL? What is is needed for in the solver?
		// - what happens for the first edge, i.e. there is no prev / next ?
		// - do we really need to fetch the next edge? Can't we just copy the mNextIndex from previous edge?
		//   => I think it's just so that it also works for the first edges indeed

		const PxU32 dstIndex = node0Index == getNode0Index(prevEdge) ? 0 : 1;
		mPartitionNodeArray[getUniqueId(prevEdge)].mNextIndex[dstIndex] = uniqueId << 1;

		const PxU32 data = getUniqueId(nextEdge) << 1;
		nodeData.mNextIndex[0] = getNode0Index(nextEdge) == node0Index ? data : data|1;
	}
	else
	{
		nodeData.mNextIndex[0] = (uniqueId<<1);
	}

	if(!partitionEdge->hasInfiniteMass1())
	{
		NodeEntryDecoded prevEdge;
		NodeEntryDecoded nextEdge;
		getPreviousAndNextReferencesInSlab(prevEdge, nextEdge, node1Index, uniqueId, slab, slabMask1);

		// PT:
		// Same as for node0, the only notable difference is that node1 will now be the second entry
		// for the LL data of previous edge, so we have an extra |1 to add.

		const PxU32 dstIndex = node1Index == getNode0Index(prevEdge) ? 0 : 1;
		mPartitionNodeArray[getUniqueId(prevEdge)].mNextIndex[dstIndex] = (uniqueId << 1)|1;

		const PxU32 data = getUniqueId(nextEdge) << 1;
		nodeData.mNextIndex[1] = getNode0Index(nextEdge) == node1Index ? data : data|1;
	}
	else
	{
		nodeData.mNextIndex[1] = (uniqueId<<1)|1;
	}
	// PT: TODO: try to rebuild this LL in a second pass, all the links at once instead of incrementally?
	// We could also multi-thread that part but because the same node appears twice in different edges we would probably
	// get some false sharing when updating the same PartitionNodeData from 2 different threads at the same time.
}

// PT: this function does the actual edge coloring + maintain a linked list of partition edges in PartitionNodeData
void PxgIncrementalPartition::removeEdgeInternal(PartitionSlab* PX_RESTRICT slab, const PartitionEdge* PX_RESTRICT edge, PxU32 id)
{
	const PxU32 uniqueId = edge->mUniqueIndex;
	slab->mPartitions[id].removeFromPartition(uniqueId, mPartitionIndexArray);
	
	const PxU32 node0Index = edge->mNode0.index();
	const PxU32 node1Index = edge->mNode1.index();

	PxU32 slabMask0 = 0;
	if(!edge->hasInfiniteMass0())
	{
		slabMask0 = slab->mNodeBitmap[node0Index] & (~(1<<id));
		slab->mNodeBitmap[node0Index] = slabMask0;

		resetNodeEntryStorage(slab->mNodeEntries[node0Index].mEdges[id]);	// PT: this is not really needed. We won't read it if the mask is not set (and we don't initialize the data)
	}

	PxU32 slabMask1 = 0;
	if(!edge->hasInfiniteMass1())
	{
		slabMask1 = slab->mNodeBitmap[node1Index] & (~(1<<id));
		slab->mNodeBitmap[node1Index] = slabMask1;

		resetNodeEntryStorage(slab->mNodeEntries[node1Index].mEdges[id]);	// PT: this is not really needed. We won't read it if the mask is not set (and we don't initialize the data)
	}

	if(slabMask0 && !edge->hasInfiniteMass0())
	{
		NodeEntryDecoded prevEdge;
		NodeEntryDecoded nextEdge;
		getPreviousAndNextReferencesInSlab(prevEdge, nextEdge, node0Index, uniqueId, slab, slabMask0);

		const PxU32 prevUniqueId = getUniqueId(prevEdge);
		const PxU32 nextUniqueId = getUniqueId(nextEdge);

		const PxU32 data = nextUniqueId << 1;
		const bool cndt = node0Index == getNode0Index(nextEdge);
		const PxU32 dstIndex = node0Index == getNode0Index(prevEdge) ? 0 : 1;
		mPartitionNodeArray[prevUniqueId].mNextIndex[dstIndex] = cndt ? data : data|1;
	}

	if(slabMask1 && !edge->hasInfiniteMass1())
	{
		NodeEntryDecoded prevEdge;
		NodeEntryDecoded nextEdge;
		getPreviousAndNextReferencesInSlab(prevEdge, nextEdge, node1Index, uniqueId, slab, slabMask1);

		const PxU32 prevUniqueId = getUniqueId(prevEdge);
		const PxU32 nextUniqueId = getUniqueId(nextEdge);

		const PxU32 data = nextUniqueId << 1;
		const bool cndt = node1Index == getNode0Index(nextEdge);
		const PxU32 dstIndex = node1Index == getNode0Index(prevEdge) ? 0 : 1;
		mPartitionNodeArray[prevUniqueId].mNextIndex[dstIndex] = cndt ? data : data|1;
	}
}

bool PxgIncrementalPartition::addJointManager(const PartitionEdge* edge, PxgBodySimManager& bodySimManager)
{
#if USE_FINE_GRAINED_PROFILE_ZONES
	PX_PROFILE_ZONE("PxgIncrementalPartition::addJointManager", mContextID);
#endif
	PX_UNUSED(bodySimManager);

	const PxU32 uniqueId = edge->mUniqueIndex;

#if RIGID_STATIC_EDGE_SOLVER
	if (!edge->isArticulation0() && edge->mNode1.isStaticBody())
		return bodySimManager.addStaticRBJoint(uniqueId, edge->mNode0);
#endif

#if ARTIC_STATIC_EDGES_INTERNAL_SOLVER || ARTIC_SELF_CONSTRAINT_SOLVER
	if (edge->isArticulation0())
	{
#if ARTIC_SELF_CONSTRAINT_SOLVER
		if (edge->mNode0.index() == edge->mNode1.index())
			return bodySimManager.addSelfArticulationJoint(uniqueId, edge->mNode0, edge->mNode1);
#endif
#if ARTIC_STATIC_EDGES_INTERNAL_SOLVER
		if (edge->hasInfiniteMass1())
			return bodySimManager.addStaticArticulationJoint(uniqueId, edge->mNode0);	//Add to articulation 0
#endif
	}
#if ARTIC_STATIC_EDGES_INTERNAL_SOLVER
	else if (edge->isArticulation1() && edge->hasInfiniteMass0())
		return bodySimManager.addStaticArticulationJoint(uniqueId, edge->mNode1);
#endif
#endif
	return false;
}

static PX_FORCE_INLINE PxgIncrementalPartition::SpecialCase isSpecialCase(const PartitionEdge* edge)
{
	PX_UNUSED(edge);
#if RIGID_STATIC_EDGE_SOLVER
	if (!edge->isArticulation0())
	{
		if(edge->hasInfiniteMass1())
			return PxgIncrementalPartition::SPECIAL_CASE_STATIC_RB;
	}
#endif

#if ARTIC_STATIC_EDGES_INTERNAL_SOLVER || ARTIC_SELF_CONSTRAINT_SOLVER
	if (edge->isArticulation0())
	{
	#if ARTIC_SELF_CONSTRAINT_SOLVER
		if (edge->mNode0.index() == edge->mNode1.index())
			return PxgIncrementalPartition::SPECIAL_CASE_ARTI_SELF;
	#endif
	#if ARTIC_STATIC_EDGES_INTERNAL_SOLVER
		if (edge->hasInfiniteMass1())
			return PxgIncrementalPartition::SPECIAL_CASE_STATIC_ARTI0;
	#endif
	}
	#if ARTIC_STATIC_EDGES_INTERNAL_SOLVER
	else if (edge->isArticulation1() && edge->hasInfiniteMass0())
		return PxgIncrementalPartition::SPECIAL_CASE_STATIC_ARTI1;
	#endif
#endif
	return PxgIncrementalPartition::SPECIAL_CASE_NONE;
}

static PX_FORCE_INLINE bool addSpecialCaseContactManager(const PxgIncrementalPartition::Part2WorkItem& item, PxgBodySimManager& manager)
{
	PX_UNUSED(item);
	PX_UNUSED(manager);

#if RIGID_STATIC_EDGE_SOLVER
	if(item.mSpecialCase == PxgIncrementalPartition::SPECIAL_CASE_STATIC_RB)
		return manager.addStaticRBContactManager(item.mPartitionEdge->mUniqueIndex, item.mPartitionEdge->mNode0);
#endif
#if ARTIC_STATIC_EDGES_INTERNAL_SOLVER || ARTIC_SELF_CONSTRAINT_SOLVER
	#if ARTIC_SELF_CONSTRAINT_SOLVER
	if(item.mSpecialCase == PxgIncrementalPartition::SPECIAL_CASE_ARTI_SELF)
		return manager.addSelfArticulationContactManager(item.mPartitionEdge->mUniqueIndex, item.mPartitionEdge->mNode0, item.mPartitionEdge->mNode1);
	#endif
	#if ARTIC_STATIC_EDGES_INTERNAL_SOLVER
	if(item.mSpecialCase == PxgIncrementalPartition::SPECIAL_CASE_STATIC_ARTI0)
		return manager.addStaticArticulationContactManager(item.mPartitionEdge->mUniqueIndex, item.mPartitionEdge->mNode0);

	if(item.mSpecialCase == PxgIncrementalPartition::SPECIAL_CASE_STATIC_ARTI1)
		return manager.addStaticArticulationContactManager(item.mPartitionEdge->mUniqueIndex, item.mPartitionEdge->mNode1);
	#endif
#endif
	return false;
}

bool PxgIncrementalPartition::addContactManager(PartitionEdge* edge, const PxcNpWorkUnit& unit, PxgBodySimManager& manager)
{
#if USE_FINE_GRAINED_PROFILE_ZONES
	PX_PROFILE_ZONE("PxgIncrementalPartition::addContactManager", mContextID);
#endif
	PX_UNUSED(manager);

	edge->setIsContact();	// PT: TODO: this could have been setup in the ctor directly

	increaseForceThresholds(unit, edge, mNbForceThresholds);

#if RIGID_STATIC_EDGE_SOLVER
	if (!edge->isArticulation0())
	{
		if(edge->hasInfiniteMass1())	// PT: covers both statics & kinematics
			return manager.addStaticRBContactManager(edge->mUniqueIndex, edge->mNode0);
	}
#endif

#if ARTIC_STATIC_EDGES_INTERNAL_SOLVER || ARTIC_SELF_CONSTRAINT_SOLVER
	if (edge->isArticulation0())
	{
#if ARTIC_SELF_CONSTRAINT_SOLVER
		if (edge->mNode0.index() == edge->mNode1.index())
			return manager.addSelfArticulationContactManager(edge->mUniqueIndex, edge->mNode0, edge->mNode1);
#endif
#if ARTIC_STATIC_EDGES_INTERNAL_SOLVER
		if (edge->hasInfiniteMass1())
			return manager.addStaticArticulationContactManager(edge->mUniqueIndex, edge->mNode0);	//Add to articulation 0
#endif
	}
#if ARTIC_STATIC_EDGES_INTERNAL_SOLVER
	else if (edge->isArticulation1() && edge->hasInfiniteMass0())
		return manager.addStaticArticulationContactManager(edge->mUniqueIndex, edge->mNode1);
#endif
#endif
	return false;
}

#if PX_PARTITION_COMPACTION
// PT: TODO: consider also passing node indices
static PX_FORCE_INLINE void updateDirtyNodeBitmap(PxBitMap& isDirtyNode, const PartitionEdge* edge, PxU32 hasInfiniteMass0, PxU32 hasInfiniteMass1, bool selfConstraint)
{
	if(!selfConstraint)
	{
		if(!hasInfiniteMass0)
		{
			const PxU32 index = edge->mNode0.index();
			if(!isDirtyNode.test(index))
				isDirtyNode.set(index);
		}

		if(!hasInfiniteMass1)
		{
			const PxU32 index = edge->mNode1.index();
			if(!isDirtyNode.test(index))
				isDirtyNode.set(index);
		}
	}
}
#else
static PX_FORCE_INLINE void updateDirtyNodeBitmap(PxBitMap&, const PartitionEdge*, PxU32, PxU32, bool)	{}
#endif

static PX_FORCE_INLINE bool removeSpecialHandled(PartitionEdge* edge, PxU32 uniqueId, PxgBodySimManager& manager, PxU32 hasInfiniteMass0, PxU32 hasInfiniteMass1)
{
	PX_UNUSED(hasInfiniteMass0);
	PX_UNUSED(hasInfiniteMass1);
	PX_UNUSED(manager);
	PX_UNUSED(uniqueId);

#if ARTIC_STATIC_EDGES_INTERNAL_SOLVER || ARTIC_SELF_CONSTRAINT_SOLVER || RIGID_STATIC_EDGE_SOLVER
	bool selfConstraint = false;
	PX_ASSERT(edge->isSpecialHandled());

	const PxU32 isArticulation0 = edge->isArticulation0();
	const PxU32 isArticulation1 = edge->isArticulation1();

	bool specialHandled = false;
	const PxU32 isContact = edge->isContact();

#if RIGID_STATIC_EDGE_SOLVER
	if (!isArticulation0 && !isArticulation1)
	{
		if(hasInfiniteMass1)
		{
			if (isContact)
				specialHandled = manager.removeStaticRBContactManager(uniqueId, edge->mNode0);
			else
				specialHandled = manager.removeStaticRBJoint(uniqueId, edge->mNode0);
		}
	}
	else
#endif
#if ARTIC_STATIC_EDGES_INTERNAL_SOLVER || ARTIC_SELF_CONSTRAINT_SOLVER
#if ARTIC_SELF_CONSTRAINT_SOLVER
	if (edge->mNode0.index() == edge->mNode1.index())
	{
		selfConstraint = true;
		if (!isContact)
			specialHandled = manager.removeSelfArticulationJoint(uniqueId, edge->mNode0, edge->mNode1);
		else
			specialHandled = manager.removeSelfArticulationContactManager(uniqueId, edge->mNode0, edge->mNode1);
	}
	else
#endif
#if ARTIC_STATIC_EDGES_INTERNAL_SOLVER
	if (isArticulation0 && hasInfiniteMass1)
	{
		if (!isContact)
			specialHandled = manager.removeStaticArticulationJoint(uniqueId, edge->mNode0);
		else
			specialHandled = manager.removeStaticArticulationContactManager(uniqueId, edge->mNode0);
	}
	else if (isArticulation1 && hasInfiniteMass0)
	{
		if (!isContact)
			specialHandled = manager.removeStaticArticulationJoint(uniqueId, edge->mNode1);
		else
			specialHandled = manager.removeStaticArticulationContactManager(uniqueId, edge->mNode1);
	}
#endif
#endif
	PX_ASSERT(specialHandled);
	PX_UNUSED(specialHandled);
#endif
	return selfConstraint;
}

void PxgIncrementalPartition::removeEdge(PartitionEdge* edge, IG::GPUExternalData& islandSimGpuData, PxgBodySimManager& manager)
{
#if USE_FINE_GRAINED_PROFILE_ZONES
	PX_PROFILE_ZONE("PxgIncrementalPartition::removeEdge", mContextID);
#endif
	PX_UNUSED(manager);

	decreaseForceThresholds(edge, mNbForceThresholds);

	// PT: TODO: could we encode the proper bucket in the edge when we add it into the system, and skip all the tests?
	const PxU32 hasInfiniteMass0 = edge->hasInfiniteMass0();
	const PxU32 hasInfiniteMass1 = edge->hasInfiniteMass1();

	const PxU32 uniqueId = edge->mUniqueIndex;

	// PT: in this new design we encode in the partition edge itself whether it's "special handled" or not. We recorded the real state rather than the
	// theoretical state: an edge can be a special case in theory, but a regular case in practice, when the PxgBodySimManager/PxgJointManager are full.
	// The benefits are the following:
	// - there cannot be a mismatch anymore between the add & remove parts. Potentially the "add" part could have added the edge to the external buffers
	//   (i.e. it's "special handled") but the "remove" part could have failed, making the code fallback to the regular case. This would break the edge
	//   coloring since we would remove an edge that would not have taken the regular codepath before. This mismatch is not possible anymore.
	// - the code immediately knows whether an edge is special or not, without doing all the tests to determine in which bucket it falls. That's faster.
	// - we know the state ahead of time, so it makes multithreading easier.

	bool selfConstraint = false;
	if(edge->isSpecialHandled())
	{
		selfConstraint = removeSpecialHandled(edge, uniqueId, manager, hasInfiniteMass0, hasInfiniteMass1);
	}
	else
	{
		PxU32 id = mPartitionIndexArray[uniqueId].mPartitionIndex;
		const PxU32 slabId = id / PXG_BATCH_SIZE;

		PartitionSlab* slab = mPartitionSlabs[slabId];

		PxU32 baseId = PXG_BATCH_SIZE*slabId;

		id -= baseId;

		PX_ASSERT(slab);

		removeEdgeInternal(slab, edge, id);
	}

	{
		const IG::EdgeIndex edgeIndex = edge->getEdgeIndex();
		const PartitionEdge* pEdge = islandSimGpuData.getFirstPartitionEdge(edgeIndex);
		if (pEdge == edge)
			islandSimGpuData.setFirstPartitionEdge(edgeIndex, edge->mNextPatch);
	}

	updateDirtyNodeBitmap(mIsDirtyNode, edge, hasInfiniteMass0, hasInfiniteMass1, selfConstraint);

	// PT: no need to reset the contact bit, it will be cleared on recycling
	mEdgeManager.putEdge(edge);
}

static PX_FORCE_INLINE PxIntBool isKinematic(const IG::IslandSim& islandSim, PxNodeIndex nodeIndex)
{
	PxIntBool infinite = true;
	if(nodeIndex.isValid())
	{
		// PT: TODO: pretty bad here to access one cache line just to read one bit
		const IG::Node& node = islandSim.getNode(nodeIndex);
		infinite = node.isKinematic();
	}
	return infinite;
}

// PT: this function does multiple things:
// 1) allocate a new PartitionEdge from the edge manager
// 2) resize/allocate internal edge data buffers (mPartitionIndexArray, mNpIndexArray, mPartitionNodeArray, mSolverConstants)
// 3) initialize some of the data for the newly allocated PartitionEdge
// 4) initialize some of the data for the newly allocated PartitionIndexData entry in mPartitionIndexArray
// 5) initialize some of the data for the newly allocated PartitionNodeData entry in mPartitionNodeArray
// 6) initialize the newly allocated entry in mNpIndexArray
// 7) initialize the newly allocated entry in mSolverConstants
PartitionEdge* PxgIncrementalPartition::addEdge_Stage1(const IG::IslandSim& islandSim, IG::EdgeIndex edgeIndex, PxU32 patchIndex, PxU32 npIndex, PxNodeIndex node1, PxNodeIndex node2)
{
#if USE_FINE_GRAINED_PROFILE_ZONES
	PX_PROFILE_ZONE("PxgIncrementalPartition::addEdge_Stage1", mContextID);
#endif

	///////////////////////////////////////////////////////////////////////////

	// PT: 1) allocate a new PartitionEdge from the edge manager
	PartitionEdge* partitionEdge = mEdgeManager.getEdge(edgeIndex);

	///////////////////////////////////////////////////////////////////////////

	// PT: 2) resize/allocate internal edge data buffers (mPartitionIndexArray, mNpIndexArray, mPartitionNodeArray, mSolverConstants)
	const PxU32 count = mEdgeManager.getEdgeCount();
	if (count >= mPartitionIndexArray.capacity())
	{
		PX_PROFILE_ZONE("ResizeEdgeBuffer", mContextID);

		const PxU32 newSize = PxMax(count, mPartitionIndexArray.capacity() * 2);
		mPartitionIndexArray.reserve(newSize);
		mNpIndexArray.reserve(newSize);
		mPartitionNodeArray.reserve(newSize);
	}

	if (count >= mPartitionIndexArray.size())
	{
		mPartitionIndexArray.resizeUninitialized(count);
		mNpIndexArray.resizeUninitialized(count);
		mPartitionNodeArray.resizeUninitialized(count);
	}

	if(count >= mSolverConstants.capacity())
	{
		const PxU32 newSize = PxMax(count, mSolverConstants.capacity() * 2);
		mSolverConstants.resize(newSize);
		//mSolverConstants.resizeUninitialized(newSize);	// PT: TODO: one of the dup code was using resize, the other one resizeUninitialized. Investigate if it makes a difference.
	}

	///////////////////////////////////////////////////////////////////////////

	// PT: 3) initialize some of the data for the newly allocated PartitionEdge

	partitionEdge->mNode0 = node1;
	partitionEdge->mNode1 = node2;

	if(isKinematic(islandSim, node1))
		partitionEdge->setInfiniteMass0();

	if(isKinematic(islandSim, node2))
		partitionEdge->setInfiniteMass1();

	///////////////////////////////////////////////////////////////////////////

	// PT: 4) initialize some of the data for the newly allocated PartitionIndexData entry in mPartitionIndexArray

	const PxU32 uniqueId = partitionEdge->mUniqueIndex;

	PartitionIndexData& indexData = mPartitionIndexArray[uniqueId];
	indexData.mPatchIndex = PxTo8(patchIndex);

	const PxU8 articulationOffset = node1.isArticulation() || node2.isArticulation() ? 2 : 0;
	const PxU32 implicitEdgeType = npIndex == 0xffffffff ? IG::Edge::EdgeType::eCONSTRAINT : IG::Edge::EdgeType::eCONTACT_MANAGER;
	indexData.mCType = PxU8(implicitEdgeType) + articulationOffset;

	///////////////////////////////////////////////////////////////////////////

	// PT: 5) initialize some of the data for the newly allocated PartitionNodeData entry in mPartitionNodeArray

	PartitionNodeData& nodeData = mPartitionNodeArray[uniqueId];
	nodeData.mNodeIndex0 = node1;
	nodeData.mNodeIndex1 = node2;

	///////////////////////////////////////////////////////////////////////////

	// PT: 6) initialize the newly allocated entry in mNpIndexArray

	mNpIndexArray[uniqueId] = npIndex;

	///////////////////////////////////////////////////////////////////////////

	// PT: 7) initialize the newly allocated entry in mSolverConstants

	mSolverConstants[uniqueId].mEdgeIndex = edgeIndex;	// PT: as far as I can tell mConstraintWriteBackIndex remains uninitialized / unused for contact managers!

	return partitionEdge;
}

static PX_FORCE_INLINE void updatePartitionEdgeLinkedListHead(IG::GPUExternalData& islandSimGpuData, IG::EdgeIndex edgeIndex, PartitionEdge* partitionEdge)
{
	partitionEdge->mNextPatch = islandSimGpuData.getFirstPartitionEdge(edgeIndex);
	islandSimGpuData.setFirstPartitionEdge(edgeIndex, partitionEdge);
}

// PT: this function does multiple things:
// 1) the actual edge coloring for the new edge, i.e. figuring out in which partition it goes. This touches a bunch of new internal buffers.
// 2) initialize the rest of the data for the newly allocated PartitionNodeData entry in mPartitionNodeArray. This is a linked-list of edges
// related to the edge coloring bits.
// 2b) initialize the rest of the data for the newly allocated PartitionIndexData entry in mPartitionIndexArray.
// 3) initialize another linked-list for patches, the one first seen in the island manager where the data is actually stored.
void PxgIncrementalPartition::addEdge_Stage2(IG::GPUExternalData& islandSimGpuData, IG::EdgeIndex edgeIndex, PartitionEdge* partitionEdge, bool specialHandled, bool doPart1, bool doPart2)
{
#if USE_FINE_GRAINED_PROFILE_ZONES
	PX_PROFILE_ZONE("PxgIncrementalPartition::addEdge_Stage2", mContextID);
#endif

	if(doPart1)
	{

	if (!specialHandled)
	{
		//Now place this edge in an appropriate slab!!!!

		const PxNodeIndex node1 = partitionEdge->mNode0;
		const PxNodeIndex node2 = partitionEdge->mNode1;

		const PxU32 hasInfiniteMass0 = partitionEdge->hasInfiniteMass0();
		const PxU32 hasInfiniteMass1 = partitionEdge->hasInfiniteMass1();

		const PxU32 index1 = node1.index();
		const PxU32 index2 = node2.index();

		bool success = false;
		const PxU32 nbSlabs = mPartitionSlabs.size();
		for (PxU32 a = 0; a < nbSlabs; ++a)
		{
			PartitionSlab* slab = mPartitionSlabs[a];
			PxU32 map = 0; // This map encodes in which of the 32 partitions of this slab either of the rigids/articulations is already present

			if (!hasInfiniteMass0)
				map = slab->mNodeBitmap[index1];

			if (!hasInfiniteMass1)
				map |= slab->mNodeBitmap[index2];

			if (map != 0xFFFFFFFF)
			{
				const PxU32 baseId = a*PXG_BATCH_SIZE;
				const PxU32 id = PxLowestSetBit(~map);
				success = true;
				addEdgeInternal(partitionEdge, slab, PxTo16(id), PxTo16(baseId));

				if ((id + baseId) == mMaxSlabCount)
					mMaxSlabCount = id + baseId + 1;
				break;
			}
		}

		if (!success)
		{
			PX_PROFILE_ZONE("NewPartitionSlab", mContextID);

			PartitionSlab* slab = PX_NEW(PartitionSlab);
			slab->mNodeBitmap.resize(mNodeCount);
			slab->mNodeEntries.resize(mNodeCount);
			mPartitionSlabs.pushBack(slab);
			const PxU32 baseId = PXG_BATCH_SIZE * (mPartitionSlabs.size() - 1);
			addEdgeInternal(partitionEdge, slab, 0, PxTo16(baseId));

			if (baseId == mMaxSlabCount)
				mMaxSlabCount = baseId + 1;
		}
	}
	else
		partitionEdge->setSpecialHandled();

	}

	if(doPart2)
		updatePartitionEdgeLinkedListHead(islandSimGpuData, edgeIndex, partitionEdge);
}

//static bool containedInDestroyedEdges(PxsContactManager* /*manager*/, PartitionEdge** /*destroyedEdges*/, const PxU32 /*destroyedEdgeCount*/)
//{
//	return false;
//}

static PX_FORCE_INLINE bool isLostPatch(const PxsContactManagerOutputCounts& output)
{
	if(output.prevPatches >= output.nbPatches)
		return true;

	if(!output.nbPatches && output.prevPatches)
		return true;

	return false;
}

static PX_FORCE_INLINE bool isFoundPatch(const PxsContactManagerOutputCounts& output)
{
	return output.prevPatches < output.nbPatches;
}

namespace
{
	class ProcessPatchesTask : public Cm::Task
	{
		PX_NOCOPY(ProcessPatchesTask)
	public:
		PxgIncrementalPartition&				mIncrementalPartition;
		IG::IslandSim&							mIslandSim;
		PxsContactManager**						mLostFoundPatchManagers;
		PxU32									mNbLostFoundPatchManagers;
		const PxsContactManagerOutputCounts*	mLostFoundPairOutputs;
		PxgBodySimManager&						mBodySimManager;
		PxgJointManager&						mJointManager;

		ProcessPatchesTask(	PxU64 contextID, PxgIncrementalPartition& partition, IG::IslandSim& islandSim,
							PxsContactManager** lostFoundPatchManagers, PxU32 nbLostFoundPatchManagers, const PxsContactManagerOutputCounts* lostFoundPairOutputs,
							PxgBodySimManager& bodySimManager, PxgJointManager& jointManager) :
			Cm::Task					(contextID),
			mIncrementalPartition		(partition),
			mIslandSim					(islandSim),
			mLostFoundPatchManagers		(lostFoundPatchManagers),
			mNbLostFoundPatchManagers	(nbLostFoundPatchManagers),
			mLostFoundPairOutputs		(lostFoundPairOutputs),
			mBodySimManager				(bodySimManager),
			mJointManager				(jointManager)
			{}

		virtual const char* getName() const	PX_OVERRIDE	PX_FINAL
		{
			return "PxgIncrementalPartitioning_ProcessPatchesTask";
		}

		virtual void runInternal()	PX_OVERRIDE	PX_FINAL
		{
			mIncrementalPartition.processLostPatches_Reference(mIslandSim, mBodySimManager, mJointManager, mLostFoundPatchManagers, mNbLostFoundPatchManagers, mLostFoundPairOutputs);
			mIncrementalPartition.processFoundPatches_Reference(mIslandSim, mBodySimManager, mLostFoundPatchManagers, mNbLostFoundPatchManagers, mLostFoundPairOutputs);
		}
	};
}

void PxgIncrementalPartition::processLostFoundPatches(	Cm::FlushPool& flushPool, PxBaseTask* continuation,
														IG::IslandSim& islandSim, PxgBodySimManager& bodySimManager, PxgJointManager& jointManager,
														PxsContactManager** lostFoundPatchManagers, PxU32 nbLostFoundPatchManagers, const PxsContactManagerOutputCounts* lostFoundPairOutputs)
{
#if USE_SPLIT_SECOND_PASS_ISLAND_GEN
	// PT: this copy is necessary when running postIslandGen in parallel with this function. Specifically
	// Sc::Scene::setEdgesConnected will call mSimpleIslandManager->setEdgeConnected and mSimpleIslandManager->secondPassIslandGenPart1
	// while this is running, and these functions will modify the active contact manager bitmap.
	{
		PX_PROFILE_ZONE("PxgIncrementalPartition::copyData", mContextID);

		PX_ASSERT(islandSim.mGpuData);
		IG::GPUExternalData& islandSimGpuData = *islandSim.mGpuData;

		// PT: TODO: use the scratch allocator instead?
		mActiveCMBitmapCopy.copy(islandSimGpuData.getActiveContactManagerBitmap());
	}
#endif

	if(!gRunDefaultVersion)
	{
		processLostPatchesMT(	islandSim, flushPool, continuation,
								lostFoundPatchManagers, nbLostFoundPatchManagers, lostFoundPairOutputs,
								bodySimManager, jointManager);
	}
	else
	{
		ProcessPatchesTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ProcessPatchesTask)), ProcessPatchesTask)(mContextID, *this, islandSim, lostFoundPatchManagers, nbLostFoundPatchManagers, lostFoundPairOutputs, bodySimManager, jointManager);
		startTask(task, continuation);
	}
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	class PreprocessTask;

	struct SharedContext
	{
		SharedContext(
			PxgIncrementalPartition& ip,
				IG::IslandSim& islandSim, Cm::FlushPool& flushPool, PxBaseTask* continuation, PxgBodySimManager& bodySimManager, PxgJointManager& jointManager,
				const PxsContactManagerOutputCounts* lostFoundPairOutputs, PxsContactManager** lostFoundPatchManagers, PxU32 nbLostFoundPatchManagers) :
			mIP(ip),
			mIslandSim(islandSim), mFlushPool(flushPool), mContinuation(continuation),
			mBodySimManager(bodySimManager), mJointManager(jointManager),
			mLostFoundPairOutputs(lostFoundPairOutputs), mLostFoundPatchManagers(lostFoundPatchManagers), mNbLostFoundPatchManagers(nbLostFoundPatchManagers), mTaskHead(NULL)
		{
		}

		PxgIncrementalPartition&				mIP;
		IG::IslandSim&							mIslandSim;
		Cm::FlushPool&							mFlushPool;
		PxBaseTask*								mContinuation;
		PxgBodySimManager&						mBodySimManager;
		PxgJointManager&						mJointManager;
		const PxsContactManagerOutputCounts*	mLostFoundPairOutputs;
		PxsContactManager**						mLostFoundPatchManagers;
		const PxU32								mNbLostFoundPatchManagers;
		PreprocessTask*							mTaskHead;
	};

	class PreprocessTask : public Cm::Task
	{
		PX_NOCOPY(PreprocessTask)
	public:
		SharedContext&									mContext;
		const PxU32										mStartIndex;
		const PxU32										mNbToProcess;
		PxgIncrementalPartition::PartitionEdgeBatch*	mBatch;
		PreprocessTask*									mNext;

		PreprocessTask(PxU64 contextID, SharedContext& context, PxU32 startIndex, PxU32 nbToProcess) :
			Cm::Task(contextID), mContext(context), mStartIndex(startIndex), mNbToProcess(nbToProcess), mBatch(NULL), mNext(NULL)
			{}

		virtual const char* getName() const	PX_OVERRIDE	PX_FINAL
		{
			return "PxgIncrementalPartitioning_PreprocessTask";
		}

		virtual void runInternal()	PX_OVERRIDE	PX_FINAL
		{
			PX_ASSERT(mContext.mIslandSim.mGpuData);
			IG::GPUExternalData& islandSimGpuData = *mContext.mIslandSim.mGpuData;

			const PxsContactManagerOutputCounts* PX_RESTRICT lostFoundPairOutputs = mContext.mLostFoundPairOutputs;
			const PxsContactManager*const* PX_RESTRICT lostFoundPatchManagers = mContext.mLostFoundPatchManagers;

			PxInt32ArrayPinned& PX_RESTRICT nodeInteractionCountArray = mContext.mIP.mNodeInteractionCountArray;
			PxU32* PX_RESTRICT dirtyMap = mContext.mIP.mIsDirtyNode.getWords();
			const PxPinnedArray<PartitionIndexData>& PX_RESTRICT partitionIndexArray = mContext.mIP.mPartitionIndexArray;
			PxArray<PartitionSlab*>& PX_RESTRICT partitionSlabs = mContext.mIP.mPartitionSlabs;

			const PxU32 startIndex = mStartIndex;
			const PxU32 last = startIndex + mNbToProcess;

			PxI32 localNbForceThresholds = 0;

			// Process lost patches. These are CMs that reduced the number of contact patches they had.
			for(PxU32 a=startIndex; a<last; a++)
			{
				// PT: skip found patches
				const PxsContactManagerOutputCounts& output = lostFoundPairOutputs[a];
			
				if(!isLostPatch(output))
					continue;

				const PxsContactManager* contactManager = lostFoundPatchManagers[a];
				const PxcNpWorkUnit& unit = contactManager->getWorkUnit();

				if(unit.mFlags & PxcNpWorkUnitFlag::eDISABLE_RESPONSE)
					continue;

				PartitionEdge* partitionEdge = islandSimGpuData.getFirstPartitionEdge(unit.mEdgeIndex);

				//KS - if this is NULL, it means this unit was also destroyed and will be included in the destroyedEdgeCount (i.e. NP detected a lost touch at the same time as BP detected a lost pair
				//PX_ASSERT(partitionEdge != NULL || containedInDestroyedEdges(manager, destroyedEdges, destroyedEdgeCount));
				if(!partitionEdge)
					continue;

				if(!output.nbPatches && output.prevPatches)
				{
					decreaseNodeInteractionCountsMT(nodeInteractionCountArray, partitionEdge->mNode0, partitionEdge->mNode1);

#if RECORD_DESTROYED_EDGES_IN_FOUND_LOST_PASSES
					#error TODO	// PT: that one would need adapting if needed
					//mDestroyedContactEdgeIndices.pushBack(edgeIndex);
#endif
				}

				for(PxU32 b = output.nbPatches; b < output.prevPatches; ++b)
				{
					if(!partitionEdge)
						break;

					// PT: fom removeEdge()
					{
						if(partitionEdge->hasThreshold())
							localNbForceThresholds++;	// PT: accumulate in local variable for now

						const PxU32 hasInfiniteMass0 = partitionEdge->hasInfiniteMass0();
						const PxU32 hasInfiniteMass1 = partitionEdge->hasInfiniteMass1();
						const PxU32 node0Index = partitionEdge->mNode0.index();
						const PxU32 node1Index = partitionEdge->mNode1.index();

#if PX_PARTITION_COMPACTION
						{
							// PT: MT version of updateDirtyNodeBitmap
							const bool selfConstraint = node0Index == node1Index;
							if(!selfConstraint)
							{
								// PT: in this version we don't bother doing the "test" before the "set"
								if(!hasInfiniteMass0)
									PxAtomicOr(reinterpret_cast<volatile PxI32*>(&dirtyMap[node0Index >> 5]), 1 << (node0Index & 31));

								if(!hasInfiniteMass1)
									PxAtomicOr(reinterpret_cast<volatile PxI32*>(&dirtyMap[node1Index >> 5]), 1 << (node1Index & 31));
							}
						}
#endif

						if(partitionEdge->isSpecialHandled())
						{
							// PT: we cannot remove edges from external managers directly, it's not thread safe. We also
							// would like to avoid re-parsing the whole input array another time so we'd like to batch
							// special edges for later processing.
							//
							// How do we batch this? We want a system that avoids reallocations and preserve determinism.
							// Each pair can have N edges because of the patch linked list so we cannot use a static buffer
							// with a known max number of entries in each task. Naively we could just have a PxArray per task,
							// push to it, and then the next process would loop over the tasks in order and process their array
							// in order. Not perfect but still better than redoing the full costly loop from scratch. We
							// don't want a real PxArray per task of course so we could have a pool of PxArrays (or, gasp,
							// a PxArray of PxArrays) and we pick one up and assign it to each task we start. Not great but
							// could work.

							PX_ASSERT(mBatch);
							mBatch->mEdges.pushBack(partitionEdge);
						}
						else
						{
							const PxU32 uniqueId = partitionEdge->mUniqueIndex;

							PxU32 id = partitionIndexArray[uniqueId].mPartitionIndex;

							const PxU32 slabId = id / PXG_BATCH_SIZE;

							PartitionSlab* slab = partitionSlabs[slabId];

							PxU32 baseId = PXG_BATCH_SIZE*slabId;

							id -= baseId;

							PX_ASSERT(slab);

							// PT: from removeEdgeInternal(slab, edge, id);
							{
								// PT: we cannot removeFromPartition here. We will do that later (REMOVE_EDGES_FROM_PARTITIONS).
								// We can however update the edge coloring:

								if(!hasInfiniteMass0)
									PxAtomicAnd(reinterpret_cast<volatile PxI32*>(&slab->mNodeBitmap[node0Index]), (~(1<<id)));

								if(!hasInfiniteMass1)
									PxAtomicAnd(reinterpret_cast<volatile PxI32*>(&slab->mNodeBitmap[node1Index]), (~(1<<id)));

								// PT: the resetNodeEntryStorage() must be avoided in this version, as the edge LL is updated in a second pass (UPDATE_EDGE_LL).

								// PT: TODO: we could do per-partition batches here
							}
						}
						// PT: we cannot do setFirstPartitionEdge calls here if we want to be able to parse the input data again.
						// PT: we cannot easily do "putEdge" either without breaking determinism.
					}

					PartitionEdge* nextPartitionEdge = partitionEdge->mNextPatch;
					partitionEdge = nextPartitionEdge;
				}
			}

			// PT: one atomic add to update the data we accumulated locally. Note the minus sign, as we don't have PxAtomicSub.
			if(localNbForceThresholds)
				PxAtomicAdd(reinterpret_cast<volatile PxI32*>(&mContext.mIP.mNbForceThresholds), -localNbForceThresholds);
		}
	};

	class RemoveBatchedSpecialEdgesTask : public Cm::Task
	{
		PX_NOCOPY(RemoveBatchedSpecialEdgesTask)
		SharedContext&	mContext;

	public:
		RemoveBatchedSpecialEdgesTask(PxU64 contextID, SharedContext& context) :
			Cm::Task(contextID), mContext(context)
			{}

		virtual const char* getName() const	PX_OVERRIDE	PX_FINAL
		{
			return "RemoveBatchedSpecialEdgesTask";
		}

		virtual void runInternal()	PX_OVERRIDE	PX_FINAL
		{
			PxgBodySimManager& bodySimManager = mContext.mBodySimManager;

			// PT: go over batched data sequentially in a single task, but at least in a separate thread. Better than nothing.

			PreprocessTask* currentTask = mContext.mTaskHead;
			while(currentTask)
			{
				PreprocessTask* nextTask = currentTask->mNext;

				PxgIncrementalPartition::PartitionEdgeBatch* currentBatch = currentTask->mBatch;
				const PxU32 size = currentBatch->mEdges.size();
				for(PxU32 i=0;i<size;i++)
				{
					PartitionEdge* partitionEdge = currentBatch->mEdges[i];

					const PxU32 hasInfiniteMass0 = partitionEdge->hasInfiniteMass0();
					const PxU32 hasInfiniteMass1 = partitionEdge->hasInfiniteMass1();

					const PxU32 uniqueId = partitionEdge->mUniqueIndex;

					// PT: part of removeEdge() that we skipped in PreprocessTask
					removeSpecialHandled(partitionEdge, uniqueId, bodySimManager, hasInfiniteMass0, hasInfiniteMass1);
				}

				currentTask = nextTask;
			}
		}
	};

	enum Codepath
	{
		DESTROY_EDGES_PROCESS_FOUND_PATCHES,
		REMOVE_EDGES_FROM_PARTITIONS,
		UPDATE_EDGE_LL,
	};

	static const char* gTaskNames[] = 
	{
		"PxgPartitioning_DestroyEdgesAndProcessFoundPatches",
		"PxgPartitioning_RemoveEdgesFromPartitions",
		"PxgPartitioning_UpdateEgeLL",
		"PxgPartitioning_RemoveSpecialEdges",
	};

	class ControlTask : public Cm::Task
	{
		PX_NOCOPY(ControlTask)
		SharedContext&	mContext;
		const Codepath	mCodepath;
		const PxU32		mStartIndex;
		const PxU32		mNbToProcess;

	public:
		ControlTask(PxU64 contextID, SharedContext& context, Codepath codepath, PxU32 startIndex=0, PxU32 nbToProcess=0xffffffff) :
			Cm::Task(contextID), mContext(context), mCodepath(codepath), mStartIndex(startIndex), mNbToProcess(nbToProcess)
			{}

		virtual const char* getName() const	PX_OVERRIDE	PX_FINAL
		{
			return gTaskNames[mCodepath];
		}

		virtual void runInternal()	PX_OVERRIDE	PX_FINAL
		{
			parse();

			if(mCodepath==DESTROY_EDGES_PROCESS_FOUND_PATCHES)
			{
				PX_ASSERT(mContext.mIslandSim.mGpuData);
				IG::GPUExternalData& islandSimGpuData = *mContext.mIslandSim.mGpuData;

				// PT: last part of processLostPatches_Reference, not multithreaded yet
				mContext.mIP.destroyEdges(mContext.mIslandSim.mCpuData, islandSimGpuData, mContext.mBodySimManager, mContext.mJointManager, true, false);

				mContext.mIP.processFoundPatches_Reference(mContext.mIslandSim, mContext.mBodySimManager,
					mContext.mLostFoundPatchManagers, mContext.mNbLostFoundPatchManagers, mContext.mLostFoundPairOutputs);
			}
		}

		void parse()
		{
			PX_ASSERT(mContext.mIslandSim.mGpuData);
			IG::GPUExternalData& islandSimGpuData = *mContext.mIslandSim.mGpuData;

			const PxsContactManagerOutputCounts* PX_RESTRICT lostFoundPairOutputs = mContext.mLostFoundPairOutputs;
			const PxsContactManager*const* PX_RESTRICT lostFoundPatchManagers = mContext.mLostFoundPatchManagers;

			PxPinnedArray<PartitionIndexData>& PX_RESTRICT partitionIndexArray = mContext.mIP.mPartitionIndexArray;
			PxPinnedArray<PartitionNodeData>& PX_RESTRICT partitionNodeArray = mContext.mIP.mPartitionNodeArray;
			PxArray<PartitionSlab*>& PX_RESTRICT partitionSlabs = mContext.mIP.mPartitionSlabs;

			// PT: TODO: unfortunately for now we do parse the full input array again. Would be great to avoid that.

			const PxU32 startIndex = mStartIndex;
			const PxU32 last = mNbToProcess == 0xffffffff ? mContext.mNbLostFoundPatchManagers : mStartIndex + mNbToProcess;

			for(PxU32 a=startIndex; a<last; a++)
			{
				const PxsContactManagerOutputCounts& output = lostFoundPairOutputs[a];
			
				if(!isLostPatch(output))
					continue;

				const PxsContactManager* contactManager = lostFoundPatchManagers[a];
				const PxcNpWorkUnit& unit = contactManager->getWorkUnit();

				if(unit.mFlags & PxcNpWorkUnitFlag::eDISABLE_RESPONSE)
					continue;

				PartitionEdge* partitionEdge = islandSimGpuData.getFirstPartitionEdge(unit.mEdgeIndex);
				if(!partitionEdge)
					continue;

				for(PxU32 b = output.nbPatches; b < output.prevPatches; ++b)
				{
					if(!partitionEdge)
						break;

					const PxU32 hasInfiniteMass0 = partitionEdge->hasInfiniteMass0();
					const PxU32 hasInfiniteMass1 = partitionEdge->hasInfiniteMass1();
					const PxU32 node0Index = partitionEdge->mNode0.index();
					const PxU32 node1Index = partitionEdge->mNode1.index();

					const PxU32 uniqueId = partitionEdge->mUniqueIndex;

					if(partitionEdge->isSpecialHandled())
					{
					}
					else if(mCodepath == REMOVE_EDGES_FROM_PARTITIONS)
					{
						// PT: TODO: the IDs here are now computed twice. Maybe we could do better.

						PxU32 id = partitionIndexArray[uniqueId].mPartitionIndex;

						const PxU32 slabId = id / PXG_BATCH_SIZE;

						PartitionSlab* slab = partitionSlabs[slabId];

						PxU32 baseId = PXG_BATCH_SIZE*slabId;

						id -= baseId;

						PX_ASSERT(slab);

						// PT: part of removeEdgeInternal() we skipped in PreprocessTask.
						slab->mPartitions[id].removeFromPartition(uniqueId, partitionIndexArray);
					}
					else if(mCodepath == UPDATE_EDGE_LL)
					{
						const PxU32 id = partitionIndexArray[uniqueId].mPartitionIndex;

						const PxU32 slabId = id / PXG_BATCH_SIZE;

						PartitionSlab* slab = partitionSlabs[slabId];

						// PT: part of removeEdgeInternal() we skipped in PreprocessTask.
						{
							// PT: at this point mNodeBitmap has already been fully updated but the mNodeEntries haven't,
							// so we can still use them to find the prev & next edges.
							if(!hasInfiniteMass0)
							{
								const PxU32 slabMask0 = slab->mNodeBitmap[node0Index];

								// PT: TODO: refactor

								if(slabMask0)
								{
									NodeEntryDecoded prevEdge;
									NodeEntryDecoded nextEdge;
									mContext.mIP.getPreviousAndNextReferencesInSlab(prevEdge, nextEdge, node0Index, uniqueId, slab, slabMask0);

									const PxU32 prevUniqueId = getUniqueId(prevEdge);
									const PxU32 nextUniqueId = getUniqueId(nextEdge);

									const PxU32 data = nextUniqueId << 1;
									const bool cndt = node0Index == getNode0Index(nextEdge);
									const PxU32 dstIndex = node0Index == getNode0Index(prevEdge) ? 0 : 1;
									partitionNodeArray[prevUniqueId].mNextIndex[dstIndex] = cndt ? data : data|1;
								}
							}

							if(!hasInfiniteMass1)
							{
								const PxU32 slabMask1 = slab->mNodeBitmap[node1Index];

								if(slabMask1)
								{
									NodeEntryDecoded prevEdge;
									NodeEntryDecoded nextEdge;
									mContext.mIP.getPreviousAndNextReferencesInSlab(prevEdge, nextEdge, node1Index, uniqueId, slab, slabMask1);

									const PxU32 prevUniqueId = getUniqueId(prevEdge);
									const PxU32 nextUniqueId = getUniqueId(nextEdge);

									const PxU32 data = nextUniqueId << 1;
									const bool cndt = node1Index == getNode0Index(nextEdge);
									const PxU32 dstIndex = node1Index == getNode0Index(prevEdge) ? 0 : 1;
									partitionNodeArray[prevUniqueId].mNextIndex[dstIndex] = cndt ? data : data|1;
								}
							}
						}
					}

					PartitionEdge* nextPartitionEdge = partitionEdge->mNextPatch;

					if(mCodepath==DESTROY_EDGES_PROCESS_FOUND_PATCHES)
					{
						// PT: part of removeEdge() we couldn't run before
						{
							const IG::EdgeIndex edgeIndex = partitionEdge->getEdgeIndex();
							PartitionEdge* pEdge = islandSimGpuData.getFirstPartitionEdge(edgeIndex);
							if (pEdge == partitionEdge)
								islandSimGpuData.setFirstPartitionEdge(edgeIndex, nextPartitionEdge);
						}
						mContext.mIP.mEdgeManager.putEdge(partitionEdge);
					}

					partitionEdge = nextPartitionEdge;
				}
			}
		}
	};

	class PreprocessEpilogueTask : public Cm::Task
	{
		PX_NOCOPY(PreprocessEpilogueTask)
		SharedContext&	mContext;
	public:
		PreprocessEpilogueTask(PxU64 contextID, SharedContext& context) :
			Cm::Task(contextID), mContext(context)
			{}

		virtual const char* getName() const	PX_OVERRIDE	PX_FINAL
		{
			return "PxgIncrementalPartitioning_PreprocessEpilogueTask";
		}

		virtual void runInternal()	PX_OVERRIDE	PX_FINAL
		{
			Cm::FlushPool& flushPool = mContext.mFlushPool;

			ControlTask* removeEdgesFromPartitionsTask = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ControlTask)), ControlTask)(mContextID, mContext, REMOVE_EDGES_FROM_PARTITIONS);
			RemoveBatchedSpecialEdgesTask* removeSpecialEdgesTask = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(RemoveBatchedSpecialEdgesTask)), RemoveBatchedSpecialEdgesTask)(mContextID, mContext);
			ControlTask* removeSerialContinuationTask = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ControlTask)), ControlTask)(mContextID, mContext, DESTROY_EDGES_PROCESS_FOUND_PATCHES);

			removeSerialContinuationTask->setContinuation(mContext.mContinuation);
			removeEdgesFromPartitionsTask->setContinuation(removeSerialContinuationTask);
			removeSpecialEdgesTask->setContinuation(removeSerialContinuationTask);
			removeSpecialEdgesTask->removeReference();
			removeEdgesFromPartitionsTask->removeReference();

			// PT: TODO: the split could be better here
			const PxU32 nbToGo = mContext.mNbLostFoundPatchManagers;
			const PxU32 numWorkerTasks = mContext.mContinuation->getTaskManager()->getCpuDispatcher()->getWorkerCount();
			PxU32 nbPerTask = 0xffffffff;
			if(numWorkerTasks>2)
			{
				nbPerTask = nbToGo/(numWorkerTasks*2);
				nbPerTask = PxMax(nbPerTask, 32u);
			}

			for(PxU32 a=0; a<nbToGo; a+=nbPerTask)
			{
				const PxU32 nbToProcess = PxMin(nbToGo - a, nbPerTask);
				ControlTask* removeUpdateEdgeLLTask = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ControlTask)), ControlTask)(mContextID, mContext, UPDATE_EDGE_LL, a, nbToProcess);
				startTask(removeUpdateEdgeLLTask, removeSerialContinuationTask);
			}

			removeSerialContinuationTask->removeReference();

		}
	};
}

// PT: a multi-threaded version of processLostPatches_Reference
void PxgIncrementalPartition::processLostPatchesMT(	IG::IslandSim& islandSim, Cm::FlushPool& flushPool, PxBaseTask* continuation,
													PxsContactManager** lostFoundPatchManagers, PxU32 nbLostFoundPatchManagers, const PxsContactManagerOutputCounts* lostFoundPairOutputs,
													PxgBodySimManager& bodySimManager, PxgJointManager& jointManager)
{
	mDestroyedContactEdgeIndices.forceSize_Unsafe(0);

	SharedContext* sharedContext = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(SharedContext)), SharedContext)(
		*this,
		islandSim, flushPool, continuation, bodySimManager, jointManager,
		lostFoundPairOutputs, lostFoundPatchManagers, nbLostFoundPatchManagers);

	// PT: this code is fairly tedious to multithread:
	// - the input buffer contains both lost & found patches, in arbitrary order. We don't know where the lost patches are so the load balancing is tricky.
	// - parsing the buffer is actually quite expensive due to all the data it fetches from random places. So ideally we would only do that once. Versions
	//   that redo the parsing in multiple threads just spread the cost to all cores.
	// - the code does heterogeneous bits of work that all require different multithreading solutions.
	// - some bits can be trivially multithreaded, some bits must remain sequential (sometimes just to preserve determinism), etc.
	//
	// This version uses 2*N + 4 tasks:
	// - N initial "preprocess" tasks that parse the input data and do as much as possible in parallel (various buffer updates, edge coloring, etc).
	// - an epilogue task that runs after these N initial tasks. This epilogue task then spawns:
	//    - a task that removes edges from partitions. (*)
	//    - a task that removes "special handled" edges from external buffers. (*)
	//    - N tasks to update the partition edge linked list.
	//    - a continuation task for the N + 2 previous tasks that runs whatever serial code is left.
	// (*) must remain single-threaded / serial to preserve determinism

	PreprocessEpilogueTask* preprocessEpilogueTask = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(PreprocessEpilogueTask)), PreprocessEpilogueTask)(mContextID, *sharedContext);
	preprocessEpilogueTask->setContinuation(continuation);

	// PT: there is no attempt at clever load balancing yet. We just trivially split the input data.
	const PxU32 nbToGo = nbLostFoundPatchManagers;
	const PxU32 numWorkerTasks = continuation->getTaskManager()->getCpuDispatcher()->getWorkerCount();
	PxU32 nbPerTask = 0xffffffff;
	if(numWorkerTasks>2)
	{
		nbPerTask = nbToGo/(numWorkerTasks*2);
		nbPerTask = PxMax(nbPerTask, 32u);
	}

	PreprocessTask* previousTask = NULL;
	PreprocessTask* taskHead = NULL;
	PxU32 nbTasks = 0;

	for(PxU32 a=0; a<nbToGo; a+=nbPerTask)
	{
		const PxU32 nbToProcess = PxMin(nbToGo - a, nbPerTask);
		PreprocessTask* preprocessTask = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(PreprocessTask)), PreprocessTask)(mContextID, *sharedContext, a, nbToProcess);

		PartitionEdgeBatch* batch = NULL;
		if(nbTasks>=mBatches.size())
		{
			const PxU32 oldSize = mBatches.size();
			const PxU32 newSize = oldSize*2 < 32 ? 32 : oldSize*2;
			mBatches.resize(newSize);
			for(PxU32 i=oldSize;i<newSize;i++)
				mBatches[i] = NULL;
		}
		batch = mBatches[nbTasks];
		if(batch)
		{
			batch->mEdges.clear();
		}
		else
		{
			batch = PX_NEW(PartitionEdgeBatch);
			mBatches[nbTasks] = batch;
		}

		preprocessTask->mBatch = batch;

		startTask(preprocessTask, preprocessEpilogueTask);

		updateTaskLinkedList(previousTask, preprocessTask, taskHead);
		nbTasks++;
	}
	sharedContext->mTaskHead = taskHead;
	preprocessEpilogueTask->removeReference();
}

///////////////////////////////////////////////////////////////////////////////

void PxgIncrementalPartition::processLostPatches_Reference(
	IG::IslandSim& islandSim, PxgBodySimManager& bodySimManager, PxgJointManager& jointManager,
	PxsContactManager** lostFoundPatchManagers, PxU32 nbLostFoundPatchManagers, const PxsContactManagerOutputCounts* lostFoundPairOutputs)
{
	PX_PROFILE_ZONE("PxgIncrementalPartition::processLostPatches", mContextID);

	const IG::CPUExternalData& islandSimCpuData = islandSim.mCpuData;
	PX_ASSERT(islandSim.mGpuData);
	IG::GPUExternalData& islandSimGpuData = *islandSim.mGpuData;

	mDestroyedContactEdgeIndices.forceSize_Unsafe(0);

	// Process lost patches. These are CMs that reduced the number of contact patches they had.
	{
		PX_PROFILE_ZONE("LostPatches", mContextID);

		for (PxU32 a = 0; a < nbLostFoundPatchManagers; ++a)
		{
			// PT: skip found patches
			const PxsContactManagerOutputCounts& output = lostFoundPairOutputs[a];
			
			if(!isLostPatch(output))
				continue;

			const PxsContactManager* manager = lostFoundPatchManagers[a];
			const PxcNpWorkUnit& unit = manager->getWorkUnit();

			if (!(unit.mFlags & PxcNpWorkUnitFlag::eDISABLE_RESPONSE))
			{
				const IG::EdgeIndex edgeIndex = unit.mEdgeIndex;
				PartitionEdge* partitionEdge = islandSimGpuData.getFirstPartitionEdge(edgeIndex);

				//KS - if this is NULL, it means this unit was also destroyed and will be included in the destroyedEdgeCount (i.e. NP detected a lost touch at the same time as BP detected a lost pair
				//PX_ASSERT(partitionEdge != NULL || containedInDestroyedEdges(manager, destroyedEdges, destroyedEdgeCount));
				if (partitionEdge)
				{
					if (output.nbPatches == 0 && output.prevPatches != 0)
					{
						decreaseNodeInteractionCounts(mNodeInteractionCountArray, partitionEdge->mNode0, partitionEdge->mNode1);

#if RECORD_DESTROYED_EDGES_IN_FOUND_LOST_PASSES
						mDestroyedContactEdgeIndices.pushBack(edgeIndex);
#endif
					}

					for (PxU32 b = output.nbPatches; b < output.prevPatches; ++b)
					{
						if(!partitionEdge)
							break;

						PartitionEdge* nextEdge = partitionEdge->mNextPatch;
						//Patches are stored last->first in the Edge structure, so we can just iterate over the edges
						removeEdge(partitionEdge, islandSimGpuData, bodySimManager);
						partitionEdge = nextEdge;
					}
				}
			}
		}
	}

	destroyEdges(islandSimCpuData, islandSimGpuData, bodySimManager, jointManager, true, false);
}

void PxgIncrementalPartition::processFoundPatches_Reference(IG::IslandSim& islandSim, PxgBodySimManager& bodySimManager,
	PxsContactManager** lostFoundPatchManagers, PxU32 nbLostFoundPatchManagers, const PxsContactManagerOutputCounts* lostFoundPairOutputs)
{
	PX_PROFILE_ZONE("PxgIncrementalPartition::processFoundPatches", mContextID);

	const IG::CPUExternalData& islandSimCpuData = islandSim.mCpuData;
	PX_ASSERT(islandSim.mGpuData);
	IG::GPUExternalData& islandSimGpuData = *islandSim.mGpuData;

	{
		PX_PROFILE_ZONE("FoundPatches", mContextID);

#if USE_SPLIT_SECOND_PASS_ISLAND_GEN
		const PxBitMap& activeCMBitmap = mActiveCMBitmapCopy;
#else
		const PxBitMap& activeCMBitmap = islandSimGpuData.getActiveContactManagerBitmap();
#endif
		for (PxU32 a = 0; a < nbLostFoundPatchManagers; ++a)
		{
			//The list can now contain found and lost patches, so we need to know which we are dealing with. If it's a lost patch, this 
			//will be processed in the next stage!
			// PT: skip lost patches
			const PxsContactManagerOutputCounts& output = lostFoundPairOutputs[a];
			if(!isFoundPatch(output))
				continue;
			const PxU32 prevPatches = output.prevPatches;

			const PxsContactManager* manager = lostFoundPatchManagers[a];
			const PxcNpWorkUnit& unit = manager->getWorkUnit();
			const IG::EdgeIndex edgeIndex = unit.mEdgeIndex;

			if (!(unit.mFlags & PxcNpWorkUnitFlag::eDISABLE_RESPONSE) && activeCMBitmap.boundedTest(unit.mEdgeIndex))
			{
				//We either add all patches, or we add only the new patches. This decision is made based on whether there is already
				//a partition edge
				const PxU32 startIndex = islandSimGpuData.getFirstPartitionEdge(edgeIndex) ? prevPatches : 0;

				const PxNodeIndex node1 = islandSimCpuData.getNodeIndex1(edgeIndex);
				const PxNodeIndex node2 = islandSimCpuData.getNodeIndex2(edgeIndex);

				for (PxU32 b = startIndex; b < output.nbPatches; ++b)
				{
					PartitionEdge* edge = addEdge_Stage1(islandSim, edgeIndex, b, unit.mNpIndex, node1, node2);
					const bool specialHandled = addContactManager(edge, unit, bodySimManager);
					addEdge_Stage2(islandSimGpuData, edgeIndex, edge, specialHandled, true, true);
				}

				if (startIndex == 0)
				{
#if RECORD_DESTROYED_EDGES_IN_FOUND_LOST_PASSES
					mDestroyedContactEdgeIndices.pushBack(edgeIndex); //KS - this will potentially hit *if* CCD is enabled
#endif
					increaseNodeInteractionCounts(mNodeInteractionCountArray, node1, node2);
				}
			}
		}
	}
}

// PT: walk the patch linked list and remove all related edges
PX_FORCE_INLINE void PxgIncrementalPartition::removeAllEdges(IG::GPUExternalData& islandSimGpuData, PxgBodySimManager& bodySimManager, PartitionEdge* partitionEdge)
{
	while(partitionEdge)
	{
		PartitionEdge* nextEdge = partitionEdge->mNextPatch;
		removeEdge(partitionEdge, islandSimGpuData, bodySimManager);
		partitionEdge = nextEdge;
	}
}

void PxgIncrementalPartition::destroyEdges(const IG::CPUExternalData& islandSimCpuData, IG::GPUExternalData& islandSimGpuData, PxgBodySimManager& bodySimManager, PxgJointManager& jointManager, bool clearDestroyedEdges, bool recordDestroyedEdges)
{
	PX_PROFILE_ZONE("DestroyedEdges", mContextID);

	const PxU32 destroyedEdgeCount = islandSimGpuData.getNbDestroyedPartitionEdges();
	//printf("destroyedEdgeCount: %d\n", destroyedEdgeCount);
	if(!destroyedEdgeCount)
		return;

#if RECORD_DESTROYED_EDGES_IN_FOUND_LOST_PASSES
	recordDestroyedEdges = true;
#endif

	PartitionEdge** destroyedEdges = islandSimGpuData.getDestroyedPartitionEdges();

	for (PxU32 a = 0; a < destroyedEdgeCount; ++a)
	{
		PartitionEdge* partitionEdge = destroyedEdges[a];
		if (partitionEdge)
		{
			decreaseNodeInteractionCounts(mNodeInteractionCountArray, partitionEdge->mNode0, partitionEdge->mNode1);

			const PxU8 edgeType = mPartitionIndexArray[partitionEdge->mUniqueIndex].mCType;

			if (edgeType == PxgEdgeType::eCONSTRAINT || edgeType == PxgEdgeType::eARTICULATION_CONSTRAINT)
				jointManager.removeJoint(partitionEdge->getEdgeIndex(), mNpIndexArray, islandSimCpuData, islandSimGpuData);
			else if(recordDestroyedEdges)
				mDestroyedContactEdgeIndices.pushBack(partitionEdge->getEdgeIndex());

			removeAllEdges(islandSimGpuData, bodySimManager, partitionEdge);
		}
	}

	if(clearDestroyedEdges)
		islandSimGpuData.clearDestroyedPartitionEdges();
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	class UpdateIncrementalIslandsTask : public Cm::Task
	{
	public:
		enum Codepath
		{
			REFERENCE_VERSION,
			PART_1_AND_2_0,
			PART_2_1,
			PART_2_2_AND_3,
			PART_2_2_ADD_CONTACT_MANAGER,
			PART_2_2_EDGE_COLORING,
			PART_2_2_UPDATE_PARTITION_EDGE_LL_HEAD,
			PART_3,
		};
	protected:
		PX_NOCOPY(UpdateIncrementalIslandsTask)

		PxgIncrementalPartition&			mIncrementalPartition;
		IG::IslandSim&						mIslandSim;
		const IG::AuxCpuData&				mIslandManagerData;
		PxsContactManagerOutputIterator&	mIterator;
		PxgBodySimManager&					mBodySimManager;
		PxgJointManager&					mJointManager;
		const Codepath						mCodepath;
	public:
		PxU32								mStartIndex;
		PxU32								mNbToProcess;

		UpdateIncrementalIslandsTask(	PxU64 contextID, PxgIncrementalPartition& partition, IG::IslandSim& islandSim, const IG::AuxCpuData& islandManagerData,
										PxsContactManagerOutputIterator& iterator, PxgBodySimManager& bodySimManager, PxgJointManager& jointManager, Codepath codepath) :
			Cm::Task				(contextID),
			mIncrementalPartition	(partition),
			mIslandSim				(islandSim),
			mIslandManagerData		(islandManagerData),
			mIterator				(iterator),
			mBodySimManager			(bodySimManager),
			mJointManager			(jointManager),
			mCodepath				(codepath),
			mStartIndex				(0),
			mNbToProcess			(0xffffffff)
			{}

		virtual const char* getName() const	PX_OVERRIDE	PX_FINAL
		{
			return "PxgIncrementalPartitioning_UpdateIncrementalIslandsTask";
		}
	
		virtual void runInternal()	PX_OVERRIDE	PX_FINAL
		{
			const Codepath codepath = mCodepath;
			if(codepath == REFERENCE_VERSION)
			{
				mIncrementalPartition.updateIncrementalIslands_Reference(mIslandSim, mIslandManagerData, mIterator, mBodySimManager, mJointManager);
			}
			else if(codepath == PART_1_AND_2_0)
			{
				mIncrementalPartition.updateIncrementalIslands_Part1(mIslandSim, mIslandManagerData, mIterator, mBodySimManager, mJointManager);
				mIncrementalPartition.updateIncrementalIslands_Part2_0(mIslandSim, mIslandManagerData, mIterator);
			}
			else if(codepath == PART_2_1)
			{
				const PxU32 nbToProcess = mNbToProcess == 0xffffffff ? mIncrementalPartition.mPart2WorkItems.size() : mNbToProcess;

				mIncrementalPartition.updateIncrementalIslands_Part2_1(mStartIndex, nbToProcess, mIslandSim, mIslandManagerData);
			}
			else if(codepath == PART_2_2_AND_3)
			{
				mIncrementalPartition.updateIncrementalIslands_Part2_2(mIslandSim, mBodySimManager, true, true, true);
				mIncrementalPartition.updateIncrementalIslands_Part3(mIslandSim, mJointManager);
			}
			else if(codepath == PART_2_2_ADD_CONTACT_MANAGER)
			{
				mIncrementalPartition.updateIncrementalIslands_Part2_2(mIslandSim, mBodySimManager, true, false, false);
			}
			else if(codepath == PART_2_2_EDGE_COLORING)
			{
				mIncrementalPartition.updateIncrementalIslands_Part2_2(mIslandSim, mBodySimManager, false, true, false);
			}
			else if(codepath == PART_2_2_UPDATE_PARTITION_EDGE_LL_HEAD)
			{
				mIncrementalPartition.updateIncrementalIslands_Part2_2(mIslandSim, mBodySimManager, false, false, true);
			}
			else if(codepath == PART_3)
			{
				mIncrementalPartition.updateIncrementalIslands_Part2_2_ProcessEdgeCases(mIslandSim);
				mIncrementalPartition.updateIncrementalIslands_Part3(mIslandSim, mJointManager);
			}
		}
	};

	class UnlockLastIncrementalIslandsTasks : public Cm::Task
	{
		PX_NOCOPY(UnlockLastIncrementalIslandsTasks)
		UpdateIncrementalIslandsTask&	mTask0;
		UpdateIncrementalIslandsTask&	mTask1;
		UpdateIncrementalIslandsTask&	mTask2;
	public:
		UnlockLastIncrementalIslandsTasks(PxU64 contextID,
			UpdateIncrementalIslandsTask& task0,
			UpdateIncrementalIslandsTask& task1,
			UpdateIncrementalIslandsTask& task2
		) : Cm::Task(contextID), mTask0(task0), mTask1(task1), mTask2(task2)	{}

		virtual const char* getName() const	PX_OVERRIDE	PX_FINAL
		{
			return "UnlockLastIncrementalIslandsTasks";
		}
	
		virtual void runInternal()	PX_OVERRIDE	PX_FINAL
		{
			mTask0.removeReference();
			mTask1.removeReference();
			mTask2.removeReference();
		}
	};
}

void PxgIncrementalPartition::updateIncrementalIslands(
	IG::IslandSim& islandSim, const IG::AuxCpuData& islandManagerData,
	Cm::FlushPool* flushPool, PxBaseTask* continuation,
	PxsContactManagerOutputIterator& iterator, PxgBodySimManager& bodySimManager, PxgJointManager& jointManager)
{
	PX_PROFILE_ZONE("PxgIncrementalPartition::updateIncrementalIslands", mContextID);

	if(flushPool && continuation)
	{
		if(gRunDefaultVersion)
		{
			// PT: run serial reference version in separate task
			UpdateIncrementalIslandsTask* task = PX_PLACEMENT_NEW(flushPool->allocate(sizeof(UpdateIncrementalIslandsTask)), UpdateIncrementalIslandsTask)
				(mContextID, *this, islandSim, islandManagerData, iterator, bodySimManager, jointManager, UpdateIncrementalIslandsTask::REFERENCE_VERSION);
			startTask(task, continuation);
		}
		else
		{
			// PT: see the plan in updateIncrementalIslands_Reference().
			//
			// We want to optimize "Part 2" (i.e. effectively the "ActivatedContacts" profile zone), which itself has multiple subparts. Code is like:
			//
			//	foreach activated contact i:
			//		const IG::EdgeIndex edgeId = activatedContacts[a];
			//		if (activeCMBitmap.test(edgeId))
			//			PxsContactManager* cm = islandManagerData.getContactManager(edgeId);
			//			if (cm)
			//				if (islandSimGpuData.getFirstPartitionEdge(edgeId) == NULL)
			//					PxcNpWorkUnit& unit = cm->getWorkUnit();
			//					increaseNodeInteractionCounts(...);						(2)
			//					foreach contact patch b:
			//						PartitionEdge* edge = addEdge_Stage1(...);			(1)(2)
			//						bool specialHandled = addContactManager(...);		(2)(3)
			//						addEdge_Stage2(...);								(3)
			//					unit.mFrictionPatchCount = 0;							(2)
			//			mDestroyedContactEdgeIndices.pushBack(edgeId);					(1)
			//
			// It is a mix of:
			// (1) things that cannot easily be multithreaded, we will need to run these serially.
			// (2) things that can be trivially multithreaded with multiple tasks.
			// (3) things that can be multithreaded using one "single-threaded" task for each aspect of them.
			//
			// Rough plan then:
			//     - run serial part / preallocate buffers (part2_0)
			//     - run Stage1 with multiple tasks (part2_1)
			//     - run addContactManager and Stage2 in 2 parallel tasks (part2_2) like we did for lost patches
			//          - in theory this is not possible as addContactManager can fail and be re-routed to Stage2. This will need some cleanup edge cases.
			//
			// This is just for part 2. We also need to run parts 1 and 3.
			//
			// We don't need to run the first part of part 2 in a separate task, we can do it right here. So the pipeline would be:
			// - part1 and part2_0 right here
			// - spawn multiple tasks for part2_1
			// - then a task that unlocks the two last parts
			// - part2_2 split into two parts running in parallel in separate tasks
			// - final task to do part3

			// PT: TODO: delegate tasks / static allocs?

			updateIncrementalIslands_Part1(islandSim, islandManagerData, iterator, bodySimManager, jointManager);
			updateIncrementalIslands_Part2_0(islandSim, islandManagerData, iterator);

			// PT: pipeline:
			// PART_2_1	=>	unlockTask =>	PART_2_2_true_false_false	=> PART_3
			// PART_2_1						PART_2_2_false_true_false
			// PART_2_1						PART_2_2_false_false_true
			// ...
			// PART_2_1

			// PT: part3 is the serial task we run in the end
			UpdateIncrementalIslandsTask* part3task = PX_PLACEMENT_NEW(flushPool->allocate(sizeof(UpdateIncrementalIslandsTask)), UpdateIncrementalIslandsTask)(mContextID, *this, islandSim, islandManagerData, iterator, bodySimManager, jointManager, UpdateIncrementalIslandsTask::PART_3);
			part3task->setContinuation(continuation);

			// PT: part2s run in parallel in separate threads
			UpdateIncrementalIslandsTask* part2_cm = PX_PLACEMENT_NEW(flushPool->allocate(sizeof(UpdateIncrementalIslandsTask)), UpdateIncrementalIslandsTask)(mContextID, *this, islandSim, islandManagerData, iterator, bodySimManager, jointManager, UpdateIncrementalIslandsTask::PART_2_2_ADD_CONTACT_MANAGER);
			part2_cm->setContinuation(part3task);

			UpdateIncrementalIslandsTask* part2_ec = PX_PLACEMENT_NEW(flushPool->allocate(sizeof(UpdateIncrementalIslandsTask)), UpdateIncrementalIslandsTask)(mContextID, *this, islandSim, islandManagerData, iterator, bodySimManager, jointManager, UpdateIncrementalIslandsTask::PART_2_2_EDGE_COLORING);
			part2_ec->setContinuation(part3task);

			UpdateIncrementalIslandsTask* part2_ll = PX_PLACEMENT_NEW(flushPool->allocate(sizeof(UpdateIncrementalIslandsTask)), UpdateIncrementalIslandsTask)(mContextID, *this, islandSim, islandManagerData, iterator, bodySimManager, jointManager, UpdateIncrementalIslandsTask::PART_2_2_UPDATE_PARTITION_EDGE_LL_HEAD);
			part2_ll->setContinuation(part3task);

			// PT: the unlock task starts part2 tasks after part1 is done
			UnlockLastIncrementalIslandsTasks* unlockTask = PX_PLACEMENT_NEW(flushPool->allocate(sizeof(UnlockLastIncrementalIslandsTasks)), UnlockLastIncrementalIslandsTasks)(mContextID, *part2_cm, *part2_ec, *part2_ll);
			unlockTask->setContinuation(continuation);

			// PT: this spawns part1 tasks
			const PxU32 nbToGo = mPart2WorkItems.size();
			const PxU32 numWorkerTasks = continuation->getTaskManager()->getCpuDispatcher()->getWorkerCount();
			PxU32 nbPerTask = 0xffffffff;
			if(numWorkerTasks>2)
			{
				nbPerTask = nbToGo/(numWorkerTasks*2);
				nbPerTask = PxMax(nbPerTask, 32u);
			}

			for(PxU32 a=0; a<nbToGo; a+=nbPerTask)
			{
				const PxU32 nbToProcess = PxMin(nbToGo - a, nbPerTask);
				UpdateIncrementalIslandsTask* task = PX_PLACEMENT_NEW(flushPool->allocate(sizeof(UpdateIncrementalIslandsTask)), UpdateIncrementalIslandsTask)(mContextID, *this, islandSim, islandManagerData, iterator, bodySimManager, jointManager, UpdateIncrementalIslandsTask::PART_2_1);

				task->mStartIndex	= a;
				task->mNbToProcess	= nbToProcess;

				startTask(task, unlockTask);
			}

			unlockTask->removeReference();
			part3task->removeReference();
		}
	}
	else
	{
		// PT: run serial version in the calling thread
		updateIncrementalIslands_Reference(islandSim, islandManagerData, iterator, bodySimManager, jointManager);
	}
}

void PxgIncrementalPartition::updateIncrementalIslands_Reference(
	IG::IslandSim& islandSim, const IG::AuxCpuData& islandManagerData,
	PxsContactManagerOutputIterator& iterator, PxgBodySimManager& bodySimManager, PxgJointManager& jointManager)
{
	PX_PROFILE_ZONE("PxgIncrementalPartition::updateIncrementalIslands_Reference", mContextID);

	// PT: the function essentially has 3 parts:
	//
	// Part 1:
	// - various pre-allocations & resizes
	// - destroy edges
	// - process deactivating joints
	// - process deactivating contacts
	// - process activating joints
	//
	// Part 2:
	// - process activated contacts
	//
	// Part 3:
	// - compaction
	// - accumulate slabs
	// - accumulate partitions
	// - finalize partitions
	//
	// Part 2 is the bottleneck in the benchmarks we looked at so far, so the initial plan is simply:
	// - run Part1 single-threaded
	// - then Part2 multi-threaded
	// - then Part3 single-threaded

	updateIncrementalIslands_Part1(islandSim, islandManagerData, iterator, bodySimManager, jointManager);
	updateIncrementalIslands_Part2(islandSim, islandManagerData, iterator, bodySimManager);
	updateIncrementalIslands_Part3(islandSim, jointManager);
}

// PT: first part of reference version that we did not touch:
// - various pre-allocations & resizes
// - destroy edges
// - process deactivating joints
// - process deactivating contacts
// - process activating joints
void PxgIncrementalPartition::updateIncrementalIslands_Part1(
	IG::IslandSim& islandSim, const IG::AuxCpuData& islandManagerData,
	PxsContactManagerOutputIterator& iterator,
	PxgBodySimManager& bodySimManager, PxgJointManager& jointManager)
{
	PX_PROFILE_ZONE("PxgIncrementalPartition::updateIncrementalIslands_Part1", mContextID);

	const IG::CPUExternalData& islandSimCpuData = islandSim.mCpuData;
	PX_ASSERT(islandSim.mGpuData);
	IG::GPUExternalData& islandSimGpuData = *islandSim.mGpuData;

	//KS - TODO - plumb articulation contacts/joints into this!

	const PxU32 destroyedEdgeCount = islandSimGpuData.getNbDestroyedPartitionEdges();
	
	/*const PxU32 destroyedEdgeCount = islandSim.getNbDestroyedEdges();
	const IG::EdgeIndex* destroyedEdges = islandSim.getDestroyedEdges();*/

	const PxU32 deactivatingJointCount = islandSim.getNbDeactivatingEdges(IG::Edge::eCONSTRAINT);
	const IG::EdgeIndex* const deactivatingJoints = islandSim.getDeactivatingEdges(IG::Edge::eCONSTRAINT);

	const PxU32 deactivatingContactCount = islandSim.getNbDeactivatingEdges(IG::Edge::eCONTACT_MANAGER);
	const IG::EdgeIndex* const deactivatingContacts = islandSim.getDeactivatingEdges(IG::Edge::eCONTACT_MANAGER);

	//const PxU32 newJointCount = islandSim.getNbDirtyEdges(IG::Edge::eCONSTRAINT);
	const PxU32 activatedJointCount = islandSim.getNbActivatedEdges(IG::Edge::eCONSTRAINT);
	const IG::EdgeIndex* const activatedJoints = islandSim.getActivatedEdges(IG::Edge::eCONSTRAINT);

	const PxU32 activatedContactCount = islandSim.getNbActivatedEdges(IG::Edge::eCONTACT_MANAGER);

	reserveNodes(islandSim.getNbNodes());

	/*mDestroyedContactEdgeIndices->setSize(0);
	mDestroyedContactEdgeIndices->reserve(sizeof(PxU32), activatedContactCount + deactivatingContactCount + destroyedEdgeCount);*/

	mDestroyedContactEdgeIndices.forceSize_Unsafe(0);
	mDestroyedContactEdgeIndices.reserve(activatedContactCount + deactivatingContactCount + destroyedEdgeCount);

	mIsDirtyNode.resize(islandSim.getNbNodes());

	//(1) Process destroyed edges. Note that these would have been destroyed last frame so there is a chance that they might have been recreated this frame
	// (in the event of the AABBs temporarily not overlapping in the BP). To simplify logic, we just remove and recreate these later...
	// PT: that comment about AABBs tells me this code was initially based on the speculative island manager...

	destroyEdges(islandSimCpuData, islandSimGpuData, bodySimManager, jointManager, false, true);

	//Deactivating joints - these are joints that have gone to sleep
	{
		PX_PROFILE_ZONE("DeactivatingJoints", mContextID);

		for (PxU32 a = 0; a < deactivatingJointCount; ++a)
		{
			//remove it from the PxgJointManager
			jointManager.removeJoint(deactivatingJoints[a], mNpIndexArray, islandSimCpuData, islandSimGpuData);

			PartitionEdge* partitionEdge = islandSimGpuData.getFirstPartitionEdge(deactivatingJoints[a]);
			if (partitionEdge)
			{
				decreaseNodeInteractionCounts(mNodeInteractionCountArray, partitionEdge->mNode0, partitionEdge->mNode1);

				removeAllEdges(islandSimGpuData, bodySimManager, partitionEdge);
			}
			islandSimGpuData.setFirstPartitionEdge(deactivatingJoints[a], NULL);
		}
	}

	//(2) Now we process the deactivating contacts. As with lost edges, deactivated edges were computed last frame so they may have been activated again.
	//    If they're activated again, we will process them afterwards in the activatingContactCount stage.
	//	  In this case, we set the prevPatches to be the same as numPatches. This ensures that any found/lost events for woken edges will not also get processed, resulting in
	//	  an incorrect internal state

	{
		PX_PROFILE_ZONE("DeactivatedContacts", mContextID);

		for (PxU32 a = 0; a < deactivatingContactCount; ++a)
		{
			const IG::EdgeIndex edgeId = deactivatingContacts[a];
			
			PartitionEdge* partitionEdge = islandSimGpuData.getFirstPartitionEdge(edgeId);
			if (partitionEdge)
			{
				decreaseNodeInteractionCounts(mNodeInteractionCountArray, partitionEdge->mNode0, partitionEdge->mNode1);

				removeAllEdges(islandSimGpuData, bodySimManager, partitionEdge);

				islandSimGpuData.setFirstPartitionEdge(edgeId, NULL);

				PxsContactManager* cm = islandManagerData.getContactManager(edgeId);
				if (cm)
				{
					PxcNpWorkUnit& workUnit = cm->getWorkUnit();
					PxsContactManagerOutput& output = iterator.getContactManagerOutput(workUnit.mNpIndex);
					output.prevPatches = output.nbPatches; //Ensure that our internal data does not get corrupted by any touch found/lost events
					workUnit.mFrictionPatchCount = 0; //Zero the friction patch count to make sure that we don't access any memory illegally
				}

				mDestroyedContactEdgeIndices.pushBack(edgeId);
			}
		}
	}

	{
		PX_PROFILE_ZONE("ActivatedJoints", mContextID);

		//Activated joints - joints that were woken this frame.
		for (PxU32 a = 0; a < activatedJointCount; ++a)
		{
			//add it to the PxgJointManager
			Dy::Constraint* constraint = islandManagerData.getConstraint(activatedJoints[a]);

			//PX_ASSERT((!constraint->bodyCore0->isKinematic()) || (!constraint->bodyCore1->isKinematic()));

			const PxU32 edgeIndex = activatedJoints[a];

			const PxNodeIndex node1 = islandSimCpuData.getNodeIndex1(edgeIndex);
			const PxNodeIndex node2 = islandSimCpuData.getNodeIndex2(edgeIndex);

			PartitionEdge* edge = addEdge_Stage1(islandSim, edgeIndex, 0, 0xFFFFFFFF, node1, node2);
			const bool specialHandled = addJointManager(edge, bodySimManager);
			addEdge_Stage2(islandSimGpuData, edgeIndex, edge, specialHandled, true, true);

			increaseNodeInteractionCounts(mNodeInteractionCountArray, node1, node2);

			jointManager.addJoint(edgeIndex, constraint, islandSim, mNpIndexArray, mSolverConstants, edge->mUniqueIndex);
		}
	}

	//processFoundPatches(islandManager, foundPatchManagers, nbFoundPatchManagers, foundManagerCounts, *simulationController);
}

void PxgIncrementalPartition::updateIncrementalIslands_Part2_0(IG::IslandSim& islandSim, const IG::AuxCpuData& islandManagerData, PxsContactManagerOutputIterator& iterator)
{
	PX_PROFILE_ZONE("PxgIncrementalPartition::updateIncrementalIslands_Part2_0", mContextID);

	PX_ASSERT(islandSim.mGpuData);
	IG::GPUExternalData& islandSimGpuData = *islandSim.mGpuData;

	const PxBitMap& activeCMBitmap = islandSimGpuData.getActiveContactManagerBitmap();

	const PxU32 activatedContactCount = islandSim.getNbActivatedEdges(IG::Edge::eCONTACT_MANAGER);
	const IG::EdgeIndex* const activatedContacts = islandSim.getActivatedEdges(IG::Edge::eCONTACT_MANAGER);

	{
		PX_PROFILE_ZONE("PreallocateEdgesAndSerialWork", mContextID);

		// PT: TODO: could we do this part earlier in the pipeline, in parallel with something else?
		// Or actually MT it?
		mPart2WorkItems.clear();
		mPart2EdgeCases.clear();

		for (PxU32 a = 0; a < activatedContactCount; ++a)
		{
			const IG::EdgeIndex edgeId = activatedContacts[a];
			if(activeCMBitmap.test(edgeId))
			{
				mDestroyedContactEdgeIndices.pushBack(edgeId);	//KS - looks a bit weird because we didn't "destroy" any edges but this just ensures that we zero the PF count for this edge

				PxsContactManager* cm = islandManagerData.getContactManager(edgeId);
				if(cm)
				{
					if(islandSimGpuData.getFirstPartitionEdge(edgeId) == NULL)
					{
						PxcNpWorkUnit& unit = cm->getWorkUnit();
						const PxsContactManagerOutput& output = iterator.getContactManagerOutput(unit.mNpIndex);

						for (PxU32 b = 0; b < output.nbPatches; ++b)
						{
							Part2WorkItem& item = mPart2WorkItems.insert();
							item.mEdgeID		= edgeId;
							item.mPatchIndex	= PxU16(b);
							item.mPartitionEdge	= mEdgeManager.getEdge(edgeId);		// PT: TODO: batch
						}
					}
				}
			}
		}
	}

	// PT: these are the resizes that happened at the start of addEdge_Stage1
	// PT: TODO: consider refactoring
	{
		PX_PROFILE_ZONE("ResizeBuffers", mContextID);

		if (mEdgeManager.getEdgeCount() >= mPartitionIndexArray.capacity())
		{
			PX_PROFILE_ZONE("ResizeEdgeBuffer", mContextID);
			const PxU32 newSize = PxMax(mEdgeManager.getEdgeCount(), mPartitionIndexArray.capacity() * 2);
			mPartitionIndexArray.reserve(newSize);
			mNpIndexArray.reserve(newSize);
			mPartitionNodeArray.reserve(newSize);
		}

		if (mEdgeManager.getEdgeCount() >= mPartitionIndexArray.size())
		{
			const PxU32 count = mEdgeManager.getEdgeCount();
			mPartitionIndexArray.resizeUninitialized(count);
			mNpIndexArray.resizeUninitialized(count);
			mPartitionNodeArray.resizeUninitialized(count);
		}

		if(mEdgeManager.getEdgeCount() >= mSolverConstants.capacity())
		{
			const PxU32 count = mEdgeManager.getEdgeCount();
			const PxU32 newSize = PxMax(count, mSolverConstants.capacity() * 2);
			mSolverConstants.resize(newSize);
		}
	}
}

// PT: this one called from multiple threads
void PxgIncrementalPartition::updateIncrementalIslands_Part2_1(PxU32 startIndex, PxU32 nbToProcess, IG::IslandSim& islandSim, const IG::AuxCpuData& islandManagerData)
{
	PX_PROFILE_ZONE("PxgIncrementalPartition::updateIncrementalIslands_Part2_1", mContextID);

	const IG::CPUExternalData& islandSimCpuData = islandSim.mCpuData;
	PX_ASSERT(islandSim.mGpuData);
	IG::GPUExternalData& islandSimGpuData = *islandSim.mGpuData;

	const PxBitMap& activeCMBitmap = islandSimGpuData.getActiveContactManagerBitmap();
	PX_UNUSED(activeCMBitmap);

	/*const*/ Part2WorkItem* workItems = mPart2WorkItems.begin();

	const PxU32 last = startIndex + nbToProcess;
	for (PxU32 i=startIndex; i<last; i++)
	{
		const IG::EdgeIndex edgeId = workItems[i].mEdgeID;

		PX_ASSERT(activeCMBitmap.test(edgeId));
		PxsContactManager* cm = islandManagerData.getContactManager(edgeId);
		PX_ASSERT(cm);
		PX_ASSERT(islandSimGpuData.getFirstPartitionEdge(edgeId) == NULL);

		PxcNpWorkUnit& unit = cm->getWorkUnit();

		PX_ASSERT((!unit.mRigidCore0->isKinematic()) || (!unit.mRigidCore1->isKinematic()));

		const PxNodeIndex node1 = islandSimCpuData.getNodeIndex1(edgeId);
		const PxNodeIndex node2 = islandSimCpuData.getNodeIndex2(edgeId);

		const PxU32 b = workItems[i].mPatchIndex;

		PartitionEdge* partitionEdge = workItems[i].mPartitionEdge;

		// PT: this block was part 3) of addEdge_Stage1

		partitionEdge->mNode0 = node1;
		partitionEdge->mNode1 = node2;

		if(b==0)	// PT: this happened only once per source edge
		{
			unit.mFrictionPatchCount = 0; //KS - ensure that the friction patch count is 0
			increaseNodeInteractionCountsMT(mNodeInteractionCountArray, node1, node2);
		}

		if(isKinematic(islandSim, node1))
			partitionEdge->setInfiniteMass0();

		if(isKinematic(islandSim, node2))
			partitionEdge->setInfiniteMass1();

		// PT: this block was part 4) of addEdge_Stage1

		const PxU32 uniqueId = partitionEdge->mUniqueIndex;

		PartitionIndexData& indexData = mPartitionIndexArray[uniqueId];
		indexData.mPatchIndex = PxTo8(b);

		const PxU8 articulationOffset = node1.isArticulation() || node2.isArticulation() ? 2 : 0;
		const PxU32 implicitEdgeType = unit.mNpIndex == 0xffffffff ? IG::Edge::EdgeType::eCONSTRAINT : IG::Edge::EdgeType::eCONTACT_MANAGER;
		indexData.mCType = PxU8(implicitEdgeType) + articulationOffset;

		// PT: this block was part 5) of addEdge_Stage1

		PartitionNodeData& nodeData = mPartitionNodeArray[uniqueId];
		nodeData.mNodeIndex0 = node1;
		nodeData.mNodeIndex1 = node2;

		// PT: this block was part 6) of addEdge_Stage1

		mNpIndexArray[uniqueId] = unit.mNpIndex;

		// PT: this block was part 7) of addEdge_Stage1

		mSolverConstants[uniqueId].mEdgeIndex = edgeId;

		// PT: this block was the first part of addContactManager

		partitionEdge->setIsContact();	// PT: TODO: this could have been setup in the ctor directly

		increaseForceThresholdsMT(unit, partitionEdge, &mNbForceThresholds);

		// PT: mark special edges. Multithreaded classification, not strictly necessary but since we're here...

		workItems[i].mSpecialCase = PxU16(isSpecialCase(partitionEdge));
	}
}

// PT: this is called from 3 different threads for the 3 different parts
void PxgIncrementalPartition::updateIncrementalIslands_Part2_2(IG::IslandSim& islandSim, PxgBodySimManager& bodySimManager, bool dopart1, bool dopart2, bool dopart3)
{
	PX_PROFILE_ZONE("PxgIncrementalPartition::updateIncrementalIslands_Part2_2", mContextID);

	PX_ASSERT(islandSim.mGpuData);
	IG::GPUExternalData& islandSimGpuData = *islandSim.mGpuData;

	// PT: we prepared these work items in updateIncrementalIslands_Part2_0, they are faster to parse now than the initial input data
	const PxU32 nbPartitionEdges = mPart2WorkItems.size();
	const Part2WorkItem* workItems = mPart2WorkItems.begin();

	if(dopart1)	// PT: main part of addContactManager
	{
		PX_PROFILE_ZONE("addContactManager", mContextID);

		for (PxU32 i=0; i<nbPartitionEdges; i++)
		{
			if(workItems[i].mSpecialCase != PxgIncrementalPartition::SPECIAL_CASE_NONE)
			{
				if(!addSpecialCaseContactManager(workItems[i], bodySimManager))
				{
					// PT: we reach this edge-case when an edge is special-handled in theory, but the external buffers in PxgBodySimManager are full
					// and we have to fallback to the regular case. The regular case is handled in another thread in parallel to this loop here, so
					// we cannot just call the edge coloring code directly. We must buffer these edges and process them later in the final serial part.
					mPart2EdgeCases.pushBack(i);
				}
			}
		}
	}

	if(dopart2)	// PT: the edge coloring part of stage 2
	{
		PX_PROFILE_ZONE("Stage2 - edge coloring", mContextID);

		for (PxU32 i=0; i<nbPartitionEdges; i++)
		{
			const IG::EdgeIndex edgeId = workItems[i].mEdgeID;
			PartitionEdge* edge = workItems[i].mPartitionEdge;
			// PT: TODO: we could probably improve that part:
			// - the special case bit could have been set before
			// - the edge coloring & LL computation could perhaps be split like we did in the remove case
			// but the current version will be hard enough to review, so, later maybe.
			addEdge_Stage2(islandSimGpuData, edgeId, edge, workItems[i].mSpecialCase != SpecialCase::SPECIAL_CASE_NONE, true, false);
		}
	}

	if(dopart3)	// PT: the last part of stage 2
	{
		PX_PROFILE_ZONE("Stage2 - setFirstPartitionEdge", mContextID);

		for (PxU32 i=0; i<nbPartitionEdges; i++)
		{
			const IG::EdgeIndex edgeId = workItems[i].mEdgeID;
			PartitionEdge* edge = workItems[i].mPartitionEdge;
			updatePartitionEdgeLinkedListHead(islandSimGpuData, edgeId, edge);
		}
	}
}

// PT: this is called after updateIncrementalIslands_Part2_2 and before updateIncrementalIslands_Part3
void PxgIncrementalPartition::updateIncrementalIslands_Part2_2_ProcessEdgeCases(IG::IslandSim& islandSim)
{
	PX_PROFILE_ZONE("PxgIncrementalPartition::updateIncrementalIslands_Part2_2_ProcessEdgeCases", mContextID);

	PxU32 nb = mPart2EdgeCases.size();
	if(!nb)
		return;

	PX_ASSERT(islandSim.mGpuData);
	IG::GPUExternalData& islandSimGpuData = *islandSim.mGpuData;

	const PxU32* indices = mPart2EdgeCases.begin();
	const Part2WorkItem* workItems = mPart2WorkItems.begin();
	// PT: we now force these edges to be regular edges, and that state must be updated in the edge itself.
	const bool specialHandled = false;
	while(nb--)
	{
		const PxU32 index = *indices++;
		const IG::EdgeIndex edgeId = workItems[index].mEdgeID;
		PartitionEdge* edge = workItems[index].mPartitionEdge;
		addEdge_Stage2(islandSimGpuData, edgeId, edge, specialHandled, true, false);
		// PT: subtle: we must also undo what addEdge_Stage2 did while we were discovering the buffer overflow,
		// otherwise this edge will still be seen as special-handled during removal.
		edge->clearSpecialHandled();
	}
}

// PT: second part of reference version that we are going to optimize:
// - process activated contacts
// This is the reference implementation.
void PxgIncrementalPartition::updateIncrementalIslands_Part2(
	IG::IslandSim& islandSim, const IG::AuxCpuData& islandManagerData,
	PxsContactManagerOutputIterator& iterator,
	PxgBodySimManager& bodySimManager)
{
	PX_PROFILE_ZONE("PxgIncrementalPartition::updateIncrementalIslands_Part2", mContextID);

	{
		PX_PROFILE_ZONE("ActivatedContacts", mContextID);

		//(2) Process activated edges
		//This edge was activated this frame so we add it to the system. This only does anything if there were not "found" pairs first.
		//Be aware that there is no guarantee that cmOutput being read has completed being written to this frame as this reads from the global buffer.
		//However, we know that, if the state changed, it would have been processed by the found pairs case rather than the activated pairs case.

		const IG::CPUExternalData& islandSimCpuData = islandSim.mCpuData;
		PX_ASSERT(islandSim.mGpuData);
		IG::GPUExternalData& islandSimGpuData = *islandSim.mGpuData;

		const PxU32 activatedContactCount = islandSim.getNbActivatedEdges(IG::Edge::eCONTACT_MANAGER);
		const IG::EdgeIndex* const activatedContacts = islandSim.getActivatedEdges(IG::Edge::eCONTACT_MANAGER);
		//printf("activatedContactCount: %d\n", activatedContactCount);

		const PxBitMap& activeCMBitmap = islandSimGpuData.getActiveContactManagerBitmap();

		for (PxU32 a = 0; a < activatedContactCount; ++a)
		{
			const IG::EdgeIndex edgeId = activatedContacts[a];

			if (activeCMBitmap.test(edgeId))
			{
				PxsContactManager* cm = islandManagerData.getContactManager(edgeId);

				if (cm)
				{
					//If the first patch is NULL, it means that the pair hasn't been activated!
					if (islandSimGpuData.getFirstPartitionEdge(edgeId) == NULL)
					{
						PxcNpWorkUnit& unit = cm->getWorkUnit();

						PX_ASSERT((!unit.mRigidCore0->isKinematic()) || (!unit.mRigidCore1->isKinematic()));
						const PxsContactManagerOutput& output = iterator.getContactManagerOutput(unit.mNpIndex);

						const PxNodeIndex node1 = islandSimCpuData.getNodeIndex1(edgeId);
						const PxNodeIndex node2 = islandSimCpuData.getNodeIndex2(edgeId);

						increaseNodeInteractionCounts(mNodeInteractionCountArray, node1, node2);

						for (PxU32 b = 0; b < output.nbPatches; ++b)
						{
							PartitionEdge* edge = addEdge_Stage1(islandSim, edgeId, b, unit.mNpIndex, node1, node2);
							const bool specialHandled = addContactManager(edge, unit, bodySimManager);
							addEdge_Stage2(islandSimGpuData, edgeId, edge, specialHandled, true, true);
						}

						PX_ASSERT(output.nbPatches == 0 || islandSimGpuData.getFirstPartitionEdge(edgeId) != NULL);

						unit.mFrictionPatchCount = 0; //KS - ensure that the friction patch count is 0
					}
				}
				mDestroyedContactEdgeIndices.pushBack(edgeId);	//KS - looks a bit weird because we didn't "destroy" any edges but this just ensures that we zero the PF count for this edge
			}
		}
	}
}

// PT: third part of reference version that we did not touch:
// - compaction
// - accumulate slabs
// - accumulate partitions
// - finalize partitions
void PxgIncrementalPartition::updateIncrementalIslands_Part3(IG::IslandSim& islandSim, PxgJointManager& jointManager)
{
	PX_PROFILE_ZONE("PxgIncrementalPartition::updateIncrementalIslands_Part3", mContextID);

	{
		// removed edges can result in partitions with 0 constraints. If such a partition
		// is in the middle of the partitions array, subsequent partitions might get
		// ignored (see the break statement in the partition for-loop further below).
		// To avoid this, compaction is needed such that all consecutive partition entries
		// have at least one constraint.

		doCompaction();
	}

	PX_ASSERT(islandSim.mGpuData);
	IG::GPUExternalData& islandSimGpuData = *islandSim.mGpuData;

	islandSimGpuData.setEdgeNodeIndexPtr(mNpIndexArray.begin());

	PxU32 nbPatches = 0;
	PxU32 totalConstraints = 0;
	PxU32 totalArticulationContacts = 0;
	PxU32 totalArticulationConstraints = 0;
	PxU32 nbPartitions = 0;
	PxU32 nbContactBatches = 0;
	PxU32 nbConstraintBatches = 0;
	PxU32 nbArtiContactBatches = 0;
	PxU32 nbArtiConstraintBatches = 0;

#if PX_ENABLE_ASSERTS
	mAccumulatedPartitionCount.clear();
	mAccumulatedConstraintCount.clear();
	mAccumulatedArtiPartitionCount.clear();
	mAccumulatedArtiConstraintCount.clear();

	PxU32 accumulation = 0;
	PxU32 accumulatedConstraint = 0;

	PxU32 accumulatedArtics = 0;
	PxU32 accumulatedArticConstraints = 0;
#endif

	const PxU32 batchMask = PXG_BATCH_SIZE - 1;

	{
		PX_PROFILE_ZONE("AccumulateSlabs", mContextID);
		const PxU32 nbSlabs = mPartitionSlabs.size();
		for (PxU32 i = 0; i < nbSlabs; ++i)
		{
			const PartitionSlab* slab = mPartitionSlabs[i];
			for (PxU32 localPartitionId = 0; localPartitionId < PXG_BATCH_SIZE; ++localPartitionId)
			{
				const PartitionIndices* partitionIndices = slab->mPartitions[localPartitionId].mPartitionIndices;
				const PxU32 nbContactManagers = partitionIndices[PxgEdgeType::eCONTACT_MANAGER].size();
				const PxU32 nbConstraints = partitionIndices[PxgEdgeType::eCONSTRAINT].size();
				const PxU32 nbArtiContactManagers = partitionIndices[PxgEdgeType::eARTICULATION_CONTACT].size();
				const PxU32 nbArtiConstraints = partitionIndices[PxgEdgeType::eARTICULATION_CONSTRAINT].size();

				if ((nbContactManagers + nbConstraints + nbArtiContactManagers + nbArtiConstraints) == 0)
					break;

				nbPartitions++;
				nbContactBatches += (nbContactManagers + batchMask) / PXG_BATCH_SIZE;
				nbConstraintBatches += (nbConstraints + batchMask) / PXG_BATCH_SIZE;
				nbArtiContactBatches += (nbArtiContactManagers + batchMask) / PXG_BATCH_SIZE;
				nbArtiConstraintBatches += (nbArtiConstraints + batchMask) / PXG_BATCH_SIZE;

				nbPatches += nbContactManagers;
				totalConstraints += nbConstraints;
				totalArticulationContacts += nbArtiContactManagers;
				totalArticulationConstraints += nbArtiConstraints;

#if PX_ENABLE_ASSERTS
				accumulation = nbContactBatches + nbConstraintBatches;
				accumulatedArtics = nbArtiContactBatches + nbArtiConstraintBatches;

				accumulatedConstraint = nbPatches + totalConstraints;
				accumulatedArticConstraints = totalArticulationContacts + totalArticulationConstraints;

				mAccumulatedPartitionCount.pushBack(accumulation);
				mAccumulatedConstraintCount.pushBack(accumulatedConstraint);

				mAccumulatedArtiPartitionCount.pushBack(accumulatedArtics);
				mAccumulatedArtiConstraintCount.pushBack(accumulatedArticConstraints);
#endif
			}
		}
	}
	//mIncrementalPartition.mAccumulatedPartitionCount.pushBack(accumulation);

	mNbConstraintBatches = nbConstraintBatches;
	mNbContactBatches = nbContactBatches;
	mNbArtiContactBatches = nbArtiContactBatches;
	mNbArtiConstraintBatches = nbArtiConstraintBatches;

	mNbPartitions = nbPartitions;
	mTotalContacts = nbPatches;
	mTotalConstraints = totalConstraints;
	mTotalArticulationContacts = totalArticulationContacts;
	mTotalArticulationConstraints = totalArticulationConstraints;

	mCSlab.mNbMaxPartitions = mCSlab.mUserNbMaxPartitions;

	//combined slabs to maximum number of slabs, which is default to be 32 but user can define the size in the descriptor
	mCSlab.clear();
	mCSlab.mNbPartitions = PxMin<PxU32>(mCSlab.mNbMaxPartitions, mNbPartitions);

	const PxU32 maxPartitionsMask = mCSlab.mNbMaxPartitions - 1;
	PX_ASSERT(PxIsPowerOfTwo(mCSlab.mNbMaxPartitions));
	{
		PX_PROFILE_ZONE("AccumulatePartitions", mContextID);
		for (PxU32 i = 0; i < mNbPartitions; ++i)
		{
			PartitionSlab* slab = mPartitionSlabs[i / PXG_BATCH_SIZE];
			const PxU32 index = i & batchMask;
			Partition& partition = slab->mPartitions[index];
			const PxU32 nbContactManagers = partition.mPartitionIndices[PxgEdgeType::eCONTACT_MANAGER].size();
			const PxU32 nbConstraints = partition.mPartitionIndices[PxgEdgeType::eCONSTRAINT].size();
			const PxU32 nbArticulationContacts = partition.mPartitionIndices[PxgEdgeType::eARTICULATION_CONTACT].size();
			const PxU32 nbArticulationConstraints = partition.mPartitionIndices[PxgEdgeType::eARTICULATION_CONSTRAINT].size();

			if ((nbContactManagers + nbConstraints + nbArticulationContacts + nbArticulationConstraints) == 0)
				break;

			const PxU32 combinedPartitionIndex = i & maxPartitionsMask;
			//mCSlab.mPartitions[index].pushBack(&partition);
			mCSlab.mPartitionArray[combinedPartitionIndex].mPartitions.pushBack(&partition);
		}
	}

	nbContactBatches = 0;
	nbConstraintBatches = 0;
	nbArtiContactBatches = 0;
	nbArtiConstraintBatches = 0;

	PxInt32ArrayPinned& startSlabIter = mStartSlabPerPartition;
	PxInt32ArrayPinned& articStartSlabIter = mArticStartSlabPerPartition;
	PxInt32ArrayPinned& jointPerPartitionIter = mNbJointsPerPartition;
	PxInt32ArrayPinned& artiJointPerPartitionIter = mNbArtiJointsPerPartition;

	startSlabIter.forceSize_Unsafe(0);
	startSlabIter.reserve(nbPartitions);
	startSlabIter.forceSize_Unsafe(nbPartitions);

	articStartSlabIter.forceSize_Unsafe(0);
	articStartSlabIter.reserve(nbPartitions);
	articStartSlabIter.forceSize_Unsafe(nbPartitions);

	jointPerPartitionIter.forceSize_Unsafe(0);
	jointPerPartitionIter.reserve(nbPartitions);
	jointPerPartitionIter.forceSize_Unsafe(nbPartitions);
	
	artiJointPerPartitionIter.forceSize_Unsafe(0);
	artiJointPerPartitionIter.reserve(nbPartitions);
	artiJointPerPartitionIter.forceSize_Unsafe(nbPartitions);

	mJointStartIndices.resize(nbPartitions);
	mContactStartIndices.resize(nbPartitions);
	mArtiJointStartIndices.resize(nbPartitions);
	mArtiContactStartIndices.resize(nbPartitions);

	PxU32 jointStartIndices = 0;
	PxU32 contactStartIndices = 0;
	PxU32 artiContactStartIndices = 0;
	PxU32 artiJointStartIndices = 0;
	{
		PX_PROFILE_ZONE("FinalizePartitions", mContextID);
		for (PxU32 i = 0; i < mCSlab.mNbPartitions; ++i)
		{
			Partition** partitions = mCSlab.mPartitionArray[i].mPartitions.begin();//mCSlab.mPartitions[i].begin();

			PxU32 nbPartitionContactBatches = 0;
			PxU32 nbPartitionConstraintBatches = 0;
			PxU32 nbPartitionArtiContactBatches = 0;
			PxU32 nbPartitionArtiConstraintBatches = 0;

			PxU32 prevBatches = nbContactBatches + nbConstraintBatches;
			PxU32 prevArticBatches = nbArtiContactBatches + nbArtiConstraintBatches;

			for (PxU32 j = 0; j < mCSlab.mPartitionArray[i].mPartitions.size(); ++j)
			{
				const PxU32 k = i + mCSlab.mNbMaxPartitions * j;

				startSlabIter[k] = prevBatches + nbPartitionContactBatches + nbPartitionConstraintBatches;
				articStartSlabIter[k] = prevArticBatches + nbPartitionArtiContactBatches + nbPartitionArtiConstraintBatches;
				Partition& partition = *partitions[j];
				const PxU32 nbContactManagers = partition.mPartitionIndices[PxgEdgeType::eCONTACT_MANAGER].size();
				const PxU32 nbConstraints = partition.mPartitionIndices[PxgEdgeType::eCONSTRAINT].size();
				const PxU32 nbArtiContactManagers = partition.mPartitionIndices[PxgEdgeType::eARTICULATION_CONTACT].size();
				const PxU32 nbArtiConstraints = partition.mPartitionIndices[PxgEdgeType::eARTICULATION_CONSTRAINT].size();

				nbPartitionContactBatches += (nbContactManagers + batchMask) / PXG_BATCH_SIZE;
				nbPartitionArtiContactBatches += (nbArtiContactManagers + batchMask) / PXG_BATCH_SIZE;

				PxU32 nbJointConstraintBatches = (nbConstraints + batchMask) / PXG_BATCH_SIZE;
				PxU32 nbArtiJointConstraintBatches = (nbArtiConstraints + batchMask) / PXG_BATCH_SIZE;

				nbPartitionConstraintBatches += nbJointConstraintBatches;
				nbPartitionArtiConstraintBatches += nbArtiJointConstraintBatches;

				jointPerPartitionIter[k] = nbJointConstraintBatches;
				artiJointPerPartitionIter[k] = nbArtiJointConstraintBatches;

				mJointStartIndices[k] = jointStartIndices;
				mContactStartIndices[k] = contactStartIndices;
				mArtiContactStartIndices[k] = artiContactStartIndices;
				mArtiJointStartIndices[k] = artiJointStartIndices;

				jointStartIndices += nbConstraints;
				contactStartIndices += nbContactManagers;
				artiContactStartIndices += nbArtiContactManagers;
				artiJointStartIndices += nbArtiConstraints;
			}

			nbContactBatches += nbPartitionContactBatches;
			nbConstraintBatches += nbPartitionConstraintBatches;

			mCSlab.mPartitionArray[i].mAccumulatedPartitionCount = nbContactBatches + nbConstraintBatches;
			nbArtiContactBatches += nbPartitionArtiContactBatches;
			nbArtiConstraintBatches += nbPartitionArtiConstraintBatches;
			mCSlab.mPartitionArray[i].mAccumulatedArtiPartitionCount = nbArtiContactBatches + nbArtiConstraintBatches;
		}
	}

	jointManager.update(mNpIndexArray);
}

#if PX_PARTITION_COMPACTION
void PxgIncrementalPartition::pullForwardConstraints(PxU32 nodeIndex)
{
	//PartitionSlab* writeSlab = mSlabs[slabId];
	PxU16 currId = 0;

	const PxU32 nbSlabs = mPartitionSlabs.size();
	for (PxU32 a = 0; a < nbSlabs; ++a)
	{
		PartitionSlab* slab = mPartitionSlabs[a];
		while (currId < PXG_BATCH_SIZE)
		{
			PxU32 mask = ~((1u << currId) - 1u);
			PxU32 bitMask = ((slab->mNodeBitmap[nodeIndex]) & mask);

			currId = PXG_BATCH_SIZE;
			if (bitMask != 0)
			{
				//There is an entry referencing this body later in this slab. Check to see if it can be pulled forward
				currId = PxTo16(PxLowestSetBit(bitMask));
				//copy to this entry...
#if STORE_INDICES_IN_NODE_ENTRIES
	#if STORE_EDGE_DATA_IN_NODE_ENTRIES
				const PartitionEdge* replaceEdge = mEdgeManager.getPartitionEdge(slab->mNodeEntries[nodeIndex].mEdges[currId].mUniqueIndex);
	#else
				const PartitionEdge* replaceEdge = mEdgeManager.getPartitionEdge(slab->mNodeEntries[nodeIndex].mEdges[currId]);
	#endif
#else
				const PartitionEdge* replaceEdge = slab->mNodeEntries[nodeIndex].mEdges[currId];
#endif
				const PxU32 node0Index = replaceEdge->mNode0.index();
				const PxU32 node1Index = replaceEdge->mNode1.index();

				for (PxU32 i = 0; i <= a; ++i)
				{
					PartitionSlab* writeSlab = mPartitionSlabs[i];
					PxU32 map = 0;
					if (!replaceEdge->hasInfiniteMass0())
						map = writeSlab->mNodeBitmap[node0Index];

					if (!replaceEdge->hasInfiniteMass1())
						map |= writeSlab->mNodeBitmap[node1Index];

					if (map != 0xFFFFFFFF)
					{
						PxU32 newId = PxLowestSetBit(~map);
						if (i < a || newId < currId)
						{
							//replaceEdge can be brought forward to an earlier partition
							removeEdgeInternal(slab, replaceEdge, currId);
							addEdgeInternal(replaceEdge, writeSlab, PxTo16(newId), (PxU16)(i*PXG_BATCH_SIZE));

							if (node0Index != nodeIndex && !replaceEdge->hasInfiniteMass0() && !mIsDirtyNode.test(node0Index))
								mIsDirtyNode.set(node0Index);
							else if (node1Index != nodeIndex && !replaceEdge->hasInfiniteMass1() && !mIsDirtyNode.test(node1Index))
								mIsDirtyNode.set(node1Index);
							break;
						}
					}
				}
				currId++; //Increment currId so that we don't loop indefinitely considering this entry
			}
		}

		currId = 0;
	}
}
#endif

void PxgIncrementalPartition::doCompaction()
{
	PX_PROFILE_ZONE("PxgIncrementalPartition::doCompaction", mContextID);

#if PX_PARTITION_COMPACTION
	const PxU32 lastIdx = 0;
	PxBitMap::PxCircularIterator iter(mIsDirtyNode, lastIdx);
	//for (PxU32 a = 0; a < mDirtyNodes.size(); ++a)
	const PxU32 MaxCount = 500;

	PxU32 count = 0;
	PxU32 dirtyIdx;
	while ((dirtyIdx = iter.getNext()) != PxBitMap::PxCircularIterator::DONE && (count++ < MaxCount))
	{
		pullForwardConstraints(dirtyIdx);
		mIsDirtyNode.reset(dirtyIdx);
	}
#endif

	if (mMaxSlabCount)
	{
		PxI32 l = 0;
		PxI32 r = PxI32(mMaxSlabCount - 1);

		while (l < r)
		{
			//Search for an empty element...
			while (l < r)
			{
				const Partition& partition = mPartitionSlabs[PxU32(l / 32)]->mPartitions[PxU32(l & 31)];
				if (	partition.mPartitionIndices[PxgEdgeType::eCONTACT_MANAGER].size() == 0
					&&	partition.mPartitionIndices[PxgEdgeType::eCONSTRAINT].size() == 0
					&&	partition.mPartitionIndices[PxgEdgeType::eARTICULATION_CONTACT].size() == 0
					&&	partition.mPartitionIndices[PxgEdgeType::eARTICULATION_CONSTRAINT].size() == 0)
					break;
				l++;
			}

			//Search for a non-empty element
			while (l < r)
			{
				const Partition& partition = mPartitionSlabs[PxU32(r / 32)]->mPartitions[PxU32(r & 31)];
				if (	partition.mPartitionIndices[PxgEdgeType::eCONTACT_MANAGER].size() != 0
					||	partition.mPartitionIndices[PxgEdgeType::eCONSTRAINT].size() != 0
					||	partition.mPartitionIndices[PxgEdgeType::eARTICULATION_CONTACT].size() != 0
					||	partition.mPartitionIndices[PxgEdgeType::eARTICULATION_CONSTRAINT].size() != 0)
					break;
				r--;
			}

			if (l < r)
			{
				//Swap!!!!
				PartitionSlab* slab = mPartitionSlabs[PxU32(r / 32)];
				PartitionSlab* writeSlab = mPartitionSlabs[PxU32(l / 32)];

				Partition& partition = slab->mPartitions[r & 31];

				for (PxU32 edgeType = 0; edgeType<PxgEdgeType::eEDGE_TYPE_COUNT; edgeType++)
				{
					for (PxU32 a = partition.mPartitionIndices[PxgEdgeType::Enum(edgeType)].size(); a > 0; --a)
					{
						const PartitionEdge* edge = mEdgeManager.getPartitionEdge(partition.mPartitionIndices[PxgEdgeType::Enum(edgeType)][a - 1]);
						removeEdgeInternal(slab, edge, PxU32(r & 31));
						addEdgeInternal(edge, writeSlab, PxU16(l & 31), PxU16(l&(~31)));
					}
				}

				r--;
			}
		}
		mMaxSlabCount = PxU32(r + 1);
	}
}
