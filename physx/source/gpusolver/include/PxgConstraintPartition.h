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

#ifndef PXG_CONSTRAINT_PARTITION_H
#define PXG_CONSTRAINT_PARTITION_H

#define PX_PARTITION_COMPACTION 1

#include "foundation/PxPinnedArray.h"
#include "foundation/PxSList.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxUtilities.h"
#include "PxgSolverBody.h"
#include "PxgSolverConstraintDesc.h"
#include "PxsSimpleIslandManager.h"
#include "PxgDynamicsConfiguration.h"
#include "PxgEdgeType.h"
#include "PxgPartitionNode.h"
#include "PxsPartitionEdge.h"

namespace physx
{
class PxsContactManagerOutputIterator;
class PxgBodySimManager;
class PxgJointManager;
struct PxsContactManagerOutputCounts;

namespace Cm
{
	class FlushPool;
}

#define SLAB_SIZE 512

// PT: defines controlling a large dense array of 32 pointers per node using up a lot of memory.
//
// STORE_INDICES_IN_NODE_ENTRIES stores indices instead of PartitionEdge ptrs in NodeEntries.
// Half the memory usage compared to initial version but more expensive address computation
// and we still decode and use a pointer in the end.
//
// STORE_EDGE_DATA_IN_NODE_ENTRIES stores the edge data directly instead of indices.
// One less indirection but same memory usage as initial version.
//
// Initial version					| Less indirection	| Better Mem usage
// ---------------------------------|-------------------|---------------------
// Initial version					| No				| No
// STORE_INDICES_IN_NODE_ENTRIES	| No				| Yes
// STORE_EDGE_DATA_IN_NODE_ENTRIES	| Yes				| No
//
// Jury is still out regarding which one is best for perf. Meanwhile we use the one with best mem usage.
#define STORE_INDICES_IN_NODE_ENTRIES		1
#if STORE_INDICES_IN_NODE_ENTRIES
	#define STORE_EDGE_DATA_IN_NODE_ENTRIES	0
	#if STORE_EDGE_DATA_IN_NODE_ENTRIES
		struct EdgeData
		{
			PxU32	mUniqueIndex;
			PxU32	mNode0Index;
		};
		typedef EdgeData NodeEntryStorage;
		typedef EdgeData NodeEntryDecoded;
		PX_FORCE_INLINE	void	resetNodeEntryStorage(EdgeData& edge)	{ edge.mUniqueIndex = IG_INVALID_EDGE;	}
		PX_FORCE_INLINE	PxU32	getUniqueId(const EdgeData& edge)		{ return edge.mUniqueIndex;				}
		PX_FORCE_INLINE	PxU32	getNode0Index(const EdgeData& edge)		{ return edge.mNode0Index;				}
	#else
		typedef PxU32 NodeEntryStorage;
		typedef const PartitionEdge* NodeEntryDecoded;
		PX_FORCE_INLINE	void	resetNodeEntryStorage(PxU32& edge)			{ edge = IG_INVALID_EDGE;		}
		PX_FORCE_INLINE	PxU32	getUniqueId(const PartitionEdge* edge)		{ return edge->mUniqueIndex;	}
		PX_FORCE_INLINE	PxU32	getNode0Index(const PartitionEdge* edge)	{ return edge->mNode0.index();	}
	#endif
#else
	typedef const PartitionEdge* NodeEntryStorage;
	typedef const PartitionEdge* NodeEntryDecoded;
	PX_FORCE_INLINE	void	resetNodeEntryStorage(const PartitionEdge*& edge)	{ edge = NULL;					}
	PX_FORCE_INLINE	PxU32	getUniqueId(const PartitionEdge* edge)				{ return edge->mUniqueIndex;	}
	PX_FORCE_INLINE	PxU32	getNode0Index(const PartitionEdge* edge)			{ return edge->mNode0.index();	}
#endif

typedef	Cm::BlockArray<PxU32>	PartitionIndices;
//typedef	PxArray<PxU32>	PartitionIndices;

struct PartitionEdgeSlab
{
	PartitionEdge mEdges[SLAB_SIZE];	//! The slabs
};

class PartitionEdgeManager
{
	PartitionEdge* mFreeEdges;

	PxArray<PartitionEdgeSlab*> mPartitionEdgeSlabs;
	PxArray<void*> mMemory;

	PxU32 mEdgeCount;

	PX_NOINLINE void allocateSlab();
public:
	PartitionEdgeManager();
	~PartitionEdgeManager();

	PX_FORCE_INLINE PartitionEdge*	getEdge(IG::EdgeIndex index);
	PX_FORCE_INLINE void			putEdge(PartitionEdge* edge);

	PX_FORCE_INLINE const PartitionEdge*	getPartitionEdge(PxU32 uniqueId)	const
	{
		return &mPartitionEdgeSlabs[uniqueId / SLAB_SIZE]->mEdges[uniqueId & (SLAB_SIZE - 1)];
	}

	PX_FORCE_INLINE PxU32 getEdgeCount() const { return mEdgeCount; }
};

struct Partition
{
	PartitionIndices mPartitionIndices[PxgEdgeType::eEDGE_TYPE_COUNT];

	Partition()	{}

	//Adds an edge to the partition
	bool addToPartition(PxU32 uniqueIndex, PartitionIndexData& indexData)
	{
		PartitionIndices& indices = mPartitionIndices[indexData.mCType];
		indexData.mPartitionEntryIndex = indices.size();
		indices.pushBack(uniqueIndex);
		return true;
	}

	void removeFromPartition(PxU32 uniqueIndex, PxPinnedArray<PartitionIndexData>& iterator)
	{
		const PartitionIndexData& indexData = iterator[uniqueIndex];
		PartitionIndices& indices = mPartitionIndices[indexData.mCType];

		// AD: defensive coding for OM-90842. If this assert hits, you maybe hit the same issue
		PxU32 size = indices.size();
		if (size == 0)
		{
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "PxgConstraintPartition: attempting to remove an edge from an empty partition. Skipping.");
			PX_ASSERT(false);
			return;
		}

		size--;

		const PxU32 uniqueIdx = indices[size];
		const PxU32 partitionEntryIndex = indexData.mPartitionEntryIndex;
		iterator[uniqueIdx].mPartitionEntryIndex = partitionEntryIndex;
		indices[partitionEntryIndex] = uniqueIdx;
		indices.forceSize_Unsafe(size);
	}
};

struct NodeEntries
{
	NodeEntryStorage	mEdges[PXG_BATCH_SIZE];
};

struct PartitionSlab : public PxUserAllocated
{
	Partition mPartitions[PXG_BATCH_SIZE];				//! Each slab has 32 partitions

	PxArray<PxU32> mNodeBitmap;			//! Each slab 1 integer per-node, recording the presence of the node in any of the slabs

	PxArray<NodeEntries> mNodeEntries;

	PartitionSlab()	{}
};

struct PartitionArray : public PxUserAllocated
{
	PxArray<Partition*> mPartitions;
	PxU32 mAccumulatedPartitionCount;
	PxU32 mAccumulatedArtiPartitionCount;

	PartitionArray() : mPartitions(1024), mAccumulatedPartitionCount(0), mAccumulatedArtiPartitionCount(0)
	{
	}

	void clear()
	{
		mPartitions.forceSize_Unsafe(0);
		mAccumulatedPartitionCount = 0;
		mAccumulatedArtiPartitionCount = 0;
	}
};

class PxgCombinedPartitionSlab : public PxUserAllocated
{
public:

	PxU32 mNbPartitions;
	const PxU32 mUserNbMaxPartitions;
	PxU32 mNbMaxPartitions;
	PartitionArray mPartitionArray[32];

	PxgCombinedPartitionSlab(PxU32 maxNumPartitions) : mUserNbMaxPartitions(maxNumPartitions), mNbMaxPartitions(maxNumPartitions)
	{
	}

	~PxgCombinedPartitionSlab()
	{
	}

	void clear()
	{
		mNbPartitions = 0;
		for (PxU32 i = 0; i< 32; ++i)
			mPartitionArray[i].clear();
	}
};

class PxgIncrementalPartition
{
	PX_NOCOPY(PxgIncrementalPartition)
public:	// PT: TODO: revisit after the dust settles

	PxArray<PartitionSlab*> mPartitionSlabs;

	PartitionEdgeManager mEdgeManager;

	PxU32 mNodeCount;
	PxU32 mNbContactBatches;
	PxU32 mNbConstraintBatches;
	PxU32 mNbArtiContactBatches;
	PxU32 mNbArtiConstraintBatches;

	PxU32 mNbPartitions;
	PxU32 mTotalContacts;
	PxU32 mTotalConstraints;
	PxU32 mTotalArticulationContacts;
	PxU32 mTotalArticulationConstraints;

	PxU32 mMaxSlabCount;
	PxU32 mNbForceThresholds;

#if PX_ENABLE_ASSERTS
	PxArray<PxU32> mAccumulatedPartitionCount; // for contact
	PxArray<PxU32> mAccumulatedConstraintCount; // for joint
	PxArray<PxU32> mAccumulatedArtiPartitionCount; //for contact
	PxArray<PxU32> mAccumulatedArtiConstraintCount; // for constraint
#endif

	PxBitMap mIsDirtyNode;

	PxArray<PxU32>										mNpIndexArray;
	PxPinnedArray<PartitionIndexData>					mPartitionIndexArray;
	PxPinnedArray<PartitionNodeData>					mPartitionNodeArray;
	PxPinnedArray<PxgSolverConstraintManagerConstants>	mSolverConstants;
	PxInt32ArrayPinned									mNodeInteractionCountArray;

	PxInt32ArrayPinned			mDestroyedContactEdgeIndices;

	PxInt32ArrayPinned			mStartSlabPerPartition;
	PxInt32ArrayPinned			mArticStartSlabPerPartition;
	PxInt32ArrayPinned			mNbJointsPerPartition;
	PxInt32ArrayPinned			mNbArtiJointsPerPartition;

	PxArray<PxU32>				mJointStartIndices;
	PxArray<PxU32>				mContactStartIndices;
	PxArray<PxU32>				mArtiContactStartIndices;
	PxArray<PxU32>				mArtiJointStartIndices;

	PxgCombinedPartitionSlab	mCSlab;

	const PxU64					mContextID;

public:

	PxgIncrementalPartition(const PxVirtualAllocator& allocator, PxU32 maxNumPartitions, PxU64 contextID);
	~PxgIncrementalPartition();

	void processLostFoundPatches(	Cm::FlushPool& flushPool, PxBaseTask* continuation,
									IG::IslandSim& islandSim, PxgBodySimManager& bodySimManager, PxgJointManager& jointManager,
									PxsContactManager** lostFoundPatchManagers, PxU32 nbLostFoundPatchManagers, const PxsContactManagerOutputCounts* lostFoundPairOutputs);

	void updateIncrementalIslands(	IG::IslandSim& islandSim, const IG::AuxCpuData& islandManagerData,
									Cm::FlushPool* flushPool, PxBaseTask* continuation,
									PxsContactManagerOutputIterator& iterator, PxgBodySimManager& bodySimManager, PxgJointManager& jointManager);

	// PT: internal reference versions, exposed for UTs
	void processLostPatches_Reference(	IG::IslandSim& islandSim, PxgBodySimManager& bodySimManager, PxgJointManager& jointManager,
										PxsContactManager** lostFoundPatchManagers, PxU32 nbLostFoundPatchManagers, const PxsContactManagerOutputCounts* lostFoundPairOutputs);

	void processFoundPatches_Reference(	IG::IslandSim& islandSim, PxgBodySimManager& bodySimManager,
										PxsContactManager** lostFoundPatchManagers, PxU32 nbLostFoundPatchManagers, const PxsContactManagerOutputCounts* lostFoundPairOutputs);

	void updateIncrementalIslands_Reference(IG::IslandSim& islandSim, const IG::AuxCpuData& islandManagerData,
											PxsContactManagerOutputIterator& iterator,
											PxgBodySimManager& bodySimManager, PxgJointManager& jointManager);

	// PT: edge data
	PX_FORCE_INLINE	const PxPinnedArray<PxgSolverConstraintManagerConstants>&	getSolverConstants()	const	{ return mSolverConstants;	}

	// PT: TODO: what's the difference between mNbPartitions and mCSlab.mNbPartitions ?
	PX_FORCE_INLINE	PxU32	getNbPartitions()					const	{ return mNbPartitions;				}
	PX_FORCE_INLINE	PxU32	getCombinedSlabMaxNbPartitions()	const	{ return mCSlab.mNbMaxPartitions;	}
	PX_FORCE_INLINE	PxU32	getCombinedSlabNbPartitions()		const	{ return mCSlab.mNbPartitions;		}

	PX_FORCE_INLINE	const PxPinnedArray<PartitionIndexData>&	getPartitionIndexArray()	const	{ return mPartitionIndexArray;	}
	PX_FORCE_INLINE	const PxPinnedArray<PartitionNodeData>&		getPartitionNodeArray()		const	{ return mPartitionNodeArray;	}

	PX_FORCE_INLINE	const PxInt32ArrayPinned&		getStartSlabPerPartition()			const	{ return mStartSlabPerPartition;		}
	PX_FORCE_INLINE	const PxInt32ArrayPinned&		getArticStartSlabPerPartition()		const	{ return mArticStartSlabPerPartition;	}
	PX_FORCE_INLINE	const PxInt32ArrayPinned&		getNbJointsPerPartition()			const	{ return mNbJointsPerPartition;			}
	PX_FORCE_INLINE	const PxInt32ArrayPinned&		getNbArticJointsPerPartition()		const	{ return mNbArtiJointsPerPartition;		}
	PX_FORCE_INLINE	const PxInt32ArrayPinned&		getNodeInteractionCountArray()		const	{ return mNodeInteractionCountArray;	}
	PX_FORCE_INLINE	const PxInt32ArrayPinned&		getDestroyedContactEdgeIndices()	const	{ return mDestroyedContactEdgeIndices;	}

	PX_FORCE_INLINE	const PxArray<PxU32>&			getNpIndexArray()					const	{ return mNpIndexArray;					}
	PX_FORCE_INLINE	const PxArray<PartitionSlab*>&	getPartitionSlabs()					const	{ return mPartitionSlabs;				}

	PX_FORCE_INLINE	const PxArray<PxU32>&			getContactStartIndices()			const	{ return mContactStartIndices;			}
	PX_FORCE_INLINE	const PxArray<PxU32>&			getJointStartIndices()				const	{ return mJointStartIndices;			}
	PX_FORCE_INLINE	const PxArray<PxU32>&			getArtiContactStartIndices()		const	{ return mArtiContactStartIndices;		}
	PX_FORCE_INLINE	const PxArray<PxU32>&			getArtiJointStartIndices()			const	{ return mArtiJointStartIndices;		}

	PX_FORCE_INLINE	PxU32	getCSlabAccumulatedPartitionCount(PxU32 index)	const
	{
		PX_ASSERT(index<32);
		return mCSlab.mPartitionArray[index].mAccumulatedPartitionCount;
	}

	PX_FORCE_INLINE	PxU32	getCSlabAccumulatedArtiPartitionCount(PxU32 index)	const
	{
		PX_ASSERT(index<32);
		return mCSlab.mPartitionArray[index].mAccumulatedArtiPartitionCount;
	}

#if PX_ENABLE_ASSERTS
	PX_FORCE_INLINE	const PxArray<PxU32>&	getAccumulatedPartitionCount()		const	{ return mAccumulatedPartitionCount;		}
	PX_FORCE_INLINE	const PxArray<PxU32>&	getAccumulatedConstraintCount()		const	{ return mAccumulatedConstraintCount;		}
	PX_FORCE_INLINE	const PxArray<PxU32>&	getAccumulatedArtiPartitionCount()	const	{ return mAccumulatedArtiPartitionCount;	}
	PX_FORCE_INLINE	const PxArray<PxU32>&	getAccumulatedArtiConstraintCount()	const	{ return mAccumulatedArtiConstraintCount;	}
#endif
	PX_FORCE_INLINE	PxU32	getNbContactBatches()				const	{ return mNbContactBatches;				}
	PX_FORCE_INLINE	PxU32	getNbConstraintBatches()			const	{ return mNbConstraintBatches;			}
	PX_FORCE_INLINE	PxU32	getNbArtiContactBatches()			const	{ return mNbArtiContactBatches;			}
	PX_FORCE_INLINE	PxU32	getNbArtiConstraintBatches()		const	{ return mNbArtiConstraintBatches;		}

	PX_FORCE_INLINE	PxU32	getTotalContacts()					const	{ return mTotalContacts;				}
	PX_FORCE_INLINE	PxU32	getTotalConstraints()				const	{ return mTotalConstraints;				}
	PX_FORCE_INLINE	PxU32	getTotalArticulationContacts()		const	{ return mTotalArticulationContacts;	}
	PX_FORCE_INLINE	PxU32	getTotalArticulationConstraints()	const	{ return mTotalArticulationConstraints;	}

	PX_FORCE_INLINE	bool	hasForceThresholds()				const	{ return mNbForceThresholds!=0;			}

//private:	// PT: TODO: revisit after the dust settles

#if USE_SPLIT_SECOND_PASS_ISLAND_GEN
	PxBitMap	mActiveCMBitmapCopy;
#endif
	void reserveNodes(PxU32 nodeCount);

	void getPreviousAndNextReferencesInSlab(NodeEntryDecoded& prev, NodeEntryDecoded& next, PxU32 index, PxU32 uniqueId, const PartitionSlab* slab, PxU32 slabMask) const;

	PartitionEdge* addEdge_Stage1(const IG::IslandSim& islandSim, IG::EdgeIndex edgeIndex, PxU32 patchIndex, PxU32 npIndex, PxNodeIndex node1, PxNodeIndex node2);

	void addEdge_Stage2(IG::GPUExternalData& islandSimGpuData, IG::EdgeIndex edgeIndex, PartitionEdge* partitionEdge, bool specialHandled, bool doPart1, bool doPart2);

	bool addJointManager(const PartitionEdge* edge, PxgBodySimManager& bodySimManager);
	bool addContactManager(PartitionEdge* edge, const PxcNpWorkUnit& unit, PxgBodySimManager& bodySimManager);

	void removeEdge(PartitionEdge* edge, IG::GPUExternalData& islandSimGpuData, PxgBodySimManager& manager);
	PX_FORCE_INLINE void removeAllEdges(IG::GPUExternalData& islandSimGpuData, PxgBodySimManager& bodySimManager, PartitionEdge* partitionEdge);
	void destroyEdges(const IG::CPUExternalData& islandSimCpuData, IG::GPUExternalData& islandSimGpuData, PxgBodySimManager& bodySimManager, PxgJointManager& jointManager, bool clearDestroyedEdges, bool recordDestroyedEdges);

	void addEdgeInternal(const PartitionEdge* PX_RESTRICT partitionEdge, PartitionSlab* PX_RESTRICT slab, PxU16 id, PxU16 baseId);
	void removeEdgeInternal(PartitionSlab* PX_RESTRICT slab, const PartitionEdge* PX_RESTRICT edge, PxU32 id);

	void doCompaction();
#if PX_PARTITION_COMPACTION
	void pullForwardConstraints(PxU32 nodeIndex);
#endif

public:	// PT: TODO: revisit after the dust settles

	void updateIncrementalIslands_Part1(
		IG::IslandSim& islandSim, const IG::AuxCpuData& islandManagerData,
		PxsContactManagerOutputIterator& iterator,
		PxgBodySimManager& bodySimManager, PxgJointManager& jointManager);

	void updateIncrementalIslands_Part2(
		IG::IslandSim& islandSim, const IG::AuxCpuData& islandManagerData,
		PxsContactManagerOutputIterator& iterator,
		PxgBodySimManager& bodySimManager);

	enum SpecialCase
	{
		SPECIAL_CASE_NONE,
		SPECIAL_CASE_STATIC_RB,
		SPECIAL_CASE_ARTI_SELF,
		SPECIAL_CASE_STATIC_ARTI0,
		SPECIAL_CASE_STATIC_ARTI1
	};

	struct Part2WorkItem
	{
		PxU32	mEdgeID;
		PxU16	mPatchIndex;
		PxU16	mSpecialCase;
		PartitionEdge*	mPartitionEdge;
	};

	PxArray<Part2WorkItem>	mPart2WorkItems;
	PxArray<PxU32>			mPart2EdgeCases;	// PT: indices into mPart2WorkItems

	void updateIncrementalIslands_Part2_0(IG::IslandSim& islandSim, const IG::AuxCpuData& islandManagerData, PxsContactManagerOutputIterator& iterator);

	void updateIncrementalIslands_Part2_1(PxU32 startIndex, PxU32 nbToProcess, IG::IslandSim& islandSim, const IG::AuxCpuData& islandManagerData);

	void updateIncrementalIslands_Part2_2(IG::IslandSim& islandSim, PxgBodySimManager& bodySimManager, bool dopart1, bool dopart2, bool dopart3);

	void updateIncrementalIslands_Part2_2_ProcessEdgeCases(IG::IslandSim& islandSim);

	void updateIncrementalIslands_Part3(IG::IslandSim& islandSim, PxgJointManager& jointManager);

	void processLostPatchesMT(	IG::IslandSim& islandSim, Cm::FlushPool& flushPool, PxBaseTask* continuation,
								PxsContactManager** lostFoundPatchManagers, PxU32 nbLostFoundPatchManagers, const PxsContactManagerOutputCounts* lostFoundPairOutputs,
								PxgBodySimManager& bodySimManager, PxgJointManager& jointManager);

	struct PartitionEdgeBatch : public PxUserAllocated
	{
		PxArray<PartitionEdge*>	mEdges;
	};
	PxArray<PartitionEdgeBatch*>	mBatches;
};

}

#endif

