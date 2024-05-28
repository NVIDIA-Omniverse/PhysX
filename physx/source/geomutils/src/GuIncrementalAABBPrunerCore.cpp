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

#include "CmVisualization.h"
#include "GuIncrementalAABBPrunerCore.h"
#include "GuSqInternal.h"
#include "GuIncrementalAABBTree.h"
#include "GuCallbackAdapter.h"
#include "GuAABBTree.h"
#include "GuAABBTreeQuery.h"
#include "GuSphere.h"
#include "GuBox.h"
#include "GuCapsule.h"
#include "GuQuery.h"

using namespace physx;
using namespace Gu;

#define PARANOIA_CHECKS 0

// PT: TODO: this is copied from SqBounds.h, should be either moved to Gu and shared or passed as a user parameter
	#define SQ_PRUNER_EPSILON	0.005f
	#define SQ_PRUNER_INFLATION	(1.0f + SQ_PRUNER_EPSILON)	// pruner test shape inflation (not narrow phase shape)

IncrementalAABBPrunerCore::IncrementalAABBPrunerCore(const PruningPool* pool) :
	mCurrentTree	(1),
	mLastTree		(0),
	mPool			(pool)
{
	mAABBTree[0].mapping.reserve(256);
	mAABBTree[1].mapping.reserve(256);
	mChangedLeaves.reserve(32);
}

IncrementalAABBPrunerCore::~IncrementalAABBPrunerCore()
{
	release();
}

void IncrementalAABBPrunerCore::release() // this can be called from purge()
{
	for(PxU32 i = 0; i < NUM_TREES; i++)
	{
		PX_DELETE(mAABBTree[i].tree);
		mAABBTree[i].mapping.clear();
		mAABBTree[i].timeStamp = 0;
	}
	mCurrentTree = 1;
	mLastTree = 0;
}

bool IncrementalAABBPrunerCore::addObject(const PoolIndex poolIndex, PxU32 timeStamp)
{
	CoreTree& tree = mAABBTree[mCurrentTree];
	if(!tree.tree || !tree.tree->getNodes())
	{
		if(!tree.tree)
			tree.tree = PX_NEW(IncrementalAABBTree)();
		tree.timeStamp = timeStamp;
	}
	PX_ASSERT(tree.timeStamp == timeStamp);

	mChangedLeaves.clear();
	IncrementalAABBTreeNode* node = tree.tree->insert(poolIndex, mPool->getCurrentWorldBoxes(), mChangedLeaves);
	updateMapping(tree.mapping, poolIndex, node);

#if PARANOIA_CHECKS
	test();
#endif

	return true;
}

void IncrementalAABBPrunerCore::updateMapping(IncrementalPrunerMap& mapping, const PoolIndex poolIndex, IncrementalAABBTreeNode* node)
{
	// if some node leaves changed, we need to update mapping
	if(!mChangedLeaves.empty())
	{
		if(node && node->isLeaf())
		{
			for(PxU32 j = 0; j < node->getNbPrimitives(); j++)
			{
				const PoolIndex index = node->getPrimitives(NULL)[j];
				mapping[index] = node;
			}
		}

		for(PxU32 i = 0; i < mChangedLeaves.size(); i++)
		{
			IncrementalAABBTreeNode* changedNode = mChangedLeaves[i];
			PX_ASSERT(changedNode->isLeaf());

			for(PxU32 j = 0; j < changedNode->getNbPrimitives(); j++)
			{
				const PoolIndex index = changedNode->getPrimitives(NULL)[j];
				mapping[index] = changedNode;
			}
		}
	}
	else
	{
		PX_ASSERT(node->isLeaf());
		mapping[poolIndex] = node;
	}
}

bool IncrementalAABBPrunerCore::removeObject(const PoolIndex poolIndex, const PoolIndex poolRelocatedLastIndex, PxU32& timeStamp)
{
	// erase the entry and get the data
	IncrementalPrunerMap::Entry entry;
	bool foundEntry = true;
	const PxU32 treeIndex = mAABBTree[mLastTree].mapping.erase(poolIndex, entry) ? mLastTree : mCurrentTree;
	// if it was not found in the last tree look at the current tree
	if(treeIndex == mCurrentTree)
		foundEntry = mAABBTree[mCurrentTree].mapping.erase(poolIndex, entry);

	// exit somethings is wrong here, entry was not found here
	// PT: removed assert to avoid crashing all UTs
//	PX_ASSERT(foundEntry);
	if(!foundEntry)
		return false;

	// tree must exist
	PX_ASSERT(mAABBTree[treeIndex].tree);
	CoreTree& tree = mAABBTree[treeIndex];
	timeStamp = tree.timeStamp;

	// remove the poolIndex from the tree, update the tree bounds immediatelly
	IncrementalAABBTreeNode* node = tree.tree->remove(entry.second, poolIndex, mPool->getCurrentWorldBoxes());
	if(node && node->isLeaf())
	{
		for(PxU32 j = 0; j < node->getNbPrimitives(); j++)
		{
			const PoolIndex index = node->getPrimitives(NULL)[j];
			tree.mapping[index] = node;
		}
	}

	// nothing to swap, last object, early exit
	if(poolIndex == poolRelocatedLastIndex)
	{
#if PARANOIA_CHECKS
	test();
#endif
		return true;
	}

	// fix the indices, we need to swap the index with last index
	// erase the relocated index from the tree it is
	IncrementalPrunerMap::Entry relocatedEntry;
	const PxU32 treeRelocatedIndex = mAABBTree[mCurrentTree].mapping.erase(poolRelocatedLastIndex, relocatedEntry) ? mCurrentTree : mLastTree;
	foundEntry = true;
	if(treeRelocatedIndex == mLastTree)
		foundEntry = mAABBTree[mLastTree].mapping.erase(poolRelocatedLastIndex, relocatedEntry);

	if(foundEntry)
	{
		CoreTree& relocatedTree = mAABBTree[treeRelocatedIndex];

		// set the new mapping
		relocatedTree.mapping[poolIndex] = relocatedEntry.second;
		// update the tree indices - swap 
		relocatedTree.tree->fixupTreeIndices(relocatedEntry.second, poolRelocatedLastIndex, poolIndex);
	}

#if PARANOIA_CHECKS
	test();
#endif
	return true;
}

void IncrementalAABBPrunerCore::swapIndex(const PoolIndex poolIndex, const PoolIndex poolRelocatedLastIndex)
{
	// fix the indices, we need to swap the index with last index
	// erase the relocated index from the tre it is
	IncrementalPrunerMap::Entry relocatedEntry;
	const PxU32 treeRelocatedIndex = mAABBTree[mCurrentTree].mapping.erase(poolRelocatedLastIndex, relocatedEntry) ? mCurrentTree : mLastTree;
	bool foundEntry = true;
	if(treeRelocatedIndex == mLastTree)
		foundEntry = mAABBTree[mLastTree].mapping.erase(poolRelocatedLastIndex, relocatedEntry);

	// relocated index is not here
	if(!foundEntry)
		return;

	CoreTree& relocatedTree = mAABBTree[treeRelocatedIndex];

	// set the new mapping
	relocatedTree.mapping[poolIndex] = relocatedEntry.second;
	// update the tree indices - swap 
	relocatedTree.tree->fixupTreeIndices(relocatedEntry.second, poolRelocatedLastIndex, poolIndex);
}

bool IncrementalAABBPrunerCore::updateObject(const PoolIndex poolIndex)
{
	const IncrementalPrunerMap::Entry* entry = mAABBTree[mLastTree].mapping.find(poolIndex);
	const PxU32 treeIndex = entry ? mLastTree : mCurrentTree;
	if(!entry)
		entry = mAABBTree[mCurrentTree].mapping.find(poolIndex);

	// we have not found it
	PX_ASSERT(entry);
	if(!entry)
		return false;

	CoreTree& tree = mAABBTree[treeIndex];
	mChangedLeaves.clear();
	IncrementalAABBTreeNode* node = tree.tree->updateFast(entry->second, poolIndex, mPool->getCurrentWorldBoxes(), mChangedLeaves);
	if(!mChangedLeaves.empty() || node != entry->second)
		updateMapping(tree.mapping, poolIndex, node);

#if PARANOIA_CHECKS
	test(false);
#endif

	return true;
}

PxU32 IncrementalAABBPrunerCore::removeMarkedObjects(PxU32 timeStamp)
{
	// early exit is no tree exists
	if(!mAABBTree[mLastTree].tree || !mAABBTree[mLastTree].tree->getNodes())
	{
		PX_ASSERT(mAABBTree[mLastTree].mapping.size() == 0);
		PX_ASSERT(!mAABBTree[mCurrentTree].tree || mAABBTree[mCurrentTree].timeStamp != timeStamp);
		return 0;
	}

	PX_UNUSED(timeStamp);
	PX_ASSERT(timeStamp == mAABBTree[mLastTree].timeStamp);

	// release the last tree
	CoreTree& tree = mAABBTree[mLastTree];
	PxU32 nbObjects = tree.mapping.size();
	tree.mapping.clear();
	tree.timeStamp = 0;

	tree.tree->release();

	return nbObjects;
}

bool IncrementalAABBPrunerCore::overlap(const ShapeData& queryVolume, PrunerOverlapCallback& pcbArgName) const
{
	bool again = true;
	OverlapCallbackAdapter pcb(pcbArgName, *mPool);

	for(PxU32 i = 0; i < NUM_TREES; i++)
	{
		const CoreTree& tree = mAABBTree[i];
		if(tree.tree && tree.tree->getNodes() && again)
		{
			switch(queryVolume.getType())
			{
				case PxGeometryType::eBOX:
				{
					if(queryVolume.isOBB())
					{	
						const DefaultOBBAABBTest test(queryVolume);
						again = AABBTreeOverlap<true, OBBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, OverlapCallbackAdapter>()(mPool->getCurrentAABBTreeBounds(), *tree.tree, test, pcb);
					}
					else
					{
						const DefaultAABBAABBTest test(queryVolume);
						again = AABBTreeOverlap<true, AABBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, OverlapCallbackAdapter>()(mPool->getCurrentAABBTreeBounds(), *tree.tree, test, pcb);
					}
				}
				break;
				case PxGeometryType::eCAPSULE:
				{
					const DefaultCapsuleAABBTest test(queryVolume, SQ_PRUNER_INFLATION);
					again = AABBTreeOverlap<true, CapsuleAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, OverlapCallbackAdapter>()(mPool->getCurrentAABBTreeBounds(), *tree.tree, test, pcb);
				}
				break;
				case PxGeometryType::eSPHERE:
				{
					const DefaultSphereAABBTest test(queryVolume);
					again = AABBTreeOverlap<true, SphereAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, OverlapCallbackAdapter>()(mPool->getCurrentAABBTreeBounds(), *tree.tree, test, pcb);
				}
				break;
				case PxGeometryType::eCONVEXMESH:
				{
					const DefaultOBBAABBTest test(queryVolume);
					again = AABBTreeOverlap<true, OBBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, OverlapCallbackAdapter>()(mPool->getCurrentAABBTreeBounds(), *tree.tree, test, pcb);			
				}
				break;
			default:
				PX_ALWAYS_ASSERT_MESSAGE("unsupported overlap query volume geometry type");
			}
		}
	}

	return again;
}

bool IncrementalAABBPrunerCore::sweep(const ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback& pcbArgName) const
{
	bool again = true;
	RaycastCallbackAdapter pcb(pcbArgName, *mPool);

	for(PxU32 i = 0; i < NUM_TREES; i++)
	{
		const CoreTree& tree = mAABBTree[i];
		if(tree.tree && tree.tree->getNodes() && again)
		{
			const PxBounds3& aabb = queryVolume.getPrunerInflatedWorldAABB();
			again = AABBTreeRaycast<true, true, IncrementalAABBTree, IncrementalAABBTreeNode, RaycastCallbackAdapter>()(mPool->getCurrentAABBTreeBounds(), *tree.tree, aabb.getCenter(), unitDir, inOutDistance, aabb.getExtents(), pcb);
		}
	}

	return again;
}

bool IncrementalAABBPrunerCore::raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback& pcbArgName) const
{
	bool again = true;
	RaycastCallbackAdapter pcb(pcbArgName, *mPool);

	for(PxU32 i = 0; i < NUM_TREES; i++)
	{
		const CoreTree& tree = mAABBTree[i];
		if(tree.tree && tree.tree->getNodes() && again)
		{
			again = AABBTreeRaycast<false, true, IncrementalAABBTree, IncrementalAABBTreeNode, RaycastCallbackAdapter>()(mPool->getCurrentAABBTreeBounds(), *tree.tree, origin, unitDir, inOutDistance, PxVec3(0.0f), pcb);
		}
	}
	return again;
}

void IncrementalAABBPrunerCore::getGlobalBounds(PxBounds3& bounds)	const
{
	bounds.setEmpty();

	// PT: TODO: optimize this
	for(PxU32 i=0; i<NUM_TREES; i++)
	{
		const CoreTree& tree = mAABBTree[i];
		if(tree.tree && tree.tree->getNodes())
		{
			PxBounds3 tmp;
			StoreBounds(tmp, tree.tree->getNodes()->mBVMin, tree.tree->getNodes()->mBVMax);
			bounds.include(tmp);
		}
	}
}

void IncrementalAABBPrunerCore::shiftOrigin(const PxVec3& shift)
{
	for(PxU32 i = 0; i < NUM_TREES; i++)
	{
		if(mAABBTree[i].tree)
		{
			mAABBTree[i].tree->shiftOrigin(shift);
		}
	}
}

void IncrementalAABBPrunerCore::visualize(PxRenderOutput& out, PxU32 color) const
{
	for(PxU32 i = 0; i < NUM_TREES; i++)
	{
		visualizeTree(out, color, mAABBTree[i].tree);

		// Render added objects not yet in the tree
		//out << PxTransform(PxIdentity);
		//out << PxU32(PxDebugColor::eARGB_WHITE);
	}
}

void IncrementalAABBPrunerCore::test(bool hierarchyCheck)
{
	PxU32 maxDepth[NUM_TREES] = { 0, 0 };
	for(PxU32 i=0; i<NUM_TREES; i++)
	{		
		if(mAABBTree[i].tree)
		{
			if(hierarchyCheck)
				mAABBTree[i].tree->hierarchyCheck(mPool->getCurrentWorldBoxes());
			for(IncrementalPrunerMap::Iterator iter = mAABBTree[i].mapping.getIterator(); !iter.done(); ++iter)
			{
				mAABBTree[i].tree->checkTreeLeaf(iter->second, iter->first);
				const PxU32 depth = mAABBTree[i].tree->getTreeLeafDepth(iter->second);
				if(depth > maxDepth[i])
					maxDepth[i] = depth;
			}
		}
	}
}
