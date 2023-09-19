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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "foundation/PxAllocator.h"
#include "SqCompoundPruningPool.h"
#include "GuPruningPool.h"
#include "GuAABBTree.h"
#include "GuBVH.h"

using namespace physx;
using namespace Cm;
using namespace Gu;
using namespace Sq;

///////////////////////////////////////////////////////////////////////////////////////////////

void CompoundTree::updateObjectAfterManualBoundsUpdates(PrunerHandle handle)
{
	const PxBounds3* newBounds = mPruningPool->getCurrentWorldBoxes();	
	const PoolIndex poolIndex = mPruningPool->getIndex(handle);
	NodeList changedLeaves;
	changedLeaves.reserve(8);
	IncrementalAABBTreeNode* node = mTree->update((*mUpdateMap)[poolIndex], poolIndex, newBounds, changedLeaves);
	// we removed node during update, need to update the mapping	
	updateMapping(poolIndex, node, changedLeaves);
}

///////////////////////////////////////////////////////////////////////////////////////////////

void CompoundTree::removeObject(PrunerHandle handle, PrunerPayloadRemovalCallback* removalCallback)
{
	const PoolIndex poolIndex = mPruningPool->getIndex(handle); // save the pool index for removed object
	const PoolIndex poolRelocatedLastIndex = mPruningPool->removeObject(handle, removalCallback); // save the lastIndex returned by removeObject

	IncrementalAABBTreeNode* node = mTree->remove((*mUpdateMap)[poolIndex], poolIndex, mPruningPool->getCurrentWorldBoxes());
	// if node moved to its parent
	if (node && node->isLeaf())
	{
		for (PxU32 j = 0; j < node->getNbPrimitives(); j++)
		{
			const PoolIndex index = node->getPrimitives(NULL)[j];
			(*mUpdateMap)[index] = node;
		}
	}

	(*mUpdateMap)[poolIndex] = (*mUpdateMap)[poolRelocatedLastIndex];
	// fix indices if we made a swap
	if(poolRelocatedLastIndex != poolIndex)
		mTree->fixupTreeIndices((*mUpdateMap)[poolIndex], poolRelocatedLastIndex, poolIndex);
}

///////////////////////////////////////////////////////////////////////////////////////////////

bool CompoundTree::addObject(PrunerHandle& result, const PxBounds3& bounds, const PrunerPayload& data, const PxTransform& transform)
{
	mPruningPool->addObjects(&result, &bounds, &data, &transform, 1);
	if (mPruningPool->mMaxNbObjects > mUpdateMap->size())
		mUpdateMap->resize(mPruningPool->mMaxNbObjects);
	
	const PoolIndex poolIndex = mPruningPool->getIndex(result);
	NodeList changedLeaves;
	changedLeaves.reserve(8);
	IncrementalAABBTreeNode* node = mTree->insert(poolIndex, mPruningPool->getCurrentWorldBoxes(), changedLeaves);
	updateMapping(poolIndex, node, changedLeaves);

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////

void CompoundTree::updateMapping(const PoolIndex poolIndex, IncrementalAABBTreeNode* node, const NodeList& changedLeaves)
{
	// if a node was split we need to update the node indices and also the sibling indices
	if(!changedLeaves.empty())
	{
		if(node && node->isLeaf())
		{
			for(PxU32 j = 0; j < node->getNbPrimitives(); j++)
			{
				const PoolIndex index = node->getPrimitives(NULL)[j];
				(*mUpdateMap)[index] = node;
			}
		}

		for(PxU32 i = 0; i < changedLeaves.size(); i++)
		{
			IncrementalAABBTreeNode* changedNode = changedLeaves[i];
			PX_ASSERT(changedNode->isLeaf());

			for(PxU32 j = 0; j < changedNode->getNbPrimitives(); j++)
			{
				const PoolIndex index = changedNode->getPrimitives(NULL)[j];
				(*mUpdateMap)[index] = changedNode;
			}
		}
	}
	else
	{
		(*mUpdateMap)[poolIndex] = node;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////

CompoundTreePool::CompoundTreePool(PxU64 contextID) :
	mNbObjects		(0),
	mMaxNbObjects	(0),
	mCompoundTrees	(NULL),
	mContextID		(contextID)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////

CompoundTreePool::~CompoundTreePool()
{
	PX_FREE(mCompoundTrees);
}

///////////////////////////////////////////////////////////////////////////////////////////////

bool CompoundTreePool::resize(PxU32 newCapacity)
{
	mCompoundBounds.resize(newCapacity, mNbObjects);

	CompoundTree* newTrees = PX_ALLOCATE(CompoundTree, newCapacity, "IncrementalTrees*");
	if(!newTrees)
		return false;

	// memzero, we need to set the pointers in the compound tree to NULL
	PxMemZero(newTrees, sizeof(CompoundTree)*newCapacity);
	if(mCompoundTrees)
		PxMemCopy(newTrees, mCompoundTrees, mNbObjects*sizeof(CompoundTree));
	mMaxNbObjects = newCapacity;
	PX_FREE(mCompoundTrees);
	mCompoundTrees	= newTrees;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////

void CompoundTreePool::preallocate(PxU32 newCapacity)
{
	if(newCapacity>mMaxNbObjects)
		resize(newCapacity);
}

///////////////////////////////////////////////////////////////////////////////////////////////

void CompoundTreePool::shiftOrigin(const PxVec3& shift)
{
	PxBounds3* bounds = mCompoundBounds.getBounds();
	for(PxU32 i=0; i < mNbObjects; i++)
	{
		bounds[i].minimum -= shift;
		bounds[i].maximum -= shift;

		mCompoundTrees[i].mGlobalPose.p -= shift;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////

PoolIndex CompoundTreePool::addCompound(PrunerHandle* results, const BVH& bvh, const PxBounds3& compoundBounds, const PxTransform& transform, bool isDynamic, const PrunerPayload* data, const PxTransform* transforms)
{
	if(mNbObjects==mMaxNbObjects) // increase the capacity on overflow
	{
		if(!resize(PxMax<PxU32>(mMaxNbObjects*2, 32)))
		{
			// pool can return an invalid handle if memory alloc fails			
			PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "CompoundTreePool::addCompound memory allocation in resize failed.");
			return INVALID_PRUNERHANDLE;
		}
	}
	PX_ASSERT(mNbObjects!=mMaxNbObjects);

	const PoolIndex index = mNbObjects++;
		
	mCompoundBounds.getBounds()[index] = compoundBounds;

	const PxU32 nbObjects = bvh.getNbBounds();

	CompoundTree& tree = mCompoundTrees[index];
	PX_ASSERT(tree.mPruningPool == NULL);
	PX_ASSERT(tree.mTree == NULL);
	PX_ASSERT(tree.mUpdateMap == NULL);

	tree.mGlobalPose = transform;
	tree.mFlags = isDynamic ? PxCompoundPrunerQueryFlag::eDYNAMIC : PxCompoundPrunerQueryFlag::eSTATIC;

	// prepare the pruning pool
	PruningPool* pool = PX_NEW(PruningPool)(mContextID, TRANSFORM_CACHE_LOCAL);
	pool->preallocate(nbObjects);
	pool->addObjects(results, bvh.getBounds(), data, transforms, nbObjects);
	tree.mPruningPool = pool;

	// prepare update map
    UpdateMap* map = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(UpdateMap), "Update map"), UpdateMap);
	map->resizeUninitialized(nbObjects);
	tree.mUpdateMap = map;

	IncrementalAABBTree* iTree = PX_NEW(IncrementalAABBTree);
	iTree->copy(bvh, *map);
	tree.mTree = iTree;

	return index;
}

///////////////////////////////////////////////////////////////////////////////////////////////

PoolIndex CompoundTreePool::removeCompound(PoolIndex indexOfRemovedObject, PrunerPayloadRemovalCallback* removalCallback)
{
	PX_ASSERT(mNbObjects);

	// release the tree
	PX_DELETE(mCompoundTrees[indexOfRemovedObject].mTree);

	mCompoundTrees[indexOfRemovedObject].mUpdateMap->clear();
	mCompoundTrees[indexOfRemovedObject].mUpdateMap->~PxArray();
	PX_FREE(mCompoundTrees[indexOfRemovedObject].mUpdateMap);

	if(removalCallback)
	{
		const PruningPool* pool = mCompoundTrees[indexOfRemovedObject].mPruningPool;
		removalCallback->invoke(pool->getNbActiveObjects(), pool->getObjects());
	}

	PX_DELETE(mCompoundTrees[indexOfRemovedObject].mPruningPool);

	const PoolIndex indexOfLastObject = --mNbObjects; // swap the object at last index with index
	if(indexOfLastObject!=indexOfRemovedObject)
	{
		// PT: move last object's data to recycled spot (from removed object)

		// PT: the last object has moved so we need to handle the mappings for this object
		mCompoundBounds.getBounds()	[indexOfRemovedObject]	= mCompoundBounds.getBounds()	[indexOfLastObject];
		mCompoundTrees				[indexOfRemovedObject]	= mCompoundTrees				[indexOfLastObject];

		mCompoundTrees	[indexOfLastObject].mPruningPool = NULL;
		mCompoundTrees	[indexOfLastObject].mUpdateMap = NULL;
		mCompoundTrees	[indexOfLastObject].mTree = NULL;
	}

	return indexOfLastObject;
}

