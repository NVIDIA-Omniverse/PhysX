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

// PT: TODO: this class isn't actually used at the moment
#define COMPILE_INCREMENTAL_AABB_PRUNER
#ifdef COMPILE_INCREMENTAL_AABB_PRUNER

#include "common/PxProfileZone.h"
#include "CmVisualization.h"
#include "foundation/PxBitUtils.h"
#include "GuIncrementalAABBPruner.h"
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

// PT: TODO: this is copied from SqBounds.h, should be either moved to Gu and shared or passed as a user parameter
	#define SQ_PRUNER_EPSILON	0.005f
	#define SQ_PRUNER_INFLATION	(1.0f + SQ_PRUNER_EPSILON)	// pruner test shape inflation (not narrow phase shape)

#define PARANOIA_CHECKS 0

IncrementalAABBPruner::IncrementalAABBPruner(PxU32 sceneLimit, PxU64 contextID) :
	mAABBTree	(NULL),
	mPool		(contextID, TRANSFORM_CACHE_GLOBAL),
	mContextID	(contextID)
{
	mMapping.resizeUninitialized(sceneLimit);
	mPool.preallocate(sceneLimit);

	mChangedLeaves.reserve(sceneLimit);
}

IncrementalAABBPruner::~IncrementalAABBPruner()
{
	release();
}

bool IncrementalAABBPruner::addObjects(PrunerHandle* results, const PxBounds3* bounds, const PrunerPayload* data, const PxTransform* transforms, PxU32 count, bool )
{
	PX_PROFILE_ZONE("SceneQuery.prunerAddObjects", mContextID);

	if(!count)
		return true;
	
	const PxU32 valid = mPool.addObjects(results, bounds, data, transforms, count);

	if(mAABBTree)
	{
		for(PxU32 i=0;i<valid;i++)
		{
			const PrunerHandle& handle = results[i];
			const PoolIndex poolIndex = mPool.getIndex(handle);
			mChangedLeaves.clear();
			IncrementalAABBTreeNode* node = mAABBTree->insert(poolIndex, mPool.getCurrentWorldBoxes(), mChangedLeaves);
			updateMapping(poolIndex, node);
		}

	#if PARANOIA_CHECKS
		test();
	#endif
	}

	return valid==count;
}

void IncrementalAABBPruner::updateMapping(const PoolIndex poolIndex, IncrementalAABBTreeNode* node)
{
	// resize mapping if needed
	if(mMapping.size() <= poolIndex)
	{
		mMapping.resize(mMapping.size() * 2);
	}

	// if a node was split we need to update the node indices and also the sibling indices
	if(!mChangedLeaves.empty())
	{
		if(node && node->isLeaf())
		{
			for(PxU32 j = 0; j < node->getNbPrimitives(); j++)
			{
				mMapping[node->getPrimitives(NULL)[j]] = node;
			}
		}

		for(PxU32 i = 0; i < mChangedLeaves.size(); i++)
		{
			IncrementalAABBTreeNode* changedNode = mChangedLeaves[i];
			PX_ASSERT(changedNode->isLeaf());

			for(PxU32 j = 0; j < changedNode->getNbPrimitives(); j++)
			{
				mMapping[changedNode->getPrimitives(NULL)[j]] = changedNode;
			}
		}
	}
	else
	{
		mMapping[poolIndex] = node;
	}
}

void IncrementalAABBPruner::updateObjects(const PrunerHandle* handles, PxU32 count, float inflation, const PxU32* boundsIndices, const PxBounds3* newBounds, const PxTransform32* newTransforms)
{
	PX_PROFILE_ZONE("SceneQuery.prunerUpdateObjects", mContextID);

	if(!count)
		return;
	
	if(handles && boundsIndices && newBounds)
		mPool.updateAndInflateBounds(handles, boundsIndices, newBounds, newTransforms, count, inflation);

	if(!mAABBTree)
		return;

	const PxBounds3* poolBounds = mPool.getCurrentWorldBoxes();	
	for(PxU32 i=0; i<count; i++)
	{
		const PrunerHandle h = handles[i];
		const PoolIndex poolIndex = mPool.getIndex(h);
		mChangedLeaves.clear();
		IncrementalAABBTreeNode* node = mAABBTree->update(mMapping[poolIndex], poolIndex, poolBounds, mChangedLeaves);
		// we removed node during update, need to update the mapping
		updateMapping(poolIndex, node);
	}

#if PARANOIA_CHECKS
	test();
#endif
}

void IncrementalAABBPruner::removeObjects(const PrunerHandle* handles, PxU32 count, PrunerPayloadRemovalCallback* removalCallback)
{
	PX_PROFILE_ZONE("SceneQuery.prunerRemoveObjects", mContextID);

	if(!count)
		return;

	for(PxU32 i=0; i<count; i++)
	{
		const PrunerHandle h = handles[i];		
		const PoolIndex poolIndex = mPool.getIndex(h); // save the pool index for removed object
		const PoolIndex poolRelocatedLastIndex = mPool.removeObject(h, removalCallback); // save the lastIndex returned by removeObject

		if(mAABBTree)
		{
			IncrementalAABBTreeNode* node = mAABBTree->remove(mMapping[poolIndex], poolIndex, mPool.getCurrentWorldBoxes());
			// if node moved to its parent
			if (node && node->isLeaf())
			{
				for (PxU32 j = 0; j < node->getNbPrimitives(); j++)
				{
					const PoolIndex index = node->getPrimitives(NULL)[j];
					mMapping[index] = node;
				}
			}
			mMapping[poolIndex] = mMapping[poolRelocatedLastIndex];
			// fix indices if we made a swap
			if(poolRelocatedLastIndex != poolIndex)
				mAABBTree->fixupTreeIndices(mMapping[poolIndex], poolRelocatedLastIndex, poolIndex);

			if(!mAABBTree->getNodes())
			{
				release();
			}
		}
	}

#if PARANOIA_CHECKS
	test();
#endif
}

bool IncrementalAABBPruner::overlap(const ShapeData& queryVolume, PrunerOverlapCallback& pcbArgName) const
{
	bool again = true;

	if(mAABBTree && mAABBTree->getNodes())
	{
		OverlapCallbackAdapter pcb(pcbArgName, mPool);

		switch(queryVolume.getType())
		{
			case PxGeometryType::eBOX:
			{
				if(queryVolume.isOBB())
				{	
					const DefaultOBBAABBTest test(queryVolume);
					again = AABBTreeOverlap<true, OBBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, OverlapCallbackAdapter>()(mPool.getCurrentAABBTreeBounds(), *mAABBTree, test, pcb);
				}
				else
				{
					const DefaultAABBAABBTest test(queryVolume);
					again = AABBTreeOverlap<true, AABBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, OverlapCallbackAdapter>()(mPool.getCurrentAABBTreeBounds(), *mAABBTree, test, pcb);
				}
			}
			break;
			case PxGeometryType::eCAPSULE:
			{
				const DefaultCapsuleAABBTest test(queryVolume, SQ_PRUNER_INFLATION);
				again = AABBTreeOverlap<true, CapsuleAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, OverlapCallbackAdapter>()(mPool.getCurrentAABBTreeBounds(), *mAABBTree, test, pcb);
			}
			break;
			case PxGeometryType::eSPHERE:
			{
				const DefaultSphereAABBTest test(queryVolume);
				again = AABBTreeOverlap<true, SphereAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, OverlapCallbackAdapter>()(mPool.getCurrentAABBTreeBounds(), *mAABBTree, test, pcb);
			}
			break;
			case PxGeometryType::eCONVEXMESH:
			{
				const DefaultOBBAABBTest test(queryVolume);
				again = AABBTreeOverlap<true, OBBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, OverlapCallbackAdapter>()(mPool.getCurrentAABBTreeBounds(), *mAABBTree, test, pcb);			
			}
			break;
		default:
			PX_ALWAYS_ASSERT_MESSAGE("unsupported overlap query volume geometry type");
		}
	}

	return again;
}

bool IncrementalAABBPruner::sweep(const ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback& pcbArgName) const
{
	bool again = true;

	if(mAABBTree && mAABBTree->getNodes())
	{
		const PxBounds3& aabb = queryVolume.getPrunerInflatedWorldAABB();
		RaycastCallbackAdapter pcb(pcbArgName, mPool);
		again = AABBTreeRaycast<true, true, IncrementalAABBTree, IncrementalAABBTreeNode, RaycastCallbackAdapter>()(mPool.getCurrentAABBTreeBounds(), *mAABBTree, aabb.getCenter(), unitDir, inOutDistance, aabb.getExtents(), pcb);
	}

	return again;
}

bool IncrementalAABBPruner::raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback& pcbArgName) const
{
	bool again = true;

	if(mAABBTree && mAABBTree->getNodes())
	{
		RaycastCallbackAdapter pcb(pcbArgName, mPool);
		again = AABBTreeRaycast<false, true, IncrementalAABBTree, IncrementalAABBTreeNode, RaycastCallbackAdapter>()(mPool.getCurrentAABBTreeBounds(), *mAABBTree, origin, unitDir, inOutDistance, PxVec3(0.0f), pcb);
	}
		
	return again;
}

// This isn't part of the pruner virtual interface, but it is part of the public interface
// of AABBPruner - it gets called by SqManager to force a rebuild, and requires a commit() before 
// queries can take place

void IncrementalAABBPruner::purge()
{
	release();	
} 

// Commit either performs a refit if background rebuild is not yet finished
// or swaps the current tree for the second tree rebuilt in the background
void IncrementalAABBPruner::commit()
{
	PX_PROFILE_ZONE("SceneQuery.prunerCommit", mContextID);

	if (!mAABBTree)
	{
		fullRebuildAABBTree();
		return;
	}
}

void IncrementalAABBPruner::fullRebuildAABBTree()
{
	// Don't bother building an AABB-tree if there isn't a single static object
	const PxU32 nbObjects = mPool.getNbActiveObjects();
	if (!nbObjects)
		return;

	const PxU32 indicesSize = PxNextPowerOfTwo(nbObjects);
	if(indicesSize > mMapping.size())
	{
		mMapping.resizeUninitialized(indicesSize);
	}
	
	// copy the temp optimized tree into the new incremental tree
	mAABBTree = PX_NEW(IncrementalAABBTree)();

	mAABBTree->build(AABBTreeBuildParams(INCR_NB_OBJECTS_PER_NODE, nbObjects, &mPool.getCurrentAABBTreeBounds()), mMapping);

#if PARANOIA_CHECKS
	test();
#endif
}

void IncrementalAABBPruner::shiftOrigin(const PxVec3& shift)
{
	mPool.shiftOrigin(shift);

	if(mAABBTree)
		mAABBTree->shiftOrigin(shift);
}

void IncrementalAABBPruner::visualize(PxRenderOutput& out, PxU32 primaryColor, PxU32 /*secondaryColor*/) const
{
	// getAABBTree() asserts when pruner is dirty. NpScene::visualization() does not enforce flushUpdate. see DE7834
	visualizeTree(out, primaryColor, mAABBTree);

	// Render added objects not yet in the tree
	//out << PxTransform(PxIdentity);
	//out << PxU32(PxDebugColor::eARGB_WHITE);
}

void IncrementalAABBPruner::release() // this can be called from purge()
{
	PX_DELETE(mAABBTree);
}

void IncrementalAABBPruner::test()
{
	if(mAABBTree)
	{
		mAABBTree->hierarchyCheck(mPool.getNbActiveObjects(), mPool.getCurrentWorldBoxes());
		for(PxU32 i = 0; i < mPool.getNbActiveObjects(); i++)
		{
			mAABBTree->checkTreeLeaf(mMapping[i], i);
		}
	}
}

void IncrementalAABBPruner::merge(const void* )
{
	//const AABBPrunerMergeData& pruningStructure = *reinterpret_cast<const AABBPrunerMergeData*> (mergeParams);

	//if(mAABBTree)
	//{
	//	// index in pruning pool, where new objects were added
	//	const PxU32 pruningPoolIndex = mPool.getNbActiveObjects() - pruningStructure.mNbObjects;

	//	// create tree from given nodes and indices
	//	AABBTreeMergeData aabbTreeMergeParams(pruningStructure.mNbNodes, pruningStructure.mAABBTreeNodes,
	//		pruningStructure.mNbObjects, pruningStructure.mAABBTreeIndices, pruningPoolIndex);

	//	if (!mIncrementalRebuild)
	//	{
	//		// merge tree directly
	//		mAABBTree->mergeTree(aabbTreeMergeParams);		
	//	}
	//	else
	//	{
	//		mBucketPruner.addTree(aabbTreeMergeParams, mTimeStamp);
	//	}
	//}
}

void IncrementalAABBPruner::getGlobalBounds(PxBounds3& bounds) const
{
	if(mAABBTree && mAABBTree->getNodes())
	{
		StoreBounds(bounds, mAABBTree->getNodes()->mBVMin, mAABBTree->getNodes()->mBVMax);
	}
	else
		bounds.setEmpty();
}

#endif
