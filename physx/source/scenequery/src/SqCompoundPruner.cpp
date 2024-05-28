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

#include "SqCompoundPruner.h"
#include "GuSqInternal.h"
#include "GuIncrementalAABBTree.h"
#include "GuPruningPool.h"
#include "GuAABBTreeQuery.h"
#include "GuAABBTreeNode.h"
#include "GuSphere.h"
#include "GuBox.h"
#include "GuCapsule.h"
#include "GuBVH.h"
#include "GuQuery.h"
#include "GuInternal.h"
#include "common/PxRenderBuffer.h"
#include "common/PxRenderOutput.h"
#include "CmVisualization.h"

using namespace physx;
using namespace Gu;
using namespace Sq;

// PT: TODO: this is copied from SqBounds.h, should be either moved to Gu and shared or passed as a user parameter
	#define SQ_PRUNER_EPSILON	0.005f
	#define SQ_PRUNER_INFLATION	(1.0f + SQ_PRUNER_EPSILON)	// pruner test shape inflation (not narrow phase shape)

#define PARANOIA_CHECKS 0

///////////////////////////////////////////////////////////////////////////////////////////////

BVHCompoundPruner::BVHCompoundPruner(PxU64 contextID) : mCompoundTreePool(contextID), mDrawStatic(false), mDrawDynamic(false)
{
	preallocate(32);
}

///////////////////////////////////////////////////////////////////////////////////////////////

BVHCompoundPruner::~BVHCompoundPruner()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////

bool BVHCompoundPruner::addCompound(PrunerHandle* results, const BVH& bvh, PrunerCompoundId compoundId, const PxTransform& transform, bool isDynamic, const PrunerPayload* data, const PxTransform* transforms)
{
	PX_ASSERT(bvh.getNbBounds());
	
	const PxBounds3 compoundBounds = PxBounds3::transformFast(transform, bvh.getNodes()->mBV);
	const PoolIndex poolIndex = mCompoundTreePool.addCompound(results, bvh, compoundBounds, transform, isDynamic, data, transforms);

	mChangedLeaves.clear();
	IncrementalAABBTreeNode* node = mMainTree.insert(poolIndex, mCompoundTreePool.getCurrentCompoundBounds(), mChangedLeaves);
	updateMapping(poolIndex, node);

	mActorPoolMap[compoundId] = poolIndex;
	mPoolActorMap[poolIndex] = compoundId;

#if PARANOIA_CHECKS
	test();
#endif
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::updateMapping(const PoolIndex poolIndex, IncrementalAABBTreeNode* node)
{
	// resize mapping if needed
	if(mMainTreeUpdateMap.size() <= poolIndex)
	{
		const PxU32 resizeSize = mMainTreeUpdateMap.size() * 2;
		mMainTreeUpdateMap.resize(resizeSize);
		mPoolActorMap.resize(resizeSize);
	}

	// if a node was split we need to update the node indices and also the sibling indices
	if(!mChangedLeaves.empty())
	{
		if(node && node->isLeaf())
		{
			for(PxU32 j = 0; j < node->getNbPrimitives(); j++)
			{
				mMainTreeUpdateMap[node->getPrimitives(NULL)[j]] = node;
			}
		}

		for(PxU32 i = 0; i < mChangedLeaves.size(); i++)
		{
			IncrementalAABBTreeNode* changedNode = mChangedLeaves[i];
			PX_ASSERT(changedNode->isLeaf());

			for(PxU32 j = 0; j < changedNode->getNbPrimitives(); j++)
			{
				mMainTreeUpdateMap[changedNode->getPrimitives(NULL)[j]] = changedNode;
			}
		}
	}
	else
	{
		mMainTreeUpdateMap[poolIndex] = node;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////

bool BVHCompoundPruner::removeCompound(PrunerCompoundId compoundId, PrunerPayloadRemovalCallback* removalCallback)
{
	const ActorIdPoolIndexMap::Entry* poolIndexEntry = mActorPoolMap.find(compoundId);
	PX_ASSERT(poolIndexEntry);

	bool isDynamic = false;

	if(poolIndexEntry)
	{
		const PoolIndex poolIndex = poolIndexEntry->second;

		CompoundTree& compoundTree = mCompoundTreePool.getCompoundTrees()[poolIndex];
		isDynamic = compoundTree.mFlags & PxCompoundPrunerQueryFlag::eDYNAMIC;

		const PoolIndex poolRelocatedLastIndex = mCompoundTreePool.removeCompound(poolIndex, removalCallback);

		IncrementalAABBTreeNode* node = mMainTree.remove(mMainTreeUpdateMap[poolIndex], poolIndex, mCompoundTreePool.getCurrentCompoundBounds());
		// if node moved to its parent
		if(node && node->isLeaf())
		{
			for (PxU32 j = 0; j < node->getNbPrimitives(); j++)
			{
				const PoolIndex index = node->getPrimitives(NULL)[j];
				mMainTreeUpdateMap[index] = node;
			}
		}	

		// fix indices if we made a swap
		if(poolRelocatedLastIndex != poolIndex)
		{
			mMainTreeUpdateMap[poolIndex] = mMainTreeUpdateMap[poolRelocatedLastIndex];
			mMainTree.fixupTreeIndices(mMainTreeUpdateMap[poolIndex], poolRelocatedLastIndex, poolIndex);

			mActorPoolMap[mPoolActorMap[poolRelocatedLastIndex]] = poolIndex;
			mPoolActorMap[poolIndex] = mPoolActorMap[poolRelocatedLastIndex];
		}

		mActorPoolMap.erase(compoundId);
	}

#if PARANOIA_CHECKS
	test();
#endif

	return isDynamic;
}

///////////////////////////////////////////////////////////////////////////////////////////////

bool BVHCompoundPruner::updateCompound(PrunerCompoundId compoundId, const PxTransform& transform)
{
	const ActorIdPoolIndexMap::Entry* poolIndexEntry = mActorPoolMap.find(compoundId);
	PX_ASSERT(poolIndexEntry);

	bool isDynamic = false;

	if(poolIndexEntry)
	{
		const PxU32 poolIndex = poolIndexEntry->second;

		CompoundTree& compoundTree = mCompoundTreePool.getCompoundTrees()[poolIndex];
		isDynamic = compoundTree.mFlags & PxCompoundPrunerQueryFlag::eDYNAMIC;

		compoundTree.mGlobalPose = transform;

		PxBounds3 localBounds;
		const IncrementalAABBTreeNode* node = compoundTree.mTree->getNodes();
		V4StoreU(node->mBVMin, &localBounds.minimum.x);
		PX_ALIGN(16, PxVec4) max4;
		V4StoreA(node->mBVMax, &max4.x);
		localBounds.maximum = PxVec3(max4.x, max4.y, max4.z);

		const PxBounds3 compoundBounds = PxBounds3::transformFast(transform, localBounds);
		mCompoundTreePool.getCurrentCompoundBounds()[poolIndex] = compoundBounds;
		mChangedLeaves.clear();
		IncrementalAABBTreeNode* mainTreeNode = mMainTree.update(mMainTreeUpdateMap[poolIndex], poolIndex, mCompoundTreePool.getCurrentCompoundBounds(), mChangedLeaves);
		// we removed node during update, need to update the mapping
		updateMapping(poolIndex, mainTreeNode);
	}

#if PARANOIA_CHECKS
	test();
#endif

	return isDynamic;
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::test()
{
	if(mMainTree.getNodes())
	{
		for(PxU32 i = 0; i < mCompoundTreePool.getNbObjects(); i++)
		{
			mMainTree.checkTreeLeaf(mMainTreeUpdateMap[i], i);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::release()
{
}

//////////////////////////////////////////////////////////////////////////
// Queries implementation
//////////////////////////////////////////////////////////////////////////

namespace
{
	struct CompoundCallbackRaycastAdapter
	{
		PX_FORCE_INLINE	CompoundCallbackRaycastAdapter(CompoundPrunerRaycastCallback& pcb, const CompoundTree& tree) : mCallback(pcb), mTree(tree)	{}

		PX_FORCE_INLINE bool	invoke(PxReal& distance, PxU32 primIndex)
		{
			return mCallback.invoke(distance, primIndex, mTree.mPruningPool->getObjects(), mTree.mPruningPool->getTransforms(), &mTree.mGlobalPose);
		}

		CompoundPrunerRaycastCallback&	mCallback;
		const CompoundTree&				mTree;
		PX_NOCOPY(CompoundCallbackRaycastAdapter)
	};

	struct CompoundCallbackOverlapAdapter
	{
		PX_FORCE_INLINE	CompoundCallbackOverlapAdapter(CompoundPrunerOverlapCallback& pcb, const CompoundTree& tree) : mCallback(pcb), mTree(tree)	{}

		PX_FORCE_INLINE bool	invoke(PxU32 primIndex)
		{
			return mCallback.invoke(primIndex, mTree.mPruningPool->getObjects(), mTree.mPruningPool->getTransforms(), &mTree.mGlobalPose);
		}

		CompoundPrunerOverlapCallback&	mCallback;
		const CompoundTree&				mTree;
		PX_NOCOPY(CompoundCallbackOverlapAdapter)
	};
}

template<class PrunerCallback>
struct MainTreeCompoundPrunerCallback
{
	MainTreeCompoundPrunerCallback(PrunerCallback& prunerCallback, PxCompoundPrunerQueryFlags flags, const CompoundTree* compoundTrees)
		: mPrunerCallback(prunerCallback), mQueryFlags(flags), mCompoundTrees(compoundTrees)
	{
	}

	virtual ~MainTreeCompoundPrunerCallback() {}

	PX_FORCE_INLINE bool filtering(const CompoundTree& compoundTree)	const
	{
		if(!(compoundTree.mFlags & mQueryFlags) || !compoundTree.mTree->getNodes())
			return true;
		return false;
	}

	protected:
	PrunerCallback&						mPrunerCallback;
	const PxCompoundPrunerQueryFlags	mQueryFlags;
	const CompoundTree*					mCompoundTrees;

	PX_NOCOPY(MainTreeCompoundPrunerCallback)
};

// Raycast/sweeps callback for main AABB tree
template<bool tInflate>
struct MainTreeRaycastCompoundPrunerCallback : MainTreeCompoundPrunerCallback<CompoundPrunerRaycastCallback>
{
	MainTreeRaycastCompoundPrunerCallback(const PxVec3& origin, const PxVec3& unitDir, const PxVec3& extent, CompoundPrunerRaycastCallback& prunerCallback, PxCompoundPrunerQueryFlags flags, const CompoundTree* compoundTrees)
		: MainTreeCompoundPrunerCallback(prunerCallback, flags, compoundTrees), mOrigin(origin), mUnitDir(unitDir), mExtent(extent)
	{
	}

	virtual ~MainTreeRaycastCompoundPrunerCallback() {}

	bool invoke(PxReal& distance, PxU32 primIndex)
	{
		const CompoundTree& compoundTree = mCompoundTrees[primIndex];

		if(filtering(compoundTree))
			return true;

		// transfer to actor local space
		const PxVec3 localOrigin = compoundTree.mGlobalPose.transformInv(mOrigin);
		const PxVec3 localDir = compoundTree.mGlobalPose.q.rotateInv(mUnitDir);
		PxVec3 localExtent = mExtent;

		if(tInflate)
		{
			const PxBounds3 wBounds = PxBounds3::centerExtents(mOrigin, mExtent);
			const PxBounds3 localBounds = PxBounds3::transformSafe(compoundTree.mGlobalPose.getInverse(), wBounds);
			localExtent = localBounds.getExtents();
		}

		// raycast the merged tree
		CompoundCallbackRaycastAdapter pcb(mPrunerCallback, compoundTree);
		return AABBTreeRaycast<tInflate, true, IncrementalAABBTree, IncrementalAABBTreeNode, CompoundCallbackRaycastAdapter>()
			(compoundTree.mPruningPool->getCurrentAABBTreeBounds(), *compoundTree.mTree, localOrigin, localDir, distance, localExtent, pcb);
	}

	PX_NOCOPY(MainTreeRaycastCompoundPrunerCallback)

private:
	const PxVec3&	mOrigin;
	const PxVec3&	mUnitDir;	
	const PxVec3&	mExtent;
};

//////////////////////////////////////////////////////////////////////////
// raycast against the compound pruner
bool BVHCompoundPruner::raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, CompoundPrunerRaycastCallback& prunerCallback, PxCompoundPrunerQueryFlags flags) const
{
	bool again = true;	

	// search the main tree if there are nodes
	if(mMainTree.getNodes())
	{
		const PxVec3 extent(0.0f);
		// main tree callback
		MainTreeRaycastCompoundPrunerCallback<false> pcb(origin, unitDir, extent, prunerCallback, flags, mCompoundTreePool.getCompoundTrees());
		// traverse the main tree
		again = AABBTreeRaycast<false, true, IncrementalAABBTree, IncrementalAABBTreeNode, MainTreeRaycastCompoundPrunerCallback<false> >()
			(mCompoundTreePool.getCurrentAABBTreeBounds(), mMainTree, origin, unitDir, inOutDistance, extent, pcb);
	}

	return again;
}

//////////////////////////////////////////////////////////////////////////
// overlap main tree callback
// A.B. templated version is complicated due to test transformations, will do a callback per primitive
struct MainTreeOverlapCompoundPrunerCallback : MainTreeCompoundPrunerCallback<CompoundPrunerOverlapCallback>
{
	MainTreeOverlapCompoundPrunerCallback(const ShapeData& queryVolume, CompoundPrunerOverlapCallback& prunerCallback, PxCompoundPrunerQueryFlags flags, const CompoundTree* compoundTrees)
		: MainTreeCompoundPrunerCallback(prunerCallback, flags, compoundTrees), mQueryVolume(queryVolume)
	{
	}

	virtual ~MainTreeOverlapCompoundPrunerCallback() {}

	PX_NOCOPY(MainTreeOverlapCompoundPrunerCallback)

protected:
	const ShapeData&	mQueryVolume;	
};

// OBB
struct MainTreeOBBOverlapCompoundPrunerCallback : public MainTreeOverlapCompoundPrunerCallback
{
	MainTreeOBBOverlapCompoundPrunerCallback(const ShapeData& queryVolume, CompoundPrunerOverlapCallback& prunerCallback, PxCompoundPrunerQueryFlags flags, const CompoundTree* compoundTrees)
		: MainTreeOverlapCompoundPrunerCallback(queryVolume, prunerCallback, flags, compoundTrees) {}

	bool invoke(PxU32 primIndex)
	{
		const CompoundTree& compoundTree = mCompoundTrees[primIndex];

		if(filtering(compoundTree))
			return true;

		const PxVec3 localPos = compoundTree.mGlobalPose.transformInv(mQueryVolume.getPrunerWorldPos());
		const PxMat33 transfMat(compoundTree.mGlobalPose.q);
		const PxMat33 localRot = transfMat.getTranspose()*mQueryVolume.getPrunerWorldRot33();

		const OBBAABBTest localTest(localPos, localRot, mQueryVolume.getPrunerBoxGeomExtentsInflated());		
		// overlap the compound local tree
		CompoundCallbackOverlapAdapter pcb(mPrunerCallback, compoundTree);
		return AABBTreeOverlap<true, OBBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, CompoundCallbackOverlapAdapter>()
			(compoundTree.mPruningPool->getCurrentAABBTreeBounds(), *compoundTree.mTree, localTest, pcb);
	}

	PX_NOCOPY(MainTreeOBBOverlapCompoundPrunerCallback)
};

// AABB
struct MainTreeAABBOverlapCompoundPrunerCallback : public MainTreeOverlapCompoundPrunerCallback
{
	MainTreeAABBOverlapCompoundPrunerCallback(const ShapeData& queryVolume, CompoundPrunerOverlapCallback& prunerCallback, PxCompoundPrunerQueryFlags flags, const CompoundTree* compoundTrees)
		: MainTreeOverlapCompoundPrunerCallback(queryVolume, prunerCallback, flags, compoundTrees) {}

	bool invoke(PxU32 primIndex)
	{
		const CompoundTree& compoundTree = mCompoundTrees[primIndex];

		if(filtering(compoundTree))
			return true;

		const PxVec3 localPos = compoundTree.mGlobalPose.transformInv(mQueryVolume.getPrunerWorldPos());
		const PxMat33 transfMat(compoundTree.mGlobalPose.q);
		const PxMat33 localRot = transfMat.getTranspose()*mQueryVolume.getPrunerWorldRot33();

		// A.B. we dont have the AABB in local space, either we test OBB local space or
		// we retest the AABB with the worldSpace AABB of the local tree???
		const OBBAABBTest localTest(localPos, localRot, mQueryVolume.getPrunerBoxGeomExtentsInflated());		
		// overlap the compound local tree
		CompoundCallbackOverlapAdapter pcb(mPrunerCallback, compoundTree);
		return AABBTreeOverlap<true, OBBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, CompoundCallbackOverlapAdapter>()
			(compoundTree.mPruningPool->getCurrentAABBTreeBounds(), *compoundTree.mTree, localTest, pcb);
	}

	PX_NOCOPY(MainTreeAABBOverlapCompoundPrunerCallback)
};

// Capsule
struct MainTreeCapsuleOverlapCompoundPrunerCallback : public MainTreeOverlapCompoundPrunerCallback
{
	MainTreeCapsuleOverlapCompoundPrunerCallback(const ShapeData& queryVolume, CompoundPrunerOverlapCallback& prunerCallback, PxCompoundPrunerQueryFlags flags, const CompoundTree* compoundTrees)
		: MainTreeOverlapCompoundPrunerCallback(queryVolume, prunerCallback, flags, compoundTrees) {}

	bool invoke(PxU32 primIndex)
	{
		const CompoundTree& compoundTree = mCompoundTrees[primIndex];

		if(filtering(compoundTree))
			return true;

		const PxMat33 transfMat(compoundTree.mGlobalPose.q);
		const Capsule& capsule = mQueryVolume.getGuCapsule();
		const CapsuleAABBTest localTest(
			compoundTree.mGlobalPose.transformInv(capsule.p1), 
			transfMat.getTranspose()*mQueryVolume.getPrunerWorldRot33().column0,
			mQueryVolume.getCapsuleHalfHeight()*2.0f, PxVec3(capsule.radius*SQ_PRUNER_INFLATION));

		// overlap the compound local tree
		CompoundCallbackOverlapAdapter pcb(mPrunerCallback, compoundTree);
		return AABBTreeOverlap<true, CapsuleAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, CompoundCallbackOverlapAdapter>()
			(compoundTree.mPruningPool->getCurrentAABBTreeBounds(), *compoundTree.mTree, localTest, pcb);
	}

	PX_NOCOPY(MainTreeCapsuleOverlapCompoundPrunerCallback)
};

// Sphere
struct MainTreeSphereOverlapCompoundPrunerCallback : public MainTreeOverlapCompoundPrunerCallback
{
	MainTreeSphereOverlapCompoundPrunerCallback(const ShapeData& queryVolume, CompoundPrunerOverlapCallback& prunerCallback, PxCompoundPrunerQueryFlags flags, const CompoundTree* compoundTrees)
		: MainTreeOverlapCompoundPrunerCallback(queryVolume, prunerCallback, flags, compoundTrees) {}

	bool invoke(PxU32 primIndex)
	{
		const CompoundTree& compoundTree = mCompoundTrees[primIndex];

		if(filtering(compoundTree))
			return true;

		const Sphere& sphere = mQueryVolume.getGuSphere();
		const SphereAABBTest localTest(compoundTree.mGlobalPose.transformInv(sphere.center), sphere.radius);

		// overlap the compound local tree
		CompoundCallbackOverlapAdapter pcb(mPrunerCallback, compoundTree);
		return AABBTreeOverlap<true, SphereAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, CompoundCallbackOverlapAdapter>()
			(compoundTree.mPruningPool->getCurrentAABBTreeBounds(), *compoundTree.mTree, localTest, pcb);
	}

	PX_NOCOPY(MainTreeSphereOverlapCompoundPrunerCallback)
};


//////////////////////////////////////////////////////////////////////////
// overlap implementation
bool BVHCompoundPruner::overlap(const ShapeData& queryVolume, CompoundPrunerOverlapCallback& prunerCallback, PxCompoundPrunerQueryFlags flags) const
{
	if(!mMainTree.getNodes())
		return true;

	bool again = true;

	const Gu::AABBTreeBounds& bounds = mCompoundTreePool.getCurrentAABBTreeBounds();

	switch (queryVolume.getType())
	{
	case PxGeometryType::eBOX:
	{
		if(queryVolume.isOBB())
		{
			const DefaultOBBAABBTest test(queryVolume);
			MainTreeOBBOverlapCompoundPrunerCallback pcb(queryVolume, prunerCallback, flags, mCompoundTreePool.getCompoundTrees());
			again = AABBTreeOverlap<true, OBBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, MainTreeOBBOverlapCompoundPrunerCallback>()(bounds, mMainTree, test, pcb);
		}
		else
		{
			const DefaultAABBAABBTest test(queryVolume);
			MainTreeAABBOverlapCompoundPrunerCallback pcb(queryVolume, prunerCallback, flags, mCompoundTreePool.getCompoundTrees());
			again = AABBTreeOverlap<true, AABBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, MainTreeAABBOverlapCompoundPrunerCallback>()(bounds, mMainTree, test, pcb);
		}
	}
	break;
	case PxGeometryType::eCAPSULE:
	{
		const DefaultCapsuleAABBTest test(queryVolume, SQ_PRUNER_INFLATION);
		MainTreeCapsuleOverlapCompoundPrunerCallback pcb(queryVolume, prunerCallback, flags, mCompoundTreePool.getCompoundTrees());
		again = AABBTreeOverlap<true, CapsuleAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, MainTreeCapsuleOverlapCompoundPrunerCallback >()(bounds, mMainTree, test, pcb);
	}
	break;
	case PxGeometryType::eSPHERE:
	{
		const DefaultSphereAABBTest test(queryVolume);
		MainTreeSphereOverlapCompoundPrunerCallback pcb(queryVolume, prunerCallback, flags, mCompoundTreePool.getCompoundTrees());
		again = AABBTreeOverlap<true, SphereAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, MainTreeSphereOverlapCompoundPrunerCallback>()(bounds, mMainTree, test, pcb);
	}
	break;
	case PxGeometryType::eCONVEXMESH:
	{
		const DefaultOBBAABBTest test(queryVolume);
		MainTreeOBBOverlapCompoundPrunerCallback pcb(queryVolume, prunerCallback, flags, mCompoundTreePool.getCompoundTrees());
		again = AABBTreeOverlap<true, OBBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, MainTreeOBBOverlapCompoundPrunerCallback>()(bounds, mMainTree, test, pcb);
	}
	break;
	default:
		PX_ALWAYS_ASSERT_MESSAGE("unsupported overlap query volume geometry type");
	}

	return again;
}

///////////////////////////////////////////////////////////////////////////////////////////////

bool BVHCompoundPruner::sweep(const ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, CompoundPrunerRaycastCallback& prunerCallback, PxCompoundPrunerQueryFlags flags) const
{
	bool again = true;

	if(mMainTree.getNodes())
	{
		const PxBounds3& aabb = queryVolume.getPrunerInflatedWorldAABB();
		const PxVec3 extents = aabb.getExtents();
		const PxVec3 center = aabb.getCenter();
		MainTreeRaycastCompoundPrunerCallback<true> pcb(center, unitDir, extents, prunerCallback, flags, mCompoundTreePool.getCompoundTrees());
		again = AABBTreeRaycast<true, true, IncrementalAABBTree, IncrementalAABBTreeNode, MainTreeRaycastCompoundPrunerCallback<true> >()
			(mCompoundTreePool.getCurrentAABBTreeBounds(), mMainTree, center, unitDir, inOutDistance, extents, pcb);
	}
	return again;
}

///////////////////////////////////////////////////////////////////////////////////////////////

const PrunerPayload& BVHCompoundPruner::getPayloadData(PrunerHandle handle, PrunerCompoundId compoundId, PrunerPayloadData* data) const
{
	const ActorIdPoolIndexMap::Entry* poolIndexEntry = mActorPoolMap.find(compoundId);
	PX_ASSERT(poolIndexEntry);

	return mCompoundTreePool.getCompoundTrees()[poolIndexEntry->second].mPruningPool->getPayloadData(handle, data);
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::preallocate(PxU32 nbEntries)
{
	mCompoundTreePool.preallocate(nbEntries);
	mMainTreeUpdateMap.resizeUninitialized(nbEntries);
	mPoolActorMap.resizeUninitialized(nbEntries);
	mChangedLeaves.reserve(nbEntries);
}

///////////////////////////////////////////////////////////////////////////////////////////////

bool BVHCompoundPruner::setTransform(PrunerHandle handle, PrunerCompoundId compoundId, const PxTransform& transform)
{
	const ActorIdPoolIndexMap::Entry* poolIndexEntry = mActorPoolMap.find(compoundId);
	PX_ASSERT(poolIndexEntry);

	return mCompoundTreePool.getCompoundTrees()[poolIndexEntry->second].mPruningPool->setTransform(handle, transform);
}

const PxTransform& BVHCompoundPruner::getTransform(PrunerCompoundId compoundId) const
{
	const ActorIdPoolIndexMap::Entry* poolIndexEntry = mActorPoolMap.find(compoundId);
	PX_ASSERT(poolIndexEntry);

	return mCompoundTreePool.getCompoundTrees()[poolIndexEntry->second].mGlobalPose;
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::updateObjectAfterManualBoundsUpdates(PrunerCompoundId compoundId, const PrunerHandle handle)
{
	const ActorIdPoolIndexMap::Entry* poolIndexEntry = mActorPoolMap.find(compoundId);
	PX_ASSERT(poolIndexEntry);
	if(!poolIndexEntry)
		return;

	mCompoundTreePool.getCompoundTrees()[poolIndexEntry->second].updateObjectAfterManualBoundsUpdates(handle);

	const PxU32 poolIndex = poolIndexEntry->second;
	updateMainTreeNode(poolIndex);
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::removeObject(PrunerCompoundId compoundId, const PrunerHandle handle, PrunerPayloadRemovalCallback* removalCallback)
{
	const ActorIdPoolIndexMap::Entry* poolIndexEntry = mActorPoolMap.find(compoundId);
	PX_ASSERT(poolIndexEntry);
	if(!poolIndexEntry)
		return;

	const PxU32 poolIndex = poolIndexEntry->second;

	mCompoundTreePool.getCompoundTrees()[poolIndex].removeObject(handle, removalCallback);

	// edge case, we removed all objects for the compound tree, we need to remove it now completely
	if(!mCompoundTreePool.getCompoundTrees()[poolIndex].mTree->getNodes())
		removeCompound(compoundId, removalCallback);
	else
		updateMainTreeNode(poolIndex);
}

///////////////////////////////////////////////////////////////////////////////////////////////

bool BVHCompoundPruner::addObject(PrunerCompoundId compoundId, PrunerHandle& result, const PxBounds3& bounds, const PrunerPayload userData, const PxTransform& transform)
{
	const ActorIdPoolIndexMap::Entry* poolIndexEntry = mActorPoolMap.find(compoundId);
	PX_ASSERT(poolIndexEntry);
	if(!poolIndexEntry)
		return false;

	mCompoundTreePool.getCompoundTrees()[poolIndexEntry->second].addObject(result, bounds, userData, transform);

	const PxU32 poolIndex = poolIndexEntry->second;
	updateMainTreeNode(poolIndex);
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::updateMainTreeNode(PoolIndex poolIndex)
{
	PxBounds3 localBounds;
	const IncrementalAABBTreeNode* node = mCompoundTreePool.getCompoundTrees()[poolIndex].mTree->getNodes();
	V4StoreU(node->mBVMin, &localBounds.minimum.x);
	PX_ALIGN(16, PxVec4) max4;
	V4StoreA(node->mBVMax, &max4.x);
	localBounds.maximum = PxVec3(max4.x, max4.y, max4.z);
	const PxBounds3 compoundBounds = PxBounds3::transformFast(mCompoundTreePool.getCompoundTrees()[poolIndex].mGlobalPose, localBounds);
	mCompoundTreePool.getCurrentCompoundBounds()[poolIndex] = compoundBounds;

	mChangedLeaves.clear();
	IncrementalAABBTreeNode* mainTreeNode = mMainTree.update(mMainTreeUpdateMap[poolIndex], poolIndex, mCompoundTreePool.getCurrentCompoundBounds(), mChangedLeaves);
	// we removed node during update, need to update the mapping
	updateMapping(poolIndex, mainTreeNode);
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::shiftOrigin(const PxVec3& shift)
{
	mCompoundTreePool.shiftOrigin(shift);

	mMainTree.shiftOrigin(shift);
}

///////////////////////////////////////////////////////////////////////////////////////////////

namespace
{
	class CompoundTreeVizCb : public DebugVizCallback
	{
		PX_NOCOPY(CompoundTreeVizCb)
		public:

		CompoundTreeVizCb(PxRenderOutput& out, const CompoundTree& tree) :
			mOut	(out),
			mPose	(tree.mGlobalPose)
		{
		}
			
		virtual	bool	visualizeNode(const IncrementalAABBTreeNode& /*node*/, const PxBounds3& bounds)
		{
			if(0)
			{
				Cm::renderOutputDebugBox(mOut, PxBounds3::transformSafe(mPose, bounds));
			}
			else
			{
				PxVec3 pts[8];
				computeBoxPoints(bounds, pts);
				for(PxU32 i=0;i<8;i++)
					pts[i] = mPose.transform(pts[i]);

				const PxU8* edges = getBoxEdges();
				for(PxU32 i=0;i<12;i++)
				{
					const PxVec3& p0 = pts[*edges++];
					const PxVec3& p1 = pts[*edges++];
					mOut.outputSegment(p0, p1);
				}
			}
			return true;
		}

		PxRenderOutput&		mOut;
		const PxTransform&	mPose;
	};

	class CompoundPrunerDebugVizCb : public DebugVizCallback
	{
		PX_NOCOPY(CompoundPrunerDebugVizCb)
		public:

		CompoundPrunerDebugVizCb(PxRenderOutput& out, const CompoundTree* trees, bool debugStatic, bool debugDynamic) :
			mOut			(out),
			mTrees			(trees),
			mDebugVizStatic	(debugStatic),
			mDebugVizDynamic(debugDynamic)
		{}
			
		virtual	bool	visualizeNode(const IncrementalAABBTreeNode& node, const PxBounds3& /*bounds*/)
		{
			if(node.isLeaf())
			{
				PxU32 nbPrims = node.getNbPrimitives();
				const PxU32* prims = node.getPrimitives(NULL);
				while(nbPrims--)
				{
					const CompoundTree& compoundTree = mTrees[*prims++];

					const bool isDynamic = compoundTree.mFlags & PxCompoundPrunerQueryFlag::eDYNAMIC;

					if((mDebugVizDynamic && isDynamic) || (mDebugVizStatic && !isDynamic))
					{
						const PxU32 color = isDynamic ? SQ_DEBUG_VIZ_DYNAMIC_COLOR : SQ_DEBUG_VIZ_STATIC_COLOR;
						CompoundTreeVizCb leafCB(mOut, compoundTree);
						visualizeTree(mOut, color, compoundTree.mTree, &leafCB);
						mOut << SQ_DEBUG_VIZ_COMPOUND_COLOR;
					}
				}
			}
			return false;
		}

		PxRenderOutput&		mOut;
		const CompoundTree*	mTrees;
		const bool			mDebugVizStatic;
		const bool			mDebugVizDynamic;
	};
}

void BVHCompoundPruner::visualize(PxRenderOutput& out, PxU32 primaryColor, PxU32 /*secondaryColor*/) const
{
	if(mDrawStatic || mDrawDynamic)
	{
		CompoundPrunerDebugVizCb cb(out, mCompoundTreePool.getCompoundTrees(), mDrawStatic, mDrawDynamic);
		visualizeTree(out, primaryColor, &mMainTree, &cb);
	}
}

void BVHCompoundPruner::visualizeEx(PxRenderOutput& out, PxU32 color, bool drawStatic, bool drawDynamic) const
{
	mDrawStatic = drawStatic;
	mDrawDynamic = drawDynamic;
	visualize(out, color, color);
}

///////////////////////////////////////////////////////////////////////////////////////////////

