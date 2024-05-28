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

#include "GuSecondaryPruner.h"
#include "GuBucketPruner.h"
#include "GuIncrementalAABBPrunerCore.h"

//#define USE_DEBUG_PRINTF

#ifdef USE_DEBUG_PRINTF
	#include <stdio.h>
#endif

using namespace physx;
using namespace Gu;

class CompanionPrunerBucket : public CompanionPruner
{
	public:
					CompanionPrunerBucket() : mPrunerCore(false)	{}
	virtual			~CompanionPrunerBucket()						{}

	virtual bool	addObject(const PrunerPayload& object, PrunerHandle handle, const PxBounds3& worldAABB, const PxTransform& transform, PxU32 timeStamp, PoolIndex poolIndex)
					{
						PX_UNUSED(poolIndex);
						PX_UNUSED(handle);
						return mPrunerCore.addObject(object, worldAABB, transform, timeStamp);
					}
	virtual	bool	updateObject(const PrunerPayload& object, PrunerHandle handle, const PxBounds3& worldAABB, const PxTransform& transform, PoolIndex poolIndex)
					{
						PX_UNUSED(poolIndex);
						PX_UNUSED(handle);
						return mPrunerCore.updateObject(worldAABB, object, transform);
					}
	virtual	bool	removeObject(const PrunerPayload& object, PrunerHandle handle, PxU32 objectIndex, PxU32 swapObjectIndex)
					{
						PX_UNUSED(objectIndex);
						PX_UNUSED(swapObjectIndex);
						PX_UNUSED(handle);
						PxU32 timeStamp;
						return mPrunerCore.removeObject(object, timeStamp);
					}
	virtual	void	swapIndex(PxU32 objectIndex, PxU32 swapObjectIndex)
					{
						PX_UNUSED(objectIndex);
						PX_UNUSED(swapObjectIndex);
					}
	virtual	PxU32	removeMarkedObjects(PxU32 timeStamp)				{ return mPrunerCore.removeMarkedObjects(timeStamp);	}
	virtual	void	shiftOrigin(const PxVec3& shift)					{ mPrunerCore.shiftOrigin(shift);						}
	virtual	void	timeStampChange()									{														}
	virtual	void	build()												{ mPrunerCore.build();									}
	virtual	PxU32	getNbObjects()								const	{ return mPrunerCore.getNbObjects();					}
	virtual	void	release()											{ mPrunerCore.release();								}
	virtual	void	visualize(PxRenderOutput& out, PxU32 color)	const	{ mPrunerCore.visualize(out, color);					}
	virtual	bool	raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback& prunerCallback)	const
					{
						if(mPrunerCore.getNbObjects())
							return mPrunerCore.raycast(origin, unitDir, inOutDistance, prunerCallback);
						return true;
					}
	virtual	bool	overlap(const ShapeData& queryVolume, PrunerOverlapCallback& prunerCallback)	const
					{
						if(mPrunerCore.getNbObjects())
							return mPrunerCore.overlap(queryVolume, prunerCallback);
						return true;
					}
	virtual	bool	sweep(const ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback& prunerCallback)	const
					{
						if(mPrunerCore.getNbObjects())
							return mPrunerCore.sweep(queryVolume, unitDir, inOutDistance, prunerCallback);
						return true;
					}
	virtual	void	getGlobalBounds(PxBounds3& bounds)	const
					{
						mPrunerCore.getGlobalBounds(bounds);
					}

			BucketPrunerCore	mPrunerCore;
};

class CompanionPrunerIncremental : public CompanionPruner
{
	public:
					CompanionPrunerIncremental(const PruningPool* pool) : mPrunerCore(pool)	{}
	virtual			~CompanionPrunerIncremental()											{}

	virtual bool	addObject(const PrunerPayload& object, PrunerHandle handle, const PxBounds3& worldAABB, const PxTransform& transform, PxU32 timeStamp, PoolIndex poolIndex)
					{
						PX_UNUSED(worldAABB);
						PX_UNUSED(transform);
						PX_UNUSED(object);
						PX_UNUSED(handle);
						return mPrunerCore.addObject(poolIndex, timeStamp);
					}
	virtual	bool	updateObject(const PrunerPayload& object, PrunerHandle handle, const PxBounds3& worldAABB, const PxTransform& transform, PoolIndex poolIndex)
					{
						PX_UNUSED(worldAABB);
						PX_UNUSED(transform);
						PX_UNUSED(object);
						PX_UNUSED(handle);
						return mPrunerCore.updateObject(poolIndex);
					}
	virtual	bool	removeObject(const PrunerPayload& object, PrunerHandle handle, PxU32 objectIndex, PxU32 swapObjectIndex)
					{
						PX_UNUSED(object);
						PX_UNUSED(handle);
						PxU32 timeStamp;
						return mPrunerCore.removeObject(objectIndex, swapObjectIndex, timeStamp);
					}
	virtual	void	swapIndex(PxU32 objectIndex, PxU32 swapObjectIndex)
					{
						mPrunerCore.swapIndex(objectIndex, swapObjectIndex);
					}
	virtual	PxU32	removeMarkedObjects(PxU32 timeStamp)				{ return mPrunerCore.removeMarkedObjects(timeStamp);	}
	virtual	void	shiftOrigin(const PxVec3& shift)					{ mPrunerCore.shiftOrigin(shift);						}
	virtual	void	timeStampChange()									{ mPrunerCore.timeStampChange();						}
	virtual	void	build()												{ mPrunerCore.build();									}
	virtual	PxU32	getNbObjects()								const	{ return mPrunerCore.getNbObjects();					}
	virtual	void	release()											{ mPrunerCore.release();								}
	virtual	void	visualize(PxRenderOutput& out, PxU32 color)	const	{ mPrunerCore.visualize(out, color);					}
	virtual	bool	raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback& prunerCallback)	const
					{
						if(mPrunerCore.getNbObjects())
							return mPrunerCore.raycast(origin, unitDir, inOutDistance, prunerCallback);
						return true;
					}
	virtual	bool	overlap(const ShapeData& queryVolume, PrunerOverlapCallback& prunerCallback)	const
					{
						if(mPrunerCore.getNbObjects())
							return mPrunerCore.overlap(queryVolume, prunerCallback);
						return true;
					}
	virtual	bool	sweep(const ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback& prunerCallback)	const
					{
						if(mPrunerCore.getNbObjects())
							return mPrunerCore.sweep(queryVolume, unitDir, inOutDistance, prunerCallback);
						return true;
					}
	virtual	void	getGlobalBounds(PxBounds3& bounds)	const
					{
						mPrunerCore.getGlobalBounds(bounds);
					}

			IncrementalAABBPrunerCore	mPrunerCore;
};



#define USE_MAVERICK_NODE

#include "GuActorShapeMap.h"
#include "GuBVH.h"
#include "GuAABBTreeNode.h"
#include "GuAABBTreeBuildStats.h"
#include "GuAABBTreeQuery.h"
#include "GuQuery.h"
#ifdef USE_MAVERICK_NODE
	#include "GuMaverickNode.h"
#endif

static const bool gUpdateTreeWhenRemovingObject = false;
static const bool gUpdateObjectBoundsWhenRemovingObject = true;

class CompanionPrunerAABBTree : public CompanionPruner
{
	enum DirtyFlags
	{
		NEEDS_REBUILD	= (1<<0),
		NEEDS_REFIT		= (1<<1)
	};

	public:
											CompanionPrunerAABBTree(PxU64 contextID, const PruningPool* pool);
	virtual									~CompanionPrunerAABBTree();

	virtual			bool					addObject(const PrunerPayload& object, PrunerHandle handle, const PxBounds3& worldAABB, const PxTransform& transform, PxU32 timeStamp, PoolIndex poolIndex);
	virtual			bool					updateObject(const PrunerPayload& object, PrunerHandle handle, const PxBounds3& worldAABB, const PxTransform& transform, PoolIndex poolIndex);
	virtual			bool					removeObject(const PrunerPayload& object, PrunerHandle handle, PxU32 objectIndex, PxU32 swapObjectIndex);
	virtual			void					swapIndex(PxU32 objectIndex, PxU32 swapObjectIndex);
	virtual			PxU32					removeMarkedObjects(PxU32 timeStamp);
	virtual			void					shiftOrigin(const PxVec3& shift);
	virtual			void					timeStampChange();
	virtual			void					build();
	virtual			PxU32					getNbObjects()								const;
	virtual			void					release();
	virtual			void					visualize(PxRenderOutput& out, PxU32 color)	const;
	virtual			bool					raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback& prunerCallback)	const;
	virtual			bool					overlap(const ShapeData& queryVolume, PrunerOverlapCallback& prunerCallback)	const;
	virtual			bool					sweep(const ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback& prunerCallback)	const;
	virtual			void					getGlobalBounds(PxBounds3& bounds)	const;

	// PT: we have multiple options here, not sure which one is best:
	// - use a Gu:BVH
	// - use a Gu:AABBTree
	// - use a full blown Pruner
	// - use/reference the master PruningPool or not
	// - use a hashmap
	// - use PoolIndex
	// - use PrunerHandle
	// - somehow return our own local index to caller and use that
	//
	// The current implementation uses a PxBVH, a reference to the master PruningPool, and PrunerHandles.

#ifdef USE_MAVERICK_NODE
					MaverickNode			mMaverick;
#endif
					const PruningPool*		mPool;

					struct LocalData
					{
						PX_FORCE_INLINE	LocalData(PxU32 timestamp, PrunerHandle handle) : mTimestamp(timestamp), mHandle(handle)	{}

						PxU32			mTimestamp;
						PrunerHandle	mHandle;

						PX_FORCE_INLINE	void	setRemoved()
						{
							mTimestamp = 0xffffffff;
							mHandle = 0xffffffff;
						}

						PX_FORCE_INLINE	bool	isValid(PxU32 lastValidTimestamp)	const
						{
							return mHandle != 0xffffffff && mTimestamp>=lastValidTimestamp;
						}
					};
					PxArray<LocalData>		mLocalData;

					BVH*					mBVH;
					PxU32*					mRemap;		// Pruner handle to local index
					PxU32					mMapSize;
					PxU32					mDirtyFlags;
					PxU32					mLastValidTimestamp;

	PX_FORCE_INLINE	PxU32					getNbObjectsFast() const	{ return mLocalData.size();	}

					bool					addObjectInternal(PrunerHandle handle, PxU32 timeStamp);
					void					releaseInternal();
					void					resizeMap(PxU32 index);
};

CompanionPrunerAABBTree::CompanionPrunerAABBTree(PxU64 /*contextID*/, const PruningPool* pool) : mPool(pool),
	mBVH				(NULL),
	mRemap				(NULL),
	mMapSize			(0),
	mDirtyFlags			(0),
	mLastValidTimestamp	(0)
{
}

CompanionPrunerAABBTree::~CompanionPrunerAABBTree()
{
	releaseInternal();
}

void CompanionPrunerAABBTree::releaseInternal()
{
	PX_DELETE(mBVH);
	PX_FREE(mRemap);
	mMapSize = 0;
	mDirtyFlags = 0;
	mLastValidTimestamp = 0;
}

void CompanionPrunerAABBTree::resizeMap(PxU32 index)
{
	PxU32 size = mMapSize ? mMapSize*2 : 64;
	const PxU32 minSize = index+1;
	if(minSize>size)
		size = minSize*2;

	PxU32* items = PX_ALLOCATE(PxU32, size, "Map");
	if(mRemap)
		PxMemCopy(items, mRemap, mMapSize*sizeof(PxU32));
	PxMemSet(items+mMapSize, 0xff, (size-mMapSize)*sizeof(PxU32));
	PX_FREE(mRemap);
	mRemap = items;
	mMapSize = size;
}

bool CompanionPrunerAABBTree::addObjectInternal(PrunerHandle handle, PxU32 timeStamp)
{
	const PxU32 localIndex = getNbObjectsFast();

#ifdef USE_DEBUG_PRINTF
	printf("add %d %d to local %d\n", handle, timeStamp, localIndex);
#endif

	PX_ASSERT(handle!=0xffffffff);
	if(handle>=mMapSize)
		resizeMap(handle);

	PX_ASSERT(mRemap[handle]==0xffffffff || !mLocalData[mRemap[handle]].isValid(mLastValidTimestamp));
	mRemap[handle] = localIndex;

	mLocalData.pushBack(LocalData(timeStamp, handle));

	PX_DELETE(mBVH);
	mDirtyFlags = NEEDS_REBUILD;

	// PT: TODO: why didn't we return a secondary pruner handle from here? Could have been stored in the padding bytes of the pruning pool's transform array for example
	return true;
}

bool CompanionPrunerAABBTree::addObject(const PrunerPayload& object, PrunerHandle handle, const PxBounds3& worldAABB, const PxTransform& transform, PxU32 timeStamp, PoolIndex poolIndex)
{
	PX_UNUSED(object);
	PX_UNUSED(worldAABB);
	PX_UNUSED(transform);
	PX_UNUSED(timeStamp);
	PX_UNUSED(poolIndex);

#ifdef USE_MAVERICK_NODE
	if(mMaverick.addObject(object, handle, worldAABB, transform, timeStamp))
		return true;

	PxU32 nbToAdd = mMaverick.mNbFree;
	for(PxU32 i=0;i<nbToAdd;i++)
		addObjectInternal(mMaverick.mFreeHandles[i], mMaverick.mFreeStamps[i]);
	mMaverick.mNbFree = 0;
#endif

	return addObjectInternal(handle, timeStamp);
}

bool CompanionPrunerAABBTree::updateObject(const PrunerPayload& object, PrunerHandle handle, const PxBounds3& worldAABB, const PxTransform& transform, PoolIndex poolIndex)
{
	PX_UNUSED(object);
	PX_UNUSED(worldAABB);
	PX_UNUSED(transform);
	PX_UNUSED(poolIndex);
	PX_UNUSED(handle);

#ifdef USE_MAVERICK_NODE
	if(mMaverick.updateObject(handle, worldAABB, transform))
		return true;
#endif

	// PT: the bounds & transform have already been updated in the source pruning pool.
	// We just need to mark the corresponding node for refit.
	PX_ASSERT(handle<mMapSize);
	const PxU32 localIndex = mRemap[handle];
	PX_ASSERT(localIndex<getNbObjectsFast());
	PX_ASSERT(localIndex!=0xffffffff);
	PX_ASSERT(mLocalData[localIndex].mHandle==handle);

	if(mBVH && mBVH->updateBoundsInternal(localIndex, worldAABB))
		mDirtyFlags |= NEEDS_REFIT;

	return true;
}

bool CompanionPrunerAABBTree::removeObject(const PrunerPayload& object, PrunerHandle handle, PxU32 objectIndex, PxU32 swapObjectIndex)
{
	PX_UNUSED(object);
	PX_UNUSED(objectIndex);
	PX_UNUSED(swapObjectIndex);
	PX_UNUSED(handle);

#ifdef USE_MAVERICK_NODE
	PxU32 unused;
	if(mMaverick.removeObject(handle, unused))
		return true;
#endif

	PX_ASSERT(handle<mMapSize);
	const PxU32 localIndex = mRemap[handle];
	PX_ASSERT(localIndex<getNbObjectsFast());
	PX_ASSERT(localIndex!=0xffffffff);
	PX_ASSERT(mLocalData[localIndex].mHandle==handle);

	// PT: technically this is all we need to mark the object as removed. We can then test the handle against 0xffffffff during
	// queries and skip the object. This is optimal in terms of remove performance, but not optimal in terms of query performance.
	// There are a number of extra steps we could do here:
	//
	// - invalidate the *object* bounds. This means the tree structure itself doesn't change, it's only the object bounds used in
	//   leaf nodes that do. Empty bounds for removed objects mean we discard the object before reaching the previously mentioned
	//   handle test. This does not need an "update map".
	//
	// - update the number of primitives in the node. In this case we update the contents of a leaf node, which means decreasing
	//   the number of primitives there and reordering them so that there is no hole in the list. This requires an update map so
	//   it uses more memory and more CPU time during the remove call. It also has a really, really, really vile side-effect of
	//   invalidating the optimization that skips the object bounds test in the traversal code when the number of primitives is 1. (*)
	//
	// - the next step would be to recompute the *node* bounds, to take into account the fact that one of the bounds involved in
	//   its computation is now empty. This would avoid visiting the node at all in some queries, so it is probably worth doing
	//   if we already do the previous step. (It also requires an update map and pretty much visiting the same memory).
	//
	// - finally the last step would be to then refit the branch involving that node. This is more complicated because it needs
	//   support for partial refit in the tree, i.e. links to parent nodes etc. If we do that though, the previous step can be
	//   skipped since the node bounds recomputation will happen automatically as part of the refit procedure. The previous step
	//   is only useful as a limited refit (limited to a single node) when parent pointers are not available. The previous step
	//   can also be used as an optimization if the recomputed bounds is the same as the old one, then we can skip the more
	//   costly refit procedure. In fact this could probably be used as an optimization for the refit loop: if the box is the
	//   same as before we could break out of the loop. Note that it is possible to skip this last step here because the new
	//   bounds are guaranteed to be smaller than or equal to the previous bounds. We couldn't skip this part in the "update
	//   object" codepath for example.
	//
	// (*) the optimization relies on the fact that the narrow-phase test is roughly as expensive as the AABB test within the
	// tree, so it skips it if there is only one primitive in the node. (With multiple primitives it's worth doing the test
	// anyway since one AABB test can skip N narrow-phase tests). The annoying bit is that removing an object can suddenly mean
	// the AABB test isn't done anymore, and while it isn't a big deal in practice it's enough to break unit tests that don't
	// expect that.

#ifdef USE_DEBUG_PRINTF
	printf("remove %d %d from %d\n", handle, mLocalData[localIndex].mTimestamp, localIndex);
#endif
	mRemap[handle] = 0xffffffff;
	mLocalData[localIndex].setRemoved();

	if(mBVH)
	{
		BVHData& data = const_cast<BVHData&>(mBVH->getData());
		const PxU32 nbNodes = data.mNbNodes;
		PX_UNUSED(nbNodes);
		BVHNode* nodes = data.mNodes;
		PxU32* indices = data.mIndices;
		PxBounds3* bounds = data.mBounds.getBounds();

		if(gUpdateObjectBoundsWhenRemovingObject)
		{
			// Invalidates the object bounds, not always needed
			// The node bounds would need recomputing, and the branch refit
			bounds[localIndex].minimum = PxVec3(GU_EMPTY_BOUNDS_EXTENTS);
			bounds[localIndex].maximum = PxVec3(-GU_EMPTY_BOUNDS_EXTENTS);
		}

		PxU32* mMapping = data.getUpdateMap();
		if(gUpdateTreeWhenRemovingObject && mMapping)
		{
			// PT: note: the following codepath has only one part (as opposed to the equivalent code in AABBTreeUpdateMap)
			// because it operates on our local indices, not on (pruning) pool indices. The difference is that our local
			// array can have holes in it for removed objects, while the AABBTree's update code works with the PruningPool
			// (no holes).

			const PxU32 treeNodeIndex = mMapping[localIndex];

			if(treeNodeIndex!=0xffffffff)
			{
				PX_ASSERT(treeNodeIndex < nbNodes);
				PX_ASSERT(nodes[treeNodeIndex].isLeaf());

				BVHNode* node = nodes + treeNodeIndex;
				const PxU32 nbPrims = node->getNbRuntimePrimitives();
				PX_ASSERT(nbPrims < 16);

				// retrieve the primitives pointer
				PxU32* primitives = node->getPrimitives(indices);
				PX_ASSERT(primitives);

				// PT: look for desired local index in the leaf
				bool foundIt = false;
				for(PxU32 i=0;i<nbPrims;i++)
				{
					PX_ASSERT(mMapping[primitives[i]] == treeNodeIndex); // PT: all primitives should point to the same leaf node

					if(localIndex == primitives[i])
					{
						foundIt = true;
						const PxU32 last = nbPrims-1;
						node->setNbRunTimePrimitives(last);

						primitives[i] = 0xffffffff;			// Mark primitive index as invalid in the node
						mMapping[localIndex] = 0xffffffff;	// invalidate the node index for pool 0

						// PT: swap within the leaf node. No need to update the mapping since they should all point
						// to the same tree node anyway.
						if(last!=i)
							PxSwap(primitives[i], primitives[last]);

						// PT: breaking here means we couldn't reuse that loop to update the node bounds
						break;
					}
				}
				PX_ASSERT(foundIt);
				PX_UNUSED(foundIt);
			}
		}
	}
	return true;
}

void CompanionPrunerAABBTree::swapIndex(PxU32 objectIndex, PxU32 swapObjectIndex)
{
	PX_UNUSED(objectIndex);
	PX_UNUSED(swapObjectIndex);
}

PxU32 CompanionPrunerAABBTree::removeMarkedObjects(PxU32 timeStamp)
{	
#ifdef USE_DEBUG_PRINTF
	printf("removeMarkedObjects %d\n", timeStamp);
#endif
	PX_UNUSED(timeStamp);

	//printf("removeMarkedObjects %d\n", timeStamp);
	mLastValidTimestamp = timeStamp+1;

	// PT: TODO: consider updating our local data as well here but is it worth it?
	if(0)
	{
		const PxU32 nbObjects = getNbObjectsFast();
		for(PxU32 i=0;i<nbObjects;i++)
		{
			LocalData& localData = mLocalData[i];
			if(localData.mTimestamp==timeStamp)
			{
				localData.setRemoved();
			}
		}
	}

#ifdef USE_MAVERICK_NODE
	mMaverick.removeMarkedObjects(timeStamp);
#endif
	return 0;
}

void CompanionPrunerAABBTree::shiftOrigin(const PxVec3& shift)
{
	if(mBVH)
	{
		BVHData& data = const_cast<BVHData&>(mBVH->getData());

		PxU32 nbNodes = data.mNbNodes;
		BVHNode* nodes = data.mNodes;
		while(nbNodes--)
		{
			nodes->mBV.minimum -= shift;
			nodes->mBV.maximum -= shift;
			nodes++;
		}

		PxU32 nbObjects = getNbObjectsFast();
		PxBounds3* bounds = data.mBounds.getBounds();
		while(nbObjects--)
		{
			if(!bounds->isEmpty())
			{
				bounds->minimum -= shift;
				bounds->maximum -= shift;
			}
			bounds++;
		}
	}

#ifdef USE_MAVERICK_NODE
	mMaverick.shiftOrigin(shift);
#endif
}

void CompanionPrunerAABBTree::timeStampChange()
{
}

void CompanionPrunerAABBTree::build()
{
	if(!mDirtyFlags)	// PT: necessary, extended bucket pruner calls this without checking first
		return;

	const PxU32 needsRebuild = mDirtyFlags & NEEDS_REBUILD;
	const PxU32 needsRefit = mDirtyFlags & NEEDS_REFIT;

	mDirtyFlags = 0;

	// PT: we want fast build for this one
	const PxU32 numPrimsPerLeaf = 15;

	if(needsRebuild)
	{
		PX_DELETE(mBVH);

		PxU32 nbObjects = getNbObjectsFast();
		if(!nbObjects)
			return;

		if(1)
		{
// PT: you know what forget it just rebuild the whole map
PX_FREE(mRemap);
PxU32* newRemap = PX_ALLOCATE(PxU32, mMapSize, "Map");
PxMemSet(newRemap, 0xff, mMapSize*sizeof(PxU32));
mRemap = newRemap;

			PxU32 offset = 0;
			PxU32 nb = nbObjects;
			while(nb--)
			{
				if(!mLocalData[offset].isValid(mLastValidTimestamp))
				{
					if(0 && mLocalData[offset].mHandle!=0xffffffff)
					{
						//PX_ASSERT(mRemap[mLocalData[offset].mHandle]==offset);
						mRemap[mLocalData[offset].mHandle] = 0xffffffff;
					}

					// This object has been removed, plug the hole
					const LocalData& movedData = mLocalData[--nbObjects];

					if(movedData.isValid(mLastValidTimestamp))
					{
#ifdef USE_DEBUG_PRINTF
						printf("move %d %d from %d to %d\n", movedData.mHandle, movedData.mTimestamp, nbObjects, offset);
						if(movedData.mHandle==22)
						{
							int stop = 1;
							(void)stop;
						}
#endif
						//PX_ASSERT(mRemap[movedData.mHandle]==nbObjects);
						//mRemap[movedData.mHandle] = offset;
mRemap[movedData.mHandle] = offset;
					}
#ifdef USE_DEBUG_PRINTF
					else
						printf("skip remap %d %d from %d to %d\n", movedData.mHandle, movedData.mTimestamp, nbObjects, offset);
#endif
					mLocalData[offset] = movedData;
				}
				else
				{
mRemap[mLocalData[offset].mHandle] = offset;
					offset++;
				}
			}
			nbObjects = offset;
			mLocalData.forceSize_Unsafe(offset);
			if(!nbObjects)
				return;
		}

		if(1)
		{
			AABBTreeBounds bounds;
			bounds.init(nbObjects);
			// PT: TODO: inflation?
			const PxBounds3* currentBounds = mPool->getCurrentWorldBoxes();
			PxBounds3* dst = bounds.getBounds();
			for(PxU32 i=0; i<nbObjects; i++)
			{
				const LocalData& localData = mLocalData[i];

				const PoolIndex poolIndex = mPool->getIndex(localData.mHandle);
				dst[i] = currentBounds[poolIndex];
			}

			mBVH = PX_NEW(BVH)(NULL);
			bool status = mBVH->init(nbObjects, &bounds, NULL, 0, BVH_SPLATTER_POINTS, numPrimsPerLeaf, 0.0);
			PX_ASSERT(status);
			PX_UNUSED(status);
		}

		{
			BVHData& data = const_cast<BVHData&>(mBVH->getData());
			data.createUpdateMap(getNbObjectsFast());
		}

		return;
	}

	if(needsRefit && mBVH)
	{
		BVHData& data = const_cast<BVHData&>(mBVH->getData());
		data.refitMarkedNodes(data.mBounds.getBounds());
	}
}

PxU32 CompanionPrunerAABBTree::getNbObjects() const
{
	PxU32 nb = getNbObjectsFast();
#ifdef USE_MAVERICK_NODE
	nb += mMaverick.getNbPrimitives();
#endif
	return nb;
}

void CompanionPrunerAABBTree::release()
{
	releaseInternal();
}

void CompanionPrunerAABBTree::visualize(PxRenderOutput& out, PxU32 color) const
{
	visualizeTree(out, color, mBVH);
}

namespace
{
	struct BVHTree
	{
		PX_FORCE_INLINE	BVHTree(const BVHData& data) : mRootNode(data.mNodes), mIndices(data.mIndices)	{}

		const BVHNode*	getNodes()		const { return mRootNode;	}
		const PxU32*	getIndices()	const { return mIndices;	}

		const BVHNode*	mRootNode;
		const PxU32*	mIndices;
	};

	struct RaycastAdapter
	{
		RaycastAdapter(const CompanionPrunerAABBTree& owner, PrunerRaycastCallback& cb, PxU32 lastValidTimestamp) : mOwner(owner), mCallback(cb), mLastValidTimestamp(lastValidTimestamp), mAbort(false)	{}

		PX_FORCE_INLINE bool invoke(PxReal& distance, PxU32 index)
		{
			if(!mOwner.mLocalData[index].isValid(mLastValidTimestamp))
				return true;	// PT: object has been removed, tree data hasn't been updated accordingly

			const PxU32 handle = mOwner.mLocalData[index].mHandle;
//			if(gUpdateTreeWhenRemovingObject)
			{
				PX_ASSERT(handle!=0xffffffff);
			}
/*			else
			{
				if(handle==0xffffffff)
				{
					// PT: object has been removed, tree data hasn't been updated accordingly
					return true;
				}
			}*/
			const PoolIndex poolIndex = mOwner.mPool->getIndex(handle);

			const PxTransform* currentTransforms = mOwner.mPool->getTransforms();
			const PrunerPayload* currentPayloads = mOwner.mPool->getObjects();

			if(mAbort || !mCallback.invoke(distance, poolIndex, currentPayloads, currentTransforms))
			{
				mAbort = true;
				return false;
			}
			return true;
		}

		const CompanionPrunerAABBTree&	mOwner;
		PrunerRaycastCallback&			mCallback;
		const PxU32						mLastValidTimestamp;
		bool							mAbort;
		PX_NOCOPY(RaycastAdapter)
	};

	struct OverlapAdapter
	{
		OverlapAdapter(const CompanionPrunerAABBTree& owner, PrunerOverlapCallback& cb, PxU32 lastValidTimestamp) : mOwner(owner), mCallback(cb), mLastValidTimestamp(lastValidTimestamp), mAbort(false)	{}

		PX_FORCE_INLINE bool invoke(PxU32 index)
		{
			if(!mOwner.mLocalData[index].isValid(mLastValidTimestamp))			
				return true;	// PT: object has been removed, tree data hasn't been updated accordingly

			const PxU32 handle = mOwner.mLocalData[index].mHandle;
			PX_ASSERT(handle!=0xffffffff);
			const PoolIndex poolIndex = mOwner.mPool->getIndex(handle);

			const PxTransform* currentTransforms = mOwner.mPool->getTransforms();
			const PrunerPayload* currentPayloads = mOwner.mPool->getObjects();

			if(mAbort || !mCallback.invoke(poolIndex, currentPayloads, currentTransforms))
			{
				mAbort = true;
				return false;
			}
			return true;
		}

		const CompanionPrunerAABBTree&	mOwner;
		PrunerOverlapCallback&			mCallback;
		const PxU32						mLastValidTimestamp;
		bool							mAbort;
		PX_NOCOPY(OverlapAdapter)
	};

#ifdef USE_MAVERICK_NODE
	struct MaverickRaycastAdapter
	{
		MaverickRaycastAdapter(const MaverickNode& owner, PrunerRaycastCallback& cb) : mOwner(owner), mCallback(cb), mAbort(false)	{}

		PX_FORCE_INLINE bool invoke(PxReal& distance, PxU32 index)
		{
			if(mAbort || !mCallback.invoke(distance, index, mOwner.mFreeObjects, mOwner.mFreeTransforms))
			{
				mAbort = true;
				return false;
			}
			return true;
		}

		const MaverickNode&		mOwner;
		PrunerRaycastCallback&	mCallback;
		bool					mAbort;
		PX_NOCOPY(MaverickRaycastAdapter)
	};

	struct MaverickOverlapAdapter
	{
		MaverickOverlapAdapter(const MaverickNode& owner, PrunerOverlapCallback& cb) : mOwner(owner), mCallback(cb), mAbort(false)	{}

		PX_FORCE_INLINE bool invoke(PxU32 index)
		{
			if(mAbort || !mCallback.invoke(index, mOwner.mFreeObjects, mOwner.mFreeTransforms))
			{
				mAbort = true;
				return false;
			}
			return true;
		}

		const MaverickNode&		mOwner;
		PrunerOverlapCallback&	mCallback;
		bool					mAbort;
		PX_NOCOPY(MaverickOverlapAdapter)
	};
#endif
}

bool CompanionPrunerAABBTree::raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback& prunerCallback) const
{
	PX_UNUSED(origin);
	PX_UNUSED(unitDir);
	PX_UNUSED(inOutDistance);
	PX_UNUSED(prunerCallback);

	PX_ASSERT(!mDirtyFlags);
//	if(mDirtyFlags)
//		const_cast<CompanionPrunerAABBTree*>(this)->build();

#ifdef USE_MAVERICK_NODE
	{
		MaverickRaycastAdapter ra(mMaverick, prunerCallback);
		Gu::RayAABBTest test(origin*2.0f, unitDir*2.0f, inOutDistance, PxVec3(0.0f));
		if(!doLeafTest<false, true, MaverickNode, MaverickRaycastAdapter>(&mMaverick, test, mMaverick.mFreeBounds, NULL, inOutDistance, ra))
			return false;
	}
#endif

	if(mBVH)
	{
		RaycastAdapter ra(*this, prunerCallback, mLastValidTimestamp);
		return AABBTreeRaycast<false, true, BVHTree, BVHNode, RaycastAdapter>()(mBVH->getData().mBounds, BVHTree(mBVH->getData()), origin, unitDir, inOutDistance, PxVec3(0.0f), ra);
	}
	return true;
}

// PT: TODO: this is copied from SqBounds.h, should be either moved to Gu and shared or passed as a user parameter
	#define SQ_PRUNER_EPSILON	0.005f
	#define SQ_PRUNER_INFLATION	(1.0f + SQ_PRUNER_EPSILON)	// pruner test shape inflation (not narrow phase shape)

bool CompanionPrunerAABBTree::overlap(const ShapeData& queryVolume, PrunerOverlapCallback& prunerCallback) const
{
	PX_UNUSED(queryVolume);
	PX_UNUSED(prunerCallback);

	PX_ASSERT(!mDirtyFlags);
//	if(mDirtyFlags)
//		const_cast<CompanionPrunerAABBTree*>(this)->build();

#ifdef USE_MAVERICK_NODE
	{
		MaverickOverlapAdapter ra(mMaverick, prunerCallback);

		switch(queryVolume.getType())
		{
			case PxGeometryType::eBOX:
			{
				if(queryVolume.isOBB())
				{	
					const DefaultOBBAABBTest test(queryVolume);
					if(!doOverlapLeafTest<true, OBBAABBTest, MaverickNode, MaverickOverlapAdapter>(test, &mMaverick, mMaverick.mFreeBounds, NULL, ra))
						return false;
				}
				else
				{
					const DefaultAABBAABBTest test(queryVolume);
					if(!doOverlapLeafTest<true, AABBAABBTest, MaverickNode, MaverickOverlapAdapter>(test, &mMaverick, mMaverick.mFreeBounds, NULL, ra))
						return false;
				}
			}
			break;

			case PxGeometryType::eCAPSULE:
			{
				const DefaultCapsuleAABBTest test(queryVolume, SQ_PRUNER_INFLATION);
				if(!doOverlapLeafTest<true, CapsuleAABBTest, MaverickNode, MaverickOverlapAdapter>(test, &mMaverick, mMaverick.mFreeBounds, NULL, ra))
					return false;
			}
			break;

			case PxGeometryType::eSPHERE:
			{
				const DefaultSphereAABBTest test(queryVolume);
				if(!doOverlapLeafTest<true, SphereAABBTest, MaverickNode, MaverickOverlapAdapter>(test, &mMaverick, mMaverick.mFreeBounds, NULL, ra))
					return false;
			}
			break;

			case PxGeometryType::eCONVEXMESH:
			{
				const DefaultOBBAABBTest test(queryVolume);
				if(!doOverlapLeafTest<true, OBBAABBTest, MaverickNode, MaverickOverlapAdapter>(test, &mMaverick, mMaverick.mFreeBounds, NULL, ra))
					return false;
			}
			break;

			default:
				PX_ALWAYS_ASSERT_MESSAGE("unsupported overlap query volume geometry type");
		}
	}
#endif

	if(mBVH)
	{
		OverlapAdapter ra(*this, prunerCallback, mLastValidTimestamp);

		switch(queryVolume.getType())
		{
			case PxGeometryType::eBOX:
			{
				if(queryVolume.isOBB())
				{	
					const DefaultOBBAABBTest test(queryVolume);
					return AABBTreeOverlap<true, OBBAABBTest, BVHTree, BVHNode, OverlapAdapter>()(mBVH->getData().mBounds, BVHTree(mBVH->getData()), test, ra);
				}
				else
				{
					const DefaultAABBAABBTest test(queryVolume);
					return AABBTreeOverlap<true, AABBAABBTest, BVHTree, BVHNode, OverlapAdapter>()(mBVH->getData().mBounds, BVHTree(mBVH->getData()), test, ra);
				}
			}

			case PxGeometryType::eCAPSULE:
			{
				const DefaultCapsuleAABBTest test(queryVolume, SQ_PRUNER_INFLATION);
				//const DefaultCapsuleAABBTest test(queryVolume, 1.0f);
				return AABBTreeOverlap<true, CapsuleAABBTest, BVHTree, BVHNode, OverlapAdapter>()(mBVH->getData().mBounds, BVHTree(mBVH->getData()), test, ra);
			}

			case PxGeometryType::eSPHERE:
			{
				const DefaultSphereAABBTest test(queryVolume);
				return AABBTreeOverlap<true, SphereAABBTest, BVHTree, BVHNode, OverlapAdapter>()(mBVH->getData().mBounds, BVHTree(mBVH->getData()), test, ra);
			}

			case PxGeometryType::eCONVEXMESH:
			{
				const DefaultOBBAABBTest test(queryVolume);
				return AABBTreeOverlap<true, OBBAABBTest, BVHTree, BVHNode, OverlapAdapter>()(mBVH->getData().mBounds, BVHTree(mBVH->getData()), test, ra);
			}

			default:
				PX_ALWAYS_ASSERT_MESSAGE("unsupported overlap query volume geometry type");
		}
	}
	return true;
}

bool CompanionPrunerAABBTree::sweep(const ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback& prunerCallback) const
{
	PX_UNUSED(queryVolume);
	PX_UNUSED(unitDir);
	PX_UNUSED(inOutDistance);
	PX_UNUSED(prunerCallback);

	PX_ASSERT(!mDirtyFlags);
//	if(mDirtyFlags)
//		const_cast<CompanionPrunerAABBTree*>(this)->build();

#ifdef USE_MAVERICK_NODE
	{
		MaverickRaycastAdapter ra(mMaverick, prunerCallback);
		const PxBounds3& aabb = queryVolume.getPrunerInflatedWorldAABB();
		Gu::RayAABBTest test(aabb.getCenter()*2.0f, unitDir*2.0f, inOutDistance, aabb.getExtents());
		if(!doLeafTest<true, true, MaverickNode, MaverickRaycastAdapter>(&mMaverick, test, mMaverick.mFreeBounds, NULL, inOutDistance, ra))
			return false;
	}
#endif

	if(mBVH)
	{
		RaycastAdapter ra(*this, prunerCallback, mLastValidTimestamp);
		const PxBounds3& aabb = queryVolume.getPrunerInflatedWorldAABB();
		return AABBTreeRaycast<true, true, BVHTree, BVHNode, RaycastAdapter>()(mBVH->getData().mBounds, BVHTree(mBVH->getData()), aabb.getCenter(), unitDir, inOutDistance, aabb.getExtents(), ra);
	}
	return true;
}

	class PxBounds3Padded : public PxBounds3
	{
	public:
		PX_FORCE_INLINE PxBounds3Padded()	{}
		PX_FORCE_INLINE ~PxBounds3Padded()	{}
		PxU32	padding;
	};

void CompanionPrunerAABBTree::getGlobalBounds(PxBounds3& bounds) const
{
	PxBounds3Padded tmp;

	if(mBVH)
	{
		tmp.minimum = mBVH->getNodes()->mBV.minimum;
		tmp.maximum = mBVH->getNodes()->mBV.maximum;
	}
	else
		tmp.setEmpty();

	Vec4V minV = V4LoadU(&tmp.minimum.x);
	Vec4V maxV = V4LoadU(&tmp.maximum.x);
#ifdef USE_MAVERICK_NODE
	{
		PxU32 nbFree = mMaverick.mNbFree;
		if(nbFree)
		{
			const PxBounds3* freeBounds = mMaverick.mFreeBounds;
			while(nbFree--)
			{
				minV = V4Min(minV, V4LoadU(&freeBounds->minimum.x));
				maxV = V4Max(maxV, V4LoadU(&freeBounds->maximum.x));
				freeBounds++;
			}
		}
	}
#endif
	StoreBounds(bounds, minV, maxV);
}



CompanionPruner* physx::Gu::createCompanionPruner(PxU64 contextID, CompanionPrunerType type, const PruningPool* pool)
{
	if(0)
//		return NULL;
		return PX_NEW(CompanionPrunerAABBTree)(contextID, pool);
		//return PX_NEW(CompanionPrunerBucket);
//		return PX_NEW(CompanionPrunerIncremental)(pool);

	PX_UNUSED(contextID);
	switch(type)
	{
		case COMPANION_PRUNER_NONE:			return NULL;
		case COMPANION_PRUNER_BUCKET:		return PX_NEW(CompanionPrunerBucket);
		case COMPANION_PRUNER_INCREMENTAL:	return PX_NEW(CompanionPrunerIncremental)(pool);
		case COMPANION_PRUNER_AABB_TREE:	return PX_NEW(CompanionPrunerAABBTree)(contextID, pool);
	}
	return NULL;
}

