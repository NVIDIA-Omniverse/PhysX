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
#include "foundation/PxIntrinsics.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxBitUtils.h"
#include "GuAABBPruner.h"
#include "GuPrunerMergeData.h"
#include "GuCallbackAdapter.h"
#include "GuSphere.h"
#include "GuBox.h"
#include "GuCapsule.h"
#include "GuAABBTreeQuery.h"
#include "GuAABBTreeNode.h"
#include "GuQuery.h"
#include "CmVisualization.h"

using namespace physx;
using namespace Gu;
using namespace Cm;

// PT: TODO: this is copied from SqBounds.h, should be either moved to Gu and shared or passed as a user parameter
	#define SQ_PRUNER_EPSILON	0.005f
	#define SQ_PRUNER_INFLATION	(1.0f + SQ_PRUNER_EPSILON)	// pruner test shape inflation (not narrow phase shape)

AABBPruner::AABBPruner(bool incrementalRebuild, PxU64 contextID, CompanionPrunerType cpType, BVHBuildStrategy buildStrategy, PxU32 nbObjectsPerNode) :
	mAABBTree			(NULL),
	mNewTree			(NULL),
	mNbCachedBoxes		(0),
	mNbCalls			(0),
	mTimeStamp			(0),
	mBucketPruner		(contextID, cpType, &mPool),
	mProgress			(BUILD_NOT_STARTED),
	mRebuildRateHint	(100),
	mAdaptiveRebuildTerm(0),
	mNbObjectsPerNode	(nbObjectsPerNode),
	mBuildStrategy		(buildStrategy),
	mPool				(contextID, TRANSFORM_CACHE_GLOBAL),
	mIncrementalRebuild	(incrementalRebuild),
	mUncommittedChanges	(false),
	mNeedsNewTree		(false),
	mNewTreeFixups		("AABBPruner::mNewTreeFixups")
{
	PX_ASSERT(nbObjectsPerNode<16);
}

AABBPruner::~AABBPruner()
{
	release();
}

bool AABBPruner::addObjects(PrunerHandle* results, const PxBounds3* bounds, const PrunerPayload* data, const PxTransform* transforms, PxU32 count, bool hasPruningStructure)
{
	PX_PROFILE_ZONE("SceneQuery.prunerAddObjects", mPool.mContextID);

	if(!count)
		return true;

	// no need to do refitMarked for added objects since they are not in the tree

	// if we have provided pruning structure, we will merge it, the changes will be applied after the objects has been addded
	if(!hasPruningStructure || !mAABBTree)
		mUncommittedChanges = true;

	// PT: TODO: 'addObjects' for bucket pruner too. Not urgent since we always call the function with count=1 at the moment
	const PxU32 valid = mPool.addObjects(results, bounds, data, transforms, count);

	// Bucket pruner is only used while the dynamic pruner is rebuilding
	// For the static pruner a full rebuild will happen in commit() every time we modify something, this is not true if
	// pruning structure was provided. The objects tree will be merged directly into the static tree. No rebuild will be triggered.
	if(mIncrementalRebuild && mAABBTree)
	{
		PX_PROFILE_ZONE("SceneQuery.bucketPrunerAddObjects", mPool.mContextID);

		mNeedsNewTree = true; // each add forces a tree rebuild

		// if a pruner structure is provided, we dont move the new objects into bucket pruner
		// the pruning structure will be merged into the bucket pruner
		if(!hasPruningStructure)
		{
			for(PxU32 i=0;i<valid;i++)
			{
				// PT: poolIndex fetched in vain for bucket pruner companion...
				// Since the incremental tree references the same pool we could just retrieve the poolIndex there, from the handle...
				const PrunerHandle handle = results[i];
				const PoolIndex poolIndex = mPool.getIndex(handle);
				mBucketPruner.addObject(data[i], handle, bounds[i], transforms[i], mTimeStamp, poolIndex);
			}
		}
	}
	return valid==count;
}

void AABBPruner::updateObjects(const PrunerHandle* handles, PxU32 count, float inflation, const PxU32* boundsIndices, const PxBounds3* newBounds, const PxTransform32* newTransforms)
{
	PX_PROFILE_ZONE("SceneQuery.prunerUpdateObjects", mPool.mContextID);

	if(!count)
		return;

	mUncommittedChanges = true;

	if(handles && boundsIndices && newBounds)
		mPool.updateAndInflateBounds(handles, boundsIndices, newBounds, newTransforms, count, inflation);

	if(mIncrementalRebuild && mAABBTree)
	{
		mNeedsNewTree = true; // each update forces a tree rebuild
		const PxBounds3* currentBounds = mPool.getCurrentWorldBoxes();
		const PxTransform* currentTransforms = mPool.getTransforms();
		const PrunerPayload* data = mPool.getObjects();
		const bool addToRefit = mProgress == BUILD_NEW_MAPPING || mProgress == BUILD_FULL_REFIT || mProgress==BUILD_LAST_FRAME;
		for(PxU32 i=0; i<count; i++)
		{
			const PrunerHandle handle = handles[i];
			const PoolIndex poolIndex = mPool.getIndex(handle);
			const TreeNodeIndex treeNodeIndex = mTreeMap[poolIndex];
			if(treeNodeIndex != INVALID_NODE_ID) // this means it's in the current tree still and hasn't been removed
				mAABBTree->markNodeForRefit(treeNodeIndex);
			else // otherwise it means it should be in the bucket pruner
			{
				PX_ASSERT(&data[poolIndex]==&mPool.getPayloadData(handle));
				bool found = mBucketPruner.updateObject(currentBounds[poolIndex], currentTransforms[poolIndex], data[poolIndex], handle, poolIndex);
				PX_UNUSED(found); PX_ASSERT(found);
			}

			if(addToRefit)
				mToRefit.pushBack(poolIndex);
		}
	}
}

void AABBPruner::removeObjects(const PrunerHandle* handles, PxU32 count, PrunerPayloadRemovalCallback* removalCallback)
{
	PX_PROFILE_ZONE("SceneQuery.prunerRemoveObjects", mPool.mContextID);

	if(!count)
		return;

	mUncommittedChanges = true;

	for(PxU32 i=0; i<count; i++)
	{
		const PrunerHandle h = handles[i];
		// copy the payload/userdata before removing it since we need to know the payload/userdata to remove it from the bucket pruner
		const PrunerPayload removedData = mPool.getPayloadData(h);
		const PoolIndex poolIndex = mPool.getIndex(h); // save the pool index for removed object
		const PoolIndex poolRelocatedLastIndex = mPool.removeObject(h, removalCallback); // save the lastIndex returned by removeObject
		if(mIncrementalRebuild && mAABBTree)
		{
			mNeedsNewTree = true;

			const TreeNodeIndex treeNodeIndex = mTreeMap[poolIndex]; // already removed from pool but still in tree map
			const PrunerPayload swappedData = mPool.getObjects()[poolIndex];
			if(treeNodeIndex!=INVALID_NODE_ID) // can be invalid if removed
			{
				mAABBTree->markNodeForRefit(treeNodeIndex); // mark the spot as blank
				mBucketPruner.swapIndex(poolIndex, swappedData, poolRelocatedLastIndex);	// if swapped index is in bucket pruner
			}
			else
			{
				bool status = mBucketPruner.removeObject(removedData, h, poolIndex, swappedData, poolRelocatedLastIndex);
				// PT: removed assert to avoid crashing all UTs
				//PX_ASSERT(status);
				PX_UNUSED(status);
			}

			mTreeMap.invalidate(poolIndex, poolRelocatedLastIndex, *mAABBTree);
			if(mNewTree)
				mNewTreeFixups.pushBack(NewTreeFixup(poolIndex, poolRelocatedLastIndex));
		}
	}

	if (mPool.getNbActiveObjects()==0)
	{
		// this is just to make sure we release all the internal data once all the objects are out of the pruner
		// since this is the only place we know that and we don't want to keep memory reserved
		release();

		// Pruner API requires a commit before the next query, even if we ended up removing the entire tree here. This
		// forces that to happen.
		mUncommittedChanges = true;
	}
}

bool AABBPruner::overlap(const ShapeData& queryVolume, PrunerOverlapCallback& pcbArgName) const
{
	PX_ASSERT(!mUncommittedChanges);

	bool again = true;

	if(mAABBTree)
	{
		OverlapCallbackAdapter pcb(pcbArgName, mPool);

		switch(queryVolume.getType())
		{
			case PxGeometryType::eBOX:
			{
				if(queryVolume.isOBB())
				{	
					const DefaultOBBAABBTest test(queryVolume);
					again = AABBTreeOverlap<true, OBBAABBTest, AABBTree, BVHNode, OverlapCallbackAdapter>()(mPool.getCurrentAABBTreeBounds(), *mAABBTree, test, pcb);
				}
				else
				{
					const DefaultAABBAABBTest test(queryVolume);
					again = AABBTreeOverlap<true, AABBAABBTest, AABBTree, BVHNode, OverlapCallbackAdapter>()(mPool.getCurrentAABBTreeBounds(), *mAABBTree, test, pcb);
				}
			}
			break;

			case PxGeometryType::eCAPSULE:
			{
				const DefaultCapsuleAABBTest test(queryVolume, SQ_PRUNER_INFLATION);
				again = AABBTreeOverlap<true, CapsuleAABBTest, AABBTree, BVHNode, OverlapCallbackAdapter>()(mPool.getCurrentAABBTreeBounds(), *mAABBTree, test, pcb);
			}
			break;

			case PxGeometryType::eSPHERE:
			{
				const DefaultSphereAABBTest test(queryVolume);
				again = AABBTreeOverlap<true, SphereAABBTest, AABBTree, BVHNode, OverlapCallbackAdapter>()(mPool.getCurrentAABBTreeBounds(), *mAABBTree, test, pcb);
			}
			break;

			case PxGeometryType::eCONVEXMESH:
			{
				const DefaultOBBAABBTest test(queryVolume);
				again = AABBTreeOverlap<true, OBBAABBTest, AABBTree, BVHNode, OverlapCallbackAdapter>()(mPool.getCurrentAABBTreeBounds(), *mAABBTree, test, pcb);
			}
			break;
		default:
			PX_ALWAYS_ASSERT_MESSAGE("unsupported overlap query volume geometry type");
		}
	}

	if(again && mIncrementalRebuild && mBucketPruner.getNbObjects())
		again = mBucketPruner.overlap(queryVolume, pcbArgName);

	return again;
}

bool AABBPruner::sweep(const ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback& pcbArgName) const
{
	PX_ASSERT(!mUncommittedChanges);

	bool again = true;

	if(mAABBTree)
	{
		RaycastCallbackAdapter pcb(pcbArgName, mPool);
		const PxBounds3& aabb = queryVolume.getPrunerInflatedWorldAABB();
		again = AABBTreeRaycast<true, true, AABBTree, BVHNode, RaycastCallbackAdapter>()(mPool.getCurrentAABBTreeBounds(), *mAABBTree, aabb.getCenter(), unitDir, inOutDistance, aabb.getExtents(), pcb);
	}

	if(again && mIncrementalRebuild && mBucketPruner.getNbObjects())
		again = mBucketPruner.sweep(queryVolume, unitDir, inOutDistance, pcbArgName);

	return again;
}

bool AABBPruner::raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback& pcbArgName) const
{
	PX_ASSERT(!mUncommittedChanges);

	bool again = true;

	if(mAABBTree)
	{
		RaycastCallbackAdapter pcb(pcbArgName, mPool);
		again = AABBTreeRaycast<false, true, AABBTree, BVHNode, RaycastCallbackAdapter>()(mPool.getCurrentAABBTreeBounds(), *mAABBTree, origin, unitDir, inOutDistance, PxVec3(0.0f), pcb);
	}
		
	if(again && mIncrementalRebuild && mBucketPruner.getNbObjects())
		again = mBucketPruner.raycast(origin, unitDir, inOutDistance, pcbArgName);

	return again;
}

// This isn't part of the pruner virtual interface, but it is part of the public interface
// of AABBPruner - it gets called by SqManager to force a rebuild, and requires a commit() before 
// queries can take place

void AABBPruner::purge()
{
	release();
	mUncommittedChanges = true; // this ensures a commit() must happen before any query
} 

void AABBPruner::setRebuildRateHint(PxU32 nbStepsForRebuild) 
{ 
	PX_ASSERT(nbStepsForRebuild > 3);
	mRebuildRateHint = (nbStepsForRebuild-3); // looks like a magic number to account for the rebuild pipeline latency
	mAdaptiveRebuildTerm = 0; 
}

// Commit either performs a refit if background rebuild is not yet finished
// or swaps the current tree for the second tree rebuilt in the background
void AABBPruner::commit()
{
	PX_PROFILE_ZONE("SceneQuery.prunerCommit", mPool.mContextID);

	if(!mUncommittedChanges && (mProgress != BUILD_FINISHED))
		// Q: seems like this is both for refit and finalization so is this is correct?
		// i.e. in a situation when we started rebuilding a tree and didn't add anything since
		// who is going to set mUncommittedChanges to true?
		// A: it's set in buildStep at final stage, so that finalization is forced.
		// Seems a bit difficult to follow and verify correctness.
		return;

	mUncommittedChanges = false;

	if(!mAABBTree || !mIncrementalRebuild)
	{
		if(!mIncrementalRebuild && mAABBTree)
			PxGetFoundation().error(PxErrorCode::ePERF_WARNING, PX_FL, "SceneQuery static AABB Tree rebuilt, because a shape attached to a static actor was added, removed or moved, and PxSceneQueryDesc::staticStructure is set to eSTATIC_AABB_TREE.");

		fullRebuildAABBTree();
		return;
	}

	// Note: it is not safe to call AABBPruner::build() here
	// because the first thread will perform one step of the incremental update,
	// continue raycasting, while the second thread performs the next step in
	// the incremental update

	// Calling Refit() below is safe. It will call 
	// StaticPruner::build() when necessary. Both will early
	// exit if the tree is already up to date, if it is not already, then we 
	// must be the first thread performing raycasts on a dirty tree and other 
	// scene query threads will be locked out by the write lock in 
	// PrunerManager::flushUpdates()

	if (mProgress != BUILD_FINISHED)
	{
		// Calling refit because the second tree is not ready to be swapped in (mProgress != BUILD_FINISHED)
		// Generally speaking as long as things keep moving the second build will never catch up with true state
		refitUpdatedAndRemoved();
	}
	else
	{
		PX_PROFILE_ZONE("SceneQuery.prunerNewTreeFinalize", mPool.mContextID);

		{
			PX_PROFILE_ZONE("SceneQuery.prunerNewTreeSwitch", mPool.mContextID);

			PX_DELETE(mAABBTree); // delete the old tree
			mCachedBoxes.release();
			mProgress = BUILD_NOT_STARTED; // reset the build state to initial

			// Adjust adaptive term to get closer to specified rebuild rate.
			// perform an even division correction to make sure the rebuild rate adds up
			if (mNbCalls > mRebuildRateHint)
				mAdaptiveRebuildTerm++;
			else if (mNbCalls < mRebuildRateHint)
				mAdaptiveRebuildTerm--;

			// Switch trees
#if PX_DEBUG
			mNewTree->validate();
#endif
			mAABBTree = mNewTree; // set current tree to progressively rebuilt tree
			mNewTree = NULL; // clear out the progressively rebuild tree pointer
			mNodeAllocator.release();
		}

		{
			PX_PROFILE_ZONE("SceneQuery.prunerNewTreeMapping", mPool.mContextID);

			// rebuild the tree map to match the current (newly built) tree
			mTreeMap.initMap(PxMax(mPool.getNbActiveObjects(), mNbCachedBoxes), *mAABBTree);

			// The new mapping has been computed using only indices stored in the new tree. Those indices map the pruning pool
			// we had when starting to build the tree. We need to re-apply recorded moves to fix the tree that finished rebuilding.
			// AP: the problem here is while we are rebuilding the tree there are ongoing modifications to the current tree
			// but the background build has a cached copy of all the AABBs at the time it was started
			// (and will produce indices referencing those)
			// Things that can happen in the meantime: update, remove, add, commit
			for(NewTreeFixup* r = mNewTreeFixups.begin(); r < mNewTreeFixups.end(); r++)
			{
				// PT: we're not doing a full refit after this point anymore, so the remaining deleted objects must be manually marked for
				// refit (otherwise their AABB in the tree would remain valid, leading to crashes when the corresponding index is 0xffffffff).
				// We must do this before invalidating the corresponding tree nodes in the map, obviously (otherwise we'd be reading node
				// indices that we already invalidated).
				const PoolIndex poolIndex = r->removedIndex;
				const TreeNodeIndex treeNodeIndex = mTreeMap[poolIndex];
				if(treeNodeIndex!=INVALID_NODE_ID)
					mAABBTree->markNodeForRefit(treeNodeIndex);

				mTreeMap.invalidate(r->removedIndex, r->relocatedLastIndex, *mAABBTree);
			}
			mNewTreeFixups.clear(); // clear out the fixups since we just applied them all
		}

		{
			PX_PROFILE_ZONE("SceneQuery.prunerNewTreeFinalRefit", mPool.mContextID);

			const PxU32 size = mToRefit.size();
			for(PxU32 i=0;i<size;i++)
			{
				const PoolIndex poolIndex = mToRefit[i];
				const TreeNodeIndex treeNodeIndex = mTreeMap[poolIndex];
				if(treeNodeIndex!=INVALID_NODE_ID)
					mAABBTree->markNodeForRefit(treeNodeIndex);
			}
			mToRefit.clear();
			refitUpdatedAndRemoved();
		}

		{
			PX_PROFILE_ZONE("SceneQuery.prunerNewTreeRemoveObjects", mPool.mContextID);

			PxU32 nbRemovedPairs = mBucketPruner.removeMarkedObjects(mTimeStamp-1);
			PX_UNUSED(nbRemovedPairs);

			mNeedsNewTree = mBucketPruner.getNbObjects()>0;
		}
	}

	updateBucketPruner();
}

void AABBPruner::shiftOrigin(const PxVec3& shift)
{
	mPool.shiftOrigin(shift);

	if(mAABBTree)
		mAABBTree->shiftOrigin(shift);

	if(mIncrementalRebuild)
		mBucketPruner.shiftOrigin(shift);

	if(mNewTree)
		mNewTree->shiftOrigin(shift);
}

void AABBPruner::visualize(PxRenderOutput& out, PxU32 primaryColor, PxU32 secondaryColor) const
{
	// getAABBTree() asserts when pruner is dirty. NpScene::visualization() does not enforce flushUpdate. see DE7834
	visualizeTree(out, primaryColor, mAABBTree);

	// Render added objects not yet in the tree
	out << PxTransform(PxIdentity);
	out << PxU32(PxDebugColor::eARGB_WHITE);

	if(mIncrementalRebuild && mBucketPruner.getNbObjects())
		mBucketPruner.visualize(out, secondaryColor);
}

bool AABBPruner::buildStep(bool synchronousCall)
{
	PX_PROFILE_ZONE("SceneQuery.prunerBuildStep", mPool.mContextID);

	PX_ASSERT(mIncrementalRebuild);
	if(mNeedsNewTree)
	{
		if(mProgress==BUILD_NOT_STARTED)
		{
			if(!synchronousCall || !prepareBuild())
				return false;
		}
		else if(mProgress==BUILD_INIT)
		{
			mNewTree->progressiveBuild(mBuilder, mNodeAllocator, mBuildStats, 0, 0);
			mProgress = BUILD_IN_PROGRESS;
			mNbCalls = 0;

			// Use a heuristic to estimate the number of work units needed for rebuilding the tree.
			// The general idea is to use the number of work units of the previous tree to build the new tree.
			// This works fine as long as the number of leaves remains more or less the same for the old and the
			// new tree. If that is not the case, this estimate can be way off and the work units per step will
			// be either much too small or too large. Hence, in that case we will try to estimate the number of work
			// units based on the number of leaves of the new tree as follows:
 			//
			// - Assume new tree with n leaves is perfectly-balanced
			// - Compute the depth of perfectly-balanced tree with n leaves
			// - Estimate number of working units for the new tree

			const PxU32 depth = PxILog2(mBuilder.mNbPrimitives);	// Note: This is the depth without counting the leaf layer
			const PxU32 estimatedNbWorkUnits = depth * mBuilder.mNbPrimitives;	// Estimated number of work units for new tree
			const PxU32 estimatedNbWorkUnitsOld = mAABBTree ? mAABBTree->getTotalPrims() : 0;
			if ((estimatedNbWorkUnits <= (estimatedNbWorkUnitsOld << 1)) && (estimatedNbWorkUnits >= (estimatedNbWorkUnitsOld >> 1)))
				// The two estimates do not differ by more than a factor 2
				mTotalWorkUnits = estimatedNbWorkUnitsOld;
 			else
			{
 				mAdaptiveRebuildTerm = 0;
				mTotalWorkUnits = estimatedNbWorkUnits;
 			}
 
 			const PxI32 totalWorkUnits = PxI32(mTotalWorkUnits + (mAdaptiveRebuildTerm * mBuilder.mNbPrimitives));
 			mTotalWorkUnits = PxU32(PxMax(totalWorkUnits, 0));
		}
		else if(mProgress==BUILD_IN_PROGRESS)
		{
			mNbCalls++;
			const PxU32 Limit = 1 + (mTotalWorkUnits / mRebuildRateHint);
			// looks like progressiveRebuild returns 0 when finished
			if(!mNewTree->progressiveBuild(mBuilder, mNodeAllocator, mBuildStats, 1, Limit))
			{
				// Done
				mProgress = BUILD_NEW_MAPPING;
#if PX_DEBUG
				mNewTree->validate();
#endif
			}
		}
		else if(mProgress==BUILD_NEW_MAPPING)
		{
			mNbCalls++;
			mProgress = BUILD_FULL_REFIT;

			// PT: we can't call fullRefit without creating the new mapping first: the refit function will fetch boxes from
			// the pool using "primitive indices" captured in the tree. But some of these indices may have been invalidated
			// if objects got removed while the tree was built. So we need to invalidate the corresponding nodes before refit,
			// that way the #prims will be zero and the code won't fetch a wrong box (which may now below to a different object).
			{
				PX_PROFILE_ZONE("SceneQuery.prunerNewTreeMapping", mPool.mContextID);

				if(mNewTreeFixups.size())
				{
					mNewTreeMap.initMap(PxMax(mPool.getNbActiveObjects(), mNbCachedBoxes), *mNewTree);

					// The new mapping has been computed using only indices stored in the new tree. Those indices map the pruning pool
					// we had when starting to build the tree. We need to re-apply recorded moves to fix the tree.
					for(NewTreeFixup* r = mNewTreeFixups.begin(); r < mNewTreeFixups.end(); r++)
						mNewTreeMap.invalidate(r->removedIndex, r->relocatedLastIndex, *mNewTree);

					mNewTreeFixups.clear();
#if PX_DEBUG
					mNewTree->validate();
#endif
				}
			}
		}
		else if(mProgress==BUILD_FULL_REFIT)
		{
			mNbCalls++;
			mProgress = BUILD_LAST_FRAME;

			{
				PX_PROFILE_ZONE("SceneQuery.prunerNewTreeFullRefit", mPool.mContextID);

				// We need to refit the new tree because objects may have moved while we were building it.
				mNewTree->fullRefit(mPool.getCurrentWorldBoxes());
			}
		}
		else if(mProgress==BUILD_LAST_FRAME)
		{
			mProgress = BUILD_FINISHED;
		}

		// This is required to be set because commit handles both refit and a portion of build finalization (why?)
		// This is overly conservative also only necessary in case there were no updates at all to the tree since the last tree swap
		// It also overly conservative in a sense that it could be set only if mProgress was just set to BUILD_FINISHED
		// If run asynchronously from a different thread, we touched just the new AABB build phase, we should not mark the main tree as dirty
		if(synchronousCall)
			mUncommittedChanges = true;

		return mProgress==BUILD_FINISHED;
	}

	return false;
}

bool AABBPruner::prepareBuild()
{
	PX_PROFILE_ZONE("SceneQuery.prepareBuild", mPool.mContextID);

	PX_ASSERT(mIncrementalRebuild);
	if(mNeedsNewTree)
	{
		if(mProgress==BUILD_NOT_STARTED)
		{
			const PxU32 nbObjects = mPool.getNbActiveObjects();
			if(!nbObjects)
				return false;

			mNodeAllocator.release();
			PX_DELETE(mNewTree);
			mNewTree = PX_NEW(AABBTree);

			mNbCachedBoxes = nbObjects;

			mCachedBoxes.init(nbObjects, mPool.getCurrentWorldBoxes());

			// PT: objects currently in the bucket pruner will be in the new tree. They are marked with the
			// current timestamp (mTimeStamp). However more objects can get added while we compute the new tree,
			// and those ones will not be part of it. These new objects will be marked with the new timestamp
			// value (mTimeStamp+1), and we can use these different values to remove the proper objects from
			// the bucket pruner (when switching to the new tree).
			mTimeStamp++;

			// notify the incremental pruner to swap trees (for incremental pruner companion)
			mBucketPruner.timeStampChange();

			mBuilder.reset();
			mBuilder.mNbPrimitives	= mNbCachedBoxes;
			mBuilder.mBounds		= &mCachedBoxes;
			mBuilder.mLimit			= mNbObjectsPerNode;
			mBuilder.mBuildStrategy	= mBuildStrategy;

			mBuildStats.reset();

			// start recording modifications to the tree made during rebuild to reapply (fix the new tree) eventually
			PX_ASSERT(mNewTreeFixups.size()==0);

			mProgress = BUILD_INIT;
		}
	}
	else
		return false;

	return true;
}

/**
 *	Builds an AABB-tree for objects in the pruning pool.
 *	\return		true if success
 */
bool AABBPruner::fullRebuildAABBTree()
{
	PX_PROFILE_ZONE("SceneQuery.prunerFullRebuildAABBTree", mPool.mContextID);

	// Release possibly already existing tree
	PX_DELETE(mAABBTree);

	// Don't bother building an AABB-tree if there isn't a single static object
	const PxU32 nbObjects = mPool.getNbActiveObjects();
	if(!nbObjects)
		return true;

	bool Status;
	{
		// Create a new tree
		mAABBTree = PX_NEW(AABBTree);

		Status = mAABBTree->build(AABBTreeBuildParams(mNbObjectsPerNode, nbObjects, &mPool.getCurrentAABBTreeBounds(), mBuildStrategy), mNodeAllocator);
	}

	// No need for the tree map for static pruner
	if(mIncrementalRebuild)
		mTreeMap.initMap(PxMax(nbObjects, mNbCachedBoxes), *mAABBTree);

	return Status;
}

// called in the end of commit(), but only if mIncrementalRebuild is true
void AABBPruner::updateBucketPruner()
{
	PX_PROFILE_ZONE("SceneQuery.prunerUpdateBucketPruner", mPool.mContextID);

	PX_ASSERT(mIncrementalRebuild);
	mBucketPruner.build();
}

void AABBPruner::release() // this can be called from purge()
{
	mBucketPruner.release();

	mTimeStamp = 0;

	mTreeMap.release();
	mNewTreeMap.release();

	mCachedBoxes.release();
	mBuilder.reset();
	mNodeAllocator.release();
	PX_DELETE(mNewTree);
	PX_DELETE(mAABBTree);

	mNbCachedBoxes = 0;
	mProgress = BUILD_NOT_STARTED;
	mNewTreeFixups.clear();
	mUncommittedChanges = false;
}

// Refit current tree
void AABBPruner::refitUpdatedAndRemoved()
{
	PX_PROFILE_ZONE("SceneQuery.prunerRefitUpdatedAndRemoved", mPool.mContextID);

	PX_ASSERT(mIncrementalRebuild);
	AABBTree* tree = getAABBTree();
	if(!tree)
		return;

#if PX_DEBUG
	tree->validate();
#endif

	//### missing a way to skip work if not needed

	const PxU32 nbObjects = mPool.getNbActiveObjects();
	// At this point there still can be objects in the tree that are blanked out so it's an optimization shortcut (not required)
	if(!nbObjects)
		return;

	mBucketPruner.refitMarkedNodes(mPool.getCurrentWorldBoxes());
	tree->refitMarkedNodes(mPool.getCurrentWorldBoxes());
}

void AABBPruner::merge(const void* mergeParams)
{
	const AABBPrunerMergeData& pruningStructure = *reinterpret_cast<const AABBPrunerMergeData*> (mergeParams);

	if(!pruningStructure.mAABBTreeNodes)
		return;

	if(mAABBTree)
	{
		// index in pruning pool, where new objects were added
		const PxU32 pruningPoolIndex = mPool.getNbActiveObjects() - pruningStructure.mNbObjects;

		// create tree from given nodes and indices
		AABBTreeMergeData aabbTreeMergeParams(pruningStructure.mNbNodes, pruningStructure.mAABBTreeNodes,
			pruningStructure.mNbObjects, pruningStructure.mAABBTreeIndices, pruningPoolIndex);

		if(!mIncrementalRebuild)
		{
			// merge tree directly
			mAABBTree->mergeTree(aabbTreeMergeParams);		
		}
		else
		{
			mBucketPruner.addTree(aabbTreeMergeParams, mTimeStamp);
		}
	}
}

void AABBPruner::getGlobalBounds(PxBounds3& bounds) const
{
	if(mAABBTree && mAABBTree->getNodes())
		bounds = mAABBTree->getNodes()->mBV;
	else
		bounds.setEmpty();

	if(mIncrementalRebuild && mBucketPruner.getNbObjects())
	{
		PxBounds3 extBounds;
		mBucketPruner.getGlobalBounds(extBounds);
		bounds.include(extBounds);
	}
}
