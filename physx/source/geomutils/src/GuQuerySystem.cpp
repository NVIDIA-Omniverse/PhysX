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

#include "GuQuerySystem.h"
#include "GuBounds.h"
#include "GuBVH.h"
#include "foundation/PxAlloca.h"
#include "common/PxProfileZone.h"

using namespace physx;
using namespace Gu;

///////////////////////////////////////////////////////////////////////////////

bool contains(PxArray<PxU32>& pruners, PxU32 index)
{
	const PxU32 nb = pruners.size();
	for(PxU32 i=0;i<nb;i++)
	{
		if(pruners[i]==index)
			return true;
	}
	return false;
}

///////////////////////////////////////////////////////////////////////////////

QuerySystem::PrunerExt::PrunerExt(Pruner* pruner, PxU32 preallocated) : mPruner(pruner), mDirtyList("QuerySystem::PrunerExt::mDirtyList"), mNbStatic(0), mNbDynamic(0), mDirtyStatic(false)
{
	if(pruner&& preallocated)
		pruner->preallocate(preallocated);
}

QuerySystem::PrunerExt::~PrunerExt()
{
	PX_DELETE(mPruner);
}

void QuerySystem::PrunerExt::flushMemory()
{
	if(!mDirtyList.size())
		mDirtyList.reset();

	// PT: TODO: flush bitmap here

	// PT: TODO: flush pruner here?
}

// PT: ok things became more complicated than before here. We'd like to delay the update of *both* the transform and the bounds,
// since immediately updating only one of them doesn't make much sense (it invalidates the pruner's data structure anyway). When both
// are delayed it gives users the ability to query the pruners *without* commiting the changes, i.e. they can query the old snapshot
// for as long as they please (i.e. a raycast wouldn't automatically trigger a structure update).
//
// Now the problem is that we need to store (at least) the transform until the update actually happens, and the initial code didn't
// support this. We also want to do this in an efficient way, which of course makes things more difficult.
//
// A naive version would simply use a per-pruner hashmap between the PrunerHandle and its data. Might be slower than before.
//
// Another version could build on the initial bitmap-based solution and use arrays of transforms/bounds as companions to the array
// of PrunerHandle (or we could mix all that data in a single structure). The issue with this is that two consecutive updates on the
// same object wouldn't work anymore: the second call would check the bitmap, see that the bit is set already, and skip the work.
// We'd need to update the cached data instead, i.e. we'd need a mapping between the PrunerHandle and its position in mDirtyList.
// And we don't have that.
//
// A potential way to fix this could be to allow the same PrunerHandle to appear multiple times in mDirtyList, with the assumption
// that users will not update the same object multiple times very often (...). The way it would work:
// - during "add", dirtyMap is set, handle/transform/bounds are pushed to mDirtyList.
// - during "remove", dirtyMap is reset *and that's it*. We don't bother purging mDirtyList (i.e. we kill the current O(n) search there)
// - during "process" we use dirtyMap to validate the update. If bit is cleared, ignore mDirtyList entry. Duplicate entries work as long
//   as mDirtyList is processed in linear order. One issue is that the current mDirtyList is also passed to the pruner as-is for the
//   update, so we'd need to rebuild a separate array for that and/or make sure all pruners accept duplicate entries in that array.
//   Deep down that specific rabbit hole we'll actually find the recently discovered issue regarding the mToRefit array...
//
// Bit tricky. This is only for user-updates anyway (as opposed to sim updates) so this probably doesn't need ultimate perf? Note however 
// that we "remove from dirty list" when an object is removed, which happens all the time with or without user updates (e.g. streaming etc).

static const bool gUseOldCode = false;

void QuerySystem::PrunerExt::addToDirtyList(PrunerHandle handle, PxU32 dynamic, const PxTransform& transform, const PxBounds3* userBounds)
{
	PxBitMap& dirtyMap = mDirtyMap;
	{
		if(dirtyMap.size() <= handle)
		{
			PxU32 size = PxMax<PxU32>(dirtyMap.size()*2, 1024);
			const PxU32 minSize = handle+1;
			if(minSize>size)
				size = minSize*2;
			dirtyMap.resize(size);
			PX_ASSERT(handle<dirtyMap.size());
			PX_ASSERT(!dirtyMap.test(handle));
		}
	}

	if(gUseOldCode)
	{
		if(!dirtyMap.test(handle))
		{
			dirtyMap.set(handle);
			mDirtyList.pushBack(handle);
		}
	}
	else
	{
		dirtyMap.set(handle);
		mDirtyList.pushBack(handle);

		Data& d = mDirtyData.insert();
		d.mPose = transform;
		if(userBounds)
			d.mBounds = *userBounds;
		else
			d.mBounds.setEmpty();
	}

	if(!dynamic)
		mDirtyStatic = true;
}

void QuerySystem::PrunerExt::removeFromDirtyList(PrunerHandle handle)
{
	PxBitMap& dirtyMap = mDirtyMap;
	if(gUseOldCode)
	{
		if(dirtyMap.boundedTest(handle))
		{
			dirtyMap.reset(handle);
			mDirtyList.findAndReplaceWithLast(handle);
		}
	}
	else
	{
		dirtyMap.boundedReset(handle);
	}

	// PT: if we remove the object that made us set mDirtyStatic to true, tough luck,
	// we don't bother fixing that bool here. It's going to potentially cause an
	// unnecessary update of the character controller's caches, which is not a big deal.
}

bool QuerySystem::PrunerExt::processDirtyList(const Adapter& adapter, float inflation)
{
	const PxU32 numDirtyList = mDirtyList.size();
	if(!numDirtyList)
		return false;

	if(gUseOldCode)
	{
		const PrunerHandle* const prunerHandles = mDirtyList.begin();
		for(PxU32 i=0; i<numDirtyList; i++)
		{
			const PrunerHandle handle = prunerHandles[i];
			mDirtyMap.reset(handle);

			// PT: we compute the new bounds and store them directly in the pruner structure to avoid copies. We delay the updateObjects() call
			// to take advantage of batching.
			PrunerPayloadData payloadData;
			const PrunerPayload& pp = mPruner->getPayloadData(handle, &payloadData);

			computeBounds(*payloadData.mBounds, adapter.getGeometry(pp), *payloadData.mTransform, 0.0f, inflation);
		}
		// PT: batch update happens after the loop instead of once per loop iteration
		mPruner->updateObjects(prunerHandles, numDirtyList);
		mDirtyList.clear();
	}
	else
	{
		// PT: TODO: this stuff is not 100% satisfying, since we do allow the same object to be updated multiple times.
		// Would be nice to revisit & improve at some point.

		PrunerHandle* prunerHandles = mDirtyList.begin();

		PxU32 nbValid = 0;
		for(PxU32 i=0; i<numDirtyList; i++)
		{
			const PrunerHandle handle = prunerHandles[i];
			if(mDirtyMap.test(handle))
			{
				// PT: we compute the new bounds and store them directly in the pruner structure to avoid copies. We delay the updateObjects() call
				// to take advantage of batching.
				PrunerPayloadData payloadData;
				const PrunerPayload& pp = mPruner->getPayloadData(handle, &payloadData);

				*payloadData.mTransform = mDirtyData[i].mPose;

				if(mDirtyData[i].mBounds.isEmpty())
					computeBounds(*payloadData.mBounds, adapter.getGeometry(pp), mDirtyData[i].mPose, 0.0f, inflation);
				else
					*payloadData.mBounds = mDirtyData[i].mBounds;

				prunerHandles[nbValid++] = handle;
			}
			else
			{
				// PT: if not set, object has been added to the list then removed
			}
		}

		// PT: batch update happens after the loop instead of once per loop iteration
		mPruner->updateObjects(prunerHandles, nbValid);

		// PT: have to reset the bits *after* the above loop now. Unclear if clearing the
		// whole map would be faster ("it depends" I guess).
		while(nbValid--)
		{
			const PrunerHandle handle = *prunerHandles++;
			mDirtyMap.reset(handle);
		}

		mDirtyList.clear();
		mDirtyData.clear();
	}

	const bool ret = mDirtyStatic;
	mDirtyStatic = false;
	return ret;
}

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE QuerySystem::PrunerExt* checkPrunerIndex(PxU32 prunerIndex, const PxArray<QuerySystem::PrunerExt*>& prunerExt)
{
	if(prunerIndex>=prunerExt.size() || !prunerExt[prunerIndex])
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "Invalid pruner index");
		return NULL;
	}

	return prunerExt[prunerIndex];
}

QuerySystem::QuerySystem(PxU64 contextID, float inflation, const Adapter& adapter, bool usesTreeOfPruners) :
	mAdapter				(adapter),
	mTreeOfPruners			(NULL),
	mContextID				(contextID),
	mStaticTimestamp		(0),
	mInflation				(inflation),
	mPrunerNeedsUpdating	(false),
	mTimestampNeedsUpdating	(false),
	mUsesTreeOfPruners		(usesTreeOfPruners)
	//mBatchUserUpdates		(batchUserUpdates)
{
}

QuerySystem::~QuerySystem()
{
	PX_DELETE(mTreeOfPruners);

	const PxU32 nb = mPrunerExt.size();
	for(PxU32 i=0;i<nb;i++)
	{
		PrunerExt* pe = mPrunerExt[i];	// Can be NULL if the pruner has been removed
		PX_DELETE(pe);
	}
}

PxU32 QuerySystem::addPruner(Pruner* pruner, PxU32 preallocated)
{
	PrunerExt* pe = PX_NEW(PrunerExt)(pruner, preallocated);

	PxU32 prunerIndex;
	if(mFreePruners.size())
	{
		prunerIndex = mFreePruners.popBack();
		mPrunerExt[prunerIndex] = pe;
	}
	else
	{
		prunerIndex = mPrunerExt.size();
		mPrunerExt.pushBack(pe);
	}
	return prunerIndex;
}

void QuerySystem::removePruner(PxU32 prunerIndex)
{
	PrunerExt* pe = checkPrunerIndex(prunerIndex, mPrunerExt);
	if(!pe)
		return;

	// PT: it is legal to delete a pruner that still contains objects, but we should still properly update the static timestamp.
	if(pe->mNbStatic)
		invalidateStaticTimestamp();

	PX_DELETE(pe);
	mPrunerExt[prunerIndex] = NULL;

	mFreePruners.pushBack(prunerIndex);

	// We don't bother searching mDirtyPruners since it's going to be cleared next frame
}

void QuerySystem::flushMemory()
{
	const PxU32 nb = mPrunerExt.size();
	for(PxU32 i=0;i<nb;i++)
	{
		PrunerExt* pe = mPrunerExt[i];	// Can be NULL if the pruner has been removed
		if(pe)
			pe->flushMemory();
	}
}

ActorShapeData QuerySystem::addPrunerShape(const PrunerPayload& payload, PxU32 prunerIndex, bool dynamic, const PxTransform& transform, const PxBounds3* userBounds)
{
	PrunerExt* pe = checkPrunerIndex(prunerIndex, mPrunerExt);
	if(!pe)
		return INVALID_ACTOR_SHAPE_DATA;

	mPrunerNeedsUpdating = true;

	if(dynamic)
	{
		pe->mNbDynamic++;
	}
	else
	{
		pe->mNbStatic++;
		invalidateStaticTimestamp();
	}

	PX_ASSERT(pe->mPruner);

	const PxBounds3* boundsPtr;
	PxBounds3 bounds;
	if(userBounds)
	{
		boundsPtr = userBounds;
	}
	else
	{
		computeBounds(bounds, mAdapter.getGeometry(payload), transform, 0.0f, 1.0f + mInflation);
		boundsPtr = &bounds;
	}

	PrunerHandle handle;
	pe->mPruner->addObjects(&handle, boundsPtr, &payload, &transform, 1, false);

	return createActorShapeData(createPrunerInfo(prunerIndex, dynamic), handle);
}

void QuerySystem::removePrunerShape(ActorShapeData data, PrunerPayloadRemovalCallback* removalCallback)
{
	const PrunerInfo info = getPrunerInfo(data);
	const PxU32 prunerIndex = getPrunerIndex(info);

	PrunerExt* pe = checkPrunerIndex(prunerIndex, mPrunerExt);
	if(!pe)
		return;

	mPrunerNeedsUpdating = true;
	const PxU32 dynamic = getDynamic(info);

	const PrunerHandle handle = getPrunerHandle(data);

	PX_ASSERT(pe->mPruner);

	if(dynamic)
	{
		PX_ASSERT(pe->mNbDynamic);
		pe->mNbDynamic--;
	}
	else
	{
		PX_ASSERT(pe->mNbStatic);
		pe->mNbStatic--;
		invalidateStaticTimestamp();
	}

	//if(mBatchUserUpdates)
		pe->removeFromDirtyList(handle);
	pe->mPruner->removeObjects(&handle, 1, removalCallback);
}

void QuerySystem::updatePrunerShape(ActorShapeData data, bool immediately, const PxTransform& transform, const PxBounds3* userBounds)
{
	const PrunerInfo info = getPrunerInfo(data);
	const PxU32 prunerIndex = getPrunerIndex(info);

	PrunerExt* pe = checkPrunerIndex(prunerIndex, mPrunerExt);
	if(!pe)
		return;

	mPrunerNeedsUpdating = true;
	const PxU32 dynamic = getDynamic(info);
	const PrunerHandle handle = getPrunerHandle(data);

	PX_ASSERT(pe->mPruner);

	Pruner* pruner = pe->mPruner;

	if(immediately)
	{
		if(!dynamic)
			invalidateStaticTimestamp();

		PrunerPayloadData payloadData;
		const PrunerPayload& pp = pruner->getPayloadData(handle, &payloadData);

		*payloadData.mTransform = transform;

		if(userBounds)
			*payloadData.mBounds = *userBounds;
		else
			computeBounds(*payloadData.mBounds, mAdapter.getGeometry(pp), transform, 0.0f, 1.0f + mInflation);

		// PT: TODO: would it be better to pass the bounds & transform directly to this function?
		pruner->updateObjects(&handle, 1);
	}
	else
	{
		// PT: we don't update the static timestamp immediately, so that users can query the
		// old state of the structure without invalidating their caches. This will be resolved
		// in processDirtyLists.

		if(gUseOldCode)
			pruner->setTransform(handle, transform);

		// PT: we don't shrink mDirtyList anymore in removePrunerShape so the size of that array can be reused as
		// a flag telling us whether we already encountered this pruner or not. If not, we add its index to mDirtyPruners.
		// Goal is to avoid processing all pruners in processDirtyLists.
		if(!pe->mDirtyList.size())
		{
			PX_ASSERT(!contains(mDirtyPruners, prunerIndex));
			mDirtyPruners.pushBack(prunerIndex);
		}
		else
		{
			PX_ASSERT(contains(mDirtyPruners, prunerIndex));
		}

		pe->addToDirtyList(handle, dynamic, transform, userBounds);
	}
}

const PrunerPayload& QuerySystem::getPayloadData(ActorShapeData data, PrunerPayloadData* ppd) const
{
	const PrunerInfo info = getPrunerInfo(data);
	const PxU32 prunerIndex = getPrunerIndex(info);

	PX_ASSERT(checkPrunerIndex(prunerIndex, mPrunerExt));

	const PrunerHandle handle = getPrunerHandle(data);

	PX_ASSERT(mPrunerExt[prunerIndex]->mPruner);
	return mPrunerExt[prunerIndex]->mPruner->getPayloadData(handle, ppd);
}

void QuerySystem::processDirtyLists()
{
	PX_PROFILE_ZONE("QuerySystem.processDirtyLists", mContextID);

	const PxU32 nbDirtyPruners = mDirtyPruners.size();
	if(!nbDirtyPruners)
		return;

	// must already have acquired writer lock here

	const float inflation = 1.0f + mInflation;

	bool mustInvalidateStaticTimestamp = false;
	for(PxU32 ii=0;ii<nbDirtyPruners;ii++)
	{
		const PxU32 i = mDirtyPruners[ii];

		PrunerExt* pe = mPrunerExt[i];	// Can be NULL if the pruner has been removed
		if(pe && pe->processDirtyList(mAdapter, inflation))
			mustInvalidateStaticTimestamp = true;
	}

	if(mustInvalidateStaticTimestamp)
		invalidateStaticTimestamp();

	mDirtyPruners.clear();
}

void QuerySystem::update(bool buildStep, bool commit)
{
	PX_PROFILE_ZONE("QuerySystem::update", mContextID);

	if(!buildStep && !commit)
	{
		//mPrunerNeedsUpdating = true;	// PT: removed, why was it here?
		return;
	}

	// flush user modified objects
//	if(mBatchUserUpdates)
		processDirtyLists();

	const PxU32 nb = mPrunerExt.size();
	for(PxU32 i=0;i<nb;i++)
	{
		PrunerExt* pe = mPrunerExt[i];	// Can be NULL if the pruner has been removed
		if(!pe)
			continue;

		Pruner* pruner = pe->mPruner;
		if(pruner)
		{
			if(buildStep && pruner->isDynamic())
				static_cast<DynamicPruner*>(pruner)->buildStep(true);

			if(commit)
				pruner->commit();
		}
	}

	if(commit)
	{
		if(mUsesTreeOfPruners)
			createTreeOfPruners();
	}

	mPrunerNeedsUpdating = !commit;
}

void QuerySystem::commitUpdates()
{
	PX_PROFILE_ZONE("QuerySystem.commitUpdates", mContextID);

	if(mPrunerNeedsUpdating)
	{
		mSQLock.lock();

		if(mPrunerNeedsUpdating)
		{
			//if(mBatchUserUpdates)
				processDirtyLists();

			const PxU32 nb = mPrunerExt.size();
			for(PxU32 i=0;i<nb;i++)
			{
				PrunerExt* pe = mPrunerExt[i];	// Can be NULL if the pruner has been removed
				if(!pe)
					continue;

				Pruner* pruner = pe->mPruner;
				if(pruner)
					pruner->commit();
			}

			if(mUsesTreeOfPruners)
				createTreeOfPruners();

			PxMemoryBarrier();
			mPrunerNeedsUpdating = false;
		}
		mSQLock.unlock();
	}
}

PxU32 QuerySystem::startCustomBuildstep()
{
	PX_PROFILE_ZONE("QuerySystem.startCustomBuildstep", mContextID);

	mTimestampNeedsUpdating = false;

	return mPrunerExt.size();
}

void QuerySystem::customBuildstep(PxU32 index)
{
	PX_PROFILE_ZONE("QuerySystem.customBuildstep", mContextID);

	PX_ASSERT(index<mPrunerExt.size());

	// PT: TODO: would be better to not schedule the update of removed pruners at all
	PrunerExt* pe = mPrunerExt[index];		// Can be NULL if the pruner has been removed
	if(!pe)
		return;

	Pruner* pruner = pe->mPruner;

	//void QuerySystem::processDirtyLists()
	{
		PX_PROFILE_ZONE("QuerySystem.processDirtyLists", mContextID);
		// must already have acquired writer lock here
		const float inflation = 1.0f + mInflation;
		// PT: note that we don't use the mDirtyPruners array here
		if(pe->processDirtyList(mAdapter, inflation))
			mTimestampNeedsUpdating = true;
	}

	if(pruner)
	{
		if(pruner->isDynamic())
			static_cast<DynamicPruner*>(pruner)->buildStep(true);	// PT: "true" because that parameter was made for PxSceneQuerySystem::sceneQueryBuildStep(), not us

		pruner->commit();
	}
}

void QuerySystem::finishCustomBuildstep()
{
	PX_PROFILE_ZONE("QuerySystem.finishCustomBuildstep", mContextID);

	if(mUsesTreeOfPruners)
		createTreeOfPruners();

	mPrunerNeedsUpdating = false;
	if(mTimestampNeedsUpdating)
		invalidateStaticTimestamp();

	mDirtyPruners.clear();
}

void QuerySystem::sync(PxU32 prunerIndex, const PrunerHandle* handles, const PxU32* boundsIndices, const PxBounds3* bounds, const PxTransform32* transforms, PxU32 count)
{
	if(!count)
		return;

	PrunerExt* pe = checkPrunerIndex(prunerIndex, mPrunerExt);
	if(!pe)
		return;

	Pruner* pruner = pe->mPruner;
	if(pruner)
		pruner->updateObjects(handles, count, mInflation, boundsIndices, bounds, transforms);
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	struct LocalRaycastCB : PxBVH::RaycastCallback
	{
		LocalRaycastCB(const PxArray<QuerySystem::PrunerExt*>& pruners, const PrunerFilter* prunerFilter, const PxVec3& origin, const PxVec3& unitDir, PrunerRaycastCallback& cb) :
			mPrunerExt(pruners), mPrunerFilter(prunerFilter), mOrigin(origin), mUnitDir(unitDir), mCB(cb)	{}

		virtual bool	reportHit(PxU32 boundsIndex, PxReal& distance)
		{
			QuerySystem::PrunerExt* pe = mPrunerExt[boundsIndex];	// Can be NULL if the pruner has been removed
			if(pe && (!mPrunerFilter || mPrunerFilter->processPruner(boundsIndex)))
			{
				Pruner* pruner = pe->mPruner;
				if(!pruner->raycast(mOrigin, mUnitDir, distance, mCB))
					return false;
			}
			return true;
		}

		const PxArray<QuerySystem::PrunerExt*>&	mPrunerExt;
		const PrunerFilter*						mPrunerFilter;
		const PxVec3&							mOrigin;
		const PxVec3&							mUnitDir;
		PrunerRaycastCallback&					mCB;

		PX_NOCOPY(LocalRaycastCB)
	};

	struct LocalOverlapCB : PxBVH::OverlapCallback
	{
		LocalOverlapCB(const PxArray<QuerySystem::PrunerExt*>& pruners, const PrunerFilter* prunerFilter, const ShapeData& queryVolume, PrunerOverlapCallback& cb) :
			mPrunerExt(pruners), mPrunerFilter(prunerFilter), mQueryVolume(queryVolume), mCB(cb)	{}

		virtual bool	reportHit(PxU32 boundsIndex)
		{
			QuerySystem::PrunerExt* pe = mPrunerExt[boundsIndex];	// Can be NULL if the pruner has been removed
			if(pe && (!mPrunerFilter || mPrunerFilter->processPruner(boundsIndex)))
			{
				Pruner* pruner = pe->mPruner;
				if(!pruner->overlap(mQueryVolume, mCB))
					return false;
			}
			return true;
		}

		const PxArray<QuerySystem::PrunerExt*>&	mPrunerExt;
		const PrunerFilter*						mPrunerFilter;
		const ShapeData&						mQueryVolume;
		PrunerOverlapCallback&					mCB;

		PX_NOCOPY(LocalOverlapCB)
	};

	struct LocalSweepCB : PxBVH::RaycastCallback
	{
		LocalSweepCB(const PxArray<QuerySystem::PrunerExt*>& pruners, const PrunerFilter* prunerFilter, const ShapeData& queryVolume, const PxVec3& unitDir, PrunerRaycastCallback& cb) :
			mPrunerExt(pruners), mPrunerFilter(prunerFilter), mQueryVolume(queryVolume), mUnitDir(unitDir), mCB(cb)	{}

		virtual bool	reportHit(PxU32 boundsIndex, PxReal& distance)
		{
			QuerySystem::PrunerExt* pe = mPrunerExt[boundsIndex];	// Can be NULL if the pruner has been removed
			if(pe && (!mPrunerFilter || mPrunerFilter->processPruner(boundsIndex)))
			{
				Pruner* pruner = pe->mPruner;
				if(!pruner->sweep(mQueryVolume, mUnitDir, distance, mCB))
					return false;
			}
			return true;
		}

		const PxArray<QuerySystem::PrunerExt*>&	mPrunerExt;
		const PrunerFilter*						mPrunerFilter;
		const ShapeData&						mQueryVolume;
		const PxVec3&							mUnitDir;
		PrunerRaycastCallback&					mCB;

		PX_NOCOPY(LocalSweepCB)
	};
}

void QuerySystem::raycast(const PxVec3& origin, const PxVec3& unitDir, float& inOutDistance, PrunerRaycastCallback& cb, const PrunerFilter* prunerFilter) const
{
	if(mTreeOfPruners)
	{
		LocalRaycastCB localCB(mPrunerExt, prunerFilter, origin, unitDir, cb);
		mTreeOfPruners->raycast(origin, unitDir, inOutDistance, localCB, PxGeometryQueryFlag::Enum(0));
	}
	else
	{
		const PxU32 nb = mPrunerExt.size();
		for(PxU32 i=0;i<nb;i++)
		{
			PrunerExt* pe = mPrunerExt[i];	// Can be NULL if the pruner has been removed
			if(!pe)
				continue;

			if(!prunerFilter || prunerFilter->processPruner(i))
			{
				Pruner* pruner = pe->mPruner;
				if(!pruner->raycast(origin, unitDir, inOutDistance, cb))
					return;
			}
		}
	}
}

void QuerySystem::overlap(const ShapeData& queryVolume, PrunerOverlapCallback& cb, const PrunerFilter* prunerFilter) const
{
	if(mTreeOfPruners)
	{
		LocalOverlapCB localCB(mPrunerExt, prunerFilter, queryVolume, cb);
		mTreeOfPruners->overlap(queryVolume, localCB, PxGeometryQueryFlag::Enum(0));
	}
	else
	{
		const PxU32 nb = mPrunerExt.size();
		for(PxU32 i=0;i<nb;i++)
		{
			PrunerExt* pe = mPrunerExt[i];	// Can be NULL if the pruner has been removed
			if(!pe)
				continue;

			if(!prunerFilter || prunerFilter->processPruner(i))
			{
				Pruner* pruner = pe->mPruner;
				if(!pruner->overlap(queryVolume, cb))
					return;
			}
		}
	}
}

void QuerySystem::sweep(const ShapeData& queryVolume, const PxVec3& unitDir, float& inOutDistance, PrunerRaycastCallback& cb, const PrunerFilter* prunerFilter) const
{
	if(mTreeOfPruners)
	{
		LocalSweepCB localCB(mPrunerExt, prunerFilter, queryVolume, unitDir, cb);
		mTreeOfPruners->sweep(queryVolume, unitDir, inOutDistance, localCB, PxGeometryQueryFlag::Enum(0));
	}
	else
	{
		const PxU32 nb = mPrunerExt.size();
		for(PxU32 i=0;i<nb;i++)
		{
			PrunerExt* pe = mPrunerExt[i];	// Can be NULL if the pruner has been removed
			if(!pe)
				continue;

			if(!prunerFilter || prunerFilter->processPruner(i))
			{
				Pruner* pruner = pe->mPruner;
				if(!pruner->sweep(queryVolume, unitDir, inOutDistance, cb))
					return;
			}
		}
	}
}

void QuerySystem::createTreeOfPruners()
{
	PX_PROFILE_ZONE("QuerySystem.createTreeOfPruners", mContextID);

	PX_DELETE(mTreeOfPruners);

	mTreeOfPruners = PX_NEW(BVH)(NULL);

	const PxU32 nb = mPrunerExt.size();

	PxBounds3* prunerBounds = reinterpret_cast<PxBounds3*>(PxAlloca(sizeof(PxBounds3)*(nb+1)));

	PxU32 nbBounds = 0;
	for(PxU32 i=0;i<nb;i++)
	{
		PrunerExt* pe = mPrunerExt[i];
		Pruner* pruner = pe->mPruner;
		if(pruner)
			pruner->getGlobalBounds(prunerBounds[nbBounds++]);
	}

	mTreeOfPruners->init(nbBounds, NULL, prunerBounds, sizeof(PxBounds3), BVH_SPLATTER_POINTS, 1, 0.01f);
}
