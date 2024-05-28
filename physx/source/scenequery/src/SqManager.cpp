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

// PT: SQ-API LEVEL 2 (Level 1 = SqPruner.h)
// PT: this file is part of a "high-level" set of files within Sq. The SqPruner API doesn't rely on them.
// PT: this should really be at Np level but moving it to Sq allows us to share it.

#include "SqManager.h"
#include "GuSqInternal.h"
#include "GuBounds.h"

using namespace physx;
using namespace Sq;
using namespace Gu;

PrunerExt::PrunerExt() : mPruner(NULL), mDirtyList("SQmDirtyList"), mDirtyStatic(false)
{
}

PrunerExt::~PrunerExt()
{
	PX_DELETE(mPruner);
}

void PrunerExt::init(Pruner* pruner)
{
	mPruner = pruner;
}

void PrunerExt::preallocate(PxU32 nbShapes)
{
//	if(nbShapes > mDirtyMap.size())
//		mDirtyMap.resize(nbShapes);

	if(mPruner)
		mPruner->preallocate(nbShapes);
}

void PrunerExt::flushMemory()
{
	if(!mDirtyList.size())
		mDirtyList.reset();

	// PT: TODO: flush bitmap here

	// PT: TODO: flush pruner here?
}

void PrunerExt::addToDirtyList(PrunerHandle handle, bool dynamic, const PxTransform& transform)
{
	if(mPruner)
		mPruner->setTransform(handle, transform);

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

	if(!dirtyMap.test(handle))
	{
		dirtyMap.set(handle);
		mDirtyList.pushBack(handle);
	}

	if(!dynamic)
		mDirtyStatic = true;
}

void PrunerExt::removeFromDirtyList(PrunerHandle handle)
{
	PxBitMap& dirtyMap = mDirtyMap;
//	if(dirtyMap.test(handle))
	if(dirtyMap.boundedTest(handle))
	{
		dirtyMap.reset(handle);
		mDirtyList.findAndReplaceWithLast(handle);
	}

	// PT: if we remove the object that made us set mDirtyStatic to true, tough luck,
	// we don't bother fixing that bool here. It's going to potentially cause an
	// unnecessary update of the character controller's caches, which is not a big deal.
}

bool PrunerExt::processDirtyList(PxU32 index, const Adapter& adapter, float inflation)
{
	const PxU32 numDirtyList = mDirtyList.size();
	if(!numDirtyList)
		return false;
	const PrunerHandle* const prunerHandles = mDirtyList.begin();

	for(PxU32 i=0; i<numDirtyList; i++)
	{
		const PrunerHandle handle = prunerHandles[i];
		mDirtyMap.reset(handle);

		// PT: we compute the new bounds and store them directly in the pruner structure to avoid copies. We delay the updateObjects() call
		// to take advantage of batching.
		PX_UNUSED(index);

		PrunerPayloadData ppd;
		const PrunerPayload& pp = mPruner->getPayloadData(handle, &ppd);

		computeBounds(*ppd.mBounds, adapter.getGeometry(pp), *ppd.mTransform, 0.0f, inflation);
	}
	// PT: batch update happens after the loop instead of once per loop iteration
	mPruner->updateObjects(prunerHandles, numDirtyList);
	mDirtyList.clear();

	const bool ret = mDirtyStatic;
	mDirtyStatic = false;
	return ret;
}

// PT: TODO: re-inline this
/*void PrunerExt::growDirtyList(PrunerHandle handle)
{
	// pruners must either provide indices in order or reuse existing indices, so this 'if' is enough to ensure we have space for the new handle
	// PT: TODO: fix this. There is just no need for any of it. The pruning pool itself could support the feature for free, similar to what we do
	// in MBP. There would be no need for the bitmap or the dirty list array. However doing this through the virtual interface would be clumsy,
	// adding the cost of virtual calls for very cheap & simple operations. It would be a lot easier to drop it and go back to what we had before.

	PxBitMap& dirtyMap = mDirtyMap;
	if(dirtyMap.size() <= handle)
		dirtyMap.resize(PxMax<PxU32>(dirtyMap.size() * 2, 1024));
	PX_ASSERT(handle<dirtyMap.size());
	dirtyMap.reset(handle);
}*/

///////////////////////////////////////////////////////////////////////////////

CompoundPrunerExt::CompoundPrunerExt() :
	mPruner	(NULL)
{
}

CompoundPrunerExt::~CompoundPrunerExt()
{
	PX_DELETE(mPruner);
}

void CompoundPrunerExt::preallocate(PxU32 nbShapes)
{
//	if(nbShapes > mDirtyList.size())
//		mDirtyList.reserve(nbShapes);

	if(mPruner)
		mPruner->preallocate(nbShapes);
}

void CompoundPrunerExt::flushMemory()
{
	if(!mDirtyList.size())
		mDirtyList.clear();
}

void CompoundPrunerExt::flushShapes(const Adapter& adapter, float inflation)
{
	const PxU32 numDirtyList = mDirtyList.size();
	if(!numDirtyList)
		return;

	const CompoundPair* const compoundPairs = mDirtyList.getEntries();

	for(PxU32 i=0; i<numDirtyList; i++)
	{
		const PrunerHandle handle = compoundPairs[i].second;
		const PrunerCompoundId compoundId = compoundPairs[i].first;		

		// PT: we compute the new bounds and store them directly in the pruner structure to avoid copies. We delay the updateObjects() call
		// to take advantage of batching.
		PrunerPayloadData ppd;
		const PrunerPayload& pp = mPruner->getPayloadData(handle, compoundId, &ppd);

		computeBounds(*ppd.mBounds, adapter.getGeometry(pp), *ppd.mTransform, 0.0f, inflation);

		// A.B. not very effective, we might do better here
		mPruner->updateObjectAfterManualBoundsUpdates(compoundId, handle);
	}
		
	mDirtyList.clear();
}

void CompoundPrunerExt::addToDirtyList(PrunerCompoundId compoundId, PrunerHandle handle, const PxTransform& transform)
{
	if(mPruner)
		mPruner->setTransform(handle, compoundId, transform);

	mDirtyList.insert(CompoundPair(compoundId, handle));
}

void CompoundPrunerExt::removeFromDirtyList(PrunerCompoundId compoundId, PrunerHandle handle)
{
	mDirtyList.erase(CompoundPair(compoundId, handle));
}

///////////////////////////////////////////////////////////////////////////////

#include "SqFactory.h"
#include "common/PxProfileZone.h"
#include "common/PxRenderBuffer.h"
#include "GuBVH.h"
#include "foundation/PxAlloca.h"
#include "PxSceneDesc.h"	// PT: for PxSceneLimits TODO: remove

namespace
{
	enum PxScenePrunerIndex
	{
		PX_SCENE_PRUNER_STATIC		= 0,
		PX_SCENE_PRUNER_DYNAMIC		= 1,
		PX_SCENE_COMPOUND_PRUNER	= 0xffffffff,
	};
}

PrunerManager::PrunerManager(	PxU64 contextID, Pruner* staticPruner, Pruner* dynamicPruner,
								PxU32 dynamicTreeRebuildRateHint, float inflation,
								const PxSceneLimits& limits, const Adapter& adapter) :
	mAdapter			(adapter),
	mContextID			(contextID),
	mStaticTimestamp	(0),
	mInflation			(inflation)
{
	mPrunerExt[PruningIndex::eSTATIC].init(staticPruner);
	mPrunerExt[PruningIndex::eDYNAMIC].init(dynamicPruner);

	setDynamicTreeRebuildRateHint(dynamicTreeRebuildRateHint);

	mCompoundPrunerExt.mPruner = createCompoundPruner(contextID);

	preallocate(PruningIndex::eSTATIC, limits.maxNbStaticShapes);
	preallocate(PruningIndex::eDYNAMIC, limits.maxNbDynamicShapes);
	preallocate(PxU32(PX_SCENE_COMPOUND_PRUNER), 32);

	mPrunerNeedsUpdating = false;
}

PrunerManager::~PrunerManager()
{
}

void PrunerManager::preallocate(PxU32 prunerIndex, PxU32 nbShapes)
{
	if(prunerIndex==PruningIndex::eSTATIC)
		mPrunerExt[PruningIndex::eSTATIC].preallocate(nbShapes);
	else if(prunerIndex==PruningIndex::eDYNAMIC)
		mPrunerExt[PruningIndex::eDYNAMIC].preallocate(nbShapes);
	else if(prunerIndex==PX_SCENE_COMPOUND_PRUNER)
		mCompoundPrunerExt.preallocate(nbShapes);
}

void PrunerManager::flushMemory()
{
	for(PxU32 i=0;i<PruningIndex::eCOUNT;i++)
		mPrunerExt[i].flushMemory();

	mCompoundPrunerExt.flushMemory();
}

PrunerData PrunerManager::addPrunerShape(const PrunerPayload& payload, bool dynamic, PrunerCompoundId compoundId, const PxBounds3& bounds, const PxTransform& transform, bool hasPruningStructure)
{
	mPrunerNeedsUpdating = true;

	const PxU32 index = PxU32(dynamic);

	if(!index)
		invalidateStaticTimestamp();

	PrunerHandle handle;
	if(compoundId == INVALID_COMPOUND_ID)
	{
		PX_ASSERT(mPrunerExt[index].pruner());
		mPrunerExt[index].pruner()->addObjects(&handle, &bounds, &payload, &transform, 1, hasPruningStructure);
		//mPrunerExt[index].growDirtyList(handle);
	}
	else
	{
		PX_ASSERT(mCompoundPrunerExt.pruner());
		mCompoundPrunerExt.pruner()->addObject(compoundId, handle, bounds, payload, transform);
	}

	return createPrunerData(index, handle);
}

void PrunerManager::removePrunerShape(PrunerCompoundId compoundId, PrunerData data, PrunerPayloadRemovalCallback* removalCallback)
{
	mPrunerNeedsUpdating = true;
	const PxU32 index = getPrunerIndex(data);
	const PrunerHandle handle = getPrunerHandle(data);

	if(!index)
		invalidateStaticTimestamp();

	if(compoundId == INVALID_COMPOUND_ID)
	{
		PX_ASSERT(mPrunerExt[index].pruner());

		mPrunerExt[index].removeFromDirtyList(handle);
		mPrunerExt[index].pruner()->removeObjects(&handle, 1, removalCallback);
	}
	else
	{
		mCompoundPrunerExt.removeFromDirtyList(compoundId, handle);
		mCompoundPrunerExt.pruner()->removeObject(compoundId, handle, removalCallback);
	}
}

void PrunerManager::markForUpdate(PrunerCompoundId compoundId, PrunerData data, const PxTransform& transform)
{ 
	mPrunerNeedsUpdating = true;
	const PxU32 index = getPrunerIndex(data);
	const PrunerHandle handle = getPrunerHandle(data);

	if(!index)
		invalidateStaticTimestamp();

	if(compoundId == INVALID_COMPOUND_ID)
		// PT: TODO: at this point do we still need a dirty list? we could just update the bounds directly?
		mPrunerExt[index].addToDirtyList(handle, index!=0, transform);
	else
		mCompoundPrunerExt.addToDirtyList(compoundId, handle, transform);
}

void PrunerManager::setDynamicTreeRebuildRateHint(PxU32 rebuildRateHint)
{
	mRebuildRateHint = rebuildRateHint;

	for(PxU32 i=0;i<PruningIndex::eCOUNT;i++)
	{
		Pruner* pruner = mPrunerExt[i].pruner();
		if(pruner && pruner->isDynamic())
			static_cast<DynamicPruner*>(pruner)->setRebuildRateHint(rebuildRateHint);
	}
}

void PrunerManager::afterSync(bool buildStep, bool commit)
{
	PX_PROFILE_ZONE("Sim.sceneQueryBuildStep", mContextID);

	if(!buildStep && !commit)
	{
		mPrunerNeedsUpdating = true;
		return;
	}

	// flush user modified objects
	flushShapes();

	for(PxU32 i=0; i<PruningIndex::eCOUNT; i++)
	{
		Pruner* pruner = mPrunerExt[i].pruner();
		if(pruner)
		{
			if(pruner->isDynamic())
				static_cast<DynamicPruner*>(pruner)->buildStep(true);

			if(commit)
				pruner->commit();
		}
	}

	mPrunerNeedsUpdating = !commit;
}

void PrunerManager::flushShapes()
{
	PX_PROFILE_ZONE("SceneQuery.flushShapes", mContextID);

	// must already have acquired writer lock here

	const float inflation = 1.0f + mInflation;

	bool mustInvalidateStaticTimestamp = false;
	for(PxU32 i=0; i<PruningIndex::eCOUNT; i++)
	{
		if(mPrunerExt[i].processDirtyList(i, mAdapter, inflation))
			mustInvalidateStaticTimestamp = true;
	}

	if(mustInvalidateStaticTimestamp)
		invalidateStaticTimestamp();

	mCompoundPrunerExt.flushShapes(mAdapter, inflation);
}

void PrunerManager::flushUpdates()
{
	PX_PROFILE_ZONE("SceneQuery.flushUpdates", mContextID);

	if(mPrunerNeedsUpdating)
	{
		// no need to take lock if manual sq update is enabled
		// as flushUpdates will only be called from NpScene::flushQueryUpdates()
		mSQLock.lock();

		if(mPrunerNeedsUpdating)
		{
			flushShapes();

			for(PxU32 i=0; i<PruningIndex::eCOUNT; i++)
				if(mPrunerExt[i].pruner())
					mPrunerExt[i].pruner()->commit();

			PxMemoryBarrier();
			mPrunerNeedsUpdating = false;
		}
		mSQLock.unlock();
	}
}

void PrunerManager::forceRebuildDynamicTree(PxU32 prunerIndex)
{
	PX_PROFILE_ZONE("SceneQuery.forceDynamicTreeRebuild", mContextID);

	PxMutex::ScopedLock lock(mSQLock);
	Pruner* pruner = mPrunerExt[prunerIndex].pruner();
	if(pruner && pruner->isDynamic())
	{
		static_cast<DynamicPruner*>(pruner)->purge();
		static_cast<DynamicPruner*>(pruner)->commit();
	}
}

void* PrunerManager::prepareSceneQueriesUpdate(PruningIndex::Enum index)
{
	bool retVal = false;
	Pruner* pruner = mPrunerExt[index].pruner();
	if(pruner && pruner->isDynamic())
		retVal = static_cast<DynamicPruner*>(pruner)->prepareBuild();

	return retVal ? pruner : NULL;
}

void PrunerManager::sceneQueryBuildStep(void* handle)
{
	PX_PROFILE_ZONE("SceneQuery.sceneQueryBuildStep", mContextID);

	Pruner* pruner = reinterpret_cast<Pruner*>(handle);
	if(pruner && pruner->isDynamic())
	{
		const bool buildFinished = static_cast<DynamicPruner*>(pruner)->buildStep(false);
		if(buildFinished)
			mPrunerNeedsUpdating = true;
	}
}

// PT: TODO: revisit this. Perhaps it should be the user's responsibility to call the pruner's
// visualize functions directly, when & how he wants.
void PrunerManager::visualize(PxU32 prunerIndex, PxRenderOutput& out) const
{
	if(prunerIndex==PX_SCENE_PRUNER_STATIC)
	{
		if(getPruner(PruningIndex::eSTATIC))
			getPruner(PruningIndex::eSTATIC)->visualize(out, SQ_DEBUG_VIZ_STATIC_COLOR, SQ_DEBUG_VIZ_STATIC_COLOR2);
	}
	else if(prunerIndex==PX_SCENE_PRUNER_DYNAMIC)
	{
		if(getPruner(PruningIndex::eDYNAMIC))
			getPruner(PruningIndex::eDYNAMIC)->visualize(out, SQ_DEBUG_VIZ_DYNAMIC_COLOR, SQ_DEBUG_VIZ_DYNAMIC_COLOR2);
	}
	else if(prunerIndex==PX_SCENE_COMPOUND_PRUNER)
	{
		const CompoundPruner* cp = mCompoundPrunerExt.pruner();
		if(cp)
			cp->visualizeEx(out, SQ_DEBUG_VIZ_COMPOUND_COLOR, true, true);
	}
}

void PrunerManager::shiftOrigin(const PxVec3& shift)
{
	for(PxU32 i=0; i<PruningIndex::eCOUNT; i++)
		mPrunerExt[i].pruner()->shiftOrigin(shift);

	mCompoundPrunerExt.pruner()->shiftOrigin(shift);
}

void PrunerManager::addCompoundShape(const PxBVH& pxbvh, PrunerCompoundId compoundId, const PxTransform& compoundTransform, PrunerData* prunerData, const PrunerPayload* payloads, const PxTransform* transforms, bool isDynamic)
{
	const BVH& bvh = static_cast<const BVH&>(pxbvh);
	const PxU32 nbShapes = bvh.Gu::BVH::getNbBounds();

	PX_ALLOCA(res, PrunerHandle, nbShapes);

	PX_ASSERT(mCompoundPrunerExt.mPruner);
	mCompoundPrunerExt.mPruner->addCompound(res, bvh, compoundId, compoundTransform, isDynamic, payloads, transforms);
	const PxU32 index = PxU32(isDynamic);
	if(!index)
		invalidateStaticTimestamp();

	for(PxU32 i = 0; i < nbShapes; i++)
		prunerData[i] = createPrunerData(index, res[i]);
}

void PrunerManager::updateCompoundActor(PrunerCompoundId compoundId, const PxTransform& compoundTransform)
{	
	PX_ASSERT(mCompoundPrunerExt.mPruner);
	const bool isDynamic = mCompoundPrunerExt.mPruner->updateCompound(compoundId, compoundTransform);
	if(!isDynamic)
		invalidateStaticTimestamp();
}

void PrunerManager::removeCompoundActor(PrunerCompoundId compoundId, PrunerPayloadRemovalCallback* removalCallback)
{
	PX_ASSERT(mCompoundPrunerExt.mPruner);
	const bool isDynamic = mCompoundPrunerExt.mPruner->removeCompound(compoundId, removalCallback);
	if(!isDynamic)
		invalidateStaticTimestamp();
}

void PrunerManager::sync(const PrunerHandle* handles, const PxU32* boundsIndices, const PxBounds3* bounds, const PxTransform32* transforms, PxU32 count, const PxBitMap& ignoredIndices)
{
	if(!count)
		return;

	Pruner* dynamicPruner = getPruner(PruningIndex::eDYNAMIC);
	if(!dynamicPruner)
		return;

	PxU32 startIndex = 0;
	PxU32 numIndices = count;

	// if shape sim map is not empty, parse the indices and skip update for the dirty one
	if(ignoredIndices.count())
	{
		// PT: I think this codepath was used with SCB / buffered changes, but it's not needed anymore
		numIndices = 0;

		for(PxU32 i=0; i<count; i++)
		{
//			if(ignoredIndices.test(boundsIndices[i]))
			if(ignoredIndices.boundedTest(boundsIndices[i]))
			{
				dynamicPruner->updateObjects(handles + startIndex, numIndices, mInflation, boundsIndices + startIndex, bounds, transforms);
				numIndices = 0;
				startIndex = i + 1;
			}
			else
				numIndices++;
		}
		// PT: we fallback to the next line on purpose - no "else"
	}

	dynamicPruner->updateObjects(handles + startIndex, numIndices, mInflation, boundsIndices + startIndex, bounds, transforms);
}
