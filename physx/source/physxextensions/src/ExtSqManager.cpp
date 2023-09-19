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

#include "ExtSqManager.h"
	#define SQ_DEBUG_VIZ_STATIC_COLOR	PxU32(PxDebugColor::eARGB_BLUE)
	#define SQ_DEBUG_VIZ_DYNAMIC_COLOR	PxU32(PxDebugColor::eARGB_RED)
	#define SQ_DEBUG_VIZ_STATIC_COLOR2	PxU32(PxDebugColor::eARGB_DARKBLUE)
	#define SQ_DEBUG_VIZ_DYNAMIC_COLOR2	PxU32(PxDebugColor::eARGB_DARKRED)
	#define SQ_DEBUG_VIZ_COMPOUND_COLOR	PxU32(PxDebugColor::eARGB_MAGENTA)
#include "GuBounds.h"

using namespace physx;
using namespace Sq;
using namespace Gu;

///////////////////////////////////////////////////////////////////////////////

#include "SqFactory.h"
#include "common/PxProfileZone.h"
#include "common/PxRenderBuffer.h"
#include "GuBVH.h"
#include "foundation/PxAlloca.h"

// PT: this is a customized version of physx::Sq::PrunerManager that supports more than 2 hardcoded pruners.
// It might not be possible to support the whole PxSceneQuerySystem API with an arbitrary number of pruners.

ExtPrunerManager::ExtPrunerManager(PxU64 contextID, float inflation, const Adapter& adapter, bool usesTreeOfPruners) :
	mAdapter				(adapter),
	mTreeOfPruners			(NULL),
	mContextID				(contextID),
	mStaticTimestamp		(0),
	mRebuildRateHint		(100),
	mInflation				(inflation),
	mPrunerNeedsUpdating	(false),
	mTimestampNeedsUpdating	(false),
	mUsesTreeOfPruners		(usesTreeOfPruners)
{
	mCompoundPrunerExt.mPruner = createCompoundPruner(contextID);
}

ExtPrunerManager::~ExtPrunerManager()
{
	PX_DELETE(mTreeOfPruners);

	const PxU32 nb = mPrunerExt.size();
	for(PxU32 i=0;i<nb;i++)
	{
		PrunerExt* pe = mPrunerExt[i];
		PX_DELETE(pe);
	}
}

PxU32 ExtPrunerManager::addPruner(Pruner* pruner, PxU32 preallocated)
{
	const PxU32 index = mPrunerExt.size();
	PrunerExt* pe = PX_NEW(PrunerExt);
	pe->init(pruner);
	if(preallocated)
		pe->preallocate(preallocated);
	mPrunerExt.pushBack(pe);
	return index;
}

void ExtPrunerManager::preallocate(PxU32 prunerIndex, PxU32 nbShapes)
{
	const bool preallocateCompoundPruner = prunerIndex==0xffffffff;
	if(preallocateCompoundPruner)
	{
		mCompoundPrunerExt.preallocate(nbShapes);
	}
	else
	{
		if(prunerIndex>=mPrunerExt.size())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "Invalid pruner index");
			return;
		}

		mPrunerExt[prunerIndex]->preallocate(nbShapes);
	}
}

void ExtPrunerManager::flushMemory()
{
	const PxU32 nb = mPrunerExt.size();
	for(PxU32 i=0;i<nb;i++)
	{
		PrunerExt* pe = mPrunerExt[i];
		pe->flushMemory();
	}

	mCompoundPrunerExt.flushMemory();
}

PrunerHandle ExtPrunerManager::addPrunerShape(const PrunerPayload& payload, PxU32 prunerIndex, bool dynamic, PrunerCompoundId compoundId, const PxBounds3& bounds, const PxTransform& transform, bool hasPruningStructure)
{
	if(compoundId==INVALID_COMPOUND_ID && prunerIndex>=mPrunerExt.size())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "Invalid pruner index");
		return INVALID_PRUNERHANDLE;
	}

	mPrunerNeedsUpdating = true;

	if(!dynamic)
		invalidateStaticTimestamp();

	PrunerHandle handle;
	if(compoundId == INVALID_COMPOUND_ID)
	{
		PX_ASSERT(prunerIndex<mPrunerExt.size());
		PX_ASSERT(mPrunerExt[prunerIndex]->pruner());
		mPrunerExt[prunerIndex]->pruner()->addObjects(&handle, &bounds, &payload, &transform, 1, hasPruningStructure);
		//mPrunerExt[prunerIndex].growDirtyList(handle);
	}
	else
	{
		PX_ASSERT(mCompoundPrunerExt.pruner());
		mCompoundPrunerExt.pruner()->addObject(compoundId, handle, bounds, payload, transform);
	}

	return handle;
}

void ExtPrunerManager::removePrunerShape(PxU32 prunerIndex, bool dynamic, PrunerCompoundId compoundId, PrunerHandle shapeHandle, PrunerPayloadRemovalCallback* removalCallback)
{
	if(compoundId==INVALID_COMPOUND_ID && prunerIndex>=mPrunerExt.size())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "Invalid pruner index");
		return;
	}

	mPrunerNeedsUpdating = true;

	if(!dynamic)
		invalidateStaticTimestamp();

	if(compoundId == INVALID_COMPOUND_ID)
	{
		PX_ASSERT(prunerIndex<mPrunerExt.size());
		PX_ASSERT(mPrunerExt[prunerIndex]->pruner());

		mPrunerExt[prunerIndex]->removeFromDirtyList(shapeHandle);
		mPrunerExt[prunerIndex]->pruner()->removeObjects(&shapeHandle, 1, removalCallback);
	}
	else
	{
		mCompoundPrunerExt.removeFromDirtyList(compoundId, shapeHandle);
		mCompoundPrunerExt.pruner()->removeObject(compoundId, shapeHandle, removalCallback);
	}
}

void ExtPrunerManager::markForUpdate(PxU32 prunerIndex, bool dynamic, PrunerCompoundId compoundId, PrunerHandle shapeHandle, const PxTransform& transform)
{ 
	if(compoundId==INVALID_COMPOUND_ID && prunerIndex>=mPrunerExt.size())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "Invalid pruner index");
		return;
	}

	mPrunerNeedsUpdating = true;

	if(!dynamic)
		invalidateStaticTimestamp();

	if(compoundId == INVALID_COMPOUND_ID)
	{
		PX_ASSERT(prunerIndex<mPrunerExt.size());
		PX_ASSERT(mPrunerExt[prunerIndex]->pruner());

		// PT: TODO: at this point do we still need a dirty list? we could just update the bounds directly?
		mPrunerExt[prunerIndex]->addToDirtyList(shapeHandle, dynamic, transform);
	}
	else
		mCompoundPrunerExt.addToDirtyList(compoundId, shapeHandle, transform);
}

void ExtPrunerManager::setDynamicTreeRebuildRateHint(PxU32 rebuildRateHint)
{
	mRebuildRateHint = rebuildRateHint;

	// PT: we are still using the same rebuild hint for all pruners here, which may or may
	// not make sense. We could also have a different build rate for each pruner.

	const PxU32 nb = mPrunerExt.size();
	for(PxU32 i=0;i<nb;i++)
	{
		PrunerExt* pe = mPrunerExt[i];
		Pruner* pruner = pe->pruner();
		if(pruner && pruner->isDynamic())
			static_cast<DynamicPruner*>(pruner)->setRebuildRateHint(rebuildRateHint);
	}
}

void ExtPrunerManager::afterSync(bool buildStep, bool commit)
{
	PX_PROFILE_ZONE("Sim.sceneQueryBuildStep", mContextID);

	if(!buildStep && !commit)
	{
		mPrunerNeedsUpdating = true;
		return;
	}

	// flush user modified objects
	flushShapes();

	// PT: TODO: with N pruners this part would be worth multi-threading

	const PxU32 nb = mPrunerExt.size();
	for(PxU32 i=0;i<nb;i++)
	{
		PrunerExt* pe = mPrunerExt[i];
		Pruner* pruner = pe->pruner();
		if(pruner)
		{
			if(pruner->isDynamic())
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

void ExtPrunerManager::flushShapes()
{
	PX_PROFILE_ZONE("SceneQuery.flushShapes", mContextID);

	// must already have acquired writer lock here

	const float inflation = 1.0f + mInflation;

	bool mustInvalidateStaticTimestamp = false;
	const PxU32 nb = mPrunerExt.size();
	for(PxU32 i=0;i<nb;i++)
	{
		PrunerExt* pe = mPrunerExt[i];
		if(pe->processDirtyList(i, mAdapter, inflation))
			mustInvalidateStaticTimestamp = true;
	}

	if(mustInvalidateStaticTimestamp)
		invalidateStaticTimestamp();

	mCompoundPrunerExt.flushShapes(mAdapter, inflation);
}

void ExtPrunerManager::flushUpdates()
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

			const PxU32 nb = mPrunerExt.size();
			for(PxU32 i=0;i<nb;i++)
			{
				PrunerExt* pe = mPrunerExt[i];
				if(pe->pruner())
					pe->pruner()->commit();
			}

			if(mUsesTreeOfPruners)
				createTreeOfPruners();

			PxMemoryBarrier();
			mPrunerNeedsUpdating = false;
		}
		mSQLock.unlock();
	}
}

void ExtPrunerManager::forceRebuildDynamicTree(PxU32 prunerIndex)
{
	// PT: beware here when called from the PxScene, the prunerIndex may not match

	PX_PROFILE_ZONE("SceneQuery.forceDynamicTreeRebuild", mContextID);

	if(prunerIndex>=mPrunerExt.size())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "Invalid pruner index");
		return;
	}

	PxMutex::ScopedLock lock(mSQLock);

	PrunerExt* pe = mPrunerExt[prunerIndex];
	Pruner* pruner = pe->pruner();

	if(pruner && pruner->isDynamic())
	{
		static_cast<DynamicPruner*>(pruner)->purge();
		static_cast<DynamicPruner*>(pruner)->commit();
	}
}

void* ExtPrunerManager::prepareSceneQueriesUpdate(PxU32 prunerIndex)
{
	// PT: beware here when called from the PxScene, the prunerIndex may not match

	if(prunerIndex>=mPrunerExt.size())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "Invalid pruner index");
		return NULL;
	}

	PX_ASSERT(mPrunerExt[prunerIndex]->pruner());

	bool retVal = false;
	Pruner* pruner = mPrunerExt[prunerIndex]->pruner();
	if(pruner && pruner->isDynamic())
		retVal = static_cast<DynamicPruner*>(pruner)->prepareBuild();

	return retVal ? pruner : NULL;
}

void ExtPrunerManager::sceneQueryBuildStep(void* handle)
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

void ExtPrunerManager::visualize(PxU32 prunerIndex, PxRenderOutput& out) const
{
	// PT: beware here when called from the PxScene, the prunerIndex may not match
	// This is quite awkward and should be improved.

	// PT: problem here is that this function will be called by the regular PhysX scene when plugged to its PxSceneDesc,
	// and the calling code only understands the regular PhysX pruners.

	const bool visualizeCompoundPruner = prunerIndex==0xffffffff;
	if(visualizeCompoundPruner)
	{
		const CompoundPruner* cp = mCompoundPrunerExt.pruner();
		if(cp)
			cp->visualizeEx(out, SQ_DEBUG_VIZ_COMPOUND_COLOR, true, true);
	}
	else
	{
		if(prunerIndex>=mPrunerExt.size())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "Invalid pruner index");
			return;
		}

		PrunerExt* pe = mPrunerExt[prunerIndex];
		Pruner* pruner = pe->pruner();
		if(pruner)
		{
			// PT: TODO: this doesn't really work: the static color was for static shapes but they
			// could still be stored in a dynamic pruner. So this code doesn't use a color scheme
			// consistent with what we use in the default code.
			if(pruner->isDynamic())
			{
				//if(visDynamic)
					pruner->visualize(out, SQ_DEBUG_VIZ_DYNAMIC_COLOR, SQ_DEBUG_VIZ_DYNAMIC_COLOR2);
			}
			else
			{
				//if(visStatic)
					pruner->visualize(out, SQ_DEBUG_VIZ_STATIC_COLOR, SQ_DEBUG_VIZ_STATIC_COLOR2);
			}
		}
	}
}

void ExtPrunerManager::shiftOrigin(const PxVec3& shift)
{
	const PxU32 nb = mPrunerExt.size();
	for(PxU32 i=0;i<nb;i++)
	{
		PrunerExt* pe = mPrunerExt[i];
		Pruner* pruner = pe->pruner();
		if(pruner)
			pruner->shiftOrigin(shift);
	}

	mCompoundPrunerExt.pruner()->shiftOrigin(shift);
}

void ExtPrunerManager::addCompoundShape(const PxBVH& pxbvh, PrunerCompoundId compoundId, const PxTransform& compoundTransform, PrunerHandle* prunerHandle, const PrunerPayload* payloads, const PxTransform* transforms, bool isDynamic)
{
	const Gu::BVH& bvh = static_cast<const Gu::BVH&>(pxbvh);

	PX_ASSERT(mCompoundPrunerExt.mPruner);
	mCompoundPrunerExt.mPruner->addCompound(prunerHandle, bvh, compoundId, compoundTransform, isDynamic, payloads, transforms);
	if(!isDynamic)
		invalidateStaticTimestamp();
}

void ExtPrunerManager::updateCompoundActor(PrunerCompoundId compoundId, const PxTransform& compoundTransform)
{	
	PX_ASSERT(mCompoundPrunerExt.mPruner);
	const bool isDynamic = mCompoundPrunerExt.mPruner->updateCompound(compoundId, compoundTransform);
	if(!isDynamic)
		invalidateStaticTimestamp();
}

void ExtPrunerManager::removeCompoundActor(PrunerCompoundId compoundId, PrunerPayloadRemovalCallback* removalCallback)
{
	PX_ASSERT(mCompoundPrunerExt.mPruner);
	const bool isDynamic = mCompoundPrunerExt.mPruner->removeCompound(compoundId, removalCallback);
	if(!isDynamic)
		invalidateStaticTimestamp();
}

void ExtPrunerManager::sync(PxU32 prunerIndex, const PrunerHandle* handles, const PxU32* boundsIndices, const PxBounds3* bounds, const PxTransform32* transforms, PxU32 count, const PxBitMap& ignoredIndices)
{
	if(!count)
		return;

	if(prunerIndex>=mPrunerExt.size())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "Invalid pruner index");
		return;
	}

	Pruner* pruner = getPruner(prunerIndex);
	if(!pruner)
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
				pruner->updateObjects(handles + startIndex, numIndices, mInflation, boundsIndices + startIndex, bounds, transforms);
				numIndices = 0;
				startIndex = i + 1;
			}
			else
				numIndices++;
		}
		// PT: we fallback to the next line on purpose - no "else"
	}

	pruner->updateObjects(handles + startIndex, numIndices, mInflation, boundsIndices + startIndex, bounds, transforms);
}

PxU32 ExtPrunerManager::startCustomBuildstep()
{
	PX_PROFILE_ZONE("SceneQuery.startCustomBuildstep", mContextID);

	// flush user modified objects
	//flushShapes();
	{
		PX_PROFILE_ZONE("SceneQuery.flushShapes", mContextID);
		// PT: only flush the compound pruner synchronously
		// must already have acquired writer lock here
		const float inflation = 1.0f + mInflation;
		mCompoundPrunerExt.flushShapes(mAdapter, inflation);
	}

	mTimestampNeedsUpdating = false;

	return mPrunerExt.size();
}

void ExtPrunerManager::customBuildstep(PxU32 index)
{
	PX_PROFILE_ZONE("SceneQuery.customBuildstep", mContextID);

	PX_ASSERT(index<mPrunerExt.size());

	PrunerExt* pe = mPrunerExt[index];
	Pruner* pruner = pe->pruner();

	//void ExtPrunerManager::flushShapes()
	{
		PX_PROFILE_ZONE("SceneQuery.flushShapes", mContextID);
		// must already have acquired writer lock here
		const float inflation = 1.0f + mInflation;
		if(pe->processDirtyList(index, mAdapter, inflation))
			mTimestampNeedsUpdating = true;
	}

	if(pruner)
	{
		if(pruner->isDynamic())
			static_cast<DynamicPruner*>(pruner)->buildStep(true);	// PT: "true" because that parameter was made for PxSceneQuerySystem::sceneQueryBuildStep(), not us

		pruner->commit();
	}
}

void ExtPrunerManager::finishCustomBuildstep()
{
	PX_PROFILE_ZONE("SceneQuery.finishCustomBuildstep", mContextID);

	if(mUsesTreeOfPruners)
		createTreeOfPruners();

	mPrunerNeedsUpdating = false;
	if(mTimestampNeedsUpdating)
		invalidateStaticTimestamp();
}

void ExtPrunerManager::createTreeOfPruners()
{
	PX_PROFILE_ZONE("SceneQuery.createTreeOfPruners", mContextID);

	PX_DELETE(mTreeOfPruners);

	mTreeOfPruners = PX_NEW(BVH)(NULL);

	const PxU32 nb = mPrunerExt.size();

	PxBounds3* prunerBounds = reinterpret_cast<PxBounds3*>(PxAlloca(sizeof(PxBounds3)*(nb+1)));

	PxU32 nbBounds = 0;
	for(PxU32 i=0;i<nb;i++)
	{
		PrunerExt* pe = mPrunerExt[i];
		Pruner* pruner = pe->pruner();
		if(pruner)
			pruner->getGlobalBounds(prunerBounds[nbBounds++]);
	}

	mTreeOfPruners->init(nbBounds, NULL, prunerBounds, sizeof(PxBounds3), BVH_SPLATTER_POINTS, 1, 0.01f);
}
