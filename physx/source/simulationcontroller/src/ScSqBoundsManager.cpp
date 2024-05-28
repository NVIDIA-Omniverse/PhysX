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

#include "GuPrunerTypedef.h"
#include "ScSqBoundsManager.h"
#include "ScBodySim.h"
#include "ScShapeSim.h"
#include "ScSqBoundsSync.h"
#include "common/PxProfileZone.h"

using namespace physx;
using namespace Sc;

#define INVALID_REF	ScPrunerHandle(Gu::INVALID_PRUNERHANDLE)

SqBoundsManager0::SqBoundsManager0() :
	mShapes			("SqBoundsManager::mShapes"),
	mRefs			("SqBoundsManager::mRefs"),
	mBoundsIndices	("SqBoundsManager::mBoundsIndices"),
	mRefless		("SqBoundsManager::mRefless")
{
}

void SqBoundsManager0::addSyncShape(ShapeSimBase& shape)
{
	PX_ASSERT(shape.getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE);
	PX_ASSERT(!shape.getBodySim()->usingSqKinematicTarget());
	PX_ASSERT(!shape.getBodySim()->isFrozen());

	const PxU32 id = mShapes.size();
	PX_ASSERT(id == mRefs.size());
	PX_ASSERT(id == mBoundsIndices.size());

	shape.setSqBoundsId(id);

	// PT: mShapes / mRefs / mBoundsIndices are "parallel arrays". These arrays are persistent.
	// mRefless is temporary/transient data to help populate mRefs each frame.
	// mRefs / mBoundsIndices will be ultimately passed to updateObjects, whose API dictates the layout here.
	// mShapes is not actually used for the sync, it's only here to be able to call setSqBoundsId in removeShape.

	mShapes.pushBack(static_cast<Sc::ShapeSim*>(&shape));
	mRefs.pushBack(INVALID_REF);
	mBoundsIndices.pushBack(shape.getElementID());
	mRefless.pushBack(static_cast<Sc::ShapeSim*>(&shape));
}

void SqBoundsManager0::removeSyncShape(ShapeSimBase& shape)
{
	const PxU32 id = shape.getSqBoundsId();
	PX_ASSERT(id!=PX_INVALID_U32);

	shape.setSqBoundsId(PX_INVALID_U32);
	mShapes[id] = mShapes.back();
	mBoundsIndices[id] = mBoundsIndices.back();
	mRefs[id] = mRefs.back();

	if(id+1 != mShapes.size())
		mShapes[id]->setSqBoundsId(id);

	mShapes.popBack();
	mRefs.popBack();
	mBoundsIndices.popBack();
}

void SqBoundsManager0::syncBounds(SqBoundsSync& sync, SqRefFinder& finder, const PxBounds3* bounds, const PxTransform32* transforms, PxU64 contextID, const PxBitMap& ignoredIndices)
{
	PX_PROFILE_ZONE("Sim.sceneQuerySyncBounds", contextID);
	PX_UNUSED(contextID);

#if PX_DEBUG
	for(PxU32 i=0;i<mShapes.size();i++)
	{
		const ShapeSimBase& shape = *mShapes[i];
		PX_UNUSED(shape);
		PX_ASSERT(shape.getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE);
		PX_ASSERT(!shape.getBodySim()->usingSqKinematicTarget());
		PX_ASSERT(!shape.getBodySim()->isFrozen());
	}
#endif

	ShapeSimBase*const * shapes = mRefless.begin();
	for(PxU32 i=0, size = mRefless.size();i<size;i++)
	{
		const PxU32 id = shapes[i]->getSqBoundsId();
		// PT:
		//
		// If id == PX_INVALID_U32, the shape has been removed and not re-added. Nothing to do in this case, we just ignore it.
		// This case didn't previously exist since mRefless only contained valid (added) shapes. But now we left removed shapes in the
		// structure, and these have an id == PX_INVALID_U32.
		//
		// Now if the id is valid but mRefs[id] == PX_INVALID_U32, this is a regular shape that has been added and not processed yet.
		// So we process it.
		//
		// Finally, if both id and mRefs[id] are not PX_INVALID_U32, this is a shape that has been added, removed, and re-added. The
		// array contains the same shape twice and we only need to process it once.
		if(id!=PX_INVALID_U32)
		{
			if(mRefs[id] == INVALID_REF)
			{
				PxU32 prunerIndex = 0xffffffff;
				mRefs[id] = finder.find(static_cast<PxRigidBody*>(shapes[i]->getBodySim()->getPxActor()), shapes[i]->getPxShape(), prunerIndex);
				PX_ASSERT(prunerIndex==1);
			}
		}
	}
	mRefless.clear();

	sync.sync(1, mRefs.begin(), mBoundsIndices.begin(), bounds, transforms, mShapes.size(), ignoredIndices);
}












// PT: we need to change the code so that the shape is added to the proper array during syncBounds, not during addSyncShape,
// because the pruner index is not known in addSyncShape. We could perhaps call the ref-finder directly in addSyncShape, but
// it would impose an order on the calls (the shape would need to be added to the pruners before addSyncShape is called. There's
// no such requirement with the initial code).
//
// Instead we do this:
// - in addSyncShape we just add the shape to a "waiting room", that's all.
// - adding the shape to the proper array is delayed until syncBounds. That way the prunerIndex will be available. Also we could
//   then take advantage of batching, since all shapes are processed/added at the same time.
// - the only catch is that we need to ensure the previous edge-cases are still properly handled, i.e. when a shape is added then
//   removed before sync is called, etc.
//

SqBoundsManagerEx::SqBoundsManagerEx() :
	mWaitingRoom		("SqBoundsManagerEx::mWaitingRoom"),
	mPrunerSyncData		(NULL),
	mPrunerSyncDataSize	(0)
{
}

SqBoundsManagerEx::~SqBoundsManagerEx()
{
	const PxU32 nbToGo = mPrunerSyncDataSize;
	for(PxU32 i=0;i<nbToGo;i++)
	{
		PrunerSyncData* psd = mPrunerSyncData[i];
		PX_DELETE(psd);
	}
	PX_FREE(mPrunerSyncData);
}

void SqBoundsManagerEx::resize(PxU32 index)
{
	PxU32 size = mPrunerSyncDataSize ? mPrunerSyncDataSize*2 : 64;
	const PxU32 minSize = index+1;
	if(minSize>size)
		size = minSize*2;

	PrunerSyncData** items = PX_ALLOCATE(PrunerSyncData*, size, "PrunerSyncData");
	if(mPrunerSyncData)
		PxMemCopy(items, mPrunerSyncData, mPrunerSyncDataSize*sizeof(PrunerSyncData*));
	PxMemZero(items+mPrunerSyncDataSize, (size-mPrunerSyncDataSize)*sizeof(PrunerSyncData*));
	PX_FREE(mPrunerSyncData);
	mPrunerSyncData = items;
	mPrunerSyncDataSize = size;
}

void SqBoundsManagerEx::addSyncShape(ShapeSimBase& shape)
{
	PX_ASSERT(shape.getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE);
	PX_ASSERT(!shape.getBodySim()->usingSqKinematicTarget());
	PX_ASSERT(!shape.getBodySim()->isFrozen());

	PX_ASSERT(shape.getSqBoundsId()==PX_INVALID_U32);
	PX_ASSERT(shape.getSqPrunerIndex()==PX_INVALID_U32);

	const PxU32 id = mWaitingRoom.size();
	mWaitingRoom.pushBack(&shape);
	shape.setSqBoundsId(id);
	shape.setSqPrunerIndex(PX_INVALID_U32);
}

void SqBoundsManagerEx::removeSyncShape(ShapeSimBase& shape)
{
	const PxU32 id = shape.getSqBoundsId();
	const PxU32 prunerIndex = shape.getSqPrunerIndex();
	PX_ASSERT(id!=PX_INVALID_U32);
	shape.setSqBoundsId(PX_INVALID_U32);
	shape.setSqPrunerIndex(PX_INVALID_U32);

	if(prunerIndex==PX_INVALID_U32)
	{
		// PT: this shape is still in the waiting room
		PX_ASSERT(mWaitingRoom[id]==&shape);

		mWaitingRoom[id] = mWaitingRoom.back();
		if(id+1 != mWaitingRoom.size())
			mWaitingRoom[id]->setSqBoundsId(id);
		mWaitingRoom.popBack();
	}
	else
	{
		// PT: this shape is active
		PX_ASSERT(prunerIndex<mPrunerSyncDataSize);
		PrunerSyncData* psd = mPrunerSyncData[prunerIndex];
		PX_ASSERT(psd);
		PX_ASSERT(psd->mShapes[id]==&shape);

		psd->mShapes[id] = psd->mShapes.back();
		psd->mBoundsIndices[id] = psd->mBoundsIndices.back();
		psd->mRefs[id] = psd->mRefs.back();

		if(id+1 != psd->mShapes.size())
			psd->mShapes[id]->setSqBoundsId(id);

		psd->mShapes.popBack();
		psd->mBoundsIndices.popBack();
		psd->mRefs.popBack();

		if(!psd->mShapes.size())
		{
			PX_DELETE(psd);
			mPrunerSyncData[prunerIndex] = NULL;
		}
	}
}

void SqBoundsManagerEx::syncBounds(SqBoundsSync& sync, SqRefFinder& finder, const PxBounds3* bounds, const PxTransform32* transforms, PxU64 contextID, const PxBitMap& ignoredIndices)
{
	PX_PROFILE_ZONE("Sim.sceneQuerySyncBounds", contextID);
	PX_UNUSED(contextID);
/*
#if PX_DEBUG
	for(PxU32 i=0;i<mShapeData.size();i++)
	{
		const ShapeSQData& shape = mShapeData[i];
		PX_UNUSED(shape);
		PX_ASSERT(shape.mSim->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE);
		PX_ASSERT(!shape.mSim->getBodySim()->usingSqKinematicTarget());
		PX_ASSERT(!shape.mSim->getBodySim()->isFrozen());
	}
#endif
*/

	const PxU32 nbToGo = mWaitingRoom.size();
	if(nbToGo)
	{
		for(PxU32 i=0;i<nbToGo;i++)
		{
			ShapeSimBase* sim = mWaitingRoom[i];
			PX_ASSERT(i==sim->getSqBoundsId());
			PX_ASSERT(PX_INVALID_U32==sim->getSqPrunerIndex());

			PxU32 prunerIndex = 0xffffffff;
			const ScPrunerHandle prunerHandle = finder.find(static_cast<PxRigidBody*>(sim->getBodySim()->getPxActor()), sim->getPxShape(), prunerIndex);

			PX_ASSERT(prunerIndex!=0xffffffff);

			if(prunerIndex>=mPrunerSyncDataSize)
				resize(prunerIndex);

			PrunerSyncData* psd = mPrunerSyncData[prunerIndex];
			if(!psd)
			{
				psd = PX_NEW(PrunerSyncData);
				mPrunerSyncData[prunerIndex] = psd;
			}

			PxArray<ShapeSimBase*>& shapes = psd->mShapes;
			PxArray<ScPrunerHandle>& refs = psd->mRefs;
			PxArray<PxU32>& boundsIndices = psd->mBoundsIndices;

			const PxU32 id = shapes.size();
			PX_ASSERT(id == refs.size());
			PX_ASSERT(id == boundsIndices.size());

			sim->setSqBoundsId(id);
			sim->setSqPrunerIndex(prunerIndex);

			// PT: mShapes / mRefs / mBoundsIndices are "parallel arrays". These arrays are persistent.
			// mRefless is temporary/transient data to help populate mRefs each frame.
			// mRefs / mBoundsIndices will be ultimately passed to updateObjects, whose API dictates the layout here.
			// mShapes is not actually used for the sync, it's only here to be able to call setSqBoundsId in removeShape.

			shapes.pushBack(sim);
			refs.pushBack(prunerHandle);
			boundsIndices.pushBack(sim->getElementID());
		}
		mWaitingRoom.clear();	// PT: TODO: optimize wasted memory here
	}

	// PT: TODO: optimize this
	{
		const PxU32 nb = mPrunerSyncDataSize;
		for(PxU32 i=0;i<nb;i++)
		{
			PrunerSyncData* psd = mPrunerSyncData[i];
			if(psd)
				sync.sync(i, psd->mRefs.begin(), psd->mBoundsIndices.begin(), bounds, transforms, psd->mRefs.size(), ignoredIndices);
		}
	}
}

