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

#include "GuPruningPool.h"
#include "foundation/PxMemory.h"
#include "common/PxProfileZone.h"

using namespace physx;
using namespace Gu;

PruningPool::PruningPool(PxU64 contextID, TransformCacheMode mode) :
	mNbObjects			(0),
	mMaxNbObjects		(0),
	mObjects			(NULL),
	mTransforms			(NULL),
	mTransformCacheMode	(mode),
	mHandleToIndex		(NULL),
	mIndexToHandle		(NULL),
	mFirstRecycledHandle(INVALID_PRUNERHANDLE),
	mContextID			(contextID)
{
}

PruningPool::~PruningPool()
{
	mWorldBoxes.release();
	PX_FREE(mIndexToHandle);
	PX_FREE(mHandleToIndex);
	PX_FREE(mTransforms);
	PX_FREE(mObjects);
}

bool PruningPool::resize(PxU32 newCapacity)
{
	PX_PROFILE_ZONE("PruningPool::resize", mContextID);

	const bool useTransforms = mTransformCacheMode!=TRANSFORM_CACHE_UNUSED;
	PxTransform* newTransforms = useTransforms ? PX_ALLOCATE(PxTransform, newCapacity, "Pruner transforms") : NULL;
	if(useTransforms && !newTransforms)
		return false;

	PrunerPayload*	newData				= PX_ALLOCATE(PrunerPayload, newCapacity, "PrunerPayload*");
	PrunerHandle*	newIndexToHandle	= PX_ALLOCATE(PrunerHandle, newCapacity, "Pruner Index Mapping");
	PoolIndex*		newHandleToIndex	= PX_ALLOCATE(PoolIndex, newCapacity, "Pruner Index Mapping");
	if( (!newData) || (!newIndexToHandle) || (!newHandleToIndex))
	{
		PX_FREE(newHandleToIndex);
		PX_FREE(newIndexToHandle);
		PX_FREE(newTransforms);
		PX_FREE(newData);
		return false;
	}

	mWorldBoxes.resize(newCapacity, mNbObjects);

	if(mObjects)		PxMemCopy(newData, mObjects, mNbObjects*sizeof(PrunerPayload));
	if(mTransforms)		PxMemCopy(newTransforms, mTransforms, mNbObjects*sizeof(PxTransform));
	if(mIndexToHandle)	PxMemCopy(newIndexToHandle, mIndexToHandle, mNbObjects*sizeof(PrunerHandle));
	if(mHandleToIndex)	PxMemCopy(newHandleToIndex, mHandleToIndex, mMaxNbObjects*sizeof(PoolIndex));	// PT: why mMaxNbObjects here? on purpose?
	mMaxNbObjects = newCapacity;

	PX_FREE(mIndexToHandle);
	PX_FREE(mHandleToIndex);
	PX_FREE(mTransforms);
	PX_FREE(mObjects);
	mObjects		= newData;
	mTransforms		= newTransforms;
	mHandleToIndex	= newHandleToIndex;
	mIndexToHandle	= newIndexToHandle;

	return true;
}

void PruningPool::preallocate(PxU32 newCapacity)
{
	if(newCapacity>mMaxNbObjects)
		resize(newCapacity);
}

PxU32 PruningPool::addObjects(PrunerHandle* results, const PxBounds3* bounds, const PrunerPayload* data, const PxTransform* transforms, PxU32 count)
{
	PX_PROFILE_ZONE("PruningPool::addObjects", mContextID);

	PX_ASSERT((!transforms && mTransformCacheMode==TRANSFORM_CACHE_UNUSED) || (transforms && mTransformCacheMode!=TRANSFORM_CACHE_UNUSED));

	for(PxU32 i=0;i<count;i++)
	{
		if(mNbObjects==mMaxNbObjects) // increase the capacity on overflow
		{
			const PxU32 newCapacity = PxU32(float(mMaxNbObjects)*1.5f);
			if(!resize(PxMax<PxU32>(newCapacity, 64)))
			//if(!resize(PxMax<PxU32>(mMaxNbObjects*2, 64)))
			{
				// pool can return an invalid handle if memory alloc fails
				// should probably have an error here or not handle this
				results[i] = INVALID_PRUNERHANDLE;	// PT: we need to write the potentially invalid handle to let users know which object failed first
				return i;
			}
		}
		PX_ASSERT(mNbObjects!=mMaxNbObjects);

		const PoolIndex index = mNbObjects++;

		// update mHandleToIndex and mIndexToHandle mappings
		PrunerHandle handle;
		if(mFirstRecycledHandle != INVALID_PRUNERHANDLE)
		{
			// mFirstRecycledHandle is an entry into a freelist for removed slots
			// this path is only taken if we have any removed slots
			handle = mFirstRecycledHandle;
			mFirstRecycledHandle = mHandleToIndex[handle];
		}
		else
		{
			handle = index;
		}

		// PT: TODO: investigate why we added mIndexToHandle/mHandleToIndex. The initial design with 'Prunable' objects didn't need these arrays.

		// PT: these arrays are "parallel"
		mWorldBoxes.getBounds()	[index] = bounds[i]; // store the payload/userData and AABB in parallel arrays
		mObjects				[index] = data[i];
		mIndexToHandle			[index] = handle;
		if(transforms && mTransforms)
			mTransforms			[index] = transforms[i];

		mHandleToIndex[handle] = index;
		results[i] = handle;
	}
	return count;
}

PoolIndex PruningPool::removeObject(PrunerHandle h, PrunerPayloadRemovalCallback* removalCallback)
{
	PX_PROFILE_ZONE("PruningPool::removeObject", mContextID);

	PX_ASSERT(mNbObjects);

	// remove the object and its AABB by provided PrunerHandle and update mHandleToIndex and mIndexToHandle mappings
	const PoolIndex indexOfRemovedObject = mHandleToIndex[h]; // retrieve object's index from handle

	if(removalCallback)
		removalCallback->invoke(1, &mObjects[indexOfRemovedObject]);

	const PoolIndex indexOfLastObject = --mNbObjects; // swap the object at last index with index
	if(indexOfLastObject!=indexOfRemovedObject)
	{
		// PT: move last object's data to recycled spot (from removed object)

		// PT: the last object has moved so we need to handle the mappings for this object
		// PT: TODO: investigate where this double-mapping comes from. It was not needed in the original design.

		// PT: these arrays are "parallel"
		PxBounds3* bounds = mWorldBoxes.getBounds();
		const PrunerHandle handleOfLastObject	= mIndexToHandle[indexOfLastObject];
		bounds			[indexOfRemovedObject]	= bounds		[indexOfLastObject];
		mObjects		[indexOfRemovedObject]	= mObjects		[indexOfLastObject];
		if(mTransforms)
			mTransforms	[indexOfRemovedObject]	= mTransforms	[indexOfLastObject];
		mIndexToHandle	[indexOfRemovedObject]	= handleOfLastObject;

		mHandleToIndex[handleOfLastObject]		= indexOfRemovedObject;
	}

	// mHandleToIndex also stores the freelist for removed handles (in place of holes formed by removed handles)
	mHandleToIndex[h] = mFirstRecycledHandle; // update linked list of available recycled handles
	mFirstRecycledHandle = h; // update the list head

	return indexOfLastObject;
}

void PruningPool::shiftOrigin(const PxVec3& shift)
{
	PX_PROFILE_ZONE("PruningPool::shiftOrigin", mContextID);

	const PxU32 nb = mNbObjects;
	PxBounds3* bounds = mWorldBoxes.getBounds();
	for(PxU32 i=0; i<nb; i++)
	{
		bounds[i].minimum -= shift;
		bounds[i].maximum -= shift;
	}

	if(mTransforms && mTransformCacheMode==TRANSFORM_CACHE_GLOBAL)
	{
		for(PxU32 i=0; i<nb; i++)
			mTransforms[i].p -= shift;
	}
}

template<const bool hasTransforms>
static void updateAndInflateBounds(PruningPool& pool, const PrunerHandle* PX_RESTRICT handles, const PxU32* PX_RESTRICT boundsIndices, const PxBounds3* PX_RESTRICT newBounds,
									const PxTransform32* PX_RESTRICT newTransforms, PxU32 count, float epsilon)
{
	PxBounds3* PX_RESTRICT bounds = pool.mWorldBoxes.getBounds();
	PxTransform* PX_RESTRICT transforms = hasTransforms ? pool.mTransforms : NULL;

	if(boundsIndices)
	{
		while(count--)
		{
			const PoolIndex poolIndex = pool.getIndex(*handles++);
			PX_ASSERT(poolIndex!=INVALID_PRUNERHANDLE);

			const PxU32 remappedIndex = *boundsIndices++;

			if(hasTransforms)
				transforms[poolIndex] = newTransforms[remappedIndex];

			inflateBounds<true>(bounds[poolIndex], newBounds[remappedIndex], epsilon);
		}
	}
	else
	{
		while(count--)
		{
			const PoolIndex poolIndex = pool.getIndex(*handles++);
			PX_ASSERT(poolIndex!=INVALID_PRUNERHANDLE);

			if(hasTransforms)
			{
				transforms[poolIndex] = *newTransforms;
				newTransforms++;
			}

			inflateBounds<true>(bounds[poolIndex], *newBounds++, epsilon);
		}
	}
}

void PruningPool::updateAndInflateBounds(const PrunerHandle* handles, const PxU32* boundsIndices, const PxBounds3* newBounds,
										const PxTransform32* newTransforms, PxU32 count, float epsilon)
{
	PX_PROFILE_ZONE("PruningPool::updateAndInflateBounds", mContextID);

	if(mTransforms)
		::updateAndInflateBounds<1>(*this, handles, boundsIndices, newBounds, newTransforms, count, epsilon);
	else
		::updateAndInflateBounds<0>(*this, handles, boundsIndices, newBounds, NULL, count, epsilon);
}
