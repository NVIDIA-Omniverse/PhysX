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

#include "GuActorShapeMap.h"
#include "foundation/PxMemory.h"

using namespace physx;
using namespace Gu;

namespace physx
{
	namespace Gu
	{
		/*PX_FORCE_INLINE*/ uint32_t PxComputeHash(const ActorShapeMap::ActorShape& owner)
		{
			PX_ASSERT(!(size_t(owner.mActor)&3));
			PX_ASSERT(!(size_t(owner.mShape)&3));
			const uint32_t id0 = uint32_t(size_t(owner.mActor)>>2);
			const uint32_t id1 = uint32_t(size_t(owner.mShape)>>2);
			const uint64_t mix = (uint64_t(id0)<<32)|uint64_t(id1);
			return ::PxComputeHash(mix);
		}
	}
}

ActorShapeMap::ActorShapeMap() : mCacheSize(0), mCache(NULL)
{
}

ActorShapeMap::~ActorShapeMap()
{
	PX_FREE(mCache);
}

void ActorShapeMap::resizeCache(PxU32 index)
{
	PxU32 size = mCacheSize ? mCacheSize*2 : 64;
	const PxU32 minSize = index+1;
	if(minSize>size)
		size = minSize*2;

	Cache* items = PX_ALLOCATE(Cache, size, "Cache");
	if(mCache)
		PxMemCopy(items, mCache, mCacheSize*sizeof(Cache));
	PxMemZero(items+mCacheSize, (size-mCacheSize)*sizeof(Cache));
	PX_FREE(mCache);
	mCache = items;
	mCacheSize = size;
}

bool ActorShapeMap::add(PxU32 actorIndex, const void* actor, const void* shape, ActorShapeData actorShapeData)
{
	if(actorIndex!=PX_INVALID_INDEX)
	{
		if(actorIndex>=mCacheSize)
			resizeCache(actorIndex);

		//if(!mCache[actorIndex].mActor)
		if(!mCache[actorIndex].mShape)
		{
			//mCache[actorIndex].mActor	= actor;
			mCache[actorIndex].mShape	= shape;
			mCache[actorIndex].mData	= actorShapeData;
			return true;
		}

		//PX_ASSERT(mCache[actorIndex].mActor==actor);
		PX_ASSERT(mCache[actorIndex].mShape);
		if(mCache[actorIndex].mShape==shape)
		{
			mCache[actorIndex].mData	= actorShapeData;
			return false;
		}
	}
	return mDatabase.insert(ActorShape(actor, shape), actorShapeData);
}

bool ActorShapeMap::remove(PxU32 actorIndex, const void* actor, const void* shape, ActorShapeData* removed)
{
	if(actorIndex!=PX_INVALID_INDEX)
	{
		//if(mCache[actorIndex].mActor==actor && mCache[actorIndex].mShape==shape)
		if(mCache[actorIndex].mShape==shape)
		{
			//mCache[actorIndex].mActor	= NULL;
			mCache[actorIndex].mShape	= NULL;
			PX_ASSERT(!mDatabase.erase(ActorShape(actor, shape)));
			if(removed)
				*removed = mCache[actorIndex].mData;
			return true;
		}
	}

	PxHashMap<ActorShape, ActorShapeData>::Entry removedEntry; 
	const bool found = mDatabase.erase(ActorShape(actor, shape), removedEntry);
	if(found && removed)
		*removed = removedEntry.second;
	return found;
}

ActorShapeData ActorShapeMap::find(PxU32 actorIndex, const void* actor, const void* shape) const
{
	if(actorIndex!=PX_INVALID_INDEX)
	{
		if(mCache[actorIndex].mShape==shape)
		//if(mCache[actorIndex].mActor==actor && mCache[actorIndex].mShape==shape)
		{
			return mCache[actorIndex].mData;
		}
	}

	const PxHashMap<ActorShape, ActorShapeData>::Entry* e = mDatabase.find(ActorShape(actor, shape));
	PX_ASSERT(e);
	return e->second;
}

