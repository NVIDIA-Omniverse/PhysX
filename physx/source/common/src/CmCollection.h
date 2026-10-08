// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CM_COLLECTION_H
#define CM_COLLECTION_H

#include "common/PxCollection.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxAllocator.h"

namespace physx
{
namespace Cm
{
	template <class Key, 
			  class Value,
			  class HashFn = PxHash<Key>, 
			  class Allocator = PxAllocator >
	class CollectionHashMap : public PxCoalescedHashMap< Key, Value, HashFn, Allocator>
	{
		typedef physx::PxHashMapBase< Key, Value, HashFn, Allocator> MapBase;	
		typedef PxPair<const Key,Value> EntryData;

		public:
			CollectionHashMap(PxU32 initialTableSize = 64, float loadFactor = 0.75f):
			    PxCoalescedHashMap< Key, Value, HashFn, Allocator>(initialTableSize,loadFactor) {}

			void insertUnique(const Key& k, const Value& v)
			{
				PX_PLACEMENT_NEW(MapBase::mBase.insertUnique(k), EntryData)(k,v);
			}
	};

	class Collection : public PxCollection, public PxUserAllocated
	{
	public:
		typedef CollectionHashMap<PxBase*, PxSerialObjectId> ObjectToIdMap;
		typedef CollectionHashMap<PxSerialObjectId, PxBase*> IdToObjectMap;
					
		virtual void						add(PxBase& object, PxSerialObjectId ref) PX_OVERRIDE;
		virtual	void						remove(PxBase& object) PX_OVERRIDE;	
		virtual bool						contains(PxBase& object) const PX_OVERRIDE;
		virtual void						addId(PxBase& object, PxSerialObjectId id) PX_OVERRIDE;
		virtual void						removeId(PxSerialObjectId id) PX_OVERRIDE;
		virtual PxBase*						find(PxSerialObjectId ref) const PX_OVERRIDE;
		virtual void						add(PxCollection& collection) PX_OVERRIDE;
		virtual void						remove(PxCollection& collection) PX_OVERRIDE;		
		virtual	PxU32						getNbObjects() const PX_OVERRIDE;
		virtual PxBase&						getObject(PxU32 index) const PX_OVERRIDE;
		virtual	PxU32						getObjects(PxBase** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const PX_OVERRIDE;

		virtual PxU32						getNbIds() const PX_OVERRIDE;		
		virtual PxSerialObjectId			getId(const PxBase& object) const PX_OVERRIDE;
		virtual	PxU32						getIds(PxSerialObjectId* userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const PX_OVERRIDE;

		virtual	void						release() PX_OVERRIDE	{ PX_DELETE_THIS; }

		// Only for internal use. Bypasses virtual calls, specialized behaviour.
		PX_INLINE	void						internalAdd(PxBase* s, PxSerialObjectId id = PX_SERIAL_OBJECT_ID_INVALID)	{ mObjects.insertUnique(s, id);	}
		PX_INLINE	PxU32						internalGetNbObjects()		const	{ return mObjects.size();												}
		PX_INLINE	PxBase*						internalGetObject(PxU32 i)	const	{ PX_ASSERT(i<mObjects.size());	return mObjects.getEntries()[i].first;	}
		PX_INLINE	const ObjectToIdMap::Entry*	internalGetObjects()		const	{ return mObjects.getEntries();											}
			
					IdToObjectMap				mIds;
					ObjectToIdMap				mObjects;
	};
}
}

#endif
