// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxAllocatorCallback.h"
#include "foundation/PxString.h"
#include "foundation/PxUserAllocated.h"
#include "extensions/PxStringTableExt.h"
#include "PxProfileAllocatorWrapper.h" //tools for using a custom allocator

using namespace physx;
using namespace physx::profile;

namespace
{
	class PxStringTableImpl : public PxStringTable, public PxUserAllocated
	{
		typedef PxProfileHashMap<const char*, PxU32> THashMapType;
		PxProfileAllocatorWrapper mWrapper;
		THashMapType mHashMap;
	public:

		PxStringTableImpl( PxAllocatorCallback& inAllocator )
			: mWrapper ( inAllocator )
			, mHashMap ( mWrapper )
		{
		}

		virtual ~PxStringTableImpl()
		{
			for ( THashMapType::Iterator iter = mHashMap.getIterator();
				iter.done() == false;
				++iter )
				PX_PROFILE_DELETE( mWrapper, const_cast<char*>( iter->first ) );
			mHashMap.clear();
		}

		virtual const char* allocateStr( const char* inSrc ) PX_OVERRIDE
		{
			if ( inSrc == NULL )
				inSrc = "";
			const THashMapType::Entry* existing( mHashMap.find( inSrc ) );
			if ( existing == NULL )
			{
				size_t len( strnlen( inSrc, UINT64_MAX - 1 ) );
				len += 1;
				char* newMem = reinterpret_cast<char*>(mWrapper.getAllocator().allocate( len, "PxStringTableImpl: const char*", PX_FL));
				physx::Pxstrlcpy( newMem, len, inSrc );
				mHashMap.insert( newMem, 1 );
				return newMem;
			}
			else
			{
				++const_cast<THashMapType::Entry*>(existing)->second;
				return existing->first;
			}
		}

		/**
		 *	Release the string table and all the strings associated with it.
		 */
		virtual void release() PX_OVERRIDE
		{
			PX_PROFILE_DELETE( mWrapper.getAllocator(), this );
		}
	};
}

PxStringTable& physx::PxStringTableExt::createStringTable( PxAllocatorCallback& inAllocator )
{
	return *PX_PROFILE_NEW( inAllocator, PxStringTableImpl )( inAllocator );
}
