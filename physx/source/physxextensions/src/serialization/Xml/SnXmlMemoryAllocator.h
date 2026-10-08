// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SN_XML_MEMORY_ALLOCATOR_H
#define SN_XML_MEMORY_ALLOCATOR_H

#include "foundation/PxSimpleTypes.h"

namespace physx { 

	class PX_DEPRECATED XmlMemoryAllocator
	{
	protected:
		virtual ~XmlMemoryAllocator(){}
	public:
		virtual PxU8* allocate(PxU64 inSize) = 0;
		virtual void deallocate( PxU8* inMem ) = 0;
		virtual PxAllocatorCallback& getAllocator() = 0;
		template<typename TObjectType>
		TObjectType* allocate()
		{
			TObjectType* retval = reinterpret_cast< TObjectType* >( allocate( sizeof( TObjectType ) ) );
			new (retval) TObjectType();
			return retval;
		}

		template<typename TObjectType, typename TArgType>
		TObjectType* allocate(const TArgType &arg)
		{
			TObjectType* retval = reinterpret_cast< TObjectType* >( allocate( sizeof( TObjectType ) ) );
			new (retval) TObjectType(arg);
			return retval;
		}

		template<typename TObjectType>
		void deallocate( TObjectType* inObject )
		{
			deallocate( reinterpret_cast<PxU8*>( inObject ) );
		}
		template<typename TObjectType>
		inline TObjectType* batchAllocate(PxU32 inCount )
		{
			TObjectType* retval = reinterpret_cast<TObjectType*>( allocate( sizeof(TObjectType) * inCount ) );
			for ( PxU32 idx = 0; idx < inCount; ++idx )
			{
				new (retval + idx) TObjectType();
			}
			return retval;
		}

		template<typename TObjectType, typename TArgType>
		inline TObjectType* batchAllocate(PxU32 inCount, const TArgType &arg)
		{
			TObjectType* retval = reinterpret_cast<TObjectType*>( allocate( sizeof(TObjectType) * inCount ) );
			for ( PxU32 idx = 0; idx < inCount; ++idx )
			{
				new (retval + idx) TObjectType(arg);
			}
			return retval;
		}


		//Duplicate function definition for gcc.
		template<typename TObjectType>
		inline TObjectType* batchAllocate(TObjectType*, PxU32 inCount )
		{
			TObjectType* retval = reinterpret_cast<TObjectType*>( allocate( sizeof(TObjectType) * inCount ) );
			for ( PxU32 idx = 0; idx < inCount; ++idx )
			{
				new (retval + idx) TObjectType();
			}
			return retval;
		}
	};
	
	struct PX_DEPRECATED XmlMemoryAllocatorImpl : public XmlMemoryAllocator
	{
		Sn::TMemoryPoolManager mManager;

		XmlMemoryAllocatorImpl( PxAllocatorCallback& inAllocator )
			: mManager( inAllocator )
		{
		}
		XmlMemoryAllocatorImpl &operator=(const XmlMemoryAllocatorImpl &);
		virtual PxAllocatorCallback& getAllocator() PX_OVERRIDE
		{
			return mManager.getWrapper().getAllocator();
		}
		
		virtual PxU8* allocate(PxU64 inSize ) PX_OVERRIDE
		{
			if ( !inSize )
				return NULL;

			return mManager.allocate( inSize );
		}
		virtual void deallocate( PxU8* inMem ) PX_OVERRIDE
		{
			if ( inMem )
				mManager.deallocate( inMem );
		}
	};
}
#endif
