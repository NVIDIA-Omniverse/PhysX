// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SN_XML_MEMORY_POOL_STREAMS_H
#define SN_XML_MEMORY_POOL_STREAMS_H

#include "foundation/PxTransform.h"
#include "foundation/PxIO.h"
#include "SnXmlMemoryPool.h"

namespace physx {

	template<typename TDataType>
	struct XmlDefaultValue
	{
		bool force_compile_error;
	};


#define XML_DEFINE_DEFAULT_VALUE(type, defVal )		\
	template<>											\
	struct XmlDefaultValue<type>						\
	{													\
		type getDefaultValue() { return type(defVal); }	\
	};

	XML_DEFINE_DEFAULT_VALUE(PxU8, 0)
	XML_DEFINE_DEFAULT_VALUE(PxI8, 0)
	XML_DEFINE_DEFAULT_VALUE(PxU16, 0)
	XML_DEFINE_DEFAULT_VALUE(PxI16, 0)
	XML_DEFINE_DEFAULT_VALUE(PxU32, 0)
	XML_DEFINE_DEFAULT_VALUE(PxI32, 0)
	XML_DEFINE_DEFAULT_VALUE(PxU64, 0)
	XML_DEFINE_DEFAULT_VALUE(PxI64, 0)
	XML_DEFINE_DEFAULT_VALUE(PxF32, 0)
	XML_DEFINE_DEFAULT_VALUE(PxF64, 0)

#undef XML_DEFINE_DEFAULT_VALUE

	template<>											
	struct XmlDefaultValue<PxVec3>						
	{
		PxVec3 getDefaultValue() { return PxVec3( 0,0,0 ); }
	};
	
	template<>											
	struct XmlDefaultValue<PxTransform>						
	{
		PxTransform getDefaultValue() { return PxTransform(PxIdentity); }
	};

	template<>											
	struct XmlDefaultValue<PxQuat>	
	{
		PxQuat getDefaultValue() { return PxQuat(PxIdentity); }
	};

/** 
 *	Mapping of PxOutputStream to a memory pool manager.
 *	Allows write-then-read semantics of a set of
 *	data.  Can safely write up to 4GB of data; then you
 *	will silently fail...
 */

template<typename TAllocatorType>
struct MemoryBufferBase : public PxOutputStream, public PxInputStream
{
	TAllocatorType* mManager;
	mutable PxU64	mWriteOffset;
	mutable PxU64	mReadOffset;
	PxU8*	mBuffer;
	PxU64	mCapacity;


	MemoryBufferBase( TAllocatorType* inManager )
		: mManager( inManager )
		, mWriteOffset( 0 )
		, mReadOffset( 0 )
		, mBuffer( NULL )
		, mCapacity( 0 )
	{
	}
	virtual						~MemoryBufferBase()
	{
		mManager->deallocate( mBuffer );
	}
	PxU8* releaseBuffer()
	{
		clear();
		mCapacity = 0;
		PxU8* retval(mBuffer);
		mBuffer = NULL;
		return retval;
	}
	void clear()
	{
		mWriteOffset = mReadOffset = 0;
	}

	virtual PxU64 read(void* dest, PxU64 count) PX_OVERRIDE
	{
		bool fits = ( mReadOffset + count ) <= mWriteOffset;
		PX_ASSERT( fits );
		if ( fits )
		{
			PxMemCopy( dest, mBuffer + mReadOffset, count );
			mReadOffset += count;
			return count;
		}
		return 0;
	}

	inline void checkCapacity( PxU64 inNewCapacity )
	{
		if ( mCapacity < inNewCapacity )
		{
			PxU64 newCapacity = 32;
			while( newCapacity < inNewCapacity )
				newCapacity = newCapacity << 1;

			PxU8* newData( mManager->allocate( newCapacity ) );
			if ( mWriteOffset )
				PxMemCopy( newData, mBuffer, mWriteOffset );
			mManager->deallocate( mBuffer );
			mBuffer = newData;
			mCapacity = newCapacity;
		}
	}

	virtual PxU64 write(const void* src, PxU64 count) PX_OVERRIDE
	{
		checkCapacity( mWriteOffset + count );
		PxMemCopy( mBuffer + mWriteOffset, src, count );
		mWriteOffset += count;
		return count;
	}
};

class MemoryBuffer : public MemoryBufferBase<CMemoryPoolManager >
{
public:
	MemoryBuffer( CMemoryPoolManager* inManager ) : MemoryBufferBase<CMemoryPoolManager >( inManager ) {}
};

}

#endif
