// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PROFILE_ALLOCATOR_WRAPPER_H
#define PX_PROFILE_ALLOCATOR_WRAPPER_H

#include "foundation/PxPreprocessor.h"
#include "foundation/PxAllocatorCallback.h"
#include "foundation/PxErrorCallback.h"
#include "foundation/PxAssert.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxArray.h"

namespace physx { namespace profile {

	/**
	\brief Helper struct to encapsulate the user allocator callback
	Useful for array and hash templates
	*/
	struct PxProfileAllocatorWrapper
	{
		PxAllocatorCallback*			mUserAllocator;

		PxProfileAllocatorWrapper( PxAllocatorCallback& inUserAllocator )
			: mUserAllocator( &inUserAllocator )
		{
		}

		PxProfileAllocatorWrapper( PxAllocatorCallback* inUserAllocator )
			: mUserAllocator( inUserAllocator )
		{
		}

		PxAllocatorCallback&		getAllocator() const
		{
			PX_ASSERT( NULL != mUserAllocator );
			return *mUserAllocator;
		}
	};

	/**
	\brief Helper class to encapsulate the reflection allocator
	*/
	template <typename T>
	class PxProfileWrapperReflectionAllocator
	{
		static const char* getName()
		{
#if PX_LINUX || PX_OSX || PX_EMSCRIPTEN || PX_SWITCH
			return __PRETTY_FUNCTION__;
#else
			return typeid(T).name();
#endif
		}
		PxProfileAllocatorWrapper* mWrapper;

	public:
		PxProfileWrapperReflectionAllocator(PxProfileAllocatorWrapper& inWrapper) : mWrapper( &inWrapper )	{}
		PxProfileWrapperReflectionAllocator( const PxProfileWrapperReflectionAllocator& inOther )
			: mWrapper( inOther.mWrapper )
		{
		}
		PxProfileWrapperReflectionAllocator& operator=( const PxProfileWrapperReflectionAllocator& inOther )
		{
			mWrapper = inOther.mWrapper;
			return *this;
		}
		PxAllocatorCallback& getAllocator() { return mWrapper->getAllocator(); }
		void* allocate(size_t size, const char* filename, int line, uint32_t* cookie=NULL)
		{
			PX_UNUSED(cookie);
#if PX_CHECKED // checked and debug builds
			if(!size)
				return 0;
			return getAllocator().allocate(size, getName(), filename, line);
#else
			return getAllocator().allocate(size, "<no allocation names in this config>", filename, line);
#endif
		}
		void deallocate(void* ptr, uint32_t* cookie=NULL)
		{
			PX_UNUSED(cookie);
			if(ptr)
				getAllocator().deallocate(ptr);
		}
	};

	/**
	\brief Helper class to encapsulate the named allocator
	*/
	struct PxProfileWrapperNamedAllocator
	{
		PxProfileAllocatorWrapper*	mWrapper;
		const char*			mAllocationName;
		PxProfileWrapperNamedAllocator(PxProfileAllocatorWrapper& inWrapper, const char* inAllocationName)
			: mWrapper( &inWrapper )
			, mAllocationName( inAllocationName )
		{}
		PxProfileWrapperNamedAllocator( const PxProfileWrapperNamedAllocator& inOther )
			: mWrapper( inOther.mWrapper )
			, mAllocationName( inOther.mAllocationName )
		{
		}
		PxProfileWrapperNamedAllocator& operator=( const PxProfileWrapperNamedAllocator& inOther )
		{
			mWrapper = inOther.mWrapper;
			mAllocationName = inOther.mAllocationName;
			return *this;
		}
		PxAllocatorCallback& getAllocator() { return mWrapper->getAllocator(); }
		void* allocate(size_t size, const char* filename, int line, uint32_t* cookie=NULL)
		{
			PX_UNUSED(cookie);
			if(!size)
				return 0;
			return getAllocator().allocate(size, mAllocationName, filename, line);
		}
		void deallocate(void* ptr, uint32_t* cookie=NULL)
		{
			PX_UNUSED(cookie);
			if(ptr)
				getAllocator().deallocate(ptr);
		}
	};

	/**
	\brief Helper struct to encapsulate the array
	*/
	template<class T>
	struct PxProfileArray : public PxArray<T, PxProfileWrapperReflectionAllocator<T> >
	{
		typedef PxProfileWrapperReflectionAllocator<T> TAllocatorType;

		PxProfileArray( PxProfileAllocatorWrapper& inWrapper )
			: PxArray<T, TAllocatorType >( TAllocatorType( inWrapper ) )
		{
		}

		PxProfileArray( const PxProfileArray< T >& inOther )
			: PxArray<T, TAllocatorType >( inOther, inOther )
		{
		}
	};

	/**
	\brief Helper struct to encapsulate the array
	*/
	template<typename TKeyType, typename TValueType, typename THashType=PxHash<TKeyType> >
	struct PxProfileHashMap : public PxHashMap<TKeyType, TValueType, THashType, PxProfileWrapperReflectionAllocator< TValueType > >
	{
		typedef PxHashMap<TKeyType, TValueType, THashType, PxProfileWrapperReflectionAllocator< TValueType > > THashMapType;
		typedef PxProfileWrapperReflectionAllocator<TValueType> TAllocatorType;
		PxProfileHashMap( PxProfileAllocatorWrapper& inWrapper )
			: THashMapType( TAllocatorType( inWrapper ) )
		{
		}
	};

	/**
	\brief Helper function to encapsulate the profile allocation
	*/
	template<typename TDataType>
	inline TDataType* PxProfileAllocate( PxAllocatorCallback* inAllocator, const char* file, int inLine )
	{
		PxProfileAllocatorWrapper wrapper( inAllocator );
		typedef PxProfileWrapperReflectionAllocator< TDataType > TAllocator;
		TAllocator theAllocator( wrapper );
		return reinterpret_cast<TDataType*>( theAllocator.allocate( sizeof( TDataType ), file, inLine ) );
	}

	/**
	\brief Helper function to encapsulate the profile allocation
	*/
	template<typename TDataType>
	inline TDataType* PxProfileAllocate( PxAllocatorCallback& inAllocator, const char* file, int inLine )
	{
		return PxProfileAllocate<TDataType>( &inAllocator, file, inLine );
	}

	/**
	\brief Helper function to encapsulate the profile deallocation
	*/
	template<typename TDataType>
	inline void PxProfileDeleteAndDeallocate( PxProfileAllocatorWrapper& inAllocator, TDataType* inDType )
	{
		PX_ASSERT(inDType);
		PxAllocatorCallback& allocator( inAllocator.getAllocator() );
		inDType->~TDataType();
		allocator.deallocate( inDType );
	}

	/**
	\brief Helper function to encapsulate the profile deallocation
	*/
	template<typename TDataType>
	inline void PxProfileDeleteAndDeallocate( PxAllocatorCallback& inAllocator, TDataType* inDType )
	{
		PxProfileAllocatorWrapper wrapper( &inAllocator );
		PxProfileDeleteAndDeallocate( wrapper, inDType );
	}

} }

#define PX_PROFILE_NEW( allocator, dtype ) new (physx::profile::PxProfileAllocate<dtype>( allocator, PX_FL)) dtype
#define PX_PROFILE_DELETE( allocator, obj ) physx::profile::PxProfileDeleteAndDeallocate( allocator, obj );

#endif

