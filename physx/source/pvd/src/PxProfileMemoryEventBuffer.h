// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PROFILE_MEMORY_EVENT_BUFFER_H
#define PX_PROFILE_MEMORY_EVENT_BUFFER_H

#include "PxProfileDataBuffer.h"
#include "PxProfileMemoryEvents.h"
#include "PxProfileMemory.h"
#include "PxProfileScopedMutexLock.h"
#include "PxProfileAllocatorWrapper.h"
#include "PxProfileEventMutex.h"

#include "foundation/PxHash.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxUserAllocated.h"

namespace physx { namespace profile {

	template<typename TMutex,
			 typename TScopedLock>
	class MemoryEventBuffer : public DataBuffer<TMutex, TScopedLock>
	{
	public:
		typedef DataBuffer<TMutex, TScopedLock> TBaseType;
		typedef typename TBaseType::TMutexType TMutexType;
		typedef typename TBaseType::TScopedLockType TScopedLockType;
		typedef typename TBaseType::TU8AllocatorType TU8AllocatorType;
		typedef typename TBaseType::TMemoryBufferType TMemoryBufferType;
		typedef typename TBaseType::TBufferClientArray TBufferClientArray;
		typedef PxHashMap<const char*, uint32_t, PxHash<const char*>, TU8AllocatorType> TCharPtrToHandleMap;

	protected:
		TCharPtrToHandleMap mStringTable;

	public:

		MemoryEventBuffer( PxAllocatorCallback& cback
					, uint32_t inBufferFullAmount
					, TMutexType* inBufferMutex )
			: TBaseType( &cback, inBufferFullAmount, inBufferMutex, "struct physx::profile::MemoryEvent" )
			, mStringTable( TU8AllocatorType( TBaseType::getWrapper(), "MemoryEventStringBuffer" ) )
		{
		}

		uint32_t getHandle( const char* inData )
		{
			if ( inData == NULL ) inData = "";
			const typename TCharPtrToHandleMap::Entry* result( mStringTable.find( inData ) );
			if ( result )
				return result->second;
			uint32_t hdl = mStringTable.size() + 1;
			mStringTable.insert( inData, hdl );
			StringTableEvent theEvent;
			theEvent.init( inData, hdl );
			sendEvent( theEvent );
			return hdl;
		}

		void onAllocation( size_t inSize, const char* inType, const char* inFile, uint32_t inLine, uint64_t addr )
		{
			if ( addr == 0 )
				return;
			uint32_t typeHdl( getHandle( inType ) );
			uint32_t fileHdl( getHandle( inFile ) );
			AllocationEvent theEvent;
			theEvent.init( inSize, typeHdl, fileHdl, inLine, addr );
			sendEvent( theEvent );
		}

		void onDeallocation( uint64_t addr )
		{
			if ( addr == 0 )
				return;
			DeallocationEvent theEvent;
			theEvent.init( addr );
			sendEvent( theEvent );
		}

		void flushProfileEvents()
		{
			TBaseType::flushEvents();
		}

	protected:
		
		template<typename TDataType>
		void sendEvent( TDataType inType )
		{
			MemoryEventHeader theHeader( getMemoryEventType<TDataType>() );
			inType.setup( theHeader );
			theHeader.streamify( TBaseType::mSerializer );
			inType.streamify( TBaseType::mSerializer, theHeader );
			if ( TBaseType::mDataArray.size() >= TBaseType::mBufferFullAmount )
				flushProfileEvents();
		}
	};

	class PxProfileMemoryEventBufferImpl : public PxUserAllocated
		, public PxProfileMemoryEventBuffer
	{
		typedef MemoryEventBuffer<PxProfileEventMutex, NullLock> TMemoryBufferType;
		TMemoryBufferType mBuffer;

	public:
		PxProfileMemoryEventBufferImpl( PxAllocatorCallback& alloc, uint32_t inBufferFullAmount )
			: mBuffer( alloc, inBufferFullAmount, NULL )
		{
		}

		virtual void onAllocation( size_t size, const char* typeName, const char* filename, int line, void* allocatedMemory ) PX_OVERRIDE
		{
			mBuffer.onAllocation( size, typeName, filename, uint32_t(line), static_cast<uint64_t>(reinterpret_cast<size_t>(allocatedMemory)) );
		}
		virtual void onDeallocation( void* allocatedMemory ) PX_OVERRIDE
		{
			mBuffer.onDeallocation(static_cast<uint64_t>(reinterpret_cast<size_t>(allocatedMemory)) );
		}
		
		virtual void addClient( PxProfileEventBufferClient& inClient ) PX_OVERRIDE { mBuffer.addClient( inClient ); }
		virtual void removeClient( PxProfileEventBufferClient& inClient ) PX_OVERRIDE { mBuffer.removeClient( inClient ); }
		virtual bool hasClients() const PX_OVERRIDE { return mBuffer.hasClients(); }

		virtual void flushProfileEvents() PX_OVERRIDE { mBuffer.flushProfileEvents(); }

		virtual void release() PX_OVERRIDE { PX_PROFILE_DELETE( mBuffer.getWrapper().getAllocator(), this ); }
	};
}}

#endif

