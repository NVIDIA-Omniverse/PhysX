// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PROFILE_DATA_BUFFER_H
#define PX_PROFILE_DATA_BUFFER_H

#include "PxProfileAllocatorWrapper.h"
#include "PxProfileMemoryBuffer.h"
#include "PxProfileEventBufferClient.h"

namespace physx { namespace profile {

	template<typename TMutex
			, typename TScopedLock>
	class DataBuffer //base class for buffers that cache data and then dump the data to clients.
	{
	public:
		typedef TMutex				TMutexType;
		typedef TScopedLock			TScopedLockType;
		typedef PxProfileWrapperNamedAllocator TU8AllocatorType;

		typedef MemoryBuffer<TU8AllocatorType > TMemoryBufferType;
		typedef PxProfileArray<PxProfileEventBufferClient*> TBufferClientArray;

	protected:
		
		PxProfileAllocatorWrapper			mWrapper;
		TMemoryBufferType					mDataArray;
		TBufferClientArray					mBufferClients;
		uint32_t							mBufferFullAmount;
		EventContextInformation				mEventContextInformation;		
		TMutexType*							mBufferMutex;
		volatile bool						mHasClients;
		EventSerializer<TMemoryBufferType >	mSerializer;

	public:
		
		DataBuffer( PxAllocatorCallback* inFoundation
					, uint32_t inBufferFullAmount
					, TMutexType* inBufferMutex
					, const char* inAllocationName )
			: mWrapper( inFoundation )
			, mDataArray( TU8AllocatorType( mWrapper, inAllocationName ) )
			, mBufferClients( mWrapper )
			, mBufferFullAmount( inBufferFullAmount )
			, mBufferMutex( inBufferMutex )
			, mHasClients( false )
			, mSerializer( &mDataArray )
		{
			//The data array is never resized really.  We ensure
			//it is bigger than it will ever need to be.
			mDataArray.reserve( inBufferFullAmount + 68 );
		}
		
		virtual ~DataBuffer()
		{
			while(mBufferClients.size() )
			{
				removeClient( *mBufferClients[0] );
			}
		}

		PxProfileAllocatorWrapper& getWrapper() { return mWrapper; }
		TMutexType*		  getBufferMutex() { return mBufferMutex; }
		void			  setBufferMutex(TMutexType* mutex) { mBufferMutex = mutex; }

		void addClient( PxProfileEventBufferClient& inClient ) 
		{ 
			TScopedLockType lock( mBufferMutex ); 
			mBufferClients.pushBack( &inClient );
			mHasClients = true;
		}

		void removeClient( PxProfileEventBufferClient& inClient ) 
		{
			TScopedLockType lock( mBufferMutex );
			for ( uint32_t idx =0; idx < mBufferClients.size(); ++idx )
			{
				if (mBufferClients[idx] == &inClient )
				{
					inClient.handleClientRemoved();
					mBufferClients.replaceWithLast( idx );
					break;
				}
			}
			mHasClients = mBufferClients.size() != 0;
		}

		
		bool hasClients() const 
		{ 
			return mHasClients;
		}

		virtual void flushEvents()
		{
			TScopedLockType lock(mBufferMutex);
			const uint8_t* theData = mDataArray.begin();
			uint32_t theDataSize = mDataArray.size();
			sendDataToClients(theData, theDataSize);
			mDataArray.clear();
			clearCachedData();
		}

		//Used for chaining together event buffers.
		virtual void handleBufferFlush( const uint8_t* inData, uint32_t inDataSize )
		{
			TScopedLockType lock( mBufferMutex );
			if ( inData && inDataSize )
			{
				clearCachedData();
				if ( mDataArray.size() + inDataSize >= mBufferFullAmount )
					flushEvents();
				if ( inDataSize >= mBufferFullAmount )
					sendDataToClients( inData, inDataSize );
				else
					mDataArray.write( inData, inDataSize );
			}
		}

	protected:
		virtual void clearCachedData()
		{
		}

	private:
			
		void sendDataToClients( const uint8_t* inData, uint32_t inDataSize )
		{
			uint32_t clientCount = mBufferClients.size();
			for( uint32_t idx =0; idx < clientCount; ++idx )
				mBufferClients[idx]->handleBufferFlush( inData, inDataSize );
		}

	};

}}

#endif

