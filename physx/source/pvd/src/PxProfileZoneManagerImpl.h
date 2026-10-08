// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PROFILE_ZONE_MANAGER_IMPL_H
#define PX_PROFILE_ZONE_MANAGER_IMPL_H

#include "PxProfileZoneManager.h"
#include "PxProfileScopedMutexLock.h"
#include "PxPvdProfileZone.h"
#include "PxProfileAllocatorWrapper.h"

#include "foundation/PxArray.h"
#include "foundation/PxMutex.h"

namespace physx { namespace profile {

	struct NullEventNameProvider : public PxProfileNameProvider
	{
		virtual PxProfileNames getProfileNames() const PX_OVERRIDE { return PxProfileNames( 0, 0 ); }
	};

	class ZoneManagerImpl : public PxProfileZoneManager
	{
		typedef ScopedLockImpl<PxMutex> TScopedLockType;
		PxProfileAllocatorWrapper					mWrapper;
		PxProfileArray<PxProfileZone*>		mZones;
		PxProfileArray<PxProfileZoneHandler*>	mHandlers;
		PxMutex mMutex;

		ZoneManagerImpl( const ZoneManagerImpl& inOther );
		ZoneManagerImpl& operator=( const ZoneManagerImpl& inOther );

	public:

		ZoneManagerImpl(PxAllocatorCallback* inFoundation) 
			: mWrapper( inFoundation )
			, mZones( mWrapper )
			, mHandlers( mWrapper ) 
		{}

		virtual ~ZoneManagerImpl()
		{
			//This assert would mean that a profile zone is outliving us.
			//This will cause a crash when the profile zone is released.
			PX_ASSERT( mZones.size() == 0 );
			while( mZones.size() )
				removeProfileZone( *mZones.back() );
		}

		virtual void addProfileZone( PxProfileZone& inSDK ) PX_OVERRIDE
		{
			TScopedLockType lock( &mMutex );
			
			if ( inSDK.getProfileZoneManager() != NULL )
			{
				if ( inSDK.getProfileZoneManager() == this )
					return;
				else //there must be two managers in the system somehow.
				{
					PX_ASSERT( false );
					inSDK.getProfileZoneManager()->removeProfileZone( inSDK );
				}
			}
			mZones.pushBack( &inSDK );
			inSDK.setProfileZoneManager( this );
			for ( uint32_t idx =0; idx < mHandlers.size(); ++idx )
				mHandlers[idx]->onZoneAdded( inSDK );
		}

		virtual void removeProfileZone( PxProfileZone& inSDK ) PX_OVERRIDE
		{
			TScopedLockType lock( &mMutex );
			if ( inSDK.getProfileZoneManager() == NULL )
				return;

			else if ( inSDK.getProfileZoneManager() != this )
			{
				PX_ASSERT( false );
				inSDK.getProfileZoneManager()->removeProfileZone( inSDK );
				return;
			}

			inSDK.setProfileZoneManager( NULL );
			for ( uint32_t idx = 0; idx < mZones.size(); ++idx )
			{
				if ( mZones[idx] == &inSDK )
				{
					for ( uint32_t handler =0; handler < mHandlers.size(); ++handler )
						mHandlers[handler]->onZoneRemoved( inSDK );
					mZones.replaceWithLast( idx );
				}
			}
		}

		virtual void flushProfileEvents() PX_OVERRIDE
		{
			uint32_t sdkCount = mZones.size();
			for ( uint32_t idx = 0; idx < sdkCount; ++idx )
				mZones[idx]->flushProfileEvents();
		}

		virtual void addProfileZoneHandler( PxProfileZoneHandler& inHandler ) PX_OVERRIDE
		{
			TScopedLockType lock( &mMutex );
			mHandlers.pushBack( &inHandler );
			for ( uint32_t idx = 0; idx < mZones.size(); ++idx )
				inHandler.onZoneAdded( *mZones[idx] );
		}

		virtual void removeProfileZoneHandler( PxProfileZoneHandler& inHandler ) PX_OVERRIDE
		{
			TScopedLockType lock( &mMutex );
			for( uint32_t idx = 0; idx < mZones.size(); ++idx )
				inHandler.onZoneRemoved( *mZones[idx] );
			for( uint32_t idx = 0; idx < mHandlers.size(); ++idx )
			{
				if ( mHandlers[idx] == &inHandler )
					mHandlers.replaceWithLast( idx );
			}
		}
		
		virtual PxProfileZone& createProfileZone( const char* inSDKName, PxProfileNameProvider* inProvider, uint32_t inEventBufferByteSize )
		{
			NullEventNameProvider nullProvider;
			if ( inProvider == NULL )
				inProvider = &nullProvider;
			return createProfileZone( inSDKName, inProvider->getProfileNames(), inEventBufferByteSize );
		}
		
		
		virtual PxProfileZone& createProfileZone( const char* inSDKName, PxProfileNames inNames, uint32_t inEventBufferByteSize ) PX_OVERRIDE
		{
			PxProfileZone& retval( PxProfileZone::createProfileZone( &mWrapper.getAllocator(), inSDKName, inNames, inEventBufferByteSize ) );
			addProfileZone( retval );
			return retval;
		}

		virtual void release() PX_OVERRIDE
		{  
			PX_PROFILE_DELETE( mWrapper.getAllocator(), this );
		}
	};
} }


#endif

