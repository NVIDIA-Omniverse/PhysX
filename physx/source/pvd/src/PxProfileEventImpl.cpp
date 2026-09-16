// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxProfileEventBuffer.h"
#include "PxProfileZoneImpl.h"
#include "PxProfileZoneManagerImpl.h"
#include "PxProfileMemoryEventBuffer.h"
#include "foundation/PxUserAllocated.h"

namespace physx { namespace profile {

	struct PxProfileNameProviderForward
	{
		PxProfileNames mNames;
		PxProfileNameProviderForward( PxProfileNames inNames )
			: mNames( inNames )
		{
		}
		PxProfileNames getProfileNames() const { return mNames; }
	};

	PxProfileZone& PxProfileZone::createProfileZone( PxAllocatorCallback* inAllocator, const char* inSDKName, PxProfileNames inNames, uint32_t inEventBufferByteSize )
	{
		typedef ZoneImpl<PxProfileNameProviderForward> TSDKType;
		return *PX_PROFILE_NEW( inAllocator, TSDKType ) ( inAllocator, inSDKName, inEventBufferByteSize, PxProfileNameProviderForward( inNames ) );
	}
	
	PxProfileZoneManager& PxProfileZoneManager::createProfileZoneManager(PxAllocatorCallback* inAllocator )
	{
		return *PX_PROFILE_NEW( inAllocator, ZoneManagerImpl ) ( inAllocator );
	}

	PxProfileMemoryEventBuffer& PxProfileMemoryEventBuffer::createMemoryEventBuffer( PxAllocatorCallback& inAllocator, uint32_t inBufferSize )
	{
		return *PX_PROFILE_NEW( &inAllocator, PxProfileMemoryEventBufferImpl )( inAllocator, inBufferSize );
	}

} }

