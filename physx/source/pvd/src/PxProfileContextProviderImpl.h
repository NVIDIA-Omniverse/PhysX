// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PROFILE_CONTEXT_PROVIDER_IMPL_H
#define PX_PROFILE_CONTEXT_PROVIDER_IMPL_H

#include "PxProfileContextProvider.h"

#include "foundation/PxThread.h"

namespace physx { namespace profile {
	
	struct PxDefaultContextProvider
	{
		PxProfileEventExecutionContext getExecutionContext() 
		{ 
			PxThread::Id theId( PxThread::getId() );
			return PxProfileEventExecutionContext( static_cast<uint32_t>( theId ), static_cast<uint8_t>( PxThreadPriority::eNORMAL ), 0 );
		}

		uint32_t getThreadId() 
		{ 
			return static_cast<uint32_t>( PxThread::getId() ); 
		}
	};
} }

#endif

