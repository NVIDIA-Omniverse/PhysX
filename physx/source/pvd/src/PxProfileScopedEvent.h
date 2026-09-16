// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PROFILE_SCOPED_EVENT_H
#define PX_PROFILE_SCOPED_EVENT_H

#include "PxProfileEventId.h"
#include "PxProfileCompileTimeEventFilter.h"

namespace physx { namespace profile {

	/**
	\brief Template version of startEvent, called directly on provided profile buffer.

	\param inBuffer Profile event buffer.
	\param inId Profile event id.
	\param inContext Profile event context.
	*/
	template<bool TEnabled, typename TBufferType>
	inline void startEvent( TBufferType* inBuffer, const PxProfileEventId& inId, uint64_t inContext )
	{
		if ( TEnabled && inBuffer ) inBuffer->startEvent( inId, inContext );
	}

	/**
	\brief Template version of stopEvent, called directly on provided profile buffer.

	\param inBuffer Profile event buffer.
	\param inId Profile event id.
	\param inContext Profile event context.
	*/
	template<bool TEnabled, typename TBufferType>
	inline void stopEvent( TBufferType* inBuffer, const PxProfileEventId& inId, uint64_t inContext )
	{
		if ( TEnabled && inBuffer ) inBuffer->stopEvent( inId, inContext );
	}
	
	/**
	\brief Template version of startEvent, called directly on provided profile buffer.

	\param inEnabled If profile event is enabled.
	\param inBuffer Profile event buffer.
	\param inId Profile event id.
	\param inContext Profile event context.
	*/
	template<typename TBufferType>
	inline void startEvent( bool inEnabled, TBufferType* inBuffer, const PxProfileEventId& inId, uint64_t inContext )
	{
		if ( inEnabled && inBuffer ) inBuffer->startEvent( inId, inContext );
	}

	/**
	\brief Template version of stopEvent, called directly on provided profile buffer.

	\param inEnabled If profile event is enabled.
	\param inBuffer Profile event buffer.
	\param inId Profile event id.
	\param inContext Profile event context.
	*/
	template<typename TBufferType>
	inline void stopEvent( bool inEnabled, TBufferType* inBuffer, const PxProfileEventId& inId, uint64_t inContext )
	{
		if ( inEnabled && inBuffer ) inBuffer->stopEvent( inId, inContext );
	}
	
	/**
	\brief Template version of eventValue, called directly on provided profile buffer.

	\param inEnabled If profile event is enabled.
	\param inBuffer Profile event buffer.
	\param inId Profile event id.
	\param inContext Profile event context.
	\param inValue Event value.
	*/
	template<typename TBufferType>
	inline void eventValue( bool inEnabled, TBufferType* inBuffer, const PxProfileEventId& inId, uint64_t inContext, int64_t inValue )
	{
		if ( inEnabled && inBuffer ) inBuffer->eventValue( inId, inContext, inValue );
	}

}}

#endif

