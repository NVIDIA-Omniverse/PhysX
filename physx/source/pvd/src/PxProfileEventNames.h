// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PROFILE_EVENT_NAMES_H
#define PX_PROFILE_EVENT_NAMES_H

#include "PxProfileEventId.h"

namespace physx { namespace profile {

	/**
	\brief Mapping from event id to name.
	*/
	struct PxProfileEventName
	{
		const char*					name;
		PxProfileEventId			eventId;

		/**
		\brief Default constructor.
		\param inName Profile event name.
		\param inId Profile event id.
		*/
		PxProfileEventName( const char* inName, PxProfileEventId inId ) : name( inName ), eventId( inId ) {}
	};

	/**
	\brief Aggregator of event id -> name mappings
	*/
	struct PxProfileNames
	{
		/**
		\brief Default constructor that doesn't point to any names.
		\param inEventCount Number of provided events.
		\param inSubsystems Event names array.
		*/
		PxProfileNames( uint32_t inEventCount = 0, const PxProfileEventName* inSubsystems = NULL )
			: eventCount( inEventCount )
			, events( inSubsystems )
		{
		}

		uint32_t							eventCount;
		const PxProfileEventName*			events;
	};

	/**
	\brief Provides a mapping from event ID -> name.
	*/
	class PxProfileNameProvider
	{
	public:
		/**
		\brief Returns profile event names.
		\return Profile event names.
		*/
		virtual PxProfileNames getProfileNames() const = 0;

	protected:
		virtual ~PxProfileNameProvider(){}
		PxProfileNameProvider& operator=(const PxProfileNameProvider&) { return *this; }
	};
} }

#endif

