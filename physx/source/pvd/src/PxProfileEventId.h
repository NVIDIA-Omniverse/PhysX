// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PROFILE_EVENT_ID_H
#define PX_PROFILE_EVENT_ID_H


namespace physx { namespace profile {
	/**
	\brief A event id structure. Optionally includes information about
	if the event was enabled at compile time.
	*/
	struct PxProfileEventId
	{
		uint16_t		eventId;
		mutable bool	compileTimeEnabled; 

		/**
		\brief Profile event id constructor.
		\param inId Profile event id.
		\param inCompileTimeEnabled Compile time enabled.
		*/
		PxProfileEventId( uint16_t inId = 0, bool inCompileTimeEnabled = true )
			: eventId( inId )
			, compileTimeEnabled( inCompileTimeEnabled )
		{
		}

		operator uint16_t () const { return eventId; }

		bool operator==( const PxProfileEventId& inOther ) const 
		{ 
			return eventId == inOther.eventId;
		}
	};

} }

#endif

