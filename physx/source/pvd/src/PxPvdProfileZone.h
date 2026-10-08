// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PVD_PROFILE_ZONE_H
#define PX_PVD_PROFILE_ZONE_H

#include "foundation/PxPreprocessor.h"

#include "PxProfileEventBufferClientManager.h"
#include "PxProfileEventNames.h"
#include "PxProfileEventSender.h"

namespace physx { 
	class PxAllocatorCallback;

	namespace profile {

	class PxProfileZoneManager;	

	/**
	\brief The profiling system was setup in the expectation that there would be several
	 systems that each had its own island of profile information.  PhysX, client code,
	 and APEX would be the first examples of these.  Each one of these islands is represented
	 by a profile zone.
	 
	 A profile zone combines a name, a place where all the events coming from its interface
	 can flushed, and a mapping from event number to full event name.
	 	
	 It also provides a top level filtering service where profile events
	 can be filtered by event id.  
	 
	 The profile zone implements a system where if there is no one
	 listening to events it doesn't provide a mechanism to send them.  In this way
	 the event system is short circuited when there aren't any clients.
	 
	 All functions on this interface should be considered threadsafe.

	 \see PxProfileZoneClientManager, PxProfileNameProvider, PxProfileEventSender, PxProfileEventFlusher
	 */
	class PxProfileZone : public PxProfileZoneClientManager
						, public PxProfileNameProvider
						, public PxProfileEventSender
						, public PxProfileEventFlusher
	{
	protected:
		virtual ~PxProfileZone(){}
	public:
		/**
		\brief Get profile zone name.
		\return Zone name.
		*/
		virtual const char* getName() = 0;
		/**
		\brief Release the profile zone.
		*/
		virtual void release() = 0;

		/**
		\brief Set profile zone manager for the zone.
		\param inMgr Profile zone manager.
		*/
		virtual void setProfileZoneManager(PxProfileZoneManager* inMgr) = 0;
		/**
		\brief Get profile zone manager for the zone.
		\return Profile zone manager.
		*/
		virtual PxProfileZoneManager* getProfileZoneManager() = 0;

		/**
		\brief Get or create a new event id for a given name.
		If you pass in a previously defined event name (including one returned)
		from the name provider) you will just get the same event id back.
		\param inName Profile event name.
		*/
		virtual uint16_t getEventIdForName( const char* inName ) = 0;

		/**
		\brief Specifies that it is a safe point to flush read-write name map into
		read-only map. Make sure getEventIdForName is not called from a different thread.
		*/
		virtual void flushEventIdNameMap() = 0;

		/**
		\brief Reserve a contiguous set of profile event ids for a set of names.
			
		This function does not do any meaningful error checking other than to ensure
		that if it does generate new ids they are contiguous.  If the first name is already
		registered, that is the ID that will be returned regardless of what other
		names are registered.  Thus either use this function alone (without the above
		function) or don't use it.  
		If you register "one","two","three" and the function returns an id of 4, then
		"one" is mapped to 4, "two" is mapped to 5, and "three" is mapped to 6.

		\param inNames set of names to register.
		\param inLen Length of the name list.

		\return The first id associated with the first name.  The rest of the names
		will be associated with monotonically incrementing uint16_t values from the first
		id.  
		 */
		virtual uint16_t getEventIdsForNames( const char** inNames, uint32_t inLen ) = 0;

		/**
		\brief Create a new profile zone.  

		\param inAllocator memory allocation is controlled through the foundation if one is passed in.
		\param inSDKName Name of the profile zone; useful for clients to understand where events came from.
		\param inNames Mapping from event id -> event name.
		\param inEventBufferByteSize Size of the canonical event buffer.  This does not need to be a large number
			as profile events are fairly small individually.
		\return a profile zone implementation.
		 */		
		static PxProfileZone& createProfileZone(PxAllocatorCallback* inAllocator, const char* inSDKName, PxProfileNames inNames = PxProfileNames(), uint32_t inEventBufferByteSize = 0x10000 /*64k*/);

	};
} }

#endif

