// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PROFILE_ZONE_MANAGER_H
#define PX_PROFILE_ZONE_MANAGER_H

#include "PxProfileEventSender.h"
#include "PxProfileEventNames.h"

namespace physx { 
	
	class PxAllocatorCallback;
	
	namespace profile {

	class PxProfileZone;
	class PxProfileNameProvider;	

	/**
	\brief Profile zone handler for zone add/remove notification.
	*/
	class PxProfileZoneHandler
	{
	protected:
		virtual ~PxProfileZoneHandler(){}
	public:
		/**
		\brief On zone added notification		

		\note Not a threadsafe call; handlers are expected to be able to handle
		this from any thread.

		\param inSDK Added zone.
		*/
		virtual void onZoneAdded( PxProfileZone& inSDK ) = 0;
		/**
		\brief On zone removed notification		

		\note Not a threadsafe call; handlers are expected to be able to handle
		this from any thread.

		\param inSDK removed zone.
		*/
		virtual void onZoneRemoved( PxProfileZone& inSDK ) = 0;
	};

	/**
	\brief The profiling system was setup in the expectation that there would be several
	systems that each had its own island of profile information.  PhysX, client code,
	and APEX would be the first examples of these.  Each one of these islands is represented
	by a profile zone.
	 	
	The Manager is a singleton-like object where all these different systems can be registered
	so that clients of the profiling system can have one point to capture *all* profiling events.
	 
	Flushing the manager implies that you want to loop through all the profile zones and flush
	each one.

	\see PxProfileEventFlusher
	*/
	class PxProfileZoneManager 
		: public PxProfileEventFlusher //Tell all SDK's to flush their queue of profile events.
	{
	protected:
		virtual ~PxProfileZoneManager(){}
	public:
		/**
		\brief Add new profile zone for the manager.
		\note Threadsafe call, can be done from any thread.  Handlers that are already connected
		will get a new callback on the current thread.

		\param inSDK Profile zone to add.
		 */
		virtual void addProfileZone( PxProfileZone& inSDK ) = 0;
		/**
		\brief Removes profile zone from the manager.
		\note Threadsafe call, can be done from any thread.  Handlers that are already connected
		will get a new callback on the current thread.

		\param inSDK Profile zone to remove.
		 */
		virtual void removeProfileZone( PxProfileZone& inSDK ) = 0;

		/**
		\brief Add profile zone handler callback for the profile zone notifications.

		\note Threadsafe call.  The new handler will immediately be notified about all
		known SDKs.

		\param inHandler Profile zone handler to add.
		 */
		virtual void addProfileZoneHandler( PxProfileZoneHandler& inHandler ) = 0;
		/**
		\brief Removes profile zone handler callback for the profile zone notifications.

		\note Threadsafe call.  The new handler will immediately be notified about all
		known SDKs.

		\param inHandler Profile zone handler to remove.
		 */
		virtual void removeProfileZoneHandler( PxProfileZoneHandler& inHandler ) = 0;


		/**
		\brief Create a new profile zone.  This means you don't need access to a PxFoundation to 
		create your profile zone object, and your object is automatically registered with
		the profile zone manager.
		
		You still need to release your object when you are finished with it.
		\param inSDKName Name of the SDK object.
		\param inNames Option set of event id to name mappings.
		\param inEventBufferByteSize rough maximum size of the event buffer.  May exceed this size
		by sizeof one event.  When full an immediate call to all listeners is made.
		*/
		virtual PxProfileZone& createProfileZone( const char* inSDKName, PxProfileNames inNames = PxProfileNames(), uint32_t inEventBufferByteSize = 0x4000 /*16k*/ ) = 0;

		/**
		\brief Releases the profile manager instance.
		*/
		virtual void release() = 0;
		
		/**
		\brief Create the profile zone manager.
		\param inAllocatorCallback Allocator callback.
		*/
		static PxProfileZoneManager& createProfileZoneManager(PxAllocatorCallback* inAllocatorCallback );
	};

} }

#endif

