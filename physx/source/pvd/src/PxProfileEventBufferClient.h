// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PROFILE_EVENT_BUFFER_CLIENT_H
#define PX_PROFILE_EVENT_BUFFER_CLIENT_H

#include "PxProfileEventNames.h"

namespace physx { namespace profile {
	
	/**
	\brief Client handles the data when an event buffer flushes.  This data
	can be parsed (PxProfileEventHandler.h) as a binary set of events.
	*/
	class PxProfileEventBufferClient
	{
	protected:
		virtual ~PxProfileEventBufferClient(){}
	public:
		/**
		\brief Callback when the event buffer is full. This data is serialized profile events
		and can be read back using: PxProfileEventHandler::parseEventBuffer.

		\param inData Provided buffer data.
		\param inLength Data length.

		\see PxProfileEventHandler::parseEventBuffer.
		 */
		virtual void handleBufferFlush( const uint8_t* inData, uint32_t inLength ) = 0;

		/**
		\brief Happens if something removes all the clients from the manager.
		*/
		virtual void handleClientRemoved() = 0; 
	};

	/**
	\brief Client handles new profile event add.
	*/
	class PxProfileZoneClient : public PxProfileEventBufferClient
	{
	protected:
		virtual ~PxProfileZoneClient(){}
	public:
		/**
		\brief Callback when new profile event is added.

		\param inName Added profile event name.
		*/
		virtual void handleEventAdded( const PxProfileEventName& inName ) = 0;
	};

} }


#endif

