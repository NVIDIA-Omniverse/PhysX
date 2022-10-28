// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.

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

		@see PxProfileEventHandler::parseEventBuffer.
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

