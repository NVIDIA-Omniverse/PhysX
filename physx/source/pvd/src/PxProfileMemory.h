// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PROFILE_MEMORY_H
#define PX_PROFILE_MEMORY_H

#include "PxProfileEventBufferClientManager.h"
#include "PxProfileEventSender.h"
#include "foundation/PxBroadcast.h"

namespace physx { namespace profile {

	/**
	\brief Record events so a late-connecting client knows about
	all outstanding allocations
	*/
	class PxProfileMemoryEventRecorder : public PxAllocationListener
	{
	protected:
		virtual ~PxProfileMemoryEventRecorder(){}
	public:
		/**
		\brief Set the allocation listener
		\param inListener Allocation listener.
		*/
		virtual void setListener(PxAllocationListener* inListener) = 0;
		/**
		\brief Release the instance.
		*/
		virtual void release() = 0;
	};

	/**
	\brief Stores memory events into the memory buffer. 
	*/
	class PxProfileMemoryEventBuffer
		: public PxAllocationListener //add a new event to the buffer
		, public PxProfileEventBufferClientManager //add clients to handle the serialized memory events
		, public PxProfileEventFlusher //flush the buffer
	{
	protected:
		virtual ~PxProfileMemoryEventBuffer(){}
	public:

		/**
		\brief Release the instance.
		*/
		virtual void release() = 0;
		
		/**
		\brief Create a non-mutex-protected event buffer.		
		\param inAllocator Allocation callback.
		\param inBufferSize Internal buffer size.
		*/
		static PxProfileMemoryEventBuffer& createMemoryEventBuffer(PxAllocatorCallback& inAllocator, uint32_t inBufferSize = 0x1000);
	};



} } // namespace physx


#endif


