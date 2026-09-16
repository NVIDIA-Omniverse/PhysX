// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PROFILE_EVENT_SENDER_H
#define PX_PROFILE_EVENT_SENDER_H


namespace physx { namespace profile {

	/**
	\brief Tagging interface to indicate an object that is capable of flushing a profile
	event stream at a certain point.
	 */
	class PxProfileEventFlusher
	{
	protected:
		virtual ~PxProfileEventFlusher(){}
	public:
		/**
		\brief Flush profile events. Sends the profile event buffer to hooked clients.
		*/
		virtual void flushProfileEvents() = 0;
	};

	/**
	\brief Sends the full events where the caller must provide the context and thread id.
	 */
	class PxProfileEventSender
	{
	protected:
		virtual ~PxProfileEventSender(){}
	public:
	
		/**
		\brief Use this as a thread id for events that start on one thread and end on another
		*/
		static const uint32_t CrossThreadId = 99999789;

		/**
		\brief Send a start profile event, optionally with a context. Events are sorted by thread
		and context in the client side.
		\param inId Profile event id.
		\param contextId Context id.
		*/
		virtual void startEvent( uint16_t inId, uint64_t contextId) = 0;
		/**
		\brief Send a stop profile event, optionally with a context. Events are sorted by thread
		and context in the client side.
		\param inId Profile event id.
		\param contextId Context id.
		*/
		virtual void stopEvent( uint16_t inId, uint64_t contextId) = 0;

		/**
		\brief Send a start profile event, optionally with a context. Events are sorted by thread
		and context in the client side.
		\param inId Profile event id.
		\param contextId Context id.
		\param threadId Thread id.
		*/
		virtual void startEvent( uint16_t inId, uint64_t contextId, uint32_t threadId) = 0;
		/**
		\brief Send a stop profile event, optionally with a context. Events are sorted by thread
		and context in the client side.
		\param inId Profile event id.
		\param contextId Context id.
		\param threadId Thread id.
		*/
		virtual void stopEvent( uint16_t inId, uint64_t contextId, uint32_t threadId ) = 0;

		virtual void atEvent(uint16_t inId, uint64_t contextId, uint32_t threadId, uint64_t start, uint64_t stop) = 0;

		/**
		\brief Set an specific events value. This is different than the profiling value
		for the event; it is a value recorded and kept around without a timestamp associated
		with it. This value is displayed when the event itself is processed.
		\param inId Profile event id.
		\param contextId Context id.
		\param inValue Value to set for the event.
		 */
		virtual void eventValue( uint16_t inId, uint64_t contextId, int64_t inValue ) = 0;
	};

} }

#endif

