// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PROFILE_EVENT_BUFFER_CLIENT_MANAGER_H
#define PX_PROFILE_EVENT_BUFFER_CLIENT_MANAGER_H

#include "PxProfileEventBufferClient.h"

namespace physx { namespace profile {
	
	/**
	\brief	Manager keep collections of PxProfileEventBufferClient clients. 

	\see PxProfileEventBufferClient
	*/
	class PxProfileEventBufferClientManager
	{
	protected:
		virtual ~PxProfileEventBufferClientManager(){}
	public:
		/**
		\brief Adds new client.
		\param inClient Client to add.
		*/
		virtual void addClient( PxProfileEventBufferClient& inClient ) = 0;

		/**
		\brief Removes a client.
		\param inClient Client to remove.
		*/
		virtual void removeClient( PxProfileEventBufferClient& inClient ) = 0;

		/**
		\brief Check if manager has clients.
		\return True if manager has added clients.
		*/
		virtual bool hasClients() const = 0;
	};

	/**
	\brief	Manager keep collections of PxProfileZoneClient clients. 

	\see PxProfileZoneClient
	*/
	class PxProfileZoneClientManager
	{
	protected:
		virtual ~PxProfileZoneClientManager(){}
	public:
		/**
		\brief Adds new client.
		\param inClient Client to add.
		*/
		virtual void addClient( PxProfileZoneClient& inClient ) = 0;

		/**
		\brief Removes a client.
		\param inClient Client to remove.
		*/
		virtual void removeClient( PxProfileZoneClient& inClient ) = 0;

		/**
		\brief Check if manager has clients.
		\return True if manager has added clients.
		*/
		virtual bool hasClients() const = 0;
	};
} }

#endif

