// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PROFILE_EVENT_MUTEX_H
#define PX_PROFILE_EVENT_MUTEX_H


namespace physx { namespace profile {
	
	/**
	 *	Mutex interface that hides implementation around lock and unlock.
	 *	The event system locks the mutex for every interaction.
	 */
	class PxProfileEventMutex
	{
	protected:
		virtual ~PxProfileEventMutex(){}
	public:
		virtual void lock() = 0;
		virtual void unlock() = 0;
	};

	/**
	 * Take any mutex type that implements lock and unlock and make an EventMutex out of it.
	 */
	template<typename TMutexType>
	struct PxProfileEventMutexImpl : public PxProfileEventMutex
	{
		TMutexType* mMutex;
		PxProfileEventMutexImpl( TMutexType* inMtx ) : mMutex( inMtx ) {}
		virtual void lock() PX_OVERRIDE { mMutex->lock(); }
		virtual void unlock() PX_OVERRIDE { mMutex->unlock(); }
	};

} }

#endif

