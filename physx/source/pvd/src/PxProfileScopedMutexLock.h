// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PROFILE_SCOPED_MUTEX_LOCK_H
#define PX_PROFILE_SCOPED_MUTEX_LOCK_H


namespace physx { namespace profile {

	/**
	 *	Generic class to wrap any mutex type that has lock and unlock methods
	 */
	template<typename TMutexType>
	struct ScopedLockImpl
	{
		TMutexType* mMutex;
		ScopedLockImpl( TMutexType* inM ) : mMutex( inM )
		{
			if ( mMutex ) mMutex->lock();
		}
		~ScopedLockImpl()
		{
			if ( mMutex ) mMutex->unlock();
		}
	};

	/**
	 *	Null locking system that does nothing.
	 */
	struct NullLock
	{
		template<typename TDataType> NullLock( TDataType*) {}
	};
}}

#endif

