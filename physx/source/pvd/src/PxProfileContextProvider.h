// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PROFILE_CONTEXT_PROVIDER_H
#define PX_PROFILE_CONTEXT_PROVIDER_H


namespace physx { namespace profile {

	struct PxProfileEventExecutionContext
	{
		uint32_t				mThreadId;
		uint8_t					mCpuId;
		uint8_t					mThreadPriority;

		PxProfileEventExecutionContext( uint32_t inThreadId = 0, uint8_t inThreadPriority = 2 /*eThreadPriorityNormal*/, uint8_t inCpuId = 0 )
			: mThreadId( inThreadId )
			, mCpuId( inCpuId )
			, mThreadPriority( inThreadPriority )
		{
		}

		bool operator==( const PxProfileEventExecutionContext& inOther ) const
		{
			return mThreadId == inOther.mThreadId
				&& mCpuId == inOther.mCpuId
				&& mThreadPriority == inOther.mThreadPriority;
		}
	};

} }

#endif

