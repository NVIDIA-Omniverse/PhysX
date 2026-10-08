// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXC_CONTACT_CACHE_H
#define PXC_CONTACT_CACHE_H

#include "foundation/PxTransform.h"
#include "PxPhysXConfig.h"
#include "PxcContactMethodImpl.h"

namespace physx
{
	class PxcNpThreadContext;

	bool PxcCacheLocalContacts(	PxcNpThreadContext& context, Gu::Cache& pairContactCache,
								const PxTransform32& tm0, const PxTransform32& tm1,
								const PxcContactMethod conMethod,
								const PxGeometry& shape0, const PxGeometry& shape1);

	struct PxcLocalContactsCache
	{
		PxTransform	mTransform0;
		PxTransform	mTransform1;
		PxU16		mNbCachedContacts;
		bool		mUseFaceIndices;
		bool		mSameNormal;

		PX_FORCE_INLINE void operator = (const PxcLocalContactsCache& other)
		{
			mTransform0			= other.mTransform0;
			mTransform1			= other.mTransform1;
			mNbCachedContacts	= other.mNbCachedContacts;
			mUseFaceIndices		= other.mUseFaceIndices;
			mSameNormal			= other.mSameNormal;
		}
	};

}

#endif  // PXC_CONTACT_CACHE_H
