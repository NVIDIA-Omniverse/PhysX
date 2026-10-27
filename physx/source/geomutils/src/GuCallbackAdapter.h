// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_CALLBACK_ADAPTER_H
#define GU_CALLBACK_ADAPTER_H

#include "GuPruner.h"
#include "GuPruningPool.h"

namespace physx
{
namespace Gu
{
	struct RaycastCallbackAdapter
	{
		PX_FORCE_INLINE	RaycastCallbackAdapter(PrunerRaycastCallback& pcb, const PruningPool& pool) : mCallback(pcb), mPool(pool)	{}

		PX_FORCE_INLINE bool	invoke(PxReal& distance, PxU32 primIndex)
		{
			return mCallback.invoke(distance, primIndex, mPool.getObjects(), mPool.getTransforms());
		}

		PrunerRaycastCallback&	mCallback;
		const PruningPool&		mPool;
		PX_NOCOPY(RaycastCallbackAdapter)
	};

	struct OverlapCallbackAdapter
	{
		PX_FORCE_INLINE	OverlapCallbackAdapter(PrunerOverlapCallback& pcb, const PruningPool& pool) : mCallback(pcb), mPool(pool)	{}

		PX_FORCE_INLINE bool	invoke(PxU32 primIndex)
		{
			return mCallback.invoke(primIndex, mPool.getObjects(), mPool.getTransforms());
		}

		PrunerOverlapCallback&	mCallback;
		const PruningPool&		mPool;
		PX_NOCOPY(OverlapCallbackAdapter)
	};

}

}

#endif
