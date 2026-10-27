// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXS_CACHED_TRANSFORM_H
#define PXS_CACHED_TRANSFORM_H

#include "foundation/PxTransform.h"

namespace physx
{
	struct PxsTransformFlag
	{
		enum Flags
		{
			eFROZEN = (1 << 0)
		};
	};

	struct PX_ALIGN_PREFIX(16) PxsCachedTransform
	{
		PxTransform transform;
		PxU32 flags;

		PX_FORCE_INLINE PxU32 isFrozenTransform() const { return flags & PxsTransformFlag::eFROZEN; }
	}
	PX_ALIGN_SUFFIX(16);
}

#endif
