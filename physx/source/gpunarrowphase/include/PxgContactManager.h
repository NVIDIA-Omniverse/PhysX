// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_CONTACT_MANAGER_H
#define PXG_CONTACT_MANAGER_H

#include "foundation/PxSimpleTypes.h"

namespace physx
{
	struct PX_ALIGN_PREFIX(16) PxgContactManagerInput
	{
		//Body refs are not needed here. World-space transforms are computed using transforCacheRefs instead!
		PxU32 shapeRef0;					//Ref to shape 0
		PxU32 shapeRef1;					//Ref to shape 1
		PxU32 transformCacheRef0;			//Ref to shape0's transforms in transform cache
		PxU32 transformCacheRef1;			//Ref to shape1's transform in transform cache
	}
	PX_ALIGN_SUFFIX(16);
}

#endif

