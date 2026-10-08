// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_BOUND_TRANSFORM_UPDATE_H
#define PXG_BOUND_TRANSFORM_UPDATE_H

#include "foundation/PxSimpleTypes.h"

namespace physx
{
	struct PxgBoundTransformUpdate
	{
		PxU32 indexTo;
		PxU32 indexFrom; //MSB stores info if the bound is new. New is copied from cpu array, not new - from gpu array
	};
}

#endif
