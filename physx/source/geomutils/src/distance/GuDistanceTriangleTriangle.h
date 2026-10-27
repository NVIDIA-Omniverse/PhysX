// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_DISTANCE_TRIANGLE_TRIANGLE_H
#define GU_DISTANCE_TRIANGLE_TRIANGLE_H

#include "common/PxPhysXCommonConfig.h"
#include "foundation/PxVec3.h"

namespace physx
{
namespace Gu
{
	float	distanceTriangleTriangleSquared(PxVec3& cp, PxVec3& cq, const PxVec3p p[3], const PxVec3p q[3]);

} // namespace Gu
}

#endif
