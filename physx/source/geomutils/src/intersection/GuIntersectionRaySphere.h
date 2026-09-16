// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_INTERSECTION_RAY_SPHERE_H
#define GU_INTERSECTION_RAY_SPHERE_H

#include "common/PxPhysXCommonConfig.h"

namespace physx
{
namespace Gu
{
	// PT: basic version, limited accuracy, might fail for long rays vs small spheres
	PX_PHYSX_COMMON_API bool intersectRaySphereBasic(const PxVec3& origin, const PxVec3& dir, PxReal length, const PxVec3& center, PxReal radius, PxReal& dist, PxVec3* hit_pos = NULL);

	// PT: version with improved accuracy
	PX_PHYSX_COMMON_API bool intersectRaySphere(const PxVec3& origin, const PxVec3& dir, PxReal length, const PxVec3& center, PxReal radius, PxReal& dist, PxVec3* hit_pos = NULL);

} // namespace Gu

}

#endif
