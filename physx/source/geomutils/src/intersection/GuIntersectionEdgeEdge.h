// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_INTERSECTION_EDGE_EDGE_H
#define GU_INTERSECTION_EDGE_EDGE_H

#include "foundation/PxVec3.h"
#include "common/PxPhysXCommonConfig.h"

namespace physx
{
namespace Gu
{

	// collide edge (p1,p2) moving in direction (dir) colliding
	// width edge (p3,p4). Return true on a collision with
	// collision distance (dist) and intersection point (ip)
	// note: dist and ip are invalid if function returns false.
	// note: ip is on (p1,p2), not (p1+dist*dir,p2+dist*dir)
	PX_PHYSX_COMMON_API bool intersectEdgeEdge(const PxVec3& p1, const PxVec3& p2, const PxVec3& dir, const PxVec3& p3, const PxVec3& p4, PxReal& dist, PxVec3& ip);

} // namespace Gu

}

#endif
