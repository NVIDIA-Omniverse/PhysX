// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_INTERSECTION_BOX_BOX_H
#define GU_INTERSECTION_BOX_BOX_H

#include "foundation/PxMat33.h"
#include "foundation/PxBounds3.h"
#include "GuBox.h"

namespace physx
{
namespace Gu
{
	PX_PHYSX_COMMON_API bool intersectOBBOBB(const PxVec3& e0, const PxVec3& c0, const PxMat33& r0, const PxVec3& e1, const PxVec3& c1, const PxMat33& r1, bool full_test);

	PX_FORCE_INLINE bool intersectOBBAABB(const Gu::Box& obb, const PxBounds3& aabb)
	{
		PxVec3 center = aabb.getCenter();
		PxVec3 extents = aabb.getExtents();
		return intersectOBBOBB(obb.extents, obb.center, obb.rot, extents, center, PxMat33(PxIdentity), true);
	}

} // namespace Gu

}

#endif
