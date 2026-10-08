// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_INTERSECTION_TETRAHEDRON_BOX_H
#define GU_INTERSECTION_TETRAHEDRON_BOX_H

#include "foundation/PxVec3.h"
#include "foundation/PxBounds3.h"
#include "common/PxPhysXCommonConfig.h"

namespace physx
{
namespace Gu
{
	class Box;
	class BoxPadded;

	/**
	Tests if a tetrahedron overlaps a box (AABB).

	\param a	[in] tetrahedron's first point
	\param b	[in] tetrahedron's second point
	\param c	[in] tetrahedron's third point
	\param d	[in] tetrahedron's fourth point
	\param box	[in] The axis aligned box box to check for overlap
	\return	true if tetrahedron overlaps box
	*/
	PX_PHYSX_COMMON_API bool intersectTetrahedronBox(const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, const PxBounds3& box);


} // namespace Gu
}

#endif
