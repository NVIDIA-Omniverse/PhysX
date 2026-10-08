// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_DISTANCE_POINT_BOX_H
#define GU_DISTANCE_POINT_BOX_H

#include "GuBox.h"

namespace physx
{
namespace Gu
{
	/**
	Return the square of the minimum distance from the surface of the box to the given point.
	\param point The point
	\param boxOrigin The origin of the box
	\param boxExtent The extent of the box
	\param boxBase The orientation of the box
	\param boxParam Set to coordinates of the closest point on the box in its local space
	*/
	PX_PHYSX_COMMON_API PxReal distancePointBoxSquared(	const PxVec3& point,
									const PxVec3& boxOrigin, 
									const PxVec3& boxExtent, 
									const PxMat33& boxBase, 
									PxVec3* boxParam=NULL);

	/**
	Return the square of the minimum distance from the surface of the box to the given point.
	\param point The point
	\param box The box
	\param boxParam Set to coordinates of the closest point on the box in its local space
	*/
	PX_FORCE_INLINE PxReal distancePointBoxSquared(	const PxVec3& point, 
													const Gu::Box& box, 
													PxVec3* boxParam=NULL)
	{
		return distancePointBoxSquared(point, box.center, box.extents, box.rot, boxParam);
	}

} // namespace Gu

}

#endif
