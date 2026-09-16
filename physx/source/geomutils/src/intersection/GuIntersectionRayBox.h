// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_INTERSECTION_RAY_BOX_H
#define GU_INTERSECTION_RAY_BOX_H

#include "foundation/PxMathIntrinsics.h"
#include "common/PxPhysXCommonConfig.h"
#include "foundation/PxVecMath.h"

namespace physx
{
namespace Gu
{
	bool	rayAABBIntersect(const PxVec3& minimum, const PxVec3& maximum, const PxVec3& origin, const PxVec3& _dir, PxVec3& coord);
	PxU32	rayAABBIntersect2(const PxVec3& minimum, const PxVec3& maximum, const PxVec3& origin, const PxVec3& _dir, PxVec3& coord, PxReal & t);

	// Collide ray defined by ray origin (rayOrigin) and ray direction (rayDirection)
	// with the bounding box. Returns -1 on no collision and the face index
	// for first intersection if a collision is found together with
	// the distance to the collision points (tnear and tfar)
	//
	// ptchernev:
	// Even though the above is the original comment by Pierre I am quite confident 
	// that the tnear and tfar parameters are parameters along rayDirection of the
	// intersection points:
	//
	// ip0 = rayOrigin + (rayDirection * tnear)
	// ip1 = rayOrigin + (rayDirection * tfar)
	//
	// The return code is:
	// -1 no intersection
	//  0 the ray first hits the plane at aabbMin.x
	//  1 the ray first hits the plane at aabbMin.y
	//  2 the ray first hits the plane at aabbMin.z
	//  3 the ray first hits the plane at aabbMax.x
	//  4 the ray first hits the plane at aabbMax.y
	//  5 the ray first hits the plane at aabbMax.z
	//
	// The return code will be -1 if the RAY does not intersect the AABB.
	// The tnear and tfar values will give the parameters of the intersection 
	// points between the INFINITE LINE and the AABB.
	int PX_PHYSX_COMMON_API intersectRayAABB(	const PxVec3& minimum, const PxVec3& maximum,
												const PxVec3& rayOrigin, const PxVec3& rayDirection,
												float& tnear, float& tfar);

	// Faster version when one-over-dir is available
	int intersectRayAABB(	const PxVec3& minimum, const PxVec3& maximum,
							const PxVec3& rayOrigin, const PxVec3& rayDirection, const PxVec3& invDirection,
							float& tnear, float& tfar);

	// minimum extent length required for intersectRayAABB2 to return true for a zero-extent box
	// this can happen when inflating the raycast by a 2-d square
	#define GU_MIN_AABB_EXTENT 1e-3f

	// a much faster version that doesn't return face codes
	bool PX_PHYSX_COMMON_API intersectRayAABB2(
		const PxVec3& minimum, const PxVec3& maximum, const PxVec3& ro, const PxVec3& rd, float maxDist, float& tnear, float& tfar);

	bool PX_PHYSX_COMMON_API intersectRayAABB2(	const aos::Vec3VArg minimum, const aos::Vec3VArg maximum, 
												const aos::Vec3VArg ro, const aos::Vec3VArg rd, const aos::FloatVArg maxDist, 
												aos::FloatV& tnear, aos::FloatV& tfar);

} // namespace Gu
}

#endif
