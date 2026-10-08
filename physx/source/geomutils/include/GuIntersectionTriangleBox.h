// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_INTERSECTION_TRIANGLE_BOX_H
#define GU_INTERSECTION_TRIANGLE_BOX_H

#include "foundation/PxMat33.h"
#include "common/PxPhysXCommonConfig.h"

namespace physx
{
namespace Gu
{
	class Box;
	class BoxPadded;

	/**
	Tests if a triangle overlaps a box (AABB). This is the reference non-SIMD code.

	\param center	[in] the box center
	\param extents	[in] the box extents
	\param p0		[in] triangle's first point
	\param p1		[in] triangle's second point
	\param p2		[in] triangle's third point
	\return	true if triangle overlaps box
	*/
	PX_PHYSX_COMMON_API PxIntBool intersectTriangleBox_ReferenceCode(const PxVec3& center, const PxVec3& extents, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2);

	/**
	Tests if a triangle overlaps a box (AABB). This is the optimized SIMD code.

	WARNING: the function has various SIMD requirements, left to the calling code:
	- function will load 4 bytes after 'center'. Make sure it's safe to load from there.
	- function will load 4 bytes after 'extents'. Make sure it's safe to load from there.
	- function will load 4 bytes after 'p0'. Make sure it's safe to load from there.
	- function will load 4 bytes after 'p1'. Make sure it's safe to load from there.
	- function will load 4 bytes after 'p2'. Make sure it's safe to load from there.
	If you can't guarantee these requirements, please use the non-SIMD reference code instead.

	\param center	[in] the box center. 
	\param extents	[in] the box extents
	\param p0		[in] triangle's first point
	\param p1		[in] triangle's second point
	\param p2		[in] triangle's third point
	\return	true if triangle overlaps box
	*/
	PX_PHYSX_COMMON_API PxIntBool intersectTriangleBox_Unsafe(const PxVec3& center, const PxVec3& extents, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2);

	/**
	Tests if a triangle overlaps a box (OBB).

	There are currently no SIMD-related requirements for p0, p1, p2.

	\param box		[in] the box
	\param p0		[in] triangle's first point
	\param p1		[in] triangle's second point
	\param p2		[in] triangle's third point
	\return	true if triangle overlaps box
	*/
	PX_PHYSX_COMMON_API PxIntBool intersectTriangleBox(const BoxPadded& box, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2);
} // namespace Gu
}

#endif
