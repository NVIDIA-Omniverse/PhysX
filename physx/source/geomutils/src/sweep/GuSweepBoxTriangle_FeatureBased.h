// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_SWEEP_BOX_TRIANGLE_FEATURE_BASED_H
#define GU_SWEEP_BOX_TRIANGLE_FEATURE_BASED_H

#include "foundation/PxVec3.h"
#include "foundation/PxPlane.h"

namespace physx
{
	class PxTriangle;
	
	namespace Gu
	{
		/**
		Sweeps a box against a triangle, using a 'feature-based' approach.

		This is currently only used for computing the box-sweep impact data, in a second pass,
		after the best triangle has been identified using faster approaches (SAT/GJK).

		\warning Returned impact normal is not normalized

		\param tri				[in] the triangle
		\param box				[in] the box
		\param motion			[in] (box) motion vector
		\param oneOverMotion	[in] precomputed inverse of motion vector
		\param hit				[out] impact point
		\param normal			[out] impact normal (warning: not normalized)
		\param d				[in/out] impact distance (please initialize with best current distance)
		\param isDoubleSided	[in] whether triangle is double-sided or not
		\return	true if an impact has been found
		*/
		bool sweepBoxTriangle(	const PxTriangle& tri, const PxBounds3& box,
								const PxVec3& motion, const PxVec3& oneOverMotion,
								PxVec3& hit, PxVec3& normal, PxReal& d, bool isDoubleSided=false);
	} // namespace Gu
}

#endif
