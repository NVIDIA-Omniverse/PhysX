// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_SWEEP_CAPSULE_TRIANGLE_H
#define GU_SWEEP_CAPSULE_TRIANGLE_H

#include "foundation/PxVec3.h"
#include "PxQueryReport.h"

namespace physx
{
	class PxTriangle;

namespace Gu
{
	class BoxPadded;
	class Capsule;

	/**
	Sweeps a capsule against a set of triangles.

	\param nbTris			[in] number of triangles in input array
	\param triangles		[in] array of input triangles
	\param capsule			[in] the capsule
	\param unitDir			[in] sweep's unit direcion
	\param distance			[in] sweep's length
	\param cachedIndex		[in] cached triangle index, or NULL. Cached triangle will be tested first.
	\param hit				[out] results
	\param triNormalOut		[out] triangle normal
	\param hitFlags			[in] query modifiers
	\param isDoubleSided	[in] true if input triangles are double-sided
	\param cullBox			[in] additional/optional culling box. Triangles not intersecting the box are quickly discarded.
	\warning	if using a cullbox, make sure all triangles can be safely V4Loaded (i.e. allocate 4 more bytes after last triangle)
	\return	true if an impact has been found
	*/
	bool sweepCapsuleTriangles_Precise(	PxU32 nbTris, const PxTriangle* PX_RESTRICT triangles,	// Triangle data
										const Capsule& capsule,									// Capsule data
										const PxVec3& unitDir, PxReal distance,					// Ray data
										const PxU32* PX_RESTRICT cachedIndex,					// Cache data
										PxGeomSweepHit& hit, PxVec3& triNormalOut,				// Results
										PxHitFlags hitFlags, bool isDoubleSided,				// Query modifiers
										const BoxPadded* cullBox=NULL);							// Cull data

} // namespace Gu

}

#endif
