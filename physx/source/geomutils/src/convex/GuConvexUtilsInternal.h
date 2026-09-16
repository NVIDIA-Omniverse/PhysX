// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_CONVEX_UTILS_INTERNALS_H
#define GU_CONVEX_UTILS_INTERNALS_H

#include "foundation/PxMat34.h"
#include "foundation/PxBounds3.h"
#include "common/PxPhysXCommonConfig.h"

namespace physx
{
class PxMeshScale;
class PxConvexMeshGeometry;
class PxConvexMesh;

namespace Cm
{
	class FastVertex2ShapeScaling;
}

namespace Gu
{
	class Box;

	void computeHullOBB(
		Gu::Box& hullOBB, const PxBounds3& hullAABB, float offset, const PxMat34& world0,
		const PxMat34& world1, const Cm::FastVertex2ShapeScaling& meshScaling, bool idtScaleMesh);

	// src = input
	// computes a box in vertex space (including skewed scale) from src world box
	void computeVertexSpaceOBB(Gu::Box& dst, const Gu::Box& src, const PxTransform& meshPose, const PxMeshScale& meshScale);

	PX_PHYSX_COMMON_API void computeOBBAroundConvex(
		Gu::Box& obb, const PxConvexMeshGeometry& convexGeom, const PxConvexMesh* cm, const PxTransform& convexPose);

}  // namespace Gu

}

#endif
