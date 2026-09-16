// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_CONVEX_GEOMETRY_H
#define GU_CONVEX_GEOMETRY_H

#include "foundation/PxVec3.h"
#include "foundation/PxMat33.h"
#include "foundation/PxBounds3.h"

namespace physx
{
	class PxConvexCoreGeometry;
	class PxBounds3;
	class PxRenderOutput;

	namespace Gu
	{
		struct ConvexShape;

		PX_PHYSX_COMMON_API bool makeConvexShape(const PxGeometry& geom, const PxTransform& pose, ConvexShape& convex);

		PX_PHYSX_COMMON_API bool isGPUCompatible(const PxConvexCoreGeometry& convex);

		PX_PHYSX_COMMON_API void computeMassInfo(const PxConvexCoreGeometry& convex, PxReal& density1Mass, PxMat33& inertiaTensor, PxVec3& centerOfMass);

		PX_PHYSX_COMMON_API void visualize(const PxConvexCoreGeometry& convex, const PxTransform& pose, bool drawCore, const PxBounds3& cullbox, PxRenderOutput& out);
	}
}

#endif
