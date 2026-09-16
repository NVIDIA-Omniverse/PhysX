// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_PCM_TRIANGLE_CONTACT_GEN_H
#define GU_PCM_TRIANGLE_CONTACT_GEN_H

#include "GuPCMContactGenUtil.h"
#include "GuPersistentContactManifold.h"

namespace physx
{
	class PxTriangleMeshGeometry;
	class PxHeightFieldGeometry;

namespace Gu
{
	bool PCMContactConvexMesh(const Gu::PolygonalData& polyData0, const Gu::SupportLocal* polyMap, const aos::FloatVArg minMargin, const PxBounds3& hullAABB, 
						const PxTriangleMeshGeometry& shapeMesh,
						const PxTransform& transform0, const PxTransform& transform1, PxReal contactDistance, PxContactBuffer& contactBuffer,
						const Cm::FastVertex2ShapeScaling& convexScaling, const Cm::FastVertex2ShapeScaling& meshScaling,
						bool idtConvexScale, bool idtMeshScale,
						Gu::MultiplePersistentContactManifold& multiManifold, PxRenderOutput* renderOutput);

	bool PCMContactConvexHeightfield(const Gu::PolygonalData& polyData0, const Gu::SupportLocal* polyMap, const aos::FloatVArg minMargin, const PxBounds3& hullAABB, 
						const PxHeightFieldGeometry& shapeHeightfield,
						const PxTransform& transform0, const PxTransform& transform1, PxReal contactDistance, PxContactBuffer& contactBuffer,
						const Cm::FastVertex2ShapeScaling& convexScaling, bool idtConvexScale,
						Gu::MultiplePersistentContactManifold& multiManifold, PxRenderOutput* renderOutput);
}
}

#endif
