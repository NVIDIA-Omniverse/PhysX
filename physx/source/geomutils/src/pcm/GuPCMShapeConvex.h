// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_PCM_SHAPE_CONVEX_H
#define GU_PCM_SHAPE_CONVEX_H

#include "GuConvexSupportTable.h"
#include "GuPersistentContactManifold.h"
#include "GuShapeConvex.h"

namespace physx
{
	class PxConvexMeshGeometry;

namespace Gu
{
	extern const PxU8 gPCMBoxPolygonData[24];
	
	class PCMPolygonalBox
	{
	public:
									PCMPolygonalBox(const PxVec3& halfSide);

			void					getPolygonalData(Gu::PolygonalData* PX_RESTRICT dst)	const;

			const PxVec3&			mHalfSide;
			PxVec3					mVertices[8];
			Gu::HullPolygonData		mPolygons[6];
	private:
			PCMPolygonalBox& operator=(const PCMPolygonalBox&);
	};

	void getPCMConvexData(const Gu::ConvexHullV& convexHull, bool idtScale, Gu::PolygonalData& polyData);
	bool getPCMConvexData(const PxConvexMeshGeometry& shapeConvex, Cm::FastVertex2ShapeScaling& scaling, PxBounds3& bounds, Gu::PolygonalData& polyData);
}
}

#endif
