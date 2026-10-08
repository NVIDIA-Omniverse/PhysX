// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_CONVEXHELPER_H
#define GU_CONVEXHELPER_H

#include "GuShapeConvex.h"

namespace physx
{
	class PxConvexMeshGeometry;
namespace Gu
{
	///////////////////////////////////////////////////////////////////////////

	void getScaledConvex(	PxVec3*& scaledVertices, PxU8*& scaledIndices, PxVec3* dstVertices, PxU8* dstIndices,
							bool idtConvexScale, const PxVec3* srcVerts, const PxU8* srcIndices, PxU32 nbVerts, const Cm::FastVertex2ShapeScaling& convexScaling);

	// PT: calling this correctly isn't trivial so let's macroize it. At least we limit the damage since it immediately calls a real function.
	#define GET_SCALEX_CONVEX(scaledVertices, stackIndices, idtScaling, nbVerts, scaling, srcVerts, srcIndices)	\
	getScaledConvex(scaledVertices, stackIndices,																\
					idtScaling ? NULL : reinterpret_cast<PxVec3*>(PxAlloca(nbVerts * sizeof(PxVec3))),			\
					idtScaling ? NULL : reinterpret_cast<PxU8*>(PxAlloca(nbVerts * sizeof(PxU8))),				\
					idtScaling, srcVerts, srcIndices, nbVerts, scaling);

	bool getConvexData(const PxConvexMeshGeometry& shapeConvex, Cm::FastVertex2ShapeScaling& scaling, PxBounds3& bounds, PolygonalData& polyData);

	struct ConvexEdge
	{
		PxU8	vref0;
		PxU8	vref1;
		PxVec3	normal;	// warning: non-unit vector!
	};

	PxU32 findUniqueConvexEdges(PxU32 maxNbEdges, ConvexEdge* PX_RESTRICT edges, PxU32 numPolygons, const Gu::HullPolygonData* PX_RESTRICT polygons, const PxU8* PX_RESTRICT vertexData);
}
}

#endif
