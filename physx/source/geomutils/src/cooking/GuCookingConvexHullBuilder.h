// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_COOKING_CONVEX_HULL_BUILDER_H
#define GU_COOKING_CONVEX_HULL_BUILDER_H

#include "cooking/PxCooking.h"

#include "GuConvexMeshData.h"
#include "foundation/PxUserAllocated.h"

namespace physx
{
	struct PxHullPolygon;
	class ConvexHullLib;

	namespace Gu
	{
		struct EdgeDescData;
		struct ConvexHullData;
	} // namespace Gu

	class ConvexHullBuilder : public PxUserAllocated
	{
		public:
												ConvexHullBuilder(Gu::ConvexHullData* hull, const bool buildGRBData);
												~ConvexHullBuilder();

					bool						init(PxU32 nbVerts, const PxVec3* verts, const PxU32* indices, const PxU32 nbIndices, const PxU32 nbPolygons, 
													const PxHullPolygon* hullPolygons, bool doValidation = true, ConvexHullLib* hullLib = NULL);

					bool						save(PxOutputStream& stream, bool platformMismatch)	const;
					bool						copy(Gu::ConvexHullData& hullData, PxU32& nb);
					
					bool						createEdgeList(bool doValidation, PxU32 nbEdges);
					bool						checkHullPolygons()	const;										

					bool						calculateVertexMapTable(PxU32 nbPolygons, bool userPolygons = false);					

		PX_INLINE	PxU32						computeNbPolygons()		const
												{
													PX_ASSERT(mHull->mNbPolygons);
													return mHull->mNbPolygons;
												}

					PxVec3*						mHullDataHullVertices;
					Gu::HullPolygonData*		mHullDataPolygons;
					PxU8*						mHullDataVertexData8;
					PxU8*						mHullDataFacesByEdges8;
					PxU8*						mHullDataFacesByVertices8;

					PxU16*						mEdgeData16;	//!< Edge indices indexed by hull polygons
					PxU16*						mEdges;			//!< Edge to vertex mapping

					Gu::ConvexHullData*			mHull;
					bool						mBuildGRBData;
	};
}

#endif


