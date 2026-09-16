// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_COOKING_CONVEX_POLYGONS_BUILDER_H
#define GU_COOKING_CONVEX_POLYGONS_BUILDER_H

#include "GuCookingConvexHullBuilder.h"
#include "GuTriangle.h"

namespace physx
{
	//////////////////////////////////////////////////////////////////////////
	// extended convex hull builder for a case where we build polygons from input triangles
	class ConvexPolygonsBuilder : public ConvexHullBuilder
	{
		public:
														ConvexPolygonsBuilder(Gu::ConvexHullData* hull, const bool buildGRBData);
														~ConvexPolygonsBuilder();

						bool							computeHullPolygons(const PxU32& nbVerts,const PxVec3* verts, const PxU32& nbTriangles, const PxU32* triangles);

		PX_FORCE_INLINE	PxU32							getNbFaces()const	{ return mNbHullFaces; }
		PX_FORCE_INLINE	const Gu::IndexedTriangle32*	getFaces()	const	{ return mFaces; }

		private:
						bool							createPolygonData();
						bool							createTrianglesFromPolygons();
		
						PxU32							mNbHullFaces;	//!< Number of faces in the convex hull
						Gu::IndexedTriangle32*			mFaces;			//!< Triangles.
	};
}

#endif


