// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_COOKING_CONVEX_MESH_BUILDER_H
#define GU_COOKING_CONVEX_MESH_BUILDER_H

#include "cooking/PxCooking.h"

#include "GuConvexMeshData.h"
#include "GuCookingConvexPolygonsBuilder.h"
#include "GuSDF.h"

namespace physx
{
	class BigConvexData;
	namespace Gu
	{
		struct ConvexHullInitData;
	}

	//////////////////////////////////////////////////////////////////////////
	// Convex mesh builder, creates the convex mesh from given polygons and creates internal data
	class ConvexMeshBuilder
	{
	public:
									ConvexMeshBuilder(const bool buildGRBData);
									~ConvexMeshBuilder();

				// loads the computed or given convex hull from descriptor. 
				// the descriptor does contain polygons directly, triangles are not allowed
				bool				build(const PxConvexMeshDesc&, PxU32 gaussMapVertexLimit, bool validateOnly = false, ConvexHullLib* hullLib = NULL);

				// save the convex mesh into stream
				bool				save(PxOutputStream& stream, bool platformMismatch)		const;

				// copy the convex mesh into internal convex mesh, which can be directly used then
				bool				copy(Gu::ConvexHullInitData& convexData);

				// loads the convex mesh from given polygons
				bool				loadConvexHull(const PxConvexMeshDesc&, ConvexHullLib* hullLib);

				// computed hull polygons from given triangles
				bool				computeHullPolygons(const PxU32& nbVerts,const PxVec3* verts, const PxU32& nbTriangles, const PxU32* triangles, PxAllocatorCallback& inAllocator,
										 PxU32& outNbVerts, PxVec3*& outVertices, PxU32& nbIndices, PxU32*& indices, PxU32& nbPolygons, PxHullPolygon*& polygons);

				// compute big convex data
				bool				computeGaussMaps();

				// compute mass, inertia tensor
				void				computeMassInfo(bool lowerPrecision);
// TEST_INTERNAL_OBJECTS
				// internal objects
				void				computeInternalObjects();
				bool				checkExtentRadiusRatio();

//~TEST_INTERNAL_OBJECTS

				void				computeSDF(const PxConvexMeshDesc& desc);

				// set big convex data
				void				setBigConvexData(BigConvexData* data) { mBigConvexData = data; }

		mutable	ConvexPolygonsBuilder	hullBuilder;

	protected:
		Gu::ConvexHullData			mHullData;		
		Gu::SDF*					mSdfData;
		BigConvexData*				mBigConvexData;		//!< optional, only for large meshes! PT: redundant with ptr in chull data? Could also be end of other buffer
		PxReal						mMass;				//this is mass assuming a unit density that can be scaled by instances!
		PxMat33						mInertia;			//in local space of mesh!

	};

}

#endif
