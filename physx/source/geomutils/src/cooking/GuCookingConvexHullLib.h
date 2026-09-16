// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_COOKING_CONVEX_HULL_LIB_H
#define GU_COOKING_CONVEX_HULL_LIB_H

#include "cooking/PxConvexMeshDesc.h"
#include "cooking/PxCooking.h"

namespace physx
{
	//////////////////////////////////////////////////////////////////////////
	// base class for the convex hull libraries - inflation based and quickhull
	class ConvexHullLib
	{
		PX_NOCOPY(ConvexHullLib)
	public:
		// functions
		ConvexHullLib(const PxConvexMeshDesc& desc, const PxCookingParams& params)
			: mConvexMeshDesc(desc), mCookingParams(params), mSwappedIndices(NULL),
			mShiftedVerts(NULL)
		{
		}

		virtual ~ConvexHullLib();
			
		// computes the convex hull from provided points
		virtual PxConvexMeshCookingResult::Enum createConvexHull() = 0;

		// fills the PxConvexMeshDesc with computed hull data
		virtual void fillConvexMeshDesc(PxConvexMeshDesc& desc) = 0;

		// compute the edge list information if possible
		virtual bool createEdgeList(const PxU32 nbIndices, const PxU8* indices, PxU8** hullDataFacesByEdges8, PxU16** edgeData16, PxU16** edges) = 0;

		static const PxU32 gpuMaxVertsPerFace = 31;

	protected:

		// clean input vertices from duplicates, normalize etc.
		bool cleanupVertices(PxU32 svcount, // input vertex count
			const PxVec3* svertices, // vertices
			PxU32 stride,		// stride
			PxU32& vcount,		// output number of vertices
			PxVec3* vertices);	// location to store the results.			

		// shift vertices around origin and clean input vertices from duplicates, normalize etc.
		bool shiftAndcleanupVertices(PxU32 svcount, // input vertex count
			const PxVec3* svertices, // vertices
			PxU32 stride,		// stride
			PxU32& vcount,		// output number of vertices
			PxVec3* vertices);	// location to store the results.			

		void swapLargestFace(PxConvexMeshDesc& desc);

		void shiftConvexMeshDesc(PxConvexMeshDesc& desc);

	protected:
		const PxConvexMeshDesc&	mConvexMeshDesc;
		const PxCookingParams&	mCookingParams;
		PxU32*					mSwappedIndices;
		PxVec3					mOriginShift;
		PxVec3*					mShiftedVerts;
	};
}

#endif
