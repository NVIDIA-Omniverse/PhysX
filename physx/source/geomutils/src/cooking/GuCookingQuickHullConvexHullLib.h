// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_COOKING_QUICKHULL_CONVEXHULLLIB_H
#define GU_COOKING_QUICKHULL_CONVEXHULLLIB_H

#include "GuCookingConvexHullLib.h"
#include "foundation/PxArray.h"
#include "foundation/PxUserAllocated.h"

namespace local
{
	class QuickHull;
	struct QuickHullVertex;
}

namespace physx
{
	class ConvexHull;

	//////////////////////////////////////////////////////////////////////////
	// Quickhull lib constructs the hull from given input points. The resulting hull 
	// will only contain a subset of the input points. The algorithm does incrementally
	// adds most furthest vertices to the starting simplex. The produced hulls are build with high precision
	// and produce more stable and correct results, than the legacy algorithm. 
	class QuickHullConvexHullLib: public ConvexHullLib, public PxUserAllocated
	{
		PX_NOCOPY(QuickHullConvexHullLib)
	public:

		// functions
		QuickHullConvexHullLib(const PxConvexMeshDesc& desc, const PxCookingParams& params);

		~QuickHullConvexHullLib();

		// computes the convex hull from provided points
		virtual PxConvexMeshCookingResult::Enum createConvexHull() PX_OVERRIDE;

		// fills the convexmeshdesc with computed hull data
		virtual void fillConvexMeshDesc(PxConvexMeshDesc& desc) PX_OVERRIDE;

		// provide the edge list information
		virtual bool createEdgeList(const PxU32, const PxU8* , PxU8** , PxU16** , PxU16**) PX_OVERRIDE;

	protected:
		// if vertex limit reached we need to expand the hull using the OBB slicing
		PxConvexMeshCookingResult::Enum expandHullOBB();

		// if vertex limit reached we need to expand the hull using the plane shifting
		PxConvexMeshCookingResult::Enum expandHull();

		// checks for collinearity and co planarity
		// returns true if the simplex was ok, we can reuse the computed tolerances and min/max values
		bool cleanupForSimplex(PxVec3* vertices, PxU32 vertexCount, local::QuickHullVertex* minimumVertex, 
			local::QuickHullVertex* maximumVertex, float& tolerance, float& planeTolerance);

		// fill the result desc from quick hull convex
		void fillConvexMeshDescFromQuickHull(PxConvexMeshDesc& desc);

		// fill the result desc from cropped hull convex
		void fillConvexMeshDescFromCroppedHull(PxConvexMeshDesc& desc);

	private:
		local::QuickHull*		mQuickHull;		// the internal quick hull representation
		ConvexHull*				mCropedConvexHull; //the hull cropped from OBB, used for vertex limit path

		PxU8*					mOutMemoryBuffer;   // memory buffer used for output data
		PxU16*					mFaceTranslateTable; // translation table mapping output faces to internal quick hull table
	};
}

#endif
