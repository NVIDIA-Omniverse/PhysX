// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CONVEX_FORMAT_H
#define CONVEX_FORMAT_H

#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxPlane.h"
#include "foundation/PxMat44.h"

#include "foundation/PxMemory.h"
#include "foundation/PxUtilities.h"

namespace physx
{

struct ConvexHullCooked
{
	struct Valency
	{
		PxU16		mCount;
		PxU16		mOffset;
	};

	struct BigConvexRawData
	{
		// Support vertex map
		PxU16		mSubdiv;		// "Gaussmap" subdivision
		PxU16		mNbSamples;		// Total #samples in gaussmap PT: this is not even needed at runtime!

		PxU8*		mSamples; //mNbSamples * 2 elements
		PX_FORCE_INLINE const PxU8*	getSamples2()	const
		{
			return mSamples + mNbSamples;
		}
		//~Support vertex map

		// Valencies data
		PxU32			mNbVerts;		//!< Number of vertices
		PxU32			mNbAdjVerts;	//!< Total number of adjacent vertices  ### PT: this is useless at runtime and should not be stored here
		Valency*		mValencies;		//!< A list of mNbVerts valencies (= number of neighbors) mNbVerts elements
		PxU8*			mAdjacentVerts;	//!< List of adjacent vertices
		//~Valencies data
	};

	struct InternalObjectsData
	{
		PxReal	mRadius;
		PxReal	mExtents[3];

		PX_FORCE_INLINE	void reset()
		{
			mRadius = 0.0f;
			mExtents[0] = 0.0f;
			mExtents[1] = 0.0f;
			mExtents[2] = 0.0f;
		}
	};

	struct HullPolygon
	{
		PxPlane	mPlane;			//!< Plane equation for this polygon	//Could drop 4th elem as it can be computed from any vertex as: d = - p.dot(n);
		PxU16	mVRef8;			//!< Offset of vertex references in hull vertex data (CS: can we assume indices are tightly packed and offsets are ascending?? DrawObjects makes and uses this assumption)
		PxU8	mNbVerts;		//!< Number of vertices/edges in the polygon
		PxU8	mMinIndex;		//!< Index of the polygon vertex that has minimal projection along this plane's normal.

		PX_FORCE_INLINE	PxReal getMin(const PxVec3* PX_RESTRICT hullVertices) const	//minimum of projection of the hull along this plane normal
		{ 
			return mPlane.n.dot(hullVertices[mMinIndex]);
		}

		PX_FORCE_INLINE	PxReal getMax() const		{ return -mPlane.d; }	//maximum of projection of the hull along this plane normal
	};

	PxBounds3			mAABB;				
	PxVec3				mCenterOfMass;		

	PxU16				mNbEdges;
	PxU8				mNbHullVertices;	
	PxU8				mNbPolygons;		

	HullPolygon*		mPolygons;			//!< Array of mNbPolygons structures
	PxVec3*				mVertices;			//mNbHullVertices elements

	PxU8*				mFacesByEdges8;		//mNbEdges * 2 elements
	PxU8*				mFacesByVertices8;	//mNbHullVertices * 3 elements
	PxU16*				mVerticesByEdges16; //mNbEdges * 2 elements
	PxU8*				mVertexData8;		//G-d knows how many elements

	BigConvexRawData*	mBigConvexRawData;	//!< Hill climbing data, only for large convexes! else NULL.

	InternalObjectsData	mInternal;

	bool isGpuFriendly()
	{
		bool ret = mNbHullVertices <= 64;

		if(!ret)
			return false;

		for(PxU32 i = 0; i < mNbPolygons; ++i)
			ret &= mPolygons[i].mNbVerts <= 31;

		return ret;
	}
};

}

#endif
