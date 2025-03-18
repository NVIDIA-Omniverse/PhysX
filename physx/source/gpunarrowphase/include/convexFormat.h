// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

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
