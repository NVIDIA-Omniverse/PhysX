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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "GuCookingConvexHullLib.h"
#include "GuQuantizer.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxMemory.h"

using namespace physx;
using namespace Gu;

namespace local
{
	//////////////////////////////////////////////////////////////////////////
	// constants	
	static const float DISTANCE_EPSILON = 0.000001f;	// close enough to consider two floating point numbers to be 'the same'.
	static const float RESIZE_VALUE = 0.01f;			// if the provided points AABB is very thin resize it to this size

	//////////////////////////////////////////////////////////////////////////
	// checks if points form a valid AABB cube, if not construct a default CUBE
	static bool checkPointsAABBValidity(PxU32 numPoints, const PxVec3* points, PxU32 stride , float distanceEpsilon,
		float resizeValue, PxU32& vcount, PxVec3* vertices, bool fCheck = false)
	{
		const char* vtx = reinterpret_cast<const char *> (points);
		PxBounds3 bounds;
		bounds.setEmpty();

		// get the bounding box		
		for (PxU32 i = 0; i < numPoints; i++)
		{
			const PxVec3& p = *reinterpret_cast<const PxVec3 *> (vtx);
			vtx += stride;

			bounds.include(p);

			vertices[i] = p;
		}

		PxVec3 dim = bounds.getDimensions();
		PxVec3 center = bounds.getCenter();

		// special case, the AABB is very thin or user provided us with only input 2 points
		// we construct an AABB cube and return it
		if ( dim.x < distanceEpsilon || dim.y < distanceEpsilon || dim.z < distanceEpsilon || numPoints < 3 )
		{
			float len = FLT_MAX;

			// pick the shortest size bigger than the distance epsilon
			if ( dim.x > distanceEpsilon && dim.x < len ) 
				len = dim.x;
			if ( dim.y > distanceEpsilon && dim.y < len ) 
				len = dim.y;
			if ( dim.z > distanceEpsilon && dim.z < len ) 
				len = dim.z;

			// if the AABB is small in all dimensions, resize it
			if ( len == FLT_MAX )
			{
				dim = PxVec3(resizeValue);
			}
			// if one edge is small, set to 1/5th the shortest non-zero edge.
			else
			{
				if ( dim.x < distanceEpsilon )
					dim.x = PxMin(len * 0.05f, resizeValue);
				else
					dim.x *= 0.5f;
				if ( dim.y < distanceEpsilon )
					dim.y = PxMin(len * 0.05f, resizeValue);
				else
					dim.y *= 0.5f;
				if ( dim.z < distanceEpsilon ) 
					dim.z = PxMin(len * 0.05f, resizeValue);
				else
					dim.z *= 0.5f;
			}

			// construct the AABB
			const PxVec3 extPos = center + dim;
			const PxVec3 extNeg = center - dim;

			if(fCheck)
				vcount = 0;

			vertices[vcount++] = extNeg;			
			vertices[vcount++] = PxVec3(extPos.x,extNeg.y,extNeg.z);
			vertices[vcount++] = PxVec3(extPos.x,extPos.y,extNeg.z);			
			vertices[vcount++] = PxVec3(extNeg.x,extPos.y,extNeg.z);			
			vertices[vcount++] = PxVec3(extNeg.x,extNeg.y,extPos.z);			
			vertices[vcount++] = PxVec3(extPos.x,extNeg.y,extPos.z);			
			vertices[vcount++] = extPos;			
			vertices[vcount++] = PxVec3(extNeg.x,extPos.y,extPos.z);			
			return true; // return cube
		}

		vcount = numPoints;
		return false;
	}

}


//////////////////////////////////////////////////////////////////////////
// shift vertices around origin and normalize point cloud, remove duplicates!
bool ConvexHullLib::shiftAndcleanupVertices(PxU32 svcount, const PxVec3* svertices, PxU32 stride,
	PxU32& vcount, PxVec3* vertices)
{
	mShiftedVerts = PX_ALLOCATE(PxVec3, svcount, "PxVec3");
	const char* vtx = reinterpret_cast<const char *> (svertices);
	PxBounds3 bounds;
	bounds.setEmpty();

	// get the bounding box		
	for (PxU32 i = 0; i < svcount; i++)
	{
		const PxVec3& p = *reinterpret_cast<const PxVec3 *> (vtx);
		vtx += stride;

		bounds.include(p);
	}
	mOriginShift = bounds.getCenter();
	vtx = reinterpret_cast<const char *> (svertices);
	for (PxU32 i = 0; i < svcount; i++)
	{
		const PxVec3& p = *reinterpret_cast<const PxVec3 *> (vtx);
		vtx += stride;

		mShiftedVerts[i] = p - mOriginShift;
	}
	return cleanupVertices(svcount, mShiftedVerts, sizeof(PxVec3), vcount, vertices);
}

//////////////////////////////////////////////////////////////////////////
// Shift verts/planes in the desc back
void ConvexHullLib::shiftConvexMeshDesc(PxConvexMeshDesc& desc)
{
	PX_ASSERT(mConvexMeshDesc.flags & PxConvexFlag::eSHIFT_VERTICES);

	PxVec3* points = reinterpret_cast<PxVec3*>(const_cast<void*>(desc.points.data));
	for(PxU32 i = 0; i < desc.points.count; i++)
	{
		points[i] = points[i] + mOriginShift;
	}

	PxHullPolygon* polygons = reinterpret_cast<PxHullPolygon*>(const_cast<void*>(desc.polygons.data));
	for(PxU32 i = 0; i < desc.polygons.count; i++)
	{
		polygons[i].mPlane[3] -= PxVec3(polygons[i].mPlane[0], polygons[i].mPlane[1], polygons[i].mPlane[2]).dot(mOriginShift);
	}
}

//////////////////////////////////////////////////////////////////////////
// normalize point cloud, remove duplicates!
bool ConvexHullLib::cleanupVertices(PxU32 svcount, const PxVec3* svertices, PxU32 stride,
	PxU32& vcount, PxVec3* vertices)
{
	if (svcount == 0) 
		return false;

	const PxVec3* verticesToClean = svertices;
	PxU32 numVerticesToClean = svcount;
	Quantizer* quantizer = NULL;

	// if quantization is enabled, parse the input vertices and produce new qantized vertices, 
	// that will be then cleaned the same way
	if (mConvexMeshDesc.flags & PxConvexFlag::eQUANTIZE_INPUT)
	{
		quantizer = createQuantizer();
		PxU32 vertsOutCount;
		const PxVec3* vertsOut = quantizer->kmeansQuantize3D(svcount, svertices, stride,true, mConvexMeshDesc.quantizedCount, vertsOutCount);

		if (vertsOut)
		{
			numVerticesToClean = vertsOutCount;
			verticesToClean = vertsOut;
		}		
	}

	const float distanceEpsilon = local::DISTANCE_EPSILON * mCookingParams.scale.length;
	const float resizeValue = local::RESIZE_VALUE * mCookingParams.scale.length;	

	vcount = 0;
	// check for the AABB from points, if its very tiny return a resized CUBE
	if (local::checkPointsAABBValidity(numVerticesToClean, verticesToClean, stride, distanceEpsilon, resizeValue, vcount, vertices, false))
	{
		if (quantizer)
			quantizer->release();
		return true;
	}

	if(vcount < 4)
		return PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "ConvexHullLib::cleanupVertices: Less than four valid vertices were found. Provide at least four valid (e.g. each at a different position) vertices.");
	
	if (quantizer)
		quantizer->release();
	return true;
}

void ConvexHullLib::swapLargestFace(PxConvexMeshDesc& desc)
{
	const PxHullPolygon* polygons = reinterpret_cast<const PxHullPolygon*>(desc.polygons.data);
	PxHullPolygon* polygonsOut = const_cast<PxHullPolygon*>(polygons);

	PxU32 largestFace = 0;
	for (PxU32 i = 1; i < desc.polygons.count; i++)
	{		
		if(polygons[largestFace].mNbVerts < polygons[i].mNbVerts)
			largestFace = i;
	}

	// early exit if no swap needs to be done
	if(largestFace == 0)
		return;

	const PxU32* indices = reinterpret_cast<const PxU32*>(desc.indices.data);
	mSwappedIndices = PX_ALLOCATE(PxU32, desc.indices.count, "PxU32");

	PxHullPolygon replacedPolygon = polygons[0];
	PxHullPolygon largestPolygon = polygons[largestFace];
	polygonsOut[0] = polygons[largestFace];
	polygonsOut[largestFace] = replacedPolygon;

	// relocate indices
	PxU16 indexBase = 0;
	for (PxU32 i = 0; i < desc.polygons.count; i++)
	{
		if(i == 0)
		{
			PxMemCopy(mSwappedIndices, &indices[largestPolygon.mIndexBase],sizeof(PxU32)*largestPolygon.mNbVerts);
			polygonsOut[0].mIndexBase = indexBase;
			indexBase += largestPolygon.mNbVerts;
		}
		else
		{
			if(i == largestFace)
			{
				PxMemCopy(&mSwappedIndices[indexBase], &indices[replacedPolygon.mIndexBase], sizeof(PxU32)*replacedPolygon.mNbVerts);
				polygonsOut[i].mIndexBase = indexBase;
				indexBase += replacedPolygon.mNbVerts;
			}
			else
			{
				PxMemCopy(&mSwappedIndices[indexBase], &indices[polygons[i].mIndexBase], sizeof(PxU32)*polygons[i].mNbVerts);
				polygonsOut[i].mIndexBase = indexBase;
				indexBase += polygons[i].mNbVerts;
			}
		}
	}

	PX_ASSERT(indexBase == desc.indices.count);
	
	desc.indices.data = mSwappedIndices;
}

ConvexHullLib::~ConvexHullLib()
{
	PX_FREE(mSwappedIndices);
	PX_FREE(mShiftedVerts);
}
