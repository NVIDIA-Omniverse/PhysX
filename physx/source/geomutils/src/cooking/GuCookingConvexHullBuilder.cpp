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

#include "foundation/PxMemory.h"
#include "cooking/PxCooking.h"

#include "GuEdgeList.h"
#include "GuTriangle.h"
#include "GuConvexMesh.h"
#include "GuMeshCleaner.h"
#include "GuCookingConvexHullBuilder.h"
#include "GuCookingConvexHullLib.h"
#include "foundation/PxArray.h"
#include "foundation/PxVecMath.h"
#include "CmRadixSort.h"

// PT: TODO: refactor/revisit this, looks like it comes from an old ICE file

// 7: added mHullDataFacesByVertices8
// 8: added mEdges
// 9: removed duplicite 'C', 'V', 'H', 'L' header
static const physx::PxU32 gVersion = 9;

using namespace physx;
using namespace Gu;
using namespace Cm;
using namespace aos;

#define USE_PRECOMPUTED_HULL_PROJECTION

///////////////////////////////////////////////////////////////////////////////

PX_IMPLEMENT_OUTPUT_ERROR

///////////////////////////////////////////////////////////////////////////////

// default constructor 
ConvexHullBuilder::ConvexHullBuilder(ConvexHullData* hull, const bool buildGRBData) : 
	mHullDataHullVertices		(NULL),
	mHullDataPolygons			(NULL),
	mHullDataVertexData8		(NULL),
	mHullDataFacesByEdges8		(NULL),
	mHullDataFacesByVertices8	(NULL),
	mEdgeData16					(NULL),
	mEdges						(NULL),
	mHull						(hull),
	mBuildGRBData				(buildGRBData)
{
}

//////////////////////////////////////////////////////////////////////////
// default destructor
ConvexHullBuilder::~ConvexHullBuilder()
{
	PX_FREE(mEdgeData16);
	PX_FREE(mEdges);

	PX_FREE(mHullDataHullVertices);
	PX_FREE(mHullDataPolygons);
	PX_FREE(mHullDataVertexData8);
	PX_FREE(mHullDataFacesByEdges8);
	PX_FREE(mHullDataFacesByVertices8);
}

//////////////////////////////////////////////////////////////////////////
// initialize the convex hull
// \param		nbVerts	[in] number of vertices used
// \param		verts	[in] vertices array
// \param		indices	[in] indices array
// \param		nbPolygons	[in] number of polygons
// \param		hullPolygons	[in] polygons array
// \param		doValidation	[in] specifies whether we should run the validation code
// \param		hullLib	[in] if hullLib is provided, we can reuse the hull create data, hulllib is NULL in case of user provided polygons
bool ConvexHullBuilder::init(PxU32 nbVerts, const PxVec3* verts, const PxU32* indices, const PxU32 nbIndices,
	const PxU32 nbPolygons, const PxHullPolygon* hullPolygons, bool doValidation, ConvexHullLib* hullLib)
{
	PX_ASSERT(indices);
	PX_ASSERT(verts);
	PX_ASSERT(hullPolygons);
	PX_ASSERT(nbVerts);
	PX_ASSERT(nbPolygons);

	mHullDataHullVertices			= NULL;
	mHullDataPolygons				= NULL;
	mHullDataVertexData8			= NULL;
	mHullDataFacesByEdges8			= NULL;
	mHullDataFacesByVertices8		= NULL;

	mEdges							= NULL;
	mEdgeData16						= NULL;

	mHull->mNbHullVertices			= PxTo8(nbVerts);
	// allocate additional vec3 for V4 safe load in VolumeInteration
	mHullDataHullVertices			= PX_ALLOCATE(PxVec3, (mHull->mNbHullVertices + 1), "PxVec3");
	PxMemCopy(mHullDataHullVertices, verts, mHull->mNbHullVertices*sizeof(PxVec3));
	
	// Cleanup
	mHull->mNbPolygons = 0;
	PX_FREE(mHullDataVertexData8);
	PX_FREE(mHullDataPolygons);

	if(nbPolygons>255)
  		return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "ConvexHullBuilder::init: convex hull has more than 255 polygons!");

	// Precompute hull polygon structures
	mHull->mNbPolygons = PxTo8(nbPolygons);
	mHullDataPolygons = PX_ALLOCATE(HullPolygonData, mHull->mNbPolygons, "Gu::HullPolygonData");

	mHullDataVertexData8 = PX_ALLOCATE(PxU8, nbIndices, "mHullDataVertexData8");
	PxU8* dest = mHullDataVertexData8;
	for(PxU32 i=0;i<nbPolygons;i++)
	{
		const PxHullPolygon& inPolygon = hullPolygons[i];
		mHullDataPolygons[i].mVRef8 = PxU16(dest - mHullDataVertexData8);	// Setup link for current polygon

		PxU32 numVerts = inPolygon.mNbVerts;
		PX_ASSERT(numVerts>=3);			// Else something very wrong happened...
		mHullDataPolygons[i].mNbVerts = PxTo8(numVerts);

		for (PxU32 j = 0; j < numVerts; j++)
		{
			dest[j] = PxTo8(indices[inPolygon.mIndexBase + j]);
		}

		mHullDataPolygons[i].mPlane = PxPlane(inPolygon.mPlane[0],inPolygon.mPlane[1],inPolygon.mPlane[2],inPolygon.mPlane[3]);				

		// Next one
		dest += numVerts;
	}

	if(!calculateVertexMapTable(nbPolygons, (hullLib != NULL) ? false : true))
		return false;

	// moved create edge list here from save, copy. This is a part of the validation process and
	// we need to create the edge list anyway
	if(!hullLib || !hullLib->createEdgeList(nbIndices, mHullDataVertexData8, &mHullDataFacesByEdges8, &mEdgeData16, &mEdges))
	{
		if (!createEdgeList(doValidation, nbIndices))
			return false;
	}
	else
	{
		mHull->mNbEdges = PxU16(nbIndices/2);
	}
		
#ifdef USE_PRECOMPUTED_HULL_PROJECTION		
	// Loop through polygons	
	for (PxU32 j = 0; j < nbPolygons; j++)
	{
		// Precompute hull projection along local polygon normal
		PxU32 NbVerts = mHull->mNbHullVertices;
		const PxVec3* Verts = mHullDataHullVertices;
		HullPolygonData& polygon = mHullDataPolygons[j];
		PxReal min = PX_MAX_F32;
		PxU8 minIndex = 0xff;
		for (PxU8 i = 0; i < NbVerts; i++)
		{
			float dp = (*Verts++).dot(polygon.mPlane.n);
			if (dp < min)
			{
				min = dp;
				minIndex = i;
			}
		}
		polygon.mMinIndex = minIndex;
	}
#endif
	
	if(doValidation)
		return checkHullPolygons();
	else
		return true;
}

//////////////////////////////////////////////////////////////////////////
// hull polygons check
bool ConvexHullBuilder::checkHullPolygons() const
{
	const PxVec3* hullVerts = mHullDataHullVertices;
	const PxU8* vertexData = mHullDataVertexData8;
	HullPolygonData* hullPolygons = mHullDataPolygons;

	// Check hull validity
	if(!hullVerts || !hullPolygons)	
		return false;

	if(mHull->mNbPolygons<4)
		return false;

	PxVec3 max(-FLT_MAX,-FLT_MAX,-FLT_MAX);

	PxVec3 hullMax = hullVerts[0];
	PxVec3 hullMin = hullVerts[0];

	for(PxU32 j=0;j<mHull->mNbHullVertices;j++)
	{
		const PxVec3& hullVert = hullVerts[j];
		if(fabsf(hullVert.x) > max.x)
			max.x = fabsf(hullVert.x);

		if(fabsf(hullVert.y) > max.y)
			max.y = fabsf(hullVert.y);

		if(fabsf(hullVert.z) > max.z)
			max.z = fabsf(hullVert.z);

		if (hullVert.x > hullMax.x)
		{
			hullMax.x = hullVert.x;			
		}
		else if (hullVert.x < hullMin.x)
		{
			hullMin.x = hullVert.x;			
		}

		if (hullVert.y > hullMax.y)
		{
			hullMax.y = hullVert.y;			
		}
		else if (hullVert.y < hullMin.y)
		{
			hullMin.y = hullVert.y;			
		}

		if (hullVert.z > hullMax.z)
		{
			hullMax.z = hullVert.z;			
		}
		else if (hullVert.z < hullMin.z)
		{
			hullMin.z = hullVert.z;			
		}
	}

	// compute the test epsilon the same way we construct the hull, verts are considered coplanar within this epsilon	
	const float planeTolerance = 0.02f;
	const float testEpsilon = PxMax(planeTolerance * (PxMax(PxAbs(hullMax.x), PxAbs(hullMin.x)) +
		PxMax(PxAbs(hullMax.y), PxAbs(hullMin.y)) +
		PxMax(PxAbs(hullMax.z), PxAbs(hullMin.z))), planeTolerance);

	max += PxVec3(testEpsilon, testEpsilon, testEpsilon);

	PxVec3 testVectors[8];
	bool	foundPlane[8];
	for (PxU32 i = 0; i < 8; i++)
	{
		foundPlane[i] = false;
	}

	testVectors[0] = PxVec3(max.x,max.y,max.z);
	testVectors[1] = PxVec3(max.x,-max.y,-max.z);
	testVectors[2] = PxVec3(max.x,max.y,-max.z);
	testVectors[3] = PxVec3(max.x,-max.y,max.z);
	testVectors[4] = PxVec3(-max.x,max.y,max.z);
	testVectors[5] = PxVec3(-max.x,-max.y,max.z);
	testVectors[6] = PxVec3(-max.x,max.y,-max.z);
	testVectors[7] = PxVec3(-max.x,-max.y,-max.z);


	// Extra convex hull validity check. This is less aggressive than previous convex decomposer!
	// Loop through polygons
	for(PxU32 i=0;i<mHull->mNbPolygons;i++)
	{
		const PxPlane& P = hullPolygons[i].mPlane;

		for (PxU32 k = 0; k < 8; k++)
		{
			if(!foundPlane[k])
			{
				const float d = P.distance(testVectors[k]);
				if(d >= 0)
				{
					foundPlane[k] = true;
				}
			}
		}

		// Test hull vertices against polygon plane
		for(PxU32 j=0;j<mHull->mNbHullVertices;j++)
		{
			// Don't test vertex if it belongs to plane (to prevent numerical issues)
			PxU32 nb = hullPolygons[i].mNbVerts;
			bool discard=false;
			for(PxU32 k=0;k<nb;k++)
			{
				if(vertexData[hullPolygons[i].mVRef8+k]==PxU8(j))
				{
					discard = true;
					break;
				}
			}

			if(!discard)
			{
				const float d = P.distance(hullVerts[j]);
//					if(d>0.0001f)					
				//if(d>0.02f)
				if(d > testEpsilon)
					return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Gu::ConvexMesh::checkHullPolygons: Some hull vertices seems to be too far from hull planes.");
			}
		}
	}

	for (PxU32 i = 0; i < 8; i++)
	{
		if(!foundPlane[i])
			return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Gu::ConvexMesh::checkHullPolygons: Hull seems to have opened volume or do (some) faces have reversed winding?");			
	}
	
	return true;
}

//////////////////////////////////////////////////////////////////////////
// hull data store
PX_COMPILE_TIME_ASSERT(sizeof(EdgeDescData)==8);
PX_COMPILE_TIME_ASSERT(sizeof(EdgeData)==8);
bool ConvexHullBuilder::save(PxOutputStream& stream, bool platformMismatch) const
{
	// Export header
	if(!WriteHeader('C', 'L', 'H', 'L', gVersion, platformMismatch, stream))
		return false;

	// Export figures

	//embed grb flag into mNbEdges
	PxU16 hasGRBData = PxU16(mBuildGRBData);
	hasGRBData = PxU16(hasGRBData << 15);
	PX_ASSERT(mHull->mNbEdges <( (1 << 15) - 1));
	const PxU16 nbEdges = PxU16(mHull->mNbEdges | hasGRBData);
	writeDword(mHull->mNbHullVertices, platformMismatch, stream);
	writeDword(nbEdges, platformMismatch, stream);
	writeDword(computeNbPolygons(), platformMismatch, stream);	// Use accessor to lazy-build
	PxU32 nb=0;
	for(PxU32 i=0;i<mHull->mNbPolygons;i++)
		nb += mHullDataPolygons[i].mNbVerts;
	writeDword(nb, platformMismatch, stream);

	// Export triangles

	writeFloatBuffer(&mHullDataHullVertices->x, PxU32(mHull->mNbHullVertices*3), platformMismatch, stream);

	// Export polygons
	// TODO: allow lazy-evaluation
	// We can't really store the buffer in one run anymore!
	for(PxU32 i=0;i<mHull->mNbPolygons;i++)
	{
		HullPolygonData tmpCopy = mHullDataPolygons[i];
		if(platformMismatch)
			flipData(tmpCopy);

		stream.write(&tmpCopy, sizeof(HullPolygonData));
	}

	// PT: why not storeBuffer here?
	for(PxU32 i=0;i<nb;i++)
		stream.write(&mHullDataVertexData8[i], sizeof(PxU8));

	stream.write(mHullDataFacesByEdges8, PxU32(mHull->mNbEdges*2));
	stream.write(mHullDataFacesByVertices8, PxU32(mHull->mNbHullVertices*3));

	if (mBuildGRBData)
		writeWordBuffer(mEdges, PxU32(mHull->mNbEdges * 2), platformMismatch, stream);

	return true;
}

//////////////////////////////////////////////////////////////////////////
bool ConvexHullBuilder::copy(ConvexHullData& hullData, PxU32& mNb)
{
	// set the numbers
	hullData.mNbHullVertices = mHull->mNbHullVertices;
	PxU16 hasGRBData = PxU16(mBuildGRBData);
	hasGRBData = PxU16(hasGRBData << 15);
	PX_ASSERT(mHull->mNbEdges <((1 << 15) - 1));	
	hullData.mNbEdges = PxU16(mHull->mNbEdges | hasGRBData);
	hullData.mNbPolygons = PxTo8(computeNbPolygons());
	PxU32 nb = 0;
	for (PxU32 i = 0; i < mHull->mNbPolygons; i++)
		nb += mHullDataPolygons[i].mNbVerts;

	mNb = nb;

	PxU32 bytesNeeded = computeBufferSize(hullData, nb);

	// allocate the memory first.	
	void* dataMemory = PX_ALLOC(bytesNeeded, "ConvexHullData data");

	PxU8* address = reinterpret_cast<PxU8*>(dataMemory);

	// set data pointers
	hullData.mPolygons = reinterpret_cast<HullPolygonData*>(address);	address += sizeof(HullPolygonData) * hullData.mNbPolygons;
	PxVec3* dataHullVertices = reinterpret_cast<PxVec3*>(address);		address += sizeof(PxVec3) * hullData.mNbHullVertices;
	PxU8* dataFacesByEdges8 = reinterpret_cast<PxU8*>(address);			address += sizeof(PxU8) * hullData.mNbEdges * 2;
	PxU8* dataFacesByVertices8 = reinterpret_cast<PxU8*>(address);		address += sizeof(PxU8) * hullData.mNbHullVertices * 3;
	PxU16* dataEdges = reinterpret_cast<PxU16*>(address); 				address += hullData.mNbEdges.isBitSet() ? sizeof(PxU16) *hullData.mNbEdges * 2 : 0;
	PxU8* dataVertexData8 = reinterpret_cast<PxU8*>(address);			address += sizeof(PxU8) * nb;	// PT: leave that one last, so that we don't need to serialize "Nb"

	PX_ASSERT(!(size_t(dataHullVertices) % sizeof(PxReal)));
	PX_ASSERT(!(size_t(hullData.mPolygons) % sizeof(PxReal)));
	PX_ASSERT(size_t(address) <= size_t(dataMemory) + bytesNeeded);

	PX_ASSERT(mHullDataHullVertices);
	PX_ASSERT(mHullDataPolygons);
	PX_ASSERT(mHullDataVertexData8);
	PX_ASSERT(mHullDataFacesByEdges8);
	PX_ASSERT(mHullDataFacesByVertices8);

	// copy the data
	PxMemCopy(dataHullVertices, &mHullDataHullVertices->x, PxU32(mHull->mNbHullVertices * 3)*sizeof(float));
	PxMemCopy(hullData.mPolygons, mHullDataPolygons , hullData.mNbPolygons*sizeof(HullPolygonData));
	PxMemCopy(dataVertexData8, mHullDataVertexData8, nb);
	PxMemCopy(dataFacesByEdges8,mHullDataFacesByEdges8, PxU32(mHull->mNbEdges * 2));
	if (mBuildGRBData)
		PxMemCopy(dataEdges, mEdges, PxU32(mHull->mNbEdges * 2) * sizeof(PxU16));
	PxMemCopy(dataFacesByVertices8, mHullDataFacesByVertices8, PxU32(mHull->mNbHullVertices * 3));	
	return true;
}

//////////////////////////////////////////////////////////////////////////
// calculate vertex map table
bool ConvexHullBuilder::calculateVertexMapTable(PxU32 nbPolygons, bool userPolygons)
{
	mHullDataFacesByVertices8 = PX_ALLOCATE(PxU8, mHull->mNbHullVertices*3u, "mHullDataFacesByVertices8");
	PxU8 vertexMarker[256];
	PxMemSet(vertexMarker, 0, mHull->mNbHullVertices);

	for (PxU32 i = 0; i < nbPolygons; i++)
	{
		const HullPolygonData& polygon = mHullDataPolygons[i];
		for (PxU32 k = 0; k < polygon.mNbVerts; ++k)
		{
			const PxU8 index = mHullDataVertexData8[polygon.mVRef8 + k];
			if (vertexMarker[index] < 3)
			{
				//Found a polygon
				mHullDataFacesByVertices8[index*3 + vertexMarker[index]++] = PxTo8(i);
			}
		}
	}

	bool noPlaneShift = false;
	for (PxU32 i = 0; i < mHull->mNbHullVertices; ++i)
	{
		if(vertexMarker[i] != 3)
			noPlaneShift = true;
	}

	if (noPlaneShift)
	{
		//PCM will use the original shape, which means it will have a huge performance drop
		if (!userPolygons)
			outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "ConvexHullBuilder: convex hull does not have vertex-to-face info! Try to use different convex mesh cooking settings.");
		else
			outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "ConvexHullBuilder: convex hull does not have vertex-to-face info! Some of the vertices have less than 3 neighbor polygons. The vertex is most likely inside a polygon or on an edge between 2 polygons, please remove those vertices.");
		for (PxU32 i = 0; i < mHull->mNbHullVertices; ++i)
		{
			mHullDataFacesByVertices8[i * 3 + 0] = 0xFF;
			mHullDataFacesByVertices8[i * 3 + 1] = 0xFF;
			mHullDataFacesByVertices8[i * 3 + 2] = 0xFF;
		}
		return false;
	}

	return true;
}


//////////////////////////////////////////////////////////////////////////
// create edge list
bool ConvexHullBuilder::createEdgeList(bool doValidation, PxU32 nbEdges)
{
	// Code below could be greatly simplified if we assume manifold meshes!

	//feodorb: ok, let's assume manifold meshes, since the code before this change 
	//would fail on non-maniflold meshes anyways

	// We need the adjacency graph for hull polygons, similar to what we have for triangles.
	// - sort the polygon edges and walk them in order
	// - each edge should appear exactly twice since a convex is a manifold mesh without boundary edges
	// - the polygon index is implicit when we walk the sorted list => get the 2 polygons back and update adjacency graph
	//
	// Two possible structures:
	// - polygon to edges: needed for local search (actually: polygon to polygons)
	// - edge to polygons: needed to compute edge normals on-the-fly

	// Below is largely copied from the edge-list code

	// Polygon to edges:
	//
	// We're dealing with convex polygons made of N vertices, defining N edges. For each edge we want the edge in
	// an edge array.
	//
	// Edges to polygon:
	//
	// For each edge in the array, we want two polygon indices - ie an edge.

	// 0) Compute the total size needed for "polygon to edges"
	const PxU32 nbPolygons = mHull->mNbPolygons;
	PxU32 nbEdgesUnshared = nbEdges;

	// in a manifold mesh, each edge is repeated exactly twice as it shares exactly 2 faces
	if (nbEdgesUnshared % 2 != 0)
		return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Cooking::cookConvexMesh: non-manifold mesh cannot be used, invalid mesh!");

	// 1) Get some bytes: I need one EdgesRefs for each face, and some temp buffers	

	// Face indices by edge indices. First face is the one where the edge is ordered from tail to head.
	PX_FREE(mHullDataFacesByEdges8);
	mHullDataFacesByEdges8 = PX_ALLOCATE(PxU8, nbEdgesUnshared, "mHullDataFacesByEdges8");

	PxU32* tempBuffer = PX_ALLOCATE(PxU32, nbEdgesUnshared*8, "tmp");	// Temp storage
	PxU32* bufferAdd = tempBuffer;
	PxU32*	PX_RESTRICT vRefs0		= tempBuffer; tempBuffer += nbEdgesUnshared;
	PxU32*	PX_RESTRICT vRefs1		= tempBuffer; tempBuffer += nbEdgesUnshared;
	PxU32*	polyIndex	= tempBuffer; tempBuffer += nbEdgesUnshared;
	PxU32*	vertexIndex	= tempBuffer; tempBuffer += nbEdgesUnshared;
	PxU32*	polyIndex2 = tempBuffer; tempBuffer += nbEdgesUnshared;
	PxU32*	vertexIndex2 = tempBuffer; tempBuffer += nbEdgesUnshared;
	PxU32*	edgeIndex = tempBuffer; tempBuffer += nbEdgesUnshared;
	PxU32*	edgeData = tempBuffer; tempBuffer += nbEdgesUnshared;	

	// TODO avoroshilov: use the same "tempBuffer"
	bool* flippedVRefs = PX_ALLOCATE(bool, nbEdgesUnshared, "tmp");	// Temp storage

	PxU32* run0 = vRefs0;
	PxU32* run1 = vRefs1;
	PxU32* run2 = polyIndex;
	PxU32* run3 = vertexIndex;
	bool* run4 = flippedVRefs;

	// 2) Create a full redundant list of edges	
	PxU32 edgeCounter = 0;
	for(PxU32 i=0;i<nbPolygons;i++)
	{
		PxU32 nbVerts = mHullDataPolygons[i].mNbVerts;
		const PxU8* PX_RESTRICT Data = mHullDataVertexData8 + mHullDataPolygons[i].mVRef8;

		// Loop through polygon vertices
		for(PxU32 j=0;j<nbVerts;j++)
		{
			PxU32 vRef0 = Data[j];
			PxU32 vRef1 = Data[(j+1)%nbVerts];
			bool flipped = vRef0>vRef1;

			if (flipped)
				physx::PxSwap(vRef0, vRef1);

			*run0++ = vRef0;
			*run1++ = vRef1;
			*run2++ = i;
			*run3++ = j;
			*run4++ = flipped;
			edgeData[edgeCounter] = edgeCounter;
			edgeCounter++;
		}
	}
	PX_ASSERT(PxU32(run0-vRefs0)==nbEdgesUnshared);
	PX_ASSERT(PxU32(run1-vRefs1)==nbEdgesUnshared);

	// 3) Sort the list according to both keys (VRefs0 and VRefs1)
	Cm::RadixSortBuffered sorter;
	const PxU32* PX_RESTRICT sorted = sorter.Sort(vRefs1, nbEdgesUnshared,Cm::RADIX_UNSIGNED).Sort(vRefs0, nbEdgesUnshared,Cm::RADIX_UNSIGNED).GetRanks();

	PX_FREE(mEdges);
	// Edges by their tail and head VRefs. NbEdgesUnshared == nbEdges * 2
	// mEdges[edgeIdx*2 + 0] = tailVref, mEdges[edgeIdx*2 + 1] = headVref
	// Tails and heads should be consistent with face refs, so that the edge is given in the order of
	// his first face and opposite to the order of his second face
	mEdges = PX_ALLOCATE(PxU16, nbEdgesUnshared, "mEdges");

	PX_FREE(mEdgeData16);
	// Face to edge mapping
	mEdgeData16 = PX_ALLOCATE(PxU16, nbEdgesUnshared, "mEdgeData16");

	// TODO avoroshilov: remove this comment
	//mHull->mNbEdges = PxTo16(nbEdgesUnshared / 2);							// #non-redundant edges
	
	mHull->mNbEdges = 0;												// #non-redundant edges

	// 4) Loop through all possible edges
	// - clean edges list by removing redundant edges
	// - create EdgesRef list	
	//	mNbFaces = nbFaces;

	// TODO avoroshilov:
	PxU32 numFacesPerEdgeVerificationCounter = 0;
	
	PxU16* edgeVertOutput = mEdges;

	PxU32 previousRef0 = PX_INVALID_U32;
	PxU32 previousRef1 = PX_INVALID_U32;
	PxU32 previousPolyId = PX_INVALID_U32;

	PxU16 nbHullEdges = 0;
	for (PxU32 i = 0; i < nbEdgesUnshared; i++)
	{
		const PxU32 sortedIndex = sorted[i];							// Between 0 and Nb
		const PxU32 polyID = polyIndex[sortedIndex];					// Poly index
		const PxU32 vertexID = vertexIndex[sortedIndex];				// Poly index
		PxU32 sortedRef0 = vRefs0[sortedIndex];				// (SortedRef0, SortedRef1) is the sorted edge
		PxU32 sortedRef1 = vRefs1[sortedIndex];
		bool flipped = flippedVRefs[sortedIndex];

		if (sortedRef0 != previousRef0 || sortedRef1 != previousRef1)
		{
			// TODO avoroshilov: remove this?
			if (i != 0 && numFacesPerEdgeVerificationCounter != 1)
				return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Cooking::cookConvexMesh: non-manifold mesh cannot be used, invalid mesh!");

			numFacesPerEdgeVerificationCounter = 0;

			// ### TODO: change this in edge list as well
			previousRef0 = sortedRef0;
			previousRef1 = sortedRef1;
			previousPolyId = polyID;

			//feodorb:restore the original order of VRefs (tail and head)
			if (flipped)
				physx::PxSwap(sortedRef0, sortedRef1);

			*edgeVertOutput++ = PxTo16(sortedRef0);
			*edgeVertOutput++ = PxTo16(sortedRef1);

			nbHullEdges++;
		}
		else
		{
			mHullDataFacesByEdges8[(nbHullEdges - 1) * 2] = PxTo8(previousPolyId);
			mHullDataFacesByEdges8[(nbHullEdges - 1) * 2 + 1] = PxTo8(polyID);

			++numFacesPerEdgeVerificationCounter;
		}

		mEdgeData16[mHullDataPolygons[polyID].mVRef8 + vertexID] = PxTo16(i / 2);

		// Create mEdgesRef on the fly

		polyIndex2[i] = polyID;
		vertexIndex2[i] = vertexID;
		edgeIndex[i] = PxU32(nbHullEdges - 1);
	}

	mHull->mNbEdges = nbHullEdges;

	//////////////////////

	// 2) Get some bytes: one Pair structure / edge	
	// create this structure only for validation purpose
	// 3) Create Counters, ie compute the #faces sharing each edge
	if(doValidation)
	{
		//
		sorted = sorter.Sort(vertexIndex2, nbEdgesUnshared, Cm::RADIX_UNSIGNED).Sort(polyIndex2, nbEdgesUnshared, Cm::RADIX_UNSIGNED).GetRanks();

		for (PxU32 i = 0; i < nbEdgesUnshared; i++)	edgeData[i] = edgeIndex[sorted[i]];

		const PxU16 nbToGo = PxU16(mHull->mNbEdges);
		EdgeDescData* edgeToTriangles = PX_ALLOCATE(EdgeDescData, nbToGo, "edgeToTriangles");
		PxMemZero(edgeToTriangles, sizeof(EdgeDescData)*nbToGo);

		PxU32* data = edgeData;
		for(PxU32 i=0;i<nbEdgesUnshared;i++)	// <= maybe not the same Nb
		{
			edgeToTriangles[*data++].Count++;
		}

		// if we don't have a manifold mesh, this can fail... but the runtime would assert in any case
		for (PxU32 i = 0; i < nbToGo; i++)
		{
			if (edgeToTriangles[i].Count != 2)
				return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Cooking::cookConvexMesh: non-manifold mesh cannot be used, invalid mesh!");
		}
		PX_FREE(edgeToTriangles);
	}

	// TODO avoroshilov: use the same "tempBuffer"
	PX_FREE(flippedVRefs);

	// ### free temp ram
	PX_FREE(bufferAdd);

	return true;
}

