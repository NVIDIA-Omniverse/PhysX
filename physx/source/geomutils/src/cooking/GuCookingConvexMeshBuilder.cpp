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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "GuConvexMesh.h"
#include "foundation/PxMathUtils.h"
#include "foundation/PxAlloca.h"
#include "GuCooking.h"
#include "GuBigConvexData2.h"
#include "GuBounds.h"
#include "GuCookingVolumeIntegration.h"
#include "GuCookingConvexMeshBuilder.h"
#include "GuCookingBigConvexDataBuilder.h"
#include "CmUtils.h"
#include "foundation/PxVecMath.h"
#include "GuCookingSDF.h"

using namespace physx;
using namespace Gu;
using namespace aos;

///////////////////////////////////////////////////////////////////////////////

PX_IMPLEMENT_OUTPUT_ERROR

///////////////////////////////////////////////////////////////////////////////

ConvexMeshBuilder::ConvexMeshBuilder(const bool buildGRBData) : hullBuilder(&mHullData, buildGRBData), mSdfData(NULL), mBigConvexData(NULL), mMass(0.0f), mInertia(PxIdentity)
{
}

ConvexMeshBuilder::~ConvexMeshBuilder()
{
	PX_DELETE(mSdfData);
	PX_DELETE(mBigConvexData);
}

// load the mesh data from given polygons
bool ConvexMeshBuilder::build(const PxConvexMeshDesc& desc, PxU32 gaussMapVertexLimit, bool validateOnly, ConvexHullLib* hullLib)
{
	if(!desc.isValid())
		return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "Gu::ConvexMesh::loadFromDesc: desc.isValid() failed!");

	if(!loadConvexHull(desc, hullLib))
		return false;

	// Compute local bounds (*after* hull has been created)
	PxBounds3 minMaxBounds;
	computeBoundsAroundVertices(minMaxBounds, mHullData.mNbHullVertices, hullBuilder.mHullDataHullVertices);
	mHullData.mAABB = CenterExtents(minMaxBounds);

	if(mHullData.mNbHullVertices > gaussMapVertexLimit)
	{
		if(!computeGaussMaps())
		{
			return false;
		}
	}

	if(validateOnly)
		return true;

// TEST_INTERNAL_OBJECTS
	computeInternalObjects();
//~TEST_INTERNAL_OBJECTS

	if (desc.sdfDesc)
	{
		computeSDF(desc);
	}

	return true;
}

PX_COMPILE_TIME_ASSERT(sizeof(PxMaterialTableIndex)==sizeof(PxU16));
bool ConvexMeshBuilder::save(PxOutputStream& stream, bool platformMismatch) const
{
	// Export header
	if(!writeHeader('C', 'V', 'X', 'M', PX_CONVEX_VERSION, platformMismatch, stream))
		return false;

	// Export serialization flags
	PxU32 serialFlags = 0;

	writeDword(serialFlags, platformMismatch, stream);

	if(!hullBuilder.save(stream, platformMismatch))
		return false;

	// Export local bounds
//	writeFloat(geomEpsilon, platformMismatch, stream);
	writeFloat(0.0f, platformMismatch, stream);
	writeFloat(mHullData.mAABB.getMin(0), platformMismatch, stream);
	writeFloat(mHullData.mAABB.getMin(1), platformMismatch, stream);
	writeFloat(mHullData.mAABB.getMin(2), platformMismatch, stream);
	writeFloat(mHullData.mAABB.getMax(0), platformMismatch, stream);
	writeFloat(mHullData.mAABB.getMax(1), platformMismatch, stream);
	writeFloat(mHullData.mAABB.getMax(2), platformMismatch, stream);

	// Export mass info
	writeFloat(mMass, platformMismatch, stream);
	writeFloatBuffer(reinterpret_cast<const PxF32*>(&mInertia), 9, platformMismatch, stream);
	writeFloatBuffer(&mHullData.mCenterOfMass.x, 3, platformMismatch, stream);

	// Export gaussmaps
	if(mBigConvexData)
	{
		writeFloat(1.0f, platformMismatch, stream);		//gauss map flag true
		BigConvexDataBuilder SVMB(&mHullData, mBigConvexData, hullBuilder.mHullDataHullVertices);
		SVMB.save(stream, platformMismatch);
	}
	else
		writeFloat(-1.0f, platformMismatch, stream);	//gauss map flag false

	if (mSdfData)
	{
		writeFloat(1.0f, platformMismatch, stream);		//sdf flag true
		// Export sdf values
		writeFloat(mSdfData->mMeshLower.x, platformMismatch, stream);
		writeFloat(mSdfData->mMeshLower.y, platformMismatch, stream);
		writeFloat(mSdfData->mMeshLower.z, platformMismatch, stream);
		writeFloat(mSdfData->mSpacing, platformMismatch, stream);
		writeDword(mSdfData->mDims.x, platformMismatch, stream);
		writeDword(mSdfData->mDims.y, platformMismatch, stream);
		writeDword(mSdfData->mDims.z, platformMismatch, stream);
		writeDword(mSdfData->mNumSdfs, platformMismatch, stream);

		writeDword(mSdfData->mNumSubgridSdfs, platformMismatch, stream);
		writeDword(mSdfData->mNumStartSlots, platformMismatch, stream);
		writeDword(mSdfData->mSubgridSize, platformMismatch, stream);
		writeDword(mSdfData->mSdfSubgrids3DTexBlockDim.x, platformMismatch, stream);
		writeDword(mSdfData->mSdfSubgrids3DTexBlockDim.y, platformMismatch, stream);
		writeDword(mSdfData->mSdfSubgrids3DTexBlockDim.z, platformMismatch, stream);

		writeFloat(mSdfData->mSubgridsMinSdfValue, platformMismatch, stream);
		writeFloat(mSdfData->mSubgridsMaxSdfValue, platformMismatch, stream);
		writeDword(mSdfData->mBytesPerSparsePixel, platformMismatch, stream);

		writeFloatBuffer(mSdfData->mSdf, mSdfData->mNumSdfs, platformMismatch, stream);
		writeByteBuffer(mSdfData->mSubgridSdf, mSdfData->mNumSubgridSdfs, stream);
		writeIntBuffer(mSdfData->mSubgridStartSlots, mSdfData->mNumStartSlots, platformMismatch, stream);
	}
	else
		writeFloat(-1.0f, platformMismatch, stream);	//sdf flag false

// TEST_INTERNAL_OBJECTS
	writeFloat(mHullData.mInternal.mRadius, platformMismatch, stream);
	writeFloat(mHullData.mInternal.mExtents[0], platformMismatch, stream);
	writeFloat(mHullData.mInternal.mExtents[1], platformMismatch, stream);
	writeFloat(mHullData.mInternal.mExtents[2], platformMismatch, stream);
//~TEST_INTERNAL_OBJECTS
	return true;
}

//////////////////////////////////////////////////////////////////////////
// instead of saving the data into stream, we copy the mesh data
// into internal Gu::ConvexMesh. 
bool ConvexMeshBuilder::copy(Gu::ConvexHullInitData& hullData)
{
	// hull builder data copy
	PxU32 nb = 0;
	hullBuilder.copy(hullData.mHullData, nb);
	hullData.mNb = nb;

	hullData.mInertia = mInertia;
	hullData.mMass = mMass;	

	// mass props
	hullData.mHullData.mAABB = mHullData.mAABB;
	hullData.mHullData.mCenterOfMass = mHullData.mCenterOfMass;

	// big convex data
	if(mBigConvexData)
	{				
		hullData.mHullData.mBigConvexRawData = &mBigConvexData->mData;
		hullData.mBigConvexData = mBigConvexData;
		mBigConvexData = NULL;
	}
	else
	{
		hullData.mHullData.mBigConvexRawData = NULL;
		hullData.mBigConvexData = NULL;
	}

	if (mSdfData)
	{
		hullData.mHullData.mSdfData = mSdfData;
		hullData.mSdfData = mSdfData;
	}
	else
	{
		hullData.mHullData.mSdfData = NULL;
		hullData.mSdfData = NULL;
		mSdfData = NULL;
	}

	// internal data
	hullData.mHullData.mInternal.mRadius = mHullData.mInternal.mRadius;
	hullData.mHullData.mInternal.mExtents[0] = mHullData.mInternal.mExtents[0];
	hullData.mHullData.mInternal.mExtents[1] = mHullData.mInternal.mExtents[1];
	hullData.mHullData.mInternal.mExtents[2] = mHullData.mInternal.mExtents[2];

	return true;
}

// compute mass and inertia of the convex mesh
void ConvexMeshBuilder::computeMassInfo(bool lowerPrecision)
{
	if(mMass <= 0.0f)		//not yet computed.
	{
		PxIntegrals integrals;
		PxConvexMeshDesc meshDesc;
		meshDesc.points.count = mHullData.mNbHullVertices;
		meshDesc.points.data = hullBuilder.mHullDataHullVertices;
		meshDesc.points.stride = sizeof(PxVec3);

		meshDesc.polygons.data = hullBuilder.mHullDataPolygons;
		meshDesc.polygons.stride = sizeof(Gu::HullPolygonData);
		meshDesc.polygons.count = hullBuilder.mHull->mNbPolygons;

		meshDesc.indices.data = hullBuilder.mHullDataVertexData8;

		// using the centroid of the convex for the volume integration solved accuracy issues in cases where the inertia tensor
		// ended up close to not being positive definite and after a few further transforms the diagonalized inertia tensor ended
		// up with negative values.
		PxVec3 mean(0.0f);
		for(PxU32 i=0; i < mHullData.mNbHullVertices; i++)
			mean += hullBuilder.mHullDataHullVertices[i];
		mean *= (1.0f / mHullData.mNbHullVertices);
		
		if(computeVolumeIntegralsEberly(meshDesc, 1.0f, integrals, mean, lowerPrecision))
		{
			integrals.getOriginInertia(mInertia);
			mHullData.mCenterOfMass = integrals.COM;

			//note: the mass will be negative for an inside-out mesh!
			if(mInertia.column0.isFinite() && mInertia.column1.isFinite() && mInertia.column2.isFinite() 
				&& mHullData.mCenterOfMass.isFinite() && PxIsFinite(PxReal(integrals.mass)))
			{
				if (integrals.mass < 0)
				{
					outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "Gu::ConvexMesh: Mesh has a negative volume! Is it open or do (some) faces have reversed winding? (Taking absolute value.)");
					integrals.mass = -integrals.mass;
					mInertia = -mInertia;
				}

				mMass = PxReal(integrals.mass);	//set mass to valid value.
				return;
			}
		}
		outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Gu::ConvexMesh: Error computing mesh mass properties!\n");
	}
}

#if PX_VC
#pragma warning(push)
#pragma warning(disable:4996)	// permitting use of gatherStrided until we have a replacement.
#endif

bool ConvexMeshBuilder::loadConvexHull(const PxConvexMeshDesc& desc, ConvexHullLib* hullLib)
{
	// gather points
	PxVec3* geometry = reinterpret_cast<PxVec3*>(PxAlloca(sizeof(PxVec3)*desc.points.count));
	immediateCooking::gatherStrided(desc.points.data, geometry, desc.points.count, sizeof(PxVec3), desc.points.stride);

	PxU32* topology = NULL;

	// gather indices
	// store the indices into topology if we have the polygon data
	if(desc.indices.data)
	{
		topology = reinterpret_cast<PxU32*>(PxAlloca(sizeof(PxU32)*desc.indices.count));
		if (desc.flags & PxConvexFlag::e16_BIT_INDICES)
		{
			// conversion; 16 bit index -> 32 bit index & stride
			PxU32* dest = topology;
			const PxU32* pastLastDest = topology + desc.indices.count;
			const PxU8* source = reinterpret_cast<const PxU8*>(desc.indices.data);
			while (dest < pastLastDest)
			{
				const PxU16 * trig16 = reinterpret_cast<const PxU16*>(source);
				*dest++ = *trig16;
				source += desc.indices.stride;
			}
		}
		else
		{
			immediateCooking::gatherStrided(desc.indices.data, topology, desc.indices.count, sizeof(PxU32), desc.indices.stride);
		}
	}

	// gather polygons
	PxHullPolygon* hullPolygons = NULL;
	if(desc.polygons.data)
	{
		hullPolygons = reinterpret_cast<PxHullPolygon*>(PxAlloca(sizeof(PxHullPolygon)*desc.polygons.count));
		immediateCooking::gatherStrided(desc.polygons.data,hullPolygons,desc.polygons.count,sizeof(PxHullPolygon),desc.polygons.stride);

		// if user polygons, make sure the largest one is the first one
		if (!hullLib)
		{
			PxU32 largestPolygon = 0;
			for (PxU32 i = 1; i < desc.polygons.count; i++)
			{
				if(hullPolygons[i].mNbVerts > hullPolygons[largestPolygon].mNbVerts)
					largestPolygon = i;
			}
			if(largestPolygon != 0)
			{
				PxHullPolygon movedPolygon = hullPolygons[0];
				hullPolygons[0] = hullPolygons[largestPolygon];
				hullPolygons[largestPolygon] = movedPolygon;
			}
		}
	}
	
	const bool doValidation = desc.flags & PxConvexFlag::eDISABLE_MESH_VALIDATION ? false : true;
  	if(!hullBuilder.init(desc.points.count, geometry, topology, desc.indices.count, desc.polygons.count, hullPolygons, doValidation, hullLib))
		return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Gu::ConvexMesh::loadConvexHull: convex hull init failed!");

	computeMassInfo(desc.flags & PxConvexFlag::eFAST_INERTIA_COMPUTATION);
  
	return true;
}

#if PX_VC
#pragma warning(pop)
#endif

// compute polygons from given triangles. This is support function used in extensions. We do not accept triangles as an input for convex mesh desc.
bool ConvexMeshBuilder::computeHullPolygons(const PxU32& nbVerts,const PxVec3* verts, const PxU32& nbTriangles, const PxU32* triangles, PxAllocatorCallback& inAllocator,
	PxU32& outNbVerts, PxVec3*& outVertices , PxU32& nbIndices, PxU32*& indices, PxU32& nbPolygons, PxHullPolygon*& polygons)
{
	if(!hullBuilder.computeHullPolygons(nbVerts,verts,nbTriangles,triangles))
		return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "ConvexMeshBuilder::computeHullPolygons: compute convex hull polygons failed. Provided triangles dont form a convex hull.");

	outNbVerts = hullBuilder.mHull->mNbHullVertices;
	nbPolygons = hullBuilder.mHull->mNbPolygons; 

	outVertices = reinterpret_cast<PxVec3*>(inAllocator.allocate(outNbVerts*sizeof(PxVec3),"PxVec3",__FILE__,__LINE__));
	PxMemCopy(outVertices,hullBuilder.mHullDataHullVertices,outNbVerts*sizeof(PxVec3));

	nbIndices = 0;
	for (PxU32 i = 0; i < nbPolygons; i++)
	{
		nbIndices += hullBuilder.mHullDataPolygons[i].mNbVerts;
	}

	indices = reinterpret_cast<PxU32*>(inAllocator.allocate(nbIndices*sizeof(PxU32),"PxU32",__FILE__,__LINE__));
	for (PxU32 i = 0; i < nbIndices; i++)
	{
		indices[i] = hullBuilder.mHullDataVertexData8[i];
	}

	polygons = reinterpret_cast<PxHullPolygon*>(inAllocator.allocate(nbPolygons*sizeof(PxHullPolygon),"PxHullPolygon",__FILE__,__LINE__));

	for (PxU32 i = 0; i < nbPolygons; i++)
	{
		const Gu::HullPolygonData& polygonData = hullBuilder.mHullDataPolygons[i];
		PxHullPolygon& outPolygon = polygons[i];
		outPolygon.mPlane[0] = polygonData.mPlane.n.x;
		outPolygon.mPlane[1] = polygonData.mPlane.n.y;
		outPolygon.mPlane[2] = polygonData.mPlane.n.z;
		outPolygon.mPlane[3] = polygonData.mPlane.d;

		outPolygon.mNbVerts = polygonData.mNbVerts;
		outPolygon.mIndexBase = polygonData.mVRef8;

		for (PxU32 j = 0; j < polygonData.mNbVerts; j++)
		{
			PX_ASSERT(indices[outPolygon.mIndexBase + j] == hullBuilder.mHullDataVertexData8[polygonData.mVRef8+j]);
		}
	}

	return true;
}

// compute big convex data
bool ConvexMeshBuilder::computeGaussMaps()
{
	// The number of polygons is limited to 256 because the gaussmap encode 256 polys maximum

	PxU32 density = 16;
	//	density = 64;
	//	density = 8;
	//	density = 2;

	PX_DELETE(mBigConvexData);	
	PX_NEW_SERIALIZED(mBigConvexData,BigConvexData);	
	BigConvexDataBuilder SVMB(&mHullData, mBigConvexData, hullBuilder.mHullDataHullVertices);
	// valencies we need to compute first, they are needed for min/max precompute	
	SVMB.computeValencies(hullBuilder);
	SVMB.precompute(density);
	
	return true;
}

// TEST_INTERNAL_OBJECTS

static void ComputeInternalExtent(Gu::ConvexHullData& data, const Gu::HullPolygonData* hullPolys)
{	
	const PxVec3 e = data.mAABB.getMax() - data.mAABB.getMin();

	// PT: For that formula, see %SDKRoot%\InternalDocumentation\Cooking\InternalExtents.png
	const float r = data.mInternal.mRadius / sqrtf(3.0f);	

	const float epsilon = 1E-7f;

	const PxU32 largestExtent = PxLargestAxis(e);
	PxU32 e0 = PxGetNextIndex3(largestExtent);
	PxU32 e1 = PxGetNextIndex3(e0);	
	if(e[e0] < e[e1])
		PxSwap<PxU32>(e0,e1);

	data.mInternal.mExtents[0] = FLT_MAX;
	data.mInternal.mExtents[1] = FLT_MAX;
	data.mInternal.mExtents[2] = FLT_MAX;

	// PT: the following code does ray-vs-plane raycasts.

	// find the largest box along the largest extent, with given internal radius
	for(PxU32 i = 0; i < data.mNbPolygons; i++)
	{
		// concurrent with search direction
		const float d = hullPolys[i].mPlane.n[largestExtent];
		if((-epsilon < d && d < epsilon))
			continue;

		const float numBase = -hullPolys[i].mPlane.d - hullPolys[i].mPlane.n.dot(data.mCenterOfMass); 
		const float denBase = 1.0f/hullPolys[i].mPlane.n[largestExtent];
		const float numn0 = r * hullPolys[i].mPlane.n[e0];
		const float numn1 = r * hullPolys[i].mPlane.n[e1];

		float num = numBase - numn0 - numn1;
		float ext = PxMax(fabsf(num*denBase), r);
		if(ext < data.mInternal.mExtents[largestExtent])
			data.mInternal.mExtents[largestExtent] = ext;

		num = numBase - numn0 + numn1;
		ext = PxMax(fabsf(num *denBase), r);
		if(ext < data.mInternal.mExtents[largestExtent])
			data.mInternal.mExtents[largestExtent] = ext;

		num = numBase + numn0 + numn1;
		ext = PxMax(fabsf(num *denBase), r);
		if(ext < data.mInternal.mExtents[largestExtent])
			data.mInternal.mExtents[largestExtent] = ext;

		num = numBase + numn0 - numn1;
		ext = PxMax(fabsf(num *denBase), r);
		if(ext < data.mInternal.mExtents[largestExtent])
			data.mInternal.mExtents[largestExtent] = ext;
	}

	// Refine the box along e0,e1
	for(PxU32 i = 0; i < data.mNbPolygons; i++)
	{
		const float denumAdd = hullPolys[i].mPlane.n[e0] + hullPolys[i].mPlane.n[e1];
		const float denumSub = hullPolys[i].mPlane.n[e0] - hullPolys[i].mPlane.n[e1];

		const float numBase = -hullPolys[i].mPlane.d - hullPolys[i].mPlane.n.dot(data.mCenterOfMass);		
		const float numn0 = data.mInternal.mExtents[largestExtent] * hullPolys[i].mPlane.n[largestExtent];		

		if(!(-epsilon < denumAdd && denumAdd < epsilon))
		{
			float num = numBase - numn0;
			float ext = PxMax(fabsf(num/ denumAdd), r);
			if(ext < data.mInternal.mExtents[e0])
				data.mInternal.mExtents[e0] = ext;

			num = numBase + numn0;
			ext = PxMax(fabsf(num / denumAdd), r);
			if(ext < data.mInternal.mExtents[e0])
				data.mInternal.mExtents[e0] = ext;
		}

		if(!(-epsilon < denumSub && denumSub < epsilon))		
		{
			float num = numBase - numn0;
			float ext = PxMax(fabsf(num / denumSub), r);
			if(ext < data.mInternal.mExtents[e0])
				data.mInternal.mExtents[e0] = ext;

			num = numBase + numn0;
			ext = PxMax(fabsf(num / denumSub), r);
			if(ext < data.mInternal.mExtents[e0])
				data.mInternal.mExtents[e0] = ext;
		}
	}
	data.mInternal.mExtents[e1] = data.mInternal.mExtents[e0];	
}

//////////////////////////////////////////////////////////////////////////
// compute internal objects, get the internal extent and radius
void ConvexMeshBuilder::computeInternalObjects()
{
	const Gu::HullPolygonData* hullPolys = hullBuilder.mHullDataPolygons;
	Gu::ConvexHullData& data = mHullData;

	// compute the internal radius
	data.mInternal.mRadius = FLT_MAX;
	for(PxU32 i=0;i<data.mNbPolygons;i++)
	{
		const float dist = fabsf(hullPolys[i].mPlane.distance(data.mCenterOfMass));
		if(dist<data.mInternal.mRadius)
			data.mInternal.mRadius = dist;
	}

	ComputeInternalExtent(data, hullPolys);		

	PX_ASSERT(PxVec3(mHullData.mInternal.mExtents[0], mHullData.mInternal.mExtents[1], mHullData.mInternal.mExtents[2]).isFinite());
	PX_ASSERT(mHullData.mInternal.mExtents[0] != 0.0f);
	PX_ASSERT(mHullData.mInternal.mExtents[1] != 0.0f);
	PX_ASSERT(mHullData.mInternal.mExtents[2] != 0.0f);
}

void ConvexMeshBuilder::computeSDF(const PxConvexMeshDesc& desc)
{

	PX_DELETE(mSdfData);
	PX_NEW_SERIALIZED(mSdfData, SDF);
	//create triangle mesh from polygons
	const PxU32 nbPolygons = mHullData.mNbPolygons;
	PxU32 nbVerts = mHullData.mNbHullVertices;

	const Gu::HullPolygonData* hullPolys = hullBuilder.mHullDataPolygons;
	const PxU8* polygons = hullBuilder.mHullDataVertexData8;
	const PxVec3* verts = hullBuilder.mHullDataHullVertices;
	
	//compute total number of triangles
	PxU32 numTotalTriangles = 0;
	for (PxU32 i = 0; i < nbPolygons; ++i)
	{
		const Gu::HullPolygonData& polyData = hullPolys[i];
		const PxU32 nbTriangles = polyData.mNbVerts - 2;
		numTotalTriangles += nbTriangles;
	}

	PxArray<PxU32> triangleIndice(numTotalTriangles * 3);

	PxU32 startIndex = 0;
	for (PxU32 i = 0; i < nbPolygons; ++i)
	{
		const Gu::HullPolygonData& polyData = hullPolys[i];
		const PxU32 nbTriangles = polyData.mNbVerts - 2;
		const PxU8 vref0 = polygons[polyData.mVRef8];
		
		for (PxU32 j = 0; j < nbTriangles; ++j)
		{
			const PxU32 index = startIndex + j * 3;

			const PxU32 vref1 = polygons[polyData.mVRef8 + 0 + j + 1];
			const PxU32 vref2 = polygons[polyData.mVRef8 + 0 + j + 2];
			triangleIndice[index + 0] = vref0;
			triangleIndice[index + 1] = vref1;
			triangleIndice[index + 2] = vref2;

		}
		startIndex += nbTriangles * 3;
	}

	PxArray<PxReal> sdfData;
	PxArray<PxU8> sdfDataSubgrids;
	PxArray<PxU32> sdfSubgridsStartSlots;

	PxTriangleMeshDesc triDesc;
	triDesc.points.count = nbVerts;
	triDesc.points.stride = sizeof(PxVec3);
	triDesc.points.data = verts;
	triDesc.triangles.count = numTotalTriangles;
	triDesc.triangles.stride = sizeof(PxU32) * 3;
	triDesc.triangles.data = triangleIndice.begin();
	triDesc.flags &= (~PxMeshFlag::e16_BIT_INDICES);
	triDesc.sdfDesc = desc.sdfDesc;

	buildSDF(triDesc, sdfData, sdfDataSubgrids, sdfSubgridsStartSlots);

	PxSDFDesc& sdfDesc = *desc.sdfDesc;

	PxReal* sdf = mSdfData->allocateSdfs(sdfDesc.meshLower, sdfDesc.spacing, sdfDesc.dims.x, sdfDesc.dims.y, sdfDesc.dims.z, 
		sdfDesc.subgridSize, sdfDesc.sdfSubgrids3DTexBlockDim.x, sdfDesc.sdfSubgrids3DTexBlockDim.y, sdfDesc.sdfSubgrids3DTexBlockDim.z, 
		sdfDesc.subgridsMinSdfValue, sdfDesc.subgridsMaxSdfValue, sdfDesc.bitsPerSubgridPixel);

	//copy, and compact to get rid of strides:
	immediateCooking::gatherStrided(sdfDesc.sdf.data, sdf, sdfDesc.dims.x*sdfDesc.dims.y*sdfDesc.dims.z, sizeof(PxReal), sdfDesc.sdf.stride);

}
//~TEST_INTERNAL_OBJECTS
