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

#include "GuCooking.h"
#include "cooking/PxTriangleMeshDesc.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxSort.h"
#include "foundation/PxFPU.h"
#include "common/PxInsertionCallback.h"

#include "GuRTreeCooking.h"
#include "GuCookingTriangleMesh.h"
#include "GuEdgeList.h"
#include "GuMeshCleaner.h"
#include "GuConvexEdgeFlags.h"
#include "GuTriangle.h"
#include "GuBV4Build.h"
#include "GuBV32Build.h"
#include "GuBounds.h"
#include "CmSerialize.h"
#include "GuCookingGrbTriangleMesh.h"
#include "GuCookingVolumeIntegration.h"
#include "GuCookingSDF.h"
#include "GuMeshAnalysis.h"

using namespace physx;
using namespace Gu;
using namespace Cm;

///////////////////////////////////////////////////////////////////////////////

PX_IMPLEMENT_OUTPUT_ERROR

///////////////////////////////////////////////////////////////////////////////

TriangleMeshBuilder::TriangleMeshBuilder(TriangleMeshData& m, const PxCookingParams& params) :
	mEdgeList	(NULL),
	mParams		(params),
	mMeshData	(m)
{
}

TriangleMeshBuilder::~TriangleMeshBuilder()
{
	PX_DELETE(mEdgeList);
}

void TriangleMeshBuilder::remapTopology(const PxU32* order)
{
	if(!mMeshData.mNbTriangles)
		return;

	GU_PROFILE_ZONE("remapTopology")

	// Remap one array at a time to limit memory usage

	IndexedTriangle32* newTopo = PX_ALLOCATE(IndexedTriangle32, mMeshData.mNbTriangles, "IndexedTriangle32");
	for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)
		newTopo[i]	= reinterpret_cast<IndexedTriangle32*>(mMeshData.mTriangles)[order[i]];
	PX_FREE(mMeshData.mTriangles);
	mMeshData.mTriangles = newTopo;

	if(mMeshData.mMaterialIndices)
	{
		PxMaterialTableIndex* newMat = PX_ALLOCATE(PxMaterialTableIndex, mMeshData.mNbTriangles, "mMaterialIndices");
		for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)
			newMat[i] = mMeshData.mMaterialIndices[order[i]];
		PX_FREE(mMeshData.mMaterialIndices);
		mMeshData.mMaterialIndices = newMat;
	}

	if(!mParams.suppressTriangleMeshRemapTable || mParams.buildGPUData)
	{
		PxU32* newMap = PX_ALLOCATE(PxU32, mMeshData.mNbTriangles, "mFaceRemap");
		for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)
			newMap[i] = mMeshData.mFaceRemap ? mMeshData.mFaceRemap[order[i]] : order[i];
		PX_FREE(mMeshData.mFaceRemap);
		mMeshData.mFaceRemap = newMap;
	}
}

///////////////////////////////////////////////////////////////////////////////

bool TriangleMeshBuilder::cleanMesh(bool validate, PxTriangleMeshCookingResult::Enum* condition)
{
	PX_ASSERT(mMeshData.mFaceRemap == NULL);

	PxF32 meshWeldTolerance = 0.0f;
	if(mParams.meshPreprocessParams & PxMeshPreprocessingFlag::eWELD_VERTICES)
	{
		if(mParams.meshWeldTolerance == 0.f)
			outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "TriangleMesh: Enable mesh welding with 0 weld tolerance!");
		else
			meshWeldTolerance = mParams.meshWeldTolerance;
	}
	MeshCleaner cleaner(mMeshData.mNbVertices, mMeshData.mVertices, mMeshData.mNbTriangles, reinterpret_cast<const PxU32*>(mMeshData.mTriangles), meshWeldTolerance);
	if(!cleaner.mNbTris)
		return false;

	if(validate)
	{
		// if we do only validate, we check if cleaning did not remove any verts or triangles. 
		// such a mesh can be then directly used for cooking without clean flag
		if((cleaner.mNbVerts != mMeshData.mNbVertices) || (cleaner.mNbTris != mMeshData.mNbTriangles))
			return false;
	}

	// PT: deal with the remap table
	{
		// PT: TODO: optimize this
		if(cleaner.mRemap)
		{
			const PxU32 newNbTris = cleaner.mNbTris;

			// Remap material array
			if(mMeshData.mMaterialIndices)
			{
				PxMaterialTableIndex* tmp = PX_ALLOCATE(PxMaterialTableIndex, newNbTris, "mMaterialIndices");
				for(PxU32 i=0;i<newNbTris;i++)
					tmp[i] = mMeshData.mMaterialIndices[cleaner.mRemap[i]];

				PX_FREE(mMeshData.mMaterialIndices);
				mMeshData.mMaterialIndices = tmp;
			}

			if (!mParams.suppressTriangleMeshRemapTable || mParams.buildGPUData)
			{
				mMeshData.mFaceRemap = PX_ALLOCATE(PxU32, newNbTris, "mFaceRemap");
				PxMemCopy(mMeshData.mFaceRemap, cleaner.mRemap, newNbTris*sizeof(PxU32));
			}
		}
	}

	// PT: deal with geometry
	{
		if(mMeshData.mNbVertices!=cleaner.mNbVerts)
		{
			PX_FREE(mMeshData.mVertices);
			mMeshData.allocateVertices(cleaner.mNbVerts);
		}
		PxMemCopy(mMeshData.mVertices, cleaner.mVerts, mMeshData.mNbVertices*sizeof(PxVec3));
	}

	// PT: deal with topology
	{
		PX_ASSERT(!(mMeshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES));
		if(mMeshData.mNbTriangles!=cleaner.mNbTris)
		{
			PX_FREE(mMeshData.mTriangles);
			mMeshData.allocateTriangles(cleaner.mNbTris, true);
		}

		const float testLength = 500.0f*500.0f*mParams.scale.length*mParams.scale.length;
		bool bigTriangle = false;
		const PxVec3* v = mMeshData.mVertices;
		for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)
		{
			const PxU32 vref0 = cleaner.mIndices[i*3+0];
			const PxU32 vref1 = cleaner.mIndices[i*3+1];
			const PxU32 vref2 = cleaner.mIndices[i*3+2];
			PX_ASSERT(vref0!=vref1 && vref0!=vref2 && vref1!=vref2);

			reinterpret_cast<IndexedTriangle32*>(mMeshData.mTriangles)[i].mRef[0] = vref0;
			reinterpret_cast<IndexedTriangle32*>(mMeshData.mTriangles)[i].mRef[1] = vref1;
			reinterpret_cast<IndexedTriangle32*>(mMeshData.mTriangles)[i].mRef[2] = vref2;

			if(		(v[vref0] - v[vref1]).magnitudeSquared() >= testLength
				||	(v[vref1] - v[vref2]).magnitudeSquared() >= testLength
				||	(v[vref2] - v[vref0]).magnitudeSquared() >= testLength
				)
				bigTriangle = true;
		}
		if(bigTriangle)
		{
			if(condition)
				*condition = PxTriangleMeshCookingResult::eLARGE_TRIANGLE;
			outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "TriangleMesh: triangles are too big, reduce their size to increase simulation stability!");
		}
	}

	return true;
}

static EdgeList* createEdgeList(const TriangleMeshData& meshData)
{
	EDGELISTCREATE create;
	create.NbFaces		= meshData.mNbTriangles;
	if(meshData.has16BitIndices())
	{
		create.DFaces	= NULL;
		create.WFaces	= reinterpret_cast<PxU16*>(meshData.mTriangles);
	}
	else
	{
		create.DFaces	= reinterpret_cast<PxU32*>(meshData.mTriangles);
		create.WFaces	= NULL;
	}
	create.FacesToEdges	= true;
	create.EdgesToFaces	= true;
	create.Verts		= meshData.mVertices;
	//create.Epsilon = 0.1f;
	//	create.Epsilon		= convexEdgeThreshold;
	EdgeList* edgeList = PX_NEW(EdgeList);
	if(!edgeList->init(create))
	{
		PX_DELETE(edgeList);
	}
	return edgeList;
}

void TriangleMeshBuilder::createSharedEdgeData(bool buildAdjacencies, bool buildActiveEdges)
{
	GU_PROFILE_ZONE("createSharedEdgeData")

	const bool savedFlag = buildActiveEdges;

	if(buildAdjacencies) // building edges is required if buildAdjacencies is requested
		buildActiveEdges = true;

	PX_ASSERT(mMeshData.mExtraTrigData == NULL);
	PX_ASSERT(mMeshData.mAdjacencies == NULL);

	if(!buildActiveEdges)
		return;

	const PxU32 nTrigs = mMeshData.mNbTriangles;
	if(0x40000000 <= nTrigs)
	{
		//mesh is too big for this algo, need to be able to express trig indices in 30 bits, and still have an index reserved for "unused":
		outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "TriangleMesh: mesh is too big for this algo!");
		return;
	}

	mMeshData.mExtraTrigData = PX_ALLOCATE(PxU8, nTrigs, "mExtraTrigData");
	PxMemZero(mMeshData.mExtraTrigData, sizeof(PxU8)*nTrigs); 

	const IndexedTriangle32* trigs = reinterpret_cast<const IndexedTriangle32*>(mMeshData.mTriangles);

	mEdgeList = createEdgeList(mMeshData);

	if(mEdgeList)
	{
		PX_ASSERT(mEdgeList->getNbFaces()==mMeshData.mNbTriangles);
		if(mEdgeList->getNbFaces()==mMeshData.mNbTriangles)
		{
			for(PxU32 i=0;i<mEdgeList->getNbFaces();i++)
			{
				const EdgeTriangleData& ET = mEdgeList->getEdgeTriangle(i);
				// Replicate flags
				if(EdgeTriangleAC::HasActiveEdge01(ET))
					mMeshData.mExtraTrigData[i] |= ETD_CONVEX_EDGE_01;
				if(EdgeTriangleAC::HasActiveEdge12(ET))
					mMeshData.mExtraTrigData[i] |= ETD_CONVEX_EDGE_12;
				if(EdgeTriangleAC::HasActiveEdge20(ET))
					mMeshData.mExtraTrigData[i] |= ETD_CONVEX_EDGE_20;
			}
		}
	}

	// fill the adjacencies
	if(buildAdjacencies)
	{
		mMeshData.mAdjacencies = PX_ALLOCATE(PxU32, nTrigs*3, "mAdjacencies");
		memset(mMeshData.mAdjacencies, 0xFFFFffff, sizeof(PxU32)*nTrigs*3);		

		PxU32 NbEdges = mEdgeList->getNbEdges();
		const EdgeDescData* ED = mEdgeList->getEdgeToTriangles();
		const EdgeData* Edges = mEdgeList->getEdges();
		const PxU32* FBE = mEdgeList->getFacesByEdges();

		while(NbEdges--)
		{
			// Get number of triangles sharing current edge
			const PxU32 Count = ED->Count;
			
			if(Count > 1)
			{
				const PxU32 FaceIndex0 = FBE[ED->Offset+0];
				const PxU32 FaceIndex1 = FBE[ED->Offset+1];

				const EdgeData& edgeData = *Edges;
				const IndexedTriangle32& T0 = trigs[FaceIndex0];
				const IndexedTriangle32& T1 = trigs[FaceIndex1];

				const PxU32 offset0 = T0.findEdgeCCW(edgeData.Ref0,edgeData.Ref1);				
				const PxU32 offset1 = T1.findEdgeCCW(edgeData.Ref0,edgeData.Ref1);

				mMeshData.setTriangleAdjacency(FaceIndex0, FaceIndex1, offset0);
				mMeshData.setTriangleAdjacency(FaceIndex1, FaceIndex0, offset1);
			}
			ED++;
			Edges++;
		}
	}

#if PX_DEBUG
	for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)
	{
		const IndexedTriangle32& T = trigs[i];
		PX_UNUSED(T);
		const EdgeTriangleData& ET = mEdgeList->getEdgeTriangle(i);
		PX_ASSERT((EdgeTriangleAC::HasActiveEdge01(ET) && (mMeshData.mExtraTrigData[i] & ETD_CONVEX_EDGE_01)) || (!EdgeTriangleAC::HasActiveEdge01(ET) && !(mMeshData.mExtraTrigData[i] & ETD_CONVEX_EDGE_01)));
		PX_ASSERT((EdgeTriangleAC::HasActiveEdge12(ET) && (mMeshData.mExtraTrigData[i] & ETD_CONVEX_EDGE_12)) || (!EdgeTriangleAC::HasActiveEdge12(ET) && !(mMeshData.mExtraTrigData[i] & ETD_CONVEX_EDGE_12)));
		PX_ASSERT((EdgeTriangleAC::HasActiveEdge20(ET) && (mMeshData.mExtraTrigData[i] & ETD_CONVEX_EDGE_20)) || (!EdgeTriangleAC::HasActiveEdge20(ET) && !(mMeshData.mExtraTrigData[i] & ETD_CONVEX_EDGE_20)));
	}
#endif

	// PT: respect the PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE flag. This is important for
	// deformable meshes - even if the edge data was needed on-the-fly to compute adjacencies.
	if(!savedFlag)
		PX_FREE(mMeshData.mExtraTrigData);
}

void TriangleMeshBuilder::createVertMapping()
{
	GU_PROFILE_ZONE("createVertMapping")

	const PxU32 nbVerts = mMeshData.mNbVertices;

	mMeshData.mAccumulatedTrianglesRef = PX_ALLOCATE(PxU32, nbVerts, "accumulatedTrianglesRef");

	PxU32* tempCounts = PX_ALLOCATE(PxU32, nbVerts, "tempCounts");

	PxU32* triangleCounts = mMeshData.mAccumulatedTrianglesRef;

	PxMemZero(triangleCounts, sizeof(PxU32) * nbVerts);
	PxMemZero(tempCounts, sizeof(PxU32) * nbVerts);

	const PxU32 nbTriangles = mMeshData.mNbTriangles;

	IndexedTriangle32* triangles = reinterpret_cast<IndexedTriangle32*>(mMeshData.mGRB_primIndices);

	for (PxU32 i = 0; i < nbTriangles; i++)
	{
		IndexedTriangle32& triangle = triangles[i];

		triangleCounts[triangle.mRef[0]]++;
		triangleCounts[triangle.mRef[1]]++;
		triangleCounts[triangle.mRef[2]]++;
	}

	//compute runsum
	PxU32 totalReference = 0;
	for (PxU32 i = 0; i < nbVerts; ++i)
	{
		PxU32 originalReference = triangleCounts[i];
		triangleCounts[i] = totalReference;
		totalReference += originalReference;
	}

	PX_ASSERT(totalReference == nbTriangles * 3);

	mMeshData.mTrianglesReferences = PX_ALLOCATE(PxU32, totalReference, "mTrianglesReferences");
	mMeshData.mNbTrianglesReferences = totalReference;

	PxU32* triangleRefs = mMeshData.mTrianglesReferences;

	for (PxU32 i = 0; i < nbTriangles; i++)
	{
		IndexedTriangle32& triangle = triangles[i];

		const PxU32 ind0 = triangle.mRef[0];
		const PxU32 ind1 = triangle.mRef[1];
		const PxU32 ind2 = triangle.mRef[2];

		triangleRefs[triangleCounts[ind0] + tempCounts[ind0]] = i;
		tempCounts[ind0]++;

		triangleRefs[triangleCounts[ind1] + tempCounts[ind1]] = i;
		tempCounts[ind1]++;

		triangleRefs[triangleCounts[ind2] + tempCounts[ind2]] = i;
		tempCounts[ind2]++;
	}

	PX_FREE(tempCounts);
}

void TriangleMeshBuilder::recordTriangleIndices()
{
	if (mParams.buildGPUData)
	{
		PX_ASSERT(!(mMeshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES));
		PX_ASSERT(mMeshData.mGRB_primIndices);

		//copy the BV4 triangle indices to GPU triangle indices buffer
		PxMemCopy(mMeshData.mGRB_primIndices, mMeshData.mTriangles, sizeof(IndTri32) *mMeshData.mNbTriangles);
	}
}

void TriangleMeshBuilder::createGRBData()
{
	GU_PROFILE_ZONE("buildAdjacencies")

	const PxU32 numTris = mMeshData.mNbTriangles;

	PX_ASSERT(!(mMeshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES));

	// Core: Mesh data
	///////////////////////////////////////////////////////////////////////////////////

	// (by using adjacency info generated by physx cooker)
	PxVec3* tempNormalsPerTri_prealloc = PX_ALLOCATE(PxVec3, numTris, "tempNormalsPerTri_prealloc");

	mMeshData.mGRB_primAdjacencies = PX_ALLOCATE(uint4, numTris, "GRB_triAdjacencies");

	buildAdjacencies(
		reinterpret_cast<uint4*>(mMeshData.mGRB_primAdjacencies),
		tempNormalsPerTri_prealloc,
		mMeshData.mVertices,
		reinterpret_cast<IndexedTriangle32*>(mMeshData.mGRB_primIndices),
		numTris
		);

	PX_FREE(tempNormalsPerTri_prealloc);
}

void TriangleMeshBuilder::createGRBMidPhaseAndData(const PxU32 originalTriangleCount)
{
	PX_UNUSED(originalTriangleCount);
	if (mParams.buildGPUData)
	{
		PX_ASSERT(!(mMeshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES));

		BV32Tree* bv32Tree = PX_NEW(BV32Tree);
		mMeshData.mGRB_BV32Tree = bv32Tree;

		BV32TriangleMeshBuilder::createMidPhaseStructure(mParams, mMeshData, *bv32Tree);
		
		createGRBData();

		if (mParams.meshPreprocessParams & PxMeshPreprocessingFlag::eENABLE_VERT_MAPPING || mParams.buildGPUData)
		{
			createVertMapping();
		}

#if BV32_VALIDATE
		IndTri32* grbTriIndices = reinterpret_cast<IndTri32*>(mMeshData.mGRB_primIndices);
		IndTri32* cpuTriIndices = reinterpret_cast<IndTri32*>(mMeshData.mTriangles);
		//map CPU remap triangle index to GPU remap triangle index
		for (PxU32 i = 0; i < mMeshData.mNbTriangles; ++i)
		{
			PX_ASSERT(grbTriIndices[i].mRef[0] == cpuTriIndices[mMeshData.mGRB_faceRemap[i]].mRef[0]);
			PX_ASSERT(grbTriIndices[i].mRef[1] == cpuTriIndices[mMeshData.mGRB_faceRemap[i]].mRef[1]);
			PX_ASSERT(grbTriIndices[i].mRef[2] == cpuTriIndices[mMeshData.mGRB_faceRemap[i]].mRef[2]);
		}
#endif
	}
}


bool TriangleMeshBuilder::loadFromDescInternal(PxTriangleMeshDesc& desc, PxTriangleMeshCookingResult::Enum* condition, bool validateMesh)
{
#ifdef PROFILE_MESH_COOKING
	printf("\n");
#endif

	const PxU32 originalTriangleCount = desc.triangles.count;
	if (!desc.isValid())
		return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "TriangleMesh::loadFromDesc: desc.isValid() failed!");

	// verify the mesh params
	if (!mParams.midphaseDesc.isValid())
		return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "TriangleMesh::loadFromDesc: mParams.midphaseDesc.isValid() failed!");

	// Save simple params
	{
		// Handle implicit topology
		PxU32* topology = NULL;
		if (!desc.triangles.data)
		{
			// We'll create 32-bit indices
			desc.flags &= ~PxMeshFlag::e16_BIT_INDICES;
			desc.triangles.stride = sizeof(PxU32) * 3;

			{
				// Non-indexed mesh => create implicit topology
				desc.triangles.count = desc.points.count / 3;
				// Create default implicit topology
				topology = PX_ALLOCATE(PxU32, desc.points.count, "topology");
				for (PxU32 i = 0; i<desc.points.count; i++)
					topology[i] = i;
				desc.triangles.data = topology;
			}
		}

		// Convert and clean the input mesh
		if (!importMesh(desc, mParams, condition, validateMesh))
		{
			PX_FREE(topology);
			return false;
		}

		// Cleanup if needed
		PX_FREE(topology);
	}

	createMidPhaseStructure();

	//copy the BV4 triangle indices to grb triangle indices if buildGRBData is true
	recordTriangleIndices();

	// Compute local bounds
	computeLocalBoundsAndGeomEpsilon(mMeshData.mVertices, mMeshData.mNbVertices, mMeshData.mAABB, mMeshData.mGeomEpsilon);	

	createSharedEdgeData(mParams.buildTriangleAdjacencies, !(mParams.meshPreprocessParams & PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE));

	createGRBMidPhaseAndData(originalTriangleCount);

	return true;
}

void TriangleMeshBuilder::buildInertiaTensor()
{
	PxTriangleMeshDesc simpleMesh;

	simpleMesh.points.count = mMeshData.mNbVertices;
	simpleMesh.points.stride = sizeof(PxVec3);
	simpleMesh.points.data = mMeshData.mVertices;
	simpleMesh.triangles.count = mMeshData.mNbTriangles;
	simpleMesh.triangles.stride = sizeof(PxU32) * 3;
	simpleMesh.triangles.data = mMeshData.mTriangles;
	simpleMesh.flags &= (~PxMeshFlag::e16_BIT_INDICES);

	PxIntegrals integrals;
	computeVolumeIntegrals(simpleMesh, 1, integrals);

	integrals.getOriginInertia(mMeshData.mInertia);
	mMeshData.mMass = PxReal(integrals.mass);
	mMeshData.mLocalCenterOfMass = integrals.COM;
}

PX_FORCE_INLINE PxReal lerp(PxReal a, PxReal b, PxReal t)
{
	return a + t * (b - a);
}

PX_FORCE_INLINE PxReal bilerp(
	PxReal f00,
	PxReal f10,
	PxReal f01,
	PxReal f11,
	PxReal tx, PxReal ty)
{
	PxReal a = lerp(f00, f10, tx);
	PxReal b = lerp(f01, f11, tx);
	return lerp(
		a,
		b,
		ty);
}

PX_FORCE_INLINE PxReal trilerp(
	PxReal f000,
	PxReal f100,
	PxReal f010,
	PxReal f110,
	PxReal f001,
	PxReal f101,
	PxReal f011,
	PxReal f111,
	PxReal tx,
	PxReal ty,
	PxReal tz)
{
	PxReal a = bilerp(f000, f100, f010, f110, tx, ty);
	PxReal b = bilerp(f001, f101, f011, f111, tx, ty);
	return lerp(
		a,
		b,
		tz);
}

template<typename T>
class DenseSDF
{
public:
	PxU32 width, height, depth;
private:
	T* sdf;

	PX_FORCE_INLINE PxU32 computeIndex(PxU32 x, PxU32 y, PxU32 z)
	{
		return z * (width + 1) * (height + 1) + y * (width + 1) + x;
	}

public:
	DenseSDF(PxU32 width, PxU32 height, PxU32 depth, T* sdf)
	{
		initialize(width, height, depth, sdf);
	}

	DenseSDF() {}

	void initialize(PxU32 width_, PxU32 height_, PxU32 depth_, T* sdf_)
	{
		this->width = width_;
		this->height = height_;
		this->depth = depth_;
		this->sdf = sdf_;
	}

	PxReal sampleSDFDirect(const PxVec3& samplePoint)
	{
		const PxU32 xBase = PxClamp(PxU32(samplePoint.x), 0u, width - 1);
		const PxU32 yBase = PxClamp(PxU32(samplePoint.y), 0u, height - 1);
		const PxU32 zBase = PxClamp(PxU32(samplePoint.z), 0u, depth - 1);

		return trilerp(
			sdf[computeIndex(xBase, yBase, zBase)],
			sdf[computeIndex(xBase + 1, yBase, zBase)],
			sdf[computeIndex(xBase, yBase + 1, zBase)],
			sdf[computeIndex(xBase + 1, yBase + 1, zBase)],
			sdf[computeIndex(xBase, yBase, zBase + 1)],
			sdf[computeIndex(xBase + 1, yBase, zBase + 1)],
			sdf[computeIndex(xBase, yBase + 1, zBase + 1)],
			sdf[computeIndex(xBase + 1, yBase + 1, zBase + 1)], samplePoint.x - xBase, samplePoint.y - yBase, samplePoint.z - zBase);
	}

};

PX_FORCE_INLINE void decodeTriple(PxU32 id, PxU32& x, PxU32& y, PxU32& z)
{
	x = id & 0x000003FF;
	id = id >> 10;
	y = id & 0x000003FF;
	id = id >> 10;
	z = id & 0x000003FF;
}

PX_FORCE_INLINE PxU32 idx(PxU32 x, PxU32 y, PxU32 z, PxU32 width, PxU32 height)
{
	return z * (width) * (height)+y * width + x;
}

PX_FORCE_INLINE PxReal decode(PxU8* data, PxU32 bytesPerSparsePixel, PxReal subgridsMinSdfValue, PxReal subgridsMaxSdfValue)
{
	switch (bytesPerSparsePixel)
	{
	case 1:
		return PxReal(data[0]) * (1.0f / 255.0f) * (subgridsMaxSdfValue - subgridsMinSdfValue) + subgridsMinSdfValue;
	case 2:
	{
		PxU16* ptr = reinterpret_cast<PxU16*>(data);
		return PxReal(ptr[0]) * (1.0f / 65535.0f) * (subgridsMaxSdfValue - subgridsMinSdfValue) + subgridsMinSdfValue;
	}
	case 4:
		//If 4 bytes per subgrid pixel are available, then normal floats are used. No need to 
		//de-normalize integer values since the floats already contain real distance values
		PxReal* ptr = reinterpret_cast<PxReal*>(data);
		return ptr[0];
	}
	return 0;
}

PX_FORCE_INLINE PxReal decode(const Gu::SDF& sdf, PxI32 xx, PxI32 yy, PxI32 zz)
{
	if (xx < 0 || yy < 0 || zz < 0 || xx > PxI32(sdf.mDims.x) || yy > PxI32(sdf.mDims.y) || zz > PxI32(sdf.mDims.z))
		return 1.0f; //Return a value >0 that counts as outside

	const PxU32 nbX = sdf.mDims.x / sdf.mSubgridSize;
	const PxU32 nbY = sdf.mDims.y / sdf.mSubgridSize;
	const PxU32 nbZ = sdf.mDims.z / sdf.mSubgridSize;
	
	PxU32 xBase = xx / sdf.mSubgridSize;
	PxU32 yBase = yy / sdf.mSubgridSize;
	PxU32 zBase = zz / sdf.mSubgridSize;

	PxU32 x = xx % sdf.mSubgridSize;
	PxU32 y = yy % sdf.mSubgridSize;
	PxU32 z = zz % sdf.mSubgridSize;

	if (xBase == nbX)
	{
		--xBase;
		x = sdf.mSubgridSize;
	}
	if (yBase == nbY)
	{
		--yBase;
		y = sdf.mSubgridSize;
	}
	if (zBase == nbZ)
	{
		--zBase;
		z = sdf.mSubgridSize;
	}

	PxU32 startId = sdf.mSubgridStartSlots[zBase * (nbX) * (nbY)+yBase * (nbX)+xBase];
	if (startId != 0xFFFFFFFFu)
	{
		decodeTriple(startId, xBase, yBase, zBase);
		xBase *= (sdf.mSubgridSize + 1);
		yBase *= (sdf.mSubgridSize + 1);
		zBase *= (sdf.mSubgridSize + 1);

		const PxU32 w = sdf.mSdfSubgrids3DTexBlockDim.x * (sdf.mSubgridSize + 1);
		const PxU32 h = sdf.mSdfSubgrids3DTexBlockDim.y * (sdf.mSubgridSize + 1);
		const PxU32 index = idx(xBase + x, yBase + y, zBase + z, w, h);
		return decode(&sdf.mSubgridSdf[sdf.mBytesPerSparsePixel * index],
			sdf.mBytesPerSparsePixel, sdf.mSubgridsMinSdfValue, sdf.mSubgridsMaxSdfValue);
	}
	else
	{
		DenseSDF<PxReal> coarseEval(nbX, nbY, nbZ, sdf.mSdf);

		PxReal s = 1.0f / sdf.mSubgridSize;
		return coarseEval.sampleSDFDirect(PxVec3(xBase + x * s, yBase + y * s, zBase + z * s));
	}
}

PX_FORCE_INLINE PxU64 key(PxI32 xId, PxI32 yId, PxI32 zId)
{
	const PxI32 offset = 1 << 19;
	return (PxU64(zId+offset) << 42) | (PxU64(yId + offset) << 21) | (PxU64(xId + offset) << 0);
}

const PxI32 offsets[3][3][3] = { { {0,-1,0}, {0,-1,-1}, {0,0,-1} },
							     { {0,0,-1}, {-1,0,-1}, {-1,0,0} } , 
							     { {-1,0,0}, {-1,-1,0}, {0,-1,0} } };

const PxI32 projections[3][2] = { {1, 2}, {2, 0}, {0, 1} };

PX_FORCE_INLINE PxReal dirSign(PxI32 principalDirection, const PxVec3& start, const PxVec3& middle, const PxVec3& end)
{
	PxReal a0 = middle[projections[principalDirection][0]] - start[projections[principalDirection][0]];
	PxReal a1 = middle[projections[principalDirection][1]] - start[projections[principalDirection][1]];

	PxReal b0 = end[projections[principalDirection][0]] - middle[projections[principalDirection][0]];
	PxReal b1 = end[projections[principalDirection][1]] - middle[projections[principalDirection][1]];

	return a0 * b1 - a1 * b0;
}

PX_FORCE_INLINE PxI32 indexOfMostConcaveCorner(PxI32 principalDirection, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d)
{
	PxReal minimum = 0;
	PxI32 result = -1;
	PxReal s = dirSign(principalDirection, a, b, c);
	if (s <= minimum)
	{
		minimum = s;
		result = 1;
	}
	s = dirSign(principalDirection, b, c, d);
	if (s <= minimum)
	{
		minimum = s;
		result = 2;
	}
	s = dirSign(principalDirection, c, d, a);
	if (s <= minimum)
	{
		minimum = s;
		result = 3;
	}
	s = dirSign(principalDirection, d, a, b);
	if (s <= minimum)
	{
		minimum = s;
		result = 0;
	}
	return result;
}

bool generatePointInCell(const Gu::SDF& sdf, PxI32 x, PxI32 y, PxI32 z, PxVec3& point, PxReal corners[2][2][2])
{
	const PxReal threshold = 0.0f;

	PxU32 positiveCounter = 0;
	PxU32 negativeCounter = 0;
	for (PxI32 xx = 0; xx <= 1; ++xx) for (PxI32 yy = 0; yy <= 1; ++yy) for (PxI32 zz = 0; zz <= 1; ++zz)
	{
		PxReal v = corners[xx][yy][zz];
		if (v > 0)
			++positiveCounter;
		if (v < 0)
			++negativeCounter;
	}
	PxBounds3 box;
	box.minimum = sdf.mMeshLower + PxVec3(x * sdf.mSpacing, y * sdf.mSpacing, z * sdf.mSpacing);
	box.maximum = box.minimum + PxVec3(sdf.mSpacing);
	if (positiveCounter == 8 || negativeCounter == 8)
	{
		//Nothing to do because surface does not cross the current cell
	}
	else
	{
		//If point is not completely inside or outside, then find a point inside the cube that divides it into 8 cuboids
		PxU32 counter = 0;
		PxVec3 sum(0.0f);
		for (PxI32 a = 0; a <= 1; ++a) for (PxI32 b = 0; b <= 1; ++b)
		{
			PxReal p = corners[a][b][0];
			PxReal q = corners[a][b][1];

			if ((p <= threshold && q >= threshold) || (q <= threshold && p >= threshold))
			{
				PxReal t = (q != p) ? PxClamp((threshold - p) / (q - p), 0.0f, 1.0f) : 0.5f;
				sum += PxVec3(PxReal(a), PxReal(b), t);
				++counter;
			}
		}
		for (PxI32 a = 0; a <= 1; ++a) for (PxI32 b = 0; b <= 1; ++b)
		{
			PxReal p = corners[b][0][a];
			PxReal q = corners[b][1][a];

			if ((p <= threshold && q >= threshold) || (q <= threshold && p >= threshold))
			{
				PxReal t = (q != p) ? PxClamp((threshold - p) / (q - p), 0.0f, 1.0f) : 0.5f;
				sum += PxVec3(PxReal(b), t, PxReal(a));
				++counter;
			}
		}
		for (PxI32 a = 0; a <= 1; ++a) for (PxI32 b = 0; b <= 1; ++b)
		{
			PxReal p = corners[0][a][b];
			PxReal q = corners[1][a][b];

			if ((p <= threshold && q >= threshold) || (q <= threshold && p >= threshold))
			{
				PxReal t = (q != p) ? PxClamp((threshold - p) / (q - p), 0.0f, 1.0f) : 0.5f;
				sum += PxVec3(t, PxReal(a), PxReal(b));
				++counter;
			}
		}
		if (counter > 0)
		{
			point = box.minimum + sum * (sdf.mSpacing / counter);
			return true;
		}
	}

	return false;
}

PX_FORCE_INLINE bool generatePointInCell(const Gu::SDF& sdf, PxI32 x, PxI32 y, PxI32 z, PxVec3& point)
{
	PxReal corners[2][2][2];
	for (PxI32 xx = 0; xx <= 1; ++xx) for (PxI32 yy = 0; yy <= 1; ++yy) for (PxI32 zz = 0; zz <= 1; ++zz)
	{
		PxReal v = decode(sdf, x + xx, y + yy, z + zz);
		corners[xx][yy][zz] = v;
	}
	return generatePointInCell(sdf, x, y, z, point, corners);
}

PX_FORCE_INLINE bool generatePointInCellUsingCache(const Gu::SDF& sdf, PxI32 xBase, PxI32 yBase, PxI32 zBase, PxI32 x, PxI32 y, PxI32 z, PxVec3& point, const PxArray<PxReal>& cache)
{
	PxReal corners[2][2][2];
	for (PxI32 xx = 0; xx <= 1; ++xx) for (PxI32 yy = 0; yy <= 1; ++yy) for (PxI32 zz = 0; zz <= 1; ++zz)
	{
		PxReal v = cache[idx(x + xx, y + yy, z + zz, sdf.mSubgridSize + 1, sdf.mSubgridSize + 1)];
		corners[xx][yy][zz] = v;
	}
	return generatePointInCell(sdf, xBase * sdf.mSubgridSize + x, yBase * sdf.mSubgridSize + y, zBase * sdf.mSubgridSize + z, point, corners);
}

PX_FORCE_INLINE PxReal getDenseSDFValue(const Gu::SDF& sdf, PxI32 x, PxI32 y, PxI32 z)
{
	if (x < 0 || y < 0 || z < 0 || x >= PxI32(sdf.mDims.x) || y >= PxI32(sdf.mDims.y) || z>= PxI32(sdf.mDims.z))
		return 1.0; //Return a value >0 that counts as outside

	return sdf.mSdf[idx(x, y, z, sdf.mDims.x, sdf.mDims.y)];
}

PX_FORCE_INLINE bool generatePointInCellDense(const Gu::SDF& sdf, PxI32 x, PxI32 y, PxI32 z, PxVec3& point)
{
	PxReal corners[2][2][2];
	for (PxI32 xx = 0; xx <= 1; ++xx) for (PxI32 yy = 0; yy <= 1; ++yy) for (PxI32 zz = 0; zz <= 1; ++zz)
	{
		PxReal v = getDenseSDFValue(sdf, x + xx, y + yy, z + zz);
		corners[xx][yy][zz] = v;
	}
	return generatePointInCell(sdf, x, y, z, point, corners);
}

PX_FORCE_INLINE bool canSkipSubgrid(const Gu::SDF& sdf, PxI32 i, PxI32 j, PxI32 k)
{
	const PxReal t = 0.1f * sdf.mSpacing;
	const PxI32 nbX = sdf.mDims.x / sdf.mSubgridSize;
	const PxI32 nbY = sdf.mDims.y / sdf.mSubgridSize;
	const PxI32 nbZ = sdf.mDims.z / sdf.mSubgridSize;

	if (i < 0 || j < 0 || k < 0 || i >= nbX || j >= nbY || k >= nbZ)
		return false;

	if (sdf.mSubgridStartSlots[k * (nbX) * (nbY)+j * (nbX)+i] == 0xFFFFFFFFu)
	{
		PxU32 positiveCounter = 0;
		PxU32 negativeCounter = 0;
		for (PxI32 xx = 0; xx <= 1; ++xx) for (PxI32 yy = 0; yy <= 1; ++yy) for (PxI32 zz = 0; zz <= 1; ++zz)
		{
			PxReal v = decode(sdf, (i + xx)* sdf.mSubgridSize, (j + yy) * sdf.mSubgridSize, (k + zz) * sdf.mSubgridSize);
			if (v > t)
				++positiveCounter;
			if (v < t)
				++negativeCounter;
		}
		if (positiveCounter == 8 || negativeCounter == 8)
			return true;
	}
	return false;
}

void createTriangles(PxI32 xId, PxI32 yId, PxI32 zId, PxReal d0, PxReal ds[3], const PxHashMap<PxU64, PxU32>& cellToPoint, const PxArray<PxVec3>& points, PxArray<PxI32>& triangleIndices)
{	
	bool flipTriangleOrientation = false;
	const PxReal threshold = 0.0f;

	PxI32 num = 0;
	for (PxI32 dim = 0; dim < 3; dim++)
	{
		PxReal d = ds[dim];
		if ((d0 <= threshold && d >= threshold) || (d <= threshold && d0 >= threshold))
			num++;
	}
	if (num == 0)
		return;


	PxI32 buffer[4];
	const PxPair<const PxU64, PxU32>* f = cellToPoint.find(key(xId, yId, zId));
	if (!f)
		return;

	buffer[0] = f->second;
	PxVec3 v0 = points[buffer[0]];

	for (PxI32 dim = 0; dim < 3; dim++)
	{
		PxReal d = ds[dim];
		bool b1 = d0 <= threshold && d >= threshold;
		bool b2 = d <= threshold && d0 >= threshold;
		if (b1 || b2)
		{
			bool flip = flipTriangleOrientation == b1;
			bool skip = false;

			for (PxI32 ii = 0; ii < 3; ++ii)
			{
				f = cellToPoint.find(key(xId + offsets[dim][ii][0], yId + offsets[dim][ii][1], zId + offsets[dim][ii][2]));
				if (f)
					buffer[ii + 1] = f->second;
				else				
					skip = true;				
			}
			if (skip)
				continue;

			PxI32 shift = PxMax(0, indexOfMostConcaveCorner(dim, v0, points[buffer[1]], points[buffer[2]], points[buffer[3]])) % 2;

			//Split the quad into two triangles
			for (PxI32 ii = 0; ii < 2; ++ii)
			{
				triangleIndices.pushBack(buffer[shift]);
				if (flip)
				{
					for (PxI32 jj = 2; jj >= 1; --jj)
						triangleIndices.pushBack(buffer[(ii + jj + shift) % 4]);
				}
				else
				{
					for (PxI32 jj = 1; jj < 3; ++jj)
						triangleIndices.pushBack(buffer[(ii + jj + shift) % 4]);
				}
			}
		}
	}
}

void TriangleMeshBuilder::buildInertiaTensorFromSDF()
{
	if (MeshAnalyzer::checkMeshWatertightness(reinterpret_cast<const Gu::Triangle*>(mMeshData.mTriangles), mMeshData.mNbTriangles))
	{
		buildInertiaTensor();
		return;
	}
	const Gu::SDF& sdf = mMeshData.mSdfData;

	const PxI32 nbX = sdf.mDims.x / PxMax(1u, sdf.mSubgridSize);
	const PxI32 nbY = sdf.mDims.y / PxMax(1u, sdf.mSubgridSize);
	const PxI32 nbZ = sdf.mDims.z / PxMax(1u, sdf.mSubgridSize);

	PxArray<PxVec3> points;
	PxHashMap<PxU64, PxU32> cellToPoint;	

	if (sdf.mSubgridSize == 0)
	{
		//Dense SDF
		for (PxI32 k = -1; k <= nbZ; ++k)		
			for (PxI32 j = -1; j <= nbY; ++j)
				for (PxI32 i = -1; i <= nbX; ++i)
				{
					PxVec3 p;
					if (generatePointInCellDense(sdf, i, j, k, p))
					{
						cellToPoint.insert(key(i, j, k), points.size());
						points.pushBack(p);
					}
				}
	}
	else
	{
		PxArray<PxReal> sdfCache;
		sdfCache.resize((sdf.mSubgridSize + 1) * (sdf.mSubgridSize + 1) * (sdf.mSubgridSize + 1));

		for (PxI32 k = -1; k <= nbZ; ++k)
		{
			for (PxI32 j = -1; j <= nbY; ++j)
			{
				for (PxI32 i = -1; i <= nbX; ++i)
				{
					if (canSkipSubgrid(sdf, i, j, k))
						continue;

					for (PxU32 z = 0; z <= sdf.mSubgridSize; ++z)				
						for (PxU32 y = 0; y <= sdf.mSubgridSize; ++y)					
							for (PxU32 x = 0; x <= sdf.mSubgridSize; ++x)
								sdfCache[idx(x, y, z, sdf.mSubgridSize + 1, sdf.mSubgridSize + 1)] = decode(sdf, i * sdf.mSubgridSize + x, j * sdf.mSubgridSize + y, k * sdf.mSubgridSize + z);

					//Process the subgrid
					for (PxU32 z = 0; z < sdf.mSubgridSize; ++z)
					{
						for (PxU32 y = 0; y < sdf.mSubgridSize; ++y)
						{
							for (PxU32 x = 0; x < sdf.mSubgridSize; ++x)
							{
								PxVec3 p;
								PxU32 xId = i * sdf.mSubgridSize + x;
								PxU32 yId = j * sdf.mSubgridSize + y;
								PxU32 zId = k * sdf.mSubgridSize + z;
								if (generatePointInCellUsingCache(sdf, i, j, k, x, y, z, p, sdfCache))
								{								
									cellToPoint.insert(key(xId, yId, zId), points.size());
									points.pushBack(p);
								}
							}
						}
					}
				}
			}
		}
	}

	PxArray<PxI32> triangleIndices;
	if (sdf.mSubgridSize == 0)
	{
		for (PxI32 k = -1; k <= nbZ; ++k)
			for (PxI32 j = -1; j <= nbY; ++j)
				for (PxI32 i = -1; i <= nbX; ++i)
				{
					PxReal d0 = getDenseSDFValue(sdf, i, j, k);
					PxReal ds[3];
					ds[0] = getDenseSDFValue(sdf, i + 1, j, k);
					ds[1] = getDenseSDFValue(sdf, i, j + 1, k);
					ds[2] = getDenseSDFValue(sdf, i, j, k + 1);

					createTriangles(i, j, k, d0, ds, cellToPoint, points, triangleIndices);
				}
	}
	else
	{
		for (PxI32 k = -1; k <= nbZ; ++k)
		{
			for (PxI32 j = -1; j <= nbY; ++j)
			{
				for (PxI32 i = -1; i <= nbX; ++i)
				{
					if (canSkipSubgrid(sdf, i, j, k))
						continue;

					//Process the subgrid
					for (PxU32 z = 0; z < sdf.mSubgridSize; ++z)
					{
						for (PxU32 y = 0; y < sdf.mSubgridSize; ++y)
						{
							for (PxU32 x = 0; x < sdf.mSubgridSize; ++x)
							{
								PxReal d0 = decode(sdf, i * sdf.mSubgridSize + x, j * sdf.mSubgridSize + y, k * sdf.mSubgridSize + z);
								PxReal ds[3];
								ds[0] = decode(sdf, i * sdf.mSubgridSize + x + 1, j * sdf.mSubgridSize + y, k * sdf.mSubgridSize + z);
								ds[1] = decode(sdf, i * sdf.mSubgridSize + x, j * sdf.mSubgridSize + y + 1, k * sdf.mSubgridSize + z);
								ds[2] = decode(sdf, i * sdf.mSubgridSize + x, j * sdf.mSubgridSize + y, k * sdf.mSubgridSize + z + 1);

								createTriangles(x + i * sdf.mSubgridSize, y + j * sdf.mSubgridSize, z + k * sdf.mSubgridSize, d0, ds, cellToPoint, points, triangleIndices);
							}
						}
					}				
				}
			}
		}
	}

	PxTriangleMeshDesc simpleMesh;

	simpleMesh.points.count = points.size(); 
	simpleMesh.points.stride = sizeof(PxVec3);
	simpleMesh.points.data = points.begin(); 
	simpleMesh.triangles.count = triangleIndices.size() / 3; 
	simpleMesh.triangles.stride = sizeof(PxU32) * 3;
	simpleMesh.triangles.data = triangleIndices.begin(); 
	simpleMesh.flags &= (~PxMeshFlag::e16_BIT_INDICES);

	PxIntegrals integrals;
	computeVolumeIntegrals(simpleMesh, 1, integrals);

	integrals.getOriginInertia(mMeshData.mInertia);
	mMeshData.mMass = PxReal(integrals.mass);
	mMeshData.mLocalCenterOfMass = integrals.COM;
}

//
// When suppressTriangleMeshRemapTable is true, the face remap table is not created.  This saves a significant amount of memory,
// but the SDK will not be able to provide information about which mesh triangle is hit in collisions, sweeps or raycasts hits.
//
// The sequence is as follows:

bool TriangleMeshBuilder::loadFromDesc(const PxTriangleMeshDesc& _desc, PxTriangleMeshCookingResult::Enum* condition, bool validateMesh)
{
	PxTriangleMeshDesc desc = _desc;
	return loadFromDescInternal(desc, condition, validateMesh);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool TriangleMeshBuilder::save(PxOutputStream& stream, bool platformMismatch, const PxCookingParams& params) const
{
	// Export header
	if(!writeHeader('M', 'E', 'S', 'H', PX_MESH_VERSION, platformMismatch, stream))
		return false;

	// Export midphase ID
	writeDword(getMidphaseID(), platformMismatch, stream);

	// Export serialization flags
	PxU32 serialFlags = 0;
	if(mMeshData.mMaterialIndices)	serialFlags |= IMSF_MATERIALS;
	if(mMeshData.mFaceRemap)		serialFlags |= IMSF_FACE_REMAP;
	if(mMeshData.mAdjacencies)		serialFlags |= IMSF_ADJACENCIES;
	if (params.buildGPUData)		serialFlags |= IMSF_GRB_DATA;
	if (mMeshData.mGRB_faceRemapInverse) serialFlags |= IMSF_GRB_INV_REMAP;
	// Compute serialization flags for indices
	PxU32 maxIndex=0;
	const IndexedTriangle32* tris = reinterpret_cast<const IndexedTriangle32*>(mMeshData.mTriangles);
	for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)
	{
		if(tris[i].mRef[0]>maxIndex)	maxIndex = tris[i].mRef[0];
		if(tris[i].mRef[1]>maxIndex)	maxIndex = tris[i].mRef[1];
		if(tris[i].mRef[2]>maxIndex)	maxIndex = tris[i].mRef[2];
	}

	bool enableSdf = mMeshData.mSdfData.mSdf ? true : false;

	if(enableSdf) serialFlags |= IMSF_SDF;

	bool enableInertia = (params.meshPreprocessParams & PxMeshPreprocessingFlag::eENABLE_INERTIA) || enableSdf;
	if(enableInertia)
		serialFlags |= IMSF_INERTIA;

	bool force32 = (params.meshPreprocessParams & PxMeshPreprocessingFlag::eFORCE_32BIT_INDICES);
	if (maxIndex <= 0xFFFF && !force32)
		serialFlags |= (maxIndex <= 0xFF ? IMSF_8BIT_INDICES : IMSF_16BIT_INDICES);

	bool enableVertexMapping = (params.buildGPUData || (params.meshPreprocessParams & PxMeshPreprocessingFlag::eENABLE_VERT_MAPPING));
	if (enableVertexMapping)
		serialFlags |= IMSF_VERT_MAPPING;

	writeDword(serialFlags, platformMismatch, stream);

	// Export mesh
	writeDword(mMeshData.mNbVertices, platformMismatch, stream);
	writeDword(mMeshData.mNbTriangles, platformMismatch, stream);
	writeFloatBuffer(&mMeshData.mVertices->x, mMeshData.mNbVertices*3, platformMismatch, stream);
	if(serialFlags & IMSF_8BIT_INDICES)
	{
		const PxU32* indices = tris->mRef;
		for(PxU32 i=0;i<mMeshData.mNbTriangles*3;i++)
		{
			PxI8 data = PxI8(indices[i]);		
			stream.write(&data, sizeof(PxU8));	
		}
	}
	else if(serialFlags & IMSF_16BIT_INDICES)
	{
		const PxU32* indices = tris->mRef;
		for(PxU32 i=0;i<mMeshData.mNbTriangles*3;i++)
			writeWord(PxTo16(indices[i]), platformMismatch, stream);
	}
	else
		writeIntBuffer(tris->mRef, mMeshData.mNbTriangles*3, platformMismatch, stream);

	if(mMeshData.mMaterialIndices)
		writeWordBuffer(mMeshData.mMaterialIndices, mMeshData.mNbTriangles, platformMismatch, stream);

	if(mMeshData.mFaceRemap)
	{
		PxU32 maxId = computeMaxIndex(mMeshData.mFaceRemap, mMeshData.mNbTriangles);
		writeDword(maxId, platformMismatch, stream);
		storeIndices(maxId, mMeshData.mNbTriangles, mMeshData.mFaceRemap, stream, platformMismatch);
//		writeIntBuffer(mMeshData.mFaceRemap, mMeshData.mNbTriangles, platformMismatch, stream);
	}

	if(mMeshData.mAdjacencies)
		writeIntBuffer(mMeshData.mAdjacencies, mMeshData.mNbTriangles*3, platformMismatch, stream);

	// Export midphase structure
	saveMidPhaseStructure(stream, platformMismatch);

	// Export local bounds
	writeFloat(mMeshData.mGeomEpsilon, platformMismatch, stream);

	writeFloat(mMeshData.mAABB.minimum.x, platformMismatch, stream);
	writeFloat(mMeshData.mAABB.minimum.y, platformMismatch, stream);
	writeFloat(mMeshData.mAABB.minimum.z, platformMismatch, stream);
	writeFloat(mMeshData.mAABB.maximum.x, platformMismatch, stream);
	writeFloat(mMeshData.mAABB.maximum.y, platformMismatch, stream);
	writeFloat(mMeshData.mAABB.maximum.z, platformMismatch, stream);

	if(mMeshData.mExtraTrigData)
	{
		writeDword(mMeshData.mNbTriangles, platformMismatch, stream);
		// No need to convert those bytes
		stream.write(mMeshData.mExtraTrigData, mMeshData.mNbTriangles*sizeof(PxU8));
	}
	else
		writeDword(0, platformMismatch, stream);

	// GRB write -----------------------------------------------------------------
	if (params.buildGPUData)
	{
		const PxU32* indices = reinterpret_cast<PxU32*>(mMeshData.mGRB_primIndices);
		if (serialFlags & IMSF_8BIT_INDICES)
		{
			for (PxU32 i = 0; i<mMeshData.mNbTriangles * 3; i++)
			{
				PxI8 data = PxI8(indices[i]);
				stream.write(&data, sizeof(PxU8));
			}
		}
		else if (serialFlags & IMSF_16BIT_INDICES)
		{
			for (PxU32 i = 0; i<mMeshData.mNbTriangles * 3; i++)
				writeWord(PxTo16(indices[i]), platformMismatch, stream);
		}
		else
			writeIntBuffer(indices, mMeshData.mNbTriangles * 3, platformMismatch, stream);


		//writeIntBuffer(reinterpret_cast<PxU32*>(mMeshData.mGRB_triIndices), , mMeshData.mNbTriangles*3, platformMismatch, stream);

		//writeIntBuffer(reinterpret_cast<PxU32 *>(mMeshData.mGRB_triIndices), mMeshData.mNbTriangles*4, platformMismatch, stream);

		writeIntBuffer(reinterpret_cast<PxU32 *>(mMeshData.mGRB_primAdjacencies), mMeshData.mNbTriangles*4, platformMismatch, stream);
		writeIntBuffer(mMeshData.mGRB_faceRemap, mMeshData.mNbTriangles, platformMismatch, stream);
		if(mMeshData.mGRB_faceRemapInverse)
			writeIntBuffer(mMeshData.mGRB_faceRemapInverse, mMeshData.mNbTriangles, platformMismatch, stream);

		//Export GPU midphase structure
		BV32Tree* bv32Tree = mMeshData.mGRB_BV32Tree;
		BV32TriangleMeshBuilder::saveMidPhaseStructure(bv32Tree, stream, platformMismatch);

		//Export vertex mapping
		if (enableVertexMapping)
		{
			writeDword(mMeshData.mNbTrianglesReferences, platformMismatch, stream);

			stream.write(mMeshData.mAccumulatedTrianglesRef, mMeshData.mNbVertices * sizeof(PxU32));

			stream.write(mMeshData.mTrianglesReferences, mMeshData.mNbTrianglesReferences * sizeof(PxU32));
		}
	}

	// End of GRB write ----------------------------------------------------------

	// Export sdf values
	if (enableSdf)
	{
		writeFloat(mMeshData.mSdfData.mMeshLower.x, platformMismatch, stream);
		writeFloat(mMeshData.mSdfData.mMeshLower.y, platformMismatch, stream);
		writeFloat(mMeshData.mSdfData.mMeshLower.z, platformMismatch, stream);
		writeFloat(mMeshData.mSdfData.mSpacing, platformMismatch, stream);
		writeDword(mMeshData.mSdfData.mDims.x, platformMismatch, stream);
		writeDword(mMeshData.mSdfData.mDims.y, platformMismatch, stream);
		writeDword(mMeshData.mSdfData.mDims.z, platformMismatch, stream);
		writeDword(mMeshData.mSdfData.mNumSdfs, platformMismatch, stream);

		writeDword(mMeshData.mSdfData.mNumSubgridSdfs, platformMismatch, stream);
		writeDword(mMeshData.mSdfData.mNumStartSlots, platformMismatch, stream);
		writeDword(mMeshData.mSdfData.mSubgridSize, platformMismatch, stream);
		writeDword(mMeshData.mSdfData.mSdfSubgrids3DTexBlockDim.x, platformMismatch, stream);
		writeDword(mMeshData.mSdfData.mSdfSubgrids3DTexBlockDim.y, platformMismatch, stream);
		writeDword(mMeshData.mSdfData.mSdfSubgrids3DTexBlockDim.z, platformMismatch, stream);

		writeFloat(mMeshData.mSdfData.mSubgridsMinSdfValue, platformMismatch, stream);
		writeFloat(mMeshData.mSdfData.mSubgridsMaxSdfValue, platformMismatch, stream);
		writeDword(mMeshData.mSdfData.mBytesPerSparsePixel, platformMismatch, stream);

		writeFloatBuffer(mMeshData.mSdfData.mSdf, mMeshData.mSdfData.mNumSdfs, platformMismatch, stream);
		writeByteBuffer(mMeshData.mSdfData.mSubgridSdf, mMeshData.mSdfData.mNumSubgridSdfs, stream);
		writeIntBuffer(mMeshData.mSdfData.mSubgridStartSlots, mMeshData.mSdfData.mNumStartSlots, platformMismatch, stream);
	}

	//Export Inertia tensor
	if(enableInertia)
	{
		writeFloat(mMeshData.mMass, platformMismatch, stream);
		writeFloatBuffer(reinterpret_cast<const PxF32*>(&mMeshData.mInertia), 9, platformMismatch, stream);
		writeFloatBuffer(&mMeshData.mLocalCenterOfMass.x, 3, platformMismatch, stream);
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////

#if PX_VC
#pragma warning(push)
#pragma warning(disable:4996)	// permitting use of gatherStrided until we have a replacement.
#endif

bool TriangleMeshBuilder::importMesh(const PxTriangleMeshDesc& desc,const PxCookingParams& params,PxTriangleMeshCookingResult::Enum* condition, bool validate)
{
	//convert and clean the input mesh
	//this is where the mesh data gets copied from user mem to our mem

	PxVec3* verts = mMeshData.allocateVertices(desc.points.count);
	IndexedTriangle32* tris = reinterpret_cast<IndexedTriangle32*>(mMeshData.allocateTriangles(desc.triangles.count, true, PxU32(params.buildGPUData)));

	//copy, and compact to get rid of strides:
	immediateCooking::gatherStrided(desc.points.data, verts, mMeshData.mNbVertices, sizeof(PxVec3), desc.points.stride);

#if PX_CHECKED
	// PT: check all input vertices are valid
	for(PxU32 i=0;i<desc.points.count;i++)
	{
		const PxVec3& p = verts[i];
		if(!PxIsFinite(p.x) || !PxIsFinite(p.y) || !PxIsFinite(p.z))
			return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "input mesh contains corrupted vertex data");
	}
#endif

	//for trigs index stride conversion and eventual reordering is also needed, I don't think flexicopy can do that for us.

	IndexedTriangle32* dest = tris;
	const IndexedTriangle32* pastLastDest = tris + mMeshData.mNbTriangles;
	const PxU8* source = reinterpret_cast<const PxU8*>(desc.triangles.data);

	//4 combos of 16 vs 32 and flip vs no flip
	PxU32 c = (desc.flags & PxMeshFlag::eFLIPNORMALS)?PxU32(1):0;
	if (desc.flags & PxMeshFlag::e16_BIT_INDICES)
	{
		//index stride conversion is also needed, I don't think flexicopy can do that for us.
		while (dest < pastLastDest)
		{
			const PxU16 * trig16 = reinterpret_cast<const PxU16*>(source);
			dest->mRef[0] = trig16[0];
			dest->mRef[1] = trig16[1+c];
			dest->mRef[2] = trig16[2-c];
			dest ++;
			source += desc.triangles.stride;
		}
	}
	else
	{
		while (dest < pastLastDest)
		{
			const PxU32 * trig32 = reinterpret_cast<const PxU32*>(source);
			dest->mRef[0] = trig32[0];
			dest->mRef[1] = trig32[1+c];
			dest->mRef[2] = trig32[2-c];
			dest ++;
			source += desc.triangles.stride;
		}
	}

	//copy the material index list if any:
	if(desc.materialIndices.data)
	{
		PxMaterialTableIndex* materials = mMeshData.allocateMaterials();
		immediateCooking::gatherStrided(desc.materialIndices.data, materials, mMeshData.mNbTriangles, sizeof(PxMaterialTableIndex), desc.materialIndices.stride);

		// Check material indices
		for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)	PX_ASSERT(materials[i]!=0xffff);
	}

	// Clean the mesh using ICE's MeshBuilder
	// This fixes the bug in ConvexTest06 where the inertia tensor computation fails for a mesh => it works with a clean mesh

	if (!(params.meshPreprocessParams & PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH) || validate)
	{
		if(!cleanMesh(validate, condition))
		{
			if(!validate)
				outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "cleaning the mesh failed");
			return false;
		}
	}
	else
	{
		// we need to fill the remap table if no cleaning was done
		if(params.suppressTriangleMeshRemapTable == false)
		{
			PX_ASSERT(mMeshData.mFaceRemap == NULL);
			mMeshData.mFaceRemap = PX_ALLOCATE(PxU32, mMeshData.mNbTriangles, "mFaceRemap");
			for (PxU32 i = 0; i < mMeshData.mNbTriangles; i++)
				mMeshData.mFaceRemap[i] = i;
		}
	}


	if (mParams.meshPreprocessParams & PxMeshPreprocessingFlag::eENABLE_INERTIA)
	{
		buildInertiaTensor();
	}

	// Copy sdf data if enabled
	if (desc.sdfDesc)
	{

		PxArray<PxReal> sdfData;
		PxArray<PxU8> sdfDataSubgrids;
		PxArray<PxU32> sdfSubgridsStartSlots;

		PxTriangleMeshDesc newDesc;
		newDesc.points.count = mMeshData.mNbVertices;
		newDesc.points.stride = sizeof(PxVec3);
		newDesc.points.data = mMeshData.mVertices;
		newDesc.triangles.count = mMeshData.mNbTriangles;
		newDesc.triangles.stride = sizeof(PxU32) * 3;
		newDesc.triangles.data = mMeshData.mTriangles;
		newDesc.flags &= (~PxMeshFlag::e16_BIT_INDICES);
		newDesc.sdfDesc = desc.sdfDesc;

		buildSDF(newDesc, sdfData, sdfDataSubgrids, sdfSubgridsStartSlots);

		PxSDFDesc& sdfDesc = *desc.sdfDesc;

		PxReal* sdf = mMeshData.mSdfData.allocateSdfs(sdfDesc.meshLower, sdfDesc.spacing, sdfDesc.dims.x, sdfDesc.dims.y, sdfDesc.dims.z, 
			sdfDesc.subgridSize, sdfDesc.sdfSubgrids3DTexBlockDim.x, sdfDesc.sdfSubgrids3DTexBlockDim.y, sdfDesc.sdfSubgrids3DTexBlockDim.z,
			sdfDesc.subgridsMinSdfValue, sdfDesc.subgridsMaxSdfValue, sdfDesc.bitsPerSubgridPixel);

		
		if (sdfDesc.subgridSize > 0)
		{
			//Sparse sdf
			immediateCooking::gatherStrided(sdfDesc.sdf.data, sdf, sdfDesc.sdf.count, sizeof(PxReal), sdfDesc.sdf.stride);

			immediateCooking::gatherStrided(sdfDesc.sdfSubgrids.data, mMeshData.mSdfData.mSubgridSdf, 
				sdfDesc.sdfSubgrids.count,
				sizeof(PxU8), sdfDesc.sdfSubgrids.stride);
			immediateCooking::gatherStrided(sdfDesc.sdfStartSlots.data, mMeshData.mSdfData.mSubgridStartSlots, sdfDesc.sdfStartSlots.count, sizeof(PxU32), sdfDesc.sdfStartSlots.stride);
		}
		else
		{
			//copy, and compact to get rid of strides:
			immediateCooking::gatherStrided(sdfDesc.sdf.data, sdf, sdfDesc.dims.x*sdfDesc.dims.y*sdfDesc.dims.z, sizeof(PxReal), sdfDesc.sdf.stride);
		}

		//Make sure there is always a valid inertia tensor for meshes with an SDF
		buildInertiaTensorFromSDF();		

#if PX_CHECKED
		// SN: check all input sdf values are valid
		for (PxU32 i = 0; i < sdfDesc.sdf.count; ++i)
		{
			if (!PxIsFinite(sdf[i]))
				return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "input sdf contains corrupted data");
		}
#endif
	}

	return true;
}

#if PX_VC
#pragma warning(pop)
#endif
///////////////////////////////////////////////////////////////////////////////

void TriangleMeshBuilder::checkMeshIndicesSize()
{
	TriangleMeshData& m = mMeshData;

	// check if we can change indices from 32bits to 16bits
	if(m.mNbVertices <= 0xffff && !m.has16BitIndices())
	{
		const PxU32 numTriangles = m.mNbTriangles;
		PxU32* PX_RESTRICT indices32 = reinterpret_cast<PxU32*> (m.mTriangles);
		PxU32* PX_RESTRICT grbIndices32 = reinterpret_cast<PxU32*>(m.mGRB_primIndices);
		
		m.mTriangles = 0;					// force a realloc
		m.allocateTriangles(numTriangles, false, grbIndices32 != NULL ? 1u : 0u);
		PX_ASSERT(m.has16BitIndices());		// realloc'ing without the force32bit flag changed it.

		PxU16* PX_RESTRICT indices16 = reinterpret_cast<PxU16*> (m.mTriangles);
		for (PxU32 i = 0; i < numTriangles * 3; i++)
			indices16[i] = PxTo16(indices32[i]);

		PX_FREE(indices32);

		if (grbIndices32)
		{
			PxU16* PX_RESTRICT grbIndices16 = reinterpret_cast<PxU16*> (m.mGRB_primIndices);
			for (PxU32 i = 0; i < numTriangles * 3; i++)
				grbIndices16[i] = PxTo16(grbIndices32[i]);
		}

		PX_FREE(grbIndices32);

		onMeshIndexFormatChange();
	}
}

///////////////////////////////////////////////////////////////////////////////

BV4TriangleMeshBuilder::BV4TriangleMeshBuilder(const PxCookingParams& params) : TriangleMeshBuilder(mData, params)
{
}

BV4TriangleMeshBuilder::~BV4TriangleMeshBuilder()
{
}

void BV4TriangleMeshBuilder::onMeshIndexFormatChange()
{
	IndTri32* triangles32 = NULL;
	IndTri16* triangles16 = NULL;
	if(mMeshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES)
		triangles16 = reinterpret_cast<IndTri16*>(mMeshData.mTriangles);
	else
		triangles32 = reinterpret_cast<IndTri32*>(mMeshData.mTriangles);

	mData.mMeshInterface.setPointers(triangles32, triangles16, mMeshData.mVertices);
}

void BV4TriangleMeshBuilder::createMidPhaseStructure()
{
	GU_PROFILE_ZONE("createMidPhaseStructure_BV4")

	const float gBoxEpsilon = 2e-4f;
//	const float gBoxEpsilon = 0.1f;
	mData.mMeshInterface.initRemap();
	mData.mMeshInterface.setNbVertices(mMeshData.mNbVertices);
	mData.mMeshInterface.setNbTriangles(mMeshData.mNbTriangles);

	IndTri32* triangles32 = NULL;
	IndTri16* triangles16 = NULL;
	if (mMeshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES)
		triangles16 = reinterpret_cast<IndTri16*>(mMeshData.mTriangles);
	else
		triangles32 = reinterpret_cast<IndTri32*>(mMeshData.mTriangles);

	mData.mMeshInterface.setPointers(triangles32, triangles16, mMeshData.mVertices);

	PX_ASSERT(mParams.midphaseDesc.getType() == PxMeshMidPhase::eBVH34);
	const PxU32 nbTrisPerLeaf = mParams.midphaseDesc.mBVH34Desc.numPrimsPerLeaf;
	const bool quantized = mParams.midphaseDesc.mBVH34Desc.quantized;

	const PxBVH34BuildStrategy::Enum strategy = mParams.midphaseDesc.mBVH34Desc.buildStrategy;
	BV4_BuildStrategy gubs = BV4_SPLATTER_POINTS_SPLIT_GEOM_CENTER;	// Default
	if(strategy==PxBVH34BuildStrategy::eSAH)
		gubs = BV4_SAH;
	else if(strategy==PxBVH34BuildStrategy::eFAST)
		gubs = BV4_SPLATTER_POINTS;
	if(!BuildBV4Ex(mData.mBV4Tree, mData.mMeshInterface, gBoxEpsilon, nbTrisPerLeaf, quantized, gubs))
	{
		outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "BV4 tree failed to build.");
		return;
	}

	{
		GU_PROFILE_ZONE("..BV4 remap")

//		remapTopology(mData.mMeshInterface);

		const PxU32* order = mData.mMeshInterface.getRemap();
		if(mMeshData.mMaterialIndices)
		{
			PxMaterialTableIndex* newMat = PX_ALLOCATE(PxMaterialTableIndex, mMeshData.mNbTriangles, "mMaterialIndices");
			for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)
				newMat[i] = mMeshData.mMaterialIndices[order[i]];
			PX_FREE(mMeshData.mMaterialIndices);
			mMeshData.mMaterialIndices = newMat;
		}

		if (!mParams.suppressTriangleMeshRemapTable || mParams.buildGPUData)
		{
			PxU32* newMap = PX_ALLOCATE(PxU32, mMeshData.mNbTriangles, "mFaceRemap");
			for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)
				newMap[i] = mMeshData.mFaceRemap ? mMeshData.mFaceRemap[order[i]] : order[i];
			PX_FREE(mMeshData.mFaceRemap);
			mMeshData.mFaceRemap = newMap;
		}
		mData.mMeshInterface.releaseRemap();
	}
}

void BV4TriangleMeshBuilder::saveMidPhaseStructure(PxOutputStream& stream, bool mismatch) const
{
	// PT: in version 1 we defined "mismatch" as:
	// const bool mismatch = (littleEndian() == 1);
	// i.e. the data was *always* saved to file in big-endian format no matter what.
	// In version>1 we now do the same as for other structures in the SDK: the data is
	// exported either as little or big-endian depending on the passed parameter.
	const PxU32 bv4StructureVersion = 3;

	writeChunk('B', 'V', '4', ' ', stream);
	writeDword(bv4StructureVersion, mismatch, stream);

	writeFloat(mData.mBV4Tree.mLocalBounds.mCenter.x, mismatch, stream);
	writeFloat(mData.mBV4Tree.mLocalBounds.mCenter.y, mismatch, stream);
	writeFloat(mData.mBV4Tree.mLocalBounds.mCenter.z, mismatch, stream);
	writeFloat(mData.mBV4Tree.mLocalBounds.mExtentsMagnitude, mismatch, stream);

	writeDword(mData.mBV4Tree.mInitData, mismatch, stream);

	writeFloat(mData.mBV4Tree.mCenterOrMinCoeff.x, mismatch, stream);
	writeFloat(mData.mBV4Tree.mCenterOrMinCoeff.y, mismatch, stream);
	writeFloat(mData.mBV4Tree.mCenterOrMinCoeff.z, mismatch, stream);
	writeFloat(mData.mBV4Tree.mExtentsOrMaxCoeff.x, mismatch, stream);
	writeFloat(mData.mBV4Tree.mExtentsOrMaxCoeff.y, mismatch, stream);
	writeFloat(mData.mBV4Tree.mExtentsOrMaxCoeff.z, mismatch, stream);

	// PT: version 3
	writeDword(PxU32(mData.mBV4Tree.mQuantized), mismatch, stream);

	writeDword(mData.mBV4Tree.mNbNodes, mismatch, stream);

#ifdef GU_BV4_USE_SLABS
	// PT: we use BVDataPacked to get the size computation right, but we're dealing with BVDataSwizzled here!
	const PxU32 NodeSize = mData.mBV4Tree.mQuantized ? sizeof(BVDataPackedQ) : sizeof(BVDataPackedNQ);
	stream.write(mData.mBV4Tree.mNodes, NodeSize*mData.mBV4Tree.mNbNodes);
	PX_ASSERT(!mismatch);
#else
	#error	Not implemented
#endif
}

///////////////////////////////////////////////////////////////////////////////

void BV32TriangleMeshBuilder::createMidPhaseStructure(const PxCookingParams& params, TriangleMeshData& meshData, BV32Tree& bv32Tree)
{
	GU_PROFILE_ZONE("createMidPhaseStructure_BV32")

	const float gBoxEpsilon = 2e-4f;

	SourceMesh meshInterface;
	//	const float gBoxEpsilon = 0.1f;
	meshInterface.initRemap();
	meshInterface.setNbVertices(meshData.mNbVertices);
	meshInterface.setNbTriangles(meshData.mNbTriangles);

	PX_ASSERT(!(meshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES));

	IndTri32* triangles32 = reinterpret_cast<IndTri32*>(meshData.mGRB_primIndices);

	meshInterface.setPointers(triangles32, NULL, meshData.mVertices);

	const PxU32 nbTrisPerLeaf = 32;

	if (!BuildBV32Ex(bv32Tree, meshInterface, gBoxEpsilon, nbTrisPerLeaf))
	{
		outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "BV32 tree failed to build.");
		return;
	}

	{
		GU_PROFILE_ZONE("..BV32 remap")

		const PxU32* order = meshInterface.getRemap();

		if (!params.suppressTriangleMeshRemapTable || params.buildGPUData)
		{
			PxU32* newMap = PX_ALLOCATE(PxU32, meshData.mNbTriangles, "mGRB_faceRemap");
			for (PxU32 i = 0; i<meshData.mNbTriangles; i++)
				newMap[i] = meshData.mGRB_faceRemap ? meshData.mGRB_faceRemap[order[i]] : order[i];
			PX_FREE(meshData.mGRB_faceRemap);
			meshData.mGRB_faceRemap = newMap;

			if (!params.suppressTriangleMeshRemapTable)
			{
				PxU32* newMapInverse = PX_ALLOCATE(PxU32, meshData.mNbTriangles, "mGRB_faceRemapInverse");
				for (PxU32 i = 0; i < meshData.mNbTriangles; ++i)
					newMapInverse[meshData.mGRB_faceRemap[i]] = i;
				PX_FREE(meshData.mGRB_faceRemapInverse);
				meshData.mGRB_faceRemapInverse = newMapInverse;
			}
		}
	
		meshInterface.releaseRemap();
	}
}

void BV32TriangleMeshBuilder::saveMidPhaseStructure(BV32Tree* bv32Tree, PxOutputStream& stream, bool mismatch)
{
	// PT: in version 1 we defined "mismatch" as:
	// const bool mismatch = (littleEndian() == 1);
	// i.e. the data was *always* saved to file in big-endian format no matter what.
	// In version>1 we now do the same as for other structures in the SDK: the data is
	// exported either as little or big-endian depending on the passed parameter.
	const PxU32 bv32StructureVersion = 2;

	writeChunk('B', 'V', '3', '2', stream);
	writeDword(bv32StructureVersion, mismatch, stream);

	writeFloat(bv32Tree->mLocalBounds.mCenter.x, mismatch, stream);
	writeFloat(bv32Tree->mLocalBounds.mCenter.y, mismatch, stream);
	writeFloat(bv32Tree->mLocalBounds.mCenter.z, mismatch, stream);
	writeFloat(bv32Tree->mLocalBounds.mExtentsMagnitude, mismatch, stream);

	writeDword(bv32Tree->mInitData, mismatch, stream);

	writeDword(bv32Tree->mNbPackedNodes, mismatch, stream);

	PX_ASSERT(bv32Tree->mNbPackedNodes > 0);
	for (PxU32 i = 0; i < bv32Tree->mNbPackedNodes; ++i)
	{
		BV32DataPacked& node = bv32Tree->mPackedNodes[i];

		const PxU32 nbElements = node.mNbNodes * 4;
		writeDword(node.mNbNodes, mismatch, stream);
		writeDword(node.mDepth, mismatch, stream);
		WriteDwordBuffer(node.mData, node.mNbNodes, mismatch, stream);
		writeFloatBuffer(&node.mMin[0].x, nbElements, mismatch, stream);
		writeFloatBuffer(&node.mMax[0].x, nbElements, mismatch, stream);
	}

	writeDword(bv32Tree->mMaxTreeDepth, mismatch, stream);

	PX_ASSERT(bv32Tree->mMaxTreeDepth > 0);
	
	for (PxU32 i = 0; i < bv32Tree->mMaxTreeDepth; ++i)
	{
		BV32DataDepthInfo& info = bv32Tree->mTreeDepthInfo[i];
		
		writeDword(info.offset, mismatch, stream);
		writeDword(info.count, mismatch, stream);
	}

	WriteDwordBuffer(bv32Tree->mRemapPackedNodeIndexWithDepth, bv32Tree->mNbPackedNodes, mismatch, stream);
}

///////////////////////////////////////////////////////////////////////////////

RTreeTriangleMeshBuilder::RTreeTriangleMeshBuilder(const PxCookingParams& params) : TriangleMeshBuilder(mData, params)
{
}

RTreeTriangleMeshBuilder::~RTreeTriangleMeshBuilder()
{
}

struct RTreeCookerRemap : RTreeCooker::RemapCallback 
{
	PxU32 mNbTris;
	RTreeCookerRemap(PxU32 numTris) : mNbTris(numTris)
	{
	}

	virtual void remap(PxU32* val, PxU32 start, PxU32 leafCount)
	{
		PX_ASSERT(leafCount > 0);
		PX_ASSERT(leafCount <= 16); // sanity check
		PX_ASSERT(start < mNbTris);
		PX_ASSERT(start+leafCount <= mNbTris);
		PX_ASSERT(val);
		LeafTriangles lt;
		// here we remap from ordered leaf index in the rtree to index in post-remap in triangles
		// this post-remap will happen later
		lt.SetData(leafCount, start);
		*val = lt.Data;
	}
};

void RTreeTriangleMeshBuilder::createMidPhaseStructure()
{
	GU_PROFILE_ZONE("createMidPhaseStructure_RTREE")

	const PxReal meshSizePerformanceTradeOff = mParams.midphaseDesc.mBVH33Desc.meshSizePerformanceTradeOff;
	const PxMeshCookingHint::Enum meshCookingHint = mParams.midphaseDesc.mBVH33Desc.meshCookingHint;

	PxArray<PxU32> resultPermute;
	RTreeCookerRemap rc(mMeshData.mNbTriangles);
	RTreeCooker::buildFromTriangles(
		mData.mRTree,
		mMeshData.mVertices, mMeshData.mNbVertices,
		(mMeshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES) ? reinterpret_cast<PxU16*>(mMeshData.mTriangles) : NULL,
		!(mMeshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES) ? reinterpret_cast<PxU32*>(mMeshData.mTriangles) : NULL,
		mMeshData.mNbTriangles, resultPermute, &rc, meshSizePerformanceTradeOff, meshCookingHint);

	PX_ASSERT(resultPermute.size() == mMeshData.mNbTriangles);

	remapTopology(resultPermute.begin());
}

void RTreeTriangleMeshBuilder::saveMidPhaseStructure(PxOutputStream& stream, bool mismatch) const
{
	// PT: in version 1 we defined "mismatch" as:
	// const bool mismatch = (littleEndian() == 1);
	// i.e. the data was *always* saved to file in big-endian format no matter what.
	// In version>1 we now do the same as for other structures in the SDK: the data is
	// exported either as little or big-endian depending on the passed parameter.
	const PxU32 rtreeStructureVersion = 2;

	// save the RTree root structure followed immediately by RTreePage pages to an output stream
	writeChunk('R', 'T', 'R', 'E', stream);

	writeDword(rtreeStructureVersion, mismatch, stream);
	const RTree& d = mData.mRTree;
	writeFloatBuffer(&d.mBoundsMin.x, 4, mismatch, stream);
	writeFloatBuffer(&d.mBoundsMax.x, 4, mismatch, stream);
	writeFloatBuffer(&d.mInvDiagonal.x, 4, mismatch, stream);
	writeFloatBuffer(&d.mDiagonalScaler.x, 4, mismatch, stream);
	writeDword(d.mPageSize, mismatch, stream);
	writeDword(d.mNumRootPages, mismatch, stream);
	writeDword(d.mNumLevels, mismatch, stream);
	writeDword(d.mTotalNodes, mismatch, stream);
	writeDword(d.mTotalPages, mismatch, stream);
	PxU32 unused = 0; writeDword(unused, mismatch, stream); // backwards compatibility
	for (PxU32 j = 0; j < d.mTotalPages; j++)
	{
		writeFloatBuffer(d.mPages[j].minx, RTREE_N, mismatch, stream);
		writeFloatBuffer(d.mPages[j].miny, RTREE_N, mismatch, stream);
		writeFloatBuffer(d.mPages[j].minz, RTREE_N, mismatch, stream);
		writeFloatBuffer(d.mPages[j].maxx, RTREE_N, mismatch, stream);
		writeFloatBuffer(d.mPages[j].maxy, RTREE_N, mismatch, stream);
		writeFloatBuffer(d.mPages[j].maxz, RTREE_N, mismatch, stream);
		WriteDwordBuffer(d.mPages[j].ptrs, RTREE_N, mismatch, stream);
	}
}

///////////////////////////////////////////////////////////////////////////////

bool immediateCooking::validateTriangleMesh(const PxCookingParams& params, const PxTriangleMeshDesc& desc)
{
	// cooking code does lots of float bitwise reinterpretation that generates exceptions
	PX_FPU_GUARD;

	if(!desc.isValid())
		return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "Cooking::validateTriangleMesh: user-provided triangle mesh descriptor is invalid!");

	// PT: validation code doesn't look at midphase data, so ideally we wouldn't build the midphase structure at all here.
	if(params.midphaseDesc.getType() == PxMeshMidPhase::eBVH33)
	{
		RTreeTriangleMeshBuilder builder(params);
		return builder.loadFromDesc(desc, NULL, true /*doValidate*/);
	}
	else if(params.midphaseDesc.getType() == PxMeshMidPhase::eBVH34)
	{
		BV4TriangleMeshBuilder builder(params);
		return builder.loadFromDesc(desc, NULL, true /*doValidate*/);
	}
	else
		return false;
}

///////////////////////////////////////////////////////////////////////////////

PxTriangleMesh* immediateCooking::createTriangleMesh(const PxCookingParams& params, const PxTriangleMeshDesc& desc, PxInsertionCallback& insertionCallback, PxTriangleMeshCookingResult::Enum* condition)
{
	struct Local
	{
		static PxTriangleMesh* createTriangleMesh(const PxCookingParams& params, TriangleMeshBuilder& builder, const PxTriangleMeshDesc& desc, PxInsertionCallback& insertionCallback, PxTriangleMeshCookingResult::Enum* condition)
		{	
			// cooking code does lots of float bitwise reinterpretation that generates exceptions
			PX_FPU_GUARD;

			if(condition)
				*condition = PxTriangleMeshCookingResult::eSUCCESS;
			if(!builder.loadFromDesc(desc, condition, false))
				return NULL;

			// check if the indices can be moved from 32bits to 16bits
			if(!(params.meshPreprocessParams & PxMeshPreprocessingFlag::eFORCE_32BIT_INDICES))
				builder.checkMeshIndicesSize();

			PxConcreteType::Enum type;
			if(builder.getMidphaseID()==PxMeshMidPhase::eBVH33)
				type = PxConcreteType::eTRIANGLE_MESH_BVH33;
			else
				type = PxConcreteType::eTRIANGLE_MESH_BVH34;

			return static_cast<PxTriangleMesh*>(insertionCallback.buildObjectFromData(type, &builder.getMeshData()));
		}
	};

	if(params.midphaseDesc.getType() == PxMeshMidPhase::eBVH33)
	{
		RTreeTriangleMeshBuilder builder(params);
		return Local::createTriangleMesh(params, builder, desc, insertionCallback, condition);
	}
	else
	{
		BV4TriangleMeshBuilder builder(params);
		return Local::createTriangleMesh(params, builder, desc, insertionCallback, condition);
	}
}

///////////////////////////////////////////////////////////////////////////////

bool immediateCooking::cookTriangleMesh(const PxCookingParams& params, const PxTriangleMeshDesc& desc, PxOutputStream& stream, PxTriangleMeshCookingResult::Enum* condition)
{
	struct Local
	{
		static bool cookTriangleMesh(const PxCookingParams& params, TriangleMeshBuilder& builder, const PxTriangleMeshDesc& desc, PxOutputStream& stream, PxTriangleMeshCookingResult::Enum* condition)
		{
			// cooking code does lots of float bitwise reinterpretation that generates exceptions
			PX_FPU_GUARD;

			if(condition)
				*condition = PxTriangleMeshCookingResult::eSUCCESS;
			if(!builder.loadFromDesc(desc, condition, false))
				return false;

			builder.save(stream, immediateCooking::platformMismatch(), params);
			return true;
		}
	};

	if(params.midphaseDesc.getType() == PxMeshMidPhase::eBVH33)
	{
		RTreeTriangleMeshBuilder builder(params);
		return Local::cookTriangleMesh(params, builder, desc, stream, condition);
	}
	else
	{
		BV4TriangleMeshBuilder builder(params);
		return Local::cookTriangleMesh(params, builder, desc, stream, condition);
	}
}

///////////////////////////////////////////////////////////////////////////////

