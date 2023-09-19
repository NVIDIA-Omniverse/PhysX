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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
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
		if(mParams.meshWeldTolerance == 0.0f)
			outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "TriangleMeshBuilder::cleanMesh: mesh welding enabled with 0 weld tolerance!");
		else
			meshWeldTolerance = mParams.meshWeldTolerance;
	}

	MeshCleaner cleaner(mMeshData.mNbVertices, mMeshData.mVertices, mMeshData.mNbTriangles, reinterpret_cast<const PxU32*>(mMeshData.mTriangles), meshWeldTolerance, mParams.meshAreaMinLimit);
	if(!cleaner.mNbTris)
	{
		if(condition)
			*condition = PxTriangleMeshCookingResult::eEMPTY_MESH;

		return outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "TriangleMeshBuilder::cleanMesh: mesh cleaning removed all triangles!");
	}

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

		const bool testEdgeLength = mParams.meshEdgeLengthMaxLimit!=0.0f;
		const float testLengthSquared = mParams.meshEdgeLengthMaxLimit * mParams.meshEdgeLengthMaxLimit * mParams.scale.length * mParams.scale.length;
		bool foundLargeTriangle = false;
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

			if(testEdgeLength)
			{
				const PxVec3& v0 = v[vref0];
				const PxVec3& v1 = v[vref1];
				const PxVec3& v2 = v[vref2];

				if(		(v0 - v1).magnitudeSquared() >= testLengthSquared
					||	(v1 - v2).magnitudeSquared() >= testLengthSquared
					||	(v2 - v0).magnitudeSquared() >= testLengthSquared
					)
					foundLargeTriangle = true;
			}
		}

		if(foundLargeTriangle)
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

bool TriangleMeshBuilder::createGRBMidPhaseAndData(const PxU32 originalTriangleCount)
{
	PX_UNUSED(originalTriangleCount);
	if (mParams.buildGPUData)
	{
		PX_ASSERT(!(mMeshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES));

		BV32Tree* bv32Tree = PX_NEW(BV32Tree);
		mMeshData.mGRB_BV32Tree = bv32Tree;

		if(!BV32TriangleMeshBuilder::createMidPhaseStructure(mParams, mMeshData, *bv32Tree))
			return false;
		
		createGRBData();

		if (mParams.meshPreprocessParams & PxMeshPreprocessingFlag::eENABLE_VERT_MAPPING || mParams.buildGPUData)
			createVertMapping();

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
	return true;
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
		if (!importMesh(desc, condition, validateMesh))
		{
			PX_FREE(topology);
			return false;
		}

		// Cleanup if needed
		PX_FREE(topology);
	}

	if(!createMidPhaseStructure())
		return false;

	//copy the BV4 triangle indices to grb triangle indices if buildGRBData is true
	recordTriangleIndices();

	// Compute local bounds
	computeLocalBoundsAndGeomEpsilon(mMeshData.mVertices, mMeshData.mNbVertices, mMeshData.mAABB, mMeshData.mGeomEpsilon);	

	createSharedEdgeData(mParams.buildTriangleAdjacencies, !(mParams.meshPreprocessParams & PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE));

	return createGRBMidPhaseAndData(originalTriangleCount);
}

void TriangleMeshBuilder::buildInertiaTensor(bool flipNormals)
{
	PxTriangleMeshDesc simpleMesh;

	simpleMesh.points.count = mMeshData.mNbVertices;
	simpleMesh.points.stride = sizeof(PxVec3);
	simpleMesh.points.data = mMeshData.mVertices;
	simpleMesh.triangles.count = mMeshData.mNbTriangles;
	simpleMesh.triangles.stride = sizeof(PxU32) * 3;
	simpleMesh.triangles.data = mMeshData.mTriangles;
	simpleMesh.flags &= (~PxMeshFlag::e16_BIT_INDICES);
	if (flipNormals)
		simpleMesh.flags.raise(PxMeshFlag::eFLIPNORMALS);

	PxIntegrals integrals;
	computeVolumeIntegrals(simpleMesh, 1, integrals);

	integrals.getOriginInertia(mMeshData.mInertia);
	mMeshData.mMass = PxReal(integrals.mass);
	mMeshData.mLocalCenterOfMass = integrals.COM;
}

void TriangleMeshBuilder::buildInertiaTensorFromSDF()
{
	if (MeshAnalyzer::checkMeshWatertightness(reinterpret_cast<const Gu::Triangle*>(mMeshData.mTriangles), mMeshData.mNbTriangles))
	{
		buildInertiaTensor();
		if (mMeshData.mMass < 0.0f)
			buildInertiaTensor(true); //The mesh can be watertight but all triangles might be oriented the wrong way round
		return;
	}
	
	PxArray<PxVec3> points;
	PxArray<PxU32> triangleIndices;
	extractIsosurfaceFromSDF(mMeshData.mSdfData, points, triangleIndices);

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

#if PX_CHECKED
bool checkInputFloats(PxU32 nb, const float* values, const char* file, PxU32 line, const char* errorMsg)
{
	while(nb--)
	{
		if(!PxIsFinite(*values++))
			return PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, file, line, errorMsg);
	}
	return true;
}
#endif

bool TriangleMeshBuilder::importMesh(const PxTriangleMeshDesc& desc, PxTriangleMeshCookingResult::Enum* condition, bool validate)
{
	//convert and clean the input mesh
	//this is where the mesh data gets copied from user mem to our mem

	PxVec3* verts = mMeshData.allocateVertices(desc.points.count);
	IndexedTriangle32* tris = reinterpret_cast<IndexedTriangle32*>(mMeshData.allocateTriangles(desc.triangles.count, true, PxU32(mParams.buildGPUData)));

	//copy, and compact to get rid of strides:
	immediateCooking::gatherStrided(desc.points.data, verts, mMeshData.mNbVertices, sizeof(PxVec3), desc.points.stride);

#if PX_CHECKED
	// PT: check all input vertices are valid
	if(!checkInputFloats(desc.points.count*3, &verts->x, PX_FL, "input mesh contains corrupted vertex data"))
		return false;
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
		for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)
			PX_ASSERT(materials[i]!=0xffff);
	}

	// Clean the mesh using ICE's MeshBuilder
	// This fixes the bug in ConvexTest06 where the inertia tensor computation fails for a mesh => it works with a clean mesh

	if (!(mParams.meshPreprocessParams & PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH) || validate)
	{
		if(!cleanMesh(validate, condition))
			return false;
	}
	else
	{
		// we need to fill the remap table if no cleaning was done
		if(mParams.suppressTriangleMeshRemapTable == false)
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
		if(!checkInputFloats(sdfDesc.sdf.count, sdf, PX_FL, "input sdf contains corrupted data"))
			return false;
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

bool BV4TriangleMeshBuilder::createMidPhaseStructure()
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
		return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "BV4 tree failed to build.");

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
	return true;
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

bool BV32TriangleMeshBuilder::createMidPhaseStructure(const PxCookingParams& params, TriangleMeshData& meshData, BV32Tree& bv32Tree)
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
		return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "BV32 tree failed to build.");

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
	return true;
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

bool RTreeTriangleMeshBuilder::createMidPhaseStructure()
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
	return true;
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
		static PxTriangleMesh* createTriangleMesh(const PxCookingParams& cookingParams_, TriangleMeshBuilder& builder, const PxTriangleMeshDesc& desc_, PxInsertionCallback& insertionCallback_, PxTriangleMeshCookingResult::Enum* condition_)
		{	
			// cooking code does lots of float bitwise reinterpretation that generates exceptions
			PX_FPU_GUARD;

			if(condition_)
				*condition_ = PxTriangleMeshCookingResult::eSUCCESS;
			if(!builder.loadFromDesc(desc_, condition_, false))
				return NULL;

			// check if the indices can be moved from 32bits to 16bits
			if(!(cookingParams_.meshPreprocessParams & PxMeshPreprocessingFlag::eFORCE_32BIT_INDICES))
				builder.checkMeshIndicesSize();

			PxConcreteType::Enum type;
			if(builder.getMidphaseID()==PxMeshMidPhase::eBVH33)
				type = PxConcreteType::eTRIANGLE_MESH_BVH33;
			else
				type = PxConcreteType::eTRIANGLE_MESH_BVH34;

			return static_cast<PxTriangleMesh*>(insertionCallback_.buildObjectFromData(type, &builder.getMeshData()));
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
		static bool cookTriangleMesh(const PxCookingParams& cookingParams_, TriangleMeshBuilder& builder, const PxTriangleMeshDesc& desc_, PxOutputStream& stream_, PxTriangleMeshCookingResult::Enum* condition_)
		{
			// cooking code does lots of float bitwise reinterpretation that generates exceptions
			PX_FPU_GUARD;

			if(condition_)
				*condition_ = PxTriangleMeshCookingResult::eSUCCESS;
			if(!builder.loadFromDesc(desc_, condition_, false))
				return false;

			builder.save(stream_, immediateCooking::platformMismatch(), cookingParams_);
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

