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

#include "geometry/PxConvexMeshGeometry.h"
#include "GuHeightFieldUtil.h"
#include "GuEntityReport.h"
#include "GuConvexMesh.h"
#include "GuSweepSharedTests.h"  
#include "GuConvexUtilsInternal.h"
#include "GuTriangleMesh.h"
#include "GuVecBox.h"
#include "GuVecTriangle.h"
#include "GuVecConvexHullNoScale.h"
#include "GuMidphaseInterface.h"
#include "GuPCMContactConvexCommon.h"
#include "GuSweepMTD.h"
#include "GuPCMShapeConvex.h"
#include "GuDistanceSegmentSegment.h"
#include "GuDistancePointSegment.h"
#include "GuInternal.h"
#include "GuConvexEdgeFlags.h"
#include "GuMTD.h"
#include "CmMatrix34.h"

using namespace physx;
using namespace Gu;
using namespace Cm;
using namespace aos;

#define	BATCH_TRIANGLE_NUMBER	32u

struct MTDTriangle : public PxTriangle
{
public:
	PxU8 extraTriData;//active edge flag data
};

struct MeshMTDGenerationCallback : MeshHitCallback<PxGeomRaycastHit>
{
public:
	
	PxArray<PxU32>&	container;

	MeshMTDGenerationCallback(PxArray<PxU32>& tempContainer)
	:	MeshHitCallback<PxGeomRaycastHit>(CallbackMode::eMULTIPLE), container(tempContainer)
	{
	}

	virtual PxAgain processHit(
		const PxGeomRaycastHit& hit, const PxVec3&, const PxVec3&, const PxVec3&, PxReal&, const PxU32*)
	{
		container.pushBack(hit.faceIndex);

		return true;
	}

	void operator=(const MeshMTDGenerationCallback&) {}
};

static bool getMTDPerTriangle(const MeshPersistentContact* manifoldContacts, const PxU32 numContacts, const PxU32 triangleIndex, Vec3V& normal, Vec3V& closestA, Vec3V& closestB, PxU32& faceIndex, FloatV& deepestPen)
{
	FloatV deepest = V4GetW(manifoldContacts[0].mLocalNormalPen);
	PxU32 index = 0;
	for(PxU32 k=1; k<numContacts; ++k)
	{
		const FloatV pen = V4GetW(manifoldContacts[k].mLocalNormalPen);
		if(FAllGrtr(deepest, pen))
		{
			deepest = pen;
			index = k;
		}
	}

	if(FAllGrtr(deepestPen, deepest))
	{
		PX_ASSERT(triangleIndex == manifoldContacts[index].mFaceIndex);
		faceIndex = triangleIndex;
		deepestPen = deepest;
		normal = Vec3V_From_Vec4V(manifoldContacts[index].mLocalNormalPen);
		closestA = manifoldContacts[index].mLocalPointB;
		closestB = manifoldContacts[index].mLocalPointA;
		return true;
	}

	return false;
}

static void midPhaseQuery(const PxTriangleMeshGeometry& meshGeom, const PxTransform& pose, const Box& bound, PxArray<PxU32>& tempContainer)
{
	TriangleMesh* meshData = static_cast<TriangleMesh*>(meshGeom.triangleMesh);

	Box vertexSpaceBox;
	computeVertexSpaceOBB(vertexSpaceBox, bound, pose, meshGeom.scale);

	MeshMTDGenerationCallback callback(tempContainer);
	Midphase::intersectOBB(meshData, vertexSpaceBox, callback, true);
}

// PT: TODO: refactor with EntityReportContainerCallback
struct MidPhaseQueryLocalReport : OverlapReport
{
	MidPhaseQueryLocalReport(PxArray<PxU32>& _container) : container(_container)
	{

	}
	virtual bool reportTouchedTris(PxU32 nb, const PxU32* indices)
	{
		for(PxU32 i=0; i<nb; i++)
			container.pushBack(indices[i]);
		return true;
	}

	PxArray<PxU32>& container;

private:
	MidPhaseQueryLocalReport operator=(MidPhaseQueryLocalReport& report);
};

static void midPhaseQuery(const HeightFieldUtil& hfUtil, const PxTransform& pose, const PxBounds3& bounds, PxArray<PxU32>& tempContainer)
{
	MidPhaseQueryLocalReport localReport(tempContainer);
	hfUtil.overlapAABBTriangles(pose, bounds, localReport);
}

static bool calculateMTD(	const CapsuleV& capsuleV, const FloatVArg inflatedRadiusV, const bool isDoubleSide, const MTDTriangle* triangles, const PxU32 nbTriangles, const PxU32 startIndex, MeshPersistentContact* manifoldContacts, 
							PxU32& numContacts, Vec3V& normal, Vec3V& closestA, Vec3V& closestB, PxU32& faceIndex, FloatV& mtd)
{
	const FloatV zero = FZero();
	bool hadContacts = false;
	FloatV deepestPen = mtd;

	for(PxU32 j=0; j<nbTriangles; ++j)
	{
		numContacts = 0;

		const MTDTriangle& curTri = triangles[j];
		TriangleV triangleV;
		triangleV.verts[0] = V3LoadU(curTri.verts[0]);
		triangleV.verts[1] = V3LoadU(curTri.verts[1]);
		triangleV.verts[2] = V3LoadU(curTri.verts[2]);
		const PxU8 triFlag = curTri.extraTriData;

		const Vec3V triangleNormal = triangleV.normal();
		const Vec3V v = V3Sub(capsuleV.getCenter(), triangleV.verts[0]);
		const FloatV dotV = V3Dot(triangleNormal, v);

		// Backface culling
		const bool culled = !isDoubleSide && (FAllGrtr(zero, dotV));
		if(culled)
			continue;
		
		PCMCapsuleVsMeshContactGeneration::processTriangle(triangleV, j+startIndex, capsuleV, inflatedRadiusV, triFlag, manifoldContacts, numContacts);

		if(numContacts ==0)
			continue;

		hadContacts = true;

		getMTDPerTriangle(manifoldContacts, numContacts, j + startIndex, normal, closestA, closestB, faceIndex, deepestPen);
	}

	mtd = deepestPen;
	return hadContacts;
}

static PX_FORCE_INLINE bool finalizeMTD(PxGeomSweepHit& hit, const Vec3VArg translationV, const Vec3VArg posV, PxU32 triangleIndex, bool foundInitial)
{
	if(foundInitial)
	{
		const FloatV translationF = V3Length(translationV);
		const FloatV distV = FNeg(translationF);

		const BoolV con = FIsGrtr(translationF, FZero());
		const Vec3V nrm = V3Sel(con, V3ScaleInv(translationV, translationF), V3Zero());
	
		FStore(distV, &hit.distance);
		V3StoreU(posV, hit.position);
		V3StoreU(nrm, hit.normal);
		hit.faceIndex = triangleIndex;
	}
	return foundInitial;
}

bool physx::Gu::computeCapsule_TriangleMeshMTD(	const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose, CapsuleV& capsuleV, PxReal inflatedRadius, 
												bool isDoubleSided, PxGeomSweepHit& hit)
{
	TriangleMesh* triMesh = static_cast<TriangleMesh*>(triMeshGeom.triangleMesh);
	const PxU8* extraTrigData = triMesh->getExtraTrigData();
	const bool flipsNormal = triMeshGeom.scale.hasNegativeDeterminant();
	
	//inflated the capsule by 15% in case of some disagreement between sweep and mtd calculation. If sweep said initial overlap, but mtd has a positive separation,
	//we are still be able to return a valid normal but we should zero the distance.
	const FloatV inflatedRadiusV = FLoad(inflatedRadius*1.15f);
	
	const PxMat34 vertexToWorldSkew = pose * triMeshGeom.scale;

	const Vec3V zeroV = V3Zero();
	Vec3V closestA = zeroV, closestB = zeroV, normal = zeroV;

	/////

	MeshPersistentContact manifoldContacts[64];   
	PxU32 numContacts = 0;

	PxArray<PxU32> tempContainer;
	tempContainer.reserve(128);

	PxU32 triangleIndex = 0xfffffff;
	Vec3V translation = zeroV;
	bool foundInitial = false;
	const PxU32 iterations = 4;

	/////

	for(PxU32 i=0; i<iterations; ++i)
	{
		tempContainer.forceSize_Unsafe(0);
		{
			Capsule inflatedCapsule;
			V3StoreU(capsuleV.p0, inflatedCapsule.p0);
			V3StoreU(capsuleV.p1, inflatedCapsule.p1);
			inflatedCapsule.radius = inflatedRadius;

			Box capsuleBox;
			computeBoxAroundCapsule(inflatedCapsule, capsuleBox);

			midPhaseQuery(triMeshGeom, pose, capsuleBox, tempContainer);
		}

		// Get results
		const PxU32 nbTriangles = tempContainer.size();
		if(!nbTriangles)
			break;

		FloatV mtd;
		{
			bool hadContacts = false;

			const PxU32 nbBatches = (nbTriangles + BATCH_TRIANGLE_NUMBER - 1)/BATCH_TRIANGLE_NUMBER;
			mtd = FMax();
			MTDTriangle triangles[BATCH_TRIANGLE_NUMBER];
			for(PxU32 a = 0; a < nbBatches; ++a)
			{
				const PxU32 startIndex = a * BATCH_TRIANGLE_NUMBER;
				const PxU32 nbTrigs = PxMin(nbTriangles - startIndex, BATCH_TRIANGLE_NUMBER);
				for(PxU32 k=0; k<nbTrigs; k++)
				{
					//triangle world space
					const PxU32 currentTriangleIndex = tempContainer[startIndex+k];
					triMesh->computeWorldTriangle(triangles[k], currentTriangleIndex, vertexToWorldSkew, flipsNormal);
					triangles[k].extraTriData = getConvexEdgeFlags(extraTrigData, currentTriangleIndex);
				}

				//ML: mtd has back face culling, so if the capsule's center is below the triangle, we won't generate any contacts
				hadContacts = calculateMTD(capsuleV, inflatedRadiusV, isDoubleSided, triangles, nbTrigs, startIndex, manifoldContacts, numContacts, normal, closestA, closestB, triangleIndex, mtd) || hadContacts;
			}

			if(!hadContacts)
				break;

			triangleIndex = tempContainer[triangleIndex];
			foundInitial = true;
		}

		//move the capsule to depenetrate it
		
		const FloatV distV = FSub(mtd, capsuleV.radius);
		if(FAllGrtr(FZero(), distV))
		{
			Vec3V center = capsuleV.getCenter();
			const Vec3V t = V3Scale(normal, distV);
			translation = V3Sub(translation, t);
			center = V3Sub(center, t);
			capsuleV.setCenter(center);
		}
		else
		{
			if(i == 0)
			{
				//First iteration so keep this normal
				hit.distance = 0.0f;
				V3StoreU(closestA, hit.position);
				V3StoreU(normal, hit.normal);
				hit.faceIndex = triangleIndex;
				return true;
			}
			break;
		}
	}

	return finalizeMTD(hit, translation, closestA, triangleIndex, foundInitial);
}


bool physx::Gu::computeCapsule_HeightFieldMTD(const PxHeightFieldGeometry& heightFieldGeom, const PxTransform& pose, CapsuleV& capsuleV, PxReal inflatedRadius, bool isDoubleSided, PxGeomSweepHit& hit)
{
	//inflated the capsule by 1% in case of some disagreement between sweep and mtd calculation.If sweep said initial overlap, but mtd has a positive separation,
	//we are still be able to return a valid normal but we should zero the distance.
	const FloatV inflatedRadiusV = FLoad(inflatedRadius*1.01f);  
	
	const HeightFieldUtil hfUtil(heightFieldGeom);

	const Vec3V zeroV = V3Zero();
	Vec3V closestA = zeroV, closestB = zeroV, normal = zeroV;

	/////

	MeshPersistentContact manifoldContacts[64]; 
	PxU32 numContacts = 0;

	PxArray<PxU32> tempContainer;
	tempContainer.reserve(128);

	PxU32 triangleIndex = 0xfffffff;
	Vec3V translation = zeroV;
	bool foundInitial = false;
	const PxU32 iterations = 4;

	/////

	for(PxU32 i=0; i<iterations; ++i)
	{
		tempContainer.forceSize_Unsafe(0);
		{
			Capsule inflatedCapsule;
			V3StoreU(capsuleV.p0, inflatedCapsule.p0);
			V3StoreU(capsuleV.p1, inflatedCapsule.p1);
			inflatedCapsule.radius = inflatedRadius;

			Box capsuleBox;
			computeBoxAroundCapsule(inflatedCapsule, capsuleBox);

			const PxTransform capsuleBoxTransform = capsuleBox.getTransform();
			const PxBounds3 bounds = PxBounds3::poseExtent(capsuleBoxTransform, capsuleBox.extents);

			midPhaseQuery(hfUtil, pose, bounds, tempContainer);
		}

		// Get results
		const PxU32 nbTriangles = tempContainer.size();
		if(!nbTriangles)
			break;

		FloatV mtd;
		{
			bool hadContacts = false;

			const PxU32 nbBatches = (nbTriangles + BATCH_TRIANGLE_NUMBER - 1)/BATCH_TRIANGLE_NUMBER;
			mtd = FMax();
			MTDTriangle triangles[BATCH_TRIANGLE_NUMBER];
			for(PxU32 a = 0; a < nbBatches; ++a)
			{
				const PxU32 startIndex = a * BATCH_TRIANGLE_NUMBER;
				const PxU32 nbTrigs = PxMin(nbTriangles - startIndex, BATCH_TRIANGLE_NUMBER);
				for(PxU32 k=0; k<nbTrigs; k++)
				{
					//triangle vertex space
					const PxU32 currentTriangleIndex = tempContainer[startIndex+k];
					hfUtil.getTriangle(pose, triangles[k], NULL, NULL, currentTriangleIndex, true);
					triangles[k].extraTriData = ETD_CONVEX_EDGE_ALL;
				}

				//ML: mtd has back face culling, so if the capsule's center is below the triangle, we won't generate any contacts
				hadContacts = calculateMTD(capsuleV, inflatedRadiusV, isDoubleSided, triangles, nbTrigs, startIndex, manifoldContacts, numContacts, normal, closestA, closestB, triangleIndex, mtd) || hadContacts;
			}

			if(!hadContacts)
				break;

			triangleIndex = tempContainer[triangleIndex];
			foundInitial = true;
		}

		const FloatV distV = FSub(mtd, capsuleV.radius);
		if(FAllGrtr(FZero(), distV))
		{
			//move the capsule to depenetrate it
			Vec3V center = capsuleV.getCenter();
			const Vec3V t = V3Scale(normal, distV);
			translation = V3Sub(translation, t);
			center = V3Sub(center, t);
			capsuleV.setCenter(center);
		}
		else
		{
			if(i == 0)
			{
				//First iteration so keep this normal
				hit.distance = 0.0f;
				V3StoreU(closestA, hit.position);
				V3StoreU(normal, hit.normal);
				hit.faceIndex = triangleIndex;
				return true;
			}
			break;
		}
	}

	return finalizeMTD(hit, translation, closestA, triangleIndex, foundInitial);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static bool calculateMTD(	const PolygonalData& polyData, const SupportLocal* polyMap, const PxTransformV& convexTransform, const PxMatTransformV& meshToConvex, bool isDoubleSided, const FloatVArg inflation, const MTDTriangle* triangles, PxU32 nbTriangles, PxU32 startIndex, 
							MeshPersistentContact* manifoldContacts, PxU32& numContacts, Vec3V& normal, Vec3V& closestA, Vec3V& closestB, PxU32& faceIndex, FloatV& mtd)
{
	bool hadContacts = false;
	FloatV deepestPen = mtd;
	
	for(PxU32 j=0; j<nbTriangles; ++j)
	{
		numContacts = 0;
		const MTDTriangle& curTri = triangles[j];
		const PxU8 triFlag = curTri.extraTriData;
		
		PCMConvexVsMeshContactGeneration::processTriangle(polyData, polyMap, curTri.verts, j+startIndex, triFlag, inflation, isDoubleSided, convexTransform, meshToConvex, manifoldContacts, numContacts);

		if(numContacts ==0)
			continue;

		hadContacts = true;
		getMTDPerTriangle(manifoldContacts, numContacts, j+startIndex, normal, closestA, closestB, faceIndex, deepestPen);
	}

	mtd = deepestPen;

	return hadContacts;
}

bool physx::Gu::computeBox_TriangleMeshMTD(const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose, const Box& _box, const PxTransform& boxTransform, PxReal inflation, bool isDoubleSided, PxGeomSweepHit& hit)
{
	TriangleMesh* triMesh = static_cast<TriangleMesh*>(triMeshGeom.triangleMesh);
	const PxU8* extraTrigData = triMesh->getExtraTrigData();
	const bool flipsNormal = triMeshGeom.scale.hasNegativeDeterminant();

	const Vec3V zeroV = V3Zero();
	Vec3V closestA = zeroV, closestB = zeroV, normal = zeroV;
	Vec3V worldNormal = zeroV, worldContactA = zeroV;//, worldContactB = zeroV;

	Box box = _box;
	
	const QuatV q0 = QuatVLoadU(&boxTransform.q.x);
	const Vec3V p0 = V3LoadU(&boxTransform.p.x);

	const Vec3V boxExtents = V3LoadU(box.extents);
	const FloatV minMargin = CalculateMTDBoxMargin(boxExtents);
	const FloatV inflationV = FAdd(FLoad(inflation), minMargin);
	PxReal boundInflation;
	FStore(inflationV, &boundInflation);

	box.extents += PxVec3(boundInflation);
	const BoxV boxV(zeroV, boxExtents);

	Vec3V boxCenter = V3LoadU(box.center);

	//create the polyData based on the original data
	PolygonalData polyData;
	const PCMPolygonalBox polyBox(_box.extents);
	polyBox.getPolygonalData(&polyData);

	const Mat33V identity = M33Identity();

	const PxMat34 meshToWorldSkew = pose * triMeshGeom.scale;

	PxTransformV boxTransformV(p0, q0);//box
	
	/////

	MeshPersistentContact manifoldContacts[64]; 
	PxU32 numContacts = 0;

	PxArray<PxU32> tempContainer;
	tempContainer.reserve(128);

	PxU32 triangleIndex = 0xfffffff;
	Vec3V translation = zeroV;
	bool foundInitial = false;
	const PxU32 iterations = 4;

	/////

	for(PxU32 i=0; i<iterations; ++i)
	{		
		tempContainer.forceSize_Unsafe(0);
		{
			midPhaseQuery(triMeshGeom, pose, box, tempContainer);
		}

		// Get results
		const PxU32 nbTriangles = tempContainer.size();
		if(!nbTriangles)
			break;

		boxTransformV.p = boxCenter;
		SupportLocalImpl<BoxV> boxMap(boxV, boxTransformV, identity, identity, true);

		boxMap.setShapeSpaceCenterofMass(zeroV);
		// Move to AABB space
		PxMat34 WorldToBox;
		computeWorldToBoxMatrix(WorldToBox, box);
		const PxMat34 meshToBox = WorldToBox*meshToWorldSkew;

		const Mat33V rot(V3LoadU(meshToBox.m.column0), V3LoadU(meshToBox.m.column1), V3LoadU(meshToBox.m.column2));
		const PxMatTransformV meshToConvex(V3LoadU(meshToBox.p), rot);

		FloatV mtd;
		{
			bool hadContacts = false;

			const PxU32 nbBatches = (nbTriangles + BATCH_TRIANGLE_NUMBER - 1)/BATCH_TRIANGLE_NUMBER;
			mtd = FMax();
			MTDTriangle triangles[BATCH_TRIANGLE_NUMBER];
			for(PxU32 a = 0; a < nbBatches; ++a)
			{
				const PxU32 startIndex = a * BATCH_TRIANGLE_NUMBER;
				const PxU32 nbTrigs = PxMin(nbTriangles - startIndex, BATCH_TRIANGLE_NUMBER);
				for(PxU32 k=0; k<nbTrigs; k++)
				{
					//triangle vertex space
					const PxU32 currentTriangleIndex = tempContainer[startIndex+k];
					triMesh->getLocalTriangle(triangles[k], currentTriangleIndex, flipsNormal);
					triangles[k].extraTriData = getConvexEdgeFlags(extraTrigData, currentTriangleIndex);
				}

				//ML: mtd has back face culling, so if the capsule's center is below the triangle, we won't generate any contacts
				hadContacts = calculateMTD(polyData, &boxMap, boxTransformV, meshToConvex, isDoubleSided, inflationV, triangles, nbTrigs, startIndex, manifoldContacts, numContacts, normal, closestA, closestB, triangleIndex, mtd) || hadContacts;
			}
	
			if(!hadContacts)
				break;

			triangleIndex = tempContainer[triangleIndex];
			foundInitial = true;
		}

		const FloatV distV = mtd;
		worldNormal = boxTransformV.rotate(normal);
		worldContactA = boxTransformV.transform(closestA);
		if(FAllGrtr(FZero(), distV))
		{
			const Vec3V t = V3Scale(worldNormal, mtd);
			translation = V3Sub(translation, t);
			boxCenter = V3Sub(boxCenter, t);
			V3StoreU(boxCenter, box.center);
		}
		else
		{
			if(i == 0)
			{
				//First iteration so keep this normal
				hit.distance = 0.0f;
				V3StoreU(worldContactA, hit.position);
				V3StoreU(worldNormal, hit.normal);
				hit.faceIndex = triangleIndex;
				return true;
			}
			break;
		}
	}

	return finalizeMTD(hit, translation, worldContactA, triangleIndex, foundInitial);
}

bool physx::Gu::computeBox_HeightFieldMTD(const PxHeightFieldGeometry& heightFieldGeom, const PxTransform& pose, const Box& _box, const PxTransform& boxTransform, PxReal inflation, bool isDoubleSided, PxGeomSweepHit& hit)
{
	const HeightFieldUtil hfUtil(heightFieldGeom);

	const Vec3V zeroV = V3Zero();
	Vec3V closestA = zeroV, closestB = zeroV, normal = zeroV;
	Vec3V worldNormal = zeroV, worldContactA = zeroV;//, worldContactB = zeroV;

	Box box = _box;
	
	const QuatV q0 = QuatVLoadU(&boxTransform.q.x);
	const Vec3V p0 = V3LoadU(&boxTransform.p.x);

	const Vec3V boxExtents = V3LoadU(box.extents);
	const FloatV minMargin = CalculateMTDBoxMargin(boxExtents);
	const FloatV inflationV = FAdd(FLoad(inflation), minMargin);
	//const FloatV inflationV = FLoad(inflation);

	PxReal boundInflation;
	FStore(inflationV, &boundInflation);
	box.extents += PxVec3(boundInflation);
	
	const BoxV boxV(zeroV, boxExtents);

	Vec3V boxCenter = V3LoadU(box.center);

	//create the polyData based on the original box
	PolygonalData polyData;
	const PCMPolygonalBox polyBox(_box.extents);
	polyBox.getPolygonalData(&polyData);

	const Mat33V identity = M33Identity();

	const Matrix34FromTransform meshToWorldSkew(pose);

	PxTransformV boxTransformV(p0, q0);//box
	
	/////

	MeshPersistentContact manifoldContacts[64]; 
	PxU32 numContacts = 0;

	PxArray<PxU32> tempContainer;
	tempContainer.reserve(128);

	PxU32 triangleIndex = 0xfffffff;
	Vec3V translation = zeroV;
	bool foundInitial = false;
	const PxU32 iterations = 4;

	/////

	for(PxU32 i=0; i<iterations; ++i)
	{
		tempContainer.forceSize_Unsafe(0);
		{
			const PxBounds3 bounds = PxBounds3::poseExtent(box.getTransform(), box.extents);

			midPhaseQuery(hfUtil, pose, bounds, tempContainer);
		}

		// Get results
		const PxU32 nbTriangles = tempContainer.size();
		if(!nbTriangles)
			break;

		boxTransformV.p = boxCenter;
		SupportLocalImpl<BoxV> boxMap(boxV, boxTransformV, identity, identity, true);
		boxMap.setShapeSpaceCenterofMass(zeroV);
		// Move to AABB space
		PxMat34 WorldToBox;
		computeWorldToBoxMatrix(WorldToBox, box);
		const PxMat34 meshToBox = WorldToBox*meshToWorldSkew;

		const Mat33V rot(V3LoadU(meshToBox.m.column0), V3LoadU(meshToBox.m.column1), V3LoadU(meshToBox.m.column2));
		const PxMatTransformV meshToConvex(V3LoadU(meshToBox.p), rot);

		FloatV mtd;
		{
			bool hadContacts = false;

			const PxU32 nbBatches = (nbTriangles + BATCH_TRIANGLE_NUMBER - 1)/BATCH_TRIANGLE_NUMBER;
			mtd = FMax();
			MTDTriangle triangles[BATCH_TRIANGLE_NUMBER];
			for(PxU32 a = 0; a < nbBatches; ++a)
			{
				const PxU32 startIndex = a * BATCH_TRIANGLE_NUMBER;
				const PxU32 nbTrigs = PxMin(nbTriangles - startIndex, BATCH_TRIANGLE_NUMBER);
				for(PxU32 k=0; k<nbTrigs; k++)
				{
					//triangle vertex space
					const PxU32 currentTriangleIndex = tempContainer[startIndex+k];
					hfUtil.getTriangle(pose, triangles[k], NULL, NULL, currentTriangleIndex, false, false);
					triangles[k].extraTriData = ETD_CONVEX_EDGE_ALL;
				}

				//ML: mtd has back face culling, so if the box's center is below the triangle, we won't generate any contacts
				hadContacts = calculateMTD(polyData, &boxMap, boxTransformV, meshToConvex, isDoubleSided, inflationV, triangles, nbTrigs, startIndex, manifoldContacts, numContacts, normal, closestA, closestB, triangleIndex, mtd) || hadContacts;
			}

			if(!hadContacts)
				break;

			triangleIndex = tempContainer[triangleIndex];
			foundInitial = true;
		}

		const FloatV distV = mtd;
		worldNormal = boxTransformV.rotate(normal);
		worldContactA = boxTransformV.transform(closestA);
		if(FAllGrtr(FZero(), distV))
		{
			//worldContactB = boxTransformV.transform(closestB);
			const Vec3V t = V3Scale(worldNormal, mtd);
			translation = V3Sub(translation, t);
			boxCenter = V3Sub(boxCenter, t);
			V3StoreU(boxCenter, box.center);
		}
		else
		{
			if(i == 0)
			{
				//First iteration so keep this normal
				hit.distance = 0.0f;
				V3StoreU(worldContactA, hit.position);
				V3StoreU(worldNormal, hit.normal);
				hit.faceIndex = triangleIndex;
				return true;
			}
			break;
		}
	}

	return finalizeMTD(hit, translation, worldContactA, triangleIndex, foundInitial);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool physx::Gu::computeConvex_TriangleMeshMTD(	const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose, PxReal inflation,
												bool isDoubleSided, PxGeomSweepHit& hit)
{
	const Vec3V zeroV = V3Zero();

	TriangleMesh* triMesh = static_cast<TriangleMesh*>(triMeshGeom.triangleMesh);
	ConvexMesh* cm = static_cast<ConvexMesh*>(convexGeom.convexMesh);
	const PxU8* extraTrigData = triMesh->getExtraTrigData();
	const bool flipsNormal = triMeshGeom.scale.hasNegativeDeterminant();

	ConvexHullData* hullData = &cm->getHull();

	const bool idtScaleConvex = convexGeom.scale.isIdentity();
	
	FastVertex2ShapeScaling convexScaling;
	if(!idtScaleConvex)
		convexScaling.init(convexGeom.scale);

	const PxVec3 _shapeSpaceCenterOfMass = convexScaling * hullData->mCenterOfMass;
	const Vec3V shapeSpaceCenterOfMass = V3LoadU(_shapeSpaceCenterOfMass);

	const QuatV q0 = QuatVLoadU(&convexPose.q.x);
	const Vec3V p0 = V3LoadU(&convexPose.p.x);
	PxTransformV convexTransformV(p0, q0);

	const Vec3V vScale = V3LoadU_SafeReadW(convexGeom.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const QuatV vQuat = QuatVLoadU(&convexGeom.scale.rotation.x);
	const ConvexHullV convexHull(hullData, V3Zero(), vScale, vQuat, idtScaleConvex);
	PX_ALIGN(16, PxU8 convexBuff[sizeof(SupportLocalImpl<ConvexHullV>)]);
	
	const FloatV convexMargin = CalculateMTDConvexMargin(hullData, vScale);
	const FloatV inflationV = FAdd(FLoad(inflation), convexMargin);
	PxReal boundInflation;
	FStore(inflationV, &boundInflation);

	Vec3V closestA = zeroV, closestB = zeroV, normal = zeroV;

	const PxMat34 meshToWorldSkew = pose * triMeshGeom.scale;

	PolygonalData polyData;
	getPCMConvexData(convexHull, idtScaleConvex, polyData);
	
	Vec3V center = p0;
	PxTransform tempConvexPose = convexPose;
	Vec3V worldNormal = zeroV, worldContactA = zeroV;//, worldContactB = zeroV;
	
	/////

	MeshPersistentContact manifoldContacts[64]; 
	PxU32 numContacts = 0;

	PxArray<PxU32> tempContainer;
	tempContainer.reserve(128);

	PxU32 triangleIndex = 0xfffffff;
	Vec3V translation = zeroV;
	bool foundInitial = false;
	const PxU32 iterations = 2;	// PT: TODO: why 2 here instead of 4?

	/////

	for(PxU32 i=0; i<iterations; ++i)
	{
		tempContainer.forceSize_Unsafe(0);
		SupportLocal* convexMap;
		{
			//ML:: construct convex hull data
			V3StoreU(center, tempConvexPose.p);
			convexTransformV.p = center;

			convexMap = idtScaleConvex ?	static_cast<SupportLocal*>(PX_PLACEMENT_NEW(convexBuff, SupportLocalImpl<ConvexHullNoScaleV>)(static_cast<const ConvexHullNoScaleV&>(convexHull), convexTransformV, convexHull.vertex2Shape, convexHull.shape2Vertex, idtScaleConvex)) : 
											static_cast<SupportLocal*>(PX_PLACEMENT_NEW(convexBuff, SupportLocalImpl<ConvexHullV>)(convexHull, convexTransformV, convexHull.vertex2Shape, convexHull.shape2Vertex, idtScaleConvex));

			convexMap->setShapeSpaceCenterofMass(shapeSpaceCenterOfMass);
		
			Box hullOBB;
			computeOBBAroundConvex(hullOBB, convexGeom, cm, tempConvexPose);

			hullOBB.extents += PxVec3(boundInflation);

			midPhaseQuery(triMeshGeom, pose, hullOBB, tempContainer);
		}

		// Get results
		const PxU32 nbTriangles = tempContainer.size();
		if(!nbTriangles)
			break;
	
		// Move to AABB space
		const Matrix34FromTransform worldToConvex(tempConvexPose.getInverse());
		const PxMat34 meshToConvex = worldToConvex*meshToWorldSkew;

		const Mat33V rot(V3LoadU(meshToConvex.m.column0), V3LoadU(meshToConvex.m.column1), V3LoadU(meshToConvex.m.column2));
		const PxMatTransformV meshToConvexV(V3LoadU(meshToConvex.p), rot);

		FloatV mtd;
		{
			bool hadContacts = false;

			const PxU32 nbBatches = (nbTriangles + BATCH_TRIANGLE_NUMBER - 1)/BATCH_TRIANGLE_NUMBER;
			mtd = FMax();
			MTDTriangle triangles[BATCH_TRIANGLE_NUMBER];
			for(PxU32 a = 0; a < nbBatches; ++a)
			{
				const PxU32 startIndex = a * BATCH_TRIANGLE_NUMBER;
				const PxU32 nbTrigs = PxMin(nbTriangles - startIndex, BATCH_TRIANGLE_NUMBER);
				for(PxU32 k=0; k<nbTrigs; k++)
				{
					//triangle vertex space
					const PxU32 currentTriangleIndex = tempContainer[startIndex+k];
					triMesh->getLocalTriangle(triangles[k], currentTriangleIndex, flipsNormal);
					triangles[k].extraTriData = getConvexEdgeFlags(extraTrigData, currentTriangleIndex);
				}

				//ML: mtd has back face culling, so if the capsule's center is below the triangle, we won't generate any contacts
				hadContacts = calculateMTD(polyData, convexMap, convexTransformV, meshToConvexV, isDoubleSided, inflationV, triangles, nbTrigs, startIndex, manifoldContacts, numContacts, normal, closestA, closestB, triangleIndex, mtd) || hadContacts;
			}
	
			if(!hadContacts)
				break;

			triangleIndex = tempContainer[triangleIndex];
			foundInitial = true;
		}

		const FloatV distV = mtd;
		worldNormal = convexTransformV.rotate(normal);
		worldContactA = convexTransformV.transform(closestA);
		if(FAllGrtr(FZero(), distV))
		{
			const Vec3V t = V3Scale(worldNormal, mtd);
			translation = V3Sub(translation, t);
			center = V3Sub(center, t);
		}
		else
		{
			if(i == 0)
			{
				//First iteration so keep this normal
				hit.distance = 0.0f;
				V3StoreU(worldContactA, hit.position);
				V3StoreU(worldNormal, hit.normal);
				hit.faceIndex = triangleIndex;
				return true;
			}
			break;
		}
	}

	return finalizeMTD(hit, translation, worldContactA, triangleIndex, foundInitial);
}

bool physx::Gu::computeConvex_HeightFieldMTD(const PxHeightFieldGeometry& heightFieldGeom, const PxTransform& pose, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose, PxReal inflation, bool isDoubleSided, PxGeomSweepHit& hit)
{
	const HeightFieldUtil hfUtil(heightFieldGeom);
	
	const Vec3V zeroV = V3Zero();
	
	ConvexMesh* cm = static_cast<ConvexMesh*>(convexGeom.convexMesh);

	ConvexHullData* hullData = &cm->getHull();

	const bool idtScaleConvex = convexGeom.scale.isIdentity();
	
	FastVertex2ShapeScaling convexScaling;
	if(!idtScaleConvex)
		convexScaling.init(convexGeom.scale);

	const PxVec3 _shapeSpaceCenterOfMass = convexScaling * hullData->mCenterOfMass;
	const Vec3V shapeSpaceCenterOfMass = V3LoadU(_shapeSpaceCenterOfMass);

	const QuatV q0 = QuatVLoadU(&convexPose.q.x);
	const Vec3V p0 = V3LoadU(&convexPose.p.x);
	PxTransformV convexTransformV(p0, q0);

	const Vec3V vScale = V3LoadU_SafeReadW(convexGeom.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const QuatV vQuat = QuatVLoadU(&convexGeom.scale.rotation.x);
	const ConvexHullV convexHull(hullData, zeroV, vScale, vQuat, idtScaleConvex);
	PX_ALIGN(16, PxU8 convexBuff[sizeof(SupportLocalImpl<ConvexHullV>)]);

	const FloatV convexMargin = CalculateMTDConvexMargin(hullData, vScale);
	const FloatV inflationV = FAdd(FLoad(inflation), convexMargin);
	PxReal boundInflation;
	FStore(inflationV, &boundInflation);

	Vec3V closestA = zeroV, closestB = zeroV, normal = zeroV;
	Vec3V worldNormal = zeroV, worldContactA = zeroV;//, worldContactB = zeroV;

	PolygonalData polyData;
	getPCMConvexData(convexHull, idtScaleConvex, polyData);

	Vec3V center = p0;
	PxTransform tempConvexPose = convexPose;
	const Matrix34FromTransform meshToWorldSkew(pose);
	
	/////

	MeshPersistentContact manifoldContacts[64]; 
	PxU32 numContacts = 0;

	PxArray<PxU32> tempContainer;
	tempContainer.reserve(128);

	PxU32 triangleIndex = 0xfffffff;
	Vec3V translation = zeroV;
	bool foundInitial = false;
	const PxU32 iterations = 2;	// PT: TODO: why 2 here instead of 4?

	/////

	for(PxU32 i=0; i<iterations; ++i)
	{
		tempContainer.forceSize_Unsafe(0);
		SupportLocal* convexMap;
		{
			//ML:: construct convex hull data
			V3StoreU(center, tempConvexPose.p);
			convexTransformV.p = center;

			convexMap = idtScaleConvex ?	static_cast<SupportLocal*>(PX_PLACEMENT_NEW(convexBuff, SupportLocalImpl<ConvexHullNoScaleV>)(static_cast<const ConvexHullNoScaleV&>(convexHull), convexTransformV, convexHull.vertex2Shape, convexHull.shape2Vertex, idtScaleConvex)) : 
											static_cast<SupportLocal*>(PX_PLACEMENT_NEW(convexBuff, SupportLocalImpl<ConvexHullV>)(convexHull, convexTransformV, convexHull.vertex2Shape, convexHull.shape2Vertex, idtScaleConvex));

			convexMap->setShapeSpaceCenterofMass(shapeSpaceCenterOfMass);
		
			Box hullOBB;
			computeOBBAroundConvex(hullOBB, convexGeom, cm, tempConvexPose);

			hullOBB.extents += PxVec3(boundInflation);

			const PxBounds3 bounds = PxBounds3::basisExtent(hullOBB.center, hullOBB.rot, hullOBB.extents);

			midPhaseQuery(hfUtil, pose, bounds, tempContainer);
		}

		// Get results
		const PxU32 nbTriangles = tempContainer.size();
		if(!nbTriangles)
			break;
	
		// Move to AABB space
		const Matrix34FromTransform worldToConvex(tempConvexPose.getInverse());
		const PxMat34 meshToConvex = worldToConvex*meshToWorldSkew;

		const Mat33V rot(V3LoadU(meshToConvex.m.column0), V3LoadU(meshToConvex.m.column1), V3LoadU(meshToConvex.m.column2));
		const PxMatTransformV meshToConvexV(V3LoadU(meshToConvex.p), rot);

		FloatV mtd;
		{
			bool hadContacts = false;

			const PxU32 nbBatches = (nbTriangles + BATCH_TRIANGLE_NUMBER - 1)/BATCH_TRIANGLE_NUMBER;
			mtd = FMax();
			MTDTriangle triangles[BATCH_TRIANGLE_NUMBER];
			for(PxU32 a = 0; a < nbBatches; ++a)
			{
				const PxU32 startIndex = a * BATCH_TRIANGLE_NUMBER;   
				const PxU32 nbTrigs = PxMin(nbTriangles - startIndex, BATCH_TRIANGLE_NUMBER);
				for(PxU32 k=0; k<nbTrigs; k++)
				{
					//triangle vertex space
					const PxU32 currentTriangleIndex = tempContainer[startIndex+k];
					hfUtil.getTriangle(pose, triangles[k], NULL, NULL, currentTriangleIndex, false, false);
					triangles[k].extraTriData = ETD_CONVEX_EDGE_ALL;
				}

				//ML: mtd has back face culling, so if the capsule's center is below the triangle, we won't generate any contacts
				hadContacts = calculateMTD(polyData, convexMap, convexTransformV, meshToConvexV, isDoubleSided, inflationV, triangles, nbTrigs, startIndex, manifoldContacts, numContacts, normal, closestA, closestB, triangleIndex, mtd) || hadContacts;
			}
	
			if(!hadContacts)
				break;

			triangleIndex = tempContainer[triangleIndex];
			foundInitial = true;
		}

		const FloatV distV = mtd;
		worldNormal = convexTransformV.rotate(normal);
		worldContactA = convexTransformV.transform(closestA);
		if(FAllGrtr(FZero(), distV))
		{
			const Vec3V t = V3Scale(worldNormal, mtd);
			translation = V3Sub(translation, t);
			center = V3Sub(center, t);
		}
		else
		{
			if(i == 0)
			{
				//First iteration so keep this normal
				hit.distance = 0.0f;
				V3StoreU(worldContactA, hit.position);
				V3StoreU(worldNormal, hit.normal);
				hit.faceIndex = triangleIndex;
				return true;
			}
			break;
		}
	}

	return finalizeMTD(hit, translation, worldContactA, triangleIndex, foundInitial);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool physx::Gu::computeSphere_SphereMTD(const Sphere& sphere0, const Sphere& sphere1, PxGeomSweepHit& hit)
{
	const PxVec3 delta = sphere1.center - sphere0.center;
	const PxReal d2 = delta.magnitudeSquared();
	const PxReal radiusSum = sphere0.radius + sphere1.radius;

	const PxReal d = manualNormalize(hit.normal, delta, d2);
	hit.distance = d - radiusSum;
	hit.position = sphere0.center + hit.normal * sphere0.radius;
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool physx::Gu::computeSphere_CapsuleMTD( const Sphere& sphere, const Capsule& capsule, PxGeomSweepHit& hit)
{
	const PxReal radiusSum = sphere.radius + capsule.radius;

	PxReal u;
	distancePointSegmentSquared(capsule, sphere.center, &u);

	const PxVec3 normal = capsule.getPointAt(u) -  sphere.center;
	
	const PxReal lenSq = normal.magnitudeSquared();
	const PxF32 d = manualNormalize(hit.normal, normal, lenSq);
	hit.distance = d - radiusSum;
	hit.position = sphere.center + hit.normal * sphere.radius;
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool physx::Gu::computeCapsule_CapsuleMTD(const Capsule& capsule0, const Capsule& capsule1, PxGeomSweepHit& hit)
{
	PxReal s,t;
	distanceSegmentSegmentSquared(capsule0, capsule1, &s, &t);

	const PxReal radiusSum = capsule0.radius + capsule1.radius;

	const PxVec3 pointAtCapsule0 = capsule0.getPointAt(s);
	const PxVec3 pointAtCapsule1 = capsule1.getPointAt(t);

	const PxVec3 normal = pointAtCapsule0 - pointAtCapsule1;
	const PxReal lenSq = normal.magnitudeSquared();
	const PxF32 len = manualNormalize(hit.normal, normal, lenSq);
	hit.distance = len - radiusSum;
	hit.position = pointAtCapsule1 + hit.normal * capsule1.radius;
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool physx::Gu::computePlane_CapsuleMTD(const PxPlane& plane, const Capsule& capsule, PxGeomSweepHit& hit)
{
	const PxReal d0 = plane.distance(capsule.p0);
	const PxReal d1 = plane.distance(capsule.p1);
	PxReal dmin;
	PxVec3 point;
	if(d0 < d1)
	{
		dmin = d0;
		point = capsule.p0;
	}
	else
	{
		dmin = d1;
		point = capsule.p1;
	}

	hit.normal		= plane.n;
	hit.distance	= dmin - capsule.radius;
	hit.position	= point - hit.normal * dmin;
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool physx::Gu::computePlane_BoxMTD(const PxPlane& plane, const Box& box, PxGeomSweepHit& hit)
{
	PxVec3 pts[8];
	box.computeBoxPoints(pts);

	PxReal dmin = plane.distance(pts[0]);
	PxU32 index = 0;
	for(PxU32 i=1;i<8;i++)
	{
		const PxReal d = plane.distance(pts[i]);
		if(dmin > d)
		{
			index = i;
			dmin = d;
		}
	}
	hit.normal		= plane.n;
	hit.distance	= dmin;
	hit.position	= pts[index] - plane.n*dmin;
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool physx::Gu::computePlane_ConvexMTD(const PxPlane& plane, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose, PxGeomSweepHit& hit)
{
	const ConvexMesh* convexMesh = static_cast<const ConvexMesh*>(convexGeom.convexMesh);
	const FastVertex2ShapeScaling convexScaling(convexGeom.scale);
	PxU32 nbVerts = convexMesh->getNbVerts();
	const PxVec3* PX_RESTRICT verts = convexMesh->getVerts();

	PxVec3 worldPointMin = convexPose.transform(convexScaling * verts[0]);
	PxReal dmin = plane.distance(worldPointMin);
	for(PxU32 i=1;i<nbVerts;i++)
	{
		const PxVec3 worldPoint = convexPose.transform(convexScaling * verts[i]);
		const PxReal d = plane.distance(worldPoint);
		if(dmin > d)
		{
			dmin = d;
			worldPointMin = worldPoint;
		}
	}

	hit.normal		= plane.n;
	hit.distance	= dmin;
	hit.position	= worldPointMin - plane.n * dmin;
	return true;
}
