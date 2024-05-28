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

#include "GuPCMContactGen.h"
#include "GuPCMShapeConvex.h"
#include "common/PxRenderOutput.h"
#include "GuPCMContactGenUtil.h"
#include "foundation/PxVecMath.h"
#include "foundation/PxAlloca.h"
#include "GuVecCapsule.h"
#include "GuVecBox.h"

using namespace physx;
using namespace Gu;
using namespace aos;

//ML: this function is shared by the sphere/capsule vs convex hulls full contact gen, capsule in the local space of polyData
static bool testPolyDataAxis(const CapsuleV& capsule, const PolygonalData& polyData, const SupportLocal* map, const FloatVArg contactDist, FloatV& minOverlap, Vec3V& separatingAxis)
{
	FloatV _minOverlap = FMax();//minOverlap;
	FloatV min0, max0;
	FloatV min1, max1;
	Vec3V tempAxis = V3UnitY();

	//capsule in the local space of polyData
	for(PxU32 i=0; i<polyData.mNbPolygons; ++i)
	{
		const HullPolygonData& polygon = polyData.mPolygons[i];

		const Vec3V minVert = V3LoadU_SafeReadW(polyData.mVerts[polygon.mMinIndex]);	// PT: safe because of the way vertex memory is allocated in ConvexHullData
		const FloatV planeDist = FLoad(polygon.mPlane.d);
		const Vec3V vertexSpacePlaneNormal = V3LoadU_SafeReadW(polygon.mPlane.n);	// PT: safe because 'd' follows 'n' in the plane class

		//transform plane n to shape space
		const Vec3V shapeSpacePlaneNormal = M33TrnspsMulV3(map->shape2Vertex, vertexSpacePlaneNormal);

		const FloatV magnitude = FRecip(V3Length(shapeSpacePlaneNormal));
		//normalize shape space normal
		const Vec3V planeN = V3Scale(shapeSpacePlaneNormal, magnitude);
		//ML::use this to avoid LHS
		min0 = FMul(V3Dot(vertexSpacePlaneNormal, minVert), magnitude);
		max0 = FMul(FNeg(planeDist), magnitude);

		const FloatV tempMin = V3Dot(capsule.p0, planeN);
		const FloatV tempMax = V3Dot(capsule.p1, planeN);
		min1 = FMin(tempMin, tempMax);
		max1 = FMax(tempMin, tempMax);

		min1 = FSub(min1, capsule.radius);
		max1 = FAdd(max1, capsule.radius);

		const BoolV con = BOr(FIsGrtr(min1, FAdd(max0, contactDist)), FIsGrtr(min0, FAdd(max1, contactDist)));

		if(BAllEqTTTT(con))
			return false;

		const FloatV tempOverlap = FSub(max0, min1);

		if(FAllGrtr(_minOverlap, tempOverlap))
		{
			_minOverlap = tempOverlap;
			tempAxis = planeN;
		}
	}

	separatingAxis = tempAxis;
	minOverlap = _minOverlap;

	return true;
}  

//ML: this function is shared by sphere/capsule. Point a and direction d need to be in the local space of polyData
static bool intersectRayPolyhedron(const Vec3VArg a, const Vec3VArg dir, const PolygonalData& polyData, FloatV& tEnter, FloatV& tExit)
{
	const FloatV zero = FZero();
	const FloatV eps = FLoad(1e-7f);
	FloatV tFirst = zero;
	FloatV tLast= FMax();

	for(PxU32 k=0; k<polyData.mNbPolygons; ++k)
	{
		const HullPolygonData& data = polyData.mPolygons[k];
		const Vec3V n = V3LoadU_SafeReadW(data.mPlane.n);	// PT: safe because 'd' follows 'n' in the plane class

		FloatV d = FLoad(data.mPlane.d);
	
		const FloatV denominator = V3Dot(n, dir);
		const FloatV distToPlane = FAdd(V3Dot(n, a), d);
		
		if(FAllGrtr(eps, FAbs(denominator)))
		{
			if(FAllGrtr(distToPlane, zero))
				return false;
		}
		else
		{
			FloatV tTemp = FNeg(FDiv(distToPlane, denominator));

			//ML: denominator < 0 means the ray is entering halfspace; denominator > 0 means the ray is exiting halfspace
			const BoolV con = FIsGrtr(zero, denominator);
			const BoolV con0 = FIsGrtr(tTemp, tFirst); 
			const BoolV con1 = FIsGrtr(tLast, tTemp);
	
			tFirst = FSel(BAnd(con, con0), tTemp, tFirst);
			tLast = FSel(BAndNot(con1, con), tTemp, tLast);
		}
			
		if(FAllGrtr(tFirst, tLast))
			return false;
	}

	//calculate the intersect p in the local space
	tEnter = tFirst;
	tExit = tLast;
	
	return true;
}

//Precompute the convex data
//     7+------+6			0 = ---
//     /|     /|			1 = +--
//    / |    / |			2 = ++-
//   / 4+---/--+5			3 = -+-
// 3+------+2 /    y   z	4 = --+
//  | /    | /     |  /		5 = +-+
//  |/     |/      |/		6 = +++
// 0+------+1      *---x	7 = -++

static bool testSATCapsulePoly(const CapsuleV& capsule, const PolygonalData& polyData, const SupportLocal* map, const FloatVArg contactDist, FloatV& minOverlap, Vec3V& separatingAxis)
{
	FloatV _minOverlap = FMax();//minOverlap;
	FloatV min0, max0;
	FloatV min1, max1;
	Vec3V tempAxis = V3UnitY();
	const FloatV eps = FEps();

	//ML: project capsule to polydata axis 
	if(!testPolyDataAxis(capsule, polyData, map, contactDist, _minOverlap, tempAxis))
		return false;

	const Vec3V capsuleAxis = V3Sub(capsule.p1, capsule.p0);
	
	for(PxU32 i=0;i<polyData.mNbPolygons;i++)
	{
		const HullPolygonData& polygon = polyData.mPolygons[i];
		const PxU8* inds1 = polyData.mPolygonVertexRefs + polygon.mVRef8;

		for(PxU32 lStart = 0, lEnd =PxU32(polygon.mNbVerts-1); lStart<polygon.mNbVerts; lEnd = lStart++)
		{
			//in the vertex space
			const Vec3V p10 = V3LoadU_SafeReadW(polyData.mVerts[inds1[lStart]]);	// PT: safe because of the way vertex memory is allocated in ConvexHullData
			const Vec3V p11 = V3LoadU_SafeReadW(polyData.mVerts[inds1[lEnd]]);		// PT: safe because of the way vertex memory is allocated in ConvexHullData
				
			const Vec3V vertexSpaceV = V3Sub(p11, p10);

			//transform v to shape space
			const Vec3V shapeSpaceV = M33TrnspsMulV3(map->shape2Vertex, vertexSpaceV);
			
			const Vec3V dir = V3Cross(capsuleAxis, shapeSpaceV);
			const FloatV lenSq = V3Dot(dir, dir);
			if(FAllGrtr(eps, lenSq))
				continue;
			const Vec3V normal = V3ScaleInv(dir, FSqrt(lenSq));

			map->doSupport(normal, min0, max0);

			const FloatV tempMin = V3Dot(capsule.p0, normal);
			const FloatV tempMax = V3Dot(capsule.p1, normal);
			min1 = FMin(tempMin, tempMax);
			max1 = FMax(tempMin, tempMax);

			min1 = FSub(min1, capsule.radius);
			max1 = FAdd(max1, capsule.radius);

			const BoolV con = BOr(FIsGrtr(min1, FAdd(max0, contactDist)), FIsGrtr(min0, FAdd(max1, contactDist)));

			if(BAllEqTTTT(con))
				return false;
	
			const FloatV tempOverlap = FSub(max0, min1);

			if(FAllGrtr(_minOverlap, tempOverlap))
			{
				_minOverlap = tempOverlap;
				tempAxis = normal;
			}
		}
	}
		
	separatingAxis = tempAxis;
	minOverlap = _minOverlap;

	return true;
}

static void generatedCapsuleBoxFaceContacts(const CapsuleV& capsule, const PolygonalData& polyData, const HullPolygonData& referencePolygon, const SupportLocal* map, const PxMatTransformV& aToB, PersistentContact* manifoldContacts, PxU32& numContacts, 
	const FloatVArg contactDist, const Vec3VArg normal)
{
	const FloatV radius = FAdd(capsule.radius, contactDist);

	//calculate the intersect point of ray to plane
	const Vec3V planeNormal = V3Normalize(M33TrnspsMulV3(map->shape2Vertex, V3LoadU(referencePolygon.mPlane.n)));
	const PxU8* inds = polyData.mPolygonVertexRefs + referencePolygon.mVRef8;
	const Vec3V a = M33MulV3(map->vertex2Shape, V3LoadU_SafeReadW(polyData.mVerts[inds[0]]));	// PT: safe because of the way vertex memory is allocated in ConvexHullData
	
	const FloatV denom0 = V3Dot(planeNormal, V3Sub(capsule.p0, a)); 
	const FloatV denom1 = V3Dot(planeNormal, V3Sub(capsule.p1, a)); 
	const FloatV projPlaneN = V3Dot(planeNormal, normal);
	const FloatV numer = FSel(FIsGrtr(projPlaneN, FZero()), FRecip(projPlaneN), FZero());
	const FloatV t0 = FMul(denom0, numer);
	const FloatV t1 = FMul(denom1, numer);

	const BoolV con0 = FIsGrtrOrEq(radius, t0);
	const BoolV con1 = FIsGrtrOrEq(radius, t1);
	if(BAllEqTTTT(BOr(con0, con1)))
	{
		const Mat33V rot = findRotationMatrixFromZAxis(planeNormal);
		Vec3V* points0In0 = reinterpret_cast<Vec3V*>(PxAllocaAligned(sizeof(Vec3V)*referencePolygon.mNbVerts, 16));
		map->populateVerts(inds, referencePolygon.mNbVerts, polyData.mVerts, points0In0);

		Vec3V rPolygonMin = V3Splat(FMax());
		Vec3V rPolygonMax = V3Neg(rPolygonMin);
		for(PxU32 i=0; i<referencePolygon.mNbVerts; ++i)
		{
			points0In0[i] = M33MulV3(rot, points0In0[i]);
			rPolygonMin = V3Min(rPolygonMin, points0In0[i]);
			rPolygonMax = V3Max(rPolygonMax, points0In0[i]);
		}

		if(BAllEqTTTT(con0))
		{
			//add contact
			const Vec3V proj = V3NegScaleSub(normal, t0, capsule.p0);//V3ScaleAdd(t0, normal, capsule.p0);
			//transform proj0 to 2D
			const Vec3V point = M33MulV3(rot, proj);

			if(contains(points0In0, referencePolygon.mNbVerts, point, rPolygonMin, rPolygonMax))
			{
				manifoldContacts[numContacts].mLocalPointA = aToB.transformInv(capsule.p0);
				manifoldContacts[numContacts].mLocalPointB = proj;
				manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(normal), t0);
			}
		}

		if(BAllEqTTTT(con1))
		{
			const Vec3V proj = V3NegScaleSub(normal, t1, capsule.p1);//V3ScaleAdd(t1, normal, capsule.p1);
			//transform proj0 to 2D
			const Vec3V point = M33MulV3(rot, proj);

			if(contains(points0In0, referencePolygon.mNbVerts, point, rPolygonMin, rPolygonMax))
			{
				manifoldContacts[numContacts].mLocalPointA = aToB.transformInv(capsule.p1);
				manifoldContacts[numContacts].mLocalPointB = proj;
				manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(normal),t1);
			}
		}
	}
}

static void generatedFaceContacts(const CapsuleV& capsule, const PolygonalData& polyData, const SupportLocal* map, const PxMatTransformV& aToB, PersistentContact* manifoldContacts, PxU32& numContacts, 
	const FloatVArg contactDist, const Vec3VArg normal)
{
	const FloatV zero = FZero();
	FloatV tEnter = zero, tExit = zero;
	const FloatV inflatedRadius = FAdd(capsule.radius, contactDist);
	const Vec3V dir = V3Neg(normal);
	const Vec3V vertexSpaceDir = M33MulV3(map->shape2Vertex, dir);
	const Vec3V p0 = M33MulV3(map->shape2Vertex, capsule.p0);

	if(intersectRayPolyhedron(p0, vertexSpaceDir, polyData, tEnter, tExit) && FAllGrtrOrEq(inflatedRadius, tEnter))
	{
		manifoldContacts[numContacts].mLocalPointA = aToB.transformInv(capsule.p0);
		manifoldContacts[numContacts].mLocalPointB = V3ScaleAdd(dir, tEnter, capsule.p0);
		manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(normal), tEnter);
	}

	const Vec3V p1 = M33MulV3(map->shape2Vertex, capsule.p1);
	if(intersectRayPolyhedron(p1, vertexSpaceDir, polyData, tEnter, tExit) && FAllGrtrOrEq(inflatedRadius, tEnter))
	{
		manifoldContacts[numContacts].mLocalPointA = aToB.transformInv(capsule.p1);
		manifoldContacts[numContacts].mLocalPointB = V3ScaleAdd(dir, tEnter, capsule.p1);
		manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(normal), tEnter);
	}	
}

static void generateEE(const Vec3VArg p, const Vec3VArg q, const Vec3VArg normal, const Vec3VArg a, const Vec3VArg b, const PxMatTransformV& aToB, 
	PersistentContact* manifoldContacts, PxU32& numContacts, const FloatVArg inflatedRadius)
{
	const FloatV zero = FZero();
	const FloatV expandedRatio = FLoad(0.005f);
	const Vec3V ab = V3Sub(b, a);
	const Vec3V n = V3Cross(ab, normal);
	const FloatV d = V3Dot(n, a);
	const FloatV np = V3Dot(n, p);
	const FloatV nq = V3Dot(n,q);
	const FloatV signP = FSub(np, d);
	const FloatV signQ = FSub(nq, d);
	const FloatV temp = FMul(signP, signQ);
	if(FAllGrtr(temp, zero)) return;//If both points in the same side as the plane, no intersect points
		
	// if colliding edge (p3,p4) and plane are parallel return no collision
	const Vec3V pq = V3Sub(q, p);
	const FloatV npq = V3Dot(n, pq); 
	if(FAllEq(npq, zero))	return;

	//calculate the intersect point in the segment pq with plane n(x - a).
	const FloatV segTValue = FDiv(FSub(d, np), npq);
	const Vec3V localPointA = V3ScaleAdd(pq, segTValue, p);

	//ML: ab, localPointA and normal is in the same plane, so that we can do 2D segment segment intersection
	//calculate a normal perpendicular to ray localPointA + normal*t, then do 2D segment segment intersection
	const Vec3V perNormal = V3Cross(normal, pq);
	const Vec3V ap = V3Sub(localPointA, a);
	const FloatV nom = V3Dot(perNormal, ap);
	const FloatV denom = V3Dot(perNormal, ab);

	const FloatV tValue = FDiv(nom, denom);

	const FloatV max = FAdd(FOne(), expandedRatio);
	const FloatV min = FSub(zero, expandedRatio);
	if(FAllGrtr(tValue, max) || FAllGrtr(min, tValue))
		return;

	const Vec3V v = V3NegScaleSub(ab, tValue, ap);
	const FloatV signedDist = V3Dot(v, normal);
	if(FAllGrtrOrEq(inflatedRadius, signedDist))
	{
		const Vec3V localPointB = V3Sub(localPointA, v);
		manifoldContacts[numContacts].mLocalPointA = aToB.transformInv(localPointA);
		manifoldContacts[numContacts].mLocalPointB = localPointB;
		manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(normal), signedDist);	
	}
}

static void generatedContactsEEContacts(const CapsuleV& capsule, const PolygonalData& polyData, const HullPolygonData& referencePolygon, const SupportLocal* map, const PxMatTransformV& aToB, 
	PersistentContact* manifoldContacts, PxU32& numContacts, const FloatVArg contactDist, const Vec3VArg contactNormal)
{
	const PxU8* inds = polyData.mPolygonVertexRefs + referencePolygon.mVRef8;

	Vec3V* points0In0 = reinterpret_cast<Vec3V*>(PxAllocaAligned(sizeof(Vec3V)*referencePolygon.mNbVerts, 16));

	//Transform all the verts from vertex space to shape space
	map->populateVerts(inds, referencePolygon.mNbVerts, polyData.mVerts, points0In0);

	const FloatV inflatedRadius = FAdd(capsule.radius, contactDist);

	for (PxU32 rStart = 0, rEnd = PxU32(referencePolygon.mNbVerts - 1); rStart < referencePolygon.mNbVerts; rEnd = rStart++) 
	{
		generateEE(capsule.p0, capsule.p1, contactNormal,points0In0[rStart], points0In0[rEnd], aToB, manifoldContacts, numContacts, inflatedRadius);
	}
}

bool Gu::generateCapsuleBoxFullContactManifold(const CapsuleV& capsule, const PolygonalData& polyData, const SupportLocal* map, const PxMatTransformV& aToB, PersistentContact* manifoldContacts, PxU32& numContacts,
	const FloatVArg contactDist, Vec3V& normal, const Vec3VArg closest, PxReal margin, bool doOverlapTest, PxReal toleranceScale, PxRenderOutput* renderOutput)
{
	PX_UNUSED(renderOutput);
	const PxU32 originalContacts = numContacts;

	const HullPolygonData* referencePolygon = NULL;

	if(doOverlapTest)
	{
		FloatV minOverlap;
		//overwrite the normal
		if(!testSATCapsulePoly(capsule, polyData, map, contactDist, minOverlap, normal))
			return false;

		referencePolygon = &polyData.mPolygons[getPolygonIndex(polyData, map, V3Neg(normal))];
	}
	else
	{
		const PxReal lowerEps = toleranceScale * PCM_WITNESS_POINT_LOWER_EPS;
		const PxReal upperEps = toleranceScale * PCM_WITNESS_POINT_UPPER_EPS;
		const PxReal tolerance = PxClamp(margin, lowerEps, upperEps);

		const PxU32 featureIndex = getWitnessPolygonIndex(polyData, map, V3Neg(normal), closest, tolerance);
		referencePolygon = &polyData.mPolygons[featureIndex];	
	}

#if PCM_LOW_LEVEL_DEBUG
	const PxU8* inds = polyData.mPolygonVertexRefs + referencePolygon->mVRef8;
	Vec3V* pointsInRef = reinterpret_cast<Vec3V*>(PxAllocaAligned(sizeof(Vec3V)*referencePolygon->mNbVerts, 16));
	map->populateVerts(inds, referencePolygon->mNbVerts, polyData.mVerts, pointsInRef);
	PersistentContactManifold::drawPolygon(*renderOutput, map->transform, pointsInRef, referencePolygon->mNbVerts, 0x00ff0000);
#endif
	generatedCapsuleBoxFaceContacts(capsule, polyData, *referencePolygon, map, aToB, manifoldContacts, numContacts, contactDist, normal);
	const PxU32 faceContacts = numContacts - originalContacts;
	if(faceContacts < 2)
	{
		generatedContactsEEContacts(capsule, polyData, *referencePolygon, map, aToB, manifoldContacts, numContacts, contactDist, normal);
	}

	return true;
}

//capsule is in the local space of polyData
bool Gu::generateFullContactManifold(const CapsuleV& capsule, const PolygonalData& polyData, const SupportLocal* map, const PxMatTransformV& aToB, PersistentContact* manifoldContacts, PxU32& numContacts,
	const FloatVArg contactDist, Vec3V& normal, const Vec3VArg closest, PxReal margin, bool doOverlapTest, PxReal toleranceLength)
{
	const PxU32 originalContacts = numContacts;

	Vec3V tNormal = normal;
	
	if(doOverlapTest)
	{	
		FloatV minOverlap;
			
		//overwrite the normal with the sperating axis
		if(!testSATCapsulePoly(capsule, polyData, map, contactDist, minOverlap, tNormal))
			return false;

		generatedFaceContacts(capsule, polyData, map, aToB, manifoldContacts, numContacts, contactDist, tNormal);

		const PxU32 faceContacts = numContacts - originalContacts;
		if(faceContacts < 2)
		{
			const HullPolygonData& referencePolygon = polyData.mPolygons[getPolygonIndex(polyData, map, V3Neg(tNormal))];
			generatedContactsEEContacts(capsule, polyData,referencePolygon, map, aToB, manifoldContacts, numContacts, contactDist, tNormal);
		}
	}
	else
	{
		generatedFaceContacts(capsule, polyData, map, aToB, manifoldContacts, numContacts, contactDist, tNormal);

		const PxU32 faceContacts = numContacts - originalContacts;
		if(faceContacts < 2)
		{
			const PxReal lowerEps = toleranceLength * PCM_WITNESS_POINT_LOWER_EPS;
			const PxReal upperEps = toleranceLength * PCM_WITNESS_POINT_UPPER_EPS;
			const PxReal tolerance = PxClamp(margin, lowerEps, upperEps);

			const PxU32 featureIndex = getWitnessPolygonIndex(polyData, map, V3Neg(tNormal), closest, tolerance);
			const HullPolygonData& referencePolygon = polyData.mPolygons[featureIndex];
			generatedContactsEEContacts(capsule, polyData,referencePolygon, map, aToB, manifoldContacts, numContacts, contactDist, tNormal);
		}
	}

	normal = tNormal;

	return true;
}

//sphere is in the local space of polyData, we treate sphere as capsule
bool Gu::generateSphereFullContactManifold(const CapsuleV& capsule, const PolygonalData& polyData, const SupportLocal* map, PersistentContact* manifoldContacts, PxU32& numContacts,
	const FloatVArg contactDist, Vec3V& normal, bool doOverlapTest)
{
	if(doOverlapTest)
	{	
		FloatV minOverlap;
		//overwrite the normal with the sperating axis
		if(!testPolyDataAxis(capsule, polyData, map, contactDist, minOverlap, normal))
			return false;
	}

	const FloatV zero = FZero();
	FloatV tEnter = zero, tExit = zero;
	const FloatV inflatedRadius = FAdd(capsule.radius, contactDist);
	const Vec3V dir = V3Neg(normal);
	const Vec3V vertexSpaceDir = M33MulV3(map->shape2Vertex, dir);
	const Vec3V p0 = M33MulV3(map->shape2Vertex, capsule.p0);
	if(intersectRayPolyhedron(p0, vertexSpaceDir, polyData, tEnter, tExit) && FAllGrtrOrEq(inflatedRadius, tEnter))
	{
		//ML: for sphere, the contact point A is always zeroV in its local space
		manifoldContacts[numContacts].mLocalPointA = V3Zero();
		manifoldContacts[numContacts].mLocalPointB = V3ScaleAdd(dir, tEnter, capsule.p0);
		manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(normal), tEnter);
	}

	return true;
}

//capsule is in the shape space of polyData 
bool Gu::computeMTD(const CapsuleV& capsule, const PolygonalData& polyData, const SupportLocal* map, FloatV& penDepth, Vec3V& normal)
{
	const FloatV contactDist = FZero();
	Vec3V separatingAxis;
	FloatV minOverlap;
	if(!testSATCapsulePoly(capsule, polyData, map, contactDist, minOverlap, separatingAxis))
		return false;

	//transform normal in to the worl space
	normal = map->transform.rotate(separatingAxis);
	penDepth = minOverlap;
		
	return true;
}
