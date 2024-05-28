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

#include "GuVecTriangle.h"
#include "GuContactMethodImpl.h"
#include "GuPCMContactConvexCommon.h"
#include "GuInternal.h"
#include "GuPCMContactMeshCallback.h"
#include "GuBarycentricCoordinates.h"
#include "GuBox.h"

using namespace physx;
using namespace Gu;
using namespace aos;

namespace
{
struct PCMCapsuleVsMeshContactGenerationCallback : PCMMeshContactGenerationCallback< PCMCapsuleVsMeshContactGenerationCallback >
{
	PCMCapsuleVsMeshContactGenerationCallback& operator=(const PCMCapsuleVsMeshContactGenerationCallback&);

	PCMCapsuleVsMeshContactGeneration	mGeneration;

	PCMCapsuleVsMeshContactGenerationCallback(
		const CapsuleV& capsule,
		const FloatVArg contactDist,
		const FloatVArg replaceBreakingThreshold,
		const PxTransformV& sphereTransform,
		const PxTransformV& meshTransform,
		MultiplePersistentContactManifold& multiManifold,
		PxContactBuffer& contactBuffer,
		const PxU8* extraTriData,
		const Cm::FastVertex2ShapeScaling& meshScaling,
		bool idtMeshScale,
		PxInlineArray<PxU32, LOCAL_PCM_CONTACTS_SIZE>* deferredContacts,
		PxRenderOutput* renderOutput = NULL
	) :
		PCMMeshContactGenerationCallback<PCMCapsuleVsMeshContactGenerationCallback>(meshScaling, extraTriData, idtMeshScale),
		mGeneration(capsule, contactDist, replaceBreakingThreshold, sphereTransform, meshTransform, multiManifold, contactBuffer, 
			deferredContacts, renderOutput)
	{
	}

	PX_FORCE_INLINE bool doTest(const PxVec3&, const PxVec3&, const PxVec3&) { return true; }

	template<PxU32 CacheSize>
	void processTriangleCache(TriangleCache<CacheSize>& cache)
	{
		mGeneration.processTriangleCache<CacheSize, PCMCapsuleVsMeshContactGeneration>(cache);
	}
};
}

bool Gu::pcmContactCapsuleMesh(GU_CONTACT_METHOD_ARGS)
{
	MultiplePersistentContactManifold& multiManifold = cache.getMultipleManifold();
	const PxCapsuleGeometry& shapeCapsule = checkedCast<PxCapsuleGeometry>(shape0);
	const PxTriangleMeshGeometry& shapeMesh = checkedCast<PxTriangleMeshGeometry>(shape1);

	//gRenderOutPut = cache.mRenderOutput;
	const FloatV capsuleRadius = FLoad(shapeCapsule.radius);
	const FloatV contactDist = FLoad(params.mContactDistance);

	const PxTransformV capsuleTransform = loadTransformA(transform0);//capsule transform
	const PxTransformV meshTransform = loadTransformA(transform1);//triangleMesh  

	const PxTransformV curTransform = meshTransform.transformInv(capsuleTransform);
	
	// We must be in local space to use the cache
	if(multiManifold.invalidate(curTransform, capsuleRadius, FLoad(0.02f)))
	{
		const FloatV replaceBreakingThreshold = FMul(capsuleRadius, FLoad(0.001f));
		//const FloatV capsuleHalfHeight = FloatV_From_F32(shapeCapsule.halfHeight);
		Cm::FastVertex2ShapeScaling meshScaling;
		const bool idtMeshScale = shapeMesh.scale.isIdentity();
		if(!idtMeshScale)
			meshScaling.init(shapeMesh.scale);

		// Capsule data
		const PxVec3 tmp = getCapsuleHalfHeightVector(transform0, shapeCapsule);
		const Segment worldCapsule(transform0.p + tmp, transform0.p - tmp);

		const Segment meshCapsule(	// Capsule in mesh space
			transform1.transformInv(worldCapsule.p0),
			transform1.transformInv(worldCapsule.p1));

		const PxReal inflatedRadius = shapeCapsule.radius + params.mContactDistance;

		const PxVec3 capsuleCenterInMesh = transform1.transformInv(transform0.p);
		const PxVec3 capsuleDirInMesh = transform1.rotateInv(tmp);
		const CapsuleV capsule(V3LoadU(capsuleCenterInMesh), V3LoadU(capsuleDirInMesh), capsuleRadius);

		// We must be in local space to use the cache
		const Capsule queryCapsule(meshCapsule, inflatedRadius);

		const TriangleMesh* meshData = _getMeshData(shapeMesh);

		multiManifold.mNumManifolds = 0;
		multiManifold.setRelativeTransform(curTransform); 

		const PxU8* PX_RESTRICT extraData = meshData->getExtraTrigData();
		// mesh scale is not baked into cached verts
		PCMCapsuleVsMeshContactGenerationCallback callback(
			capsule,
			contactDist,
			replaceBreakingThreshold,
			capsuleTransform,
			meshTransform,
			multiManifold,
			contactBuffer,
			extraData,
			meshScaling,
			idtMeshScale,
			NULL,
			renderOutput);

		//bound the capsule in shape space by an OBB:
		Box queryBox;
		queryBox.create(queryCapsule);

		//apply the skew transform to the box:
		if(!idtMeshScale)
			meshScaling.transformQueryBounds(queryBox.center, queryBox.extents, queryBox.rot);

		Midphase::intersectOBB(meshData, queryBox, callback, true);

		callback.flushCache();
	
		callback.mGeneration.processContacts(GU_CAPSULE_MANIFOLD_CACHE_SIZE, false);
	}
	else
	{
		const PxMatTransformV aToB(curTransform);
		const FloatV projectBreakingThreshold = FMul(capsuleRadius, FLoad(0.05f));
		const FloatV refereshDistance = FAdd(capsuleRadius, contactDist);
		//multiManifold.refreshManifold(aToB, projectBreakingThreshold, contactDist);
		multiManifold.refreshManifold(aToB, projectBreakingThreshold, refereshDistance);
	}

	//multiManifold.drawManifold(*gRenderOutPut, capsuleTransform, meshTransform);
	return multiManifold.addManifoldContactsToContactBuffer(contactBuffer, capsuleTransform, meshTransform, capsuleRadius);
}

void Gu::PCMCapsuleVsMeshContactGeneration::generateEE(const Vec3VArg p, const Vec3VArg q, const FloatVArg sqInflatedRadius,
														const Vec3VArg normal, PxU32 triangleIndex, const Vec3VArg a, const Vec3VArg b,
														MeshPersistentContact* manifoldContacts, PxU32& numContacts)
{
	const FloatV zero = FZero();
	const Vec3V ab = V3Sub(b, a);
	const Vec3V n = V3Cross(ab, normal);
	const FloatV d = V3Dot(n, a);
	const FloatV np = V3Dot(n, p);
	const FloatV nq = V3Dot(n,q);
	const FloatV signP = FSub(np, d);
	const FloatV signQ = FSub(nq, d);
	const FloatV temp = FMul(signP, signQ);
	if(FAllGrtr(temp, zero))
		return;//If both points in the same side as the plane, no intersect points
	
	// if colliding edge (p3,p4) and plane are parallel return no collision
	const Vec3V pq = V3Sub(q, p);
	const FloatV npq= V3Dot(n, pq); 
	if(FAllEq(npq, zero))
		return;

	const FloatV one = FOne();
	//calculate the intersect point in the segment pq
	const FloatV segTValue = FDiv(FSub(d, np), npq);
	const Vec3V localPointA = V3ScaleAdd(pq, segTValue, p);

	//calculate a normal perpendicular to ray localPointA + normal, 2D segment segment intersection
	const Vec3V perNormal = V3Cross(normal, pq);
	const Vec3V ap = V3Sub(localPointA, a);
	const FloatV nom = V3Dot(perNormal, ap);
	const FloatV denom = V3Dot(perNormal, ab);

	//const FloatV tValue = FClamp(FDiv(nom, denom), zero, FOne());
	const FloatV tValue = FDiv(nom, denom);
	const BoolV con = BAnd(FIsGrtrOrEq(one, tValue), FIsGrtrOrEq(tValue, zero));
	if(BAllEqFFFF(con))
		return;

	//const Vec3V localPointB = V3ScaleAdd(ab, tValue, a); v = V3Sub(localPointA, localPointB); v = V3NegScaleSub(ab, tValue, tap)
	const Vec3V v = V3NegScaleSub(ab, tValue, ap);
	const FloatV sqDist = V3Dot(v, v);
	
	if(FAllGrtr(sqInflatedRadius, sqDist))
	{	
		const Vec3V localPointB = V3Sub(localPointA, v);
		const FloatV signedDist = V3Dot(v, normal);
	
		manifoldContacts[numContacts].mLocalPointA = localPointA;
		manifoldContacts[numContacts].mLocalPointB = localPointB;
		manifoldContacts[numContacts].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(normal), signedDist);
		manifoldContacts[numContacts++].mFaceIndex = triangleIndex;
	}
}

void Gu::PCMCapsuleVsMeshContactGeneration::generateEEContacts(const Vec3VArg a, const Vec3VArg b,
															   const Vec3VArg c, const Vec3VArg normal,
															   PxU32 triangleIndex, const Vec3VArg p, 
															   const Vec3VArg q, const FloatVArg sqInflatedRadius,
															   PxU32 previousNumContacts, MeshPersistentContact* manifoldContacts, 
															   PxU32& numContacts)
{
	if((numContacts - previousNumContacts) < 2)
		generateEE(p, q, sqInflatedRadius, normal, triangleIndex, a, b, manifoldContacts, numContacts);
	if((numContacts - previousNumContacts) < 2)
		generateEE(p, q, sqInflatedRadius, normal, triangleIndex, b, c, manifoldContacts, numContacts);
	if((numContacts - previousNumContacts) < 2)
		generateEE(p, q, sqInflatedRadius, normal, triangleIndex, a, c, manifoldContacts, numContacts);
}

void Gu::PCMCapsuleVsMeshContactGeneration::generateEEMTD(	const Vec3VArg p, const Vec3VArg q, const FloatVArg inflatedRadius,
															const Vec3VArg normal, PxU32 triangleIndex, const Vec3VArg a, const Vec3VArg b,
															MeshPersistentContact* manifoldContacts, PxU32& numContacts)
{
	const FloatV zero = FZero();
	const Vec3V ab = V3Sub(b, a);
	const Vec3V n = V3Cross(ab, normal);
	const FloatV d = V3Dot(n, a);
	const FloatV np = V3Dot(n, p);
	const FloatV nq = V3Dot(n,q);
	const FloatV signP = FSub(np, d);
	const FloatV signQ = FSub(nq, d);
	const FloatV temp = FMul(signP, signQ);
	if(FAllGrtr(temp, zero))
		return;//If both points in the same side as the plane, no intersect points
	
	// if colliding edge (p3,p4) and plane are parallel return no collision
	const Vec3V pq = V3Sub(q, p);
	const FloatV npq= V3Dot(n, pq); 
	if(FAllEq(npq, zero))
		return;

	//calculate the intersect point in the segment pq
	const FloatV segTValue = FDiv(FSub(d, np), npq);
	const Vec3V localPointA = V3ScaleAdd(pq, segTValue, p);

	//calculate a normal perpendicular to ray localPointA + normal, 2D segment segment intersection
	const Vec3V perNormal = V3Cross(normal, pq);
	const Vec3V ap = V3Sub(localPointA, a);
	const FloatV nom = V3Dot(perNormal, ap);
	const FloatV denom = V3Dot(perNormal, ab);

	const FloatV tValue = FClamp(FDiv(nom, denom), zero, FOne());

	const Vec3V v = V3NegScaleSub(ab, tValue, ap);
	const FloatV signedDist = V3Dot(v, normal);

	if(FAllGrtr(inflatedRadius, signedDist))
	{
		const Vec3V localPointB = V3Sub(localPointA, v);
		manifoldContacts[numContacts].mLocalPointA = localPointA;
		manifoldContacts[numContacts].mLocalPointB = localPointB;
		manifoldContacts[numContacts].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(normal), signedDist);
		manifoldContacts[numContacts++].mFaceIndex = triangleIndex;
	}
}

void Gu::PCMCapsuleVsMeshContactGeneration::generateEEContactsMTD(	const Vec3VArg a, const Vec3VArg b,
																	const Vec3VArg c, const Vec3VArg normal,
																	PxU32 triangleIndex, const Vec3VArg p, 
																	const Vec3VArg q, const FloatVArg inflatedRadius,
																	MeshPersistentContact* manifoldContacts, PxU32& numContacts)
{	
	generateEEMTD(p, q, inflatedRadius, normal, triangleIndex, a, b, manifoldContacts, numContacts);
	generateEEMTD(p, q, inflatedRadius, normal, triangleIndex, b, c, manifoldContacts, numContacts);
	generateEEMTD(p, q, inflatedRadius, normal, triangleIndex, a, c, manifoldContacts, numContacts);
}

void Gu::PCMCapsuleVsMeshContactGeneration::generateContacts(const Vec3VArg a, const Vec3VArg b,
															 const Vec3VArg c, const Vec3VArg planeNormal, 
															 const Vec3VArg normal, PxU32 triangleIndex,
															 const Vec3VArg p, const Vec3VArg q,
															 const FloatVArg inflatedRadius,
															 MeshPersistentContact* manifoldContacts, PxU32& numContacts)
{
	const Vec3V ab = V3Sub(b, a);
	const Vec3V ac = V3Sub(c, a);
	const Vec3V ap = V3Sub(p, a);
	const Vec3V aq = V3Sub(q, a);

	//This is used to calculate the barycentric coordinate
	const FloatV d00 = V3Dot(ab, ab);
	const FloatV d01 = V3Dot(ab, ac);
	const FloatV d11 = V3Dot(ac, ac);

	const FloatV zero = FZero();
	const FloatV largeValue = FLoad(1e+20f);
	const FloatV tDenom = FSub(FMul(d00, d11), FMul(d01, d01));
	const FloatV bdenom = FSel(FIsGrtr(tDenom, zero), FRecip(tDenom), largeValue);

	//compute the intersect point of p and triangle plane abc
	const FloatV inomp = V3Dot(planeNormal, V3Neg(ap));
	const FloatV ideom = V3Dot(planeNormal, normal);

	const FloatV ipt = FSel(FIsGrtr(ideom, zero), FNeg(FDiv(inomp, ideom)), largeValue);

	const Vec3V closestP31 = V3NegScaleSub(normal, ipt, p);
	const Vec3V closestP30 = p;

	//Compute the barycentric coordinate for project point of q
	const Vec3V pV20 = V3Sub(closestP31, a);
	const FloatV pD20 = V3Dot(pV20, ab);
	const FloatV pD21 = V3Dot(pV20, ac);
	const FloatV v0 = FMul(FSub(FMul(d11, pD20), FMul(d01, pD21)), bdenom);
	const FloatV w0 = FMul(FSub(FMul(d00, pD21), FMul(d01, pD20)), bdenom);

	//check closestP3 is inside the triangle
	const BoolV con0 = isValidTriangleBarycentricCoord(v0, w0);

	const BoolV tempCon0 = BAnd(con0, FIsGrtr(inflatedRadius, ipt));
	if(BAllEqTTTT(tempCon0))
	{
		manifoldContacts[numContacts].mLocalPointA = closestP30;//transform to B space, it will get convert to A space later
		manifoldContacts[numContacts].mLocalPointB = closestP31;
		manifoldContacts[numContacts].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(normal), ipt);
		manifoldContacts[numContacts++].mFaceIndex = triangleIndex;
	}
	
	const FloatV inomq = V3Dot(planeNormal, V3Neg(aq));

	const FloatV iqt = FSel(FIsGrtr(ideom, FZero()), FNeg(FDiv(inomq, ideom)), FLoad(1e+20f));

	const Vec3V closestP41 = V3NegScaleSub(normal, iqt, q);
	const Vec3V closestP40 = q;

	//Compute the barycentric coordinate for project point of q
	const Vec3V qV20 = V3Sub(closestP41, a);
	const FloatV qD20 = V3Dot(qV20, ab);
	const FloatV qD21 = V3Dot(qV20, ac);
	const FloatV v1 = FMul(FSub(FMul(d11, qD20), FMul(d01, qD21)), bdenom);
	const FloatV w1 = FMul(FSub(FMul(d00, qD21), FMul(d01, qD20)), bdenom);

	const BoolV con1 = isValidTriangleBarycentricCoord(v1, w1);
	
	const BoolV tempCon1 = BAnd(con1, FIsGrtr(inflatedRadius, iqt));
	if(BAllEqTTTT(tempCon1))
	{
		manifoldContacts[numContacts].mLocalPointA = closestP40;
		manifoldContacts[numContacts].mLocalPointB = closestP41;
		manifoldContacts[numContacts].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(normal), iqt);
		manifoldContacts[numContacts++].mFaceIndex = triangleIndex;
	}
}

static PX_FORCE_INLINE Vec4V pcmDistanceSegmentSegmentSquared4(const Vec3VArg p, const Vec3VArg d0, 
																const Vec3VArg p02, const Vec3VArg d02, 
																const Vec3VArg p12, const Vec3VArg d12, 
																const Vec3VArg p22, const Vec3VArg d22,
																const Vec3VArg p32, const Vec3VArg d32,
																Vec4V& s, Vec4V& t)
{
	const Vec4V zero = V4Zero();
	const Vec4V one = V4One();
	const Vec4V eps = V4Eps();
	const Vec4V half = V4Splat(FHalf());
 
	const Vec4V d0X = V4Splat(V3GetX(d0));
	const Vec4V d0Y = V4Splat(V3GetY(d0));
	const Vec4V d0Z = V4Splat(V3GetZ(d0));
	const Vec4V pX  = V4Splat(V3GetX(p));
	const Vec4V pY  = V4Splat(V3GetY(p));
	const Vec4V pZ  = V4Splat(V3GetZ(p));

	Vec4V d024 = Vec4V_From_Vec3V(d02);
	Vec4V d124 = Vec4V_From_Vec3V(d12);
	Vec4V d224 = Vec4V_From_Vec3V(d22);
	Vec4V d324 = Vec4V_From_Vec3V(d32);

	Vec4V p024 = Vec4V_From_Vec3V(p02);
	Vec4V p124 = Vec4V_From_Vec3V(p12);
	Vec4V p224 = Vec4V_From_Vec3V(p22);
	Vec4V p324 = Vec4V_From_Vec3V(p32);

	Vec4V d0123X, d0123Y, d0123Z;
	Vec4V p0123X, p0123Y, p0123Z;

	PX_TRANSPOSE_44_34(d024, d124, d224, d324, d0123X, d0123Y, d0123Z);
	PX_TRANSPOSE_44_34(p024, p124, p224, p324, p0123X, p0123Y, p0123Z);

	const Vec4V rX = V4Sub(pX, p0123X);
	const Vec4V rY = V4Sub(pY, p0123Y);
	const Vec4V rZ = V4Sub(pZ, p0123Z);

	//TODO - store this in a transposed state and avoid so many dot products?

	const FloatV dd = V3Dot(d0, d0);

	const Vec4V e = V4MulAdd(d0123Z, d0123Z, V4MulAdd(d0123X, d0123X, V4Mul(d0123Y, d0123Y)));
	const Vec4V b = V4MulAdd(d0Z, d0123Z, V4MulAdd(d0X, d0123X, V4Mul(d0Y, d0123Y)));
	const Vec4V c = V4MulAdd(d0Z, rZ, V4MulAdd(d0X, rX, V4Mul(d0Y, rY)));
	const Vec4V f = V4MulAdd(d0123Z, rZ, V4MulAdd(d0123X, rX, V4Mul(d0123Y, rY))); 

	const Vec4V a(V4Splat(dd));

	const Vec4V aRecip(V4Recip(a));
	const Vec4V eRecip(V4Recip(e));

	//if segments not parallell, compute closest point on two segments and clamp to segment1
	const Vec4V denom = V4Sub(V4Mul(a, e), V4Mul(b, b));
	const Vec4V temp = V4Sub(V4Mul(b, f), V4Mul(c, e));
	//const Vec4V s0 = V4Clamp(V4Div(temp, denom), zero, one);
	//In PS3, 0(temp)/0(denom) will produce QNaN and V4Clamp can't clamp the value to zero and one. In PC, 0/0 will produce inf and V4Clamp clamp the value to be one.
	//Therefore, we need to add the select code to protect against this case
	const Vec4V value = V4Sel(V4IsEq(denom, zero), one, V4Div(temp, denom));
	const Vec4V s0 = V4Clamp(value, zero, one);
  
	//test whether segments are parallel
	const BoolV con2 = V4IsGrtrOrEq(eps, denom);     
	const Vec4V sTmp = V4Sel(con2, half, s0);
      
	//compute point on segment2 closest to segment1
	//const Vec4V tTmp = V4Mul(V4Add(V4Mul(b, sTmp), f), eRecip);
	const Vec4V tTmp = V4Sel(V4IsEq(e, zero), one, V4Mul(V4Add(V4Mul(b, sTmp), f), eRecip));

	//if t is in [zero, one], done. otherwise clamp t
	const Vec4V t2 = V4Clamp(tTmp, zero, one);

	//recompute s for the new value
	//const Vec4V comp = V4Mul(V4Sub(V4Mul(b,t2), c), aRecip);
	const Vec4V comp = V4Sel(V4IsEq(a, zero), one, V4Mul(V4Sub(V4Mul(b,t2), c), aRecip));
	const Vec4V s2 = V4Clamp(comp, zero, one);

	s = s2;
	t = t2;

	const Vec4V closest1X = V4MulAdd(d0X, s2, pX);
	const Vec4V closest1Y = V4MulAdd(d0Y, s2, pY);
	const Vec4V closest1Z = V4MulAdd(d0Z, s2, pZ);

	const Vec4V closest2X = V4MulAdd(d0123X, t2, p0123X);
	const Vec4V closest2Y = V4MulAdd(d0123Y, t2, p0123Y);
	const Vec4V closest2Z = V4MulAdd(d0123Z, t2, p0123Z);

	const Vec4V vvX = V4Sub(closest1X, closest2X);
	const Vec4V vvY = V4Sub(closest1Y, closest2Y);
	const Vec4V vvZ = V4Sub(closest1Z, closest2Z);

	const Vec4V vd = V4MulAdd(vvX, vvX, V4MulAdd(vvY, vvY, V4Mul(vvZ, vvZ)));

	return vd;
}

//	t is the barycenteric coordinate of a segment
//	u is the barycenteric coordinate of a triangle
//	v is the barycenteric coordinate of a triangle
static FloatV pcmDistanceSegmentTriangleSquared(const Vec3VArg p, const Vec3VArg q,
												const Vec3VArg a, const Vec3VArg b, const Vec3VArg c,
												FloatV& t, FloatV& u, FloatV& v)
{
	const FloatV one = FOne();
	const FloatV zero = FZero();

	const Vec3V pq = V3Sub(q, p);
	const Vec3V ab = V3Sub(b, a);
	const Vec3V ac = V3Sub(c, a);
	const Vec3V bc = V3Sub(c, b);
	const Vec3V ap = V3Sub(p, a);
	const Vec3V aq = V3Sub(q, a);

	const Vec3V n = V3Normalize(V3Cross(ab, ac)); // normalize vector

	const Vec4V combinedDot = V3Dot4(ab, ab, ab, ac, ac, ac, ap, n);
	const FloatV d00 = V4GetX(combinedDot);
	const FloatV d01 = V4GetY(combinedDot);
	const FloatV d11 = V4GetZ(combinedDot);
	const FloatV dist3 = V4GetW(combinedDot);

	// PT: the new version is copied from Gu::distanceSegmentTriangleSquared
	const FloatV tDenom = FSub(FMul(d00, d11), FMul(d01, d01));
	const FloatV bdenom = FSel(FIsGrtr(tDenom, zero), FRecip(tDenom), zero);

	const FloatV sqDist3 = FMul(dist3, dist3);

	//compute the closest point of q and triangle plane abc
	const FloatV dist4 = V3Dot(aq, n);
	const FloatV sqDist4 = FMul(dist4, dist4);
	const FloatV dMul = FMul(dist3, dist4);
	const BoolV con = FIsGrtr(zero, dMul);

	if(BAllEqTTTT(con))
	{
		//compute the intersect point
		const FloatV nom = FNeg(V3Dot(n, ap));
		const FloatV denom = FRecip(V3Dot(n, pq));
		const FloatV t0 = FMul(nom, denom);
		const Vec3V ip = V3ScaleAdd(pq, t0, p);//V3Add(p, V3Scale(pq, t));
		const Vec3V v2 = V3Sub(ip, a);
		const FloatV d20 = V3Dot(v2, ab);
		const FloatV d21 = V3Dot(v2, ac);

		const FloatV v0 = FMul(FNegScaleSub(d01, d21, FMul(d11, d20)), bdenom);
		const FloatV w0 = FMul(FNegScaleSub(d01, d20, FMul(d00, d21)), bdenom);

		const BoolV con0 = isValidTriangleBarycentricCoord(v0, w0);
		if(BAllEqTTTT(con0))
		{
			t = t0;
			u = v0;
			v = w0;
			return zero;
		}
	}

	const Vec3V closestP31 = V3NegScaleSub(n, dist3, p);//V3Sub(p, V3Scale(n, dist3));
	const Vec3V closestP41 = V3NegScaleSub(n, dist4, q);// V3Sub(q, V3Scale(n, dist4));

	//Compute the barycentric coordinate for project point of q
	const Vec3V pV20 = V3Sub(closestP31, a);
	const Vec3V qV20 = V3Sub(closestP41, a);

	const Vec4V pD2 = V3Dot4(pV20, ab, pV20, ac, qV20, ab, qV20, ac);

	const Vec4V pD2Swizzle = V4PermYXWZ(pD2);

	const Vec4V d11d00 = V4UnpackXY(V4Splat(d11), V4Splat(d00));

	const Vec4V v0w0v1w1 = V4Scale(V4NegMulSub(V4Splat(d01), pD2Swizzle, V4Mul(d11d00, pD2)), bdenom);

	const FloatV v0 = V4GetX(v0w0v1w1);
	const FloatV w0 = V4GetY(v0w0v1w1);
	const FloatV v1 = V4GetZ(v0w0v1w1);
	const FloatV w1 = V4GetW(v0w0v1w1);

	const BoolV _con = isValidTriangleBarycentricCoord2(v0w0v1w1);

	const BoolV con0 = BGetX(_con);
	const BoolV con1 = BGetY(_con);

	const BoolV cond2 = BAnd(con0, con1);	

	if(BAllEqTTTT(cond2))
	{
		//	both p and q project points are interior point 
		const BoolV d2 = FIsGrtr(sqDist4, sqDist3);
		t = FSel(d2, zero, one);
		u = FSel(d2, v0, v1);
		v = FSel(d2, w0, w1);
		return FSel(d2, sqDist3, sqDist4);
	}
	else
	{
		Vec4V t40, t41;
		const Vec4V sqDist44 = pcmDistanceSegmentSegmentSquared4(p,pq,a,ab, b,bc, a,ac, a,ab, t40, t41); 

		const FloatV t00 = V4GetX(t40);
		const FloatV t10 = V4GetY(t40);
		const FloatV t20 = V4GetZ(t40);

		const FloatV t01 = V4GetX(t41);
		const FloatV t11 = V4GetY(t41);
		const FloatV t21 = V4GetZ(t41);

		//edge ab
		const FloatV u01 = t01;
		const FloatV v01 = zero;

		//edge bc
		const FloatV u11 = FSub(one, t11);
		const FloatV v11 = t11;

		//edge ac
		const FloatV u21 = zero;
		const FloatV v21 = t21;

		const FloatV sqDist0(V4GetX(sqDist44));
		const FloatV sqDist1(V4GetY(sqDist44));
		const FloatV sqDist2(V4GetZ(sqDist44));

		const BoolV con2 = BAnd(FIsGrtr(sqDist1, sqDist0), FIsGrtr(sqDist2, sqDist0));
		const BoolV con3 = FIsGrtr(sqDist2, sqDist1);
		const FloatV sqDistPE = FSel(con2, sqDist0, FSel(con3, sqDist1, sqDist2));
		const FloatV uEdge = FSel(con2, u01, FSel(con3, u11, u21));
		const FloatV vEdge = FSel(con2, v01, FSel(con3, v11, v21));
		const FloatV tSeg = FSel(con2, t00, FSel(con3, t10, t20));

		if(BAllEqTTTT(con0))
		{
			//p's project point is an interior point
			const BoolV d2 = FIsGrtr(sqDistPE, sqDist3);
			t = FSel(d2, zero, tSeg);
			u = FSel(d2, v0, uEdge);
			v = FSel(d2, w0, vEdge);
			return FSel(d2, sqDist3, sqDistPE);
		}
		else if(BAllEqTTTT(con1))
		{
			//q's project point is an interior point
			const BoolV d2 = FIsGrtr(sqDistPE, sqDist4);
			t = FSel(d2, one, tSeg);
			u = FSel(d2, v1, uEdge);
			v = FSel(d2, w1, vEdge);
			return FSel(d2, sqDist4, sqDistPE);
		}
		else
		{
			t = tSeg;
			u = uEdge;
			v = vEdge;
			return sqDistPE;
		}
	}
}

static bool selectNormal(const FloatVArg u, FloatVArg v, PxU8 data)
{
	const FloatV zero = FLoad(1e-6f);
	const FloatV one = FLoad(0.999999f);
	// Analysis
	if(FAllGrtr(zero, u))
	{
		if(FAllGrtr(zero, v))
		{
			// Vertex 0
			if(!(data & (ETD_CONVEX_EDGE_01|ETD_CONVEX_EDGE_20)))
				return true;
		}
		else if(FAllGrtr(v, one))
		{
			// Vertex 2
			if(!(data & (ETD_CONVEX_EDGE_12|ETD_CONVEX_EDGE_20)))
				return true;
		}
		else
		{
			// Edge 0-2
			if(!(data & ETD_CONVEX_EDGE_20))
				return true;
		}
	}
	else if(FAllGrtr(u,one))
	{
		if(FAllGrtr(zero, v))
		{
			// Vertex 1
			if(!(data & (ETD_CONVEX_EDGE_01|ETD_CONVEX_EDGE_12)))
				return true;
		}
	}
	else
	{
		if(FAllGrtr(zero, v))
		{
			// Edge 0-1
			if(!(data & ETD_CONVEX_EDGE_01))
				return true;
		}
		else
		{
			const FloatV threshold = FLoad(0.9999f);
			const FloatV temp = FAdd(u, v);
			if(FAllGrtrOrEq(temp, threshold))
			{
				// Edge 1-2
				if(!(data & ETD_CONVEX_EDGE_12))
					return true;
			}
			else
			{
				// Face
				return true;
			}
		}
	}
	return false;
}

bool Gu::PCMCapsuleVsMeshContactGeneration::processTriangle(const PxVec3* verts, PxU32 triangleIndex, PxU8 triFlags, const PxU32* vertInds)
{
	PX_UNUSED(vertInds);

	const FloatV zero = FZero();

	const Vec3V p0 = V3LoadU(verts[0]);
	const Vec3V p1 = V3LoadU(verts[1]);
	const Vec3V p2 = V3LoadU(verts[2]);

	const Vec3V p10 = V3Sub(p1, p0);
	const Vec3V p20 = V3Sub(p2, p0);

	const Vec3V n = V3Normalize(V3Cross(p10, p20));//(p1 - p0).cross(p2 - p0).getNormalized();
	const FloatV d = V3Dot(p0, n);//d = -p0.dot(n);

	const FloatV dist = FSub(V3Dot(mCapsule.getCenter(), n), d);//p.dot(n) + d;
	
	// Backface culling
	if(FAllGrtr(zero, dist))
		return false;

	FloatV t, u, v;
	const FloatV sqDist = pcmDistanceSegmentTriangleSquared(mCapsule.p0, mCapsule.p1, p0, p1, p2, t, u, v);
	
	if(FAllGrtr(mSqInflatedRadius, sqDist))
	{
		Vec3V patchNormalInTriangle;
		if(selectNormal(u, v, triFlags))
		{
			patchNormalInTriangle = n;
		}
		else
		{
			if(FAllEq(sqDist, zero))
			{
				//segment intersect with the triangle
				patchNormalInTriangle = n;
			}
			else
			{
				const Vec3V pq = V3Sub(mCapsule.p1, mCapsule.p0);
				const Vec3V pointOnSegment = V3ScaleAdd(pq, t, mCapsule.p0);
				const FloatV w = FSub(FOne(), FAdd(u, v));
				const Vec3V pointOnTriangle = V3ScaleAdd(p0, w, V3ScaleAdd(p1, u, V3Scale(p2, v)));
				patchNormalInTriangle = V3Normalize(V3Sub(pointOnSegment, pointOnTriangle));	
			}
		}

		const PxU32 previousNumContacts = mNumContacts;

		generateContacts(p0, p1, p2, n, patchNormalInTriangle, triangleIndex, mCapsule.p0, mCapsule.p1, mInflatedRadius, mManifoldContacts, mNumContacts);
		
		//ML: this need to use the sqInflatedRadius to avoid some bad contacts
		generateEEContacts(p0, p1, p2, patchNormalInTriangle, triangleIndex, mCapsule.p0, mCapsule.p1, mSqInflatedRadius, previousNumContacts, mManifoldContacts, mNumContacts);

		PxU32 numContacts = mNumContacts - previousNumContacts;

		PX_ASSERT(numContacts <= 2);

		if(numContacts > 0)
		{ 
			FloatV maxPen = FMax();
			for(PxU32 i = previousNumContacts; i<mNumContacts; ++i)
			{
				const FloatV pen = V4GetW(mManifoldContacts[i].mLocalNormalPen);
				mManifoldContacts[i].mLocalPointA = mMeshToConvex.transform(mManifoldContacts[i].mLocalPointA);
				maxPen = FMin(maxPen, pen);
			}

			for(PxU32 i = previousNumContacts; i<mNumContacts; ++i)
			{
				Vec3V contact0 = mManifoldContacts[i].mLocalPointB;
				for(PxU32 j=i+1; j<mNumContacts; ++j)
				{
					Vec3V contact1 = mManifoldContacts[j].mLocalPointB;
					Vec3V dif = V3Sub(contact1, contact0);
					FloatV d1 = V3Dot(dif, dif);
					if(FAllGrtr(mSqReplaceBreakingThreshold, d1))
					{
						mManifoldContacts[j] = mManifoldContacts[mNumContacts-1];
						mNumContacts--;
						j--;
					}
				}
			}

			PX_ASSERT(mNumContactPatch <PCM_MAX_CONTACTPATCH_SIZE);

			addManifoldPointToPatch(patchNormalInTriangle, maxPen, previousNumContacts);
			
			PX_ASSERT(mNumContactPatch <PCM_MAX_CONTACTPATCH_SIZE);
			if(mNumContacts >= 16)
			{
				PX_ASSERT(mNumContacts <= 64);
				processContacts(GU_CAPSULE_MANIFOLD_CACHE_SIZE);
			}
		}
	}

	return true;
}

bool Gu::PCMCapsuleVsMeshContactGeneration::processTriangle(const TriangleV& triangleV, PxU32 triangleIndex, const CapsuleV& capsule, const FloatVArg inflatedRadius, const PxU8 trigFlag,
															MeshPersistentContact* manifoldContacts, PxU32& numContacts)
{
	const FloatV zero = FZero();

	const Vec3V p0 = triangleV.verts[0];
	const Vec3V p1 = triangleV.verts[1];
	const Vec3V p2 = triangleV.verts[2];

	const Vec3V n = triangleV.normal();

	const FloatV sqInflatedRadius = FMul(inflatedRadius, inflatedRadius);
	
	FloatV t, u, v;
	const FloatV sqDist = pcmDistanceSegmentTriangleSquared(capsule.p0, capsule.p1, p0, p1, p2, t, u, v);
	
	if(FAllGrtr(sqInflatedRadius, sqDist))
	{
		Vec3V patchNormalInTriangle;
		if(selectNormal(u, v, trigFlag))
		{
			patchNormalInTriangle = n;
		}
		else
		{
			if(FAllEq(sqDist, zero))
			{
				//segment intersect with the triangle
				patchNormalInTriangle = n;
			}
			else
			{
				const Vec3V pq = V3Sub(capsule.p1, capsule.p0);
				const Vec3V pointOnSegment = V3ScaleAdd(pq, t, capsule.p0);
				const FloatV w = FSub(FOne(), FAdd(u, v));
				const Vec3V pointOnTriangle = V3ScaleAdd(p0, w, V3ScaleAdd(p1, u, V3Scale(p2, v)));
				patchNormalInTriangle = V3Normalize(V3Sub(pointOnSegment, pointOnTriangle));
			}
		}

		generateContacts(p0, p1, p2, n, patchNormalInTriangle, triangleIndex, capsule.p0, capsule.p1, inflatedRadius, manifoldContacts, numContacts);
	
		generateEEContactsMTD(p0, p1, p2, patchNormalInTriangle, triangleIndex, capsule.p0, capsule.p1, inflatedRadius, manifoldContacts, numContacts);
	}
	return true;
}

// PT: below is just for internal testing
PX_PHYSX_COMMON_API FloatV pcmDistanceSegmentTriangleSquaredExported(	const Vec3VArg p, const Vec3VArg q,
																		const Vec3VArg a, const Vec3VArg b, const Vec3VArg c,
																		FloatV& t, FloatV& u, FloatV& v)
{
	return pcmDistanceSegmentTriangleSquared(p, q, a, b, c, t, u, v);
}
