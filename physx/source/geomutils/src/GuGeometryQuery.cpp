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

#include "geometry/PxGeometryQuery.h"
#include "geometry/PxSphereGeometry.h"
#include "geometry/PxBoxGeometry.h"
#include "geometry/PxPlaneGeometry.h"
#include "geometry/PxCapsuleGeometry.h"
#include "geometry/PxTriangleMeshGeometry.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "geometry/PxHeightFieldGeometry.h"
#include "geometry/PxParticleSystemGeometry.h"
#include "geometry/PxCustomGeometry.h"
#include "geometry/PxConvexCoreGeometry.h"
#include "foundation/PxAtomic.h"

#include "GuInternal.h"
#include "GuOverlapTests.h"
#include "GuSweepTests.h"
#include "GuRaycastTests.h"
#include "GuBoxConversion.h"
#include "GuTriangleMesh.h"
#include "GuMTD.h"
#include "GuBounds.h"
#include "GuDistancePointSegment.h"
#include "GuConvexMesh.h"
#include "GuDistancePointBox.h"
#include "GuMidphaseInterface.h"
#include "foundation/PxFPU.h"

#include "GuConvexEdgeFlags.h"
#include "GuVecBox.h"
#include "GuVecConvexHull.h"
#include "GuPCMShapeConvex.h"
#include "GuPCMContactConvexCommon.h"

using namespace physx;
using namespace Gu;

extern GeomSweepFuncs gGeomSweepFuncs;
extern GeomOverlapTable gGeomOverlapMethodTable[];
extern RaycastFunc gRaycastMap[PxGeometryType::eGEOMETRY_COUNT];

///////////////////////////////////////////////////////////////////////////////

bool PxGeometryQuery::isValid(const PxGeometry& g)
{
	switch(PxU32(g.getType()))
	{
		case PxGeometryType::eSPHERE:			return static_cast<const PxSphereGeometry&>(g).isValid();
		case PxGeometryType::ePLANE:			return static_cast<const PxPlaneGeometry&>(g).isValid();
		case PxGeometryType::eCAPSULE:			return static_cast<const PxCapsuleGeometry&>(g).isValid();
		case PxGeometryType::eBOX:				return static_cast<const PxBoxGeometry&>(g).isValid();
		case PxGeometryType::eCONVEXCORE:		return static_cast<const PxConvexCoreGeometry&>(g).isValid();
		case PxGeometryType::eCONVEXMESH:		return static_cast<const PxConvexMeshGeometry&>(g).isValid();
		case PxGeometryType::eTRIANGLEMESH:		return static_cast<const PxTriangleMeshGeometry&>(g).isValid();
		case PxGeometryType::eHEIGHTFIELD:		return static_cast<const PxHeightFieldGeometry&>(g).isValid();
		case PxGeometryType::eTETRAHEDRONMESH:	return static_cast<const PxTetrahedronMeshGeometry&>(g).isValid();
		case PxGeometryType::ePARTICLESYSTEM:	return static_cast<const PxParticleSystemGeometry&>(g).isValid();
		case PxGeometryType::eCUSTOM:			return static_cast<const PxCustomGeometry&>(g).isValid();
	}
	return false;
}

///////////////////////////////////////////////////////////////////////////////

bool PxGeometryQuery::sweep(const PxVec3& unitDir, const PxReal distance,
							const PxGeometry& geom0, const PxTransform& pose0,
							const PxGeometry& geom1, const PxTransform& pose1,
							PxGeomSweepHit& sweepHit, PxHitFlags hitFlags,
							const PxReal inflation, PxGeometryQueryFlags queryFlags, PxSweepThreadContext* threadContext)
{
	PX_SIMD_GUARD_CNDT(queryFlags & PxGeometryQueryFlag::eSIMD_GUARD)
	PX_CHECK_AND_RETURN_VAL(pose0.isValid(), "PxGeometryQuery::sweep(): pose0 is not valid.", false);
	PX_CHECK_AND_RETURN_VAL(pose1.isValid(), "PxGeometryQuery::sweep(): pose1 is not valid.", false);
	PX_CHECK_AND_RETURN_VAL(unitDir.isFinite(), "PxGeometryQuery::sweep(): unitDir is not valid.", false);
	PX_CHECK_AND_RETURN_VAL(PxIsFinite(distance), "PxGeometryQuery::sweep(): distance is not valid.", false);
	PX_CHECK_AND_RETURN_VAL((distance >= 0.0f && !(hitFlags & PxHitFlag::eASSUME_NO_INITIAL_OVERLAP)) || distance > 0.0f,
		"PxGeometryQuery::sweep(): sweep distance must be >=0 or >0 with eASSUME_NO_INITIAL_OVERLAP.", 0);
#if PX_CHECKED
	if(!PxGeometryQuery::isValid(geom0))
		return PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "Provided geometry 0 is not valid");

	if(!PxGeometryQuery::isValid(geom1))
		return PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "Provided geometry 1 is not valid");
#endif

	const GeomSweepFuncs& sf = gGeomSweepFuncs;

	switch(geom0.getType())
	{
		case PxGeometryType::eSPHERE:
		{
			const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);

			const PxCapsuleGeometry capsuleGeom(sphereGeom.radius, 0.0f);

			const Capsule worldCapsule(pose0.p, pose0.p, sphereGeom.radius);

			const bool precise = hitFlags & PxHitFlag::ePRECISE_SWEEP;
			const SweepCapsuleFunc func = precise ? sf.preciseCapsuleMap[geom1.getType()] : sf.capsuleMap[geom1.getType()];

			return func(geom1, pose1, capsuleGeom, pose0, worldCapsule, unitDir, distance, sweepHit, hitFlags, inflation, threadContext);
		}

		case PxGeometryType::eCAPSULE:
		{
			const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom0);

			Capsule worldCapsule;
			getCapsule(worldCapsule, capsuleGeom, pose0);

			const bool precise = hitFlags & PxHitFlag::ePRECISE_SWEEP;
			const SweepCapsuleFunc func = precise ? sf.preciseCapsuleMap[geom1.getType()] : sf.capsuleMap[geom1.getType()];

			return func(geom1, pose1, capsuleGeom, pose0, worldCapsule, unitDir, distance, sweepHit, hitFlags, inflation, threadContext);
		}

		case PxGeometryType::eBOX:
		{
			const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom0);

			Box box;
			buildFrom(box, pose0.p, boxGeom.halfExtents, pose0.q);

			const bool precise = hitFlags & PxHitFlag::ePRECISE_SWEEP;
			const SweepBoxFunc func = precise ? sf.preciseBoxMap[geom1.getType()] : sf.boxMap[geom1.getType()];

			return func(geom1, pose1, boxGeom, pose0, box, unitDir, distance, sweepHit, hitFlags, inflation, threadContext);
		}

		case PxGeometryType::eCONVEXMESH:
		{
			const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom0);

			const SweepConvexFunc func = sf.convexMap[geom1.getType()];

			return func(geom1, pose1, convexGeom, pose0, unitDir, distance, sweepHit, hitFlags, inflation, threadContext);
		}
		default:
			PX_CHECK_MSG(false, "PxGeometryQuery::sweep(): first geometry object parameter must be sphere, capsule, box or convex geometry.");
	}

	return false;
}

///////////////////////////////////////////////////////////////////////////////

bool PxGeometryQuery::overlap(	const PxGeometry& geom0, const PxTransform& pose0,
								const PxGeometry& geom1, const PxTransform& pose1,
								PxGeometryQueryFlags queryFlags, PxOverlapThreadContext* threadContext)
{
	PX_SIMD_GUARD_CNDT(queryFlags & PxGeometryQueryFlag::eSIMD_GUARD)
	return Gu::overlap(geom0, pose0, geom1, pose1, gGeomOverlapMethodTable, threadContext);
}

///////////////////////////////////////////////////////////////////////////////

PxU32 PxGeometryQuery::raycast(	const PxVec3& rayOrigin, const PxVec3& rayDir,
								const PxGeometry& geom, const PxTransform& pose,
								PxReal maxDist, PxHitFlags hitFlags, PxU32 maxHits, PxGeomRaycastHit* PX_RESTRICT rayHits, PxU32 stride,
								PxGeometryQueryFlags queryFlags, PxRaycastThreadContext* threadContext)
{
	PX_SIMD_GUARD_CNDT(queryFlags & PxGeometryQueryFlag::eSIMD_GUARD)
	PX_CHECK_AND_RETURN_VAL(rayDir.isFinite(), "PxGeometryQuery::raycast(): rayDir is not valid.", 0);
	PX_CHECK_AND_RETURN_VAL(rayOrigin.isFinite(), "PxGeometryQuery::raycast(): rayOrigin is not valid.", 0);
	PX_CHECK_AND_RETURN_VAL(pose.isValid(), "PxGeometryQuery::raycast(): pose is not valid.", 0);
	PX_CHECK_AND_RETURN_VAL(maxDist >= 0.0f, "PxGeometryQuery::raycast(): maxDist is negative.", false);
	PX_CHECK_AND_RETURN_VAL(PxIsFinite(maxDist), "PxGeometryQuery::raycast(): maxDist is not valid.", false);
	PX_CHECK_AND_RETURN_VAL(PxAbs(rayDir.magnitudeSquared()-1)<1e-4f, "PxGeometryQuery::raycast(): ray direction must be unit vector.", false);

	const RaycastFunc func = gRaycastMap[geom.getType()];
	return func(geom, pose, rayOrigin, rayDir, maxDist, hitFlags, maxHits, rayHits, stride, threadContext);
}

///////////////////////////////////////////////////////////////////////////////

bool pointConvexDistance(PxVec3& normal_, PxVec3& closestPoint_, PxReal& sqDistance, const PxVec3& pt, const ConvexMesh* convexMesh, const PxMeshScale& meshScale, const PxTransform32& convexPose);

PxReal PxGeometryQuery::pointDistance(const PxVec3& point, const PxGeometry& geom, const PxTransform& pose, PxVec3* closestPoint, PxU32* closestIndex, PxGeometryQueryFlags queryFlags)
{
	PX_SIMD_GUARD_CNDT(queryFlags & PxGeometryQueryFlag::eSIMD_GUARD)
	PX_CHECK_AND_RETURN_VAL(pose.isValid(), "PxGeometryQuery::pointDistance(): pose is not valid.", -1.0f);

	switch(geom.getType())
	{
		case PxGeometryType::eSPHERE:
		{
			const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);

			const PxReal r = sphereGeom.radius;

			PxVec3 delta = point - pose.p;
			const PxReal d = delta.magnitude();
			if(d<=r)
				return 0.0f;

			if(closestPoint)
			{
				delta /= d;
				*closestPoint = pose.p + delta * r;
			}

			return (d - r)*(d - r);
		}
		case PxGeometryType::eCAPSULE:
		{
			const PxCapsuleGeometry& capsGeom = static_cast<const PxCapsuleGeometry&>(geom);

			Capsule capsule;
			getCapsule(capsule, capsGeom, pose);

			const PxReal r = capsGeom.radius;

			PxReal param;
			const PxReal sqDistance = distancePointSegmentSquared(capsule, point, &param);
			if(sqDistance<=r*r)
				return 0.0f;

			const PxReal d = physx::intrinsics::sqrt(sqDistance);

			if(closestPoint)
			{
				const PxVec3 cp = capsule.getPointAt(param);

				PxVec3 delta = point - cp;
				delta.normalize();

				*closestPoint = cp + delta * r;
			}
			return (d - r)*(d - r);
		}
		case PxGeometryType::eBOX:
		{
			const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);

			Box obb;
			buildFrom(obb, pose.p, boxGeom.halfExtents, pose.q);

			PxVec3 boxParam;
			const PxReal sqDistance = distancePointBoxSquared(point, obb, &boxParam);
			if(closestPoint && sqDistance!=0.0f)
			{
				*closestPoint = obb.transform(boxParam);
			}
			return sqDistance;
		}
		case PxGeometryType::eCONVEXMESH:
		{
			const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom);

			const PxTransform32 poseA(pose);

			PxVec3 normal, cp;
			PxReal sqDistance;
			const bool intersect = pointConvexDistance(normal, cp, sqDistance, point, static_cast<ConvexMesh*>(convexGeom.convexMesh), convexGeom.scale, poseA);
			if(!intersect && closestPoint)
				*closestPoint = cp;
			return sqDistance;
		}
		case PxGeometryType::eTRIANGLEMESH:
		{
			const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom);

			PxU32 index;
			float dist;
			PxVec3 cp;
			Midphase::pointMeshDistance(static_cast<TriangleMesh*>(meshGeom.triangleMesh), meshGeom, pose, point, FLT_MAX, index, dist, cp);
			if(closestPoint)
				*closestPoint = cp;
			if(closestIndex)
				*closestIndex = index;
			return dist*dist;
		}
		default:
			PX_CHECK_MSG(false, "PxGeometryQuery::pointDistance(): geometry object parameter must be sphere, capsule, box, convex or mesh geometry.");
			break;
	}
	return -1.0f;
}

///////////////////////////////////////////////////////////////////////////////

void PxGeometryQuery::computeGeomBounds(PxBounds3& bounds, const PxGeometry& geom, const PxTransform& pose, float offset, float inflation, PxGeometryQueryFlags queryFlags)
{
	PX_SIMD_GUARD_CNDT(queryFlags & PxGeometryQueryFlag::eSIMD_GUARD)
	PX_CHECK_AND_RETURN(pose.isValid(), "PxGeometryQuery::computeGeomBounds(): pose is not valid.");

	Gu::computeBounds(bounds, geom, pose, offset, inflation);
	PX_ASSERT(bounds.isValid());
}

///////////////////////////////////////////////////////////////////////////////

extern GeomMTDFunc gGeomMTDMethodTable[][PxGeometryType::eGEOMETRY_COUNT];

bool PxGeometryQuery::computePenetration(	PxVec3& mtd, PxF32& depth,
											const PxGeometry& geom0, const PxTransform& pose0,
											const PxGeometry& geom1, const PxTransform& pose1, PxGeometryQueryFlags queryFlags)
{
	PX_SIMD_GUARD_CNDT(queryFlags & PxGeometryQueryFlag::eSIMD_GUARD)
	PX_CHECK_AND_RETURN_VAL(pose0.isValid(), "PxGeometryQuery::computePenetration(): pose0 is not valid.", false);
	PX_CHECK_AND_RETURN_VAL(pose1.isValid(), "PxGeometryQuery::computePenetration(): pose1 is not valid.", false);

	const PxTransform32 pose0A(pose0);
	const PxTransform32 pose1A(pose1);

	if(geom0.getType() > geom1.getType())
	{
		GeomMTDFunc mtdFunc = gGeomMTDMethodTable[geom1.getType()][geom0.getType()];
		PX_ASSERT(mtdFunc);
		if(!mtdFunc(mtd, depth, geom1, pose1A, geom0, pose0A))
			return false;
		mtd = -mtd;
		return true;
	}
	else
	{
		GeomMTDFunc mtdFunc = gGeomMTDMethodTable[geom0.getType()][geom1.getType()];
		PX_ASSERT(mtdFunc);
		return mtdFunc(mtd, depth, geom0, pose0A, geom1, pose1A);
	}
}

///////////////////////////////////////////////////////////////////////////////

bool PxGeometryQuery::generateTriangleContacts(const PxGeometry& geom, const PxTransform& pose, const PxVec3 triangleVertices[3], PxU32 triangleIndex, PxReal contactDistance, PxReal meshContactMargin, PxReal toleranceLength, PxContactBuffer& contactBuffer)
{
	using namespace aos;

	const PxU32 triangleIndices[3]{ 0, 1, 2 };
	PxInlineArray<PxU32, LOCAL_PCM_CONTACTS_SIZE> deferredContacts;
	Gu::MultiplePersistentContactManifold multiManifold;
	multiManifold.initialize();
	PxContactBuffer contactBuffer0; contactBuffer0.reset();
	const PxTransformV geomTransform = loadTransformU(pose);
	const PxTransformV triangleTransform = loadTransformU(PxTransform(PxIdentity));
	float radius0 = 0;
	float radius1 = meshContactMargin;

	PxU32 oldCount = contactBuffer.count;

	switch (geom.getType())
	{
		case PxGeometryType::eCAPSULE:
		{
			const PxCapsuleGeometry& capsule = static_cast<const PxCapsuleGeometry&>(geom);
			radius0 = capsule.radius;

			const FloatV capsuleRadius = FLoad(capsule.radius);
			const FloatV contactDist = FLoad(contactDistance + meshContactMargin);
			const FloatV replaceBreakingThreshold = FMul(capsuleRadius, FLoad(0.001f));

			const PxTransformV capsuleTransform = geomTransform;
			const PxTransformV meshTransform = triangleTransform;

			multiManifold.setRelativeTransform(capsuleTransform);

			const Gu::CapsuleV capsuleV(V3LoadU(pose.p), V3LoadU(pose.q.rotate(PxVec3(capsule.halfHeight, 0, 0))), capsuleRadius);

			Gu::PCMCapsuleVsMeshContactGeneration contactGeneration(capsuleV, contactDist, replaceBreakingThreshold, capsuleTransform, meshTransform, multiManifold, contactBuffer0, &deferredContacts);
			contactGeneration.processTriangle(triangleVertices, triangleIndex, Gu::ETD_CONVEX_EDGE_ALL, triangleIndices);
			contactGeneration.processContacts(GU_CAPSULE_MANIFOLD_CACHE_SIZE, false);

			break;
		}
		case PxGeometryType::eBOX:
		{
			const PxBoxGeometry& box = static_cast<const PxBoxGeometry&>(geom);

			const PxBounds3 hullAABB(-box.halfExtents, box.halfExtents);
			const Vec3V boxExtents = V3LoadU(box.halfExtents);
			const FloatV minMargin = Gu::CalculatePCMBoxMargin(boxExtents, toleranceLength, GU_PCM_MESH_MANIFOLD_EPSILON);

			Cm::FastVertex2ShapeScaling idtScaling;

			const FloatV contactDist = FLoad(contactDistance + meshContactMargin);
			const FloatV replaceBreakingThreshold = FMul(minMargin, FLoad(0.05f));

			const BoxV boxV(V3Zero(), boxExtents);

			const PxTransformV boxTransform = geomTransform;
			const PxTransformV meshTransform = triangleTransform;

			PolygonalData polyData;
			PCMPolygonalBox polyBox(box.halfExtents);
			polyBox.getPolygonalData(&polyData);

			const Mat33V identity = M33Identity();
			SupportLocalImpl<BoxV> boxMap(boxV, boxTransform, identity, identity, true);

			Gu::PCMConvexVsMeshContactGeneration contactGeneration(contactDist, replaceBreakingThreshold, boxTransform, meshTransform, multiManifold, contactBuffer0, polyData, &boxMap, &deferredContacts, idtScaling, true, true, NULL);
			contactGeneration.processTriangle(triangleVertices, triangleIndex, Gu::ETD_CONVEX_EDGE_ALL, triangleIndices);
			contactGeneration.generateLastContacts();
			contactGeneration.processContacts(GU_SINGLE_MANIFOLD_CACHE_SIZE, false);

			break;
		}
		case PxGeometryType::eCONVEXMESH:
		{
			const PxConvexMeshGeometry& convex = static_cast<const PxConvexMeshGeometry&>(geom);

			const ConvexHullData* hullData = _getHullData(convex);

			Cm::FastVertex2ShapeScaling convexScaling;
			PxBounds3 hullAABB;
			PolygonalData polyData;
			const bool idtConvexScale = getPCMConvexData(convex, convexScaling, hullAABB, polyData);
			const QuatV vQuat = QuatVLoadU(&convex.scale.rotation.x);
			const Vec3V vScale = V3LoadU_SafeReadW(convex.scale.scale);
			const FloatV minMargin = CalculatePCMConvexMargin(hullData, vScale, toleranceLength, GU_PCM_MESH_MANIFOLD_EPSILON);

			const ConvexHullV convexHull(hullData, V3Zero(), vScale, vQuat, idtConvexScale);

			const FloatV contactDist = FLoad(contactDistance + meshContactMargin);
			const FloatV replaceBreakingThreshold = FMul(minMargin, FLoad(0.05f));

			const PxTransformV convexTransform = geomTransform;
			const PxTransformV meshTransform = triangleTransform;

			SupportLocalImpl<Gu::ConvexHullV> convexMap(convexHull, convexTransform, convexHull.vertex2Shape, convexHull.shape2Vertex, false);

			Gu::PCMConvexVsMeshContactGeneration contactGeneration(contactDist, replaceBreakingThreshold, convexTransform, meshTransform, multiManifold, contactBuffer0, polyData, &convexMap, &deferredContacts, convexScaling, idtConvexScale, true, NULL);
			contactGeneration.processTriangle(triangleVertices, triangleIndex, Gu::ETD_CONVEX_EDGE_ALL, triangleIndices);
			contactGeneration.generateLastContacts();
			contactGeneration.processContacts(GU_SINGLE_MANIFOLD_CACHE_SIZE, false);

			break;
		}
		default:
			PX_ASSERT(0); // Unsupported gepmetry type
			break;
	}

	for (PxU32 manifoldIndex = 0; manifoldIndex < multiManifold.mNumManifolds; ++manifoldIndex)
	{
		Gu::SinglePersistentContactManifold& manifold = *multiManifold.getManifold(manifoldIndex);
		PxVec3 normal; V3StoreU(manifold.getWorldNormal(triangleTransform), normal);
		for (PxU32 contactIndex = 0; contactIndex < manifold.getNumContacts(); ++contactIndex)
		{
			Gu::MeshPersistentContact& meshContact = manifold.getContactPoint(contactIndex);
			PxContactPoint contact;
			PxVec3 p0; V3StoreU(geomTransform.transform(meshContact.mLocalPointA), p0); p0 -= normal * radius0;
			PxVec3 p1; V3StoreU(meshContact.mLocalPointB, p1); p1 += normal * radius1;
			contact.point = (p0 + p1) * 0.5f;
			contact.normal = normal;
			contact.separation = normal.dot(p0 - p1);
			contact.internalFaceIndex1 = triangleIndex;
			contactBuffer.contact(contact);
		}
	}

	return oldCount < contactBuffer.count;
}

///////////////////////////////////////////////////////////////////////////////

PxU32 PxCustomGeometry::getUniqueID()
{
    static PxU32 uniqueID(0);
    PxAtomicIncrement(reinterpret_cast<volatile PxI32*>(&uniqueID));
    return uniqueID;
}
