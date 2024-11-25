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

#include "geometry/PxSphereGeometry.h"
#include "GuMidphaseInterface.h"
#include "CmScaling.h"
#include "GuSphere.h"
#include "GuInternal.h"
#include "GuConvexUtilsInternal.h"
#include "GuVecTriangle.h"
#include "GuVecConvexHull.h"
#include "GuConvexMesh.h"
#include "GuGJK.h"
#include "GuSweepSharedTests.h"
#include "CmMatrix34.h"

using namespace physx;
using namespace Cm;
using namespace Gu;
using namespace aos;

bool GeomOverlapCallback_SphereMesh(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eTRIANGLEMESH);
	PX_UNUSED(cache);
	PX_UNUSED(threadContext);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom1);	

	const Sphere worldSphere(pose0.p, sphereGeom.radius);

	TriangleMesh* meshData = static_cast<TriangleMesh*>(meshGeom.triangleMesh);
	return Midphase::intersectSphereVsMesh(worldSphere, *meshData, pose1, meshGeom.scale, NULL);
}

bool GeomOverlapCallback_CapsuleMesh(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCAPSULE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eTRIANGLEMESH);
	PX_UNUSED(cache);
	PX_UNUSED(threadContext);

	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom0);
	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom1);

	TriangleMesh* meshData = static_cast<TriangleMesh*>(meshGeom.triangleMesh);

	Capsule capsule;
	getCapsule(capsule, capsuleGeom, pose0);
	return Midphase::intersectCapsuleVsMesh(capsule, *meshData, pose1, meshGeom.scale, NULL);
}

bool GeomOverlapCallback_BoxMesh(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eBOX);
	PX_ASSERT(geom1.getType()==PxGeometryType::eTRIANGLEMESH);
	PX_UNUSED(cache);
	PX_UNUSED(threadContext);

	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom0);
	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom1);

	TriangleMesh* meshData = static_cast<TriangleMesh*>(meshGeom.triangleMesh);

	Box box;
	buildFrom(box, pose0.p, boxGeom.halfExtents, pose0.q);
	return Midphase::intersectBoxVsMesh(box, *meshData, pose1, meshGeom.scale, NULL);
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
struct ConvexVsMeshOverlapCallback : MeshHitCallback<PxGeomRaycastHit>
{
	PxMatTransformV MeshToBoxV;
	Vec3V boxExtents;

	ConvexVsMeshOverlapCallback(
		const ConvexMesh& cm, const PxMeshScale& convexScale, const FastVertex2ShapeScaling& meshScale,
		const PxTransform& tr0, const PxTransform& tr1, bool identityScale, const Box& meshSpaceOBB)
		:
			MeshHitCallback<PxGeomRaycastHit>(CallbackMode::eMULTIPLE),
			mAnyHit			(false),
			mIdentityScale	(identityScale)
	{
		if (!identityScale) // not done in initializer list for performance
			mMeshScale = aos::Mat33V(
				V3LoadU(meshScale.getVertex2ShapeSkew().column0),
				V3LoadU(meshScale.getVertex2ShapeSkew().column1),
				V3LoadU(meshScale.getVertex2ShapeSkew().column2) );
		using namespace aos;

		const ConvexHullData* hullData = &cm.getHull();

		const Vec3V vScale0 = V3LoadU_SafeReadW(convexScale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
		const QuatV vQuat0 = QuatVLoadU(&convexScale.rotation.x);

		mConvex =  ConvexHullV(hullData, V3Zero(), vScale0, vQuat0, convexScale.isIdentity());
		aToB = PxMatTransformV(tr0.transformInv(tr1));
		
		{
			// Move to AABB space
			PxMat34 MeshToBox;
			computeWorldToBoxMatrix(MeshToBox, meshSpaceOBB);

			const Vec3V base0 = V3LoadU(MeshToBox.m.column0);
			const Vec3V base1 = V3LoadU(MeshToBox.m.column1);
			const Vec3V base2 = V3LoadU(MeshToBox.m.column2);
			const Mat33V matV(base0, base1, base2);
			const Vec3V p  = V3LoadU(MeshToBox.p);
			MeshToBoxV = PxMatTransformV(p, matV);
			boxExtents = V3LoadU(meshSpaceOBB.extents+PxVec3(0.001f));
		}
	}
	virtual ~ConvexVsMeshOverlapCallback()	{}

	virtual PxAgain processHit( // all reported coords are in mesh local space including hit.position
		const PxGeomRaycastHit&, const PxVec3& v0a, const PxVec3& v1a, const PxVec3& v2a, PxReal&, const PxU32*)
	{
		using namespace aos;
		Vec3V v0 = V3LoadU(v0a);
		Vec3V v1 = V3LoadU(v1a);
		Vec3V v2 = V3LoadU(v2a);

		// test triangle AABB in box space vs box AABB in box local space
		{
			const Vec3V triV0 = MeshToBoxV.transform(v0); // AP: MeshToBoxV already includes mesh scale so we have to use unscaled verts here
			const Vec3V triV1 = MeshToBoxV.transform(v1);
			const Vec3V triV2 = MeshToBoxV.transform(v2);
			const Vec3V triMn = V3Min(V3Min(triV0, triV1), triV2);
			const Vec3V triMx = V3Max(V3Max(triV0, triV1), triV2);
			const Vec3V negExtents = V3Neg(boxExtents);
			const BoolV minSeparated = V3IsGrtr(triMn, boxExtents), maxSeparated = V3IsGrtr(negExtents, triMx);
			const BoolV bSeparated = BAnyTrue3(BOr(minSeparated, maxSeparated));
			if(BAllEqTTTT(bSeparated))
				return true; // continue traversal
		}

		if(!mIdentityScale)
		{
			v0 = M33MulV3(mMeshScale, v0);
			v1 = M33MulV3(mMeshScale, v1);
			v2 = M33MulV3(mMeshScale, v2);
		}

		TriangleV triangle(v0, v1, v2);
		Vec3V contactA, contactB, normal;
		FloatV dist;
		const RelativeConvex<TriangleV> convexA(triangle, aToB);
		const LocalConvex<ConvexHullV> convexB(mConvex);
		const GjkStatus status = gjk(convexA, convexB, aToB.p, FZero(), contactA, contactB, normal, dist);
		if(status == GJK_CONTACT || status == GJK_CLOSE)// || FAllGrtrOrEq(mSqTolerance, sqDist))
		{
			mAnyHit = true;
			return false; // abort traversal
		}
		return true; // continue traversal
	}
	
	ConvexHullV		mConvex;
	PxMatTransformV	aToB;
	aos::Mat33V		mMeshScale;
	bool			mAnyHit;
	const bool		mIdentityScale;

private:
	ConvexVsMeshOverlapCallback& operator=(const ConvexVsMeshOverlapCallback&);
};
}

// PT: TODO: refactor bits of this with convex-vs-mesh code
bool GeomOverlapCallback_ConvexMesh(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCONVEXMESH);
	PX_ASSERT(geom1.getType()==PxGeometryType::eTRIANGLEMESH);
	PX_UNUSED(cache);
	PX_UNUSED(threadContext);

	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom0);
	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom1);

	ConvexMesh* cm = static_cast<ConvexMesh*>(convexGeom.convexMesh);
	TriangleMesh* meshData = static_cast<TriangleMesh*>(meshGeom.triangleMesh);

	const bool idtScaleConvex = convexGeom.scale.isIdentity();
	const bool idtScaleMesh = meshGeom.scale.isIdentity();

	FastVertex2ShapeScaling convexScaling;
	if (!idtScaleConvex)
		convexScaling.init(convexGeom.scale);

	FastVertex2ShapeScaling meshScaling;
	if (!idtScaleMesh)
		meshScaling.init(meshGeom.scale);

	PX_ASSERT(!cm->getLocalBoundsFast().isEmpty());
	const PxBounds3 hullAABB = cm->getLocalBoundsFast().transformFast(convexScaling.getVertex2ShapeSkew());

	Box hullOBB;
	{
		const Matrix34FromTransform world0(pose0);
		const Matrix34FromTransform world1(pose1);
		computeHullOBB(hullOBB, hullAABB, 0.0f, world0, world1, meshScaling, idtScaleMesh);
	}

	ConvexVsMeshOverlapCallback cb(*cm, convexGeom.scale, meshScaling, pose0, pose1, idtScaleMesh, hullOBB);
	Midphase::intersectOBB(meshData, hullOBB, cb, true, false);

	return cb.mAnyHit;
}

///////////////////////////////////////////////////////////////////////////////

bool GeomOverlapCallback_MeshMesh(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eTRIANGLEMESH);
	PX_ASSERT(geom1.getType()==PxGeometryType::eTRIANGLEMESH);
	PX_UNUSED(cache);
	PX_UNUSED(threadContext);

	const PxTriangleMeshGeometry& meshGeom0 = static_cast<const PxTriangleMeshGeometry&>(geom0);
	const PxTriangleMeshGeometry& meshGeom1 = static_cast<const PxTriangleMeshGeometry&>(geom1);

	const TriangleMesh* tm0 = static_cast<const TriangleMesh*>(meshGeom0.triangleMesh);
	const TriangleMesh* tm1 = static_cast<const TriangleMesh*>(meshGeom1.triangleMesh);

	// PT: only implemented for BV4
	if(!tm0 || !tm1 || tm0->getConcreteType()!=PxConcreteType::eTRIANGLE_MESH_BVH34 || tm1->getConcreteType()!=PxConcreteType::eTRIANGLE_MESH_BVH34)
		return PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxGeometryQuery::overlap(): only available between two BVH34 triangles meshes.");

	class AnyHitReportCallback : public PxReportCallback<PxGeomIndexPair>
	{
		public:
		AnyHitReportCallback()
		{
			mCapacity = 1;
		}

		virtual	bool	flushResults(PxU32, const PxGeomIndexPair*)
		{
			return false;
		}
	};

	AnyHitReportCallback callback;

	// PT: ...so we don't need a table like for the other ops, just go straight to BV4
	return intersectMeshVsMesh_BV4(callback, *tm0, pose0, meshGeom0.scale, *tm1, pose1, meshGeom1.scale, PxMeshMeshQueryFlag::eDEFAULT, 0.0f);
}
