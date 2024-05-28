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

#include "GuOverlapTests.h"
#include "GuHeightFieldUtil.h"
#include "GuBoxConversion.h"
#include "GuInternal.h"
#include "GuVecConvexHull.h"
#include "GuEntityReport.h"
#include "GuDistancePointTriangle.h"
#include "GuIntersectionCapsuleTriangle.h"
#include "GuDistanceSegmentTriangle.h"
#include "GuBounds.h"
#include "GuBV4_Common.h"
#include "GuVecTriangle.h"
#include "GuConvexMesh.h"
#include "GuGJK.h"
#include "geometry/PxSphereGeometry.h"

using namespace physx;
using namespace Gu;
using namespace aos;

///////////////////////////////////////////////////////////////////////////////

namespace
{
	struct HeightfieldOverlapReport : Gu::OverlapReport
	{
		PX_NOCOPY(HeightfieldOverlapReport)
	public:
		HeightfieldOverlapReport(const PxHeightFieldGeometry& hfGeom, const PxTransform& hfPose) : mHfUtil(hfGeom), mHFPose(hfPose), mOverlap(PxIntFalse)	{}

		const HeightFieldUtil	mHfUtil;
		const PxTransform&		mHFPose;
		PxIntBool				mOverlap;
	};
}

bool GeomOverlapCallback_SphereHeightfield(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);
	PX_UNUSED(cache);
	PX_UNUSED(threadContext);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom1);

	struct SphereOverlapReport : HeightfieldOverlapReport
	{
		Sphere	mLocalSphere;

		SphereOverlapReport(const PxHeightFieldGeometry& hfGeom_, const PxTransform& hfPose, const PxVec3& localSphereCenter, float sphereRadius) : HeightfieldOverlapReport(hfGeom_, hfPose)
		{
			mLocalSphere.center = localSphereCenter;
			mLocalSphere.radius = sphereRadius * sphereRadius;
		}

		virtual bool reportTouchedTris(PxU32 nb, const PxU32* indices)
		{
			while(nb--)
			{
				const PxU32 triangleIndex = *indices++;

				PxTriangle currentTriangle;
				mHfUtil.getTriangle(mHFPose, currentTriangle, NULL, NULL, triangleIndex, false, false);

				const PxVec3& p0 = currentTriangle.verts[0];
				const PxVec3& p1 = currentTriangle.verts[1];
				const PxVec3& p2 = currentTriangle.verts[2];

				const PxVec3 edge10 = p1 - p0;
				const PxVec3 edge20 = p2 - p0;
				const PxVec3 cp = closestPtPointTriangle2(mLocalSphere.center, p0, p1, p2, edge10, edge20);
				const float sqrDist = (cp - mLocalSphere.center).magnitudeSquared();
				if(sqrDist <= mLocalSphere.radius)	// mLocalSphere.radius has been pre-squared in the ctor
				{
					mOverlap = PxIntTrue;
					return false;
				}
			}
			return true;
		}	
	};

	PxBounds3 localBounds;
	const PxVec3 localSphereCenter = getLocalSphereData(localBounds, pose0, pose1, sphereGeom.radius);

	SphereOverlapReport report(hfGeom, pose1, localSphereCenter, sphereGeom.radius);

	report.mHfUtil.overlapAABBTriangles(localBounds, report, 4);
	return report.mOverlap!=PxIntFalse;
}

///////////////////////////////////////////////////////////////////////////////

bool GeomOverlapCallback_CapsuleHeightfield(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCAPSULE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);
	PX_UNUSED(cache);
	PX_UNUSED(threadContext);

	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom0);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom1);

	struct CapsuleOverlapReport : HeightfieldOverlapReport
	{
		Capsule						mLocalCapsule;
		CapsuleTriangleOverlapData	mData;

		CapsuleOverlapReport(const PxHeightFieldGeometry& hfGeom_, const PxTransform& hfPose) : HeightfieldOverlapReport(hfGeom_, hfPose)	{}

		virtual bool reportTouchedTris(PxU32 nb, const PxU32* indices)
		{
			while(nb--)
			{
				const PxU32 triangleIndex = *indices++;

				PxTriangle currentTriangle;
				mHfUtil.getTriangle(mHFPose, currentTriangle, NULL, NULL, triangleIndex, false, false);

				const PxVec3& p0 = currentTriangle.verts[0];
				const PxVec3& p1 = currentTriangle.verts[1];
				const PxVec3& p2 = currentTriangle.verts[2];

				if(0)
				{
					PxReal t,u,v;
					const PxVec3 p1_p0 = p1 - p0;
					const PxVec3 p2_p0 = p2 - p0;
					const PxReal sqrDist = distanceSegmentTriangleSquared(mLocalCapsule, p0, p1_p0, p2_p0, &t, &u, &v);
					if(sqrDist <= mLocalCapsule.radius*mLocalCapsule.radius)
					{
						mOverlap = PxIntTrue;
						return false;
					}
				}
				else
				{
					const PxVec3 normal = (p0 - p1).cross(p0 - p2);
					if(intersectCapsuleTriangle(normal, p0, p1, p2, mLocalCapsule, mData))
					{
						mOverlap = PxIntTrue;
						return false;
					}
				}
			}
			return true;
		}	
	};

	CapsuleOverlapReport report(hfGeom, pose1);

	// PT: TODO: move away from internal header
	const PxVec3 tmp = getCapsuleHalfHeightVector(pose0, capsuleGeom);

	// PT: TODO: refactor - but might be difficult because we reuse relPose for two tasks here
	const PxTransform relPose = pose1.transformInv(pose0);
	const PxVec3 localDelta = pose1.rotateInv(tmp);

	report.mLocalCapsule.p0		= relPose.p + localDelta;
	report.mLocalCapsule.p1		= relPose.p - localDelta;
	report.mLocalCapsule.radius	= capsuleGeom.radius;
	report.mData.init(report.mLocalCapsule);

	PxBounds3 localBounds;
	computeCapsuleBounds(localBounds, capsuleGeom, relPose);

	report.mHfUtil.overlapAABBTriangles(localBounds, report, 4);
	//hfUtil.overlapAABBTriangles(pose0, pose1, getLocalCapsuleBounds(capsuleGeom.radius, capsuleGeom.halfHeight), report, 4);
	return report.mOverlap!=PxIntFalse;
}

///////////////////////////////////////////////////////////////////////////////

PxIntBool intersectTriangleBoxBV4(const PxVec3& p0, const PxVec3& p1, const PxVec3& p2,
									const PxMat33& rotModelToBox, const PxVec3& transModelToBox, const PxVec3& extents);

bool GeomOverlapCallback_BoxHeightfield(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eBOX);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);
	PX_UNUSED(cache);	
	PX_UNUSED(threadContext);

	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom0);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom1);

	struct BoxOverlapReport : HeightfieldOverlapReport
	{
		PxMat33		mRModelToBox;
		PxVec3p		mTModelToBox;
		PxVec3p		mBoxExtents;

		BoxOverlapReport(const PxHeightFieldGeometry& hfGeom_, const PxTransform& hfPose) : HeightfieldOverlapReport(hfGeom_, hfPose)	{}

		virtual bool reportTouchedTris(PxU32 nb, const PxU32* indices)
		{
			while(nb--)
			{
				const PxU32 triangleIndex = *indices++;

				PxTrianglePadded currentTriangle;
				mHfUtil.getTriangle(mHFPose, currentTriangle, NULL, NULL, triangleIndex, false, false);

				if(intersectTriangleBoxBV4(currentTriangle.verts[0], currentTriangle.verts[1], currentTriangle.verts[2], mRModelToBox, mTModelToBox, mBoxExtents))
				{
					mOverlap = PxIntTrue;
					return false;
				}
			}
			return true;
		}	
	};

	BoxOverlapReport report(hfGeom, pose1);

	// PT: TODO: revisit / refactor all this code
	const PxTransform relPose = pose1.transformInv(pose0);
	Box localBox;
	buildFrom(localBox, relPose.p, boxGeom.halfExtents, relPose.q);

	invertBoxMatrix(report.mRModelToBox, report.mTModelToBox, localBox);

	report.mBoxExtents = localBox.extents;

	PxBounds3 localBounds;
	{
		// PT: TODO: refactor with bounds code?
		const PxMat33& basis = localBox.rot;

			// extended basis vectors
			const Vec4V c0V = V4Scale(V4LoadU(&basis.column0.x), FLoad(localBox.extents.x));
			const Vec4V c1V = V4Scale(V4LoadU(&basis.column1.x), FLoad(localBox.extents.y));
			const Vec4V c2V = V4Scale(V4LoadU(&basis.column2.x), FLoad(localBox.extents.z));

			// find combination of base vectors that produces max. distance for each component = sum of abs()
			Vec4V extentsV = V4Add(V4Abs(c0V), V4Abs(c1V));
			extentsV = V4Add(extentsV, V4Abs(c2V));

		const PxVec3p origin(localBox.center);

		const Vec4V originV = V4LoadU(&origin.x);
		const Vec4V minV = V4Sub(originV, extentsV);
		const Vec4V maxV = V4Add(originV, extentsV);

		StoreBounds(localBounds, minV, maxV);
	}
	report.mHfUtil.overlapAABBTriangles(localBounds, report, 4);
	return report.mOverlap!=PxIntFalse;
}

///////////////////////////////////////////////////////////////////////////////

bool GeomOverlapCallback_ConvexHeightfield(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCONVEXMESH);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);
	PX_UNUSED(cache);	
	PX_UNUSED(threadContext);

	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom0);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom1);

	struct ConvexOverlapReport : HeightfieldOverlapReport
	{
		ConvexHullV		mConvex;
		PxMatTransformV	aToB;

		ConvexOverlapReport(const PxHeightFieldGeometry& hfGeom_, const PxTransform& hfPose) : HeightfieldOverlapReport(hfGeom_, hfPose)	{}

		virtual bool reportTouchedTris(PxU32 nb, const PxU32* indices)
		{
			while(nb--)
			{
				const PxU32 triangleIndex = *indices++;

				PxTrianglePadded currentTriangle;
				mHfUtil.getTriangle(mHFPose, currentTriangle, NULL, NULL, triangleIndex, false, false);

				const PxVec3& p0 = currentTriangle.verts[0];
				const PxVec3& p1 = currentTriangle.verts[1];
				const PxVec3& p2 = currentTriangle.verts[2];

				// PT: TODO: consider adding an extra triangle-vs-box culling test here

				// PT: TODO: optimize
				const Vec3V v0 = V3LoadU(p0);
				const Vec3V v1 = V3LoadU(p1);
				const Vec3V v2 = V3LoadU(p2);

				// PT: TODO: refactor with ConvexVsMeshOverlapCallback
				TriangleV triangle(v0, v1, v2);
				Vec3V contactA, contactB, normal;
				FloatV dist;
				const RelativeConvex<TriangleV> convexA(triangle, aToB);
				const LocalConvex<ConvexHullV> convexB(mConvex);
				const GjkStatus status = gjk(convexA, convexB, aToB.p, FZero(), contactA, contactB, normal, dist);
				if(status == GJK_CONTACT || status == GJK_CLOSE)// || FAllGrtrOrEq(mSqTolerance, sqDist))
				{
					mOverlap = PxIntTrue;
					return false;
				}
			}
			return true;
		}	
	};

	ConvexOverlapReport report(hfGeom, pose1);

	const ConvexMesh* cm = static_cast<const ConvexMesh*>(convexGeom.convexMesh);

	const bool idtScaleConvex = convexGeom.scale.isIdentity();

	{
		const ConvexHullData* hullData = &cm->getHull();

		const Vec3V vScale0 = V3LoadU_SafeReadW(convexGeom.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
		const QuatV vQuat0 = QuatVLoadU(&convexGeom.scale.rotation.x);

		report.mConvex = ConvexHullV(hullData, V3Zero(), vScale0, vQuat0, idtScaleConvex);
		// PT: TODO: is that transform correct? It looks like the opposite of what we do for other prims?
		report.aToB = PxMatTransformV(pose0.transformInv(pose1));
		//report.aToB = PxMatTransformV(pose1.transformInv(pose0));
	}

	const PxTransform relPose = pose1.transformInv(pose0);

	PxBounds3 localBounds;
	computeBounds(localBounds, convexGeom, relPose, 0.0f, 1.0f);

	report.mHfUtil.overlapAABBTriangles(localBounds, report, 4);
	return report.mOverlap!=PxIntFalse;
}
