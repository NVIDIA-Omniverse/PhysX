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

#include "GuVecCapsule.h"
#include "GuVecBox.h"
#include "GuVecConvexHull.h"
#include "GuVecTriangle.h"
#include "GuGJKRaycast.h"
#include "GuCCDSweepConvexMesh.h"
#include "GuHeightFieldUtil.h"
#include "foundation/PxInlineArray.h"
#include "GuEntityReport.h"
#include "PxContact.h"
#include "GuDistancePointTriangle.h"
#include "GuBox.h"
#include "GuInternal.h"
#include "GuBoxConversion.h"
#include "GuConvexUtilsInternal.h"
#include "GuMidphaseInterface.h"
#include "geometry/PxGeometryQuery.h"

// PT: this one makes the "behavior after impact" PEEL test "fail" (rocks stop after impact)
// It also makes these UTs fail:
// [  FAILED  ] CCDReportTest.CCD_soakTest_mesh
// [  FAILED  ] CCDNegativeScalingTest.SLOW_ccdNegScaledMesh
static const bool gUseGeometryQuery = false;

// PT: this one seems to work.
// Timings for PEEL's "limits of speculative contacts test2", for 3 runs:
// false:		true:
// Time: 504	220
// Time: 89		7
// Time: 5		84
// Time: 8		11
// Time: 423	56
// Time: 103	14
// Time: 10		11
// Time: 10		9
// Time: 418	60
// Time: 139	17
// Time: 9		9
// Time: 9		10
static const bool gUseGeometryQueryEst = false;

//#define CCD_BASIC_PROFILING
#ifdef CCD_BASIC_PROFILING
	#include <stdio.h>
#endif

namespace physx
{
namespace Gu
{

PxReal SweepShapeTriangle(GU_TRIANGLE_SWEEP_METHOD_ARGS);

using namespace aos;

namespace 
{
struct AccumCallback: public MeshHitCallback<PxGeomRaycastHit>
{
	PX_NOCOPY(AccumCallback)
	public:

	PxInlineArray<PxU32, 64>& mResult;
	AccumCallback(PxInlineArray<PxU32, 64>& result)
		:	MeshHitCallback<PxGeomRaycastHit>(CallbackMode::eMULTIPLE),
			mResult(result)
	{
	}

	virtual PxAgain processHit( // all reported coords are in mesh local space including hit.position
		const PxGeomRaycastHit& hit, const PxVec3&, const PxVec3&, const PxVec3&, PxReal&, const PxU32*)	PX_OVERRIDE PX_FINAL
	{
		mResult.pushBack(hit.faceIndex);
		return true;
	}
};

// PT: TODO: refactor with MidPhaseQueryLocalReport
struct EntityReportContainerCallback : public OverlapReport
{
	PxInlineArray<PxU32, 64>& container;
	EntityReportContainerCallback(PxInlineArray<PxU32,64>& container_) : container(container_)
	{
		container.forceSize_Unsafe(0);
	}
	virtual ~EntityReportContainerCallback() {}

	virtual bool reportTouchedTris(PxU32 nb, const PxU32* indices)	PX_OVERRIDE PX_FINAL
	{
		for(PxU32 i=0; i<nb; i++)
			container.pushBack(indices[i]);
		return true;
	}
private:
	EntityReportContainerCallback& operator=(const EntityReportContainerCallback&);
};

class TriangleHelper
{
public:
										TriangleHelper(const PxTriangleMeshGeometry& shapeMesh, 
														   const Cm::FastVertex2ShapeScaling& skew, // object is not copied, beware!
														   const PxU32 triangleIndex);

					void				getBounds(PxBounds3& bounds, const physx::PxTransform& transform)	const;

	//non-virtuals:
	PX_FORCE_INLINE	const TriangleMesh*	getMeshData()		const			{ return _getMeshData(mShapeMesh); }

					PxVec3				getPolygonNormal()	const;
private:
	TriangleHelper& operator=(const TriangleHelper&);

	const PxTriangleMeshGeometry&		mShapeMesh;
	const Cm::FastVertex2ShapeScaling&	mVertex2ShapeSkew;
	const PxU32							mTriangleIndex;
};

TriangleHelper::TriangleHelper(const PxTriangleMeshGeometry& md, const Cm::FastVertex2ShapeScaling& skew, const PxU32 tg) 
: mShapeMesh(md), mVertex2ShapeSkew(skew), mTriangleIndex(tg)
{
}

void TriangleHelper::getBounds(PxBounds3& bounds, const physx::PxTransform& transform) const
{
	PxTriangle localTri;
	getMeshData()->getLocalTriangle(localTri, mTriangleIndex, false);	// PT: 'false': no need to flip winding to compute bounds

	//gotta take bounds in shape space because building it in vertex space and transforming it out would skew it.	
	bounds = PxBounds3::empty();
	bounds.include(transform.transform(mVertex2ShapeSkew * localTri.verts[0]));
	bounds.include(transform.transform(mVertex2ShapeSkew * localTri.verts[1]));
	bounds.include(transform.transform(mVertex2ShapeSkew * localTri.verts[2]));
}

PxVec3 TriangleHelper::getPolygonNormal() const
{
	PxTriangle localTri;
	getMeshData()->getLocalTriangle(localTri, mTriangleIndex, mVertex2ShapeSkew.flipsNormal());

	const PxVec3 t0 = mVertex2ShapeSkew * localTri.verts[0];
	const PxVec3 t1 = mVertex2ShapeSkew * localTri.verts[1];
	const PxVec3 t2 = mVertex2ShapeSkew * localTri.verts[2];
	const PxVec3 v0 = t0 - t1;
	const PxVec3 v1 = t0 - t2;
	const PxVec3 nor = v0.cross(v1);
	return nor.getNormalized();
}

}

PxReal SweepAnyShapeHeightfield(GU_SWEEP_METHOD_ARGS)
{
	PX_UNUSED(toiEstimate);

	PX_ASSERT(shape1.mGeometry->getType()==PxGeometryType::eHEIGHTFIELD);
	const HeightFieldUtil hfUtil(static_cast<const PxHeightFieldGeometry&>(*shape1.mGeometry));

	PxInlineArray<PxU32,64> tempContainer;

	EntityReportContainerCallback callback(tempContainer);

	const PxVec3 trA = transform0.p - lastTm0.p;
	const PxVec3 trB = transform1.p - lastTm1.p;

	const PxVec3 relTr = trA - trB;
	const PxVec3 halfRelTr = relTr * 0.5f;

	const PxVec3 ext = shape0.mExtents + halfRelTr.abs() + PxVec3(restDistance);
	const PxVec3 cent = shape0.mCenter + halfRelTr;

	const PxBounds3 bounds0(cent - ext, cent + ext);

	hfUtil.overlapAABBTriangles(transform1, bounds0, callback);

	PxArray<PxU32> orderedContainer(tempContainer.size());
	
	PxArray<PxU32> distanceEntries(tempContainer.size());

	PxU32* orderedList = orderedContainer.begin();
	PxF32* distances = reinterpret_cast<PxF32*>(distanceEntries.begin());
	
	const PxVec3 origin = shape0.mCenter;
	const PxVec3 extent = shape0.mExtents + PxVec3(restDistance);

	PxReal minTOI = PX_MAX_REAL;

	PxU32 numTrigs = tempContainer.size();
	PxU32* trianglesIndices = tempContainer.begin();

	PxU32 count = 0;
	for(PxU32 a = 0; a < numTrigs; ++a)
	{
		PxTriangle tri;
		hfUtil.getTriangle(shape1.mPrevTransform, tri, 0, 0, trianglesIndices[a], true, true);

		PxVec3 resultNormal = -(tri.verts[1]-tri.verts[0]).cross(tri.verts[2]-tri.verts[0]);
		resultNormal.normalize();

		if(relTr.dot(resultNormal) >= fastMovingThreshold)
		{
			PxBounds3 bounds;
			bounds.setEmpty();
			bounds.include(tri.verts[0]);
			bounds.include(tri.verts[1]);
			bounds.include(tri.verts[2]);

			PxF32 toi = sweepAABBAABB(origin, extent * 1.1f, bounds.getCenter(), (bounds.getExtents() + PxVec3(0.01f, 0.01f, 0.01f)) * 1.1f, trA, trB);

			PxU32 index = 0;
			if(toi <= 1.f)
			{
				for(PxU32 b = count; b > 0; --b)
				{
					if(distances[b-1] <= toi)
					{
						//shuffle down and swap
						index = b;
						break;
					}
					PX_ASSERT(b > 0);
					PX_ASSERT(b < numTrigs);
					distances[b] = distances[b-1];
					orderedList[b] = orderedList[b-1];
				}
				PX_ASSERT(index < numTrigs);
				orderedList[index] = trianglesIndices[a];
				distances[index] = toi;
				count++;
			}
		}
	}

	worldNormal = PxVec3(PxReal(0));
	worldPoint = PxVec3(PxReal(0));
	Cm::FastVertex2ShapeScaling idScale;
	PxU32 ccdFaceIndex = PXC_CONTACT_NO_FACE_INDEX;

	const PxVec3 sphereCenter(shape0.mPrevTransform.p);
	const PxF32 inSphereRadius = shape0.mFastMovingThreshold;
	const PxF32 inRadSq = inSphereRadius * inSphereRadius;

	const PxVec3 sphereCenterInTr1 = transform1.transformInv(sphereCenter);
	const PxVec3 sphereCenterInTr1T0 = transform1.transformInv(lastTm0.p);

	PxVec3 tempWorldNormal(0.f), tempWorldPoint(0.f);

	for (PxU32 ti = 0; ti < count; ti++)
	{
		PxTriangle tri;
		hfUtil.getTriangle(lastTm1, tri, 0, 0, orderedList[ti], false, false);

		PxVec3 resultNormal, resultPoint;

		TriangleV triangle(V3LoadU(tri.verts[0]), V3LoadU(tri.verts[1]), V3LoadU(tri.verts[2]));

		//do sweep

		PxReal res = SweepShapeTriangle(
			*shape0.mGeometry, *shape1.mGeometry, transform0, transform1, lastTm0, lastTm1, restDistance,
			resultNormal, resultPoint, Cm::FastVertex2ShapeScaling(), triangle,
			0.f);

		if(res <= 0.f)
		{
			res = 0.f;

			const PxVec3 v0 = tri.verts[1] - tri.verts[0];
			const PxVec3 v1 = tri.verts[2] - tri.verts[0];

			//Now we have a 0 TOI, lets see if the in-sphere hit it!

			PxF32 distanceSq = distancePointTriangleSquared( sphereCenterInTr1, tri.verts[0], v0, v1);

			if(distanceSq < inRadSq)
			{
				const PxVec3 nor = v0.cross(v1);
				const PxF32 distance = PxSqrt(distanceSq);
				res = distance - inSphereRadius;
				const PxF32 d = nor.dot(tri.verts[0]);
				const PxF32 dd = nor.dot(sphereCenterInTr1T0);
				if((dd - d) > 0.f)
				{
					//back side, penetration 
					res = -(2.f * inSphereRadius - distance);
				}
			}			
		}

		if (res < minTOI)
		{
			const PxVec3 v0 = tri.verts[1] - tri.verts[0];
			const PxVec3 v1 = tri.verts[2] - tri.verts[0];

			PxVec3 resultNormal1 = v0.cross(v1);
			resultNormal1.normalize();
			//if(norDotRel > 1e-6f)
			{
				tempWorldNormal = resultNormal1;
				tempWorldPoint = resultPoint;
				minTOI = res;
				ccdFaceIndex = orderedList[ti];
			}
		}
	}

	worldNormal = transform1.rotate(tempWorldNormal);
	worldPoint = tempWorldPoint;

	outCCDFaceIndex = ccdFaceIndex;

	return minTOI;
}

PxReal SweepEstimateAnyShapeHeightfield(GU_SWEEP_ESTIMATE_ARGS)
{
	PX_ASSERT(shape1.mGeometry->getType()==PxGeometryType::eHEIGHTFIELD);
	const HeightFieldUtil hfUtil(static_cast<const PxHeightFieldGeometry&>(*shape1.mGeometry));

	PxInlineArray<PxU32,64> tempContainer;
	
	EntityReportContainerCallback callback(tempContainer);

	const PxTransform& transform0 = shape0.mCurrentTransform;
	const PxTransform& lastTr0 = shape0.mPrevTransform;
	const PxTransform& transform1 = shape1.mCurrentTransform;
	const PxTransform& lastTr1 = shape1.mPrevTransform;

	const PxVec3 trA = transform0.p - lastTr0.p;
	const PxVec3 trB = transform1.p - lastTr1.p;

	const PxVec3 relTr = trA - trB;
	const PxVec3 halfRelTr = relTr * 0.5f;

	const PxVec3 extents = shape0.mExtents + halfRelTr.abs() + PxVec3(restDistance);
	const PxVec3 center = shape0.mCenter + halfRelTr;

	const PxBounds3 bounds0(center - extents, center + extents);

	hfUtil.overlapAABBTriangles(transform1, bounds0, callback);
	
	PxVec3 origin = shape0.mCenter;
	PxVec3 extent = shape0.mExtents;

	PxReal minTOI = PX_MAX_REAL;

	PxU32 numTrigs = tempContainer.size();
	PxU32* trianglesIndices = tempContainer.begin();

	for(PxU32 a = 0; a < numTrigs; ++a)
	{
		PxTriangle tri;
		hfUtil.getTriangle(shape1.mPrevTransform, tri, 0, 0, trianglesIndices[a], true, true);

		PxVec3 resultNormal = -(tri.verts[1]-tri.verts[0]).cross(tri.verts[2]-tri.verts[0]);
		resultNormal.normalize();

		if(relTr.dot(resultNormal) >= fastMovingThreshold)
		{
			PxBounds3 bounds;
			bounds.setEmpty();
			bounds.include(tri.verts[0]);
			bounds.include(tri.verts[1]);
			bounds.include(tri.verts[2]);

			PxF32 toi = sweepAABBAABB(origin, extent * 1.1f, bounds.getCenter(), (bounds.getExtents() + PxVec3(0.01f, 0.01f, 0.01f)) * 1.1f, trA, trB);

			minTOI = PxMin(minTOI, toi);
		}
	}

	return minTOI;
}

PxReal SweepAnyShapeMesh(GU_SWEEP_METHOD_ARGS)
{
	PX_UNUSED(toiEstimate);
	// this is the trimesh midphase for convex vs mesh sweep. shape0 is the convex shape.

	const PxVec3 trA = transform0.p - lastTm0.p;
	const PxVec3 trB = transform1.p - lastTm1.p;

	const PxVec3 relTr = trA - trB;
	PxVec3 unitDir = relTr;
	const PxReal length = unitDir.normalize();

	PX_UNUSED(restDistance);
	PX_UNUSED(fastMovingThreshold);

	if(gUseGeometryQuery)
	{
		PxGeomSweepHit sweepHit;
		if(!PxGeometryQuery::sweep(unitDir, length, *shape0.mGeometry, lastTm0, *shape1.mGeometry, lastTm1, sweepHit, PxHitFlag::eDEFAULT, 0.0f, PxGeometryQueryFlag::Enum(0), NULL))
		//if(!PxGeometryQuery::sweep(unitDir, length, *shape0.mGeometry, transform0, *shape1.mGeometry, transform1, sweepHit, PxHitFlag::eDEFAULT, 0.0f, PxGeometryQueryFlag::Enum(0), NULL))
			return PX_MAX_REAL;

		worldNormal = sweepHit.normal;
		worldPoint = sweepHit.position;
		outCCDFaceIndex = sweepHit.faceIndex;

		return sweepHit.distance/length;
	}
	else
	{
		// Get actual shape data
		PX_ASSERT(shape1.mGeometry->getType()==PxGeometryType::eTRIANGLEMESH);
		const PxTriangleMeshGeometry& shapeMesh = static_cast<const PxTriangleMeshGeometry&>(*shape1.mGeometry);

		const Cm::FastVertex2ShapeScaling meshScaling(shapeMesh.scale);

		const PxMat33 matRot(PxIdentity);

		//1) Compute the swept bounds
		Box sweptBox;
		computeSweptBox(sweptBox, shape0.mExtents, shape0.mCenter, matRot, unitDir, length);

		Box vertexSpaceBox;
		if (shapeMesh.scale.isIdentity())
			vertexSpaceBox = transformBoxOrthonormal(sweptBox, transform1.getInverse());
		else
			computeVertexSpaceOBB(vertexSpaceBox, sweptBox, transform1, shapeMesh.scale);

		vertexSpaceBox.extents += PxVec3(restDistance);

		PxInlineArray<PxU32, 64> tempContainer;

		AccumCallback callback(tempContainer);

		// AP scaffold: early out opportunities, should probably use fat raycast
		Midphase::intersectOBB(_getMeshData(shapeMesh), vertexSpaceBox, callback, true);

		if (tempContainer.size() == 0)
			return PX_MAX_REAL;

		// Intersection found, fetch triangles
		PxU32 numTrigs = tempContainer.size();
		const PxU32* triangleIndices = tempContainer.begin();

		PxVec3 origin = shape0.mCenter;
		PxVec3 extent = shape0.mExtents + PxVec3(restDistance);
	
		PxInlineArray<PxU32, 64> orderedContainer;
		orderedContainer.resize(tempContainer.size());
	
		PxInlineArray<PxU32, 64> distanceEntries;
		distanceEntries.resize(tempContainer.size());
	
		PxU32* orderedList = orderedContainer.begin();
		PxF32* distances = reinterpret_cast<PxF32*>(distanceEntries.begin());

		PxReal minTOI = PX_MAX_REAL;

		PxU32 count = 0;
		for(PxU32 a = 0; a < numTrigs; ++a)
		{
			const TriangleHelper convexPartOfMesh1(shapeMesh, meshScaling, triangleIndices[a]);

			const PxVec3 resultNormal = -transform1.rotate(convexPartOfMesh1.getPolygonNormal());

			if(relTr.dot(resultNormal) >= fastMovingThreshold)
			{
				PxBounds3 bounds;
				convexPartOfMesh1.getBounds(bounds, lastTm1);
				//OK, we have all 3 vertices, now calculate bounds...

				PxF32 toi = sweepAABBAABB(origin, extent, bounds.getCenter(), bounds.getExtents() + PxVec3(0.02f, 0.02f, 0.02f), trA, trB);

				PxU32 index = 0;
				if(toi <= 1.f)
				{
					for(PxU32 b = count; b > 0; --b)
					{
						if(distances[b-1] <= toi)
						{
							//shuffle down and swap
							index = b;
							break;
						}
						PX_ASSERT(b > 0);
						PX_ASSERT(b < numTrigs);
						distances[b] = distances[b-1];
						orderedList[b] = orderedList[b-1];
					}
					PX_ASSERT(index < numTrigs);
					orderedList[index] = triangleIndices[a];
					distances[index] = toi;
					count++;
				}
			}
		}

		PxVec3 tempWorldNormal(0.f), tempWorldPoint(0.f);

		Cm::FastVertex2ShapeScaling idScale;
		PxU32 ccdFaceIndex = PXC_CONTACT_NO_FACE_INDEX;

		const PxVec3 sphereCenter(lastTm1.p);
		const PxF32 inSphereRadius = shape0.mFastMovingThreshold;
		//PxF32 inRadSq = inSphereRadius * inSphereRadius;

		const PxVec3 sphereCenterInTransform1 = transform1.transformInv(sphereCenter);

		const PxVec3 sphereCenterInTransform0p = transform1.transformInv(lastTm0.p);

		for (PxU32 ti = 0; ti < count /*&& PxMax(minTOI, 0.f) >= distances[ti]*/; ti++)
		{
			const TriangleHelper convexPartOfMesh1(shapeMesh, meshScaling, orderedList[ti]);

			PxVec3 resultNormal, resultPoint;
			PxTriangle localTri;
			_getMeshData(shapeMesh)->getLocalTriangle(localTri, orderedList[ti], meshScaling.flipsNormal());

			const PxVec3 v0 = meshScaling * localTri.verts[0];
			const PxVec3 v1 = meshScaling * localTri.verts[1];
			const PxVec3 v2 = meshScaling * localTri.verts[2];

			TriangleV triangle(V3LoadU(v0), V3LoadU(v1), V3LoadU(v2));

			//do sweep
			PxReal res = SweepShapeTriangle(
				*shape0.mGeometry, *shape1.mGeometry, transform0, transform1, lastTm0, lastTm1, restDistance,
				resultNormal, resultPoint, Cm::FastVertex2ShapeScaling(), triangle,
				0.f);

			resultNormal = -resultNormal;

			if(res <= 0.f)
			{
				res = 0.f;

				const PxF32 inRad = inSphereRadius + restDistance;
				const PxF32 inRadSq = inRad*inRad;

				const PxVec3 vv0 = v1 - v0;
				const PxVec3 vv1 = v2 - v0;
				const PxVec3 nor = vv0.cross(vv1);

				//Now we have a 0 TOI, lets see if the in-sphere hit it!

				const PxF32 distanceSq = distancePointTriangleSquared( sphereCenterInTransform1, v0, vv0, vv1);

				if(distanceSq < inRadSq)
				{
					const PxF32 distance = PxSqrt(distanceSq);
					res = distance - inRad;
					const PxF32 d = nor.dot(v0);
					const PxF32 dd = nor.dot(sphereCenterInTransform0p);
					if((dd - d) < 0.f)
					{
						//back side, penetration 
						res = -(2.f * inRad - distance);
					}
				}	
				PX_ASSERT(PxIsFinite(res));
				resultNormal = transform1.rotate(convexPartOfMesh1.getPolygonNormal());			
			}

			if (res < minTOI)
			{
				tempWorldNormal = resultNormal;//convexPartOfMesh1.getPolygonNormal(0);//transform1.rotate(convexPartOfMesh1.getPolygonNormal(0));
				tempWorldPoint = resultPoint;
				minTOI = res;
				ccdFaceIndex = orderedList[ti];
			}
		}

		worldNormal = tempWorldNormal;//transform1.rotate(tempWorldNormal);
		worldPoint = tempWorldPoint;
		outCCDFaceIndex = ccdFaceIndex;
		return minTOI;
	}
}

/**
\brief This code performs a conservative estimate of the TOI of a shape v mesh.
*/
PxReal SweepEstimateAnyShapeMesh(GU_SWEEP_ESTIMATE_ARGS)
{
	// this is the trimesh midphase for convex vs mesh sweep. shape0 is the convex shape.
	// Get actual shape data
	PX_ASSERT(shape1.mGeometry->getType()==PxGeometryType::eTRIANGLEMESH);
	const PxTriangleMeshGeometry& shapeMesh = static_cast<const PxTriangleMeshGeometry&>(*shape1.mGeometry);

	const PxTransform& transform0 = shape0.mCurrentTransform;
	const PxTransform& lastTr0 = shape0.mPrevTransform;
	const PxTransform& transform1 = shape1.mCurrentTransform;
	const PxTransform& lastTr1 = shape1.mPrevTransform;

	const PxVec3 trA = transform0.p - lastTr0.p;
	const PxVec3 trB = transform1.p - lastTr1.p;

	const PxVec3 relTr = trA - trB;
	PxVec3 unitDir = relTr;
	const PxReal length = unitDir.normalize();

#ifdef CCD_BASIC_PROFILING
	unsigned long long time = __rdtsc();
#endif

	if(gUseGeometryQueryEst)
	{
		PX_UNUSED(restDistance);
		PX_UNUSED(fastMovingThreshold);
		PX_UNUSED(shapeMesh);
		{
			PxGeomSweepHit sweepHit;

			bool status = PxGeometryQuery::sweep(unitDir, length, *shape0.mGeometry, lastTr0, *shape1.mGeometry, lastTr1, sweepHit, PxHitFlag::eDEFAULT, 0.0f, PxGeometryQueryFlag::Enum(0), NULL);
#ifdef CCD_BASIC_PROFILING
			unsigned long long time2 = __rdtsc();
			printf("Time: %d\n", PxU32(time2 - time)/1024);
#endif
			return status ? sweepHit.distance/length : PX_MAX_REAL;
		}
	}
	else
	{
		const Cm::FastVertex2ShapeScaling meshScaling(shapeMesh.scale);

		const PxMat33 matRot(PxIdentity);

		//1) Compute the swept bounds
		Box sweptBox;
		computeSweptBox(sweptBox, shape0.mExtents, shape0.mCenter, matRot, unitDir, length);

		Box vertexSpaceBox;
		computeVertexSpaceOBB(vertexSpaceBox, sweptBox, transform1, shapeMesh.scale);

		vertexSpaceBox.extents += PxVec3(restDistance);

		// TODO: implement a cached mode that fetches the trigs from a cache rather than per opcode if there is little motion.

		struct CB : MeshHitCallback<PxGeomRaycastHit>
		{
			PxReal minTOI;
			PxReal sumFastMovingThresh;
			const PxTriangleMeshGeometry& shapeMesh;
			const Cm::FastVertex2ShapeScaling& meshScaling;
			const PxVec3& relTr;
			const PxVec3& trA;
			const PxVec3& trB;
			const PxTransform& transform1;
			const PxVec3& origin;
			const PxVec3& extent;

			CB(PxReal aSumFast, const PxTriangleMeshGeometry& aShapeMesh, const Cm::FastVertex2ShapeScaling& aMeshScaling,
				const PxVec3& aRelTr, const PxVec3& atrA, const PxVec3& atrB, const PxTransform& aTransform1, const PxVec3& aOrigin, const PxVec3& aExtent)
				:	MeshHitCallback<PxGeomRaycastHit>(CallbackMode::eMULTIPLE),
					sumFastMovingThresh(aSumFast), shapeMesh(aShapeMesh), meshScaling(aMeshScaling), relTr(aRelTr), trA(atrA), trB(atrB),
					transform1(aTransform1), origin(aOrigin), extent(aExtent)
			{
				minTOI = PX_MAX_REAL;
			}

			virtual PxAgain processHit( // all reported coords are in mesh local space including hit.position
				const PxGeomRaycastHit& hit, const PxVec3&, const PxVec3&, const PxVec3&, PxReal& shrunkMaxT, const PxU32*)	PX_OVERRIDE PX_FINAL
			{
				const TriangleHelper convexPartOfMesh1(shapeMesh, meshScaling, hit.faceIndex);
				const PxVec3 resultNormal = -transform1.rotate(convexPartOfMesh1.getPolygonNormal());
				if(relTr.dot(resultNormal) >= sumFastMovingThresh)
				{
					PxBounds3 bounds;
					convexPartOfMesh1.getBounds(bounds, transform1);
					//OK, we have all 3 vertices, now calculate bounds...

					PxF32 toi = sweepAABBAABB(
						origin, extent * 1.1f, bounds.getCenter(), (bounds.getExtents() + PxVec3(0.01f, 0.01f, 0.01f)) * 1.1f, trA, trB);

					minTOI = PxMin(minTOI, toi);
					shrunkMaxT = minTOI;
				}

				return (minTOI > 0.0f); // stop traversal if minTOI == 0.0f
			}

			void operator=(const CB&) {}
		};

		const PxVec3& origin = shape0.mCenter;
		const PxVec3 extent = shape0.mExtents + PxVec3(restDistance);

		CB callback(fastMovingThreshold, shapeMesh, meshScaling, relTr, trA, trB, transform1, origin, extent);
		Midphase::intersectOBB(_getMeshData(shapeMesh), vertexSpaceBox, callback, true);

#ifdef CCD_BASIC_PROFILING
		unsigned long long time2 = __rdtsc();
		printf("Time: %d\n", PxU32(time2 - time)/1024);
#endif

		return callback.minTOI;
	}
}

}
}

