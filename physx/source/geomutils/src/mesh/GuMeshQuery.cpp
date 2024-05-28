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

#include "common/PxProfileZone.h"
#include "geometry/PxMeshQuery.h"
#include "geometry/PxSphereGeometry.h"
#include "geometry/PxGeometryQuery.h"

#include "GuInternal.h"
#include "GuEntityReport.h"
#include "GuHeightFieldUtil.h"
#include "GuBoxConversion.h"
#include "GuIntersectionTriangleBox.h"
#include "CmScaling.h"
#include "GuSweepTests.h"
#include "GuMidphaseInterface.h"
#include "foundation/PxFPU.h"

using namespace physx;
using namespace Gu;

namespace {

	class HfTrianglesEntityReport2 : public OverlapReport, public LimitedResults
	{
	public:
		HfTrianglesEntityReport2(
			PxU32* results, PxU32 maxResults, PxU32 startIndex,
			HeightFieldUtil& hfUtil,
			const PxVec3& boxCenter, const PxVec3& boxExtents, const PxQuat& boxRot,
			bool aabbOverlap) :
				LimitedResults	(results, maxResults, startIndex),
				mHfUtil			(hfUtil),
				mAABBOverlap	(aabbOverlap)
		{
			buildFrom(mBox2Hf, boxCenter, boxExtents, boxRot);
		}

		virtual bool reportTouchedTris(PxU32 nbEntities, const PxU32* entities)
		{
			if(mAABBOverlap)
			{
				while(nbEntities--)
					if(!add(*entities++))
						return false;
			}
			else
			{
				const PxTransform idt(PxIdentity);
				for(PxU32 i=0; i<nbEntities; i++)
				{
					PxTrianglePadded tri;
					mHfUtil.getTriangle(idt, tri, NULL, NULL, entities[i], false, false);  // First parameter not needed if local space triangle is enough

					// PT: this one is safe because triangle class is padded
					if(intersectTriangleBox(mBox2Hf, tri.verts[0], tri.verts[1], tri.verts[2]))
					{
						if(!add(entities[i]))
							return false;
					}
				}
			}
			return true;
		}

			HeightFieldUtil&	mHfUtil;
			BoxPadded			mBox2Hf;
			bool				mAABBOverlap;

	private:
		HfTrianglesEntityReport2& operator=(const HfTrianglesEntityReport2&);
	};

} // namespace

void physx::PxMeshQuery::getTriangle(const PxTriangleMeshGeometry& triGeom, const PxTransform& globalPose, PxTriangleID triangleIndex, PxTriangle& triangle, PxU32* vertexIndices, PxU32* adjacencyIndices)
{
	const TriangleMesh* tm = static_cast<const TriangleMesh*>(triGeom.triangleMesh);

	PX_CHECK_AND_RETURN(triangleIndex<tm->getNbTriangles(), "PxMeshQuery::getTriangle: triangle index is out of bounds");

	if(adjacencyIndices && !tm->getAdjacencies())
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "Adjacency information not created. Set buildTriangleAdjacencies on Cooking params.");

	const PxMat34 vertex2worldSkew = globalPose * triGeom.scale;
	tm->computeWorldTriangle(triangle, triangleIndex, vertex2worldSkew, triGeom.scale.hasNegativeDeterminant(), vertexIndices, adjacencyIndices);
}

///////////////////////////////////////////////////////////////////////////////

void physx::PxMeshQuery::getTriangle(const PxHeightFieldGeometry& hfGeom, const PxTransform& globalPose, PxTriangleID triangleIndex, PxTriangle& triangle, PxU32* vertexIndices, PxU32* adjacencyIndices)
{
	HeightFieldUtil hfUtil(hfGeom);
	
	hfUtil.getTriangle(globalPose, triangle, vertexIndices, adjacencyIndices, triangleIndex, true, true);
}

///////////////////////////////////////////////////////////////////////////////

PxU32 physx::PxMeshQuery::findOverlapTriangleMesh(
	const PxGeometry& geom, const PxTransform& geomPose,
	const PxTriangleMeshGeometry& meshGeom, const PxTransform& meshPose,
	PxU32* results, PxU32 maxResults, PxU32 startIndex, bool& overflow, PxGeometryQueryFlags queryFlags)
{
	PX_SIMD_GUARD_CNDT(queryFlags & PxGeometryQueryFlag::eSIMD_GUARD)

	LimitedResults limitedResults(results, maxResults, startIndex);

	const TriangleMesh* tm = static_cast<const TriangleMesh*>(meshGeom.triangleMesh);

	switch(geom.getType())
	{
		case PxGeometryType::eBOX:
		{
			const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);

			Box box;
			buildFrom(box, geomPose.p, boxGeom.halfExtents, geomPose.q);

			Midphase::intersectBoxVsMesh(box, *tm, meshPose, meshGeom.scale, &limitedResults);
			break;
		}

		case PxGeometryType::eCAPSULE:
		{
			const PxCapsuleGeometry& capsGeom = static_cast<const PxCapsuleGeometry&>(geom);

			Capsule capsule;
			getCapsule(capsule, capsGeom, geomPose);

			Midphase::intersectCapsuleVsMesh(capsule, *tm, meshPose, meshGeom.scale, &limitedResults);
			break;
		}

		case PxGeometryType::eSPHERE:
		{
			const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);
			Midphase::intersectSphereVsMesh(Sphere(geomPose.p, sphereGeom.radius), *tm, meshPose, meshGeom.scale, &limitedResults);
			break;
		}

		default:
		{
			PX_CHECK_MSG(false, "findOverlapTriangleMesh: Only box, capsule and sphere geometries are supported.");
		}
	}

	overflow = limitedResults.mOverflow;
	return limitedResults.mNbResults;
}

///////////////////////////////////////////////////////////////////////////////

bool physx::PxMeshQuery::findOverlapTriangleMesh(	PxReportCallback<PxGeomIndexPair>& callback,
													const PxTriangleMeshGeometry& meshGeom0, const PxTransform& meshPose0,
													const PxTriangleMeshGeometry& meshGeom1, const PxTransform& meshPose1,
													PxGeometryQueryFlags queryFlags, PxMeshMeshQueryFlags meshMeshFlags, float tolerance)
{
	PX_SIMD_GUARD_CNDT(queryFlags & PxGeometryQueryFlag::eSIMD_GUARD)

	const TriangleMesh* tm0 = static_cast<const TriangleMesh*>(meshGeom0.triangleMesh);
	const TriangleMesh* tm1 = static_cast<const TriangleMesh*>(meshGeom1.triangleMesh);

	// PT: only implemented for BV4
	if(!tm0 || !tm1 || tm0->getConcreteType()!=PxConcreteType::eTRIANGLE_MESH_BVH34 || tm1->getConcreteType()!=PxConcreteType::eTRIANGLE_MESH_BVH34)
		return PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxMeshQuery::findOverlapTriangleMesh(): only available between two BVH34 triangles meshes.");

	// PT: ...so we don't need a table like for the other ops, just go straight to BV4
	return intersectMeshVsMesh_BV4(callback, *tm0, meshPose0, meshGeom0.scale, *tm1, meshPose1, meshGeom1.scale, meshMeshFlags, tolerance);
}

///////////////////////////////////////////////////////////////////////////////

PxU32 physx::PxMeshQuery::findOverlapHeightField(	const PxGeometry& geom, const PxTransform& geomPose,
													const PxHeightFieldGeometry& hfGeom, const PxTransform& hfPose,
													PxU32* results, PxU32 maxResults, PxU32 startIndex, bool& overflow, PxGeometryQueryFlags queryFlags)
{
	PX_SIMD_GUARD_CNDT(queryFlags & PxGeometryQueryFlag::eSIMD_GUARD)

	const PxTransform localPose0 = hfPose.transformInv(geomPose);
	PxBoxGeometry boxGeom;

	switch(geom.getType())
	{
		case PxGeometryType::eCAPSULE:
		{
			const PxCapsuleGeometry& cap = static_cast<const PxCapsuleGeometry&>(geom);
			boxGeom.halfExtents = PxVec3(cap.halfHeight+cap.radius, cap.radius, cap.radius);
			// PT: TODO: improve these bounds - see computeCapsuleBounds
		}
		break;
		case PxGeometryType::eSPHERE:
		{
			const PxSphereGeometry& sph = static_cast<const PxSphereGeometry&>(geom);
			boxGeom.halfExtents = PxVec3(sph.radius);

			// PT: TODO: could this codepath be improved using the following?
			//PxBounds3 localBounds;
			//const PxVec3 localSphereCenter = getLocalSphereData(localBounds, pose0, pose1, sphereGeom.radius);
		}
		break;
		case PxGeometryType::eBOX:
			boxGeom = static_cast<const PxBoxGeometry&>(geom);
		break;
		default:
		{
			overflow = false;
			PX_CHECK_AND_RETURN_VAL(false, "findOverlapHeightField: Only box, sphere and capsule queries are supported.", false);
		}
	}

	const bool isAABB = ((localPose0.q.x == 0.0f) && (localPose0.q.y == 0.0f) && (localPose0.q.z == 0.0f));
	
	PxBounds3 bounds;
	if (isAABB)
		bounds = PxBounds3::centerExtents(localPose0.p, boxGeom.halfExtents);
	else
		bounds = PxBounds3::poseExtent(localPose0, boxGeom.halfExtents); // box.halfExtents is really extent

	HeightFieldUtil hfUtil(hfGeom);
	HfTrianglesEntityReport2 entityReport(results, maxResults, startIndex, hfUtil, localPose0.p, boxGeom.halfExtents, localPose0.q, isAABB);

	hfUtil.overlapAABBTriangles(bounds, entityReport);
	overflow = entityReport.mOverflow;
	return entityReport.mNbResults;
}

///////////////////////////////////////////////////////////////////////////////

bool physx::PxMeshQuery::sweep(	const PxVec3& unitDir, const PxReal maxDistance,
								const PxGeometry& geom, const PxTransform& pose,
								PxU32 triangleCount, const PxTriangle* triangles,
								PxGeomSweepHit& sweepHit, PxHitFlags hitFlags,
								const PxU32* cachedIndex, const PxReal inflation, bool doubleSided, PxGeometryQueryFlags queryFlags)
{
	PX_SIMD_GUARD_CNDT(queryFlags & PxGeometryQueryFlag::eSIMD_GUARD)
	PX_CHECK_AND_RETURN_VAL(pose.isValid(), "PxMeshQuery::sweep(): pose is not valid.", false);
	PX_CHECK_AND_RETURN_VAL(unitDir.isFinite(), "PxMeshQuery::sweep(): unitDir is not valid.", false);
	PX_CHECK_AND_RETURN_VAL(PxIsFinite(maxDistance), "PxMeshQuery::sweep(): distance is not valid.", false);
	PX_CHECK_AND_RETURN_VAL(maxDistance > 0, "PxMeshQuery::sweep(): sweep distance must be greater than 0.", false);

	PX_PROFILE_ZONE("MeshQuery.sweep", 0);

	const PxReal distance = PxMin(maxDistance, PX_MAX_SWEEP_DISTANCE);

	switch(geom.getType())
	{
		case PxGeometryType::eSPHERE:
		{
			const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);

			const PxCapsuleGeometry capsuleGeom(sphereGeom.radius, 0.0f);

			return sweepCapsuleTriangles(	triangleCount, triangles, doubleSided, capsuleGeom, pose, unitDir, distance,
											sweepHit, cachedIndex, inflation, hitFlags);
		}

		case PxGeometryType::eCAPSULE:
		{
			const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom);

			return sweepCapsuleTriangles(	triangleCount, triangles, doubleSided, capsuleGeom, pose, unitDir, distance,
											sweepHit, cachedIndex, inflation, hitFlags);
		}

		case PxGeometryType::eBOX:
		{
			const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);

			if(hitFlags & PxHitFlag::ePRECISE_SWEEP)
			{
				return sweepBoxTriangles_Precise(	triangleCount, triangles, doubleSided, boxGeom, pose, unitDir, distance, sweepHit, cachedIndex,
													inflation, hitFlags);
			}
			else
			{
				return sweepBoxTriangles(	triangleCount, triangles, doubleSided, boxGeom, pose, unitDir, distance, sweepHit, cachedIndex,
											inflation, hitFlags);
			}
		}	
		default:
			PX_CHECK_MSG(false, "PxMeshQuery::sweep(): geometry object parameter must be sphere, capsule or box geometry.");
	}
	return false;
}

///////////////////////////////////////////////////////////////////////////////

