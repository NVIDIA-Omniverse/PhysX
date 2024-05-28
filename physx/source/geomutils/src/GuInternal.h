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

#ifndef GU_INTERNAL_H
#define GU_INTERNAL_H

#include "geometry/PxCapsuleGeometry.h"
#include "geometry/PxBoxGeometry.h"
#include "GuCapsule.h"
#include "foundation/PxTransform.h"
#include "foundation/PxMathUtils.h"
#include "foundation/PxUtilities.h"
#include "foundation/PxMat33.h"

#define GU_EPSILON_SAME_DISTANCE 1e-3f

namespace physx
{
namespace Gu
{
	class Box;

	// PT: TODO: now that the Gu files are not exposed to users anymore, we should move back capsule-related functions
	// to GuCapsule.h, etc

	PX_PHYSX_COMMON_API const PxU8*	getBoxEdges();

	PX_PHYSX_COMMON_API void		computeBoxPoints(const PxBounds3& bounds, PxVec3* PX_RESTRICT pts);

						void		computeBoxAroundCapsule(const Capsule& capsule, Box& box);

						PxPlane		getPlane(const PxTransform& pose);

	PX_FORCE_INLINE		PxVec3		getCapsuleHalfHeightVector(const PxTransform& transform, const PxCapsuleGeometry& capsuleGeom)
									{
										return transform.q.getBasisVector0() * capsuleGeom.halfHeight;
									}

	PX_FORCE_INLINE		void		getCapsuleSegment(const PxTransform& transform, const PxCapsuleGeometry& capsuleGeom, Gu::Segment& segment)
									{
										const PxVec3 tmp = getCapsuleHalfHeightVector(transform, capsuleGeom);
										segment.p0 = transform.p + tmp;
										segment.p1 = transform.p - tmp;
									}

	PX_FORCE_INLINE		void		getCapsule(Gu::Capsule& capsule, const PxCapsuleGeometry& capsuleGeom, const PxTransform& pose)
									{
										getCapsuleSegment(pose, capsuleGeom, capsule);
										capsule.radius = capsuleGeom.radius;
									}

						void		computeSweptBox(Gu::Box& box, const PxVec3& extents, const PxVec3& center, const PxMat33& rot, const PxVec3& unitDir, PxReal distance);

	/**
	*	PT: computes "alignment value" used to select the "best" triangle in case of identical impact distances (for sweeps).
	*	This simply computes how much a triangle is aligned with a given sweep direction.
	*	Captured in a function to make sure it is always computed correctly, i.e. working for double-sided triangles.
	*
	*	\param		triNormal	[in] triangle's normal
	*	\param		unitDir		[in] sweep direction (normalized)
	*	\return		alignment value in [-1.0f, 0.0f]. -1.0f for fully aligned, 0.0f for fully orthogonal.
	*/
	PX_FORCE_INLINE		PxReal		computeAlignmentValue(const PxVec3& triNormal, const PxVec3& unitDir)
	{
		PX_ASSERT(triNormal.isNormalized());
		// PT: initial dot product gives the angle between the two, with "best" triangles getting a +1 or -1 score
		// depending on their winding. We take the absolute value to ignore the impact of winding. We negate the result
		// to make the function compatible with the initial code, which assumed single-sided triangles and expected -1
		// for best triangles.
		return -PxAbs(triNormal.dot(unitDir));
	}

	/**
	*	PT: sweeps: determines if a newly touched triangle is "better" than best one so far.
	*	In this context "better" means either clearly smaller impact distance, or a similar impact
	*	distance but a normal more aligned with the sweep direction.
	*
	*	\param		triImpactDistance	[in] new triangle's impact distance
	*	\param		triAlignmentValue	[in] new triangle's alignment value (as computed by computeAlignmentValue)
	*	\param		bestImpactDistance	[in] current best triangle's impact distance
	*	\param		bestAlignmentValue	[in] current best triangle's alignment value (as computed by computeAlignmentValue)
	*   \param		maxDistance			[in] maximum distance of the query, hit cannot be longer than this maxDistance
	*	\return		true if new triangle is better
	*/
	PX_FORCE_INLINE		bool		keepTriangle(	float triImpactDistance, float triAlignmentValue,
													float bestImpactDistance, float bestAlignmentValue, float maxDistance)
	{
		// Reject triangle if further than the maxDistance
		if(triImpactDistance > maxDistance)
			return false;

		// If initial overlap happens, keep the triangle
		if(triImpactDistance == 0.0f)
			return true;

		// tris have "similar" impact distances if the difference is smaller than 2*distEpsilon
		float distEpsilon = GU_EPSILON_SAME_DISTANCE; // pick a farther hit within distEpsilon that is more opposing than the previous closest hit

		// PT: make it a relative epsilon to make sure it still works with large distances
		distEpsilon *= PxMax(1.0f, PxMax(triImpactDistance, bestImpactDistance));

		// If new distance is more than epsilon closer than old distance
		if(triImpactDistance < bestImpactDistance - distEpsilon)
			return true;

		// If new distance is no more than epsilon farther than oldDistance and "face is more opposing than previous"
		if(triImpactDistance < bestImpactDistance+distEpsilon && triAlignmentValue < bestAlignmentValue)
			return true;

		// If alignment value is the same, but the new triangle is closer than the best distance
		if(triAlignmentValue == bestAlignmentValue && triImpactDistance < bestImpactDistance)
			return true;

		return false;
	}

	PX_FORCE_INLINE		bool		keepTriangleBasic(float triImpactDistance, float bestImpactDistance, float maxDistance)
	{
		// Reject triangle if further than the maxDistance
		if(triImpactDistance > maxDistance)
			return false;

		// If initial overlap happens, keep the triangle
		if(triImpactDistance == 0.0f)
			return true;

		// If new distance is more than epsilon closer than old distance
		if(triImpactDistance < bestImpactDistance)
			return true;

		return false;
	}

	PX_FORCE_INLINE PxVec3 cross100(const PxVec3& b)
	{
		return PxVec3(0.0f, -b.z, b.y);
	}
	PX_FORCE_INLINE PxVec3 cross010(const PxVec3& b)
	{
		return PxVec3(b.z, 0.0f, -b.x);
	}
	PX_FORCE_INLINE PxVec3 cross001(const PxVec3& b)
	{
		return PxVec3(-b.y, b.x, 0.0f);
	}

	//! Compute point as combination of barycentric coordinates
	PX_FORCE_INLINE PxVec3 computeBarycentricPoint(const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, PxReal u, PxReal v)
	{
		// This seems to confuse the compiler...
		// return (1.0f - u - v)*p0 + u*p1 + v*p2;
		const PxF32 w = 1.0f - u - v;
		return PxVec3(w * p0.x + u * p1.x + v * p2.x, w * p0.y + u * p1.y + v * p2.y, w * p0.z + u * p1.z + v * p2.z);
	}

	PX_FORCE_INLINE PxReal computeTetrahedronVolume(const PxVec3& x0, const PxVec3& x1, const PxVec3& x2, const PxVec3& x3, PxMat33& edgeMatrix)
	{
		const PxVec3 u1 = x1 - x0;
		const PxVec3 u2 = x2 - x0;
		const PxVec3 u3 = x3 - x0;

		edgeMatrix = PxMat33(u1, u2, u3);

		const PxReal det = edgeMatrix.getDeterminant();

		const PxReal volume = det / 6.0f;
		return volume;
	}

	PX_FORCE_INLINE PxReal computeTetrahedronVolume(const PxVec3& x0, const PxVec3& x1, const PxVec3& x2, const PxVec3& x3)
	{
		PxMat33 edgeMatrix;
		return computeTetrahedronVolume(x0, x1, x2, x3, edgeMatrix);
	}
	
	// IndexType should be PxU16 or PxU32.
	template<typename IndexType>
    PX_FORCE_INLINE PxReal computeTriangleMeshVolume(const PxVec3* vertices, const IndexType* indices,
                                                     const PxU32 numTriangles)
	{
		// See https://twitter.com/keenanisalive/status/1437178786286653445?lang=en
	    float volume = 0.0f;

	    for(PxU32 i = 0; i < numTriangles; ++i)
	    {
		    PxVec3 v0 = vertices[indices[3*i]];
		    PxVec3 v1 = vertices[indices[3 * i + 1]];
		    PxVec3 v2 = vertices[indices[3 * i + 2]];

			PxVec3 v0v1 = v0.cross(v1);

		    volume += v0v1.dot(v2);
	    }
		return volume / 6.0f;
	}

	// IndexType should be PxU16 or PxU32.
	// W in PxVec4 of vertices are ignored.
    template <typename IndexType>
    PX_FORCE_INLINE PxReal computeTriangleMeshVolume(const PxVec4* vertices, const IndexType* indices,
                                                     const PxU32 numTriangles)
    {
	    // See https://twitter.com/keenanisalive/status/1437178786286653445?lang=en
	    float volume = 0.0f;

		for(PxU32 i = 0; i < numTriangles; ++i)
	    {
		    PxVec3 v0 = vertices[indices[3 * i]].getXYZ();
		    PxVec3 v1 = vertices[indices[3 * i + 1]].getXYZ();
		    PxVec3 v2 = vertices[indices[3 * i + 2]].getXYZ();

		    PxVec3 v0v1 = v0.cross(v1);

		    volume += v0v1.dot(v2);
	    }
	    return volume / 6.0f;
    }

	/*!
	Extend an edge along its length by a factor
	*/
	PX_FORCE_INLINE void makeFatEdge(PxVec3& p0, PxVec3& p1, PxReal fatCoeff)
	{
		PxVec3 delta = p1 - p0;

		const PxReal m = delta.magnitude();
		if (m > 0.0f)
		{
			delta *= fatCoeff / m;
			p0 -= delta;
			p1 += delta;
		}
	}

#if 0
	/*!
	Extend an edge along its length by a factor
	*/
	PX_FORCE_INLINE void makeFatEdge(aos::Vec3V& p0, aos::Vec3V& p1, const aos::FloatVArg fatCoeff)
	{
		const aos::Vec3V delta = aos::V3Sub(p1, p0);
		const aos::FloatV m = aos::V3Length(delta);
		const aos::BoolV con = aos::FIsGrtr(m, aos::FZero());
		const aos::Vec3V fatDelta = aos::V3Scale(aos::V3ScaleInv(delta, m), fatCoeff);
		p0 = aos::V3Sel(con, aos::V3Sub(p0, fatDelta), p0);
		p1 = aos::V3Sel(con, aos::V3Add(p1, fatDelta), p1);
	}
#endif

	PX_FORCE_INLINE PxU32 closestAxis(const PxVec3& v, PxU32& j, PxU32& k)
	{
		// find largest 2D plane projection
		const PxF32 absPx = PxAbs(v.x);
		const PxF32 absNy = PxAbs(v.y);
		const PxF32 absNz = PxAbs(v.z);

		PxU32 m = 0; // x biggest axis
		j = 1;
		k = 2;
		if (absNy > absPx && absNy > absNz)
		{
			// y biggest
			j = 2;
			k = 0;
			m = 1;
		}
		else if (absNz > absPx)
		{
			// z biggest
			j = 0;
			k = 1;
			m = 2;
		}
		return m;
	}

	PX_FORCE_INLINE bool isAlmostZero(const PxVec3& v)
	{
		if (PxAbs(v.x) > 1e-6f || PxAbs(v.y) > 1e-6f || PxAbs(v.z) > 1e-6f)
			return false;
		return true;
	}

}  // namespace Gu

}

#endif
