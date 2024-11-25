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

#ifndef GU_MESH_MESH_COLLISION_H
#define GU_MESH_MESH_COLLISION_H

#include "foundation/PxAssert.h"
#include "foundation/PxMath.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "GuCollisionSDF.h"
#include "GuDistancePointTriangle.h"
#include "GuTriangle.h"

namespace physx
{
namespace Gu
{

template <typename T>
int argmin(const T& v0, const T& v1)
{
	if (v0 < v1)
		return 0;
	return 1;
}

template <typename T>
int argmin(const T& v0, const T& v1, const T& v2, const T& v3)
{
	const int ma = argmin(v0, v1), mb = argmin(v2, v3);
	if (!argmin((!ma ? v0 : v1),(!mb ? v2: v3)))
		return ma;
	return mb+2;
}

// Based on one SDF evaluation at the centroid, can the circumsphere of the triangle
// `v0`, `v1`, `v2` get closer to the surface given by `sdf` than `cutoffDistance`?
static PX_INLINE bool sdfTriangleSphericalCull(
		const CollisionSDF& PX_RESTRICT sdf,
		const PxVec3& PX_RESTRICT v0, const PxVec3& PX_RESTRICT v1, const PxVec3& PX_RESTRICT v2,
		PxReal cutoffDistance)
{
	const PxReal third = 1.0f / 3.0f;
	const PxVec3 centroid = (v0 + v1 + v2) * third;

	const PxReal sphereRadiusSq = PxMax(
		(v0 - centroid).magnitudeSquared(),
		PxMax((v1 - centroid).magnitudeSquared(), (v2 - centroid).magnitudeSquared()));

	const PxVec3 boxPos = sdf.clampToBox(centroid);
	const PxReal centroidToBoxSq = (centroid - boxPos).magnitudeSquared();
	// TODO(CA): consider minimum of SDF on box boundary, making this check tighter
	if (PxSqrt(centroidToBoxSq) > PxSqrt(sphereRadiusSq) + cutoffDistance)
		return false; //Early out without touching SDF data

	const PxReal centroidSdf = sdf.dist(centroid);

	return centroidSdf < PxSqrt(sphereRadiusSq) + cutoffDistance;
}


// Find maximum separation of an sdf and a triangle and find the contact point and normal separation is below `cutoffDistance`
// Return the separation, or`PX\_MAX\_F32` if it exceeds `cutoffDistance`
template <PxU32 TMaxLineSearchIters = 0, PxU32 TMaxPGDIterations = 32, bool TFastGrad = true>
PX_INLINE PxReal sdfTriangleCollision(
		const CollisionSDF& PX_RESTRICT sdf,
		const PxVec3& PX_RESTRICT v0, const PxVec3& PX_RESTRICT v1, const PxVec3& PX_RESTRICT v2,
		PxVec3& point, PxVec3& dir, PxReal cutoffDistance)
{
	const PxReal third = 1.0f / 3.0f;
	const PxVec3 centroid = (v0 + v1 + v2) * third;

	// barycentric coordinates, corresponding to v0, v1, v2
	PxVec3 c(0.f);

	// choose starting iterate
	const int start = argmin(sdf.dist(v0), sdf.dist(v1), sdf.dist(v2), sdf.dist(centroid));
	switch (start)
	{
		case 0:
			c.x = 1.f; break;
		case 1:
			c.y = 1.f; break;
		case 2:
			c.z = 1.f; break;
		case 3:
			c = PxVec3(third); break;
		default:
			PX_ASSERT(false);
	}

	PxReal stepSize = 0.25;
							 // we could also compute the gradient's lipschitz constant when baking!
	PxVec3 dfdp;  // gradient w.r.t. p
	const PxReal toleranceSq = 1e-10f;

	for (PxU32 i = 0; i < TMaxPGDIterations; ++i)
	{
		const PxVec3 p = c.x * v0 + c.y * v1 + c.z * v2;
		PxReal dist_old = 0;
		if (TFastGrad)
			dist_old = sdf.dist(p, &dfdp);
		else
			dfdp = sdf.grad(p);
		const PxReal dfdpMagSq = dfdp.magnitudeSquared();

		if (dfdpMagSq == 0.0f)
		{
			// TODO(CA): consider expanding this into a stopping criterion
			// At a critical point. Take a small step away into an arbitrary direction
			dfdp = PxVec3(0.5718465865353257f, 0.7055450997557186f, 0.41856611625714474f);
		}
		else
			dfdp *= PxRecipSqrt(dfdpMagSq);

		// Simple projected gradient descent
		const PxVec3 dfdc = PxVec3(dfdp.dot(v0-p), dfdp.dot(v1-p), dfdp.dot(v2-p));

		const PxVec3 c_old = c;
		if (TMaxLineSearchIters) //Line Search is quite expensive since it increases the number of expensive calls to sdf.dist by approximately a factor of MAX_LINE_SEARCH_ITERATIONS
		{
			PxReal s = 1;
			if (!TFastGrad)
				dist_old = sdf.dist(p);
			c = closestPtPointBaryTriangle(c_old - s * dfdc);
			for (PxU32 ls_it = 0; ls_it < TMaxLineSearchIters; ++ls_it)
			{
				// restore barycentric coordinates
				const PxVec3 p_new = c.x * v0 + c.y * v1 + c.z * v2;
				if (sdf.dist(p_new) <= dist_old)
				{
#if 0
					if (ls_it > 0)
						printf("%d: %d ls iterations\n", i, ls_it+1);
#endif
					break;
				}
				s *= 0.5f;
				c = closestPtPointBaryTriangle(c_old - s * dfdc);
			}
		}
		else
		{
			// take step and restore barycentric coordinates
			c = closestPtPointBaryTriangle(c - stepSize * dfdc);
		}
		// this detects a minimum found on the boundary
		if ((c - c_old).magnitudeSquared() < toleranceSq)
			break;
		stepSize *= 0.8f; // line search will get rid of this
	}

	const PxVec3 p = c.x * v0 + c.y * v1 + c.z * v2;
	point = p;
	return sdf.distUsingGradient(p, dir, cutoffDistance);
}

// Get the indices of the triangle at position `triIdx`.
// `T` should be `PxU16` if the mesh has 16 bit indices, and `PxU32` otherwise
template <typename T>
PX_INLINE Gu::IndexedTriangle32 getTriangleVertexIndices(const void* triangles, PxU32 triIdx)
{
	const T* trisCast = reinterpret_cast<const T*>(triangles);
	return {trisCast[triIdx*3], trisCast[triIdx*3+1], trisCast[triIdx*3+2]};
}

}  // namespace Gu
}  // namespace physx
#endif
