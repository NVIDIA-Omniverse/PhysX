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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef __TRIANGLE_TRIANGLE_CUH__
#define __TRIANGLE_TRIANGLE_CUH__

#include "foundation/PxPlane.h"
#include "foundation/PxVec3.h"

struct Triangle
{
	physx::PxVec3 verts[3];
	physx::PxPlane triPlane;
	physx::PxVec3 centroid; //mShapeCoMInTrimeshVertexSpace
};


__device__ PxReal minProject(const PxPlane& plane, const Triangle& triangle)
{
	/*return PxMin(plane.distance(tet.verts[0]), PxMin(plane.distance(tet.verts[1]),
		PxMin(plane.distance(tet.verts[2]), plane.distance(tet.verts[3]))));*/

	return PxMin(plane.n.dot(triangle.verts[0]), PxMin(plane.n.dot(triangle.verts[1]),
		plane.n.dot(triangle.verts[2]))) + plane.d;
}


__device__ inline PxReal testSeparatingAxis(const PxVec3& axis, const Triangle& triangle0, const Triangle& triangle1)
{

	PxReal min0, max0, min1, max1;
	min0 = max0 = triangle0.verts[0].dot(axis);
	min1 = max1 = triangle1.verts[0].dot(axis);
	for (PxU32 i = 1; i < 3; ++i)
	{
		PxReal proj0 = triangle0.verts[i].dot(axis);
		PxReal proj1 = triangle1.verts[i].dot(axis);

		min0 = PxMin(proj0, min0);
		max0 = PxMax(proj0, max0);
		min1 = PxMin(proj1, min1);
		max1 = PxMax(proj1, max1);
	}

	return PxMax(min1 - max0, min0 - max1);
}

template <bool TDoCross = true>
__device__ inline PxReal satIntersect(const Triangle& triangle0, const Triangle& triangle1, const PxReal tolerance)
{
	PxReal sep = minProject(triangle0.triPlane, triangle1);
	if (sep > tolerance)
		return sep;

	sep = PxMax(sep, minProject(triangle1.triPlane, triangle0));
	if (sep > tolerance)
		return sep;

	if (TDoCross)
	{
		PxVec3 axes0[3];
		PxVec3 axes1[3];

		axes0[0] = triangle0.verts[1] - triangle0.verts[0];
		axes0[1] = triangle0.verts[2] - triangle0.verts[0];
		axes0[2] = triangle0.verts[2] - triangle0.verts[1];

		axes1[0] = triangle1.verts[1] - triangle1.verts[0];
		axes1[1] = triangle1.verts[2] - triangle1.verts[0];
		axes1[2] = triangle1.verts[2] - triangle1.verts[1];

		for (PxU32 i = 0; i < 3; ++i)
		{
			const PxVec3 axis0 = axes0[i];
			for (PxU32 j = 0; j < 3; ++j)
			{
				const PxVec3 axis1 = axes1[j];
				PxVec3 sepAxis = axis0.cross(axis1);
				const PxReal magSq = sepAxis.magnitudeSquared();
				if (magSq > 1e-5f)
				{
					sepAxis = sepAxis * (1.f / PxSqrt(magSq));
					const PxReal tSep = testSeparatingAxis(sepAxis, triangle0, triangle1);
					sep = PxMax(sep, tSep);
					if (sep > tolerance)
						return sep;

				}
			}
		}
	}

	return sep;
}

#endif
