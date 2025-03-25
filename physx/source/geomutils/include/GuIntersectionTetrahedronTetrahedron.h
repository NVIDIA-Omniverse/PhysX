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

#ifndef GU_INTERSECTION_TETRAHEDRON_TETRAHEDRON_H
#define GU_INTERSECTION_TETRAHEDRON_TETRAHEDRON_H

#include "foundation/PxPlane.h"
#include "common/PxPhysXCommonConfig.h"

namespace physx
{
	namespace Gu
	{
		struct Tetrahedron
		{
			PxVec3 verts[4];
			PxPlane planes[4]; 
			PxVec3 centroid;
		};

		PX_INLINE PX_CUDA_CALLABLE PxPlane createPlane(const PxVec3& pa, const PxVec3& pb, const PxVec3& pc, const PxVec3& pd)
		{
			PxPlane plane(pa, pb, pc);
			PxReal distance = plane.distance(pd);
			if (distance > 0.f)
			{
				plane.n = -plane.n;
				plane.d = -plane.d;
			}
			return plane;
		}

		PX_INLINE PX_CUDA_CALLABLE void constructTetrahedron(const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d,
			Tetrahedron& tet)
		{
			tet.verts[0] = a; tet.verts[1] = b; tet.verts[2] = c; tet.verts[3] = d;
			tet.planes[0] = createPlane(a, b, c, d);
			tet.planes[1] = createPlane(a, b, d, c);
			tet.planes[2] = createPlane(a, c, d, b);
			tet.planes[3] = createPlane(b, c, d, a); 
			tet.centroid = (a + b + c) * (1.f / 3.f);
		}

		PX_INLINE PX_CUDA_CALLABLE PxReal minProject(const PxPlane& plane, const Tetrahedron& tet)
		{
			/*return PxMin(plane.distance(tet.verts[0]), PxMin(plane.distance(tet.verts[1]), 
				PxMin(plane.distance(tet.verts[2]), plane.distance(tet.verts[3]))));*/

		return PxMin(plane.n.dot(tet.verts[0]),PxMin(plane.n.dot(tet.verts[1]),
			PxMin(plane.n.dot(tet.verts[2]), plane.n.dot(tet.verts[3])))) + plane.d;
		}


		PX_INLINE PX_CUDA_CALLABLE PxReal testSeparatingAxis(const PxVec3& axis, const Tetrahedron& tet0, const Tetrahedron& tet1)
		{

			PxReal min0, max0, min1, max1;
			min0 = max0 = tet0.verts[0].dot(axis);
			min1 = max1 = tet1.verts[0].dot(axis);
			for (PxU32 i = 1; i < 4; ++i)
			{
				PxReal proj0 = tet0.verts[i].dot(axis);
				PxReal proj1 = tet1.verts[i].dot(axis);

				min0 = PxMin(proj0, min0);
				max0 = PxMax(proj0, max0);
				min1 = PxMin(proj1, min1);
				max1 = PxMax(proj1, max1);
			}

			return PxMax(min1 - max0, min0 - max1);
		}

		template <bool TDoCross = true>
		PX_INLINE PX_CUDA_CALLABLE PxReal satIntersect(const Tetrahedron& tet0, const Tetrahedron& tet1, const PxReal tolerance)
		{
			PxReal sep = minProject(tet0.planes[0], tet1);
			if (sep > tolerance)
				return sep;
			sep = PxMax(sep, minProject(tet0.planes[1], tet1));
			if (sep > tolerance)
				return sep;
			sep = PxMax(sep, minProject(tet0.planes[2], tet1));
			if (sep > tolerance)
				return sep;
			sep = PxMax(sep, minProject(tet0.planes[3], tet1));
			if (sep > tolerance)
				return sep;

			sep = PxMax(sep, minProject(tet1.planes[0], tet0));
			if (sep > tolerance)
				return sep;
			sep = PxMax(sep, minProject(tet1.planes[1], tet0));
			if (sep > tolerance)
				return sep;
			sep = PxMax(sep, minProject(tet1.planes[2], tet0));
			if (sep > tolerance)
				return sep;
			sep = PxMax(sep, minProject(tet1.planes[3], tet0));
			if (sep > tolerance)
				return sep;

			if (TDoCross)
			{
				PxVec3 axes0[6];
				PxVec3 axes1[6];

				axes0[0] = tet0.verts[1] - tet0.verts[0];
				axes0[1] = tet0.verts[2] - tet0.verts[0];
				axes0[2] = tet0.verts[3] - tet0.verts[0];
				axes0[3] = tet0.verts[2] - tet0.verts[1];
				axes0[4] = tet0.verts[3] - tet0.verts[1];
				axes0[5] = tet0.verts[3] - tet0.verts[2];

				axes1[0] = tet1.verts[1] - tet1.verts[0];
				axes1[1] = tet1.verts[2] - tet1.verts[0];
				axes1[2] = tet1.verts[3] - tet1.verts[0];
				axes1[3] = tet1.verts[2] - tet1.verts[1];
				axes1[4] = tet1.verts[3] - tet1.verts[1];
				axes1[5] = tet1.verts[3] - tet1.verts[2];

				for (PxU32 i = 0; i < 6; ++i)
				{
					const PxVec3 axis0 = axes0[i];
					for (PxU32 j = 0; j < 6; ++j)
					{
						const PxVec3 axis1 = axes1[j];
						PxVec3 sepAxis = axis0.cross(axis1);
						const PxReal magSq = sepAxis.magnitudeSquared();
						if (magSq > 1e-5f)
						{
							sepAxis = sepAxis * (1.f / PxSqrt(magSq));
							const PxReal tSep = testSeparatingAxis(sepAxis, tet0, tet1);
							sep = PxMax(sep, tSep);
							if (sep > tolerance)
								return sep;

						}
					}
				}
			}

			return sep;
		}
	} // namespace Gu
}

#endif
