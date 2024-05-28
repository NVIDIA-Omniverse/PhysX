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

#ifndef GU_DISTANCE_POINT_TETRAHEDRON_H
#define GU_DISTANCE_POINT_TETRAHEDRON_H

#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "common/PxPhysXCommonConfig.h"
#include "GuDistancePointTriangle.h"
#include "foundation/PxMathUtils.h"

namespace physx
{
	namespace Gu
	{
		PX_INLINE PX_CUDA_CALLABLE PxVec4 PointOutsideOfPlane4(const PxVec3& p, const PxVec3& _a, const PxVec3& _b,
			const PxVec3& _c, const PxVec3& _d)
		{
			const PxVec3 ap = p - _a;
			const PxVec3 ab = _b - _a;
			const PxVec3 ac = _c - _a;
			const PxVec3 ad = _d - _a;

			const PxVec3 v0 = ab.cross(ac);
			const float signa0 = v0.dot(ap);
			const float signd0 = v0.dot(ad);// V3Dot(v0, _d);

			const PxVec3 v1 = ac.cross(ad);
			const float signa1 = v1.dot(ap);
			const float signd1 = v1.dot(ab);

			const PxVec3 v2 = ad.cross(ab);
			const float signa2 = v2.dot(ap);
			const float signd2 = v2.dot(ac);// V3Dot(v2, _c);

			const PxVec3 bd = _d - _b;
			const PxVec3 bc = _c - _b;

			const PxVec3 v3 = bd.cross(bc);
			const float signd3 = v3.dot(p - _b);
			const float signa3 = v3.dot(_a - _b);

			//if combined signDist is least zero, p is outside of that face
			PxVec4 result = PxVec4(signa0 * signd0, signa1 * signd1, signa2 * signd2, signa3 * signd3);

			return result;
		}

		PX_PHYSX_COMMON_API PxVec3 closestPtPointTetrahedron(const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, const PxVec4& result);

		PX_INLINE PX_CUDA_CALLABLE PxVec3 closestPtPointTetrahedron(const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d)
		{
			const PxVec3 ab = b - a;
			const PxVec3 ac = c - a;
			const PxVec3 ad = d - a;
			const PxVec3 bc = c - b;
			const PxVec3 bd = d - b;

			//point to face 0, 1, 2
			PxVec3 bestClosestPt = closestPtPointTriangle2(p, a, b, c, ab, ac);
			PxVec3 diff = bestClosestPt - p;
			PxReal bestSqDist = diff.dot(diff);

			// 0, 2, 3
			PxVec3 closestPt = closestPtPointTriangle2(p, a, c, d, ac, ad);
			diff = closestPt - p;
			PxReal sqDist = diff.dot(diff);
			if (sqDist < bestSqDist)
			{
				bestClosestPt = closestPt;
				bestSqDist = sqDist;
			}

			// 0, 3, 1
			closestPt = closestPtPointTriangle2(p, a, d, b, ad, ab);
			diff = closestPt - p;
			sqDist = diff.dot(diff);
			if (sqDist < bestSqDist)
			{
				bestClosestPt = closestPt;
				bestSqDist = sqDist;
			}

			// 1, 3, 2
			closestPt = closestPtPointTriangle2(p, b, d, c, bd, bc);
			diff = closestPt - p;
			sqDist = diff.dot(diff);
			if (sqDist < bestSqDist)
			{
				bestClosestPt = closestPt;
				bestSqDist = sqDist;
			}

			return bestClosestPt;
		}

		PX_INLINE PX_CUDA_CALLABLE PxVec3 closestPtPointTetrahedronWithInsideCheck(const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, const PxReal eps = 0)
		{
			PxVec4 tmpBarycentric;
			computeBarycentric(a, b, c, d, p, tmpBarycentric);
			if ((tmpBarycentric.x >= -eps && tmpBarycentric.x <= 1.f + eps) && (tmpBarycentric.y >= -eps && tmpBarycentric.y <= 1.f + eps) &&
				(tmpBarycentric.z >= -eps && tmpBarycentric.z <= 1.f + eps) && (tmpBarycentric.w >= -eps && tmpBarycentric.w <= 1.f + eps))
				return p;
			return closestPtPointTetrahedron(p, a, b, c, d);
		}
	}
}

#endif

