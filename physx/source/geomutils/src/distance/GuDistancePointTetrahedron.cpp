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

#include "GuDistancePointTetrahedron.h"
#include "GuDistancePointTriangle.h"

using namespace physx;

PxVec3 Gu::closestPtPointTetrahedron(const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, const PxVec4& result)
{
	const PxVec3 ab = b - a;
	const PxVec3 ac = c - a;
	const PxVec3 ad = d - a;
	const PxVec3 bc = c - b;
	const PxVec3 bd = d - b;
	//point is outside of this face
	PxVec3 bestClosestPt(0.f, 0.f, 0.f);
	PxReal bestSqDist = PX_MAX_F32;
	if (result.x < 0.f)
	{
		// 0, 1, 2
		bestClosestPt = closestPtPointTriangle2(p, a, b, c, ab, ac);
		bestSqDist = bestClosestPt.dot(bestClosestPt);
	}

	if (result.y < 0.f)
	{
		// 0, 2, 3
		const PxVec3 closestPt = closestPtPointTriangle2(p, a, c, d, ac, ad);
		const PxReal sqDist = closestPt.dot(closestPt);
		if (sqDist < bestSqDist)
		{
			bestClosestPt = closestPt;
			bestSqDist = sqDist;
		}
	}

	if (result.z < 0.f)
	{
		// 0, 3, 1
		const PxVec3 closestPt = closestPtPointTriangle2(p, a, d, b, ad, ab);
		const PxReal sqDist = closestPt.dot(closestPt);
		if (sqDist < bestSqDist)
		{
			bestClosestPt = closestPt;
			bestSqDist = sqDist;
		}
	}

	if (result.w < 0.f)
	{
		// 1, 3, 2
		const PxVec3 closestPt = closestPtPointTriangle2(p, b, d, c, bd, bc);
		const PxReal sqDist = closestPt.dot(closestPt);
		if (sqDist < bestSqDist)
		{
			bestClosestPt = closestPt;
			bestSqDist = sqDist;
		}
	}

	return bestClosestPt;
}
