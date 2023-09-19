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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "GuWindingNumberT.h"
#include "GuWindingNumber.h"

namespace physx
{
namespace Gu
{
	PxF32 computeWindingNumber(const Gu::BVHNode* tree, const PxVec3& q, PxF32 beta, const PxHashMap<PxU32, ClusterApproximation>& clusters,
		const PxU32* triangles, const PxVec3* points)
	{
		return Gu::computeWindingNumber<PxF32, PxVec3>(tree, q, beta, clusters, triangles, points);
	}

	PxF32 computeWindingNumber(const Gu::BVHNode* tree, const PxVec3& q, const PxHashMap<PxU32, ClusterApproximation>& clusters,
		const PxU32* triangles, const PxVec3* points)
	{
		return Gu::computeWindingNumber<PxF32, PxVec3>(tree, q, 2.0f, clusters, triangles, points);
	}

	void precomputeClusterInformation(const Gu::BVHNode* tree, const PxU32* triangles, const PxU32 numTriangles,
		const PxVec3* points, PxHashMap<PxU32, ClusterApproximation>& result, PxI32 rootNodeIndex)
	{
		Gu::precomputeClusterInformation<PxF32, PxVec3>(tree, triangles, numTriangles, points, result, rootNodeIndex);
	}

	PxF32 computeWindingNumber(const PxVec3& q, const PxU32* triangles, const PxU32 numTriangles, const PxVec3* points)
	{
		PxReal windingNumber = 0.0f;
		for (PxU32 i = 0; i < numTriangles; ++i) 
		{
			const PxU32* tri = &triangles[3 * i];
			windingNumber += Gu::evaluateExact<PxReal, PxVec3>(points[tri[0]], points[tri[1]], points[tri[2]], q);
		}
		return windingNumber;
	}
}
}
