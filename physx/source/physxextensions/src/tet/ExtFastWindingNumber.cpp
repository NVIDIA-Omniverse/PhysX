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

#include "ExtFastWindingNumber.h"
#include "foundation/PxSort.h"

#include "foundation/PxMath.h"

#include "ExtUtilities.h"

//The following paper explains all the techniques used in this file
//http://www.dgp.toronto.edu/projects/fast-winding-numbers/fast-winding-numbers-for-soups-and-clouds-siggraph-2018-barill-et-al.pdf

namespace physx
{
namespace Ext
{
	PxF64 computeWindingNumber(const PxArray<Gu::BVHNode>& tree, const PxVec3d& q, PxF64 beta, const PxHashMap<PxU32, ClusterApproximationF64>& clusters,
		const PxArray<Triangle>& triangles, const PxArray<PxVec3d>& points)
	{
		return Gu::computeWindingNumber<PxF64, PxVec3d>(tree.begin(), q, beta, clusters, reinterpret_cast<const PxU32*>(triangles.begin()), points.begin());
	}

	void precomputeClusterInformation(PxArray<Gu::BVHNode>& tree, const PxArray<Triangle>& triangles,
		const PxArray<PxVec3d>& points, PxHashMap<PxU32, ClusterApproximationF64>& result, PxI32 rootNodeIndex)
	{
		Gu::precomputeClusterInformation<PxF64, PxVec3d>(tree.begin(), reinterpret_cast<const PxU32*>(triangles.begin()), triangles.size(), points.begin(), result, rootNodeIndex);
	}
}
}
