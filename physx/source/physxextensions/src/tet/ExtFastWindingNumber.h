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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.

#ifndef EXT_FAST_WINDING_NUMBER_H
#define EXT_FAST_WINDING_NUMBER_H


#include "ExtVec3.h"
#include "GuWindingNumberT.h"

namespace physx
{
namespace Ext
{
	using Triangle = Gu::IndexedTriangleT<PxI32>;
	using Triangle16 = Gu::IndexedTriangleT<PxI16>;
	
	typedef Gu::ClusterApproximationT<PxF64, Vec3> ClusterApproximationF64;
	typedef Gu::SecondOrderClusterApproximationT<PxF64, Vec3> SecondOrderClusterApproximationF64;
	
	PxF64 computeWindingNumber(const PxArray<Gu::BVHNode>& tree, const Vec3& q, PxF64 beta, const PxHashMap<PxU32, ClusterApproximationF64>& clusters,
		const PxArray<Triangle>& triangles, const PxArray<Vec3>& points);

	void precomputeClusterInformation(PxArray<Gu::BVHNode>& tree, const PxArray<Triangle>& triangles,
		const PxArray<Vec3>& points, PxHashMap<PxU32, ClusterApproximationF64>& result, PxI32 rootNodeIndex = 0);
}
}

#endif

