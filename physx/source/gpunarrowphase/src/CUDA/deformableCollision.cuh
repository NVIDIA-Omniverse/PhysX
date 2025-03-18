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

#ifndef __DEFORMABLE_COLLISION_CUH__
#define __DEFORMABLE_COLLISION_CUH__

#include "foundation/PxVecMath.h"
#include "GuDistancePointTriangle.h"

PX_FORCE_INLINE __device__ PxBounds3 triBoundingBox(const PxVec3& worldV0, const PxVec3& worldV1, const PxVec3& worldV2)
{
	PxBounds3 triangleBound;

	PxReal tX0 = PxMin(worldV0.x, worldV1.x);
	PxReal tY0 = PxMin(worldV0.y, worldV1.y);
	PxReal tZ0 = PxMin(worldV0.z, worldV1.z);

	triangleBound.minimum.x = PxMin(tX0, worldV2.x);
	triangleBound.minimum.y = PxMin(tY0, worldV2.y);
	triangleBound.minimum.z = PxMin(tZ0, worldV2.z);

	// compute max
	tX0 = PxMax(worldV0.x, worldV1.x);
	tY0 = PxMax(worldV0.y, worldV1.y);
	tZ0 = PxMax(worldV0.z, worldV1.z);

	triangleBound.maximum.x = PxMax(tX0, worldV2.x);
	triangleBound.maximum.y = PxMax(tY0, worldV2.y);
	triangleBound.maximum.z = PxMax(tZ0, worldV2.z);

	return triangleBound;
}

PX_FORCE_INLINE __device__ PxBounds3 tetBoundingBox(const PxVec3& worldV0, const PxVec3& worldV1, const PxVec3& worldV2, const PxVec3& worldV3)
{
	PxBounds3 result;

	PxReal tX0 = PxMin(worldV0.x, worldV1.x);
	PxReal tY0 = PxMin(worldV0.y, worldV1.y);
	PxReal tZ0 = PxMin(worldV0.z, worldV1.z);

	PxReal tX1 = PxMin(worldV2.x, worldV3.x);
	PxReal tY1 = PxMin(worldV2.y, worldV3.y);
	PxReal tZ1 = PxMin(worldV2.z, worldV3.z);

	result.minimum.x = PxMin(tX0, tX1);
	result.minimum.y = PxMin(tY0, tY1);
	result.minimum.z = PxMin(tZ0, tZ1);

	// compute max
	tX0 = PxMax(worldV0.x, worldV1.x);
	tY0 = PxMax(worldV0.y, worldV1.y);
	tZ0 = PxMax(worldV0.z, worldV1.z);

	tX1 = PxMax(worldV2.x, worldV3.x);
	tY1 = PxMax(worldV2.y, worldV3.y);
	tZ1 = PxMax(worldV2.z, worldV3.z);

	result.maximum.x = PxMax(tX0, tX1);
	result.maximum.y = PxMax(tY0, tY1);
	result.maximum.z = PxMax(tZ0, tZ1);

	return result;
}

__device__ inline bool computeTetBarycentric(const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, const PxVec3& point,
											 float4& barycentric, const PxReal eps = 1e-5f)
{
	const PxReal minEps = -eps;
	const PxReal maxEps = 1.f + eps;
	const PxVec3 ba = b - a;
	const PxVec3 ca = c - a;
	const PxVec3 da = d - a;
	const PxVec3 pa = point - a;

	// PxMat33 bcd(ba, ca, da);
	// const PxReal detBcd = bcd.getDeterminant();
	const PxReal detBcd = ba.dot(ca.cross(da));

	/*PxMat33 pcd(pa, ca, da);
	const PxReal detPcd = pcd.getDeterminant();*/
	const PxReal detPcd = pa.dot(ca.cross(da));

	const PxReal v = detPcd / detBcd;

	/*PxMat33 bpd(ba, pa, da);
	const PxReal detBpd = bpd.getDeterminant();*/
	const PxReal detBpd = ba.dot(pa.cross(da));
	const PxReal w = detBpd / detBcd;

	/*PxMat33 bcp(ba, ca, pa);
	const PxReal detBcp = bcp.getDeterminant();*/
	const PxReal detBcp = ba.dot(ca.cross(pa));

	const PxReal x = detBcp / detBcd;

	const PxReal u = 1.f - v - w - x;
	barycentric = make_float4(u, v, w, x);

	// After clamping the barycentrics might not sum up to 1 anymore
	// Clamping should be done because otherwise slightly negative barycentrics can lead to a negative inverse mass, e. g.
	// when 3 vertices of a tetrahedron have inverse mass zero and the forth vertex happens to get a negative barycentric coordinate
	barycentric.x = PxClamp(barycentric.x, 0.0f, 1.0f);
	barycentric.y = PxClamp(barycentric.y, 0.0f, 1.0f);
	barycentric.z = PxClamp(barycentric.z, 0.0f, 1.0f);
	barycentric.w = PxClamp(barycentric.w, 0.0f, 1.0f);

	return (u >= minEps && u <= maxEps) && (v >= minEps && v <= maxEps) && (w >= minEps && w <= maxEps) && (x >= minEps && x <= maxEps);
}

__device__ inline bool computeTetBarycentric(PxVec3* verts, const PxVec3& point, float4& barycentric, const float eps = 1e-5f)
{
	return computeTetBarycentric(verts[0], verts[1], verts[2], verts[3], point, barycentric, eps);
}

__device__ inline PxVec3 closestPtPointTriangle(const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, float& s, float& t)
{
	// Check if P in vertex region outside A
	const PxVec3 ab = b - a;
	const PxVec3 ac = c - a;
	const PxVec3 ap = p - a;
	const float d1 = ab.dot(ap);
	const float d2 = ac.dot(ap);
	if(d1 <= 0.0f && d2 <= 0.0f)
	{
		s = 0.0f;
		t = 0.0f;
		return a; // Barycentric coords 1,0,0
	}

	// Check if P in vertex region outside B
	const PxVec3 bp = p - b;
	const float d3 = ab.dot(bp);
	const float d4 = ac.dot(bp);
	if(d3 >= 0.0f && d4 <= d3)
	{
		s = 1.0f;
		t = 0.0f;
		return b; // Barycentric coords 0,1,0
	}

	// Check if P in edge region of AB, if so return projection of P onto AB
	const float vc = d1 * d4 - d3 * d2;
	if(vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
	{
		const float v = d1 / (d1 - d3);
		s = v;
		t = 0.0f;
		return a + v * ab; // barycentric coords (1-v, v, 0)
	}

	// Check if P in vertex region outside C
	const PxVec3 cp = p - c;
	const float d5 = ab.dot(cp);
	const float d6 = ac.dot(cp);
	if(d6 >= 0.0f && d5 <= d6)
	{
		s = 0.0f;
		t = 1.0f;
		return c; // Barycentric coords 0,0,1
	}

	// Check if P in edge region of AC, if so return projection of P onto AC
	const float vb = d5 * d2 - d1 * d6;
	if(vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
	{
		const float w = d2 / (d2 - d6);
		s = 0.0f;
		t = w;
		return a + w * ac; // barycentric coords (1-w, 0, w)
	}

	// Check if P in edge region of BC, if so return projection of P onto BC
	const float va = d3 * d6 - d5 * d4;
	if(va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
	{
		const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		s = 1.0f - w;
		t = w;
		return b + w * (c - b); // barycentric coords (0, 1-w, w)
	}

	// P inside face region. Compute Q through its barycentric coords (u,v,w)
	const float denom = 1.0f / (va + vb + vc);
	const float v = vb * denom;
	const float w = vc * denom;
	s = v;
	t = w;
	return a + ab * v + ac * w;
}

__device__ static PxReal distancePointTriangleSquared(const PxVec3 p, const PxVec3 a, const PxVec3 b, const PxVec3 c, PxVec3& closestP)
{
	const PxVec3 ab = b - a;
	const PxVec3 ac = c - a;
	closestP = Gu::closestPtPointTriangle2(p, a, b, c, ab, ac);
	return (p - closestP).magnitudeSquared();
}

#endif // __DEFORMABLE_COLLISION_CUH__