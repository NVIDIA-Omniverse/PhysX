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

#include "foundation/PxVec2.h"
#include "foundation/PxVecMath.h"
#include "GuDistancePointTriangle.h"
#include "PxgFEMCloth.h"

#define DEFORMABLE_BARYCENTRIC_THRESHOLD (1.0e-6f)
#define DEFORMABLE_ONE_MINUS_BARYCENTRIC_THRESHOLD (1.0f - DEFORMABLE_BARYCENTRIC_THRESHOLD)

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

__device__ static void closestPtPointEdge(const PxVec3& v0, const PxVec3& ev0, const PxVec3& ev1, PxReal& w)
{
	const PxVec3 r = ev1 - ev0;
	const PxReal d2 = r.dot(r);

	w = (d2 > 0.0f) ? PxClamp(r.dot(v0 - ev0) / d2, 0.0f, 1.0f) : 0.5f;
}

// Computes the closest points on two infinite lines.
// Returns false if the edges are parallel or degenerate.
// Outputs:
//   - s, t: parametric positions along edge0 and edge1 (can be negative for infinite lines)
//   - distSq: squared distance between the two closest points
PX_FORCE_INLINE __device__ static bool closestPtLineLine(const PxVec3& e0v0, const PxVec3& e0v1, const PxVec3& e1v0, const PxVec3& e1v1,
														 PxReal& s, PxReal& t, PxReal& distSq)
{
	const PxVec3 r0 = e0v1 - e0v0; // Edge0 direction
	const PxVec3 r1 = e1v1 - e1v0; // Edge1 direction
	const PxVec3 d = e1v0 - e0v0;  // Displacement between starting points

	const PxReal r0Len2 = r0.magnitudeSquared();
	const PxReal r1Len2 = r1.magnitudeSquared();

	// Handle degenerate edges: If either edge has zero length, return midpoints
	if(r0Len2 < 1.0e-18f | r1Len2 < 1.0e-18f)
	{
		s = t = 0.0f;
		distSq = d.magnitudeSquared();
		return false;
	}

	const PxReal r0Len = PxSqrt(r0Len2);
	const PxReal r1Len = PxSqrt(r1Len2);

	const PxVec3 u0 = r0 / r0Len;
	const PxVec3 u1 = r1 / r1Len;

	const PxReal cosTheta = PxAbs(u0.dot(u1));

	// Edges are nearly parallel (angle < 0.81).
	if(cosTheta > 0.9999f)
	{
		// Pick point e0v0 on edge0 and project to edge1
		PxReal tProj = (e0v0 - e1v0).dot(r1) / r1Len2;
		t = tProj;
		s = 0.0f;

		PxVec3 pt0 = e0v0;
		PxVec3 pt1 = e1v0 + r1 * tProj;
		distSq = (pt1 - pt0).magnitudeSquared();
		return false;
	}

	// PxReal a00 = 1.0f;
	// PxReal a11 = 1.0f;

	PxReal a01 = -u0.dot(u1);
	PxReal b0 = u0.dot(d) / r0Len;
	PxReal b1 = -u1.dot(d) / r1Len;

	PxReal det = 1.0f - a01 * a01;

	if(det < 1.0e-9f)
	{
		s = t = 0.0f;
		distSq = d.magnitudeSquared();
		return false;
	}

	// Compute actual closest points
	const PxReal invDet = 1.0f / det;
	s = (b0 - b1 * a01) * invDet;
	t = (b1 - a01 * b0) * invDet;

	const PxVec3 pt0 = e0v0 + s * r0;
	const PxVec3 pt1 = e1v0 + t * r1;
	distSq = (pt1 - pt0).magnitudeSquared();

	return true;
}

// Computes the closest points on two edges (finite edges)
PX_FORCE_INLINE __device__ static bool closestPtEdgeEdge(const PxVec3& e0v0, const PxVec3& e0v1, const PxVec3& e1v0, const PxVec3& e1v1,
														 PxReal& s, PxReal& t, PxReal& distSq)
{
	if(!closestPtLineLine(e0v0, e0v1, e1v0, e1v1, s, t, distSq))
	{
		return false; // Degenerate edges
	}

	// Falls back to point edge distance.
	if(s<0.0f | s> 1.0f | t<0.0f | t> 1.0f)
	{
		PxReal w[4];
		closestPtPointEdge(e0v0, e1v0, e1v1, w[0]);
		closestPtPointEdge(e0v1, e1v0, e1v1, w[1]);
		closestPtPointEdge(e1v0, e0v0, e0v1, w[2]);
		closestPtPointEdge(e1v1, e0v0, e0v1, w[3]);

		// Candidates for closest point selection
		const PxReal candidates[4][2] = { { 0.0f, w[0] }, { 1.0f, w[1] }, { w[2], 0.0f }, { w[3], 1.0f } };

		PxI32 minIndex = 0;
		distSq = PX_MAX_REAL;

#pragma unroll
		// Compare the four closest points and select the minimum distance pair.
		for(int i = 0; i < 4; ++i)
		{
			const PxReal sC = candidates[i][0];
			const PxReal tC = candidates[i][1];
			const PxVec3 p0 = e0v0 * (1.0f - sC) + e0v1 * sC;
			const PxVec3 p1 = e1v0 * (1.0f - tC) + e1v1 * tC;
			const PxReal d2 = (p0 - p1).magnitudeSquared();
			minIndex = (d2 < distSq) ? i : minIndex;
			distSq = PxMin(d2, distSq);
		}

		// Assign the best closest point.
		assert(minIndex != -1);
		s = candidates[minIndex][0];
		t = candidates[minIndex][1];
	}

	return true;
}

// Type0: minimal triangle set covering all edges and vertices (compact encoding)
// Type1: more uniformly distributed edges and vertices across triangles (balanced encoding)
PX_FORCE_INLINE __device__ static bool isType0EdgeActive(PxU32 edgeAuthorship, PxU32 localEdgeIndex)
{
	return (edgeAuthorship & (1U << (EdgeEncoding::TYPE0_EDGE_BASE_POS + localEdgeIndex))) != 0;
}

PX_FORCE_INLINE __device__ static bool isType1EdgeActive(PxU32 edgeAuthorship, PxU32 localEdgeIndex)
{
	return (edgeAuthorship & (1U << (EdgeEncoding::TYPE1_EDGE_BASE_POS + localEdgeIndex))) != 0;
}

PX_FORCE_INLINE __device__ static bool isType0VertexActive(PxU32 edgeAuthorship, PxU32 localVertexIndex)
{
	return (edgeAuthorship & (1U << (EdgeEncoding::TYPE0_VERTEX0_ACTIVE_POS + localVertexIndex))) != 0;
}

PX_FORCE_INLINE __device__ static bool isType1VertexActive(PxU32 edgeAuthorship, PxU32 localVertexIndex)
{
	return (edgeAuthorship & (1U << (EdgeEncoding::TYPE1_VERTEX0_ACTIVE_POS + localVertexIndex))) != 0;
}

#endif // __DEFORMABLE_COLLISION_CUH__