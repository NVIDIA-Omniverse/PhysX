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

#include "extensions/PxRemeshingExt.h"
#include "foundation/PxHashMap.h"

using namespace physx;

static void assignTriangle(PxArray<PxU32>& triangles, PxU32 triIndex, PxU32 a, PxU32 b, PxU32 c)
{
	triangles[3 * triIndex] = a;
	triangles[3 * triIndex + 1] = b;
	triangles[3 * triIndex + 2] = c;
}

static void addTriangle(PxArray<PxU32>& triangles, PxU32 a, PxU32 b, PxU32 c)
{
	triangles.pushBack(a);
	triangles.pushBack(b);
	triangles.pushBack(c);
}

static void subdivideTriangle(int i, int ab, int bc, int ac, PxArray<PxU32>& triangles, PxArray<PxVec3>& points)
{
	PxU32 tri[3] = { triangles[3 * i],triangles[3 * i + 1],triangles[3 * i + 2] };
	if (ab >= 0 && bc >= 0 && ac >= 0)
	{
		addTriangle(triangles, tri[0], ab, ac);
		addTriangle(triangles, tri[1], bc, ab);
		addTriangle(triangles, tri[2], ac, bc);
		assignTriangle(triangles, i, ab, bc, ac);
	}
	else if (ac >= 0 && ab >= 0)
	{
		float dB = (points[ac] - points[tri[1]]).magnitudeSquared();
		float dC = (points[ab] - points[tri[2]]).magnitudeSquared();
		if (dB < dC)
		{
			addTriangle(triangles, tri[1], tri[2], ac);
			addTriangle(triangles, tri[1], ac, ab);
		}
		else
		{
			addTriangle(triangles, tri[1], tri[2], ab);
			addTriangle(triangles, tri[2], ac, ab);
		}
		assignTriangle(triangles, i, tri[0], ab, ac);
	}
	else if (ab >= 0 && bc >= 0)
	{
		float dA = (points[bc] - points[tri[0]]).magnitudeSquared();
		float dC = (points[ab] - points[tri[2]]).magnitudeSquared();
		if (dC < dA)
		{
			addTriangle(triangles, tri[2], tri[0], ab);
			addTriangle(triangles, tri[2], ab, bc);
		}
		else
		{
			addTriangle(triangles, tri[2], tri[0], bc);
			addTriangle(triangles, tri[0], ab, bc);
		}
		assignTriangle(triangles, i, tri[1], bc, ab);
	}
	else if (bc >= 0 && ac >= 0)
	{
		float dA = (points[bc] - points[tri[0]]).magnitudeSquared();
		float dB = (points[ac] - points[tri[1]]).magnitudeSquared();
		if (dA < dB)
		{
			addTriangle(triangles, tri[0], tri[1], bc);
			addTriangle(triangles, tri[0], bc, ac);
		}
		else
		{
			addTriangle(triangles, tri[0], tri[1], ac);
			addTriangle(triangles, tri[1], bc, ac);
		}
		assignTriangle(triangles, i, tri[2], ac, bc);
	}
	else if (ab >= 0)
	{
		addTriangle(triangles, tri[1], tri[2], ab);
		assignTriangle(triangles, i, tri[2], tri[0], ab);
	}
	else if (bc >= 0)
	{
		addTriangle(triangles, tri[2], tri[0], bc);
		assignTriangle(triangles, i, tri[0], tri[1], bc);
	}
	else if (ac >= 0)
	{
		addTriangle(triangles, tri[1], tri[2], ac);
		assignTriangle(triangles, i, tri[0], tri[1], ac);
	}
}

static PX_FORCE_INLINE PxU64 key(PxU32 a, PxU32 b)
{
	if (a < b)
		return ((PxU64(a)) << 32) | (PxU64(b));
	else
		return ((PxU64(b)) << 32) | (PxU64(a));
}

static void checkEdge(PxU32 a, PxU32 b, PxHashMap<PxU64, PxI32>& edges, PxArray<PxVec3>& points, PxReal maxEdgeLength)
{
	if ((points[a] - points[b]).magnitudeSquared() < maxEdgeLength * maxEdgeLength)
		return;

	PxU64 k = key(a, b);
	if (edges.find(k))
		return;

	edges.insert(k, points.size());
	points.pushBack(0.5f * (points[a] + points[b]));
}

static PX_FORCE_INLINE PxI32 getEdge(PxU32 a, PxU32 b, PxHashMap<PxU64, PxI32>& edges)
{
	if (const PxPair<const PxU64, PxI32>* ptr = edges.find(key(a, b)))
		return ptr->second;
	return -1;
}

namespace
{
	struct Info
	{
		PxI32 StartIndex;
		PxI32 Count;

		Info(PxI32 startIndex, PxI32 count)
		{
			StartIndex = startIndex;
			Count = count;
		}
	};
}

static void checkEdge(PxU32 a, PxU32 b, PxHashMap<PxU64, Info>& edges, PxArray<PxVec3>& points, PxReal maxEdgeLength)
{
	if (a > b)
		PxSwap(a, b);

	PxReal l = (points[a] - points[b]).magnitudeSquared();
	if (l < maxEdgeLength * maxEdgeLength)
		return;

	l = PxSqrt(l);

	PxU32 numSubdivisions = (PxU32)(l / maxEdgeLength);
	if (numSubdivisions <= 1)
		return;

	PxU64 k = key(a, b);
	if (edges.find(k))
		return;

	edges.insert(k, Info(points.size(), numSubdivisions - 1));
	for (PxU32 i = 1; i < numSubdivisions; ++i)
	{
		PxReal p = (PxReal)i / numSubdivisions;
		points.pushBack((1 - p) * points[a] + p * points[b]);
	}
}
	
static PX_FORCE_INLINE Info getEdge(PxU32 a, PxU32 b, PxHashMap<PxU64, Info>& edges)
{
	const PxPair<const PxU64, Info>* value = edges.find(key(a, b));
	if (value)
		return value->second;
	return Info(-1, -1);
}

static PX_FORCE_INLINE void addPoints(PxArray<PxU32>& polygon, const Info& ab, bool reverse)
{
	if (reverse)
	{
		for (PxI32 i = ab.Count - 1; i >= 0; --i)
			polygon.pushBack(ab.StartIndex + i);
	}
	else
	{
		for (PxI32 i = 0; i < ab.Count; ++i)
			polygon.pushBack(ab.StartIndex + i);
	}
}

static PX_FORCE_INLINE PxReal angle(const PxVec3& l, const PxVec3& r)
{
	PxReal d = l.dot(r) / PxSqrt(l.magnitudeSquared() * r.magnitudeSquared());
	if (d <= -1) return PxPi;
	if (d >= 1) return 0.0f;
	return PxAcos(d);
}

static PX_FORCE_INLINE PxReal evaluateCost(const PxArray<PxVec3>& vertices, PxU32 a, PxU32 b, PxU32 c)
{
	const PxVec3& aa = vertices[a];
	const PxVec3& bb = vertices[b];
	const PxVec3& cc = vertices[c];

	PxReal a1 = angle(bb - aa, cc - aa);
	PxReal a2 = angle(aa - bb, cc - bb);
	PxReal a3 = angle(aa - cc, bb - cc);

	return PxMax(a1, PxMax(a2, a3));

	//return (aa - bb).magnitude() + (aa - cc).magnitude() + (bb - cc).magnitude();
}

static void triangulateConvex(PxU32 originalTriangleIndexTimes3, PxArray<PxU32>& polygon, const PxArray<PxVec3>& vertices, PxArray<PxU32>& triangles, PxArray<PxI32>& offsets)
{
	offsets.forceSize_Unsafe(0);
	offsets.reserve(polygon.size());
	for (PxU32 i = 0; i < polygon.size(); ++i)
		offsets.pushBack(1);

	PxI32 start = 0;
	PxU32 count = polygon.size();
	PxU32 triCounter = 0;
	while (count > 2)
	{
		PxReal minCost = FLT_MAX;
		PxI32 best = -1;
		PxI32 i = start;
		for (PxU32 j = 0; j < count; ++j)
		{
			PxU32 a = polygon[i];

			PX_ASSERT(offsets[i] >= 0);

			PxI32 n = (i + offsets[i]) % polygon.size();

			PX_ASSERT(offsets[n] >= 0);

			PxU32 b = polygon[n];
			PxU32 nn = (n + offsets[n]) % polygon.size();

			PX_ASSERT(offsets[nn] >= 0);

			PxU32 c = polygon[nn];

			PxReal cost = evaluateCost(vertices, a, b, c);
			if (cost < minCost)
			{
				minCost = cost;
				best = i;
			}
			i = n;
		}
		{
			PxU32 a = polygon[best];
			PxI32 n = (best + offsets[best]) % polygon.size();
			PxU32 b = polygon[n];
			PxU32 nn = (n + offsets[n]) % polygon.size();
			PxU32 c = polygon[nn];

			if (n == start)
				start += offsets[n];

			offsets[best] += offsets[n];
			offsets[n] = -1;

			PX_ASSERT(offsets[(best + offsets[best]) % polygon.size()] >= 0);

			if (triCounter == 0)
			{
				triangles[originalTriangleIndexTimes3 + 0] = a;
				triangles[originalTriangleIndexTimes3 + 1] = b;
				triangles[originalTriangleIndexTimes3 + 2] = c;
			}
			else
			{
				triangles.pushBack(a);
				triangles.pushBack(b);
				triangles.pushBack(c);
			}
			++triCounter;
		}

		--count;
	}
}

static bool limitMaxEdgeLengthAdaptive(PxArray<PxU32>& triangles, PxArray<PxVec3>& points, PxReal maxEdgeLength, PxArray<PxU32>* triangleMap = NULL)
{
	PxHashMap<PxU64, Info> edges;
	bool split = false;

	//Analyze edges
	for (PxU32 i = 0; i < triangles.size(); i += 3)
	{
		const PxU32* t = &triangles[i];
		checkEdge(t[0], t[1], edges, points, maxEdgeLength);
		checkEdge(t[1], t[2], edges, points, maxEdgeLength);
		checkEdge(t[0], t[2], edges, points, maxEdgeLength);
	}

	PxArray<PxI32> offsets;
	PxArray<PxU32> polygon;

	//Subdivide triangles if required
	PxU32 size = triangles.size();
	for (PxU32 i = 0; i < size; i += 3)
	{
		const PxU32* t = &triangles[i];
		Info ab = getEdge(t[0], t[1], edges);
		Info bc = getEdge(t[1], t[2], edges);
		Info ac = getEdge(t[0], t[2], edges);
		if (ab.StartIndex >= 0 || bc.StartIndex >= 0 || ac.StartIndex >= 0)
		{
			polygon.forceSize_Unsafe(0);
			polygon.pushBack(t[0]);
			addPoints(polygon, ab, t[0] > t[1]);
			polygon.pushBack(t[1]);
			addPoints(polygon, bc, t[1] > t[2]);
			polygon.pushBack(t[2]);
			addPoints(polygon, ac, t[2] > t[0]);

			PxU32 s = triangles.size();
			triangulateConvex(i, polygon, points, triangles, offsets);
			split = true;

			if (triangleMap != NULL)
			{
				for (PxU32 j = s; j < triangles.size(); ++j)
					triangleMap->pushBack((*triangleMap)[i]);
			}
		}
		/*else
		{
			result.pushBack(t[0]);
			result.pushBack(t[1]);
			result.pushBack(t[2]);
			if (triangleMap != NULL)
				triangleMap->pushBack((*triangleMap)[i]);
		}*/
	}
	return split;
}

bool PxRemeshingExt::reduceSliverTriangles(PxArray<PxU32>& triangles, PxArray<PxVec3>& points, PxReal maxEdgeLength, PxU32 maxIterations, PxArray<PxU32>* triangleMap, PxU32 triangleCountThreshold)
{
	bool split = limitMaxEdgeLengthAdaptive(triangles, points, maxEdgeLength, triangleMap);
	if (!split)
		return false;

	for (PxU32 i = 1; i < maxIterations; ++i)
	{
		split = limitMaxEdgeLengthAdaptive(triangles, points, maxEdgeLength, triangleMap);
			
		if (!split)
			break;

		if (triangles.size() >= triangleCountThreshold)
			break;
	}
		
	return true;
}

bool PxRemeshingExt::limitMaxEdgeLength(PxArray<PxU32>& triangles, PxArray<PxVec3>& points, PxReal maxEdgeLength, PxU32 maxIterations, PxArray<PxU32>* triangleMap, PxU32 triangleCountThreshold)
{
	if (triangleMap)
	{
		triangleMap->clear();
		triangleMap->reserve(triangles.size() / 3);
		for (PxU32 i = 0; i < triangles.size() / 3; ++i)
			triangleMap->pushBack(i);
	}

	PxU32 numIndices = triangles.size();
	PxHashMap<PxU64, PxI32> edges;
	bool success = true;
	for (PxU32 k = 0; k < maxIterations && success; ++k)
	{
		success = false;
		edges.clear();

		//Analyze edges
		for (PxU32 i = 0; i < triangles.size(); i += 3)
		{
			checkEdge(triangles[i], triangles[i + 1], edges, points, maxEdgeLength);
			checkEdge(triangles[i + 1], triangles[i + 2], edges, points, maxEdgeLength);
			checkEdge(triangles[i], triangles[i + 2], edges, points, maxEdgeLength);
		}

		//Subdivide triangles if required
		PxU32 size = triangles.size();
		for (PxU32 i = 0; i < size; i += 3)
		{
			PxI32 ab = getEdge(triangles[i], triangles[i + 1], edges);
			PxI32 bc = getEdge(triangles[i + 1], triangles[i + 2], edges);
			PxI32 ac = getEdge(triangles[i], triangles[i + 2], edges);
			if (ab >= 0 || bc >= 0 || ac >= 0)
			{
				PxU32 s = triangles.size();
				subdivideTriangle(i / 3, ab, bc, ac, triangles, points);
				success = true;
				if (triangleMap)
				{
					for (PxU32 j = s / 3; j < triangles.size() / 3; ++j)
						triangleMap->pushBack((*triangleMap)[i / 3]);
				}
			}
		}
		if (triangles.size() >= triangleCountThreshold)
			break;
	}
	return numIndices != triangles.size();
}
