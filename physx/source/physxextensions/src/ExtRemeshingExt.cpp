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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "extensions/PxRemeshingExt.h"
#include "foundation/PxHashMap.h"

namespace physx
{
	void assignTriangle(PxArray<PxU32>& triangles, PxU32 triIndex, PxU32 a, PxU32 b, PxU32 c)
	{
		triangles[3 * triIndex] = a;
		triangles[3 * triIndex + 1] = b;
		triangles[3 * triIndex + 2] = c;
	}

	void addTriangle(PxArray<PxU32>& triangles, PxU32 a, PxU32 b, PxU32 c)
	{
		triangles.pushBack(a);
		triangles.pushBack(b);
		triangles.pushBack(c);
	}

	void subdivideTriangle(int i, int ab, int bc, int ac, PxArray<PxU32>& triangles, PxArray<PxVec3>& points)
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

	PX_FORCE_INLINE PxU64 key(PxU32 a, PxU32 b)
	{
		if (a < b)
			return ((PxU64(a)) << 32) | (PxU64(b));
		else
			return ((PxU64(b)) << 32) | (PxU64(a));
	}

	void checkEdge(PxU32 a, PxU32 b, PxHashMap<PxU64, PxI32>& edges, PxArray<PxVec3>& points, PxReal maxEdgeLength)
	{
		if ((points[a] - points[b]).magnitudeSquared() < maxEdgeLength * maxEdgeLength)
			return;

		PxU64 k = key(a, b);
		if (edges.find(k))
			return;

		edges.insert(k, points.size());
		points.pushBack(0.5f * (points[a] + points[b]));
	}

	PX_FORCE_INLINE PxI32 getEdge(PxU32 a, PxU32 b, PxHashMap<PxU64, PxI32>& edges)
	{
		if (const PxPair<const PxU64, PxI32>* ptr = edges.find(key(a, b)))
			return ptr->second;
		return -1;
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
}
