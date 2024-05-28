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

#include "foundation/PxVec3.h"
#include "foundation/PxArray.h"
#include "GuMeshAnalysis.h"

using namespace physx;
using namespace Gu;

PX_FORCE_INLINE PxU64 key(PxI32 a, PxI32 b)
{
	if (a < b)
		return ((PxU64(a)) << 32) | (PxU64(b));
	else
		return ((PxU64(b)) << 32) | (PxU64(a));
}

#define INITIAL_VALUE -3

const static PxU32 neighborEdges[3][2] = { { 0, 1 }, { 2, 0 }, { 1, 2 } };
//const static PxU32 triTip[3] = { 2, 1, 0 };
bool MeshAnalyzer::buildTriangleAdjacency(const Triangle* tris, PxU32 numTriangles, PxArray<PxI32>& result, PxHashMap<PxU64, PxI32>& edges)
{
	PxU32 l = 4 * numTriangles; //Still factor 4 - waste one entry per triangle to get a power of 2 which allows for bit shift usage instead of modulo
	result.clear();
	result.resize(l, -1);

	for (PxU32 i = 3; i < l; i += 4)
		result[i] = INITIAL_VALUE; //Mark the fields that get never accessed because they are just not used, this is useful for debugging

	edges.clear();
	for (PxU32 i = 0; i < numTriangles; ++i)
	{
		const Triangle& tri = tris[i];
		if (tri[0] < 0)
			continue;

		for (PxU32 j = 0; j < 3; ++j)
		{
			PxU64 edge = key(tri[neighborEdges[j][0]], tri[neighborEdges[j][1]]);
			if (const PxPair<const PxU64, PxI32>* ptr = edges.find(edge))
			{
				if (ptr->second < 0)
					return false; //Edge shared by more than 2 triangles
				if (result[4 * i + j] == -4 || result[ptr->second] == -4)
				{
					result[4 * i + j] = -4; //Mark as non-manifold edge
					result[ptr->second] = -4;
				}
				else
				{
					if (result[4 * i + j] != -1 || result[ptr->second] != -1)
					{
						result[4 * i + j] = -4; //Mark as non-manifold edge
						result[ptr->second] = -4;
					}

					result[4 * i + j] = ptr->second;
					result[ptr->second] = 4 * i + j;
				}
				edges.erase(ptr->first);
				edges.insert(edge, -1); //Mark as processed
			}
			else
				edges.insert(edge, 4 * i + j);
		}
	}
	return true;
}

PxI32 indexOf(const Triangle& tri, PxI32 node)
{
	if (tri[0] == node) return 0;
	if (tri[1] == node) return 1;
	if (tri[2] == node) return 2;
	return 0xFFFFFFFF;
}

bool MeshAnalyzer::checkConsistentTriangleOrientation(const Triangle* tris, PxU32 numTriangles)
{
	PxArray<bool> flip;
	PxHashMap<PxU64, PxI32> edges;
	PxArray<PxArray<PxU32>> connectedTriangleGroups;
	if (!buildConsistentTriangleOrientationMap(tris, numTriangles, flip, edges, connectedTriangleGroups))
		return false;

	for (PxU32 i = 0; i < flip.size(); ++i)
	{
		if (flip[i])
			return false;
	}

	return true;
}

bool MeshAnalyzer::buildConsistentTriangleOrientationMap(const Triangle* tris, PxU32 numTriangles, PxArray<bool>& flip,
	PxHashMap<PxU64, PxI32>& edges, PxArray<PxArray<PxU32>>& connectedTriangleGroups)
{
	PxArray<PxI32> adj;
	if (!buildTriangleAdjacency(tris, numTriangles, adj, edges))
		return false;

	PxU32 l = numTriangles;
	PxArray<bool> done;
	done.resize(l, false);
	flip.clear();
	flip.resize(l, false);

	PxU32 seedIndex = 0;
	PxArray<PxI32> stack;

	while (true)
	{
		if (stack.size() == 0)
		{
			while (seedIndex < done.size() && done[seedIndex])
				++seedIndex;

			if (seedIndex == done.size())
				break;

			done[seedIndex] = true;
			flip[seedIndex] = false;
			stack.pushBack(seedIndex);
			PxArray<PxU32> currentGroup;
			currentGroup.pushBack(seedIndex);
			connectedTriangleGroups.pushBack(currentGroup);
		}

		PxI32 index = stack.popBack();
		bool f = flip[index];
		const Triangle& tri = tris[index];

		for (PxU32 i = 0; i < 3; ++i)
		{
			if (adj[4 * index + i] >= 0 && !done[adj[4 * index + i] >> 2])
			{
				PxI32 neighborTriIndex = adj[4 * index + i] >> 2;

				done[neighborTriIndex] = true;
				connectedTriangleGroups[connectedTriangleGroups.size() - 1].pushBack(neighborTriIndex);

				const Triangle& neighborTri = tris[neighborTriIndex];
				PxI32 j = indexOf(neighborTri, tri[neighborEdges[i][0]]);
				flip[neighborTriIndex] = (neighborTri[(j + 1) % 3] == tri[neighborEdges[i][1]]) != f;

				stack.pushBack(neighborTriIndex);
			}
		}
	}

	return true;
}

bool MeshAnalyzer::makeTriOrientationConsistent(Triangle* tris, PxU32 numTriangles, bool invertOrientation)
{
	PxHashMap<PxU64, PxI32> edges;
	PxArray<bool> flipTriangle;
	PxArray<PxArray<PxU32>> connectedTriangleGroups;
	if (!buildConsistentTriangleOrientationMap(tris, numTriangles, flipTriangle, edges, connectedTriangleGroups))
		return false;

	for (PxU32 i = 0; i < flipTriangle.size(); ++i)
	{
		Triangle& t = tris[i];
		if (flipTriangle[i] != invertOrientation)
			PxSwap(t[0], t[1]);
	}
	return true;
}

bool MeshAnalyzer::checkMeshWatertightness(const Triangle* tris, PxU32 numTriangles, bool treatInconsistentWindingAsNonWatertight)
{
	PxArray<bool> flip;
	PxHashMap<PxU64, PxI32> edges;
	PxArray<PxArray<PxU32>> connectedTriangleGroups;
	if (!MeshAnalyzer::buildConsistentTriangleOrientationMap(tris, numTriangles, flip, edges, connectedTriangleGroups))
		return false;

	if (treatInconsistentWindingAsNonWatertight) 
	{
		for (PxU32 i = 0; i < flip.size(); ++i)
		{
			if (flip[i])
				return false;
		}
	}

	for (PxHashMap<PxU64, PxI32>::Iterator iter = edges.getIterator(); !iter.done(); ++iter)
	{
		if (iter->second >= 0)		
			return false;		
	}
	return true;
}
