// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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

// PxVec3T has no float -> double converting constructor.
static PX_FORCE_INLINE PxVec3d toVec3d(const PxVec3& v)
{
	return PxVec3d(PxF64(v.x), PxF64(v.y), PxF64(v.z));
}

bool MeshAnalyzer::orientTrianglesOutward(Triangle* tris, PxU32 numTriangles, const PxVec3* vertices, const PxArray<bool>& consistencyFlipMap)
{
	if (numTriangles == 0)
		return false;

	bool changed = false;

	// Make the winding consistent across the mesh.
	for (PxU32 i = 0; i < consistencyFlipMap.size(); ++i)
	{
		if (consistencyFlipMap[i])
		{
			PxSwap(tris[i][0], tris[i][1]);
			changed = true;
		}
	}

	// Orient outward: flip the whole mesh if it encloses a negative signed volume. The volume is accumulated in
	// double precision and relative to a reference vertex, so the magnitude of the individual terms is governed
	// by the extent of the mesh rather than by its distance from the origin. For a closed mesh the reference
	// point cancels out and does not change the result. The factor of 6 is dropped since only the sign matters.
	const PxVec3d ref = toVec3d(vertices[tris[0][0]]);
	PxF64 signedVolumeX6 = 0.0;
	for (PxU32 i = 0; i < numTriangles; ++i)
	{
		const PxVec3d a = toVec3d(vertices[tris[i][0]]) - ref;
		const PxVec3d b = toVec3d(vertices[tris[i][1]]) - ref;
		const PxVec3d c = toVec3d(vertices[tris[i][2]]) - ref;
		signedVolumeX6 += a.dot(b.cross(c));
	}
	if (signedVolumeX6 < 0.0)
	{
		for (PxU32 i = 0; i < numTriangles; ++i)
			PxSwap(tris[i][1], tris[i][2]);
		changed = true;
	}

	return changed;
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
