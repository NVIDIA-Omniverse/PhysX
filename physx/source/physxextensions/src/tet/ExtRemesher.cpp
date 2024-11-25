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

#include "ExtRemesher.h"
#include "ExtBVH.h"
#include "ExtMarchingCubesTable.h"
#include "foundation/PxFPU.h"
#include "foundation/PxSort.h"
#include "GuIntersectionTriangleBox.h"
#include "GuBox.h"

using namespace physx;
using namespace Ext;

// -------------------------------------------------------------------------------------
void Remesher::clear()
{
	cells.clear();
	vertices.clear();
	normals.clear();
	triIds.clear();
	triNeighbors.clear();
}

#define HASH_SIZE 170111

// -------------------------------------------------------------------------------------
PX_FORCE_INLINE static PxU32 hash(PxI32 xi, PxI32 yi, PxI32 zi)
{
	PxU32 h = (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481);
	return h % HASH_SIZE;
}

// -------------------------------------------------------------------------------------
void Remesher::addCell(PxI32 xi, PxI32 yi, PxI32 zi)
{
	Cell c;
	c.init(xi, yi, zi);
	PxU32 h = hash(xi, yi, zi);
	c.next = firstCell[h];
	firstCell[h] = PxI32(cells.size());
	cells.pushBack(c);
}

// -------------------------------------------------------------------------------------
PxI32 Remesher::getCellNr(PxI32 xi, PxI32 yi, PxI32 zi) const
{
	PxU32 h = hash(xi, yi, zi);
	PxI32 nr = firstCell[h];
	while (nr >= 0) 
	{
		const Cell& c = cells[nr];
		if (c.xi == xi && c.yi == yi && c.zi == zi)
			return nr;
		nr = c.next;
	}
	return -1;
}

// -------------------------------------------------------------------------------------
PX_FORCE_INLINE bool Remesher::cellExists(PxI32 xi, PxI32 yi, PxI32 zi) const
{
	return getCellNr(xi, yi, zi) >= 0;
}

// -------------------------------------------------------------------------------------
void Remesher::remesh(const PxArray<PxVec3>& inputVerts, const PxArray<PxU32>& inputTriIds,
	PxU32 resolution, PxArray<PxU32> *vertexMap)
{
	PX_SIMD_GUARD
	remesh(inputVerts.begin(), inputVerts.size(), inputTriIds.begin(), inputTriIds.size(), resolution, vertexMap);
}

void Remesher::remesh(const PxVec3* inputVerts, PxU32 nbVertices, const PxU32* inputTriIds, PxU32 nbTriangleIndices, PxU32 resolution, PxArray<PxU32> *vertexMap)
{
	clear();

	PxBounds3 meshBounds;
	meshBounds.setEmpty();

	for (PxU32 i = 0; i < nbVertices; i++)
		meshBounds.include(inputVerts[i]);

	PxVec3 dims = meshBounds.getDimensions();

	float spacing = PxMax(dims.x, PxMax(dims.y, dims.z)) / resolution;

	meshBounds.fattenFast(3.0f * spacing);

	PxU32 numTris = nbTriangleIndices / 3;
	PxBounds3 triBounds, cellBounds;
	Gu::BoxPadded box;
	box.rot = PxMat33(PxIdentity);

	firstCell.clear();
	firstCell.resize(HASH_SIZE, -1);

	// create sparse overlapping cells

	for (PxU32 i = 0; i < numTris; i++) 
	{
		const PxVec3& p0 = inputVerts[inputTriIds[3 * i]];
		const PxVec3& p1 = inputVerts[inputTriIds[3 * i + 1]];
		const PxVec3& p2 = inputVerts[inputTriIds[3 * i + 2]];

		triBounds.setEmpty();
		triBounds.include(p0);
		triBounds.include(p1);
		triBounds.include(p2);

		PxI32 x0 = PxI32(PxFloor((triBounds.minimum.x - meshBounds.minimum.x) / spacing));
		PxI32 y0 = PxI32(PxFloor((triBounds.minimum.y - meshBounds.minimum.y) / spacing));
		PxI32 z0 = PxI32(PxFloor((triBounds.minimum.z - meshBounds.minimum.z) / spacing));
				
		PxI32 x1 = PxI32(PxFloor((triBounds.maximum.x - meshBounds.minimum.x) / spacing)) + 1;
		PxI32 y1 = PxI32(PxFloor((triBounds.maximum.y - meshBounds.minimum.y) / spacing)) + 1;
		PxI32 z1 = PxI32(PxFloor((triBounds.maximum.z - meshBounds.minimum.z) / spacing)) + 1;
																				
		for (PxI32 xi = x0; xi <= x1; xi++) 
		{
			for (PxI32 yi = y0; yi <= y1; yi++) 
			{
				for (PxI32 zi = z0; zi <= z1; zi++) 
				{
					cellBounds.minimum.x = meshBounds.minimum.x + xi * spacing;
					cellBounds.minimum.y = meshBounds.minimum.y + yi * spacing;
					cellBounds.minimum.z = meshBounds.minimum.z + zi * spacing;
					cellBounds.maximum = cellBounds.minimum + PxVec3(spacing, spacing, spacing);
					cellBounds.fattenFast(1e-5f);

					box.center = cellBounds.getCenter();
					box.extents = cellBounds.getExtents();

					if (!Gu::intersectTriangleBox(box, p0, p1, p2))
						continue;

					if (!cellExists(xi, yi, zi))
						addCell(xi, yi, zi);
				}
			}
		}
	}

	// using marching cubes to create boundaries

	vertices.clear();
	cellOfVertex.clear();
	triIds.clear();

	PxI32 edgeVertId[12];
	PxVec3 cornerPos[8];
	int cornerVoxelNr[8];

	for (PxI32 i = 0; i < PxI32(cells.size()); i++)
	{
		Cell& c = cells[i];

		// we need to handle a 2 x 2 x 2 block of cells to cover the boundary

		for (PxI32 dx = 0; dx < 2; dx++)
		{
			for (PxI32 dy = 0; dy < 2; dy++)
			{
				for (PxI32 dz = 0; dz < 2; dz++) 
				{
					PxI32 xi = c.xi + dx;
					PxI32 yi = c.yi + dy;
					PxI32 zi = c.zi + dz;

					// are we responsible for this cell?

					PxI32 maxCellNr = i;

					for (PxI32 rx = xi - 1; rx <= xi; rx++) 
						for (PxI32 ry = yi - 1; ry <= yi; ry++) 
							for (PxI32 rz = zi - 1; rz <= zi; rz++)
								maxCellNr = PxMax(maxCellNr, getCellNr(rx, ry, rz));

					if (maxCellNr != i)
						continue;

					PxI32 code = 0;
					for (PxI32 j = 0; j < 8; j++) 
					{
						PxI32 mx = xi - 1 + marchingCubeCorners[j][0];
						PxI32 my = yi - 1 + marchingCubeCorners[j][1];
						PxI32 mz = zi - 1 + marchingCubeCorners[j][2];
						cornerVoxelNr[j] = getCellNr(mx, my, mz);

						if (cornerVoxelNr[j] >= 0)
							code |= (1 << j);

						cornerPos[j].x = meshBounds.minimum.x + (mx + 0.5f) * spacing;
						cornerPos[j].y = meshBounds.minimum.y + (my + 0.5f) * spacing;
						cornerPos[j].z = meshBounds.minimum.z + (mz + 0.5f) * spacing;
					}
					PxI32 first = firstMarchingCubesId[code];
					PxI32 num = (firstMarchingCubesId[code + 1] - first);

					// create vertices and tris

					for (PxI32 j = 0; j < 12; j++)
						edgeVertId[j] = -1;

					for (PxI32 j = num - 1; j >= 0; j--) 
					{
						PxI32 edgeId = marchingCubesIds[first + j];
						if (edgeVertId[edgeId] < 0) 
						{
							PxI32 id0 = marchingCubeEdges[edgeId][0];
							PxI32 id1 = marchingCubeEdges[edgeId][1];
							PxVec3& p0 = cornerPos[id0];
							PxVec3& p1 = cornerPos[id1];
							edgeVertId[edgeId] = vertices.size();
							vertices.pushBack((p0 + p1) * 0.5f);
							cellOfVertex.pushBack(PxMax(cornerVoxelNr[id0], cornerVoxelNr[id1]));
						}
						triIds.pushBack(edgeVertId[edgeId]);
					}
				}
			}
		}
	}

	removeDuplicateVertices();
	pruneInternalSurfaces();

	project(inputVerts, inputTriIds, nbTriangleIndices, 2.0f * spacing, 0.1f * spacing);

	if (vertexMap)
		createVertexMap(inputVerts, nbVertices, meshBounds.minimum, spacing, *vertexMap);

	computeNormals();
}

// -------------------------------------------------------------------------------------
void Remesher::removeDuplicateVertices()
{
	PxF32 eps = 1e-5f;

	struct Ref 
	{
		PxF32 d;
		PxI32 id;
		bool operator < (const Ref& r) const 
		{
			return d < r.d;
		}
	};

	PxI32 numVerts = PxI32(vertices.size());

	PxArray<Ref> refs(numVerts);
	for (PxI32 i = 0; i < numVerts; i++) 
	{
		PxVec3& p = vertices[i];
		refs[i].d = p.x + 0.3f * p.y + 0.1f * p.z;
		refs[i].id = i;
	}

	PxSort(refs.begin(), refs.size());

	PxArray<PxI32> idMap(vertices.size(), -1);
	PxArray<PxVec3> oldVerts = vertices;
	PxArray<PxI32> oldCellOfVertex = cellOfVertex;
	vertices.clear();
	cellOfVertex.clear();

	PxI32 nr = 0;
	while (nr < numVerts) 
	{
		Ref& r = refs[nr];
		nr++;
		if (idMap[r.id] >= 0)
			continue;
		idMap[r.id] = vertices.size();
		vertices.pushBack(oldVerts[r.id]);
		cellOfVertex.pushBack(oldCellOfVertex[r.id]);

		PxI32 i = nr;
		while (i < numVerts && fabsf(refs[i].d - r.d) < eps) 
		{
			PxI32 id = refs[i].id;
			if ((oldVerts[r.id] - oldVerts[id]).magnitudeSquared() < eps * eps)
				idMap[id] = idMap[r.id];
			i++;
		}
	}

	for (PxI32 i = 0; i < PxI32(triIds.size()); i++)
		triIds[i] = idMap[triIds[i]];
}

// -------------------------------------------------------------------------------------
void Remesher::findTriNeighbors()
{
	PxI32 numTris = PxI32(triIds.size()) / 3;
	triNeighbors.clear();
	triNeighbors.resize(3 * numTris, -1);

	struct Edge 
	{
		void init(PxI32 _id0, PxI32 _id1, PxI32 _triNr, PxI32 _edgeNr) 
		{
			this->id0 = PxMin(_id0, _id1);
			this->id1 = PxMax(_id0, _id1);
			this->triNr = _triNr; 
			this->edgeNr = _edgeNr;
		}
		bool operator < (const Edge& e) const 
		{
			if (id0 < e.id0) return true;
			if (id0 > e.id0) return false;
			return id1 < e.id1;
		}
		bool operator == (const Edge& e) const 
		{
			return id0 == e.id0 && id1 == e.id1;
		}
		PxI32 id0, id1, triNr, edgeNr;
	};

	PxArray<Edge> edges(triIds.size());

	for (PxI32 i = 0; i < numTris; i++) 
	{
		for (PxI32 j = 0; j < 3; j++) {
			PxI32 id0 = triIds[3 * i + j];
			PxI32 id1 = triIds[3 * i + (j + 1) % 3];
			edges[3 * i + j].init(id0, id1, i, j);
		}
	}
	PxSort(edges.begin(), edges.size());

	PxI32 nr = 0;
	while (nr < PxI32(edges.size())) 
	{
		Edge& e0 = edges[nr];
		nr++;
		while (nr < PxI32(edges.size()) && edges[nr] == e0) {
			Edge& e1 = edges[nr];
			triNeighbors[3 * e0.triNr + e0.edgeNr] = e1.triNr;
			triNeighbors[3 * e1.triNr + e1.edgeNr] = e0.triNr;
			nr++;
		}
	}
}

// -------------------------------------------------------------------------------------
void Remesher::pruneInternalSurfaces()
{
	// flood islands, if the enclosed volume is negative remove it

	findTriNeighbors();

	PxI32 numTris = PxI32(triIds.size()) / 3;

	PxArray<PxI32> oldTriIds = triIds;
	triIds.clear();

	PxArray<bool> visited(numTris, false);
	PxArray<PxI32> stack;

	for (PxI32 i = 0; i < numTris; i++) 
	{
		if (visited[i])
			continue;
		stack.clear();
		stack.pushBack(i);
		PxI32 islandStart = PxI32(triIds.size());

		float vol = 0.0f;

		while (!stack.empty()) 
		{
			PxI32 triNr = stack.back();
			stack.popBack();
			if (visited[triNr])
				continue;
			visited[triNr] = true;
			for (PxI32 j = 0; j < 3; j++)
				triIds.pushBack(oldTriIds[3 * triNr + j]);

			const PxVec3& p0 = vertices[oldTriIds[3 * triNr]];
			const PxVec3& p1 = vertices[oldTriIds[3 * triNr + 1]];
			const PxVec3& p2 = vertices[oldTriIds[3 * triNr + 2]];
			vol += p0.cross(p1).dot(p2);

			for (PxI32 j = 0; j < 3; j++) 
			{
				PxI32 n = triNeighbors[3 * triNr + j];
				if (n >= 0 && !visited[n])
					stack.pushBack(n);
			}
		}

		if (vol <= 0.0f)
			triIds.resize(islandStart);
	}

	// remove unreferenced vertices

	PxArray<PxI32> idMap(vertices.size(), -1);
	PxArray<PxVec3> oldVerts = vertices;
	PxArray<PxI32> oldCellOfVertex = cellOfVertex;
	vertices.clear();
	cellOfVertex.clear();

	for (int i = 0; i < PxI32(triIds.size()); i++) 
	{
		PxI32 id = triIds[i];
		if (idMap[id] < 0) 
		{
			idMap[id] = vertices.size();
			vertices.pushBack(oldVerts[id]);
			cellOfVertex.pushBack(oldCellOfVertex[id]);
		}
		triIds[i] = idMap[id];
	}
}

// -----------------------------------------------------------------------------------
// PT: TODO: refactor with other implementation
static void getClosestPointOnTriangle(const PxVec3& pos, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2,
	PxVec3& closest, PxVec3& bary)
{
	PxVec3 e0 = p1 - p0;
	PxVec3 e1 = p2 - p0;
	PxVec3 tmp = p0 - pos;

	float a = e0.dot(e0);
	float b = e0.dot(e1);
	float c = e1.dot(e1);
	float d = e0.dot(tmp);
	float e = e1.dot(tmp);
	PxVec3 coords, clampedCoords;
	coords.x = b * e - c * d;    // s * det
	coords.y = b * d - a * e;    // t * det
	coords.z = a * c - b * b;    // det

	clampedCoords = PxVec3(PxZero);
	if (coords.x <= 0.0f) 
	{
		if (c != 0.0f)
			clampedCoords.y = -e / c;
	}
	else if (coords.y <= 0.0f) 
	{
		if (a != 0.0f)
			clampedCoords.x = -d / a;
	}
	else if (coords.x + coords.y > coords.z) 
	{
		float denominator = a + c - b - b;
		float numerator = c + e - b - d;
		if (denominator != 0.0f) 
		{
			clampedCoords.x = numerator / denominator;
			clampedCoords.y = 1.0f - clampedCoords.x;
		}
	}
	else 
	{    // all inside
		if (coords.z != 0.0f) {
			clampedCoords.x = coords.x / coords.z;
			clampedCoords.y = coords.y / coords.z;
		}
	}
	bary.y = PxMin(PxMax(clampedCoords.x, 0.0f), 1.0f);
	bary.z = PxMin(PxMax(clampedCoords.y, 0.0f), 1.0f);
	bary.x = 1.0f - bary.y - bary.z;
	closest = p0 * bary.x + p1 * bary.y + p2 * bary.z;
}

// -------------------------------------------------------------------------------------
void Remesher::project(const PxVec3* inputVerts, const PxU32* inputTriIds, PxU32 nbTriangleIndices,
	float searchDist, float surfaceDist)
{
	// build a bvh for the input mesh

	PxI32 numInputTris = PxI32(nbTriangleIndices) / 3;

	if (numInputTris == 0)
		return;

	bvhBounds.resize(numInputTris);
	bvhTris.clear();

	for (PxI32 i = 0; i < numInputTris; i++) {
		PxBounds3& b = bvhBounds[i];
		b.setEmpty();
		b.include(inputVerts[inputTriIds[3 * i]]);
		b.include(inputVerts[inputTriIds[3 * i + 1]]);
		b.include(inputVerts[inputTriIds[3 * i + 2]]);
	}

	BVHDesc bvh;
	BVHBuilder::build(bvh, &bvhBounds[0], bvhBounds.size());

	// project vertices to closest point on surface

	PxBounds3 pb;
	for (PxU32 i = 0; i < vertices.size(); i++)
	{
		PxVec3& p = vertices[i];
		pb.setEmpty();
		pb.include(p);
		pb.fattenFast(searchDist);
		bvh.query(pb, bvhTris);

		float minDist2 = PX_MAX_F32;
		PxVec3 closest(PxZero);

		for (PxU32 j = 0; j < bvhTris.size(); j++) 
		{
			PxI32 triNr = bvhTris[j];
			const PxVec3& p0 = inputVerts[inputTriIds[3 * triNr]];
			const PxVec3& p1 = inputVerts[inputTriIds[3 * triNr + 1]];
			const PxVec3& p2 = inputVerts[inputTriIds[3 * triNr + 2]];
			PxVec3 c, bary;
			getClosestPointOnTriangle(p, p0, p1, p2, c, bary);
			float dist2 = (c - p).magnitudeSquared();
			if (dist2 < minDist2) {
				minDist2 = dist2;
				closest = c;
			}
		}

		if (minDist2 < PX_MAX_F32) {
			PxVec3 n = p - closest;
			n.normalize();
			p = closest + n * surfaceDist;
		}
	}

}

static const int cellNeighbors[6][3] = { { -1,0,0 }, {1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1} };

// -------------------------------------------------------------------------------------
void Remesher::createVertexMap(const PxVec3* inputVerts, PxU32 nbVertices, const PxVec3 &gridOrigin, PxF32 &gridSpacing,
	PxArray<PxU32> &vertexMap)
{
	PxArray<PxI32> vertexOfCell(cells.size(), -1);
	PxArray<PxI32 > front[2];
	PxI32 frontNr = 0;

	// compute inverse links

	for (PxI32 i = 0; i < PxI32(vertices.size()); i++) 
	{
		PxI32 cellNr = cellOfVertex[i];
		if (cellNr >= 0) 
		{
			if (vertexOfCell[cellNr] < 0) {
				vertexOfCell[cellNr] = i;
				front[frontNr].pushBack(cellNr);
			}
		}
	}

	// propagate cell->vertex links through the voxel mesh

	while (!front[frontNr].empty()) 
	{
		front[1 - frontNr].clear();

		for (PxI32 i = 0; i < PxI32(front[frontNr].size()); i++) 
		{
			int cellNr = front[frontNr][i];
			Cell& c = cells[cellNr];
			for (PxI32 j = 0; j < 6; j++) 
			{
				PxI32 n = getCellNr(c.xi + cellNeighbors[j][0],
					c.yi + cellNeighbors[j][1],
					c.zi + cellNeighbors[j][2]);
				if (n >= 0 && vertexOfCell[n] < 0) {
					vertexOfCell[n] = vertexOfCell[cellNr];
					front[1 - frontNr].pushBack(n);
				}
			}
		}
		frontNr = 1 - frontNr;
	}

	// create the map

	vertexMap.clear();
	vertexMap.resize(nbVertices, 0);

	for (PxU32 i = 0; i < nbVertices; i++)
	{
		const PxVec3& p = inputVerts[i];
		PxI32 xi = PxI32(PxFloor((p.x - gridOrigin.x) / gridSpacing));
		PxI32 yi = PxI32(PxFloor((p.y - gridOrigin.y) / gridSpacing));
		PxI32 zi = PxI32(PxFloor((p.z - gridOrigin.z) / gridSpacing));

		PxI32 cellNr = getCellNr(xi, yi, zi);
		vertexMap[i] = cellNr >= 0 ? vertexOfCell[cellNr] : 0;
	}
}

// -------------------------------------------------------------------------------------
void Remesher::computeNormals()
{
	normals.clear();
	normals.resize(vertices.size(), PxVec3(PxZero));

	for (PxI32 i = 0; i < PxI32(triIds.size()); i += 3) 
	{
		PxI32* ids = &triIds[i];

		PxVec3& p0 = vertices[ids[0]];
		PxVec3& p1 = vertices[ids[1]];
		PxVec3& p2 = vertices[ids[2]];
		PxVec3 n = (p1 - p0).cross(p2 - p0);
		normals[ids[0]] += n;
		normals[ids[1]] += n;
		normals[ids[2]] += n;
	}

	for (PxI32 i = 0; i < PxI32(normals.size()); i++)
		normals[i].normalize();
}

// -------------------------------------------------------------------------------------
void Remesher::readBack(PxArray<PxVec3>& outputVertices, PxArray<PxU32>& outputTriIds)
{
	outputVertices = vertices;
	outputTriIds.resize(triIds.size());
	for (PxU32 i = 0; i < triIds.size(); i++)
		outputTriIds[i] = PxU32(triIds[i]);
}
