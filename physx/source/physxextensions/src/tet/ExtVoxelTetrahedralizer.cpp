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

#include "ExtVoxelTetrahedralizer.h"
#include "CmRandom.h"
#include "ExtTetUnionFind.h"

using namespace physx;
using namespace Ext;

// -------------------------------------------------------------------------------------
static PxI32 cubeNeighbors[6][3] = { { -1,0,0 }, {1,0,0}, {0,-1,0}, {0,1,0}, {0,0,-1}, {0,0,1} };
static const PxI32 cubeCorners[8][3] = { {0,0,0}, {1,0,0},{1,1,0},{0,1,0}, {0,0,1}, {1,0,1},{1,1,1},{0,1,1} };
static const PxI32 cubeFaces[6][4] = { {0,3,7,4},{1,2,6,5},{0,1,5,4},{3,2,6,7},{0,1,2,3},{4,5,6,7} };
static const PxI32 oppNeighbor[6] = { 1,0,3,2,5,4 };

static const PxI32 tetEdges[12][2] = { {0,1},{1,2},{2,0},{0,3},{1,3},{2,3},  {1,0},{2,1},{0,2},{3,0},{3,1},{3,2} };

static PxI32 cubeSixTets[6][4] = {
	{ 0, 4, 5, 7 },{ 1, 5, 6, 7 },{ 1, 0, 5, 7 },{ 1, 2, 3, 6 },{ 3, 1, 6, 7 },{ 0, 1, 3, 7 } };

static PxI32 cubeFiveTets[2][5][4] = {
	{ { 0, 1, 2, 5 },{ 0, 2, 3, 7 },{ 0, 5, 2, 7 },{ 0, 5, 7, 4 },{ 2, 7, 5, 6 } },
	{ { 1, 2, 3, 6 },{ 1, 3, 0, 4 },{ 1, 6, 3, 4 },{ 1, 6, 4, 5 },{ 3, 4, 6, 7 } },
};

static PxI32 cubeSixSubdivTets[12][4] = {
	{0,4,5,8}, {0,5,1,8}, {3,2,6,8}, {3,6,7,8},
	{0,3,7,8}, {0,7,4,8}, {1,5,6,8}, {1,6,2,8},
	{0,1,3,8}, {1,2,3,8}, {5,4,7,8}, {5,7,6,8}
};

static PxI32 cubeFiveSubdivTets[2][12][4] = {
	{
		{0,1,2,8}, {0,2,3,8}, {4,7,5,8}, {5,7,6,8},
		{0,7,4,8}, {0,3,7,8}, {1,5,2,8}, {2,5,6,8},
		{0,5,1,8}, {0,4,5,8}, {3,2,7,8}, {2,6,7,8}
	},
	{
		{0,1,3,8}, {1,2,3,8}, {4,7,6,8}, {4,6,5,8},
		{0,3,4,8}, {3,7,4,8}, {1,5,6,8}, {1,6,2,8},
		{0,4,1,8}, {1,4,5,8}, {3,2,6,8}, {3,6,7,8}
	}
};

static const PxI32 volIdOrder[4][3] = { {1, 3, 2}, {0, 2, 3}, {0, 3, 1}, {0, 1, 2} };

// -------------------------------------------------------------------------------------
static bool boxTriangleIntersection(
	PxVec3 p0, PxVec3 p1, PxVec3 p2, PxVec3 center, PxVec3 extents);
static void getClosestPointOnTriangle(
	PxVec3 p1, PxVec3 p2, PxVec3 p3, PxVec3 p, PxVec3& closest, PxVec3& bary);

// -------------------------------------------------------------------------------------
VoxelTetrahedralizer::VoxelTetrahedralizer()
{
	clear();
}

// -------------------------------------------------------------------------------------
void VoxelTetrahedralizer::clear()
{
	surfaceVerts.clear();
	surfaceTriIds.clear();
	surfaceBounds.setEmpty();

	tetVerts.clear();
	origTetVerts.clear();
	isSurfaceVert.clear();
	targetVertPos.clear();

	tetIds.clear();
	voxels.clear();
	gridOrigin = PxVec3(PxZero);
	gridSpacing = 0.0f;
}

// -----------------------------------------------------------------------------------
void VoxelTetrahedralizer::readBack(PxArray<PxVec3>& _tetVertices, PxArray<PxU32>& _tetIndices)
{
	_tetVertices = tetVerts;
	_tetIndices.resize(tetIds.size());

	for (PxU32 i = 0; i < tetIds.size(); i++)
		_tetIndices[i] = PxU32(tetIds[i]);
}

// -----------------------------------------------------------------------------------
void VoxelTetrahedralizer::createTetMesh(const PxArray<PxVec3>& verts, const PxArray<PxU32>& triIds,
	PxI32 resolution, PxI32 numRelaxationIters, PxF32 relMinTetVolume)
{
	surfaceVerts = verts;
	surfaceTriIds.resize(triIds.size());
	for (PxU32 i = 0; i < triIds.size(); i++)
		surfaceTriIds[i] = triIds[i];

	surfaceBounds.setEmpty();

	for (PxU32 i = 0; i < surfaceVerts.size(); i++)
		surfaceBounds.include(surfaceVerts[i]);

	buildBVH();

	voxelize(resolution);

	bool subdivBorder = true;
	int numTetsPerVoxel = 5;		// or 6

	createTets(subdivBorder, numTetsPerVoxel);

	findTargetPositions(0.2f * gridSpacing);

	relax(numRelaxationIters, relMinTetVolume);
}

// -----------------------------------------------------------------------------------
void VoxelTetrahedralizer::buildBVH()
{
	PxI32 numTris = PxI32(surfaceTriIds.size()) / 3;

	if (numTris == 0)
		return;

	PxArray<PxBounds3> bvhBounds(numTris);

	for (PxI32 i = 0; i < numTris; i++) {
		PxBounds3& b = bvhBounds[i];
		b.setEmpty();
		b.include(surfaceVerts[surfaceTriIds[3 * i]]);
		b.include(surfaceVerts[surfaceTriIds[3 * i + 1]]);
		b.include(surfaceVerts[surfaceTriIds[3 * i + 2]]);
	}

	BVHBuilder::build(bvh, &bvhBounds[0], bvhBounds.size());
}

// -----------------------------------------------------------------------------------
void VoxelTetrahedralizer::voxelize(PxU32 resolution)
{
	tetIds.clear();
	tetVerts.clear();

	PxBounds3 meshBounds;
	meshBounds.setEmpty();

	for (PxU32 i = 0; i < surfaceVerts.size(); i++)
		meshBounds.include(surfaceVerts[i]);

	gridSpacing = meshBounds.getDimensions().magnitude() / resolution;
	meshBounds.fattenSafe(gridSpacing);
	gridOrigin = meshBounds.minimum;

	voxels.clear();

	PxI32 numX = PxI32((meshBounds.maximum.x - meshBounds.minimum.x) / gridSpacing) + 1;
	PxI32 numY = PxI32((meshBounds.maximum.y - meshBounds.minimum.y) / gridSpacing) + 1;
	PxI32 numZ = PxI32((meshBounds.maximum.z - meshBounds.minimum.z) / gridSpacing) + 1;
	PxI32 numCells = numX * numY * numZ;

	PxArray<PxI32> voxelOfCell(numCells, -1);
	PxBounds3 voxelBounds, faceBounds;

	// create intersected voxels

	for (PxI32 i = 0; i < numCells; i++) {
		PxI32 zi = i % numZ;
		PxI32 yi = (i / numZ) % numY;
		PxI32 xi = (i / numZ / numY);

		voxelBounds.minimum = meshBounds.minimum + PxVec3(PxF32(xi), PxF32(yi), PxF32(zi)) * gridSpacing;
		voxelBounds.maximum = voxelBounds.minimum + PxVec3(gridSpacing);

		bvh.query(voxelBounds, queryTris);

		for (PxU32 j = 0; j < queryTris.size(); j++) {
			PxI32 triNr = queryTris[j];

			const PxVec3& p0 = surfaceVerts[surfaceTriIds[3 * triNr]];
			const PxVec3& p1 = surfaceVerts[surfaceTriIds[3 * triNr + 1]];
			const PxVec3& p2 = surfaceVerts[surfaceTriIds[3 * triNr + 2]];

			if (boxTriangleIntersection(p0, p1, p2, voxelBounds.getCenter(), voxelBounds.getExtents())) {
				// volume
				if (voxelOfCell[i] < 0) {
					voxelOfCell[i] = voxels.size();
					voxels.resize(voxels.size() + 1);
					voxels.back().init(xi, yi, zi);
				}
			}
		}
	}

	// flood outside

	PxArray<PxI32> stack;
	stack.pushBack(0);

	while (!stack.empty()) {
		PxI32 nr = stack.back();
		stack.popBack();

		if (voxelOfCell[nr] == -1) {
			voxelOfCell[nr] = -2;		// outside

			PxI32 z0 = nr % numZ;
			PxI32 y0 = (nr / numZ) % numY;
			PxI32 x0 = (nr / numZ / numY);

			for (PxI32 i = 0; i < 6; i++) {
				PxI32 xi = x0 + cubeNeighbors[i][0];
				PxI32 yi = y0 + cubeNeighbors[i][1];
				PxI32 zi = z0 + cubeNeighbors[i][2];
				if (xi >= 0 && xi < numX && yi >= 0 && yi < numY && zi >= 0 && zi < numZ) {
					PxI32 adj = (xi * numY + yi) * numZ + zi;
					if (voxelOfCell[adj] == -1)
						stack.pushBack(adj);
				}
			}
		}
	}

	// create voxels for the inside

	for (PxI32 i = 0; i < numCells; i++) {
		if (voxelOfCell[i] == -1) {
			voxelOfCell[i] = voxels.size();
			voxels.resize(voxels.size() + 1);
			PxI32 zi = i % numZ;
			PxI32 yi = (i / numZ) % numY;
			PxI32 xi = (i / numZ / numY);
			voxels.back().init(xi, yi, zi);
			voxels.back().inner = true;
		}
	}

	// find neighbors

	for (PxU32 i = 0; i < voxels.size(); i++) {
		Voxel& v = voxels[i];

		voxelBounds.minimum = meshBounds.minimum + PxVec3(PxF32(v.xi), PxF32(v.yi), PxF32(v.zi)) * gridSpacing;
		voxelBounds.maximum = voxelBounds.minimum + PxVec3(gridSpacing);

		for (PxI32 j = 0; j < 6; j++) {

			PxI32 xi = v.xi + cubeNeighbors[j][0];
			PxI32 yi = v.yi + cubeNeighbors[j][1];
			PxI32 zi = v.zi + cubeNeighbors[j][2];

			if (xi < 0 || xi >= numX || yi < 0 || yi >= numY || zi < 0 || zi >= numZ)
				continue;

			PxI32 neighbor = voxelOfCell[(xi * numY + yi) * numZ + zi];
			if (neighbor < 0)
				continue;

			if (v.inner || voxels[neighbor].inner) {
				v.neighbors[j] = neighbor;
				continue;
			}

			faceBounds = voxelBounds;
			PxF32 eps = 1e-4f;
			switch (j) {
				case 0: faceBounds.maximum.x = faceBounds.minimum.x + eps; break;
				case 1: faceBounds.minimum.x = faceBounds.maximum.x - eps; break;
				case 2: faceBounds.maximum.y = faceBounds.minimum.y + eps; break;
				case 3: faceBounds.minimum.y = faceBounds.maximum.y - eps; break;
				case 4: faceBounds.maximum.z = faceBounds.minimum.z + eps; break;
				case 5: faceBounds.minimum.z = faceBounds.maximum.z - eps; break;
			}
			bvh.query(faceBounds, queryTris);

			bool intersected = false;

			for (PxU32 k = 0; k < queryTris.size(); k++) {
				PxI32 triNr = queryTris[k];

				const PxVec3& p0 = surfaceVerts[surfaceTriIds[3 * triNr]];
				const PxVec3& p1 = surfaceVerts[surfaceTriIds[3 * triNr + 1]];
				const PxVec3& p2 = surfaceVerts[surfaceTriIds[3 * triNr + 2]];

				if (boxTriangleIntersection(p0, p1, p2, faceBounds.getCenter(), faceBounds.getExtents())) {
					intersected = true;
					break;
				}
			}

			if (intersected)
				v.neighbors[j] = neighbor;
		}
	}
}

// -----------------------------------------------------------------------------------
void VoxelTetrahedralizer::createUniqueTetVertices()
{
	// start with each voxel having its own vertices

	PxArray<PxVec3> verts;
	for (PxU32 i = 0; i < voxels.size(); i++) {
		Voxel& v = voxels[i];

		for (PxI32 j = 0; j < 8; j++) {
			v.ids[j] = verts.size();
			verts.pushBack(gridOrigin + PxVec3(
				PxF32(v.xi + cubeCorners[j][0]),
				PxF32(v.yi + cubeCorners[j][1]),
				PxF32(v.zi + cubeCorners[j][2])) * gridSpacing);
		}
	}

	// unify vertices

	UnionFind* u = new UnionFind();
	u->init(verts.size());

	for (PxU32 i = 0; i < voxels.size(); i++) {
		Voxel& v0 = voxels[i];
		for (PxI32 j = 0; j < 6; j++) {
			PxI32 n = v0.neighbors[j];
			if (n < 0)
				continue;
			Voxel& v1 = voxels[n];

			for (PxI32 k = 0; k < 4; k++) {
				PxI32 id0 = v0.ids[cubeFaces[j][k]];
				PxI32 id1 = v1.ids[cubeFaces[oppNeighbor[j]][k]];
				u->makeSet(id0, id1);
			}
		}
	}

	u->computeSetNrs();

	tetVerts.clear();

	for (PxU32 i = 0; i < voxels.size(); i++) {
		Voxel& v = voxels[i];

		for (PxI32 j = 0; j < 8; j++) {
			PxI32 setNr = u->getSetNr(v.ids[j]);
			if (PxI32(tetVerts.size()) <= setNr)
				tetVerts.resize(setNr + 1, PxVec3(PxZero));
			tetVerts[setNr] = verts[v.ids[j]];
			v.ids[j] = setNr;
		}
	}

	origTetVerts = tetVerts;

	delete u;
}

// -------------------------------------------------------------------------------------
void VoxelTetrahedralizer::findTargetPositions(PxF32 surfaceDist)
{
	targetVertPos = tetVerts;

	for (PxU32 i = 0; i < voxels.size(); i++) {

		Voxel& v = voxels[i];

		PxBounds3 voxelBounds;
		voxelBounds.minimum = gridOrigin + PxVec3(PxF32(v.xi), PxF32(v.yi), PxF32(v.zi)) * gridSpacing;
		voxelBounds.maximum = voxelBounds.minimum + PxVec3(gridSpacing);
		voxelBounds.fattenFast(0.1f * gridSpacing);
		bvh.query(voxelBounds, queryTris);

		for (PxI32 j = 0; j < 8; j++) {
			PxI32 id = v.ids[j];
			if (!isSurfaceVert[id])
				continue;

			PxVec3& p = tetVerts[id];

			PxF32 minDist2 = PX_MAX_F32;
			PxVec3 closest(PxZero);

			for (PxU32 k = 0; k < queryTris.size(); k++) {

				PxI32 triNr = queryTris[k];
				const PxVec3& p0 = surfaceVerts[surfaceTriIds[3 * triNr]];
				const PxVec3& p1 = surfaceVerts[surfaceTriIds[3 * triNr + 1]];
				const PxVec3& p2 = surfaceVerts[surfaceTriIds[3 * triNr + 2]];
				PxVec3 c, bary;
				getClosestPointOnTriangle(p0, p1, p2, p, c, bary);
				PxF32 dist2 = (c - p).magnitudeSquared();
				if (dist2 < minDist2) {
					minDist2 = dist2;
					closest = c;
				}
			}
			if (minDist2 < PX_MAX_F32) {
				PxVec3 n = p - closest;
				n.normalize();
				targetVertPos[id] = closest + n * surfaceDist;
			}
		}
	}
}

// -----------------------------------------------------------------------------------
void VoxelTetrahedralizer::createTets(bool subdivBorder, PxU32 numTetsPerVoxel)
{
	if (numTetsPerVoxel < 5 || numTetsPerVoxel > 6)
		return;

	createUniqueTetVertices();

	PxArray<Voxel> prevVoxels;

	PxArray<PxI32> numVertVoxels(tetVerts.size(), 0);
	tetIds.clear();

	for (PxU32 i = 0; i < voxels.size(); i++) {
		Voxel& v = voxels[i];
		for (PxI32 j = 0; j < 8; j++)
			numVertVoxels[v.ids[j]]++;

		PxI32 parity = (v.xi + v.yi + v.zi) % 2;

		if (v.inner || !subdivBorder) {
			if (numTetsPerVoxel == 6) {
				for (PxI32 j = 0; j < 6; j++) {
					tetIds.pushBack(v.ids[cubeSixTets[j][0]]);
					tetIds.pushBack(v.ids[cubeSixTets[j][1]]);
					tetIds.pushBack(v.ids[cubeSixTets[j][2]]);
					tetIds.pushBack(v.ids[cubeSixTets[j][3]]);
				}
			}
			else if (numTetsPerVoxel == 5) {
				for (PxI32 j = 0; j < 5; j++) {
					tetIds.pushBack(v.ids[cubeFiveTets[parity][j][0]]);
					tetIds.pushBack(v.ids[cubeFiveTets[parity][j][1]]);
					tetIds.pushBack(v.ids[cubeFiveTets[parity][j][2]]);
					tetIds.pushBack(v.ids[cubeFiveTets[parity][j][3]]);
				}
			}
		}
		else {
			PxVec3 p(PxZero);
			for (PxI32 j = 0; j < 8; j++)
				p += tetVerts[v.ids[j]];
			p /= 8.0;
			PxI32 newId = tetVerts.size();
			tetVerts.pushBack(p);
			origTetVerts.pushBack(p);
			numVertVoxels.pushBack(8);

			for (PxI32 j = 0; j < 12; j++) {

				const int* localIds;
				if (numTetsPerVoxel == 6)
					localIds = cubeSixSubdivTets[j];
				else
					localIds = cubeFiveSubdivTets[parity][j];

				for (PxI32 k = 0; k < 4; k++) {
					PxI32 id = localIds[k] < 8 ? v.ids[localIds[k]] : newId;
					tetIds.pushBack(id);
				}
			}
		}
	}

	isSurfaceVert.resize(tetVerts.size(), false);
	for (PxU32 i = 0; i < tetVerts.size(); i++)
		isSurfaceVert[i] = numVertVoxels[i] < 8;

	// randomize tets

	PxU32 numTets = tetIds.size() / 4;

	//for (PxU32 i = 0; i < numTets - 1; i++) {
	//	PxI32 ri = i + rand() % (numTets - i);
	//	for (PxI32 j = 0; j < 4; j++) {
	//		PxI32 id = tetIds[4 * i + j]; tetIds[4 * i + j] = tetIds[4 * ri + j]; tetIds[4 * ri + j] = id;
	//	}
	//}

	// edges

	MultiList<int> adjVerts;
	edgeIds.clear();

	adjVerts.clear();
	adjVerts.reserve(tetVerts.size());

	for (PxU32 i = 0; i < numTets; i++) {
		for (PxI32 j = 0; j < 6; j++) {
			PxI32 id0 = tetIds[4 * i + tetEdges[j][0]];
			PxI32 id1 = tetIds[4 * i + tetEdges[j][1]];

			if (!adjVerts.exists(id0, id1)) {
				edgeIds.pushBack(id0);
				edgeIds.pushBack(id1);

				adjVerts.addUnique(id0, id1);
				adjVerts.addUnique(id1, id0);
			}
		}
	}
}

// -----------------------------------------------------------------------------------
void VoxelTetrahedralizer::conserveVolume(PxF32 relMinVolume)
{
	PxVec3 grads[4];
	PxU32 numTets = tetIds.size() / 4;

	for (PxU32 i = 0; i < numTets; i++) {
		PxI32* ids = &tetIds[4 * i];

		PxF32 w = 0.0f;

		for (PxI32 j = 0; j < 4; j++) {
			PxI32 id0 = ids[volIdOrder[j][0]];
			PxI32 id1 = ids[volIdOrder[j][1]];
			PxI32 id2 = ids[volIdOrder[j][2]];

			grads[j] = (tetVerts[id1] - tetVerts[id0]).cross(tetVerts[id2] - tetVerts[id0]);
			w += grads[j].magnitudeSquared();
		}

		if (w == 0.0f)
			continue;

		PxVec3& p0 = tetVerts[ids[0]];
		PxF32 V = (tetVerts[ids[1]] - p0).cross(tetVerts[ids[2]] - p0).dot(tetVerts[ids[3]] - p0);

		PxVec3& origP0 = origTetVerts[ids[0]];
		PxF32 origV = (origTetVerts[ids[1]] - origP0).cross(origTetVerts[ids[2]] - origP0).dot(origTetVerts[ids[3]] - origP0);

		PxF32 minV = relMinVolume * origV;

		if (V < minV) {

			PxF32 C = V - minV;
			PxF32 lambda = -C / w;

			for (PxI32 j = 0; j < 4; j++) {
				tetVerts[ids[j]] += grads[j] * lambda;
			}
		}
	}
}

// -------------------------------------------------------------------------------------
void VoxelTetrahedralizer::relax(PxI32 numIters, PxF32 relMinVolume)
{
	const PxF32 targetScale = 0.3f;
	const PxF32 edgeScale = 0.3f;

	for (PxI32 iter = 0; iter < numIters; iter++) {
		PxU32 numVerts = tetVerts.size();

		for (PxU32 i = 0; i < numVerts; i++) {
			if (isSurfaceVert[i]) {
				PxVec3 offset = (targetVertPos[i] - tetVerts[i]) * targetScale;
				tetVerts[i] += offset;
			}
		}

		for (PxU32 i = 0; i < edgeIds.size(); i += 2) {
			PxI32 id0 = edgeIds[i];
			PxI32 id1 = edgeIds[i + 1];
			PxF32 w0 = isSurfaceVert[id0] ? 0.0f : 1.0f;
			PxF32 w1 = isSurfaceVert[id1] ? 0.0f : 1.0f;
			PxF32 w = w0 + w1;
			if (w == 0.0f)
				continue;
			PxVec3& p0 = tetVerts[id0];
			PxVec3& p1 = tetVerts[id1];

			PxVec3 e = (p1 - p0) * edgeScale;

			if (w == 1.0f)
				e *= 0.5f;

			p0 += w0 / w * e;
			p1 -= w1 / w * e;
		}
		conserveVolume(relMinVolume);
	}

	PxI32 volIters = 2;

	for (PxI32 volIter = 0; volIter < volIters; volIter++) 
		conserveVolume(relMinVolume);
}

// -----------------------------------------------------------------------------------
static PxF32 max3(PxF32 f0, PxF32 f1, PxF32 f2) {
	return PxMax(f0, PxMax(f1, f2));
}

static PxF32 min3(PxF32 f0, PxF32 f1, PxF32 f2) {
	return PxMin(f0, PxMin(f1, f2));
}

static PxF32 minMax(PxF32 f0, PxF32 f1, PxF32 f2) {
	return PxMax(-max3(f0, f1, f2), min3(f0, f1, f2));
}

// -----------------------------------------------------------------------------------
// PT: TODO: refactor with other SDK implementation
static bool boxTriangleIntersection(
	PxVec3 p0, PxVec3 p1, PxVec3 p2, PxVec3 center, PxVec3 extents)
{
	PxVec3 v0 = p0 - center, v1 = p1 - center, v2 = p2 - center;
	PxVec3 f0 = p1 - p0, f1 = p2 - p1, f2 = p0 - p2;
	PxF32 r;

	PxVec3 n = f0.cross(f1);
	PxF32 d = n.dot(v0);
	r = extents.x * fabsf(n.x) + extents.y * fabsf(n.y) + extents.z * fabsf(n.z);
	if (d > r || d < -r)
		return false;

	if (max3(v0.x, v1.x, v2.x) < -extents.x || min3(v0.x, v1.x, v2.x) > extents.x)
		return false;

	if (max3(v0.y, v1.y, v2.y) < -extents.y || min3(v0.y, v1.y, v2.y) > extents.y)
		return false;

	if (max3(v0.z, v1.z, v2.z) < -extents.z || min3(v0.z, v1.z, v2.z) > extents.z)
		return false;

	PxVec3 a00(0.0f, -f0.z, f0.y);
	r = extents.y * fabsf(f0.z) + extents.z * fabsf(f0.y);
	if (minMax(v0.dot(a00), v1.dot(a00), v2.dot(a00)) > r)
		return false;

	PxVec3 a01(0.0f, -f1.z, f1.y);
	r = extents.y * fabsf(f1.z) + extents.z * fabsf(f1.y);
	if (minMax(v0.dot(a01), v1.dot(a01), v2.dot(a01)) > r)
		return false;

	PxVec3 a02(0.0f, -f2.z, f2.y);
	r = extents.y * fabsf(f2.z) + extents.z * fabsf(f2.y);
	if (minMax(v0.dot(a02), v1.dot(a02), v2.dot(a02)) > r)
		return false;

	PxVec3 a10(f0.z, 0.0f, -f0.x);
	r = extents.x * fabsf(f0.z) + extents.z * fabsf(f0.x);
	if (minMax(v0.dot(a10), v1.dot(a10), v2.dot(a10)) > r)
		return false;

	PxVec3 a11(f1.z, 0.0f, -f1.x);
	r = extents.x * fabsf(f1.z) + extents.z * fabsf(f1.x);
	if (minMax(v0.dot(a11), v1.dot(a11), v2.dot(a11)) > r)
		return false;

	PxVec3 a12(f2.z, 0.0f, -f2.x);
	r = extents.x * fabsf(f2.z) + extents.z * fabsf(f2.x);
	if (minMax(v0.dot(a12), v1.dot(a12), v2.dot(a12)) > r)
		return false;

	PxVec3 a20(-f0.y, f0.x, 0.0f);
	r = extents.x * fabsf(f0.y) + extents.y * fabsf(f0.x);
	if (minMax(v0.dot(a20), v1.dot(a20), v2.dot(a20)) > r)
		return false;

	PxVec3 a21(-f1.y, f1.x, 0.0f);
	r = extents.x * fabsf(f1.y) + extents.y * fabsf(f1.x);
	if (minMax(v0.dot(a21), v1.dot(a21), v2.dot(a21)) > r)
		return false;

	PxVec3 a22(-f2.y, f2.x, 0.0f);
	r = extents.x * fabsf(f2.y) + extents.y * fabsf(f2.x);
	if (minMax(v0.dot(a22), v1.dot(a22), v2.dot(a22)) > r)
		return false;

	return true;
}

// -----------------------------------------------------------------------------------
// PT: TODO: refactor with other implementation
static void getClosestPointOnTriangle(
	PxVec3 p1, PxVec3 p2, PxVec3 p3, PxVec3 p, PxVec3& closest, PxVec3& bary)
{
	PxVec3 e0 = p2 - p1;
	PxVec3 e1 = p3 - p1;
	PxVec3 tmp = p1 - p;

	PxF32 a = e0.dot(e0);
	PxF32 b = e0.dot(e1);
	PxF32 c = e1.dot(e1);
	PxF32 d = e0.dot(tmp);
	PxF32 e = e1.dot(tmp);
	PxVec3 coords, clampedCoords;
	coords.x = b * e - c * d;    // s * det
	coords.y = b * d - a * e;    // t * det
	coords.z = a * c - b * b;    // det

	clampedCoords = PxVec3(0.0f, 0.0f, 0.0f);
	if (coords.x <= 0.0f) {
		if (c != 0.0f)
			clampedCoords.y = -e / c;
	}
	else if (coords.y <= 0.0f) {
		if (a != 0.0f)
			clampedCoords.x = -d / a;
	}
	else if (coords.x + coords.y > coords.z) {
		PxF32 denominator = a + c - b - b;
		PxF32 numerator = c + e - b - d;
		if (denominator != 0.0f) {
			clampedCoords.x = numerator / denominator;
			clampedCoords.y = 1.0f - clampedCoords.x;
		}
	}
	else {    // all inside
		if (coords.z != 0.0f) {
			clampedCoords.x = coords.x / coords.z;
			clampedCoords.y = coords.y / coords.z;
		}
	}
	clampedCoords.x = PxMax(clampedCoords.x, 0.0f);
	clampedCoords.y = PxMax(clampedCoords.y, 0.0f);
	clampedCoords.x = PxMin(clampedCoords.x, 1.0f);
	clampedCoords.y = PxMin(clampedCoords.y, 1.0f);

	closest = p1 + e0 * clampedCoords.x + e1 * clampedCoords.y;

	bary.x = 1.0f - clampedCoords.x - clampedCoords.y;
	bary.y = clampedCoords.x;
	bary.z = clampedCoords.y;
}
