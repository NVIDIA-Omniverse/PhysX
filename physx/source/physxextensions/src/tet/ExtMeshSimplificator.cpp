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

#include "ExtMeshSimplificator.h"
#include "foundation/PxSort.h"

using namespace physx;
using namespace Ext;

// -------------------------------------------------------------------------------------
MeshSimplificator::MeshSimplificator()
{
	currentVertMark = 0;
	numMeshTris = 0;
}

// -------------------------------------------------------------------------------------
void MeshSimplificator::findTriNeighbors()
{
	PxI32 numTris = PxI32(triIds.size()) / 3;
	triNeighbors.clear();
	triNeighbors.resize(3 * numTris, -1);

	struct Edge 
	{
		PX_FORCE_INLINE void init(PxI32 _id0, PxI32 _id1, PxI32 _triNr, PxI32 _edgeNr) 
		{
			this->id0 = PxMin(_id0, _id1);
			this->id1 = PxMax(_id0, _id1);
			this->triNr = _triNr;
			this->edgeNr = _edgeNr;
		}
		PX_FORCE_INLINE bool operator < (const Edge& e) const
		{
			return (id0 < e.id0 || (id0 == e.id0 && id1 < e.id1));
		}
		PX_FORCE_INLINE bool operator == (const Edge& e) const
		{
			return id0 == e.id0 && id1 == e.id1;
		}
		PxI32 id0, id1, triNr, edgeNr;
	};

	PxArray<Edge> edges(PxI32(triIds.size()));

	for (PxI32 i = 0; i < numTris; i++) 
	{
		for (PxI32 j = 0; j < 3; j++) 
		{
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
		while (nr < PxI32(edges.size()) && edges[nr] == e0) 
		{
			Edge& e1 = edges[nr];
			triNeighbors[3 * e0.triNr + e0.edgeNr] = e1.triNr;
			triNeighbors[3 * e1.triNr + e1.edgeNr] = e0.triNr;
			nr++;
		}
	}
}

// -------------------------------------------------------------------------------------
bool MeshSimplificator::getAdjTris(PxI32 triNr, PxI32 vertNr, PxI32& valence, bool& open, PxArray<PxI32>* tris) const
{
	open = false;

	if (tris)
		tris->clear();

	PxI32 cnt = 0;
	valence = 0;
	PxI32 nr = triNr;

	// counter clock
	do 
	{
		if (tris)
			tris->pushBack(nr);
		valence++;
		if (triIds[3 * nr] == vertNr)
			nr = triNeighbors[3 * nr + 2];
		else if (triIds[3 * nr + 1] == vertNr)
			nr = triNeighbors[3 * nr];
		else
			nr = triNeighbors[3 * nr + 1];
		cnt++;
	} 
	while (nr >= 0 && nr != triNr && cnt < 100);

	if (cnt >= 100) 
	{
		valence = 0;
		return false;
	}

	cnt = 0;

	if (nr < 0) 
	{		// open: search clockwise too
		open = true;
		nr = triNr;
		do 
		{
			if (nr != triNr) 
			{
				if (tris)
					tris->pushBack(nr);
				valence++;
			}
			if (triIds[3 * nr] == vertNr)
				nr = triNeighbors[3 * nr];
			else if (triIds[3 * nr + 1] == vertNr)
				nr = triNeighbors[3 * nr + 1];
			else
				nr = triNeighbors[3 * nr + 2];
			cnt++;
		} 
		while (nr >= 0 && nr != triNr && cnt < 100);

		valence++;		// num tris + 1 if open

		if (cnt > 100) 
		{
			valence = 0;
			return false;
		}
	}

	return true;
}

// -------------------------------------------------------------------------------------
bool MeshSimplificator::getAdjTris(PxI32 triNr, PxI32 vertNr, PxArray<PxI32>& tris) const
{
	PxI32 valence;
	bool open;
	return getAdjTris(triNr, vertNr, valence, open, &tris);

}

// -------------------------------------------------------------------------------------
void MeshSimplificator::replaceNeighbor(PxI32 triNr, PxI32 oldNeighbor, PxI32 newNeighbor)
{
	if (triNr < 0)
		return;
	for (PxI32 i = 0; i < 3; i++) 
	{
		if (triNeighbors[3 * triNr + i] == oldNeighbor)
			triNeighbors[3 * triNr + i] = newNeighbor;
	}
}

// -------------------------------------------------------------------------------------
PxI32 MeshSimplificator::getEdgeId(PxI32 triNr, PxI32 edgeNr)
{
	PxI32 n = triNeighbors[3 * triNr + edgeNr];
	if (n < 0 || triNr < n)
		return 3 * triNr + edgeNr;
	else 
	{
		for (PxI32 i = 0; i < 3; i++)
		{
			if (triNeighbors[3 * n + i] == triNr)
				return 3 * n + i;
		}
	}
	return 0;
}

inline PxVec3Ex MeshSimplificator::projectPoint(const PxVec3& p)
{
	PxU32 triangleId;
	PxVec3 pos = projector->projectPoint(p, triangleId);
	return PxVec3Ex(pos, triangleId);
}

// -------------------------------------------------------------------------------------
PxVec3Ex MeshSimplificator::evalEdgeCost(PxI32 triNr, PxI32 edgeNr, PxReal &cost)
{
	const PxI32 numSteps = 10;
	cost = -1.0f;
	PxI32 id0 = triIds[3 * triNr + edgeNr];
	PxI32 id1 = triIds[3 * triNr + (edgeNr + 1) % 3];

	PxReal minCost = FLT_MAX;	
	PxReal maxCost = -FLT_MAX;		

	Quadric q; q = quadrics[id0] + quadrics[id1];
	PxVec3Ex pos;

	PxReal edgeLength = (vertices[id0].p - vertices[id1].p).magnitude();

	for (PxI32 i = 0; i <= numSteps; i++) 
	{
		float r = 1.0f / numSteps * i;
		pos.p = vertices[id0].p * (1.0f - r) + vertices[id1].p * r;
		if (projector)
			pos = projectPoint(pos.p);

		float c = q.outerProduct(pos.p);
		c += edgeLengthCostWeight * edgeLength;
		if (cost < 0.0f || c < cost) 
		{
			cost = c; 
		}
		if (cost > maxCost) 				
			maxCost = cost; 				
		if (cost < minCost) 				
			minCost = cost; 				
	}

	if (maxCost - minCost < flatnessDetectionThreshold)
	{
		float r = 0.5f;
		pos.p = vertices[id0].p * (1.0f - r) + vertices[id1].p * r;
		if (projector)
			pos = projectPoint(pos.p);
		cost = q.outerProduct(pos.p) + edgeLengthCostWeight * edgeLength;
	}			
	return pos;
}

// -------------------------------------------------------------------------------------
bool MeshSimplificator::collapseEdge(PxI32 triNr, PxI32 edgeNr)
{
	if (triIds[3 * triNr] == triIds[3 * triNr + 1])		// the triangle deleted
		return false;

	PxI32 id0 = triIds[3 * triNr + edgeNr];
	PxI32 id1 = triIds[3 * triNr + (edgeNr + 1) % 3];
	PxI32 id2 = triIds[3 * triNr + (edgeNr + 2) % 3];
	PxI32 id3 = -1;

	PxI32 n = triNeighbors[3 * triNr + edgeNr];
	PxI32 nEdgeNr = 0;
	if (n >= 0) 
	{
		if (triNeighbors[3 * n] == triNr) nEdgeNr = 0;
		else if (triNeighbors[3 * n + 1] == triNr) nEdgeNr = 1;
		else if (triNeighbors[3 * n + 2] == triNr) nEdgeNr = 2;
		id3 = triIds[3 * n + (nEdgeNr + 2) % 3];
	}

	// not legal if there exists id != id0,id1 with (id0,id) and (id1,id) edges
	// but (id,id0,id1) is not a triangle

	bool OK = getAdjTris(triNr, id0, adjTris);
	currentVertMark++;
	for (PxI32 i = 0; i < PxI32(adjTris.size()); i++) 
	{
		PxI32 adj = adjTris[i];
		for (PxI32 j = 0; j < 3; j++)
			vertMarks[triIds[3 * adj + j]] = currentVertMark;
	}
	OK = OK && getAdjTris(triNr, id1, adjTris);
	if (!OK)
		return false;
	for (PxI32 i = 0; i < PxI32(adjTris.size()); i++) 
	{
		PxI32 adj = adjTris[i];
		for (PxI32 j = 0; j < 3; j++) 
		{
			PxI32 id = triIds[3 * adj + j];
			if (vertMarks[id] == currentVertMark &&
				id != id0 && id != id1 && id != id2 && id != id3)
				return false;
		}
	}

	// new center pos

	PxReal cost;
	PxVec3Ex newPos = evalEdgeCost(triNr, edgeNr, cost);
	//PxVec3 newPos = vertices[id0] * (1.0f - ratio) + vertices[id1] * ratio;
			
	// any triangle flips?

	for (PxI32 side = 0; side < 2; side++) 
	{
		getAdjTris(triNr, side == 0 ? id0 : id1, adjTris);
		PxI32 other = side == 0 ? id1 : id0;
		for (PxU32 i = 0; i < adjTris.size(); i++) 
		{
			PxI32 adj = adjTris[i];
			PxVec3 p[3], q[3];
			bool deleted = false;
			for (PxI32 j = 0; j < 3; j++) 
			{
				PxI32 id = triIds[3 * adj + j];
				if (id == other)
					deleted = true;
				p[j] = vertices[id].p;
				q[j] = (id == id0 || id == id1) ? newPos.p : p[j];
			}
			if (!deleted)
			{
				PxVec3 n0 = (p[1] - p[0]).cross(p[2] - p[0]);
				PxVec3 n1 = (q[1] - q[0]).cross(q[2] - q[0]);
				if (n0.dot(n1) <= 0.0f)
					return false;
			}
		}
	}

	// remove adjacent edges from heap

	for (PxI32 side = 0; side < 2; side++)
	{
		PxI32 id = side == 0 ? id0 : id1;
		getAdjTris(triNr, id, adjTris);
		for (PxU32 i = 0; i < adjTris.size(); i++)
		{
			PxI32 adj = adjTris[i];
			for (PxI32 j = 0; j < 3; j++) 
			{
				PxI32 adj0 = triIds[3 * adj + j];
				PxI32 adj1 = triIds[3 * adj + (j + 1) % 3];
				if (adj0 == id0 || adj0 == id1 || adj1 == id0 || adj1 == id1) 
				{
					PxI32 edgeId = getEdgeId(adj, j);
					heap.remove(edgeId);
				}
			}
		}
	}

	// move vertex

	if (id0 > id1) {
		int id = id0; id0 = id1; id1 = id;
	}

	vertices[id0] = newPos;
	quadrics[id0] += quadrics[id1];

	// collapse edge

	getAdjTris(triNr, id1, adjTris);

	for (PxU32 i = 0; i < adjTris.size(); i++)
	{
		PxI32 adj = adjTris[i];
		for (PxI32 j = 0; j < 3; j++) {
			PxI32& id = triIds[3 * adj + j];
			if (id == id1)
				id = id0;
		}
	}

	simplificationMap[id1] = id0;

	// mark triangles as deleted (duplicate indices, can still be rendered)

	triIds[3 * triNr + 2] = triIds[3 * triNr + 1] = triIds[3 * triNr];
	numMeshTris--;
	if (n >= 0) 
	{
		triIds[3 * n + 2] = triIds[3 * n + 1] = triIds[3 * n];
		numMeshTris--;
	}

	// update neighbors

	PxI32 right = triNeighbors[3 * triNr + (edgeNr + 1) % 3];
	PxI32 left = triNeighbors[3 * triNr + (edgeNr + 2) % 3];
	replaceNeighbor(right, triNr, left);
	replaceNeighbor(left, triNr, right);
	PxI32 startTriNr = PxMax(right, left);

	if (n >= 0)
	{
		right = triNeighbors[3 * n + (nEdgeNr + 1) % 3];
		left = triNeighbors[3 * n + (nEdgeNr + 2) % 3];
		replaceNeighbor(right, n, left);
		replaceNeighbor(left, n, right);
		startTriNr = PxMax(startTriNr, PxMax(right, left));
	}

	// add new edges to heap

	if (startTriNr >= 0) 
	{
		getAdjTris(startTriNr, id0, adjTris);
		for (PxU32 i = 0; i < adjTris.size(); i++) 
		{
			PxI32 adj = adjTris[i];
			for (PxI32 j = 0; j < 3; j++)
			{
				PxI32 adj0 = triIds[3 * adj + j];
				PxI32 adj1 = triIds[3 * adj + (j + 1) % 3];
				if (adj0 == id0 || adj1 == id0) 
				{
					evalEdgeCost(adj, j, cost);
					PxI32 id = getEdgeId(adj, j);

					heap.insert(HeapElem(adj, j, cost), id);
				}
			}
		}
	}
	return true;
}

#if PX_LINUX
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#endif

static void minMax(const PxArray<PxVec3Ex>& points, PxVec3& min, PxVec3& max)
{
	min = PxVec3(FLT_MAX, FLT_MAX, FLT_MAX);
	max = PxVec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	for (PxU32 i = 0; i < points.size(); ++i)
	{
		const PxVec3& p = points[i].p;				
		if (p.x > max.x) max.x = p.x; if (p.y > max.y) max.y = p.y; if (p.z > max.z) max.z = p.z;
		if (p.x < min.x) min.x = p.x; if (p.y < min.y) min.y = p.y; if (p.z < min.z) min.z = p.z;
	}
}

#if PX_LINUX
#pragma GCC diagnostic pop
#endif

void MeshSimplificator::transformPointsToUnitBox(PxArray<PxVec3Ex>& points)
{
	PxVec3 min, max;
	minMax(points, min, max);
	origin = min;
	PxVec3 size = max - min;

	scaling = 1.0f / PxMax(PxMax(1e-6f, size.x), PxMax(size.y, size.z));

	for (PxU32 i = 0; i < points.size(); ++i)
		points[i].p = (points[i].p - min) * scaling;
}

void MeshSimplificator::transformPointsToOriginalPosition(PxArray<PxVec3>& points)
{
	PxReal s = 1.0f / scaling;
	for (PxU32 i = 0; i < points.size(); ++i)
		points[i] = points[i] * s + origin;
}

// -------------------------------------------------------------------------------------
void MeshSimplificator::init(const PxSimpleTriangleMesh& inputMesh, PxReal edgeLengthCostWeight_, 
	PxReal flatnessDetectionThreshold_, bool projectSimplifiedPointsOnInputMeshSurface)
{
	edgeLengthCostWeight = edgeLengthCostWeight_;
	flatnessDetectionThreshold = flatnessDetectionThreshold_;

	vertices.resize(inputMesh.points.count);
	for (PxU32 i = 0; i < inputMesh.points.count; i++)
		vertices[i] = PxVec3Ex(inputMesh.points.at<PxVec3>(i));

	transformPointsToUnitBox(vertices);

	PxI32 numIndices = 3 * inputMesh.triangles.count;
	triIds.resize(numIndices);

	if (inputMesh.flags & PxMeshFlag::e16_BIT_INDICES)
	{
		for (PxI32 i = 0; i < numIndices; i++)
			triIds[i] = PxI32(inputMesh.triangles.at<PxU16>(i));
	}
	else 
	{
		for (PxI32 i = 0; i < numIndices; i++)
			triIds[i] = PxI32(inputMesh.triangles.at<PxU32>(i));
	}

	for (PxU32 i = 0; i < triIds.size(); i++)
		vertices[triIds[i]].i = i / 3;

	if (projectSimplifiedPointsOnInputMeshSurface)
	{
		originalTriIds.resize(triIds.size());
		for (PxU32 i = 0; i < triIds.size(); ++i)
			originalTriIds[i] = triIds[i];
		scaledOriginalVertices.resize(inputMesh.points.count);
		for (PxU32 i = 0; i < inputMesh.points.count; i++)
			scaledOriginalVertices[i] = vertices[i].p;
		projector = Gu::PxCreatePointOntoTriangleMeshProjector(scaledOriginalVertices.begin(), originalTriIds.begin(), inputMesh.triangles.count);
	}
	else
		projector = NULL;

	init();
}

// -------------------------------------------------------------------------------------
void MeshSimplificator::init(const PxArray<PxVec3> &inputVertices, const PxArray<PxU32> &inputTriIds, PxReal edgeLengthCostWeight_, 
	PxReal flatnessDetectionThreshold_, bool projectSimplifiedPointsOnInputMeshSurface)
{
	edgeLengthCostWeight = edgeLengthCostWeight_;
	flatnessDetectionThreshold = flatnessDetectionThreshold_;

	vertices.resize(inputVertices.size());
	for (PxU32 i = 0; i < inputVertices.size(); i++)
		vertices[i] = PxVec3Ex(inputVertices[i]);

	for (PxU32 i = 0; i < inputTriIds.size(); i++)
		vertices[inputTriIds[i]].i = i / 3;

	transformPointsToUnitBox(vertices);

	triIds.resize(inputTriIds.size());
	for (PxU32 i = 0; i < inputTriIds.size(); i++)
		triIds[i] = PxI32(inputTriIds[i]);

	if (projectSimplifiedPointsOnInputMeshSurface)
	{
		scaledOriginalVertices.resize(inputVertices.size());
		for (PxU32 i = 0; i < inputVertices.size(); i++)
			scaledOriginalVertices[i] = vertices[i].p;
		projector = Gu::PxCreatePointOntoTriangleMeshProjector(scaledOriginalVertices.begin(), inputTriIds.begin(), inputTriIds.size() / 3);
	}
	else
		projector = NULL;

	init();
}

MeshSimplificator::~MeshSimplificator()
{
	if (projector)
	{
		PX_RELEASE(projector)
	}
}

// -------------------------------------------------------------------------------------
void MeshSimplificator::init()
{
	vertMarks.clear();
	vertMarks.resize(vertices.size(), 0);
	currentVertMark = 0;

	findTriNeighbors();

	// init vertex quadrics

	quadrics.resize(vertices.size());
	for (PxU32 i = 0; i < vertices.size(); i++)
		quadrics[i].zero();

	Quadric q;

	PxI32 numTris = PxI32(triIds.size()) / 3;

	for (PxI32 i = 0; i < numTris; i++)
	{
		PxI32 id0 = triIds[3 * i];
		PxI32 id1 = triIds[3 * i + 1];
		PxI32 id2 = triIds[3 * i + 2];
		q.setFromPlane(vertices[id0].p, vertices[id1].p, vertices[id2].p);
		quadrics[id0] += q;
		quadrics[id1] += q;
		quadrics[id2] += q;
	}

	// init heap

	heap.clear();

	for (PxI32 i = 0; i < numTris; i++)
	{
		for (PxI32 j = 0; j < 3; j++) 
		{
			PxI32 n = triNeighbors[3 * i + j];
			if (n < 0 || i < n) 
			{
				PxReal cost;
				evalEdgeCost(i, j, cost);
				heap.insert(HeapElem(i, j, cost), getEdgeId(i, j));
			}
		}
	}
	numMeshTris = numTris;

	// init simplification map

	simplificationMap.resize(vertices.size());
	for (PxI32 i = 0; i < PxI32(vertices.size()); i++)
		simplificationMap[i] = i; // each vertex is a root
}

// -------------------------------------------------------------------------------------
bool MeshSimplificator::step(PxF32 maximalEdgeLength)
{
	int heapMinSize = 20;

	if (heap.size() < heapMinSize)
		return false;

	while (heap.size() > heapMinSize) 
	{
		HeapElem e = heap.deleteMin();

		PxI32 id0 = triIds[3 * e.triNr + e.edgeNr];
		PxI32 id1 = triIds[3 * e.triNr + (e.edgeNr + 1) % 3];
		PxF32 length = (vertices[id0].p - vertices[id1].p).magnitude();
		if (maximalEdgeLength == 0.0f || length < maximalEdgeLength) 
		{
			collapseEdge(e.triNr, e.edgeNr);
			return true;
		}
	}
	return false;
}

// -------------------------------------------------------------------------------------
void MeshSimplificator::decimateByRatio(PxF32 relativeOutputMeshSize, PxF32 maximalEdgeLength)
{
	relativeOutputMeshSize = PxClamp(relativeOutputMeshSize, 0.1f, 0.99f);
	PxI32 numSteps = PxI32(PxFloor(PxF32(heap.size()) * (1.0f - relativeOutputMeshSize)));
	for (PxI32 i = 0; i < numSteps; i++)
	{
		if (!step(maximalEdgeLength))
			break;
	}
}

// -------------------------------------------------------------------------------------
void MeshSimplificator::decimateBySize(PxI32 targetTriangleCount, PxF32 maximalEdgeLength)
{
	while (numMeshTris > targetTriangleCount) 
	{
		if (!step(maximalEdgeLength))
			break;
	}
}

// -------------------------------------------------------------------------------------
void MeshSimplificator::readBack(PxArray<PxVec3>& outVertices, PxArray<PxU32>& outTriIds, PxArray<PxU32> *vertexMap, PxArray<PxU32> *outputVertexToInputTriangle)
{
	outVertices.clear();
	outTriIds.clear();
	PxArray<PxI32> idMap(vertices.size(), -1);

	PxI32 numTris = PxI32(triIds.size()) / 3;

	for (PxI32 i = 0; i < numTris; i++)
	{
		if (triIds[3 * i] == triIds[3 * i + 1])		// deleted
			continue;

		for (PxI32 j = 0; j < 3; j++)
		{
			PxI32 id = triIds[3 * i + j];
			if (idMap[id] < 0) 
			{
				idMap[id] = outVertices.size();
				outVertices.pushBack(vertices[id].p);
				if(outputVertexToInputTriangle && projector)
					outputVertexToInputTriangle->pushBack(vertices[id].i);
			}
			outTriIds.pushBack(PxU32(idMap[id]));
		}
	}

	transformPointsToOriginalPosition(outVertices);

	if (vertexMap)
	{
		for (PxU32 i = 0; i < simplificationMap.size(); ++i)
		{
			PxI32 id = i;
			while (id != simplificationMap[id])
			{
				id = simplificationMap[id];							
			}
			const PxI32 finalLink = id;
			id = i;
			simplificationMap[i] = finalLink;					
			while (id != simplificationMap[id])
			{
				PxI32 oldId = id;
				id = simplificationMap[id];
				simplificationMap[oldId] = finalLink;						
			}
		}

		vertexMap->resize(vertices.size());
		for (PxU32 i = 0; i < simplificationMap.size(); ++i)
		{
			PxI32 mapped = idMap[simplificationMap[i]];				
			(*vertexMap)[i] = mapped;
		}
	}
}

