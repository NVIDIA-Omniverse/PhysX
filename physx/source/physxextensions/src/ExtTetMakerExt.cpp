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

#include "tet/ExtDelaunayBoundaryInserter.h"
#include "extensions/PxTetMakerExt.h"
#include "cooking/PxTetrahedronMeshDesc.h"
#include "geometry/PxTriangleMesh.h"
#include "tet/ExtMeshSimplificator.h"
#include "tet/ExtRemesher.h"
#include "tet/ExtOctreeTetrahedralizer.h"
#include "tet/ExtVoxelTetrahedralizer.h"
#include "foundation/PxMat33.h"
#include <stdio.h>

using namespace physx;

PX_FORCE_INLINE PxReal computeTetrahedronVolume(const PxVec3& x0, const PxVec3& x1, const PxVec3& x2, const PxVec3& x3)
{
	const PxVec3 u1 = x1 - x0;
	const PxVec3 u2 = x2 - x0;
	const PxVec3 u3 = x3 - x0;

	PxMat33 edgeMatrix = PxMat33(u1, u2, u3);

	const PxReal det = edgeMatrix.getDeterminant();

	const PxReal volume = det / 6.0f;
	return volume;
}

//Remove tets with small volume
void removeSmallVolumeTetrahedra(PxArray<::physx::PxVec3>& vertices, PxArray<PxU32>& indices, PxReal volumeThreshold = 1e-8f)
{
	uint32_t indexer = 0;

	for (uint32_t i = 0; i < indices.size(); i += 4)
	{
		for (uint32_t j = 0; j < 4; ++j)
		{
			indices[indexer + j] = indices[i + j];
		}

		if (computeTetrahedronVolume(vertices[indices[i]], vertices[indices[i + 1]], vertices[indices[i + 2]], vertices[indices[i + 3]]) >= volumeThreshold)
		{
			indexer += 4;
		}
	}

	if (indexer < indices.size())
	{
		indices.removeRange(indexer, indices.size() - indexer);
	}
}

//Removes vertices not referenced by any tetrahedron and maps the tet's indices to match the compacted vertex list
void removeUnusedVertices(PxArray<::physx::PxVec3>& vertices, PxArray<PxU32>& tets, PxU32 numPointsToKeepAtBeginning = 0)
{
	PxArray<PxI32> compressorMap;
	compressorMap.resize(vertices.size());

	for (PxU32 i = 0; i < numPointsToKeepAtBeginning; ++i)
		compressorMap[i] = 0;
	for (PxU32 i = numPointsToKeepAtBeginning; i < compressorMap.size(); ++i)
		compressorMap[i] = -1;

	for (PxU32 i = 0; i < tets.size(); i += 4)
	{
		const PxU32* tet = &tets[i];
		if (tet[0] == 0xFFFFFFFFu)
			continue;
		compressorMap[tet[0]] = 0;
		compressorMap[tet[1]] = 0;
		compressorMap[tet[2]] = 0;
		compressorMap[tet[3]] = 0;
	}

	PxU32 indexer = 0;
	for (PxU32 i = 0; i < compressorMap.size(); ++i)
	{
		if (compressorMap[i] >= 0)
		{
			compressorMap[i] = indexer;
			vertices[indexer] = vertices[i];
			indexer++;
		}
	}

	for (PxU32 i = 0; i < tets.size(); i += 4)
	{
		PxU32* tet = &tets[i];
		if (tet[0] == 0xFFFFFFFFu)
			continue;
		tet[0] = compressorMap[tet[0]];
		tet[1] = compressorMap[tet[1]];
		tet[2] = compressorMap[tet[2]];
		tet[3] = compressorMap[tet[3]];
	}

	if (indexer < vertices.size())
		vertices.removeRange(indexer, vertices.size() - indexer);
}


PX_FORCE_INLINE PxU64 buildKey(PxI32 a, PxI32 b)
{
	if (a < b)
		return ((PxU64(a)) << 32) | (PxU64(b));
	else
		return ((PxU64(b)) << 32) | (PxU64(a));
}

static const PxI32 neighborEdgeList[3][2] = { { 0, 1 }, { 0, 2 }, { 1, 2 } };

static void buildTriangleNeighborhood(const PxI32* tris, PxU32 numTris, PxArray<PxI32>& result)
{
	PxU32 l = 4 * numTris; //Waste one element in neighborhood info but allow bit shift access instead
	result.clear();
	result.resize(l, -1);

	PxHashMap<PxU64, PxI32> faces;
	for (PxU32 i = 0; i < numTris; ++i)
	{
		const PxI32* tri = &tris[3 * i];
		if (tris[0] < 0)
			continue;

		for (PxI32 j = 0; j < 3; ++j)
		{
			PxU64 key = buildKey(tri[neighborEdgeList[j][0]], tri[neighborEdgeList[j][1]]);
			if (const PxPair<const PxU64, PxI32>* ptr = faces.find(key))
			{
				if (ptr->second < 0)
				{
					//PX_ASSERT(false); //Invalid tetmesh since a face is shared by more than 2 tetrahedra
					continue;
				}

				result[4 * i + j] = ptr->second;
				result[ptr->second] = 4 * i + j;

				faces[key] = -1;
			}
			else
				faces.insert(key, 4 * i + j);
		}
	}
}
void PxTetMaker::detectTriangleIslands(const PxI32* triangles, PxU32 numTriangles, PxArray<PxU32>& islandIndexPerTriangle)
{
	//Detect islands
	PxArray<PxI32> neighborhood;
	buildTriangleNeighborhood(triangles, numTriangles, neighborhood);
	const PxU32 noIslandAssignedMarker = 0xFFFFFFFF;
	islandIndexPerTriangle.resize(numTriangles, noIslandAssignedMarker);
	PxU32 start = 0;
	PxI32 color = -1;
	PxArray<PxI32> stack;
	while (true)
	{
		stack.clear();
		while (start < islandIndexPerTriangle.size())
		{
			if (islandIndexPerTriangle[start] == noIslandAssignedMarker)
			{
				stack.pushBack(start);
				++color;
				islandIndexPerTriangle[start] = color;
				break;
			}
			++start;
		}

		if (start == islandIndexPerTriangle.size())
			break;

		while (stack.size() > 0)
		{
			PxI32 id = stack.popBack();
			for (PxI32 i = 0; i < 3; ++i)
			{
				PxI32 a = neighborhood[4 * id + i];
				PxI32 tetId = a >> 2;
				if (tetId >= 0 && islandIndexPerTriangle[tetId] == noIslandAssignedMarker)
				{
					stack.pushBack(tetId);
					islandIndexPerTriangle[tetId] = color;
				}
			}
		}
	}
}

PxU32 PxTetMaker::findLargestIslandId(const PxU32* islandIndexPerTriangle, PxU32 numTriangles)
{
	PxU32 numIslands = 0;
	for (PxU32 i = 0; i < numTriangles; ++i)
		numIslands = PxMax(numIslands, islandIndexPerTriangle[i]);
	++numIslands;

	PxArray<PxU32> numEntriesPerColor;
	numEntriesPerColor.resize(numIslands, 0);
	for (PxU32 i = 0; i < numTriangles; ++i)
		numEntriesPerColor[islandIndexPerTriangle[i]] += 1;

	PxU32 colorWithHighestTetCount = 0;
	for (PxU32 i = 1; i < numEntriesPerColor.size(); ++i)
		if (numEntriesPerColor[i] > numEntriesPerColor[colorWithHighestTetCount])
			colorWithHighestTetCount = i;

	return colorWithHighestTetCount;
}

bool PxTetMaker::createConformingTetrahedronMesh(const PxSimpleTriangleMesh& triangleMesh,
	physx::PxArray<physx::PxVec3>& outVertices, physx::PxArray<physx::PxU32>& outTetIndices, const bool validate, PxReal volumeThreshold)
{
	if (validate)
	{
		PxTriangleMeshAnalysisResults result = PxTetMaker::validateTriangleMesh(triangleMesh);		
		if (result & PxTriangleMeshAnalysisResult::eMESH_IS_INVALID) 
		{
			PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, PX_FL, "createConformingTetrahedronMesh(): Input triangle mesh is not suited to create a tetmesh due to deficiencies. Please call PxTetMaker::validateTriangleMesh(triangleMesh) for more details.");
			return false;
		}
	}

	Ext::generateTetmesh(triangleMesh.points, triangleMesh.triangles, triangleMesh.flags & PxMeshFlag::e16_BIT_INDICES, outVertices, outTetIndices);
	
	if (volumeThreshold > 0.0f)
		removeSmallVolumeTetrahedra(outVertices, outTetIndices, volumeThreshold);

	PxU32 numRemoveAtEnd = Ext::removeDisconnectedIslands(reinterpret_cast<PxI32*>(outTetIndices.begin()), outTetIndices.size() / 4);
	if (numRemoveAtEnd > 0)
		outTetIndices.removeRange(outTetIndices.size() - 4 * numRemoveAtEnd, 4 * numRemoveAtEnd);

	removeUnusedVertices(outVertices, outTetIndices, triangleMesh.points.count);

	return true;
}

bool PxTetMaker::createVoxelTetrahedronMesh(const PxTetrahedronMeshDesc& tetMesh,
	const PxU32 numVoxelsAlongLongestBoundingBoxAxis, physx::PxArray<physx::PxVec3>& outVertices, physx::PxArray<physx::PxU32>& outTetIndices,
	PxI32* intputPointToOutputTetIndex, const PxU32* anchorNodeIndices, PxU32 numTetsPerVoxel)
{
	//numTetsPerVoxel has only two valid values.
	if (numTetsPerVoxel != 5 && numTetsPerVoxel != 6)
		numTetsPerVoxel = 5;
	Ext::generateVoxelTetmesh(tetMesh.points, tetMesh.tetrahedrons,	numVoxelsAlongLongestBoundingBoxAxis, outVertices, outTetIndices, intputPointToOutputTetIndex, anchorNodeIndices, numTetsPerVoxel);
	return true;
}

bool PxTetMaker::createVoxelTetrahedronMeshFromEdgeLength(const PxTetrahedronMeshDesc& tetMesh,
	const PxReal voxelEdgeLength, physx::PxArray<physx::PxVec3>& outVertices, physx::PxArray<physx::PxU32>& outTetIndices, 
	PxI32* intputPointToOutputTetIndex, const PxU32* anchorNodeIndices, PxU32 numTetsPerVoxel)
{
	//numTetsPerVoxel has only two valid values.
	if (numTetsPerVoxel != 5 && numTetsPerVoxel != 6)
		numTetsPerVoxel = 5;
	Ext::generateVoxelTetmesh(tetMesh.points, tetMesh.tetrahedrons, voxelEdgeLength, outVertices, outTetIndices, intputPointToOutputTetIndex, anchorNodeIndices, numTetsPerVoxel);
	return true;
}

PxTriangleMeshAnalysisResults PxTetMaker::validateTriangleMesh(const PxSimpleTriangleMesh& triangleMesh, const PxReal minVolumeThreshold, const PxReal minTriangleAngleRadians)
{
	return Ext::validateTriangleMesh(triangleMesh.points, triangleMesh.triangles, triangleMesh.flags & PxMeshFlag::e16_BIT_INDICES, minVolumeThreshold, minTriangleAngleRadians);
}

PxTetrahedronMeshAnalysisResults PxTetMaker::validateTetrahedronMesh(const PxBoundedData& points, const PxBoundedData& tetrahedra, const PxReal minTetVolumeThreshold)
{
	return Ext::validateTetrahedronMesh(points, tetrahedra, false, minTetVolumeThreshold);
}

void PxTetMaker::simplifyTriangleMesh(const PxArray<PxVec3>& inputVertices, const PxArray<PxU32>&inputIndices, int targetTriangleCount, PxF32 maximalEdgeLength,
	PxArray<PxVec3>& outputVertices, PxArray<PxU32>& outputIndices,
	PxArray<PxU32> *vertexMap, PxReal edgeLengthCostWeight, PxReal flatnessDetectionThreshold,
	bool projectSimplifiedPointsOnInputMeshSurface, PxArray<PxU32>* outputVertexToInputTriangle, bool removeDisconnectedPatches)
{
	Ext::MeshSimplificator ms;

	PxArray<PxU32> indexMapToFullTriangleSet;
	if (removeDisconnectedPatches)
	{
		PxU32 numTriangles = inputIndices.size() / 3;
		PxArray<PxU32> islandIndexPerTriangle;
		PxTetMaker::detectTriangleIslands(reinterpret_cast<const PxI32*>(inputIndices.begin()), numTriangles, islandIndexPerTriangle);

		PxU32 biggestIslandIndex = PxTetMaker::findLargestIslandId(islandIndexPerTriangle.begin(), islandIndexPerTriangle.size());

		PxArray<PxU32> connectedTriangleSet;
		for (PxU32 i = 0; i < numTriangles; ++i)
		{
			if (islandIndexPerTriangle[i] == biggestIslandIndex)
			{
				for (PxU32 j = 0; j < 3; ++j)
					connectedTriangleSet.pushBack(inputIndices[3 * i + j]);
				indexMapToFullTriangleSet.pushBack(i);
			}
		}

		ms.init(inputVertices, connectedTriangleSet, edgeLengthCostWeight, flatnessDetectionThreshold, projectSimplifiedPointsOnInputMeshSurface);
		ms.decimateBySize(targetTriangleCount, maximalEdgeLength);
		ms.readBack(outputVertices, outputIndices, vertexMap, outputVertexToInputTriangle);
	}
	else
	{
		ms.init(inputVertices, inputIndices, edgeLengthCostWeight, flatnessDetectionThreshold, projectSimplifiedPointsOnInputMeshSurface);
		ms.decimateBySize(targetTriangleCount, maximalEdgeLength);
		ms.readBack(outputVertices, outputIndices, vertexMap, outputVertexToInputTriangle);	
	}

	if (removeDisconnectedPatches && projectSimplifiedPointsOnInputMeshSurface && outputVertexToInputTriangle)
	{
		for (PxU32 i = 0; i < outputVertexToInputTriangle->size(); ++i)		
			(*outputVertexToInputTriangle)[i] = indexMapToFullTriangleSet[(*outputVertexToInputTriangle)[i]];		
	}
}

void PxTetMaker::remeshTriangleMesh(const PxArray<PxVec3>& inputVertices, const PxArray<PxU32>&inputIndices, PxU32 gridResolution,
	PxArray<PxVec3>& outputVertices, PxArray<PxU32>& outputIndices, PxArray<PxU32> *vertexMap)
{
	Ext::Remesher rm;
	rm.remesh(inputVertices, inputIndices, gridResolution, vertexMap);
	rm.readBack(outputVertices, outputIndices);
}

void PxTetMaker::remeshTriangleMesh(const PxVec3* inputVertices, PxU32 nbVertices, const PxU32* inputIndices, PxU32 nbIndices, PxU32 gridResolution,
	PxArray<PxVec3>& outputVertices, PxArray<PxU32>& outputIndices, PxArray<PxU32> *vertexMap)
{
	Ext::Remesher rm;
	rm.remesh(inputVertices, nbVertices, inputIndices, nbIndices, gridResolution, vertexMap);
	rm.readBack(outputVertices, outputIndices);
}

void PxTetMaker::createTreeBasedTetrahedralMesh(const PxArray<PxVec3>& inputVertices, const PxArray<PxU32>&inputIndices,
	bool useTreeNodes, PxArray<PxVec3>& outputVertices, PxArray<PxU32>& outputIndices, PxReal volumeThreshold)
{
	Ext::OctreeTetrahedralizer ot;
	ot.createTetMesh(inputVertices, inputIndices, useTreeNodes);
	ot.readBack(outputVertices, outputIndices);

	if (volumeThreshold > 0.0f)
		removeSmallVolumeTetrahedra(outputVertices, outputIndices, volumeThreshold);

	PxU32 numRemoveAtEnd = Ext::removeDisconnectedIslands(reinterpret_cast<PxI32*>(outputIndices.begin()), outputIndices.size() / 4);
	if (numRemoveAtEnd > 0)
		outputIndices.removeRange(outputIndices.size() - 4 * numRemoveAtEnd, 4 * numRemoveAtEnd);

	removeUnusedVertices(outputVertices, outputIndices, inputVertices.size());
}

void PxTetMaker::createRelaxedVoxelTetrahedralMesh(const PxArray<PxVec3>& inputVertices, const PxArray<PxU32>&inputIndices,
	PxArray<PxVec3>& outputVertices, PxArray<PxU32>& outputIndices,
	PxI32 resolution, PxI32 numRelaxationIters, PxF32 relMinTetVolume)
{
	Ext::VoxelTetrahedralizer vt;
	vt.createTetMesh(inputVertices, inputIndices, resolution, numRelaxationIters, relMinTetVolume);
	vt.readBack(outputVertices, outputIndices);
}


