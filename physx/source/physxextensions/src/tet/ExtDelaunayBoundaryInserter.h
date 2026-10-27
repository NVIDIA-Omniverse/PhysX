// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_DELAUNAY_BOUNDARY_INSERTER_H
#define EXT_DELAUNAY_BOUNDARY_INSERTER_H

#include "foundation/PxHashSet.h"
#include "foundation/PxVec3.h"
#include "foundation/PxMat33.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxArray.h"
#include "common/PxCoreUtilityTypes.h"
#include "extensions/PxTriangleMeshAnalysisResult.h"
#include "extensions/PxTetrahedronMeshAnalysisResult.h"

#include "ExtDelaunayTetrahedralizer.h"
namespace physx
{
namespace Ext
{
	//Generates a tetmesh that matches the specified triangle mesh. All triangle mesh points are present in the tetmesh, additional points on edges and faces might be present.
	void generateTetmesh(const PxBoundedData& inputPoints, const PxBoundedData& inputTriangles, const bool has16bitIndices, PxArray<PxVec3>& tetPoints, PxArray<PxU32>& finalTets);

	//Generates a tetmesh that that matches the surface of the input tetmesh approximately but creats very regular shaped tetrahedra.
	void generateVoxelTetmesh(const PxBoundedData& inputPoints, const PxBoundedData& inputTets, PxU32 numVoxelsX, PxU32 numVoxelsY, PxU32 numVoxelsZ,
		PxArray<PxVec3>& voxelPoints, PxArray<PxU32>& voxelTets, PxI32* intputPointToOutputTetIndex, const PxU32* anchorNodeIndices = NULL, PxU32 numTetsPerVoxel = 5,
		bool avoidVoxelDuplication = false);

	//Generates a tetmesh that that matches the surface of the input tetmesh approximately but creats very regular shaped tetrahedra.
	void generateVoxelTetmesh(const PxBoundedData& inputPoints, const PxBoundedData& inputTets, PxReal voxelEdgeLength,
		PxArray<PxVec3>& voxelPoints, PxArray<PxU32>& voxelTets, PxI32* intputPointToOutputTetIndex, const PxU32* anchorNodeIndices = NULL, PxU32 numTetsPerVoxel = 5,
		bool avoidVoxelDuplication = false);

	//Generates a tetmesh that that matches the surface of the input tetmesh approximately but creats very regular shaped tetrahedra.
	void generateVoxelTetmesh(const PxBoundedData& inputPoints, const PxBoundedData& inputTets, PxU32 numVoxelsAlongLongestBoundingBoxAxis,
		PxArray<PxVec3>& voxelPoints, PxArray<PxU32>& voxelTets, PxI32* intputPointToOutputTetIndex, const PxU32* anchorNodeIndices = NULL, PxU32 numTetsPerVoxel = 5,
		bool avoidVoxelDuplication = false);

	//Extracts the surface triangles from the specified tetrahedra
	void extractTetmeshSurface(const PxArray<PxI32>& tets, PxArray<PxI32>& triangles);

	//Computes the lumped mass per vertex for the specified tetmesh
	void pointMasses(const PxArray<PxVec3>& tetVerts, const PxArray<PxU32>& tets, PxF32 density, PxArray<PxF32>& mass);

	//Computes a rest pose matrix for every tetrahedron in the specified tetmesh
	void restPoses(const PxArray<PxVec3>& tetVerts, const PxArray<PxU32>& tets, PxArray<PxMat33>& restPoses);

	//Computes a fiber direction for every tetrahedron in the specified tetmesh. Currently just returns dummy values.
	void tetFibers(const PxArray<PxVec3>& tetVerts, const PxArray<PxU32>& tets, PxArray<PxVec3>& tetFibers);

	//Analyzes the triangle mesh to get a report about deficiencies. Some deficiencies can be handled by the tetmesher, others cannot.
	PxTriangleMeshAnalysisResults validateTriangleMesh(const PxBoundedData& points, const PxBoundedData& triangles, const bool has16BitIndices, const PxReal minVolumeThreshold = 1e-6f, const PxReal minTriangleAngleRadians = 10.0f*3.1415926535898f / 180.0f);
	
	//Analyzes the tetrahedron mesh to get a report about deficiencies. Some deficiencies can be handled by the softbody cooker, others cannot.
	PxTetrahedronMeshAnalysisResults validateTetrahedronMesh(const PxBoundedData& points, const PxBoundedData& tetrahedra, const bool has16BitIndices, const PxReal minTetVolumeThreshold = 1e-8f);

	PxU32 removeDisconnectedIslands(PxI32* finalTets, PxU32 numTets);
}
}

#endif

