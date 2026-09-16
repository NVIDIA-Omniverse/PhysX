// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuCooking.h"
#include "GuBVH.h"

using namespace physx;
using namespace Gu;

PxInsertionCallback* PxGetStandaloneInsertionCallback()
{
	return immediateCooking::getInsertionCallback();
}

bool PxCookBVH(const PxBVHDesc& desc, PxOutputStream& stream)
{
	return immediateCooking::cookBVH(desc, stream);
}

PxBVH* PxCreateBVH(const PxBVHDesc& desc, PxInsertionCallback& insertionCallback)
{
	return immediateCooking::createBVH(desc, insertionCallback);
}

bool PxCookHeightField(const PxHeightFieldDesc& desc, PxOutputStream& stream)
{
	return immediateCooking::cookHeightField(desc, stream);
}

PxHeightField* PxCreateHeightField(const PxHeightFieldDesc& desc, PxInsertionCallback& insertionCallback)
{
	return immediateCooking::createHeightField(desc, insertionCallback);
}

bool PxCookConvexMesh(const PxCookingParams& params, const PxConvexMeshDesc& desc, PxOutputStream& stream, PxConvexMeshCookingResult::Enum* condition)
{
	return immediateCooking::cookConvexMesh(params, desc, stream, condition);
}

PxConvexMesh* PxCreateConvexMesh(const PxCookingParams& params, const PxConvexMeshDesc& desc, PxInsertionCallback& insertionCallback, PxConvexMeshCookingResult::Enum* condition)
{
	return immediateCooking::createConvexMesh(params, desc, insertionCallback, condition);
}

bool PxValidateConvexMesh(const PxCookingParams& params, const PxConvexMeshDesc& desc)
{
	return immediateCooking::validateConvexMesh(params, desc);
}

bool PxComputeHullPolygons(const PxCookingParams& params, const PxSimpleTriangleMesh& mesh, PxAllocatorCallback& inCallback, PxU32& nbVerts, PxVec3*& vertices, PxU32& nbIndices, PxU32*& indices, PxU32& nbPolygons, PxHullPolygon*& hullPolygons)
{
	return immediateCooking::computeHullPolygons(params, mesh, inCallback, nbVerts, vertices, nbIndices, indices, nbPolygons, hullPolygons);
}

bool PxValidateTriangleMesh(const PxCookingParams& params, const PxTriangleMeshDesc& desc)
{
	return immediateCooking::validateTriangleMesh(params, desc);
}

PxTriangleMesh* PxCreateTriangleMesh(const PxCookingParams& params, const PxTriangleMeshDesc& desc, PxInsertionCallback& insertionCallback, PxTriangleMeshCookingResult::Enum* condition)
{
	return immediateCooking::createTriangleMesh(params, desc, insertionCallback, condition);
}

bool PxCookTriangleMesh(const PxCookingParams& params, const PxTriangleMeshDesc& desc, PxOutputStream& stream, PxTriangleMeshCookingResult::Enum* condition)
{
	return immediateCooking::cookTriangleMesh(params, desc, stream, condition);
}

bool PxCookTetrahedronMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& meshDesc, PxOutputStream& stream)
{
	return immediateCooking::cookTetrahedronMesh(params, meshDesc, stream);
}

PxTetrahedronMesh* PxCreateTetrahedronMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& meshDesc, PxInsertionCallback& insertionCallback)
{
	return immediateCooking::createTetrahedronMesh(params, meshDesc, insertionCallback);
}

bool PxCookDeformableVolumeMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& simulationMeshDesc, const PxTetrahedronMeshDesc& collisionMeshDesc,
	const PxDeformableVolumeSimulationDataDesc& softbodyDataDesc, PxOutputStream& stream)
{
	return immediateCooking::cookDeformableVolumeMesh(params, simulationMeshDesc, collisionMeshDesc, softbodyDataDesc, stream);
}

PxDeformableVolumeMesh* PxCreateDeformableVolumeMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& simulationMeshDesc,
	const PxTetrahedronMeshDesc& collisionMeshDesc, const PxDeformableVolumeSimulationDataDesc& softbodyDataDesc, PxInsertionCallback& insertionCallback)
{
	return immediateCooking::createDeformableVolumeMesh(params, simulationMeshDesc, collisionMeshDesc, softbodyDataDesc, insertionCallback);
}

PxCollisionMeshMappingData* PxComputeModelsMapping(const PxCookingParams& params, PxTetrahedronMeshData& simulationMesh,
	const PxTetrahedronMeshData& collisionMesh, const PxDeformableVolumeCollisionData& collisionData, const PxBoundedData* vertexToTet)
{
	return immediateCooking::computeModelsMapping(params, simulationMesh, collisionMesh, collisionData, vertexToTet);
}
	
PxCollisionTetrahedronMeshData* PxComputeCollisionData(const PxCookingParams& params, const PxTetrahedronMeshDesc& collisionMeshDesc)
{
	return immediateCooking::computeCollisionData(params, collisionMeshDesc);
}

PxSimulationTetrahedronMeshData* PxComputeSimulationData(const PxCookingParams& params, const PxTetrahedronMeshDesc& simulationMeshDesc)
{
	return immediateCooking::computeSimulationData(params, simulationMeshDesc);
}

PxDeformableVolumeMesh* PxAssembleDeformableVolumeMesh(PxTetrahedronMeshData& simulationMesh, PxDeformableVolumeSimulationData& simulationData,
	PxTetrahedronMeshData& collisionMesh, PxDeformableVolumeCollisionData& collisionData, PxCollisionMeshMappingData& mappingData,
	PxInsertionCallback& insertionCallback)
{
	return immediateCooking::assembleDeformableVolumeMesh(simulationMesh, simulationData, collisionMesh, collisionData, mappingData, insertionCallback);
}

