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

#include "Cooking.h"
#include "GuCooking.h"
#include "GuBVH.h"

///////////////////////////////////////////////////////////////////////////////

using namespace physx;
using namespace Gu;

///////////////////////////////////////////////////////////////////////////////

void Cooking::setParams(const PxCookingParams& params)
{
	mParams = params;
}

const PxCookingParams& Cooking::getParams() const
{
	return mParams;
}

///////////////////////////////////////////////////////////////////////////////

bool Cooking::platformMismatch() const
{
	return immediateCooking::platformMismatch();
}

///////////////////////////////////////////////////////////////////////////////

void Cooking::release()
{
	PX_DELETE_THIS;

	PxDecFoundationRefCount();
}

///////////////////////////////////////////////////////////////////////////////

bool Cooking::validateTriangleMesh(const PxTriangleMeshDesc& desc) const
{
	return immediateCooking::validateTriangleMesh(mParams, desc);
}

bool Cooking::cookTriangleMesh(const PxTriangleMeshDesc& desc, PxOutputStream& stream, PxTriangleMeshCookingResult::Enum* condition) const
{
	return immediateCooking::cookTriangleMesh(mParams, desc, stream, condition);
}

PxTriangleMesh* Cooking::createTriangleMesh(const PxTriangleMeshDesc& desc, PxInsertionCallback& insertionCallback, PxTriangleMeshCookingResult::Enum* condition) const
{
	return immediateCooking::createTriangleMesh(mParams, desc, insertionCallback, condition);
}

///////////////////////////////////////////////////////////////////////////////

PxTetrahedronMesh* Cooking::createTetrahedronMesh(const PxTetrahedronMeshDesc& meshDesc, PxInsertionCallback& insertionCallback) const
{
	return immediateCooking::createTetrahedronMesh(mParams, meshDesc, insertionCallback);
}

bool Cooking::cookTetrahedronMesh(const PxTetrahedronMeshDesc& meshDesc, PxOutputStream& stream) const
{
	return immediateCooking::cookTetrahedronMesh(mParams, meshDesc, stream);
}

///////////////////////////////////////////////////////////////////////////////

bool Cooking::cookSoftBodyMesh(const PxTetrahedronMeshDesc& simulationMeshDesc, const PxTetrahedronMeshDesc& collisionMeshDesc,
	const PxSoftBodySimulationDataDesc& softbodyDataDesc, PxOutputStream& stream) const
{
	return immediateCooking::cookSoftBodyMesh(mParams, simulationMeshDesc, collisionMeshDesc, softbodyDataDesc, stream);
}

PxCollisionMeshMappingData* Cooking::computeModelsMapping(PxTetrahedronMeshData& simulationMesh, const PxTetrahedronMeshData& collisionMesh, const PxSoftBodyCollisionData& collisionData, const PxBoundedData* vertexToTet) const
{
	return immediateCooking::computeModelsMapping(mParams, simulationMesh, collisionMesh, collisionData, vertexToTet);
}

PxCollisionTetrahedronMeshData* Cooking::computeCollisionData(const PxTetrahedronMeshDesc& collisionMeshDesc) const
{
	return immediateCooking::computeCollisionData(mParams, collisionMeshDesc);
}

PxSimulationTetrahedronMeshData* Cooking::computeSimulationData(const PxTetrahedronMeshDesc& simulationMeshDesc) const
{
	return immediateCooking::computeSimulationData(mParams, simulationMeshDesc);
}

PxSoftBodyMesh* Cooking::assembleSoftBodyMesh(PxTetrahedronMeshData& simulationMesh, PxSoftBodySimulationData& simulationData, PxTetrahedronMeshData& collisionMesh,
	PxSoftBodyCollisionData& collisionData, PxCollisionMeshMappingData& mappingData, PxInsertionCallback& insertionCallback) const
{
	return immediateCooking::assembleSoftBodyMesh(simulationMesh, simulationData, collisionMesh, collisionData, mappingData, insertionCallback);
}

PxSoftBodyMesh* Cooking::assembleSoftBodyMesh(PxSimulationTetrahedronMeshData& simulationMesh, PxCollisionTetrahedronMeshData& collisionMesh, PxCollisionMeshMappingData& mappingData, PxInsertionCallback& insertionCallback) const
{
	return immediateCooking::assembleSoftBodyMesh_Sim(simulationMesh, collisionMesh, mappingData, insertionCallback);
}

PxSoftBodyMesh* Cooking::createSoftBodyMesh(const PxTetrahedronMeshDesc& simulationMeshDesc, const PxTetrahedronMeshDesc& collisionMeshDesc,
	const PxSoftBodySimulationDataDesc& softbodyDataDesc, PxInsertionCallback& insertionCallback) const
{
	return immediateCooking::createSoftBodyMesh(mParams, simulationMeshDesc, collisionMeshDesc, softbodyDataDesc, insertionCallback);
}

///////////////////////////////////////////////////////////////////////////////

// cook convex mesh from given desc, save the results into stream
bool Cooking::cookConvexMesh(const PxConvexMeshDesc& desc, PxOutputStream& stream, PxConvexMeshCookingResult::Enum* condition) const
{	
	return immediateCooking::cookConvexMesh(mParams, desc, stream, condition);
}

// cook convex mesh from given desc, copy the results into internal convex mesh
// and insert the mesh into PxPhysics
PxConvexMesh* Cooking::createConvexMesh(const PxConvexMeshDesc& desc, PxInsertionCallback& insertionCallback, PxConvexMeshCookingResult::Enum* condition) const
{
	return immediateCooking::createConvexMesh(mParams, desc, insertionCallback, condition);
}

bool Cooking::validateConvexMesh(const PxConvexMeshDesc& desc) const
{
	return immediateCooking::validateConvexMesh(mParams, desc);
}

bool Cooking::computeHullPolygons(const PxSimpleTriangleMesh& mesh, PxAllocatorCallback& inCallback,PxU32& nbVerts, PxVec3*& vertices,
									PxU32& nbIndices, PxU32*& indices, PxU32& nbPolygons, PxHullPolygon*& hullPolygons) const
{
	return immediateCooking::computeHullPolygons(mParams, mesh, inCallback, nbVerts, vertices, nbIndices, indices, nbPolygons, hullPolygons);
}

///////////////////////////////////////////////////////////////////////////////

bool Cooking::cookHeightField(const PxHeightFieldDesc& desc, PxOutputStream& stream) const
{
	return immediateCooking::cookHeightField(desc, stream);
}

PxHeightField* Cooking::createHeightField(const PxHeightFieldDesc& desc, PxInsertionCallback& insertionCallback) const
{
	return immediateCooking::createHeightField(desc, insertionCallback);
}

///////////////////////////////////////////////////////////////////////////////

bool Cooking::cookBVH(const PxBVHDesc& desc, PxOutputStream& stream) const
{
	return immediateCooking::cookBVH(desc, stream);
}

PxBVH* Cooking::createBVH(const PxBVHDesc& desc, PxInsertionCallback& insertionCallback) const
{
	return immediateCooking::createBVH(desc, insertionCallback);
}

///////////////////////////////////////////////////////////////////////////////

PxInsertionCallback& Cooking::getStandaloneInsertionCallback()
{
	return *immediateCooking::getInsertionCallback();
}

///////////////////////////////////////////////////////////////////////////////

PxCooking* PxCreateCooking(PxU32 /*version*/, PxFoundation& foundation, const PxCookingParams& params)
{
	PX_ASSERT(&foundation == &PxGetFoundation());
	PX_UNUSED(foundation);

	PxIncFoundationRefCount();

	return PX_NEW(Cooking)(params);
}

///////////////////////////////////////////////////////////////////////////////

// PT: temporary for Kit

#include "cooking/PxCookingInternal.h"
#include "GuTriangleMeshBV4.h"
PxTriangleMesh* Cooking::createTriangleMesh(const PxTriangleMeshInternalData& data) const
{
	TriangleMesh* np;
	PX_NEW_SERIALIZED(np, BV4TriangleMesh)(data);
	return np;
}

PxBVH* Cooking::createBVH(const PxBVHInternalData& data) const
{
	BVH* np;
	PX_NEW_SERIALIZED(np, BVH)(data);
	return np;
}

physx::PxTriangleMesh* PxCreateTriangleMeshInternal(const physx::PxTriangleMeshInternalData& data, const physx::PxCooking& cooking)
{
	return static_cast<const Cooking&>(cooking).createTriangleMesh(data);
}

physx::PxBVH* PxCreateBVHInternal(const physx::PxBVHInternalData& data, const physx::PxCooking& cooking)
{
	return static_cast<const Cooking&>(cooking).createBVH(data);
}

//~ PT: temporary for Kit

///////////////////////////////////////////////////////////////////////////////

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

bool PxCookSoftBodyMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& simulationMeshDesc, const PxTetrahedronMeshDesc& collisionMeshDesc, const PxSoftBodySimulationDataDesc& softbodyDataDesc, PxOutputStream& stream)
{
	return immediateCooking::cookSoftBodyMesh(params, simulationMeshDesc, collisionMeshDesc, softbodyDataDesc, stream);
}

PxSoftBodyMesh* PxCreateSoftBodyMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& simulationMeshDesc, const PxTetrahedronMeshDesc& collisionMeshDesc, const PxSoftBodySimulationDataDesc& softbodyDataDesc, PxInsertionCallback& insertionCallback)
{
	return immediateCooking::createSoftBodyMesh(params, simulationMeshDesc, collisionMeshDesc, softbodyDataDesc, insertionCallback);
}

PxCollisionMeshMappingData* PxComputeModelsMapping(const PxCookingParams& params, PxTetrahedronMeshData& simulationMesh, const PxTetrahedronMeshData& collisionMesh, const PxSoftBodyCollisionData& collisionData, const PxBoundedData* vertexToTet)
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

PxSoftBodyMesh*	PxAssembleSoftBodyMesh(PxTetrahedronMeshData& simulationMesh, PxSoftBodySimulationData& simulationData, PxTetrahedronMeshData& collisionMesh, PxSoftBodyCollisionData& collisionData, PxCollisionMeshMappingData& mappingData, PxInsertionCallback& insertionCallback)
{
	return immediateCooking::assembleSoftBodyMesh(simulationMesh, simulationData, collisionMesh, collisionData, mappingData, insertionCallback);
}
	
PxSoftBodyMesh*	PxAssembleSoftBodyMesh_Sim(PxSimulationTetrahedronMeshData& simulationMesh, PxCollisionTetrahedronMeshData& collisionMesh, PxCollisionMeshMappingData& mappingData, PxInsertionCallback& insertionCallback)
{
	return immediateCooking::assembleSoftBodyMesh_Sim(simulationMesh, collisionMesh, mappingData, insertionCallback);
}

