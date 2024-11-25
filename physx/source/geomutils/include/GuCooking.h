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

#ifndef GU_COOKING_H
#define GU_COOKING_H

// PT: TODO: the SDK always had this questionable design decision that all APIs can include all high-level public headers,
// regardless of where they fit in the header hierarchy. For example PhysXCommon can include headers from the higher-level
// PhysX DLL. We take advantage of that here by including PxCooking from PhysXCommon. That way we can reuse the same code
// as before without decoupling it from high-level classes like PxConvexMeshDesc/etc. A cleaner solution would be to decouple
// the two and only use PxConvexMeshDesc/etc in the higher level cooking DLL. The lower-level Gu functions below would then
// operate either on Gu-level types (see e.g. PxBVH / GuBVH which was done this way), or on basic types like float and ints
// to pass vertex & triangle data around. We could also split the kitchen-sink PxCookingParams structure into separate classes
// for convex / triangle mesh / etc. Overall there might be some more refactoring to do here, and that's why these functions
// have been put in the "semi public" Gu API for now, instead of the Px API (which is more strict in terms of backward
// compatibility and how we deal with deprecated functions).
#include "cooking/PxCooking.h"

#include "common/PxPhysXCommonConfig.h"
#include "foundation/PxUtilities.h"
#include "foundation/PxMemory.h"

namespace physx
{
	class PxInsertionCallback;
	class PxOutputStream;
	class PxBVHDesc;
	class PxBVH;
	class PxHeightField;
	struct PxCookingParams;

	namespace immediateCooking
	{
		PX_FORCE_INLINE static void	gatherStrided(const void* src, void* dst, PxU32 nbElem, PxU32 elemSize, PxU32 stride)
		{
			const PxU8* s = reinterpret_cast<const PxU8*>(src);
			PxU8* d = reinterpret_cast<PxU8*>(dst);
			while(nbElem--)
			{
				PxMemCopy(d, s, elemSize);
				d += elemSize;
				s += stride;
			}
		}

		PX_INLINE static bool platformMismatch()
		{
			// Get current endianness (the one for the platform where cooking is performed)
			const PxI8 currentEndian = PxLittleEndian();

			const bool mismatch = currentEndian!=1;	// The files must be little endian - we don't have big endian platforms anymore.
			return mismatch;
		}

		PX_C_EXPORT PX_PHYSX_COMMON_API	PxInsertionCallback* getInsertionCallback();	// PT: should be a reference but using a pointer for C

		// BVH
		PX_C_EXPORT PX_PHYSX_COMMON_API	bool cookBVH(const PxBVHDesc& desc, PxOutputStream& stream);
		PX_C_EXPORT PX_PHYSX_COMMON_API	PxBVH* createBVH(const PxBVHDesc& desc, PxInsertionCallback& insertionCallback);

		PX_FORCE_INLINE	PxBVH* createBVH(const PxBVHDesc& desc)
		{
			return createBVH(desc, *getInsertionCallback());
		}

		// Heightfield
		PX_C_EXPORT PX_PHYSX_COMMON_API	bool cookHeightField(const PxHeightFieldDesc& desc, PxOutputStream& stream);
		PX_C_EXPORT PX_PHYSX_COMMON_API	PxHeightField* createHeightField(const PxHeightFieldDesc& desc, PxInsertionCallback& insertionCallback);

		PX_FORCE_INLINE	PxHeightField* createHeightField(const PxHeightFieldDesc& desc)
		{
			return createHeightField(desc, *getInsertionCallback());
		}

		// Convex meshes
		PX_C_EXPORT PX_PHYSX_COMMON_API	bool cookConvexMesh(const PxCookingParams& params, const PxConvexMeshDesc& desc, PxOutputStream& stream, PxConvexMeshCookingResult::Enum* condition=NULL);
		PX_C_EXPORT PX_PHYSX_COMMON_API	PxConvexMesh* createConvexMesh(const PxCookingParams& params, const PxConvexMeshDesc& desc, PxInsertionCallback& insertionCallback, PxConvexMeshCookingResult::Enum* condition=NULL);

		PX_FORCE_INLINE	PxConvexMesh* createConvexMesh(const PxCookingParams& params, const PxConvexMeshDesc& desc)
		{
			return createConvexMesh(params, desc, *getInsertionCallback());
		}

		PX_C_EXPORT PX_PHYSX_COMMON_API	bool validateConvexMesh(const PxCookingParams& params, const PxConvexMeshDesc& desc);
		PX_C_EXPORT PX_PHYSX_COMMON_API	bool computeHullPolygons(const PxCookingParams& params, const PxSimpleTriangleMesh& mesh, PxAllocatorCallback& inCallback, PxU32& nbVerts, PxVec3*& vertices,
																PxU32& nbIndices, PxU32*& indices, PxU32& nbPolygons, PxHullPolygon*& hullPolygons);

		// Triangle meshes
		PX_C_EXPORT PX_PHYSX_COMMON_API	bool validateTriangleMesh(const PxCookingParams& params, const PxTriangleMeshDesc& desc);
		PX_C_EXPORT PX_PHYSX_COMMON_API	PxTriangleMesh* createTriangleMesh(const PxCookingParams& params, const PxTriangleMeshDesc& desc, PxInsertionCallback& insertionCallback, PxTriangleMeshCookingResult::Enum* condition=NULL);
		PX_C_EXPORT PX_PHYSX_COMMON_API	bool cookTriangleMesh(const PxCookingParams& params, const PxTriangleMeshDesc& desc, PxOutputStream& stream, PxTriangleMeshCookingResult::Enum* condition=NULL);
		
		PX_FORCE_INLINE	PxTriangleMesh*	createTriangleMesh(const PxCookingParams& params, const PxTriangleMeshDesc& desc)
		{
			return createTriangleMesh(params, desc, *getInsertionCallback());
		}

		// Tetrahedron & deformable volume meshes
		PX_C_EXPORT PX_PHYSX_COMMON_API	bool cookTetrahedronMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& meshDesc, PxOutputStream& stream);
		PX_C_EXPORT PX_PHYSX_COMMON_API	PxTetrahedronMesh* createTetrahedronMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& meshDesc, PxInsertionCallback& insertionCallback);

		PX_FORCE_INLINE	PxTetrahedronMesh*	createTetrahedronMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& meshDesc)
		{
			return createTetrahedronMesh(params, meshDesc, *getInsertionCallback());
		}

		PX_C_EXPORT PX_PHYSX_COMMON_API	bool cookDeformableVolumeMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& simulationMeshDesc, const PxTetrahedronMeshDesc& collisionMeshDesc,
																const PxDeformableVolumeSimulationDataDesc& softbodyDataDesc, PxOutputStream& stream);

		PX_C_EXPORT PX_PHYSX_COMMON_API	PxDeformableVolumeMesh* createDeformableVolumeMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& simulationMeshDesc, const PxTetrahedronMeshDesc& collisionMeshDesc,
																		const PxDeformableVolumeSimulationDataDesc& softbodyDataDesc, PxInsertionCallback& insertionCallback);

		PX_FORCE_INLINE	PxDeformableVolumeMesh*	createDeformableVolumeMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& simulationMeshDesc, const PxTetrahedronMeshDesc& collisionMeshDesc,
			const PxDeformableVolumeSimulationDataDesc& deformableVolumeDataDesc)
		{
			return createDeformableVolumeMesh(params, simulationMeshDesc, collisionMeshDesc, deformableVolumeDataDesc, *getInsertionCallback());
		}

		PX_C_EXPORT PX_PHYSX_COMMON_API	PxCollisionMeshMappingData* computeModelsMapping(const PxCookingParams& params, 
			PxTetrahedronMeshData& simulationMesh, const PxTetrahedronMeshData& collisionMesh, 
			const PxDeformableVolumeCollisionData& collisionData, const PxBoundedData* vertexToTet = NULL);
	
		PX_C_EXPORT PX_PHYSX_COMMON_API	PxCollisionTetrahedronMeshData* computeCollisionData(const PxCookingParams& params, const PxTetrahedronMeshDesc& collisionMeshDesc);

		PX_C_EXPORT PX_PHYSX_COMMON_API	PxSimulationTetrahedronMeshData* computeSimulationData(const PxCookingParams& params, const PxTetrahedronMeshDesc& simulationMeshDesc);

		PX_C_EXPORT PX_PHYSX_COMMON_API	PxDeformableVolumeMesh*	assembleDeformableVolumeMesh(PxTetrahedronMeshData& simulationMesh,
			PxDeformableVolumeSimulationData& simulationData, PxTetrahedronMeshData& collisionMesh, PxDeformableVolumeCollisionData& collisionData,
			PxCollisionMeshMappingData& mappingData, PxInsertionCallback& insertionCallback);
	}
}

#endif
