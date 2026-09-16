// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_COOKING_TETRAHEDRON_MESH_H
#define GU_COOKING_TETRAHEDRON_MESH_H

#include "cooking/PxCooking.h"
#include "GuMeshData.h"

namespace physx
{
	class TetrahedronMeshBuilder
	{
		PX_NOCOPY(TetrahedronMeshBuilder)
	public:

		static bool	loadFromDesc(const PxTetrahedronMeshDesc& simulationMeshDesc, const PxTetrahedronMeshDesc& collisionMeshDesc,
			PxDeformableVolumeSimulationDataDesc deformableVolumeDataDesc, Gu::TetrahedronMeshData& simulationMesh, Gu::DeformableVolumeSimulationData& simulationData,
			Gu::TetrahedronMeshData& collisionMesh, Gu::DeformableVolumeCollisionData& collisionData, Gu::CollisionMeshMappingData& mappingData,
			const PxCookingParams&	params, bool validateMesh = false);

		static bool	saveTetrahedronMeshData(PxOutputStream& stream, bool platformMismatch, const PxCookingParams& params,
			const Gu::TetrahedronMeshData& mesh);

		static bool	saveDeformableVolumeMeshData(PxOutputStream& stream, bool platformMismatch, const PxCookingParams& params,
			const Gu::TetrahedronMeshData& simulationMesh, const Gu::DeformableVolumeSimulationData& simulationData,
			const Gu::TetrahedronMeshData& collisionMesh, const Gu::DeformableVolumeCollisionData& collisionData,
			const Gu::CollisionMeshMappingData& mappingData);

		//PxMeshMidPhase::Enum				getMidphaseID()	const { return PxMeshMidPhase::eBVH34; }
		static bool	createMidPhaseStructure(Gu::TetrahedronMeshData& collisionMesh, Gu::DeformableVolumeCollisionData& collisionData, const PxCookingParams& params);
		static void	saveMidPhaseStructure(PxOutputStream& stream, bool mismatch, const Gu::DeformableVolumeCollisionData& collisionData);

		static void	computeTetData(const PxTetrahedronMeshDesc& desc, Gu::TetrahedronMeshData& mesh);

		static bool	createGRBMidPhaseAndData(const PxU32 originalTriangleCount, Gu::TetrahedronMeshData& collisionMesh, Gu::DeformableVolumeCollisionData& collisionData, const PxCookingParams& params);
		static void	computeSimData(const PxTetrahedronMeshDesc& desc, Gu::TetrahedronMeshData& simulationMesh, Gu::DeformableVolumeSimulationData& simulationData, const PxCookingParams& params);
		static bool	computeModelsMapping(Gu::TetrahedronMeshData& simulationMesh, const Gu::TetrahedronMeshData& collisionMesh, const Gu::DeformableVolumeCollisionData& collisionData,
																	Gu::CollisionMeshMappingData& mappingData, bool buildGPUData, const PxBoundedData* vertexToTet);
		static bool	createCollisionModelMapping(const Gu::TetrahedronMeshData& collisionMesh, const Gu::DeformableVolumeCollisionData& collisionData, Gu::CollisionMeshMappingData& mappingData);
		
		static void	recordTetrahedronIndices(const Gu::TetrahedronMeshData& collisionMesh, Gu::DeformableVolumeCollisionData& collisionData, bool buildGPUData);
		static bool	importMesh(const PxTetrahedronMeshDesc& collisionMeshDesc, const PxCookingParams& params, 
								Gu::TetrahedronMeshData& collisionMesh, Gu::DeformableVolumeCollisionData& collisionData, bool validate = false);
		
		static bool	computeCollisionData(const PxTetrahedronMeshDesc& collisionMeshDesc, Gu::TetrahedronMeshData& collisionMesh, Gu::DeformableVolumeCollisionData& collisionData,
										const PxCookingParams&	params, bool validateMesh = false);
	};

	class BV32TetrahedronMeshBuilder
	{
	public:
		static	bool	createMidPhaseStructure(const PxCookingParams& params, Gu::TetrahedronMeshData& meshData, Gu::BV32Tree& bv32Tree, Gu::DeformableVolumeCollisionData& collisionData);
		static	void	saveMidPhaseStructure(Gu::BV32Tree* tree, PxOutputStream& stream, bool mismatch);
	};
}

#endif
