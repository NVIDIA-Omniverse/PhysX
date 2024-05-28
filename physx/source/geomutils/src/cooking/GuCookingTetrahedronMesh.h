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
								PxSoftBodySimulationDataDesc softbodyDataDesc, Gu::TetrahedronMeshData& simulationMesh, Gu::SoftBodySimulationData& simulationData,
								Gu::TetrahedronMeshData& collisionMesh, Gu::SoftBodyCollisionData& collisionData, Gu::CollisionMeshMappingData& mappingData, const PxCookingParams&	params, bool validateMesh = false);
		static bool	saveTetrahedronMeshData(PxOutputStream& stream, bool platformMismatch, const PxCookingParams& params,
											const Gu::TetrahedronMeshData& mesh);
		static bool	saveSoftBodyMeshData(PxOutputStream& stream, bool platformMismatch, const PxCookingParams& params,
										const Gu::TetrahedronMeshData& simulationMesh, const Gu::SoftBodySimulationData& simulationData, const Gu::TetrahedronMeshData& collisionMesh, 
										const Gu::SoftBodyCollisionData& collisionData, const Gu::CollisionMeshMappingData& mappingData);

		//PxMeshMidPhase::Enum				getMidphaseID()	const { return PxMeshMidPhase::eBVH34; }
		static bool	createMidPhaseStructure(Gu::TetrahedronMeshData& collisionMesh, Gu::SoftBodyCollisionData& collisionData, const PxCookingParams& params);
		static void	saveMidPhaseStructure(PxOutputStream& stream, bool mismatch, const Gu::SoftBodyCollisionData& collisionData);

		static void	computeTetData(const PxTetrahedronMeshDesc& desc, Gu::TetrahedronMeshData& mesh);

		static bool	createGRBMidPhaseAndData(const PxU32 originalTriangleCount, Gu::TetrahedronMeshData& collisionMesh, Gu::SoftBodyCollisionData& collisionData, const PxCookingParams& params);
		static void	computeSimData(const PxTetrahedronMeshDesc& desc, Gu::TetrahedronMeshData& simulationMesh, Gu::SoftBodySimulationData& simulationData, const PxCookingParams& params);
		static void	computeModelsMapping(Gu::TetrahedronMeshData& simulationMesh, const Gu::TetrahedronMeshData& collisionMesh, const Gu::SoftBodyCollisionData& collisionData, 
																	Gu::CollisionMeshMappingData& mappingData, bool buildGPUData, const PxBoundedData* vertexToTet);
		static void	createCollisionModelMapping(const Gu::TetrahedronMeshData& collisionMesh, const Gu::SoftBodyCollisionData& collisionData, Gu::CollisionMeshMappingData& mappingData);
		
		static void	recordTetrahedronIndices(const Gu::TetrahedronMeshData& collisionMesh, Gu::SoftBodyCollisionData& collisionData, bool buildGPUData);
		static bool	importMesh(const PxTetrahedronMeshDesc& collisionMeshDesc, const PxCookingParams& params, 
								Gu::TetrahedronMeshData& collisionMesh, Gu::SoftBodyCollisionData& collisionData, bool validate = false);
		
		static bool	computeCollisionData(const PxTetrahedronMeshDesc& collisionMeshDesc, Gu::TetrahedronMeshData& collisionMesh, Gu::SoftBodyCollisionData& collisionData,
										const PxCookingParams&	params, bool validateMesh = false);
	};

	class BV32TetrahedronMeshBuilder
	{
	public:
		static	bool	createMidPhaseStructure(const PxCookingParams& params, Gu::TetrahedronMeshData& meshData, Gu::BV32Tree& bv32Tree, Gu::SoftBodyCollisionData& collisionData);
		static	void	saveMidPhaseStructure(Gu::BV32Tree* tree, PxOutputStream& stream, bool mismatch);
	};
}

#endif
