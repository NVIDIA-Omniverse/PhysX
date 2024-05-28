## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##  * Redistributions of source code must retain the above copyright
##    notice, this list of conditions and the following disclaimer.
##  * Redistributions in binary form must reproduce the above copyright
##    notice, this list of conditions and the following disclaimer in the
##    documentation and/or other materials provided with the distribution.
##  * Neither the name of NVIDIA CORPORATION nor the names of its
##    contributors may be used to endorse or promote products derived
##    from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
## EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
## PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
## CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
## EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
## PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
## PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
## OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##
## Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.

#
# Build PhysX (PROJECT not SOLUTION) common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(PX_SOURCE_DIR ${PHYSX_SOURCE_DIR}/physx/src)
SET(MD_SOURCE_DIR ${PHYSX_SOURCE_DIR}/physxmetadata)

SET(PHYSX_PLATFORM_LINK_FLAGS " ")
SET(PHYSX_PLATFORM_LINK_FLAGS_DEBUG " ")
SET(PHYSX_PLATFORM_LINK_FLAGS_CHECKED " ")
SET(PHYSX_PLATFORM_LINK_FLAGS_PROFILE " ")
SET(PHYSX_PLATFORM_LINK_FLAGS_RELEASE " ")

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysX.cmake)

SET(PHYSX_HEADERS
	${PHYSX_ROOT_DIR}/include/PxActor.h
	${PHYSX_ROOT_DIR}/include/PxActorData.h
	${PHYSX_ROOT_DIR}/include/PxAggregate.h
	${PHYSX_ROOT_DIR}/include/PxArticulationFlag.h
	${PHYSX_ROOT_DIR}/include/PxArticulationJointReducedCoordinate.h
	${PHYSX_ROOT_DIR}/include/PxArticulationLink.h
	${PHYSX_ROOT_DIR}/include/PxArticulationReducedCoordinate.h
	${PHYSX_ROOT_DIR}/include/PxArticulationTendon.h
	${PHYSX_ROOT_DIR}/include/PxArticulationTendonData.h
	${PHYSX_ROOT_DIR}/include/PxArticulationMimicJoint.h
	${PHYSX_ROOT_DIR}/include/PxAttachment.h
	${PHYSX_ROOT_DIR}/include/PxBroadPhase.h
	${PHYSX_ROOT_DIR}/include/PxClient.h
	${PHYSX_ROOT_DIR}/include/PxConeLimitedConstraint.h
	${PHYSX_ROOT_DIR}/include/PxConstraint.h
	${PHYSX_ROOT_DIR}/include/PxConstraintDesc.h
	${PHYSX_ROOT_DIR}/include/PxContact.h
	${PHYSX_ROOT_DIR}/include/PxContactModifyCallback.h
	${PHYSX_ROOT_DIR}/include/PxDeletionListener.h
	${PHYSX_ROOT_DIR}/include/PxFEMParameter.h
	${PHYSX_ROOT_DIR}/include/PxFEMClothFlags.h
	${PHYSX_ROOT_DIR}/include/PxFiltering.h
	${PHYSX_ROOT_DIR}/include/PxForceMode.h
	${PHYSX_ROOT_DIR}/include/PxHairSystemFlag.h
	${PHYSX_ROOT_DIR}/include/PxImmediateMode.h
	${PHYSX_ROOT_DIR}/include/PxLockedData.h
	${PHYSX_ROOT_DIR}/include/PxNodeIndex.h
	${PHYSX_ROOT_DIR}/include/PxParticleBuffer.h
	${PHYSX_ROOT_DIR}/include/PxParticleGpu.h
	${PHYSX_ROOT_DIR}/include/PxParticleSolverType.h
	${PHYSX_ROOT_DIR}/include/PxParticleSystem.h
	${PHYSX_ROOT_DIR}/include/PxParticleSystemFlag.h
	${PHYSX_ROOT_DIR}/include/PxPBDParticleSystem.h
	${PHYSX_ROOT_DIR}/include/PxPhysics.h
	${PHYSX_ROOT_DIR}/include/PxPhysicsAPI.h
	${PHYSX_ROOT_DIR}/include/PxPhysicsSerialization.h
	${PHYSX_ROOT_DIR}/include/PxPhysXConfig.h
	${PHYSX_ROOT_DIR}/include/PxPruningStructure.h
	${PHYSX_ROOT_DIR}/include/PxQueryFiltering.h
	${PHYSX_ROOT_DIR}/include/PxQueryReport.h
	${PHYSX_ROOT_DIR}/include/PxRigidActor.h
	${PHYSX_ROOT_DIR}/include/PxRigidBody.h
	${PHYSX_ROOT_DIR}/include/PxRigidDynamic.h
	${PHYSX_ROOT_DIR}/include/PxRigidStatic.h
	${PHYSX_ROOT_DIR}/include/PxScene.h
	${PHYSX_ROOT_DIR}/include/PxSceneDesc.h
	${PHYSX_ROOT_DIR}/include/PxSceneLock.h
	${PHYSX_ROOT_DIR}/include/PxSceneQueryDesc.h
	${PHYSX_ROOT_DIR}/include/PxSceneQuerySystem.h
	${PHYSX_ROOT_DIR}/include/PxShape.h
	${PHYSX_ROOT_DIR}/include/PxSimulationEventCallback.h
	${PHYSX_ROOT_DIR}/include/PxSimulationStatistics.h
	${PHYSX_ROOT_DIR}/include/PxSoftBody.h
	${PHYSX_ROOT_DIR}/include/PxSoftBodyFlag.h
	${PHYSX_ROOT_DIR}/include/PxSparseGridParams.h
	${PHYSX_ROOT_DIR}/include/PxVisualizationParameter.h
    ${PHYSX_ROOT_DIR}/include/PxIsosurfaceExtraction.h
    ${PHYSX_ROOT_DIR}/include/PxSmoothing.h
    ${PHYSX_ROOT_DIR}/include/PxAnisotropy.h
    ${PHYSX_ROOT_DIR}/include/PxParticleNeighborhoodProvider.h
    ${PHYSX_ROOT_DIR}/include/PxArrayConverter.h
    ${PHYSX_ROOT_DIR}/include/PxLineStripSkinning.h
	${PHYSX_ROOT_DIR}/include/PxSDFBuilder.h
	${PHYSX_ROOT_DIR}/include/PxResidual.h
	${PHYSX_ROOT_DIR}/include/PxDirectGPUAPI.h
)
IF(NOT PX_GENERATE_SOURCE_DISTRO AND NOT PUBLIC_RELEASE)
	LIST(APPEND PHYSX_HEADERS
		${PHYSX_ROOT_DIR}/include/PxFEMCloth.h
		${PHYSX_ROOT_DIR}/include/PxHairSystem.h
	)
ENDIF()
SOURCE_GROUP(include FILES ${PHYSX_HEADERS})

SET(PHYSX_MATERIAL_HEADERS
	${PHYSX_ROOT_DIR}/include/PxBaseMaterial.h
	${PHYSX_ROOT_DIR}/include/PxFEMMaterial.h
	${PHYSX_ROOT_DIR}/include/PxFEMSoftBodyMaterial.h
	${PHYSX_ROOT_DIR}/include/PxFEMClothMaterial.h
	${PHYSX_ROOT_DIR}/include/PxParticleMaterial.h
	${PHYSX_ROOT_DIR}/include/PxPBDMaterial.h
	${PHYSX_ROOT_DIR}/include/PxMaterial.h
)
SOURCE_GROUP(include\\materials FILES ${PHYSX_MATERIAL_HEADERS})

SET(PHYSX_COMMON_HEADERS
	${PHYSX_ROOT_DIR}/include/common/PxBase.h
	${PHYSX_ROOT_DIR}/include/common/PxCollection.h
	${PHYSX_ROOT_DIR}/include/common/PxCoreUtilityTypes.h
	${PHYSX_ROOT_DIR}/include/common/PxInsertionCallback.h
	${PHYSX_ROOT_DIR}/include/common/PxMetaData.h
	${PHYSX_ROOT_DIR}/include/common/PxMetaDataFlags.h
	${PHYSX_ROOT_DIR}/include/common/PxPhysXCommonConfig.h
	${PHYSX_ROOT_DIR}/include/common/PxProfileZone.h
	${PHYSX_ROOT_DIR}/include/common/PxRenderBuffer.h
	${PHYSX_ROOT_DIR}/include/common/PxRenderOutput.h
	${PHYSX_ROOT_DIR}/include/common/PxSerialFramework.h
	${PHYSX_ROOT_DIR}/include/common/PxSerializer.h
	${PHYSX_ROOT_DIR}/include/common/PxStringTable.h
	${PHYSX_ROOT_DIR}/include/common/PxTolerancesScale.h
	${PHYSX_ROOT_DIR}/include/common/PxTypeInfo.h
)
SOURCE_GROUP(include\\common FILES ${PHYSX_COMMON_HEADERS})

SET(PHYSX_OMNIPVD_HEADERS
	${PHYSX_ROOT_DIR}/include/omnipvd/PxOmniPvd.h
)
SOURCE_GROUP(include\\omnipvd FILES ${PHYSX_OMNIPVD_HEADERS})

SET(PHYSX_PVD_HEADERS
	${PHYSX_ROOT_DIR}/include/pvd/PxPvdSceneClient.h
	${PHYSX_ROOT_DIR}/include/pvd/PxPvd.h
	${PHYSX_ROOT_DIR}/include/pvd/PxPvdTransport.h
)
SOURCE_GROUP(include\\pvd FILES ${PHYSX_PVD_HEADERS})

SET(PHYSX_COLLISION_HEADERS
	${PHYSX_ROOT_DIR}/include/collision/PxCollisionDefs.h
)
SOURCE_GROUP(include\\collision FILES ${PHYSX_COLLISION_HEADERS})

SET(PHYSX_SOLVER_HEADERS
	${PHYSX_ROOT_DIR}/include/solver/PxSolverDefs.h
)
SOURCE_GROUP(include\\solver FILES ${PHYSX_SOLVER_HEADERS})

SET(PHYSX_METADATA_HEADERS
	${MD_SOURCE_DIR}/core/include/PvdMetaDataDefineProperties.h
	${MD_SOURCE_DIR}/core/include/PvdMetaDataExtensions.h
	${MD_SOURCE_DIR}/core/include/PvdMetaDataPropertyVisitor.h
	${MD_SOURCE_DIR}/core/include/PxAutoGeneratedMetaDataObjectNames.h
	${MD_SOURCE_DIR}/core/include/PxAutoGeneratedMetaDataObjects.h
	${MD_SOURCE_DIR}/core/include/PxMetaDataCompare.h
	${MD_SOURCE_DIR}/core/include/PxMetaDataCppPrefix.h
	${MD_SOURCE_DIR}/core/include/PxMetaDataObjects.h
	${MD_SOURCE_DIR}/core/include/RepXMetaDataPropertyVisitor.h
)
SOURCE_GROUP(metadata\\include FILES ${PHYSX_METADATA_HEADERS})

SET(PHYSX_METADATA_SOURCE
	${MD_SOURCE_DIR}/core/src/PxAutoGeneratedMetaDataObjects.cpp
	${MD_SOURCE_DIR}/core/src/PxMetaDataObjects.cpp
)
SOURCE_GROUP(metadata\\src FILES ${PHYSX_METADATA_SOURCE})

SET(PHYSX_OMNIPVD_SOURCE
	${PX_SOURCE_DIR}/omnipvd/NpOmniPvd.h
	${PX_SOURCE_DIR}/omnipvd/NpOmniPvd.cpp
    ${PX_SOURCE_DIR}/omnipvd/NpOmniPvdRegistrationData.h
	${PX_SOURCE_DIR}/omnipvd/NpOmniPvdRegistrationData.cpp
    ${PX_SOURCE_DIR}/omnipvd/NpOmniPvdSetData.h
	${PX_SOURCE_DIR}/omnipvd/NpOmniPvdMetaData.h
	${PX_SOURCE_DIR}/omnipvd/NpOmniPvdMetaData.cpp
	${PX_SOURCE_DIR}/omnipvd/OmniPvdPxSampler.cpp
	${PX_SOURCE_DIR}/omnipvd/OmniPvdPxSampler.h
	${PX_SOURCE_DIR}/omnipvd/OmniPvdChunkAlloc.cpp
	${PX_SOURCE_DIR}/omnipvd/OmniPvdChunkAlloc.h
	${PX_SOURCE_DIR}/omnipvd/OmniPvdTypes.h
)
SOURCE_GROUP(src\\omnipvd FILES ${PHYSX_OMNIPVD_SOURCE})

SET(PHYSX_PVD_SOURCE
	${PX_SOURCE_DIR}/NpPvdSceneClient.cpp
	${PX_SOURCE_DIR}/NpPvdSceneClient.h
	${PX_SOURCE_DIR}/NpPvdSceneQueryCollector.cpp
	${PX_SOURCE_DIR}/NpPvdSceneQueryCollector.h
	${PX_SOURCE_DIR}/PvdMetaDataPvdBinding.cpp
	${PX_SOURCE_DIR}/PvdPhysicsClient.cpp
	${PX_SOURCE_DIR}/PvdMetaDataBindingData.h
	${PX_SOURCE_DIR}/PvdMetaDataPvdBinding.h
	${PX_SOURCE_DIR}/PvdPhysicsClient.h
	${PX_SOURCE_DIR}/PvdTypeNames.h
)
SOURCE_GROUP(src\\pvd FILES ${PHYSX_PVD_SOURCE})

SET(PHYSX_IMMEDIATEMODE_SOURCE
	${PHYSX_ROOT_DIR}/source/immediatemode/src/NpImmediateMode.cpp
)
SOURCE_GROUP(src\\immediatemode FILES ${PHYSX_IMMEDIATEMODE_SOURCE})

SET(PHYSX_MATERIALS_SOURCE
	${PX_SOURCE_DIR}/NpMaterial.cpp
	${PX_SOURCE_DIR}/NpFEMSoftBodyMaterial.cpp
	${PX_SOURCE_DIR}/NpFEMClothMaterial.cpp
	${PX_SOURCE_DIR}/NpPBDMaterial.cpp
	${PX_SOURCE_DIR}/NpPBDMaterial.h
	${PX_SOURCE_DIR}/NpFEMSoftBodyMaterial.h
	${PX_SOURCE_DIR}/NpFEMClothMaterial.h
	${PX_SOURCE_DIR}/NpMaterial.h
)
SOURCE_GROUP(src\\materials FILES ${PHYSX_MATERIALS_SOURCE})

SET(PHYSX_ARTICULATIONS_SOURCE
	${PX_SOURCE_DIR}/NpArticulationReducedCoordinate.cpp
	${PX_SOURCE_DIR}/NpArticulationJointReducedCoordinate.cpp
	${PX_SOURCE_DIR}/NpArticulationLink.cpp
	${PX_SOURCE_DIR}/NpArticulationTendon.cpp
	${PX_SOURCE_DIR}/NpArticulationMimicJoint.cpp
	${PX_SOURCE_DIR}/NpArticulationReducedCoordinate.h
	${PX_SOURCE_DIR}/NpArticulationJointReducedCoordinate.h
	${PX_SOURCE_DIR}/NpArticulationLink.h
	${PX_SOURCE_DIR}/NpArticulationTendon.h
	${PX_SOURCE_DIR}/NpArticulationMimicJoint.h
)
SOURCE_GROUP(src\\articulations FILES ${PHYSX_ARTICULATIONS_SOURCE})

SET(PHYSX_CORE_SOURCE
	${PX_SOURCE_DIR}/NpActor.cpp
	${PX_SOURCE_DIR}/NpAggregate.cpp
	${PX_SOURCE_DIR}/NpSoftBody.cpp
	${PX_SOURCE_DIR}/NpFEMCloth.cpp
	${PX_SOURCE_DIR}/NpPBDParticleSystem.cpp
	${PX_SOURCE_DIR}/NpParticleBuffer.cpp
	${PX_SOURCE_DIR}/NpHairSystem.cpp
	${PX_SOURCE_DIR}/NpConstraint.cpp
	${PX_SOURCE_DIR}/NpFactory.cpp
	${PX_SOURCE_DIR}/NpMetaData.cpp
	${PX_SOURCE_DIR}/NpPhysics.cpp
	${PX_SOURCE_DIR}/NpBounds.h
	${PX_SOURCE_DIR}/NpBounds.cpp
	${PX_SOURCE_DIR}/NpPruningStructure.h
	${PX_SOURCE_DIR}/NpPruningStructure.cpp
	${PX_SOURCE_DIR}/NpCheck.cpp
	${PX_SOURCE_DIR}/NpRigidDynamic.cpp
	${PX_SOURCE_DIR}/NpRigidStatic.cpp
	${PX_SOURCE_DIR}/NpScene.cpp
	${PX_SOURCE_DIR}/NpSceneFetchResults.cpp
	${PX_SOURCE_DIR}/NpSceneQueries.cpp
	${PX_SOURCE_DIR}/NpSerializerAdapter.cpp
	${PX_SOURCE_DIR}/NpShape.cpp
	${PX_SOURCE_DIR}/NpShapeManager.cpp
	${PX_SOURCE_DIR}/NpBase.h
	${PX_SOURCE_DIR}/NpActor.h
	${PX_SOURCE_DIR}/NpActorTemplate.h
	${PX_SOURCE_DIR}/NpAggregate.h
	${PX_SOURCE_DIR}/NpSoftBody.h
	${PX_SOURCE_DIR}/NpFEMCloth.h
	${PX_SOURCE_DIR}/NpPBDParticleSystem.h
	${PX_SOURCE_DIR}/NpParticleBuffer.h
	${PX_SOURCE_DIR}/NpHairSystem.h
	${PX_SOURCE_DIR}/NpConnector.h
	${PX_SOURCE_DIR}/NpConstraint.h
	${PX_SOURCE_DIR}/NpFactory.h
	${PX_SOURCE_DIR}/NpMaterialManager.h
	${PX_SOURCE_DIR}/NpPhysics.h
	${PX_SOURCE_DIR}/NpPhysicsInsertionCallback.h
	${PX_SOURCE_DIR}/NpPtrTableStorageManager.h
	${PX_SOURCE_DIR}/NpCheck.h
	${PX_SOURCE_DIR}/NpRigidActorTemplate.h
	${PX_SOURCE_DIR}/NpRigidActorTemplateInternal.h
	${PX_SOURCE_DIR}/NpRigidBodyTemplate.h
	${PX_SOURCE_DIR}/NpRigidDynamic.h
	${PX_SOURCE_DIR}/NpRigidStatic.h
	${PX_SOURCE_DIR}/NpScene.h
	${PX_SOURCE_DIR}/NpSceneQueries.h
	${PX_SOURCE_DIR}/NpSceneAccessor.h
	${PX_SOURCE_DIR}/NpShape.h
	${PX_SOURCE_DIR}/NpShapeManager.h
	${PX_SOURCE_DIR}/NpDebugViz.h
	${PX_SOURCE_DIR}/NpDebugViz.cpp
	${PX_SOURCE_DIR}/NpDirectGPUAPI.h
	${PX_SOURCE_DIR}/NpDirectGPUAPI.cpp
)
SOURCE_GROUP(src FILES ${PHYSX_CORE_SOURCE})

ADD_LIBRARY(PhysX ${PHYSX_LIBTYPE}
	${PHYSX_HEADERS}
	${PHYSX_COMMON_HEADERS}
	${PHYSX_MATERIAL_HEADERS}
	${PHYSX_PVD_HEADERS}
	${PHYSX_OMNIPVD_HEADERS}
	${PHYSX_OMNIPVD_SOURCE}
	${PHYSX_PVD_SOURCE}
	${PHYSX_SOLVER_HEADERS}
	${PHYSX_COLLISION_HEADERS}

	${PHYSX_METADATA_HEADERS}
	${PHYSX_METADATA_SOURCE}

	${PHYSX_CORE_SOURCE}
	${PHYSX_BUFFERING_SOURCE}
	${PHYSX_IMMEDIATEMODE_SOURCE}
	${PHYSX_MATERIALS_SOURCE}
	${PHYSX_ARTICULATIONS_SOURCE}

	${PHYSX_PLATFORM_SRC_FILES}
)

# Add the headers to the install
INSTALL(FILES ${PHYSX_HEADERS} DESTINATION include)
INSTALL(FILES ${PHYSX_MATERIAL_HEADERS} DESTINATION include)
INSTALL(FILES ${PHYSX_COMMON_HEADERS} DESTINATION include/common)
INSTALL(FILES ${PHYSX_PVD_HEADERS} DESTINATION include/pvd)
INSTALL(FILES ${PHYSX_OMNIPVD_HEADERS} DESTINATION include/omnipvd)
INSTALL(FILES ${PHYSX_COLLISION_HEADERS} DESTINATION include/collision)
INSTALL(FILES ${PHYSX_SOLVER_HEADERS} DESTINATION include/solver)
# install the custom config file
INSTALL(FILES ${PHYSX_ROOT_DIR}/include/PxConfig.h DESTINATION include)

TARGET_INCLUDE_DIRECTORIES(PhysX
	PRIVATE ${PHYSX_PLATFORM_INCLUDES}

	PRIVATE ${PHYSX_ROOT_DIR}/include

	PRIVATE ${PHYSX_SOURCE_DIR}/common/include
	PRIVATE ${PHYSX_SOURCE_DIR}/common/src

	PRIVATE ${PHYSX_SOURCE_DIR}/physx/src
	PRIVATE ${PHYSX_SOURCE_DIR}/physx/src/device
	PRIVATE ${PHYSX_SOURCE_DIR}/physxgpu/include


	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/include
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/contact
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/common
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/convex
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/distance
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/sweep
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/gjk
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/intersection
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/mesh
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/hf
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/pcm
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/ccd

	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/api/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/software/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include/pipeline
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include/utils

	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevelaabb/include

	PRIVATE ${PHYSX_SOURCE_DIR}/lowleveldynamics/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowleveldynamics/shared

	PRIVATE ${PHYSX_SOURCE_DIR}/simulationcontroller/include
	PRIVATE ${PHYSX_SOURCE_DIR}/simulationcontroller/src

	PRIVATE ${PHYSX_SOURCE_DIR}/scenequery/include

	PRIVATE ${PHYSX_SOURCE_DIR}/physxmetadata/core/include

    PRIVATE ${PHYSX_SOURCE_DIR}/immediatemode/include

    PRIVATE ${PHYSX_SOURCE_DIR}/pvd/include

	PRIVATE ${PHYSX_SOURCE_DIR}/gpucommon/include
	PRIVATE ${PHYSX_SOURCE_DIR}/gpucommon/src/DX
	PRIVATE ${PHYSX_SOURCE_DIR}/gpucommon/src/CUDA

  PRIVATE ${PHYSX_SOURCE_DIR}/omnipvd
  PRIVATE ${PHYSX_ROOT_DIR}/pvdruntime/include
)

TARGET_COMPILE_DEFINITIONS(PhysX

	# Common to all configurations
	PRIVATE ${PHYSX_COMPILE_DEFS}
)

SET_TARGET_PROPERTIES(PhysX PROPERTIES
	OUTPUT_NAME PhysX

)

IF(PHYSX_LIBTYPE STREQUAL "STATIC")
	SET_TARGET_PROPERTIES(PhysX PROPERTIES
		ARCHIVE_OUTPUT_NAME_DEBUG "PhysX_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysX_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysX_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysX_static"
	)
ENDIF()

IF(PHYSX_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(PhysX PROPERTIES
		COMPILE_PDB_NAME_DEBUG "${PHYSX_COMPILE_PDB_NAME_DEBUG}"
		COMPILE_PDB_NAME_CHECKED "${PHYSX_COMPILE_PDB_NAME_CHECKED}"
		COMPILE_PDB_NAME_PROFILE "${PHYSX_COMPILE_PDB_NAME_PROFILE}"
		COMPILE_PDB_NAME_RELEASE "${PHYSX_COMPILE_PDB_NAME_RELEASE}"
	)
ENDIF()

TARGET_LINK_LIBRARIES(PhysX
	PRIVATE ${PHYSX_PRIVATE_PLATFORM_LINKED_LIBS}
	PRIVATE PhysXPvdSDK PhysXCommon PhysXFoundation
	PUBLIC ${PHYSX_PLATFORM_LINKED_LIBS}
)

SET_TARGET_PROPERTIES(PhysX PROPERTIES
	LINK_FLAGS "${PHYSX_PLATFORM_LINK_FLAGS}"
	LINK_FLAGS_DEBUG "${PHYSX_PLATFORM_LINK_FLAGS_DEBUG}"
	LINK_FLAGS_CHECKED "${PHYSX_PLATFORM_LINK_FLAGS_CHECKED}"
	LINK_FLAGS_PROFILE "${PHYSX_PLATFORM_LINK_FLAGS_PROFILE}"
	LINK_FLAGS_RELEASE "${PHYSX_PLATFORM_LINK_FLAGS_RELEASE}"
)

IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_COMMON_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_MATERIAL_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_PVD_HEADERS})
  	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_OMNIPVD_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_OMNIPVD_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_PVD_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_METADATA_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_METADATA_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_CORE_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_BUFFERING_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_IMMEDIATEMODE_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_MATERIALS_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_ARTICULATIONS_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_PLATFORM_SRC_FILES})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_SOLVER_HEADERS})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysX PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

