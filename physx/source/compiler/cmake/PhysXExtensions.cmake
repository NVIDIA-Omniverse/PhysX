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
## Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.

#
# Build PhysXExtensions common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(LL_SOURCE_DIR ${PHYSX_SOURCE_DIR}/physxextensions/src)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysXExtensions.cmake)

SET(PHYSX_EXTENSIONS_SOURCE
	${LL_SOURCE_DIR}/ExtBroadPhase.cpp
	${LL_SOURCE_DIR}/ExtCollection.cpp
	${LL_SOURCE_DIR}/ExtConvexMeshExt.cpp
	${LL_SOURCE_DIR}/ExtCpuWorkerThread.cpp
	${LL_SOURCE_DIR}/ExtDefaultCpuDispatcher.cpp
	${LL_SOURCE_DIR}/ExtDefaultErrorCallback.cpp
	${LL_SOURCE_DIR}/ExtDefaultSimulationFilterShader.cpp
	${LL_SOURCE_DIR}/ExtDefaultStreams.cpp
	${LL_SOURCE_DIR}/ExtExtensions.cpp
	${LL_SOURCE_DIR}/ExtMetaData.cpp
	${LL_SOURCE_DIR}/ExtPvd.cpp
	${LL_SOURCE_DIR}/ExtPxStringTable.cpp
	${LL_SOURCE_DIR}/ExtRaycastCCD.cpp
	${LL_SOURCE_DIR}/ExtRigidBodyExt.cpp
	${LL_SOURCE_DIR}/ExtRigidActorExt.cpp	
	${LL_SOURCE_DIR}/ExtSceneQueryExt.cpp
	${LL_SOURCE_DIR}/ExtSceneQuerySystem.cpp
	${LL_SOURCE_DIR}/ExtCustomSceneQuerySystem.cpp
	${LL_SOURCE_DIR}/ExtSqQuery.cpp
	${LL_SOURCE_DIR}/ExtSqQuery.h
	${LL_SOURCE_DIR}/ExtSqManager.cpp
	${LL_SOURCE_DIR}/ExtSqManager.h
	${LL_SOURCE_DIR}/ExtSimpleFactory.cpp
	${LL_SOURCE_DIR}/ExtSmoothNormals.cpp
	${LL_SOURCE_DIR}/ExtSoftBodyExt.cpp
	${LL_SOURCE_DIR}/ExtTriangleMeshExt.cpp
	${LL_SOURCE_DIR}/ExtTetrahedronMeshExt.cpp
	${LL_SOURCE_DIR}/ExtRemeshingExt.cpp
	${LL_SOURCE_DIR}/ExtCpuWorkerThread.h
	${LL_SOURCE_DIR}/ExtDefaultCpuDispatcher.h
	${LL_SOURCE_DIR}/ExtInertiaTensor.h
	${LL_SOURCE_DIR}/ExtPlatform.h
	${LL_SOURCE_DIR}/ExtPvd.h
	${LL_SOURCE_DIR}/ExtSerialization.h
	${LL_SOURCE_DIR}/ExtSharedQueueEntryPool.h
	${LL_SOURCE_DIR}/ExtTaskQueueHelper.h
	${LL_SOURCE_DIR}/ExtSampling.cpp
	${LL_SOURCE_DIR}/ExtTetMakerExt.cpp
	${LL_SOURCE_DIR}/ExtGjkQueryExt.cpp
	${LL_SOURCE_DIR}/ExtCustomGeometryExt.cpp
)

#TODO, create a propper define for whether GPU features are enabled or not!
IF ((PUBLIC_RELEASE OR PX_GENERATE_GPU_PROJECTS) AND (CMAKE_SIZEOF_VOID_P EQUAL 8))
	LIST(APPEND PHYSX_EXTENSIONS_SOURCE "${LL_SOURCE_DIR}/ExtParticleExt.cpp")
	LIST(APPEND PHYSX_EXTENSIONS_SOURCE "${LL_SOURCE_DIR}/ExtParticleClothCooker.cpp")
ENDIF()

SOURCE_GROUP(src FILES ${PHYSX_EXTENSIONS_SOURCE})

SET(PHYSX_EXTENSIONS_JOINTS_SOURCE
	${LL_SOURCE_DIR}/ExtGearJoint.cpp
	${LL_SOURCE_DIR}/ExtGearJoint.h
	${LL_SOURCE_DIR}/ExtRackAndPinionJoint.cpp
	${LL_SOURCE_DIR}/ExtRackAndPinionJoint.h
	${LL_SOURCE_DIR}/ExtD6Joint.cpp
	${LL_SOURCE_DIR}/ExtD6JointCreate.cpp
	${LL_SOURCE_DIR}/ExtDistanceJoint.cpp
	${LL_SOURCE_DIR}/ExtContactJoint.cpp
	${LL_SOURCE_DIR}/ExtFixedJoint.cpp
	${LL_SOURCE_DIR}/ExtJoint.cpp
	${LL_SOURCE_DIR}/ExtPrismaticJoint.cpp
	${LL_SOURCE_DIR}/ExtRevoluteJoint.cpp
	${LL_SOURCE_DIR}/ExtSphericalJoint.cpp
	${LL_SOURCE_DIR}/ExtConstraintHelper.h
	${LL_SOURCE_DIR}/ExtD6Joint.h
	${LL_SOURCE_DIR}/ExtDistanceJoint.h
	${LL_SOURCE_DIR}/ExtContactJoint.h
	${LL_SOURCE_DIR}/ExtFixedJoint.h
	${LL_SOURCE_DIR}/ExtJoint.h
	${LL_SOURCE_DIR}/ExtJointData.h
	${LL_SOURCE_DIR}/ExtJointMetaDataExtensions.h
	${LL_SOURCE_DIR}/ExtPrismaticJoint.h
	${LL_SOURCE_DIR}/ExtRevoluteJoint.h
	${LL_SOURCE_DIR}/ExtSphericalJoint.h
)
SOURCE_GROUP(src\\joints FILES ${PHYSX_EXTENSIONS_JOINTS_SOURCE})

SET(PHYSX_EXTENSIONS_TET_SOURCE
	${LL_SOURCE_DIR}/tet/ExtUtilities.h
	${LL_SOURCE_DIR}/tet/ExtUtilities.cpp
	${LL_SOURCE_DIR}/tet/ExtTetUnionFind.h
	${LL_SOURCE_DIR}/tet/ExtTetUnionFind.cpp
	${LL_SOURCE_DIR}/tet/ExtDelaunayBoundaryInserter.cpp
	${LL_SOURCE_DIR}/tet/ExtDelaunayBoundaryInserter.h
	${LL_SOURCE_DIR}/tet/ExtDelaunayTetrahedralizer.cpp
	${LL_SOURCE_DIR}/tet/ExtDelaunayTetrahedralizer.h
	${LL_SOURCE_DIR}/tet/ExtVec3.h
	${LL_SOURCE_DIR}/tet/ExtTetSplitting.cpp
	${LL_SOURCE_DIR}/tet/ExtTetSplitting.h
	${LL_SOURCE_DIR}/tet/ExtFastWindingNumber.cpp
	${LL_SOURCE_DIR}/tet/ExtFastWindingNumber.h
	${LL_SOURCE_DIR}/tet/ExtRandomAccessHeap.h
	${LL_SOURCE_DIR}/tet/ExtQuadric.h
	${LL_SOURCE_DIR}/tet/ExtMeshSimplificator.h
	${LL_SOURCE_DIR}/tet/ExtMeshSimplificator.cpp
	${LL_SOURCE_DIR}/tet/ExtBVH.cpp
	${LL_SOURCE_DIR}/tet/ExtBVH.h
	${LL_SOURCE_DIR}/tet/ExtRemesher.cpp
	${LL_SOURCE_DIR}/tet/ExtRemesher.h
	${LL_SOURCE_DIR}/tet/ExtMarchingCubesTable.h
	${LL_SOURCE_DIR}/tet/ExtMultiList.h
	${LL_SOURCE_DIR}/tet/ExtInsideTester.cpp
	${LL_SOURCE_DIR}/tet/ExtInsideTester.h
	${LL_SOURCE_DIR}/tet/ExtOctreeTetrahedralizer.cpp
	${LL_SOURCE_DIR}/tet/ExtOctreeTetrahedralizer.h
	${LL_SOURCE_DIR}/tet/ExtVoxelTetrahedralizer.cpp
	${LL_SOURCE_DIR}/tet/ExtVoxelTetrahedralizer.h
)
SOURCE_GROUP(src\\tet FILES ${PHYSX_EXTENSIONS_TET_SOURCE})

SET(PHYSX_EXTENSIONS_METADATA_SOURCE
	${PHYSX_SOURCE_DIR}/physxmetadata/extensions/src/PxExtensionAutoGeneratedMetaDataObjects.cpp
	${PHYSX_SOURCE_DIR}/physxmetadata/extensions/include/PxExtensionAutoGeneratedMetaDataObjectNames.h
	${PHYSX_SOURCE_DIR}/physxmetadata/extensions/include/PxExtensionAutoGeneratedMetaDataObjects.h
	${PHYSX_SOURCE_DIR}/physxmetadata/extensions/include/PxExtensionMetaDataObjects.h
)
SOURCE_GROUP(src\\metadata FILES ${PHYSX_EXTENSIONS_METADATA_SOURCE})

SET(PHYSX_EXTENSIONS_OMNIPVD_SOURCE
	${LL_SOURCE_DIR}/omnipvd/OmniPvdPxExtensionsTypes.h
	${LL_SOURCE_DIR}/omnipvd/OmniPvdPxExtensionsSampler.h
	${LL_SOURCE_DIR}/omnipvd/OmniPvdPxExtensionsSampler.cpp
)
SOURCE_GROUP(src\\omnipvd FILES ${PHYSX_EXTENSIONS_OMNIPVD_SOURCE})

SET(PHYSX_EXTENSIONS_HEADERS
	${PHYSX_ROOT_DIR}/include/extensions/PxBinaryConverter.h
	${PHYSX_ROOT_DIR}/include/extensions/PxBroadPhaseExt.h
	${PHYSX_ROOT_DIR}/include/extensions/PxCollectionExt.h
	${PHYSX_ROOT_DIR}/include/extensions/PxConvexMeshExt.h
	${PHYSX_ROOT_DIR}/include/extensions/PxDefaultAllocator.h
	${PHYSX_ROOT_DIR}/include/extensions/PxDefaultCpuDispatcher.h
	${PHYSX_ROOT_DIR}/include/extensions/PxDefaultErrorCallback.h
	${PHYSX_ROOT_DIR}/include/extensions/PxDefaultSimulationFilterShader.h
	${PHYSX_ROOT_DIR}/include/extensions/PxDefaultStreams.h
	${PHYSX_ROOT_DIR}/include/extensions/PxExtensionsAPI.h
	${PHYSX_ROOT_DIR}/include/extensions/PxMassProperties.h
	${PHYSX_ROOT_DIR}/include/extensions/PxRaycastCCD.h
	${PHYSX_ROOT_DIR}/include/extensions/PxRepXSerializer.h
	${PHYSX_ROOT_DIR}/include/extensions/PxRepXSimpleType.h
	${PHYSX_ROOT_DIR}/include/extensions/PxRigidActorExt.h
	${PHYSX_ROOT_DIR}/include/extensions/PxRigidBodyExt.h
	${PHYSX_ROOT_DIR}/include/extensions/PxSceneQueryExt.h
	${PHYSX_ROOT_DIR}/include/extensions/PxSceneQuerySystemExt.h
	${PHYSX_ROOT_DIR}/include/extensions/PxCustomSceneQuerySystem.h
	${PHYSX_ROOT_DIR}/include/extensions/PxSerialization.h
	${PHYSX_ROOT_DIR}/include/extensions/PxShapeExt.h
	${PHYSX_ROOT_DIR}/include/extensions/PxSimpleFactory.h
	${PHYSX_ROOT_DIR}/include/extensions/PxSmoothNormals.h
	${PHYSX_ROOT_DIR}/include/extensions/PxSoftBodyExt.h
	${PHYSX_ROOT_DIR}/include/extensions/PxStringTableExt.h
	${PHYSX_ROOT_DIR}/include/extensions/PxTriangleMeshExt.h
	${PHYSX_ROOT_DIR}/include/extensions/PxTetrahedronMeshExt.h
	${PHYSX_ROOT_DIR}/include/extensions/PxRemeshingExt.h
	${PHYSX_ROOT_DIR}/include/extensions/PxTriangleMeshAnalysisResult.h
	${PHYSX_ROOT_DIR}/include/extensions/PxTetrahedronMeshAnalysisResult.h
	${PHYSX_ROOT_DIR}/include/extensions/PxTetMakerExt.h
	${PHYSX_ROOT_DIR}/include/extensions/PxGjkQueryExt.h
	${PHYSX_ROOT_DIR}/include/extensions/PxCustomGeometryExt.h
	${PHYSX_ROOT_DIR}/include/extensions/PxSamplingExt.h
)

#TODO, create a propper define for whether GPU features are enabled or not!
IF ((PUBLIC_RELEASE OR PX_GENERATE_GPU_PROJECTS) AND (CMAKE_SIZEOF_VOID_P EQUAL 8))
	LIST(APPEND PHYSX_EXTENSIONS_HEADERS "${PHYSX_ROOT_DIR}/include/extensions/PxParticleClothCooker.h")
	LIST(APPEND PHYSX_EXTENSIONS_HEADERS "${PHYSX_ROOT_DIR}/include/extensions/PxParticleExt.h")
ENDIF()

SOURCE_GROUP(include FILES ${PHYSX_EXTENSIONS_HEADERS})

SET(PHYSX_JOINT_HEADERS
	${PHYSX_ROOT_DIR}/include/extensions/PxConstraintExt.h
	${PHYSX_ROOT_DIR}/include/extensions/PxContactJoint.h
	${PHYSX_ROOT_DIR}/include/extensions/PxD6Joint.h
	${PHYSX_ROOT_DIR}/include/extensions/PxD6JointCreate.h
	${PHYSX_ROOT_DIR}/include/extensions/PxDistanceJoint.h
	${PHYSX_ROOT_DIR}/include/extensions/PxContactJoint.h
	${PHYSX_ROOT_DIR}/include/extensions/PxFixedJoint.h
	${PHYSX_ROOT_DIR}/include/extensions/PxGearJoint.h
	${PHYSX_ROOT_DIR}/include/extensions/PxRackAndPinionJoint.h
	${PHYSX_ROOT_DIR}/include/extensions/PxJoint.h
	${PHYSX_ROOT_DIR}/include/extensions/PxJointLimit.h
#	${PHYSX_ROOT_DIR}/include/extensions/PxJointRepXSerializer.h
	${PHYSX_ROOT_DIR}/include/extensions/PxPrismaticJoint.h
	${PHYSX_ROOT_DIR}/include/extensions/PxRevoluteJoint.h
	${PHYSX_ROOT_DIR}/include/extensions/PxSphericalJoint.h
)
SOURCE_GROUP(include\\joints FILES ${PHYSX_JOINT_HEADERS})

SET(PHYSX_FILEBUF_HEADERS
	${PHYSX_ROOT_DIR}/include/filebuf/PxFileBuf.h
)
SOURCE_GROUP(include\\filebuf FILES ${PHYSX_FILEBUF_HEADERS})

SET(PHYSX_EXTENSIONS_SERIALIZATION_SOURCE
	${LL_SOURCE_DIR}/serialization/SnSerialization.cpp
	${LL_SOURCE_DIR}/serialization/SnSerializationRegistry.cpp
	${LL_SOURCE_DIR}/serialization/SnSerializationRegistry.h
	${LL_SOURCE_DIR}/serialization/SnSerialUtils.cpp
	${LL_SOURCE_DIR}/serialization/SnSerialUtils.h
)
SOURCE_GROUP(serialization FILES ${PHYSX_EXTENSIONS_SERIALIZATION_SOURCE})

SET(PHYSX_EXTENSIONS_SERIALIZATION_XML_SOURCE
	${LL_SOURCE_DIR}/serialization/Xml/SnJointRepXSerializer.cpp
	${LL_SOURCE_DIR}/serialization/Xml/SnJointRepXSerializer.h
	${LL_SOURCE_DIR}/serialization/Xml/SnRepXCoreSerializer.cpp
	${LL_SOURCE_DIR}/serialization/Xml/SnRepXUpgrader.cpp
	${LL_SOURCE_DIR}/serialization/Xml/SnXmlSerialization.cpp
	${LL_SOURCE_DIR}/serialization/Xml/SnPxStreamOperators.h
	${LL_SOURCE_DIR}/serialization/Xml/SnRepX1_0Defaults.h
	${LL_SOURCE_DIR}/serialization/Xml/SnRepX3_1Defaults.h
	${LL_SOURCE_DIR}/serialization/Xml/SnRepX3_2Defaults.h
	${LL_SOURCE_DIR}/serialization/Xml/SnRepXCollection.h
	${LL_SOURCE_DIR}/serialization/Xml/SnRepXCoreSerializer.h
	${LL_SOURCE_DIR}/serialization/Xml/SnRepXSerializerImpl.h
	${LL_SOURCE_DIR}/serialization/Xml/SnRepXUpgrader.h
	${LL_SOURCE_DIR}/serialization/Xml/SnSimpleXmlWriter.h
	${LL_SOURCE_DIR}/serialization/Xml/SnXmlDeserializer.h
	${LL_SOURCE_DIR}/serialization/Xml/SnXmlImpl.h
	${LL_SOURCE_DIR}/serialization/Xml/SnXmlMemoryAllocator.h
	${LL_SOURCE_DIR}/serialization/Xml/SnXmlMemoryPool.h
	${LL_SOURCE_DIR}/serialization/Xml/SnXmlMemoryPoolStreams.h
	${LL_SOURCE_DIR}/serialization/Xml/SnXmlReader.h
	${LL_SOURCE_DIR}/serialization/Xml/SnXmlSerializer.h
	${LL_SOURCE_DIR}/serialization/Xml/SnXmlSimpleXmlWriter.h
	${LL_SOURCE_DIR}/serialization/Xml/SnXmlStringToType.h
	${LL_SOURCE_DIR}/serialization/Xml/SnXmlVisitorReader.h
	${LL_SOURCE_DIR}/serialization/Xml/SnXmlVisitorWriter.h
	${LL_SOURCE_DIR}/serialization/Xml/SnXmlWriter.h
)
SOURCE_GROUP(serialization\\xml FILES ${PHYSX_EXTENSIONS_SERIALIZATION_XML_SOURCE})

SET(PHYSX_EXTENSIONS_SERIALIZATION_FILE_SOURCE
	${LL_SOURCE_DIR}/serialization/File/SnFile.h
)
SOURCE_GROUP(serialization\\file FILES ${PHYSX_EXTENSIONS_SERIALIZATION_FILE_SOURCE})

SET(PHYSX_EXTENSIONS_SERIALIZATION_BINARY_SOURCE
	${LL_SOURCE_DIR}/serialization/Binary/SnBinaryDeserialization.cpp
	${LL_SOURCE_DIR}/serialization/Binary/SnBinarySerialization.cpp
	${LL_SOURCE_DIR}/serialization/Binary/SnConvX.cpp
	${LL_SOURCE_DIR}/serialization/Binary/SnConvX_Align.cpp
	${LL_SOURCE_DIR}/serialization/Binary/SnConvX_Convert.cpp
	${LL_SOURCE_DIR}/serialization/Binary/SnConvX_Error.cpp
	${LL_SOURCE_DIR}/serialization/Binary/SnConvX_MetaData.cpp
	${LL_SOURCE_DIR}/serialization/Binary/SnConvX_Output.cpp
	${LL_SOURCE_DIR}/serialization/Binary/SnConvX_Union.cpp
	${LL_SOURCE_DIR}/serialization/Binary/SnSerializationContext.cpp
	${LL_SOURCE_DIR}/serialization/Binary/SnConvX.h
	${LL_SOURCE_DIR}/serialization/Binary/SnConvX_Align.h
	${LL_SOURCE_DIR}/serialization/Binary/SnConvX_Common.h
	${LL_SOURCE_DIR}/serialization/Binary/SnConvX_MetaData.h
	${LL_SOURCE_DIR}/serialization/Binary/SnConvX_Output.h
	${LL_SOURCE_DIR}/serialization/Binary/SnConvX_Union.h
	${LL_SOURCE_DIR}/serialization/Binary/SnSerializationContext.h
)
SOURCE_GROUP(serialization\\binary FILES ${PHYSX_EXTENSIONS_SERIALIZATION_BINARY_SOURCE})

ADD_LIBRARY(PhysXExtensions ${PHYSXEXTENSIONS_LIBTYPE} 
	${PHYSXEXTENSIONS_PLATFORM_SRC_FILES}
	
	${PHYSX_EXTENSIONS_SOURCE}
	${PHYSX_EXTENSIONS_TET_SOURCE}
	${PHYSX_EXTENSIONS_JOINTS_SOURCE}
	${PHYSX_EXTENSIONS_METADATA_SOURCE}
	${PHYSX_EXTENSIONS_OMNIPVD_SOURCE}
	
	${PHYSX_EXTENSIONS_HEADERS}
	${PHYSX_JOINT_HEADERS}	
	${PHYSX_FILEBUF_HEADERS}
	
	${PHYSX_EXTENSIONS_SERIALIZATION_SOURCE}
	${PHYSX_EXTENSIONS_SERIALIZATION_XML_SOURCE}
	${PHYSX_EXTENSIONS_SERIALIZATION_FILE_SOURCE}
	${PHYSX_EXTENSIONS_SERIALIZATION_BINARY_SOURCE}
)

INSTALL(FILES ${PHYSX_EXTENSIONS_HEADERS} DESTINATION include/extensions)
INSTALL(FILES ${PHYSX_JOINT_HEADERS} DESTINATION include/extensions)
INSTALL(FILES ${PHYSX_FILEBUF_HEADERS} DESTINATION include/filebuf)

TARGET_INCLUDE_DIRECTORIES(PhysXExtensions 

	PRIVATE ${PHYSXEXTENSIONS_PLATFORM_INCLUDES}

	PRIVATE ${PHYSX_ROOT_DIR}/include

	PRIVATE ${PHYSX_SOURCE_DIR}/common/include
	PRIVATE ${PHYSX_SOURCE_DIR}/common/src
	
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/include
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/intersection
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/mesh

	PRIVATE ${PHYSX_SOURCE_DIR}/physxmetadata/core/include
	PRIVATE ${PHYSX_SOURCE_DIR}/physxmetadata/extensions/include
	
	PRIVATE ${PHYSX_SOURCE_DIR}/physxextensions/src
	PRIVATE ${PHYSX_SOURCE_DIR}/physxextensions/src/serialization/Xml
	PRIVATE ${PHYSX_SOURCE_DIR}/physxextensions/src/serialization/Binary
	PRIVATE ${PHYSX_SOURCE_DIR}/physxextensions/src/serialization/File

	PRIVATE ${PHYSX_SOURCE_DIR}/physx/src
	
	PRIVATE ${PHYSX_SOURCE_DIR}/pvd/include

	PRIVATE ${PHYSX_SOURCE_DIR}/scenequery/include

	PRIVATE ${PHYSX_SOURCE_DIR}/fastxml/include
)

TARGET_LINK_LIBRARIES(PhysXExtensions
	PRIVATE ${PHYSXEXTENSIONS_PRIVATE_PLATFORM_LINKED_LIBS}
	PUBLIC PhysXFoundation
	PUBLIC PhysXPvdSDK 
	PUBLIC PhysX
)

# Use generator expressions to set config specific preprocessor definitions
TARGET_COMPILE_DEFINITIONS(PhysXExtensions 
	PRIVATE ${PHYSXEXTENSIONS_COMPILE_DEFS}
)


SET_TARGET_PROPERTIES(PhysXExtensions PROPERTIES
	OUTPUT_NAME PhysXExtensions
)


IF(NV_USE_GAMEWORKS_OUTPUT_DIRS AND PHYSXEXTENSIONS_LIBTYPE STREQUAL "STATIC")
	SET_TARGET_PROPERTIES(PhysXExtensions PROPERTIES 
		ARCHIVE_OUTPUT_NAME_DEBUG "PhysXExtensions_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysXExtensions_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysXExtensions_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysXExtensions_static"
	)
ENDIF()

IF(PHYSXEXTENSIONS_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(PhysXExtensions PROPERTIES 
		COMPILE_PDB_NAME_DEBUG ${PHYSXEXTENSIONS_COMPILE_PDB_NAME_DEBUG}
		COMPILE_PDB_NAME_CHECKED ${PHYSXEXTENSIONS_COMPILE_PDB_NAME_CHECKED}
		COMPILE_PDB_NAME_PROFILE ${PHYSXEXTENSIONS_COMPILE_PDB_NAME_PROFILE}
		COMPILE_PDB_NAME_RELEASE ${PHYSXEXTENSIONS_COMPILE_PDB_NAME_RELEASE}
	)
ENDIF()

IF(PX_GENERATE_SOURCE_DISTRO)	
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXEXTENSIONS_PLATFORM_SRC_FILES})	
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_EXTENSIONS_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_EXTENSIONS_TET_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_EXTENSIONS_JOINTS_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_EXTENSIONS_METADATA_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_EXTENSIONS_OMNIPVD_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_EXTENSIONS_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_JOINT_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_FILEBUF_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_EXTENSIONS_SERIALIZATION_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_EXTENSIONS_SERIALIZATION_XML_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_EXTENSIONS_SERIALIZATION_FILE_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_EXTENSIONS_SERIALIZATION_BINARY_SOURCE})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysXExtensions PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
