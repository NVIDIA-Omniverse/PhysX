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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

// Declare OMNI_PVD Types and Attributes here!
// The last two attribute parameters could now be derived from the other data, so could be removed in a refactor, 
// though explicit control may be better.
// Note that HANDLE attributes have to use (Type const *) style, otherwise it won't compile!

////////////////////////////////////////////////////////////////////////////////
// Bitfields
////////////////////////////////////////////////////////////////////////////////

OMNI_PVD_ENUM_BEGIN		(PxSceneFlag)
OMNI_PVD_ENUM_VALUE		(PxSceneFlag, eENABLE_ACTIVE_ACTORS)
OMNI_PVD_ENUM_VALUE		(PxSceneFlag, eENABLE_CCD)
OMNI_PVD_ENUM_VALUE		(PxSceneFlag, eDISABLE_CCD_RESWEEP)
OMNI_PVD_ENUM_VALUE		(PxSceneFlag, eENABLE_PCM)
OMNI_PVD_ENUM_VALUE		(PxSceneFlag, eDISABLE_CONTACT_REPORT_BUFFER_RESIZE)
OMNI_PVD_ENUM_VALUE		(PxSceneFlag, eDISABLE_CONTACT_CACHE)
OMNI_PVD_ENUM_VALUE		(PxSceneFlag, eREQUIRE_RW_LOCK)
OMNI_PVD_ENUM_VALUE		(PxSceneFlag, eENABLE_STABILIZATION)
OMNI_PVD_ENUM_VALUE		(PxSceneFlag, eENABLE_AVERAGE_POINT)
OMNI_PVD_ENUM_VALUE		(PxSceneFlag, eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS)
OMNI_PVD_ENUM_VALUE		(PxSceneFlag, eENABLE_GPU_DYNAMICS)
OMNI_PVD_ENUM_VALUE		(PxSceneFlag, eENABLE_ENHANCED_DETERMINISM)
OMNI_PVD_ENUM_VALUE		(PxSceneFlag, eENABLE_FRICTION_EVERY_ITERATION)
OMNI_PVD_ENUM_VALUE		(PxSceneFlag, eENABLE_DIRECT_GPU_API)
OMNI_PVD_ENUM_END		(PxSceneFlag)

OMNI_PVD_ENUM_BEGIN		(PxMaterialFlag)
OMNI_PVD_ENUM_VALUE		(PxMaterialFlag, eDISABLE_FRICTION)
OMNI_PVD_ENUM_VALUE		(PxMaterialFlag, eDISABLE_STRONG_FRICTION)
OMNI_PVD_ENUM_VALUE		(PxMaterialFlag, eIMPROVED_PATCH_FRICTION)
OMNI_PVD_ENUM_END		(PxMaterialFlag)

OMNI_PVD_ENUM_BEGIN		(PxActorFlag)
OMNI_PVD_ENUM_VALUE		(PxActorFlag, eVISUALIZATION)
OMNI_PVD_ENUM_VALUE		(PxActorFlag, eDISABLE_GRAVITY)
OMNI_PVD_ENUM_VALUE		(PxActorFlag, eSEND_SLEEP_NOTIFIES)
OMNI_PVD_ENUM_VALUE		(PxActorFlag, eDISABLE_SIMULATION)
OMNI_PVD_ENUM_END		(PxActorFlag)

OMNI_PVD_ENUM_BEGIN		(PxRigidBodyFlag)
OMNI_PVD_ENUM_VALUE		(PxRigidBodyFlag, eKINEMATIC)
OMNI_PVD_ENUM_VALUE		(PxRigidBodyFlag, eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES)
OMNI_PVD_ENUM_VALUE		(PxRigidBodyFlag, eENABLE_CCD)
OMNI_PVD_ENUM_VALUE		(PxRigidBodyFlag, eENABLE_CCD_FRICTION)
OMNI_PVD_ENUM_VALUE		(PxRigidBodyFlag, eENABLE_POSE_INTEGRATION_PREVIEW)
OMNI_PVD_ENUM_VALUE		(PxRigidBodyFlag, eENABLE_SPECULATIVE_CCD)
OMNI_PVD_ENUM_VALUE		(PxRigidBodyFlag, eENABLE_CCD_MAX_CONTACT_IMPULSE)
OMNI_PVD_ENUM_VALUE		(PxRigidBodyFlag, eRETAIN_ACCELERATIONS)
OMNI_PVD_ENUM_VALUE		(PxRigidBodyFlag, eFORCE_KINE_KINE_NOTIFICATIONS)
OMNI_PVD_ENUM_VALUE		(PxRigidBodyFlag, eFORCE_STATIC_KINE_NOTIFICATIONS)
OMNI_PVD_ENUM_VALUE		(PxRigidBodyFlag, eENABLE_GYROSCOPIC_FORCES)
OMNI_PVD_ENUM_VALUE		(PxRigidBodyFlag, eRESERVED)
OMNI_PVD_ENUM_END		(PxRigidBodyFlag)

OMNI_PVD_ENUM_BEGIN		(PxArticulationFlag)
OMNI_PVD_ENUM_VALUE		(PxArticulationFlag, eFIX_BASE)
OMNI_PVD_ENUM_VALUE		(PxArticulationFlag, eDRIVE_LIMITS_ARE_FORCES)
OMNI_PVD_ENUM_VALUE		(PxArticulationFlag, eDISABLE_SELF_COLLISION)
OMNI_PVD_ENUM_VALUE		(PxArticulationFlag, eCOMPUTE_JOINT_FORCES)
OMNI_PVD_ENUM_END		(PxArticulationFlag)

OMNI_PVD_ENUM_BEGIN		(PxRigidDynamicLockFlag)
OMNI_PVD_ENUM_VALUE		(PxRigidDynamicLockFlag, eLOCK_LINEAR_X)
OMNI_PVD_ENUM_VALUE		(PxRigidDynamicLockFlag, eLOCK_LINEAR_Y)
OMNI_PVD_ENUM_VALUE		(PxRigidDynamicLockFlag, eLOCK_LINEAR_Z)
OMNI_PVD_ENUM_VALUE		(PxRigidDynamicLockFlag, eLOCK_ANGULAR_X)
OMNI_PVD_ENUM_VALUE		(PxRigidDynamicLockFlag, eLOCK_ANGULAR_Y)
OMNI_PVD_ENUM_VALUE		(PxRigidDynamicLockFlag, eLOCK_ANGULAR_Z)
OMNI_PVD_ENUM_END		(PxRigidDynamicLockFlag)

OMNI_PVD_ENUM_BEGIN		(PxShapeFlag)
OMNI_PVD_ENUM_VALUE		(PxShapeFlag, eSIMULATION_SHAPE)
OMNI_PVD_ENUM_VALUE		(PxShapeFlag, eSCENE_QUERY_SHAPE)
OMNI_PVD_ENUM_VALUE		(PxShapeFlag, eTRIGGER_SHAPE)
OMNI_PVD_ENUM_VALUE		(PxShapeFlag, eVISUALIZATION)
OMNI_PVD_ENUM_END		(PxShapeFlag)

////////////////////////////////////////////////////////////////////////////////
// Single value enums
////////////////////////////////////////////////////////////////////////////////

OMNI_PVD_ENUM_BEGIN		(PxFrictionType)
OMNI_PVD_ENUM_VALUE		(PxFrictionType, ePATCH)
OMNI_PVD_ENUM_VALUE		(PxFrictionType, eONE_DIRECTIONAL)
OMNI_PVD_ENUM_VALUE		(PxFrictionType, eTWO_DIRECTIONAL)
OMNI_PVD_ENUM_END		(PxFrictionType)

OMNI_PVD_ENUM_BEGIN		(PxBroadPhaseType)
OMNI_PVD_ENUM_VALUE		(PxBroadPhaseType, eSAP)
OMNI_PVD_ENUM_VALUE		(PxBroadPhaseType, eMBP)
OMNI_PVD_ENUM_VALUE		(PxBroadPhaseType, eABP)
OMNI_PVD_ENUM_VALUE		(PxBroadPhaseType, eGPU)
OMNI_PVD_ENUM_END		(PxBroadPhaseType)

OMNI_PVD_ENUM_BEGIN		(PxSolverType)
OMNI_PVD_ENUM_VALUE		(PxSolverType, ePGS)
OMNI_PVD_ENUM_VALUE		(PxSolverType, eTGS)
OMNI_PVD_ENUM_END		(PxSolverType)

OMNI_PVD_ENUM_BEGIN		(PxPairFilteringMode)
OMNI_PVD_ENUM_VALUE		(PxPairFilteringMode, eKEEP)
OMNI_PVD_ENUM_VALUE		(PxPairFilteringMode, eSUPPRESS)
OMNI_PVD_ENUM_VALUE		(PxPairFilteringMode, eKILL)
OMNI_PVD_ENUM_END		(PxPairFilteringMode)

OMNI_PVD_ENUM_BEGIN		(PxCombineMode)
OMNI_PVD_ENUM_VALUE		(PxCombineMode,	eAVERAGE)
OMNI_PVD_ENUM_VALUE		(PxCombineMode,	eMIN)
OMNI_PVD_ENUM_VALUE		(PxCombineMode,	eMULTIPLY)
OMNI_PVD_ENUM_VALUE		(PxCombineMode,	eMAX)
OMNI_PVD_ENUM_END		(PxCombineMode)

OMNI_PVD_ENUM_BEGIN		(PxActorType)
OMNI_PVD_ENUM_VALUE		(PxActorType, eRIGID_STATIC)
OMNI_PVD_ENUM_VALUE		(PxActorType, eRIGID_DYNAMIC)
OMNI_PVD_ENUM_VALUE		(PxActorType, eARTICULATION_LINK)
OMNI_PVD_ENUM_VALUE		(PxActorType, eSOFTBODY)
OMNI_PVD_ENUM_VALUE		(PxActorType, eFEMCLOTH)
OMNI_PVD_ENUM_VALUE		(PxActorType, ePBD_PARTICLESYSTEM)
OMNI_PVD_ENUM_VALUE		(PxActorType, eFLIP_PARTICLESYSTEM)
OMNI_PVD_ENUM_VALUE		(PxActorType, eMPM_PARTICLESYSTEM)
OMNI_PVD_ENUM_VALUE		(PxActorType, eHAIRSYSTEM)
OMNI_PVD_ENUM_END		(PxActorType)

OMNI_PVD_ENUM_BEGIN		(PxArticulationJointType)
OMNI_PVD_ENUM_VALUE		(PxArticulationJointType, eFIX)
OMNI_PVD_ENUM_VALUE		(PxArticulationJointType, ePRISMATIC)
OMNI_PVD_ENUM_VALUE		(PxArticulationJointType, eREVOLUTE)
OMNI_PVD_ENUM_VALUE		(PxArticulationJointType, eREVOLUTE_UNWRAPPED)
OMNI_PVD_ENUM_VALUE		(PxArticulationJointType, eSPHERICAL)
OMNI_PVD_ENUM_VALUE		(PxArticulationJointType, eUNDEFINED)
OMNI_PVD_ENUM_END		(PxArticulationJointType)

OMNI_PVD_ENUM_BEGIN		(PxArticulationMotion)
OMNI_PVD_ENUM_VALUE		(PxArticulationMotion, eLOCKED)
OMNI_PVD_ENUM_VALUE		(PxArticulationMotion, eLIMITED)
OMNI_PVD_ENUM_VALUE		(PxArticulationMotion, eFREE)
OMNI_PVD_ENUM_END		(PxArticulationMotion)

OMNI_PVD_ENUM_BEGIN		(PxArticulationDriveType)
OMNI_PVD_ENUM_VALUE		(PxArticulationDriveType, eFORCE)
OMNI_PVD_ENUM_VALUE		(PxArticulationDriveType, eACCELERATION)
OMNI_PVD_ENUM_VALUE		(PxArticulationDriveType, eTARGET)
OMNI_PVD_ENUM_VALUE		(PxArticulationDriveType, eVELOCITY)
OMNI_PVD_ENUM_VALUE		(PxArticulationDriveType, eNONE)
OMNI_PVD_ENUM_END		(PxArticulationDriveType)

////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// PxPhysics
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN				(PxPhysics)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, scenes,				PxScene)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, heightFields,		PxHeightField)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, convexMeshes,		PxConvexMesh)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, triangleMeshes,		PxTriangleMesh)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, tetrahedronMeshes,	PxTetrahedronMesh)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, softBodyMeshes,		PxSoftBodyMesh)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, shapes,				PxShape)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, bvhs,				PxBVH)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, materials,			PxMaterial)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, FEMSoftBodyMaterials,	PxFEMSoftBodyMaterial)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, FEMClothMaterials,	PxFEMClothMaterial)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, PBDMaterials,		PxPBDMaterial)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, FLIPMaterials,		PxFLIPMaterial)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, MPMMaterials,		PxMPMMaterial)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, rigidDynamics,		PxActor)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, rigidStatics,		PxActor)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, aggregates,			PxAggregate)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, articulations,		PxArticulationReducedCoordinate)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE	(PxPhysics, tolerancesScale,    PxTolerancesScale, OmniPvdDataType::eFLOAT32, 2)
OMNI_PVD_CLASS_END					(PxPhysics)

////////////////////////////////////////////////////////////////////////////////
// PxScene
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN					(PxScene)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST			(PxScene,		actors,					PxActor)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST			(PxScene,		articulations,			PxArticulationReducedCoordinate)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST			(PxScene,		aggregates,				PxAggregate)
OMNI_PVD_ATTRIBUTE_FLAG					(PxScene,		flags,					PxSceneFlags,			PxSceneFlag)
OMNI_PVD_ATTRIBUTE_FLAG					(PxScene,		frictionType,			PxFrictionType::Enum,	PxFrictionType)
OMNI_PVD_ATTRIBUTE_FLAG					(PxScene,		broadPhaseType,			PxBroadPhaseType::Enum,	PxBroadPhaseType)
OMNI_PVD_ATTRIBUTE_FLAG					(PxScene,		kineKineFilteringMode,	PxPairFilteringMode::Enum,	PxPairFilteringMode)
OMNI_PVD_ATTRIBUTE_FLAG					(PxScene,		staticKineFilteringMode,PxPairFilteringMode::Enum,	PxPairFilteringMode)
OMNI_PVD_ATTRIBUTE_FLAG					(PxScene,		solverType,				PxSolverType::Enum,			PxSolverType)
OMNI_PVD_ATTRIBUTE_STRING				(PxScene,		name)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE		(PxScene,		gravity,				PxVec3,		OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE						(PxScene,		bounceThresholdVelocity,PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		frictionOffsetThreshold,PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		frictionCorrelationDistance, PxReal,	OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		solverOffsetSlop,		PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		solverBatchSize,		PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		solverArticulationBatchSize, PxU32,	OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		nbContactDataBlocks,	PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		maxNbContactDataBlocks, PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		maxBiasCoefficient,		PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		contactReportStreamBufferSize, PxU32,	OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		ccdMaxPasses,			PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		ccdThreshold,			PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		ccdMaxSeparation,		PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		wakeCounterResetValue,	PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		hasCPUDispatcher,		bool,		OmniPvdDataType::eUINT8)
OMNI_PVD_ATTRIBUTE						(PxScene,		hasCUDAContextManager,	bool,		OmniPvdDataType::eUINT8)
OMNI_PVD_ATTRIBUTE						(PxScene,		hasSimulationEventCallback, bool,		OmniPvdDataType::eUINT8)
OMNI_PVD_ATTRIBUTE						(PxScene,		hasContactModifyCallback, bool,		OmniPvdDataType::eUINT8)
OMNI_PVD_ATTRIBUTE						(PxScene,		hasCCDContactModifyCallback, bool,		OmniPvdDataType::eUINT8)
OMNI_PVD_ATTRIBUTE						(PxScene,		hasBroadPhaseCallback,	bool,		OmniPvdDataType::eUINT8)
OMNI_PVD_ATTRIBUTE						(PxScene,		hasFilterCallback,		bool,		OmniPvdDataType::eUINT8)
OMNI_PVD_ATTRIBUTE						(PxScene,		limitsMaxNbActors,		PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		limitsMaxNbBodies,		PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		limitsMaxNbStaticShapes,PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		limitsMaxNbDynamicShapes,	PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		limitsMaxNbAggregates,	PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		limitsMaxNbConstraints,	PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		limitsMaxNbRegions,		PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		limitsMaxNbBroadPhaseOverlaps,	PxU32,	OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE		(PxScene,		sanityBounds,			PxBounds3,	OmniPvdDataType::eFLOAT32, 6)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE		(PxScene,		gpuDynamicsConfig,		PxgDynamicsMemoryConfig,	OmniPvdDataType::eUINT32, 12)
OMNI_PVD_ATTRIBUTE						(PxScene,		gpuMaxNumPartitions,	PxU32,	OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		gpuMaxNumStaticPartitions,	PxU32,	OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		gpuComputeVersion,		PxU32,	OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxScene,		contactPairSlabSize,	PxU32,	OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE		(PxScene,		tolerancesScale,		PxTolerancesScale,	OmniPvdDataType::eFLOAT32, 2)
OMNI_PVD_ATTRIBUTE						(PxScene,		pairCount,					PxU32,			OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxScene,		pairsActors,				PxActor*,		OmniPvdDataType::eOBJECT_HANDLE) // 2 for each pair
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxScene,		pairsContactCounts,			PxU32,			OmniPvdDataType::eUINT32) // 1 for each pair
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxScene,		pairsContactPoints,			PxReal,			OmniPvdDataType::eFLOAT32) // 3 for each contact
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxScene,		pairsContactNormals,		PxReal,			OmniPvdDataType::eFLOAT32) // 3 for each contact
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxScene,		pairsContactSeparations,	PxReal,			OmniPvdDataType::eFLOAT32) // 1 for each contact
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxScene,		pairsContactShapes,			PxShape*,		OmniPvdDataType::eOBJECT_HANDLE) // 2 for each contact
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxScene,		pairsContactFacesIndices,	PxU32,			OmniPvdDataType::eUINT32) // 2 for each contact
OMNI_PVD_CLASS_END						(PxScene)


////////////////////////////////////////////////////////////////////////////////
// PxBaseMaterial
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN		(PxBaseMaterial)
OMNI_PVD_CLASS_END          (PxBaseMaterial)

////////////////////////////////////////////////////////////////////////////////
// PxMaterial
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxMaterial, PxBaseMaterial)
OMNI_PVD_ATTRIBUTE_FLAG			(PxMaterial, flags,					PxMaterialFlags,			PxMaterialFlag)
OMNI_PVD_ATTRIBUTE_FLAG			(PxMaterial, frictionCombineMode,	PxCombineMode::Enum,		PxCombineMode)
OMNI_PVD_ATTRIBUTE_FLAG			(PxMaterial, restitutionCombineMode,PxCombineMode::Enum,		PxCombineMode)
OMNI_PVD_ATTRIBUTE				(PxMaterial, staticFriction,		PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxMaterial, dynamicFriction,		PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxMaterial, restitution,			PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxMaterial, damping,				PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_CLASS_END				(PxMaterial)

////////////////////////////////////////////////////////////////////////////////
// PxFEMSoftBodyMaterial
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxFEMSoftBodyMaterial, PxBaseMaterial)
OMNI_PVD_CLASS_END              (PxFEMSoftBodyMaterial)

////////////////////////////////////////////////////////////////////////////////
// PxFEMClothMaterial
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxFEMClothMaterial, PxBaseMaterial)
OMNI_PVD_CLASS_END              (PxFEMClothMaterial)

////////////////////////////////////////////////////////////////////////////////
// PxPBDMaterial
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxPBDMaterial, PxBaseMaterial)
OMNI_PVD_CLASS_END              (PxPBDMaterial)

////////////////////////////////////////////////////////////////////////////////
// PxFLIPMaterial
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxFLIPMaterial, PxBaseMaterial)
OMNI_PVD_CLASS_END              (PxFLIPMaterial)

////////////////////////////////////////////////////////////////////////////////
// PxMPMMaterial
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxMPMMaterial, PxBaseMaterial)
OMNI_PVD_CLASS_END              (PxMPMMaterial)

////////////////////////////////////////////////////////////////////////////////
// PxAggregate
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN			(PxAggregate)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST	(PxAggregate, actors,			PxActor)
OMNI_PVD_ATTRIBUTE				(PxAggregate, selfCollision,	bool, OmniPvdDataType::eUINT8)
OMNI_PVD_ATTRIBUTE				(PxAggregate, maxNbShapes,		PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE				(PxAggregate, scene,			PxScene* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_CLASS_END              (PxAggregate)

////////////////////////////////////////////////////////////////////////////////
// PxActor
// Missing
//   aggregate?
//   scene?
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN					(PxActor)
OMNI_PVD_ATTRIBUTE_FLAG					(PxActor, type,					PxActorType::Enum,	PxActorType)
OMNI_PVD_ATTRIBUTE_FLAG					(PxActor, flags,				PxActorFlags,		PxActorFlag)
OMNI_PVD_ATTRIBUTE_STRING				(PxActor, name)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE		(PxActor, worldBounds,			PxBounds3,			OmniPvdDataType::eFLOAT32,	6)
OMNI_PVD_ATTRIBUTE						(PxActor, dominance,			PxDominanceGroup,	OmniPvdDataType::eUINT8)
OMNI_PVD_ATTRIBUTE						(PxActor, ownerClient,			PxClientID,			OmniPvdDataType::eUINT8)
OMNI_PVD_CLASS_END						(PxActor)

////////////////////////////////////////////////////////////////////////////////
// PxRigidActor
// Missing
//   internalActorIndex?
//   constraints?
// Remap
//   translation + rotation -> globalPose?
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN		(PxRigidActor, PxActor)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE	(PxRigidActor, translation, PxVec3, OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE	(PxRigidActor, rotation, PxQuat, OmniPvdDataType::eFLOAT32, 4)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxRigidActor, shapes, PxShape)
OMNI_PVD_CLASS_END					(PxRigidActor)

////////////////////////////////////////////////////////////////////////////////
// PxRigidStatic
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxRigidStatic, PxRigidActor)
OMNI_PVD_CLASS_END              (PxRigidStatic)

////////////////////////////////////////////////////////////////////////////////
// PxRigidBody
// Missing
//   islandNodeIndex?
//	 force?
//	 torque?
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN		(PxRigidBody, PxRigidActor)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE	(PxRigidBody, cMassLocalPose, PxTransform, OmniPvdDataType::eFLOAT32, 7)
OMNI_PVD_ATTRIBUTE					(PxRigidBody, mass, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE	(PxRigidBody, massSpaceInertiaTensor, PxVec3, OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE					(PxRigidBody, linearDamping, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE					(PxRigidBody, angularDamping, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE	(PxRigidBody, linearVelocity, PxVec3, OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE	(PxRigidBody, angularVelocity, PxVec3, OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE					(PxRigidBody, maxLinearVelocity, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE					(PxRigidBody, maxAngularVelocity, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_FLAG				(PxRigidBody, rigidBodyFlags, PxRigidBodyFlags, PxRigidBodyFlag)
OMNI_PVD_ATTRIBUTE					(PxRigidBody, minAdvancedCCDCoefficient, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE					(PxRigidBody, maxDepenetrationVelocity, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE					(PxRigidBody, maxContactImpulse, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE					(PxRigidBody, contactSlopCoefficient, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_CLASS_END					(PxRigidBody)

////////////////////////////////////////////////////////////////////////////////
// PxRigidDynamic
// Missing
//	 kinematicTarget?
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxRigidDynamic, PxRigidBody)
OMNI_PVD_ATTRIBUTE				(PxRigidDynamic, isSleeping, bool, OmniPvdDataType::eUINT8)
OMNI_PVD_ATTRIBUTE				(PxRigidDynamic, sleepThreshold, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxRigidDynamic, stabilizationThreshold, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_FLAG			(PxRigidDynamic, rigidDynamicLockFlags, PxRigidDynamicLockFlags, PxRigidDynamicLockFlag)
OMNI_PVD_ATTRIBUTE				(PxRigidDynamic, wakeCounter, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxRigidDynamic, positionIterations, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE				(PxRigidDynamic, velocityIterations, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE				(PxRigidDynamic, contactReportThreshold, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_CLASS_END				(PxRigidDynamic)

////////////////////////////////////////////////////////////////////////////////
// PxArticulationLink
// Missing
//   inboundJoint?
//   children?
//   linkIndex?
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxArticulationLink, PxRigidBody)
OMNI_PVD_ATTRIBUTE				(PxArticulationLink, articulation, PxArticulationReducedCoordinate* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_ATTRIBUTE				(PxArticulationLink, inboundJointDOF, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE				(PxArticulationLink, CFMScale, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_CLASS_END				(PxArticulationLink)

////////////////////////////////////////////////////////////////////////////////
// PxArticulationReducedCoordinate
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN				(PxArticulationReducedCoordinate)
OMNI_PVD_ATTRIBUTE					(PxArticulationReducedCoordinate, positionIterations,			PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE					(PxArticulationReducedCoordinate, velocityIterations,			PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE					(PxArticulationReducedCoordinate, isSleeping,					bool, OmniPvdDataType::eUINT8)
OMNI_PVD_ATTRIBUTE					(PxArticulationReducedCoordinate, sleepThreshold,				PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE					(PxArticulationReducedCoordinate, stabilizationThreshold,		PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE					(PxArticulationReducedCoordinate, wakeCounter,					PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE					(PxArticulationReducedCoordinate, maxLinearVelocity,			PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE					(PxArticulationReducedCoordinate, maxAngularVelocity,			PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxArticulationReducedCoordinate, links,						PxArticulationLink)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE	(PxArticulationReducedCoordinate, worldBounds,					PxBounds3, OmniPvdDataType::eFLOAT32, 6)
OMNI_PVD_ATTRIBUTE_FLAG				(PxArticulationReducedCoordinate, articulationFlags,			PxArticulationFlags, PxArticulationFlag)
OMNI_PVD_ATTRIBUTE					(PxArticulationReducedCoordinate, dofs,							PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_CLASS_END					(PxArticulationReducedCoordinate)

////////////////////////////////////////////////////////////////////////////////
// PxArticulationJointReducedCoordinate
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN					(PxArticulationJointReducedCoordinate)
OMNI_PVD_ATTRIBUTE						(PxArticulationJointReducedCoordinate, parentLink,				PxArticulationLink* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_ATTRIBUTE						(PxArticulationJointReducedCoordinate, childLink,				PxArticulationLink* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE		(PxArticulationJointReducedCoordinate, parentTranslation,		PxVec3, OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE		(PxArticulationJointReducedCoordinate, parentRotation,			PxQuat, OmniPvdDataType::eFLOAT32, 4)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE		(PxArticulationJointReducedCoordinate, childTranslation,		PxVec3, OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE		(PxArticulationJointReducedCoordinate, childRotation,			PxQuat, OmniPvdDataType::eFLOAT32, 4)
OMNI_PVD_ATTRIBUTE_FLAG					(PxArticulationJointReducedCoordinate, type,					PxArticulationJointType::Enum, PxArticulationJointType)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxArticulationJointReducedCoordinate, motion,					PxArticulationMotion::Enum, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxArticulationJointReducedCoordinate, armature,				PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxArticulationJointReducedCoordinate, frictionCoefficient,		PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxArticulationJointReducedCoordinate, maxJointVelocity,		PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxArticulationJointReducedCoordinate, jointPosition,			PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxArticulationJointReducedCoordinate, jointVelocity,			PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_STRING				(PxArticulationJointReducedCoordinate, concreteTypeName)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxArticulationJointReducedCoordinate, limitLow,				PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxArticulationJointReducedCoordinate, limitHigh,				PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxArticulationJointReducedCoordinate, driveStiffness,			PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxArticulationJointReducedCoordinate, driveDamping,			PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxArticulationJointReducedCoordinate, driveMaxForce,			PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxArticulationJointReducedCoordinate, driveType,				PxArticulationDriveType::Enum, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxArticulationJointReducedCoordinate, driveTarget,				PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxArticulationJointReducedCoordinate, driveVelocity,			PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_CLASS_END						(PxArticulationJointReducedCoordinate)

////////////////////////////////////////////////////////////////////////////////
// PxShape
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN	(PxShape)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE		(PxShape, translation,				PxVec3,		OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE		(PxShape, rotation,					PxQuat,		OmniPvdDataType::eFLOAT32, 4)
OMNI_PVD_ATTRIBUTE						(PxShape, isExclusive,				bool,		OmniPvdDataType::eUINT8)
OMNI_PVD_ATTRIBUTE						(PxShape, geom,						PxGeometry* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_ATTRIBUTE						(PxShape, contactOffset,			PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxShape, restOffset,				PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxShape, densityForFluid,			PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxShape, torsionalPatchRadius,		PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxShape, minTorsionalPatchRadius,	PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_FLAG					(PxShape, shapeFlags,				PxShapeFlags, PxShapeFlag)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE		(PxShape, simulationFilterData,		PxFilterData, OmniPvdDataType::eUINT32, 4)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE		(PxShape, queryFilterData,			PxFilterData, OmniPvdDataType::eUINT32, 4)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxShape, materials,				PxMaterial* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_CLASS_END						(PxShape)


////////////////////////////////////////////////////////////////////////////////
// PxGeometry
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN	(PxGeometry)
OMNI_PVD_CLASS_END		(PxGeometry)

////////////////////////////////////////////////////////////////////////////////
// PxSphereGeometry
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxSphereGeometry, PxGeometry)
OMNI_PVD_ATTRIBUTE				(PxSphereGeometry, radius, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_CLASS_END				(PxSphereGeometry)

////////////////////////////////////////////////////////////////////////////////
// PxCapsuleGeometry
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxCapsuleGeometry, PxGeometry)
OMNI_PVD_ATTRIBUTE				(PxCapsuleGeometry, halfHeight, PxReal,	OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxCapsuleGeometry, radius,	PxReal,	OmniPvdDataType::eFLOAT32)
OMNI_PVD_CLASS_END				(PxCapsuleGeometry)

////////////////////////////////////////////////////////////////////////////////
// PxBoxGeometry
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN		(PxBoxGeometry, PxGeometry)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE	(PxBoxGeometry, halfExtents, PxVec3, OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_CLASS_END					(PxBoxGeometry)

////////////////////////////////////////////////////////////////////////////////
// PxPlaneGeometry
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxPlaneGeometry, PxGeometry)
OMNI_PVD_CLASS_END				(PxPlaneGeometry)

////////////////////////////////////////////////////////////////////////////////
// PxConvexMeshGeometry
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN		(PxConvexMeshGeometry, PxGeometry)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE	(PxConvexMeshGeometry, scale, PxVec3,		OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE					(PxConvexMeshGeometry, convexMesh, PxConvexMesh* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_CLASS_END					(PxConvexMeshGeometry)

////////////////////////////////////////////////////////////////////////////////
// PxConvexMesh
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN					(PxConvexMesh)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxConvexMesh,	verts,	PxReal,	OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxConvexMesh,	tris,	PxU32,	OmniPvdDataType::eUINT32)
OMNI_PVD_CLASS_END						(PxConvexMesh)

////////////////////////////////////////////////////////////////////////////////
// PxHeightFieldGeometry
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN		(PxHeightFieldGeometry, PxGeometry)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE	(PxHeightFieldGeometry, scale, PxVec3,		OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE					(PxHeightFieldGeometry, heightField, PxHeightField* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_CLASS_END					(PxHeightFieldGeometry)

////////////////////////////////////////////////////////////////////////////////
// PxHeightField
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN					(PxHeightField)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxHeightField, verts,	PxReal,	OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxHeightField, tris,	PxU32,	OmniPvdDataType::eUINT32)
OMNI_PVD_CLASS_END						(PxHeightField)

////////////////////////////////////////////////////////////////////////////////
// PxTriangleMeshGeometry
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN		(PxTriangleMeshGeometry, PxGeometry)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE	(PxTriangleMeshGeometry, scale, PxVec3,	OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE					(PxTriangleMeshGeometry, triangleMesh, PxTriangleMesh* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_CLASS_END					(PxTriangleMeshGeometry)

////////////////////////////////////////////////////////////////////////////////
// PxTriangleMesh
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN					(PxTriangleMesh)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxTriangleMesh, verts,	PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxTriangleMesh, tris,	PxU32,	OmniPvdDataType::eUINT32)
OMNI_PVD_CLASS_END						(PxTriangleMesh)

////////////////////////////////////////////////////////////////////////////////
// PxTetrahedronMeshGeometry
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxTetrahedronMeshGeometry, PxGeometry)
OMNI_PVD_ATTRIBUTE				(PxTetrahedronMeshGeometry, tetrahedronMesh, PxTetrahedronMesh* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_CLASS_END				(PxTetrahedronMeshGeometry)

////////////////////////////////////////////////////////////////////////////////
// PxTetrahedronMesh
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN					(PxTetrahedronMesh)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxTetrahedronMesh, verts,	PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxTetrahedronMesh, tets,	PxU32,	OmniPvdDataType::eUINT32)
OMNI_PVD_CLASS_END						(PxTetrahedronMesh)

////////////////////////////////////////////////////////////////////////////////
// PxCustomGeometry
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxCustomGeometry, PxGeometry)
OMNI_PVD_ATTRIBUTE				(PxCustomGeometry, callbacks,	PxCustomGeometry::Callbacks* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_CLASS_END				(PxCustomGeometry)

////////////////////////////////////////////////////////////////////////////////
// PxSoftBodyMesh
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN	(PxSoftBodyMesh)
OMNI_PVD_ATTRIBUTE		(PxSoftBodyMesh, collisionMesh,	PxTetrahedronMesh* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_ATTRIBUTE		(PxSoftBodyMesh, simulationMesh,PxTetrahedronMesh* const,	OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_CLASS_END		(PxSoftBodyMesh)

////////////////////////////////////////////////////////////////////////////////
// PxBVH
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN	(PxBVH)
OMNI_PVD_CLASS_END		(PxBVH)
