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

OMNI_PVD_ENUM_BEGIN		(PxParticleFlag)
OMNI_PVD_ENUM_VALUE		(PxParticleFlag, eDISABLE_SELF_COLLISION)
OMNI_PVD_ENUM_VALUE		(PxParticleFlag, eDISABLE_RIGID_COLLISION)
OMNI_PVD_ENUM_VALUE		(PxParticleFlag, eFULL_DIFFUSE_ADVECTION)
OMNI_PVD_ENUM_VALUE		(PxParticleFlag, eENABLE_SPECULATIVE_CCD)
OMNI_PVD_ENUM_END		(PxParticleFlag)

OMNI_PVD_ENUM_BEGIN		(PxParticleLockFlag)
OMNI_PVD_ENUM_VALUE		(PxParticleLockFlag, eLOCK_X)
OMNI_PVD_ENUM_VALUE		(PxParticleLockFlag, eLOCK_Y)
OMNI_PVD_ENUM_VALUE		(PxParticleLockFlag, eLOCK_Z)
OMNI_PVD_ENUM_END		(PxParticleLockFlag)

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
OMNI_PVD_ENUM_VALUE		(PxActorType, eDEFORMABLE_SURFACE)
OMNI_PVD_ENUM_VALUE		(PxActorType, eDEFORMABLE_VOLUME)
OMNI_PVD_ENUM_VALUE		(PxActorType, ePBD_PARTICLESYSTEM)
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

OMNI_PVD_ENUM_BEGIN		(PxArticulationAxis)
OMNI_PVD_ENUM_VALUE		(PxArticulationAxis, eTWIST)
OMNI_PVD_ENUM_VALUE		(PxArticulationAxis, eSWING1)
OMNI_PVD_ENUM_VALUE		(PxArticulationAxis, eSWING2)
OMNI_PVD_ENUM_VALUE		(PxArticulationAxis, eX)
OMNI_PVD_ENUM_VALUE		(PxArticulationAxis, eY)
OMNI_PVD_ENUM_VALUE		(PxArticulationAxis, eZ)
OMNI_PVD_ENUM_END		(PxArticulationAxis)

OMNI_PVD_ENUM_BEGIN(PxErrorCode)
OMNI_PVD_ENUM_VALUE(PxErrorCode, eNO_ERROR)
OMNI_PVD_ENUM_VALUE(PxErrorCode, eDEBUG_INFO)
OMNI_PVD_ENUM_VALUE(PxErrorCode, eDEBUG_WARNING)
OMNI_PVD_ENUM_VALUE(PxErrorCode, eINVALID_PARAMETER)
OMNI_PVD_ENUM_VALUE(PxErrorCode, eINVALID_OPERATION)
OMNI_PVD_ENUM_VALUE(PxErrorCode, eOUT_OF_MEMORY)
OMNI_PVD_ENUM_VALUE(PxErrorCode, eINTERNAL_ERROR)
OMNI_PVD_ENUM_VALUE(PxErrorCode, eABORT)
OMNI_PVD_ENUM_VALUE(PxErrorCode, ePERF_WARNING)
OMNI_PVD_ENUM_END(PxErrorCode)

////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// PxOmniPvdMetaData
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_UNTYPED_BEGIN (PxOmniPvdMetaData)
OMNI_PVD_ATTRIBUTE	 (PxOmniPvdMetaData, physxVersionMajor, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE	 (PxOmniPvdMetaData, physxVersionMinor, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE	 (PxOmniPvdMetaData, physxVersionBugfix, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE	 (PxOmniPvdMetaData, ovdIntegrationVersionMajor, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE	 (PxOmniPvdMetaData, ovdIntegrationVersionMinor, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_CLASS_END   (PxOmniPvdMetaData)

////////////////////////////////////////////////////////////////////////////////
// PxPhysics
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN				(PxPhysics)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, scenes,						PxScene)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, constraints,				PxConstraint)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, heightFields,				PxHeightField)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, convexMeshes,				PxConvexMesh)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, triangleMeshes,				PxTriangleMesh)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, tetrahedronMeshes,			PxTetrahedronMesh)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, deformableVolumeMeshes,		PxDeformableVolumeMesh)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, shapes,						PxShape)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, bvhs,						PxBVH)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, materials,					PxMaterial)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, deformableSurfaceMaterials,	PxDeformableSurfaceMaterial)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, deformableVolumeMaterials,	PxDeformableVolumeMaterial)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, PBDMaterials,				PxPBDMaterial)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, rigidDynamics,				PxActor)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, rigidStatics,				PxActor)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, pbdParticleSystems,			PxActor)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, particleBuffers,			PxParticleBuffer)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, aggregates,					PxAggregate)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST		(PxPhysics, articulations,				PxArticulationReducedCoordinate)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE	(PxPhysics, tolerancesScale,			PxTolerancesScale, OmniPvdDataType::eFLOAT32, 2)
OMNI_PVD_CLASS_END					(PxPhysics)

////////////////////////////////////////////////////////////////////////////////
// PxGpuDynamicsMemoryConfig
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN	(PxGpuDynamicsMemoryConfig)
OMNI_PVD_ATTRIBUTE		(PxGpuDynamicsMemoryConfig,		tempBufferCapacity,					PxU64,		OmniPvdDataType::eUINT64)
OMNI_PVD_ATTRIBUTE		(PxGpuDynamicsMemoryConfig,		maxRigidContactCount,				PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE		(PxGpuDynamicsMemoryConfig,		maxRigidPatchCount,					PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE		(PxGpuDynamicsMemoryConfig,		heapCapacity,						PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE		(PxGpuDynamicsMemoryConfig,		foundLostPairsCapacity,				PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE		(PxGpuDynamicsMemoryConfig,		foundLostAggregatePairsCapacity,	PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE		(PxGpuDynamicsMemoryConfig,		totalAggregatePairsCapacity,		PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE		(PxGpuDynamicsMemoryConfig,		maxDeformableSurfaceContacts,		PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE		(PxGpuDynamicsMemoryConfig,		maxDeformableVolumeContacts,		PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE		(PxGpuDynamicsMemoryConfig,		maxParticleContacts,				PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE		(PxGpuDynamicsMemoryConfig,		collisionStackSize,					PxU32,		OmniPvdDataType::eUINT32)
OMNI_PVD_CLASS_END		(PxGpuDynamicsMemoryConfig)

////////////////////////////////////////////////////////////////////////////////
// PxScene
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN					(PxScene)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST			(PxScene,		actors,					PxActor)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST			(PxScene,		articulations,			PxArticulationReducedCoordinate)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST			(PxScene,		aggregates,				PxAggregate)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST			(PxScene,		constraints,			PxConstraint)
OMNI_PVD_ATTRIBUTE_FLAG					(PxScene,		flags,					PxSceneFlags,			PxSceneFlag)
OMNI_PVD_ATTRIBUTE_FLAG					(PxScene,		frictionType,			PxFrictionType::Enum,	PxFrictionType)
OMNI_PVD_ATTRIBUTE_FLAG					(PxScene,		broadPhaseType,			PxBroadPhaseType::Enum,	PxBroadPhaseType)
OMNI_PVD_ATTRIBUTE_FLAG					(PxScene,		kineKineFilteringMode,	PxPairFilteringMode::Enum,	PxPairFilteringMode)
OMNI_PVD_ATTRIBUTE_FLAG					(PxScene,		staticKineFilteringMode,PxPairFilteringMode::Enum,	PxPairFilteringMode)
OMNI_PVD_ATTRIBUTE_FLAG					(PxScene,		solverType,				PxSolverType::Enum,			PxSolverType)
OMNI_PVD_ATTRIBUTE_STRING				(PxScene,		name)
OMNI_PVD_ATTRIBUTE						(PxScene,		elapsedTime,			PxReal,		OmniPvdDataType::eFLOAT32)
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
OMNI_PVD_ATTRIBUTE						(PxScene,		gpuDynamicsConfig,	PxGpuDynamicsMemoryConfig* const, OmniPvdDataType::eOBJECT_HANDLE)
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
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxScene,		pairsContactImpulses,		PxReal,			OmniPvdDataType::eFLOAT32) // 1 for each contact
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxScene,		pairsFrictionAnchorCounts,		PxU32,			OmniPvdDataType::eUINT32) // 1 for each pair
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxScene,		pairsFrictionAnchorPositions,	PxReal,			OmniPvdDataType::eFLOAT32) // 3 for each friction anchor
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxScene,		pairsFrictionAnchorNormals,		PxReal,			OmniPvdDataType::eFLOAT32) // 3 for each friction anchor
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxScene,		pairsFrictionAnchorImpulses,	PxReal,			OmniPvdDataType::eFLOAT32) // 3 for each friction anchor
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
OMNI_PVD_ATTRIBUTE_FLAG			(PxMaterial, dampingCombineMode,	PxCombineMode::Enum,		PxCombineMode)
OMNI_PVD_ATTRIBUTE				(PxMaterial, staticFriction,		PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxMaterial, dynamicFriction,		PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxMaterial, restitution,			PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxMaterial, damping,				PxReal,		OmniPvdDataType::eFLOAT32)
OMNI_PVD_CLASS_END				(PxMaterial)

////////////////////////////////////////////////////////////////////////////////
// PxDeformableSurfaceMaterial
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxDeformableSurfaceMaterial, PxBaseMaterial)
OMNI_PVD_CLASS_END              (PxDeformableSurfaceMaterial)

////////////////////////////////////////////////////////////////////////////////
// PxDeformableVolumeMaterial
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxDeformableVolumeMaterial, PxBaseMaterial)
OMNI_PVD_CLASS_END              (PxDeformableVolumeMaterial)

////////////////////////////////////////////////////////////////////////////////
// PxPBDMaterial
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxPBDMaterial, PxBaseMaterial)
OMNI_PVD_ATTRIBUTE				(PxPBDMaterial, friction, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxPBDMaterial, damping, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxPBDMaterial, adhesion, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxPBDMaterial, gravityScale, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxPBDMaterial, adhesionRadiusScale, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxPBDMaterial, viscosity, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxPBDMaterial, vorticityConfinement, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxPBDMaterial, surfaceTension, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxPBDMaterial, cohesion, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxPBDMaterial, lift, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxPBDMaterial, drag, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxPBDMaterial, CFLCoefficient, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxPBDMaterial, particleFrictionScale, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxPBDMaterial, particleAdhesionScale, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_CLASS_END              (PxPBDMaterial)

////////////////////////////////////////////////////////////////////////////////
// PxAggregate
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN			(PxAggregate)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST	(PxAggregate, actors,			PxActor)
OMNI_PVD_ATTRIBUTE				(PxAggregate, selfCollision,	bool, OmniPvdDataType::eUINT8)
OMNI_PVD_ATTRIBUTE				(PxAggregate, environmentID,	PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE				(PxAggregate, maxNbShapes,		PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE				(PxAggregate, scene,			PxScene* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_CLASS_END              (PxAggregate)

////////////////////////////////////////////////////////////////////////////////
// PxConstraint
////////////////////////////////////////////////////////////////////////////////
// Just a place holder class to be extended and improved in
// https://jirasw.nvidia.com/browse/PX-3394
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN				(PxConstraint)
OMNI_PVD_CLASS_END					(PxConstraint)

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
OMNI_PVD_ATTRIBUTE						(PxActor, environmentID,		PxU32,				OmniPvdDataType::eUINT32)
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
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE	(PxRigidActor, globalPose, PxTransform, OmniPvdDataType::eFLOAT32, 7)
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
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE	(PxRigidBody, force, PxVec3, OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE	(PxRigidBody, torque, PxVec3, OmniPvdDataType::eFLOAT32, 3)
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
OMNI_PVD_ATTRIBUTE				(PxArticulationLink, inboundJoint, PxArticulationJointReducedCoordinate* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_ATTRIBUTE				(PxArticulationLink, articulation, PxArticulationReducedCoordinate* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_ATTRIBUTE				(PxArticulationLink, inboundJointDOF, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE				(PxArticulationLink, CFMScale, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_CLASS_END				(PxArticulationLink)

////////////////////////////////////////////////////////////////////////////////
// PxPBDParticeSystem
////////////////////////////////////////////////////////////////////////////////

OMNI_PVD_CLASS_DERIVED_BEGIN			(PxPBDParticleSystem, PxActor)
OMNI_PVD_ATTRIBUTE						(PxPBDParticleSystem, positionIterations, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxPBDParticleSystem, velocityIterations, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE		(PxPBDParticleSystem, simulationFilterData, PxFilterData, OmniPvdDataType::eUINT32, 4)
OMNI_PVD_ATTRIBUTE_FLAG					(PxPBDParticleSystem, particleFlags, PxParticleFlags, PxParticleFlag)
OMNI_PVD_ATTRIBUTE						(PxPBDParticleSystem, maxDepenetrationVelocity, PxF32, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxPBDParticleSystem, maxVelocity, PxF32, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxPBDParticleSystem, restOffset, PxF32, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxPBDParticleSystem, contactOffset, PxF32, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxPBDParticleSystem, particleContactOffset, PxF32, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxPBDParticleSystem, solidRestOffset, PxF32, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE_FLAG					(PxPBDParticleSystem, particleLockFlags, PxParticleLockFlags, PxParticleLockFlag)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE		(PxPBDParticleSystem, wind, PxVec3, OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE						(PxPBDParticleSystem, fluidBoundaryDensityScale, PxF32, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxPBDParticleSystem, fluidRestOffset, PxF32, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxPBDParticleSystem, gridSizeX, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxPBDParticleSystem, gridSizeY, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE						(PxPBDParticleSystem, gridSizeZ, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE_UNIQUE_LIST			(PxPBDParticleSystem, particleBuffers, PxParticleBuffer)
OMNI_PVD_CLASS_END						(PxPBDParticleSystem)

////////////////////////////////////////////////////////////////////////////////
// PxArticulationReducedCoordinate
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN				(PxArticulationReducedCoordinate)
OMNI_PVD_ATTRIBUTE_STRING			(PxArticulationReducedCoordinate, name)
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
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxArticulationJointReducedCoordinate, jointForce,				PxReal, OmniPvdDataType::eFLOAT32)
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
// PxArticulationMimicJoint
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN					(PxArticulationMimicJoint)
OMNI_PVD_ATTRIBUTE						(PxArticulationMimicJoint,			   articulation,			PxArticulationReducedCoordinate* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_ATTRIBUTE						(PxArticulationMimicJoint,			   jointA,					PxArticulationJointReducedCoordinate* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_ATTRIBUTE						(PxArticulationMimicJoint,			   jointB,					PxArticulationJointReducedCoordinate* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_ATTRIBUTE_FLAG					(PxArticulationMimicJoint,			   axisA,					PxArticulationAxis::Enum, PxArticulationAxis)
OMNI_PVD_ATTRIBUTE_FLAG					(PxArticulationMimicJoint,			   axisB,					PxArticulationAxis::Enum, PxArticulationAxis)
OMNI_PVD_ATTRIBUTE						(PxArticulationMimicJoint,			   gearRatio,				PxReal,	OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxArticulationMimicJoint,			   offset,					PxReal,	OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxArticulationMimicJoint,			   naturalFrequency,		PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE						(PxArticulationMimicJoint,			   dampingRatio,			PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_CLASS_END						(PxArticulationMimicJoint)

////////////////////////////////////////////////////////////////////////////////
// PxShape
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN	(PxShape)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE		(PxShape, localPose,				PxTransform,		OmniPvdDataType::eFLOAT32, 7)
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
// PxConvexCoreGeometry
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN	(PxConvexCoreGeometry, PxGeometry)
OMNI_PVD_ATTRIBUTE				(PxConvexCoreGeometry, core, void* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_ATTRIBUTE				(PxConvexCoreGeometry, margin, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_CLASS_END				(PxConvexCoreGeometry)

////////////////////////////////////////////////////////////////////////////////
// PxConvexCorePoint
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN			(PxConvexCorePoint)
OMNI_PVD_CLASS_END				(PxConvexCorePoint)

////////////////////////////////////////////////////////////////////////////////
// PxConvexCoreSegment
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN			(PxConvexCoreSegment)
OMNI_PVD_ATTRIBUTE				(PxConvexCoreSegment, length, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_CLASS_END				(PxConvexCoreSegment)

////////////////////////////////////////////////////////////////////////////////
// PxConvexCoreBox
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN			(PxConvexCoreBox)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE(PxConvexCoreBox, extents, PxVec3, OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_CLASS_END				(PxConvexCoreBox)

////////////////////////////////////////////////////////////////////////////////
// PxConvexCoreEllipsoid
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN			(PxConvexCoreEllipsoid)
OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE(PxConvexCoreEllipsoid, radii, PxVec3, OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_CLASS_END				(PxConvexCoreEllipsoid)

////////////////////////////////////////////////////////////////////////////////
// PxConvexCoreCylinder
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN			(PxConvexCoreCylinder)
OMNI_PVD_ATTRIBUTE				(PxConvexCoreCylinder, height, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxConvexCoreCylinder, radius, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_CLASS_END				(PxConvexCoreCylinder)

////////////////////////////////////////////////////////////////////////////////
// PxConvexCoreCone
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN			(PxConvexCoreCone)
OMNI_PVD_ATTRIBUTE				(PxConvexCoreCone, height, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE				(PxConvexCoreCone, radius, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_CLASS_END				(PxConvexCoreCone)

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
// PxDeformableVolumeMesh
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN	(PxDeformableVolumeMesh)
OMNI_PVD_ATTRIBUTE		(PxDeformableVolumeMesh, collisionMesh,	PxTetrahedronMesh* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_ATTRIBUTE		(PxDeformableVolumeMesh, simulationMesh,PxTetrahedronMesh* const,	OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_CLASS_END		(PxDeformableVolumeMesh)

////////////////////////////////////////////////////////////////////////////////
// PxBVH
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN	(PxBVH)
OMNI_PVD_CLASS_END		(PxBVH)

////////////////////////////////////////////////////////////////////////////////
// PxParticleBuffer
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN						(PxParticleBuffer)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE		(PxParticleBuffer, positionInvMasses, PxReal, OmniPvdDataType::eFLOAT32) // 4 each
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE		(PxParticleBuffer, velocities, PxReal, OmniPvdDataType::eFLOAT32) // 4 each
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE		(PxParticleBuffer, phases, PxU32, OmniPvdDataType::eUINT32) // 1 each
OMNI_PVD_ATTRIBUTE							(PxParticleBuffer, maxParticles, PxU32,  OmniPvdDataType::eUINT32)
//can't support this right now, we can't represent an array of PxParticleVolume with a OmniPvdDataType
//OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE	(PxParticleBuffer, particleVolumes, PxParticleVolume, ???)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE		(PxParticleBuffer, volumeBounds, PxReal, OmniPvdDataType::eFLOAT32) // 6 each
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE		(PxParticleBuffer, volumeParticleIndicesOffsets, PxU32, OmniPvdDataType::eUINT32) // 1 each
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE		(PxParticleBuffer, volumeNumParticles, PxU32, OmniPvdDataType::eUINT32) // 1 each
OMNI_PVD_ATTRIBUTE							(PxParticleBuffer, maxParticleVolumes, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE							(PxParticleBuffer, flatListStartIndex, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE							(PxParticleBuffer, uniqueId, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_CLASS_END							(PxParticleBuffer)

////////////////////////////////////////////////////////////////////////////////
// PxDiffuseParticleParams
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_BEGIN						(PxDiffuseParticleParams)
OMNI_PVD_ATTRIBUTE							(PxDiffuseParticleParams, threshold, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE							(PxDiffuseParticleParams, lifetime, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE							(PxDiffuseParticleParams, airDrag, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE							(PxDiffuseParticleParams, bubbleDrag, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE							(PxDiffuseParticleParams, buoyancy, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE							(PxDiffuseParticleParams, kineticEnergyWeight, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE							(PxDiffuseParticleParams, pressureWeight, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE							(PxDiffuseParticleParams, divergenceWeight, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE							(PxDiffuseParticleParams, collisionDecay, PxReal, OmniPvdDataType::eFLOAT32)
OMNI_PVD_ATTRIBUTE							(PxDiffuseParticleParams, useAccurateVelocity, bool, OmniPvdDataType::eINT8)
OMNI_PVD_CLASS_END							(PxDiffuseParticleParams)

////////////////////////////////////////////////////////////////////////////////
// PxParticleAndDiffuseBuffer
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN				(PxParticleAndDiffuseBuffer, PxParticleBuffer)
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE		(PxParticleAndDiffuseBuffer, diffusePositionLifeTime, PxReal, OmniPvdDataType::eFLOAT32) // 4 each
OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE		(PxParticleAndDiffuseBuffer, diffuseVelocities, PxReal, OmniPvdDataType::eFLOAT32) // 4 each
OMNI_PVD_ATTRIBUTE							(PxParticleAndDiffuseBuffer, maxDiffuseParticles, PxU32, OmniPvdDataType::eUINT32)
OMNI_PVD_ATTRIBUTE							(PxParticleAndDiffuseBuffer, diffuseParticleParams, PxDiffuseParticleParams* const, OmniPvdDataType::eOBJECT_HANDLE)
OMNI_PVD_CLASS_END							(PxParticleAndDiffuseBuffer)

////////////////////////////////////////////////////////////////////////////////
// PxParticleClothBuffer
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN(PxParticleClothBuffer, PxParticleBuffer)
OMNI_PVD_CLASS_END(PxParticleClothBuffer)

////////////////////////////////////////////////////////////////////////////////
// PxParticleRigidBuffer
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED_BEGIN(PxParticleRigidBuffer, PxParticleBuffer)
OMNI_PVD_CLASS_END(PxParticleRigidBuffer)