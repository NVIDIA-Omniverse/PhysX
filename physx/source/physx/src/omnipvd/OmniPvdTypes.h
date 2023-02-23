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
// Note also that if we update the PVD USD reader code to not need different names than we use in the source code we don't need to pass both e.g. "scene" and "PxScene" and we can simplify


////////////////////////////////////////////////////////////////////////////////
// Bitfields
////////////////////////////////////////////////////////////////////////////////

OMNI_PVD_ENUM			(sceneflag,				PxSceneFlag)
OMNI_PVD_ENUM_VALUE		(sceneflag,				eENABLE_ACTIVE_ACTORS,	PxSceneFlag::eENABLE_ACTIVE_ACTORS)
OMNI_PVD_ENUM_VALUE		(sceneflag,				eENABLE_CCD,			PxSceneFlag::eENABLE_CCD)
OMNI_PVD_ENUM_VALUE		(sceneflag,				eDISABLE_CCD_RESWEEP,	PxSceneFlag::eDISABLE_CCD_RESWEEP)
OMNI_PVD_ENUM_VALUE		(sceneflag,				eENABLE_PCM,			PxSceneFlag::eENABLE_PCM)
OMNI_PVD_ENUM_VALUE		(sceneflag,				eDISABLE_CONTACT_REPORT_BUFFER_RESIZE,	PxSceneFlag::eDISABLE_CONTACT_REPORT_BUFFER_RESIZE)
OMNI_PVD_ENUM_VALUE		(sceneflag,				eDISABLE_CONTACT_CACHE,	PxSceneFlag::eDISABLE_CONTACT_CACHE)
OMNI_PVD_ENUM_VALUE		(sceneflag,				eREQUIRE_RW_LOCK,		PxSceneFlag::eREQUIRE_RW_LOCK)
OMNI_PVD_ENUM_VALUE		(sceneflag,				eENABLE_STABILIZATION,	PxSceneFlag::eENABLE_STABILIZATION)
OMNI_PVD_ENUM_VALUE		(sceneflag,				eENABLE_AVERAGE_POINT,	PxSceneFlag::eENABLE_AVERAGE_POINT)
OMNI_PVD_ENUM_VALUE		(sceneflag,				eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS,	PxSceneFlag::eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS)
OMNI_PVD_ENUM_VALUE		(sceneflag,				eENABLE_GPU_DYNAMICS,	PxSceneFlag::eENABLE_GPU_DYNAMICS)
OMNI_PVD_ENUM_VALUE		(sceneflag,				eENABLE_ENHANCED_DETERMINISM,			PxSceneFlag::eENABLE_ENHANCED_DETERMINISM)
OMNI_PVD_ENUM_VALUE		(sceneflag,				eENABLE_FRICTION_EVERY_ITERATION,		PxSceneFlag::eENABLE_FRICTION_EVERY_ITERATION)
OMNI_PVD_ENUM_VALUE		(sceneflag,				eSUPPRESS_READBACK,		PxSceneFlag::eSUPPRESS_READBACK)
OMNI_PVD_ENUM_VALUE		(sceneflag,				eFORCE_READBACK,		PxSceneFlag::eFORCE_READBACK)

OMNI_PVD_ENUM			(materialflag,			PxMaterialFlag)
OMNI_PVD_ENUM_VALUE		(materialflag,			eDISABLE_FRICTION,		PxMaterialFlag::eDISABLE_FRICTION)
OMNI_PVD_ENUM_VALUE		(materialflag,			eDISABLE_STRONG_FRICTION,PxMaterialFlag::eDISABLE_STRONG_FRICTION)
OMNI_PVD_ENUM_VALUE		(materialflag,			eIMPROVED_PATCH_FRICTION,PxMaterialFlag::eIMPROVED_PATCH_FRICTION)

OMNI_PVD_ENUM			(actorflag,				PxActorFlag)
OMNI_PVD_ENUM_VALUE		(actorflag,				eVISUALIZATION,			PxActorFlag::eVISUALIZATION)
OMNI_PVD_ENUM_VALUE		(actorflag,				eDISABLE_GRAVITY,		PxActorFlag::eDISABLE_GRAVITY)
OMNI_PVD_ENUM_VALUE		(actorflag,				eSEND_SLEEP_NOTIFIES,	PxActorFlag::eSEND_SLEEP_NOTIFIES)
OMNI_PVD_ENUM_VALUE		(actorflag,				eDISABLE_SIMULATION,	PxActorFlag::eDISABLE_SIMULATION)

OMNI_PVD_ENUM			(rigidbodyflag,			PxRigidBodyFlag)
OMNI_PVD_ENUM_VALUE		(rigidbodyflag,			eKINEMATIC,									PxRigidBodyFlag::eKINEMATIC)
OMNI_PVD_ENUM_VALUE		(rigidbodyflag,			eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES,	PxRigidBodyFlag::eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES)
OMNI_PVD_ENUM_VALUE		(rigidbodyflag,			eENABLE_CCD,								PxRigidBodyFlag::eENABLE_CCD)
OMNI_PVD_ENUM_VALUE		(rigidbodyflag,			eENABLE_CCD_FRICTION,						PxRigidBodyFlag::eENABLE_CCD_FRICTION)
OMNI_PVD_ENUM_VALUE		(rigidbodyflag,			eENABLE_POSE_INTEGRATION_PREVIEW,			PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW)
OMNI_PVD_ENUM_VALUE		(rigidbodyflag,			eENABLE_SPECULATIVE_CCD,					PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
OMNI_PVD_ENUM_VALUE		(rigidbodyflag,			eENABLE_CCD_MAX_CONTACT_IMPULSE,			PxRigidBodyFlag::eENABLE_CCD_MAX_CONTACT_IMPULSE)
OMNI_PVD_ENUM_VALUE		(rigidbodyflag,			eRETAIN_ACCELERATIONS,						PxRigidBodyFlag::eRETAIN_ACCELERATIONS)
OMNI_PVD_ENUM_VALUE		(rigidbodyflag,			eFORCE_KINE_KINE_NOTIFICATIONS,				PxRigidBodyFlag::eFORCE_KINE_KINE_NOTIFICATIONS)
OMNI_PVD_ENUM_VALUE		(rigidbodyflag,			eFORCE_STATIC_KINE_NOTIFICATIONS,			PxRigidBodyFlag::eFORCE_STATIC_KINE_NOTIFICATIONS)
OMNI_PVD_ENUM_VALUE		(rigidbodyflag,			eENABLE_GYROSCOPIC_FORCES,					PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES)
OMNI_PVD_ENUM_VALUE		(rigidbodyflag,			eRESERVED,									PxRigidBodyFlag::eRESERVED)

OMNI_PVD_ENUM			(articulationflag,		PxArticulationFlag)
OMNI_PVD_ENUM_VALUE		(articulationflag,		eFIX_BASE,									PxArticulationFlag::eFIX_BASE)
OMNI_PVD_ENUM_VALUE		(articulationflag,		eDRIVE_LIMITS_ARE_FORCES,					PxArticulationFlag::eDRIVE_LIMITS_ARE_FORCES)
OMNI_PVD_ENUM_VALUE		(articulationflag,		eDISABLE_SELF_COLLISION,					PxArticulationFlag::eDISABLE_SELF_COLLISION)
OMNI_PVD_ENUM_VALUE		(articulationflag,		eCOMPUTE_JOINT_FORCES,						PxArticulationFlag::eCOMPUTE_JOINT_FORCES)

OMNI_PVD_ENUM			(rigiddynamiclockflag,		PxRigidDynamicLockFlag)
OMNI_PVD_ENUM_VALUE		(rigiddynamiclockflag,		eLOCK_LINEAR_X,								PxRigidDynamicLockFlag::eLOCK_LINEAR_X)
OMNI_PVD_ENUM_VALUE		(rigiddynamiclockflag,		eLOCK_LINEAR_Y,								PxRigidDynamicLockFlag::eLOCK_LINEAR_Y)
OMNI_PVD_ENUM_VALUE		(rigiddynamiclockflag,		eLOCK_LINEAR_Z,								PxRigidDynamicLockFlag::eLOCK_LINEAR_Z)
OMNI_PVD_ENUM_VALUE		(rigiddynamiclockflag,		eLOCK_ANGULAR_X,							PxRigidDynamicLockFlag::eLOCK_ANGULAR_X)
OMNI_PVD_ENUM_VALUE		(rigiddynamiclockflag,		eLOCK_ANGULAR_Y,							PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y)
OMNI_PVD_ENUM_VALUE		(rigiddynamiclockflag,		eLOCK_ANGULAR_Z,							PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z)

OMNI_PVD_ENUM			(shapeflag,		PxShapeFlag)
OMNI_PVD_ENUM_VALUE		(shapeflag,		eSIMULATION_SHAPE,							PxShapeFlag::eSIMULATION_SHAPE)
OMNI_PVD_ENUM_VALUE		(shapeflag,		eSCENE_QUERY_SHAPE,							PxShapeFlag::eSCENE_QUERY_SHAPE)
OMNI_PVD_ENUM_VALUE		(shapeflag,		eTRIGGER_SHAPE,								PxShapeFlag::eTRIGGER_SHAPE)
OMNI_PVD_ENUM_VALUE		(shapeflag,		eVISUALIZATION,								PxShapeFlag::eVISUALIZATION)

////////////////////////////////////////////////////////////////////////////////
// Single value enums
////////////////////////////////////////////////////////////////////////////////

OMNI_PVD_ENUM			(frictiontype,			PxFrictionType)
OMNI_PVD_ENUM_VALUE		(frictiontype,			ePATCH,					PxFrictionType::ePATCH)
OMNI_PVD_ENUM_VALUE		(frictiontype,			eONE_DIRECTIONAL,		PxFrictionType::eONE_DIRECTIONAL)
OMNI_PVD_ENUM_VALUE		(frictiontype,			eTWO_DIRECTIONAL,		PxFrictionType::eTWO_DIRECTIONAL)

OMNI_PVD_ENUM			(broadphasetype,		PxBroadPhaseType)
OMNI_PVD_ENUM_VALUE		(broadphasetype,		eSAP,					PxBroadPhaseType::eSAP)
OMNI_PVD_ENUM_VALUE		(broadphasetype,		eMBP,					PxBroadPhaseType::eMBP)
OMNI_PVD_ENUM_VALUE		(broadphasetype,		eABP,					PxBroadPhaseType::eABP)
OMNI_PVD_ENUM_VALUE		(broadphasetype,		eGPU,					PxBroadPhaseType::eGPU)

OMNI_PVD_ENUM			(solvertype,			PxSolverType)
OMNI_PVD_ENUM_VALUE		(solvertype,			ePGS,					PxSolverType::ePGS)
OMNI_PVD_ENUM_VALUE		(solvertype,			eTGS,					PxSolverType::eTGS)

OMNI_PVD_ENUM			(pairfilteringmode,		PxPairFilteringMode)
OMNI_PVD_ENUM_VALUE		(pairfilteringmode,		eKEEP,					PxPairFilteringMode::eKEEP)
OMNI_PVD_ENUM_VALUE		(pairfilteringmode,		eSUPPRESS,				PxPairFilteringMode::eSUPPRESS)
OMNI_PVD_ENUM_VALUE		(pairfilteringmode,		eKILL,					PxPairFilteringMode::eKILL)

OMNI_PVD_ENUM			(combinemode,			PxCombineMode)
OMNI_PVD_ENUM_VALUE		(combinemode,			eAVERAGE,				PxCombineMode::eAVERAGE)
OMNI_PVD_ENUM_VALUE		(combinemode,			eMIN,					PxCombineMode::eMIN)
OMNI_PVD_ENUM_VALUE		(combinemode,			eMULTIPLY,				PxCombineMode::eMULTIPLY)
OMNI_PVD_ENUM_VALUE		(combinemode,			eMAX,					PxCombineMode::eMAX)

OMNI_PVD_ENUM			(actortype,				PxActorType)
OMNI_PVD_ENUM_VALUE		(actortype,				eRIGID_STATIC,							PxActorType::eRIGID_STATIC)
OMNI_PVD_ENUM_VALUE		(actortype,				eRIGID_DYNAMIC,							PxActorType::eRIGID_DYNAMIC)
OMNI_PVD_ENUM_VALUE		(actortype,				eARTICULATION_LINK,						PxActorType::eARTICULATION_LINK)
OMNI_PVD_ENUM_VALUE		(actortype,				eSOFTBODY,								PxActorType::eSOFTBODY)
OMNI_PVD_ENUM_VALUE		(actortype,				eFEMCLOTH,								PxActorType::eFEMCLOTH)
OMNI_PVD_ENUM_VALUE		(actortype,				ePBD_PARTICLESYSTEM,					PxActorType::ePBD_PARTICLESYSTEM)
OMNI_PVD_ENUM_VALUE		(actortype,				eFLIP_PARTICLESYSTEM,					PxActorType::eFLIP_PARTICLESYSTEM)
OMNI_PVD_ENUM_VALUE		(actortype,				eMPM_PARTICLESYSTEM,					PxActorType::eMPM_PARTICLESYSTEM)
OMNI_PVD_ENUM_VALUE		(actortype,				eCUSTOM_PARTICLESYSTEM,					PxActorType::eCUSTOM_PARTICLESYSTEM)
OMNI_PVD_ENUM_VALUE		(actortype,				eHAIRSYSTEM,							PxActorType::eHAIRSYSTEM)

OMNI_PVD_ENUM			(articulationjointtype,		PxArticulationJointType)
OMNI_PVD_ENUM_VALUE		(articulationjointtype,		eFIX,						PxArticulationJointType::eFIX)
OMNI_PVD_ENUM_VALUE		(articulationjointtype,		ePRISMATIC,					PxArticulationJointType::ePRISMATIC)
OMNI_PVD_ENUM_VALUE		(articulationjointtype,		eREVOLUTE,					PxArticulationJointType::eREVOLUTE)
OMNI_PVD_ENUM_VALUE		(articulationjointtype,		eREVOLUTE_UNWRAPPED,		PxArticulationJointType::eREVOLUTE_UNWRAPPED)
OMNI_PVD_ENUM_VALUE		(articulationjointtype,		eSPHERICAL,					PxArticulationJointType::eSPHERICAL)
OMNI_PVD_ENUM_VALUE		(articulationjointtype,		eUNDEFINED,					PxArticulationJointType::eUNDEFINED)

OMNI_PVD_ENUM			(articulationmotion,		PxArticulationMotion)
OMNI_PVD_ENUM_VALUE		(articulationmotion,		eLOCKED,				PxArticulationMotion::eLOCKED)
OMNI_PVD_ENUM_VALUE		(articulationmotion,		eLIMITED,				PxArticulationMotion::eLIMITED)
OMNI_PVD_ENUM_VALUE		(articulationmotion,		eFREE,					PxArticulationMotion::eFREE)

OMNI_PVD_ENUM			(articulationdrivetype,		PxArticulationDriveType)
OMNI_PVD_ENUM_VALUE		(articulationdrivetype,		eFORCE,						PxArticulationDriveType::eFORCE)
OMNI_PVD_ENUM_VALUE		(articulationdrivetype,		eACCELERATION,				PxArticulationDriveType::eACCELERATION)
OMNI_PVD_ENUM_VALUE		(articulationdrivetype,		eTARGET,					PxArticulationDriveType::eTARGET)
OMNI_PVD_ENUM_VALUE		(articulationdrivetype,		eVELOCITY,					PxArticulationDriveType::eVELOCITY)
OMNI_PVD_ENUM_VALUE		(articulationdrivetype,		eNONE,						PxArticulationDriveType::eNONE)

////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//						PVD name,	PVD attr,	Px classT,	Px attrT, PVD basicT, PVD basicTsize
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Physics
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(physics,						PxPhysics)
OMNI_PVD_ATTRIBUTE_SET	(physics, scenes,				PxPhysics, PxScene)
OMNI_PVD_ATTRIBUTE_SET	(physics, heightFields,			PxPhysics, PxHeightField)
OMNI_PVD_ATTRIBUTE_SET	(physics, convexMeshes,			PxPhysics, PxConvexMesh)
OMNI_PVD_ATTRIBUTE_SET	(physics, triangleMeshes,		PxPhysics, PxTriangleMesh)
OMNI_PVD_ATTRIBUTE_SET	(physics, tetrahedronMeshes,	PxPhysics, PxTetrahedronMesh)
OMNI_PVD_ATTRIBUTE_SET	(physics, softBodyMeshes,		PxPhysics, PxSoftBodyMesh)
OMNI_PVD_ATTRIBUTE_SET	(physics, shapes,				PxPhysics, PxShape)
OMNI_PVD_ATTRIBUTE_SET	(physics, bvhs,					PxPhysics, PxBVH)
OMNI_PVD_ATTRIBUTE_SET	(physics, materials,			PxPhysics, PxMaterial)
OMNI_PVD_ATTRIBUTE_SET	(physics, FEMSoftBodyMaterials,	PxPhysics, PxFEMSoftBodyMaterial)
OMNI_PVD_ATTRIBUTE_SET	(physics, FEMClothMaterials,	PxPhysics, PxFEMClothMaterial)
OMNI_PVD_ATTRIBUTE_SET	(physics, PBDMaterials,			PxPhysics, PxPBDMaterial)
OMNI_PVD_ATTRIBUTE_SET	(physics, FLIPMaterials,		PxPhysics, PxFLIPMaterial)
OMNI_PVD_ATTRIBUTE_SET	(physics, MPMMaterials,			PxPhysics, PxMPMMaterial)
OMNI_PVD_ATTRIBUTE_SET	(physics, softBodies,			PxPhysics, PxActor)
OMNI_PVD_ATTRIBUTE_SET	(physics, rigidDynamics,		PxPhysics, PxActor)
OMNI_PVD_ATTRIBUTE_SET	(physics, rigidStatics,			PxPhysics, PxActor)
OMNI_PVD_ATTRIBUTE_SET	(physics, aggregates,			PxPhysics, PxAggregate)
OMNI_PVD_ATTRIBUTE_SET	(physics, articulations,		PxPhysics, PxArticulationReducedCoordinate)
OMNI_PVD_ATTRIBUTE      (physics, tolerancesScale,      PxPhysics, PxTolerancesScale, OmniPvdDataTypeEnum::eFLOAT32, 2)

////////////////////////////////////////////////////////////////////////////////
// Scene
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(scene,					PxScene)
OMNI_PVD_ATTRIBUTE_SET	(scene,		actors,					PxScene,	PxActor)
OMNI_PVD_ATTRIBUTE_SET	(scene,		articulations,			PxScene,	PxArticulationReducedCoordinate)
OMNI_PVD_ATTRIBUTE_SET	(scene,		aggregates,				PxScene,	PxAggregate)
OMNI_PVD_ATTRIBUTE_FLAG	(scene,		flags,		PxScene,	PxSceneFlags,			sceneflag)
OMNI_PVD_ATTRIBUTE_FLAG	(scene,		frictionType,PxScene,	PxFrictionType::Enum,	frictiontype)
OMNI_PVD_ATTRIBUTE_FLAG	(scene,		broadPhaseType,PxScene,	PxBroadPhaseType::Enum,	broadphasetype)
OMNI_PVD_ATTRIBUTE_FLAG	(scene,		kineKineFilteringMode,	PxScene,	PxPairFilteringMode::Enum,	pairfilteringmode)
OMNI_PVD_ATTRIBUTE_FLAG	(scene,		staticKineFilteringMode,PxScene,	PxPairFilteringMode::Enum,	pairfilteringmode)
OMNI_PVD_ATTRIBUTE_FLAG (scene,		solverType,				PxScene,	PxSolverType::Enum,			solvertype)
OMNI_PVD_ATTRIBUTE		(scene,		gravity,				PxScene,	PxVec3,		OmniPvdDataTypeEnum::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(scene,		bounceThresholdVelocity,PxScene,	PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		frictionOffsetThreshold,PxScene,	PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		frictionCorrelationDistance, PxScene, PxReal,	OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		solverOffsetSlop,		PxScene,	PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		solverBatchSize,		PxScene,	PxU32,		OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		solverArticulationBatchSize, PxScene, PxU32,	OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		nbContactDataBlocks,	PxScene,	PxU32,		OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		maxNbContactDataBlocks, PxScene,	PxU32,		OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		maxBiasCoefficient,		PxScene,	PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		contactReportStreamBufferSize, PxScene,	PxU32,	OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		ccdMaxPasses,			PxScene,	PxU32,		OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		ccdThreshold,			PxScene,	PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		ccdMaxSeparation,		PxScene,	PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		wakeCounterResetValue,	PxScene,	PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		hasCPUDispatcher,		PxScene,	bool,		OmniPvdDataTypeEnum::eUINT8, 1)
OMNI_PVD_ATTRIBUTE		(scene,		hasCUDAContextManager,	PxScene,	bool,		OmniPvdDataTypeEnum::eUINT8, 1)
OMNI_PVD_ATTRIBUTE		(scene,		hasSimulationEventCallback, PxScene,bool,		OmniPvdDataTypeEnum::eUINT8, 1)
OMNI_PVD_ATTRIBUTE		(scene,		hasContactModifyCallback, PxScene,	bool,		OmniPvdDataTypeEnum::eUINT8, 1)
OMNI_PVD_ATTRIBUTE		(scene,		hasCCDContactModifyCallback, PxScene,bool,		OmniPvdDataTypeEnum::eUINT8, 1)
OMNI_PVD_ATTRIBUTE		(scene,		hasBroadPhaseCallback,	PxScene,	bool,		OmniPvdDataTypeEnum::eUINT8, 1)
OMNI_PVD_ATTRIBUTE		(scene,		hasFilterCallback,		PxScene,	bool,		OmniPvdDataTypeEnum::eUINT8, 1)
OMNI_PVD_ATTRIBUTE		(scene,		limitsMaxNbActors,		PxScene,	PxU32,		OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		limitsMaxNbBodies,		PxScene,	PxU32,		OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		limitsMaxNbStaticShapes,PxScene,	PxU32,		OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		limitsMaxNbDynamicShapes,PxScene,	PxU32,		OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		limitsMaxNbAggregates,	PxScene,	PxU32,		OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		limitsMaxNbConstraints,PxScene,	PxU32,		OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		limitsMaxNbRegions,	PxScene,	PxU32,		OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		limitsMaxNbBroadPhaseOverlaps,PxScene,	PxU32,	OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		sanityBounds,			PxScene,	PxBounds3,	OmniPvdDataTypeEnum::eFLOAT32, 6)
OMNI_PVD_ATTRIBUTE		(scene,		gpuDynamicsConfig,		PxScene,	PxgDynamicsMemoryConfig,	OmniPvdDataTypeEnum::eUINT32, 12)
OMNI_PVD_ATTRIBUTE		(scene,		gpuMaxNumPartitions,	PxScene,	PxU32,	OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		gpuMaxNumStaticPartitions,PxScene,	PxU32,	OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		gpuComputeVersion,		PxScene,	PxU32,	OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		contactPairSlabSize,	PxScene,	PxU32,	OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(scene,		tolerancesScale,		PxScene,	PxTolerancesScale,	OmniPvdDataTypeEnum::eFLOAT32, 2)
//OMNI_PVD_SET(scene, sceneQuerySystem, PxScene, npScene->getSQAPI())//needs class

OMNI_PVD_ATTRIBUTE		(scene,		pairCount,					PxScene,	PxU32,			OmniPvdDataTypeEnum::eUINT32,			1)
OMNI_PVD_ATTRIBUTE		(scene,		pairsActors,				PxScene,	PxActor*,		OmniPvdDataTypeEnum::eOBJECT_HANDLE,	0) // 2 for each pair
OMNI_PVD_ATTRIBUTE		(scene,		pairsContactCounts,			PxScene,	PxU32,			OmniPvdDataTypeEnum::eUINT32,			0) // 1 for each pair
OMNI_PVD_ATTRIBUTE		(scene,		pairsContactPoints,			PxScene,	PxReal,			OmniPvdDataTypeEnum::eFLOAT32,			0) // 3 for each contact
OMNI_PVD_ATTRIBUTE		(scene,		pairsContactNormals,		PxScene,	PxReal,			OmniPvdDataTypeEnum::eFLOAT32,			0) // 3 for each contact
OMNI_PVD_ATTRIBUTE		(scene,		pairsContactSeparations,	PxScene,	PxReal,			OmniPvdDataTypeEnum::eFLOAT32,			0) // 1 for each contact
OMNI_PVD_ATTRIBUTE		(scene,		pairsContactShapes,			PxScene,	PxShape*,		OmniPvdDataTypeEnum::eOBJECT_HANDLE,	0) // 2 for each contact
OMNI_PVD_ATTRIBUTE		(scene,		pairsContactFacesIndices,	PxScene,	PxU32,			OmniPvdDataTypeEnum::eUINT32,			0) // 2 for each contact

////////////////////////////////////////////////////////////////////////////////
// Material
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(material,							PxMaterial)
OMNI_PVD_ATTRIBUTE_FLAG	(material,	flags,					PxMaterial,	PxMaterialFlags,			materialflag)
OMNI_PVD_ATTRIBUTE_FLAG	(material,	frictionCombineMode,	PxMaterial,	PxCombineMode::Enum,		combinemode)
OMNI_PVD_ATTRIBUTE_FLAG	(material,	restitutionCombineMode,	PxMaterial,	PxCombineMode::Enum,		combinemode)
OMNI_PVD_ATTRIBUTE		(material,	staticFriction,			PxMaterial,	PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(material,	dynamicFriction,		PxMaterial,	PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(material,	restitution,			PxMaterial,	PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(material,	damping,				PxMaterial,	PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)

////////////////////////////////////////////////////////////////////////////////
// FEMSoftBodyMaterial
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(femsoftbodymaterial,			PxFEMSoftBodyMaterial)

////////////////////////////////////////////////////////////////////////////////
// FEMClothMaterial
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(femclothmaterial,				PxFEMClothMaterial)

////////////////////////////////////////////////////////////////////////////////
// PBDMaterial
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(pbdmaterial,					PxPBDMaterial)

////////////////////////////////////////////////////////////////////////////////
// FLIPMaterial
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(flipmaterial,					PxFLIPMaterial)

////////////////////////////////////////////////////////////////////////////////
// MPMMaterial
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(mpmmaterial,					PxMPMMaterial)

////////////////////////////////////////////////////////////////////////////////
// Aggregate
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(aggregate,					PxAggregate)
OMNI_PVD_ATTRIBUTE_SET	(aggregate, actors,			PxAggregate, PxActor)
OMNI_PVD_ATTRIBUTE		(aggregate, selfCollision,	PxAggregate, bool, OmniPvdDataTypeEnum::eUINT8, 1)
OMNI_PVD_ATTRIBUTE		(aggregate, maxNbShapes,	PxAggregate, PxU32, OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(aggregate, scene,			PxAggregate, PxScene const*, OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1)

////////////////////////////////////////////////////////////////////////////////
// Actor
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(actor,									PxActor)
//name - is there a string type?
OMNI_PVD_ATTRIBUTE_FLAG	(actor,		type,						PxActor,	PxActorType::Enum,	actortype)
OMNI_PVD_ATTRIBUTE_FLAG	(actor,		flags,						PxActor,	PxActorFlags,		actorflag)
OMNI_PVD_ATTRIBUTE_FLAG	(actor,		rigidBodyFlags,				PxActor,	PxRigidBodyFlags,	rigidbodyflag)
OMNI_PVD_ATTRIBUTE		(actor,		name,						PxActor,	char,	OmniPvdDataTypeEnum::eSTRING, 1)
OMNI_PVD_ATTRIBUTE		(actor,		dominance,					PxActor,	PxDominanceGroup,	OmniPvdDataTypeEnum::eUINT8, 1)
OMNI_PVD_ATTRIBUTE		(actor,		ownerClient,				PxActor,	PxClientID,			OmniPvdDataTypeEnum::eUINT8, 1)
OMNI_PVD_ATTRIBUTE		(actor,		translation,				PxActor,	PxVec3,	OmniPvdDataTypeEnum::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(actor,		rotation,					PxActor,	PxQuat,	OmniPvdDataTypeEnum::eFLOAT32, 4)
OMNI_PVD_ATTRIBUTE		(actor,		scale,						PxActor,	PxVec3,	OmniPvdDataTypeEnum::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(actor,		cMassLocalPose,				PxActor,	PxTransform, OmniPvdDataTypeEnum::eFLOAT32, 7)
OMNI_PVD_ATTRIBUTE		(actor,		mass,						PxActor,	PxReal,	OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(actor,		massSpaceInertiaTensor,		PxActor,	PxVec3, OmniPvdDataTypeEnum::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(actor,		linearDamping,				PxActor,	PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(actor,		angularDamping,				PxActor,	PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(actor,		linearVelocity,				PxActor,	PxVec3, OmniPvdDataTypeEnum::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(actor,		angularVelocity,			PxActor,	PxVec3, OmniPvdDataTypeEnum::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(actor,		maxLinearVelocity,			PxActor,	PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(actor,		maxAngularVelocity,			PxActor,	PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(actor,		minAdvancedCCDCoefficient,	PxActor,	PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(actor,		maxDepenetrationVelocity,	PxActor,	PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(actor,		maxContactImpulse,			PxActor,	PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(actor,		contactSlopCoefficient,		PxActor,	PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE_SET	(actor,		shapes,						PxActor,	PxShape)
OMNI_PVD_ATTRIBUTE		(actor,		articulation,				PxActor,	PxArticulationReducedCoordinate const*, OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1)
OMNI_PVD_ATTRIBUTE		(actor,		worldBounds,				PxActor,	PxBounds3, OmniPvdDataTypeEnum::eFLOAT32, 6)
OMNI_PVD_ATTRIBUTE		(actor,		isSleeping,					PxActor,	bool, OmniPvdDataTypeEnum::eUINT8, 1)
OMNI_PVD_ATTRIBUTE		(actor,		sleepThreshold,				PxActor,	PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(actor,		wakeCounter,				PxActor,	PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(actor,		stabilizationThreshold,		PxActor,	PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(actor,		positionIterations,			PxActor,	PxU32, OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(actor,		velocityIterations,			PxActor,	PxU32, OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE_FLAG	(actor,		rigidDynamicLockFlags,		PxActor,	PxRigidDynamicLockFlags, rigiddynamiclockflag)
OMNI_PVD_ATTRIBUTE		(actor,		contactReportThreshold,		PxActor,	PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(actor,		CFMScale,					PxActor,	PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(actor,		inboundJointDOF,			PxActor,	PxU32, OmniPvdDataTypeEnum::eUINT32, 1)

////////////////////////////////////////////////////////////////////////////////
// Articulation
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(articulation,								PxArticulationReducedCoordinate)
OMNI_PVD_ATTRIBUTE		(articulation, positionIterations,			PxArticulationReducedCoordinate, PxU32, OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(articulation, velocityIterations,			PxArticulationReducedCoordinate, PxU32, OmniPvdDataTypeEnum::eUINT32, 1)
OMNI_PVD_ATTRIBUTE		(articulation, isSleeping,					PxArticulationReducedCoordinate, bool, OmniPvdDataTypeEnum::eUINT8, 1)
OMNI_PVD_ATTRIBUTE		(articulation, sleepThreshold,				PxArticulationReducedCoordinate, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(articulation, stabilizationThreshold,		PxArticulationReducedCoordinate, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(articulation, wakeCounter,					PxArticulationReducedCoordinate, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(articulation, maxLinVelocity,				PxArticulationReducedCoordinate, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(articulation, maxAngVelocity,				PxArticulationReducedCoordinate, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE_SET	(articulation, links,						PxArticulationReducedCoordinate, PxArticulationLink)
OMNI_PVD_ATTRIBUTE		(articulation, worldBounds,					PxArticulationReducedCoordinate, PxBounds3, OmniPvdDataTypeEnum::eFLOAT32, 6)
OMNI_PVD_ATTRIBUTE_FLAG	(articulation, articulationFlags,			PxArticulationReducedCoordinate, PxArticulationFlags, articulationflag)
OMNI_PVD_ATTRIBUTE		(articulation, dofs,						PxArticulationReducedCoordinate, PxU32, OmniPvdDataTypeEnum::eUINT32, 1)

////////////////////////////////////////////////////////////////////////////////
// Articulation Joint
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(articulationjoint,							PxArticulationJointReducedCoordinate)
OMNI_PVD_ATTRIBUTE		(articulationjoint, parentLink,				PxArticulationJointReducedCoordinate, PxArticulationLink const *, OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1)
OMNI_PVD_ATTRIBUTE		(articulationjoint, childLink,				PxArticulationJointReducedCoordinate, PxArticulationLink const *, OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1)
OMNI_PVD_ATTRIBUTE		(articulationjoint, parentTranslation,		PxArticulationJointReducedCoordinate, PxVec3, OmniPvdDataTypeEnum::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(articulationjoint, parentRotation,			PxArticulationJointReducedCoordinate, PxQuat, OmniPvdDataTypeEnum::eFLOAT32, 4)
OMNI_PVD_ATTRIBUTE		(articulationjoint, childTranslation,		PxArticulationJointReducedCoordinate, PxVec3, OmniPvdDataTypeEnum::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(articulationjoint, childRotation,			PxArticulationJointReducedCoordinate, PxQuat, OmniPvdDataTypeEnum::eFLOAT32, 4)
OMNI_PVD_ATTRIBUTE_FLAG	(articulationjoint, type,					PxArticulationJointReducedCoordinate, PxArticulationJointType::Enum, articulationjointtype)
OMNI_PVD_ATTRIBUTE		(articulationjoint, motion,					PxArticulationJointReducedCoordinate, PxArticulationMotion::Enum, OmniPvdDataTypeEnum::eUINT32, 0)
OMNI_PVD_ATTRIBUTE		(articulationjoint, armature,				PxArticulationJointReducedCoordinate, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(articulationjoint, frictionCoefficient,	PxArticulationJointReducedCoordinate, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(articulationjoint, maxJointVelocity,		PxArticulationJointReducedCoordinate, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(articulationjoint, jointPosition,			PxArticulationJointReducedCoordinate, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(articulationjoint, jointVelocity,			PxArticulationJointReducedCoordinate, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(articulationjoint, concreteTypeName,		PxArticulationJointReducedCoordinate, char, OmniPvdDataTypeEnum::eSTRING, 1)
OMNI_PVD_ATTRIBUTE		(articulationjoint, limitLow,				PxArticulationJointReducedCoordinate, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(articulationjoint, limitHigh,				PxArticulationJointReducedCoordinate, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(articulationjoint, driveStiffness,			PxArticulationJointReducedCoordinate, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(articulationjoint, driveDamping,			PxArticulationJointReducedCoordinate, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(articulationjoint, driveMaxForce,			PxArticulationJointReducedCoordinate, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(articulationjoint, driveType,				PxArticulationJointReducedCoordinate, PxArticulationDriveType::Enum, OmniPvdDataTypeEnum::eUINT32, 0)
OMNI_PVD_ATTRIBUTE		(articulationjoint, driveTarget,			PxArticulationJointReducedCoordinate, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(articulationjoint, driveVelocity,			PxArticulationJointReducedCoordinate, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)

////////////////////////////////////////////////////////////////////////////////
// Shape
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(shape,									PxShape)
OMNI_PVD_ATTRIBUTE		(shape,		translation,				PxShape,	PxVec3,		OmniPvdDataTypeEnum::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(shape,		rotation,					PxShape,	PxQuat,		OmniPvdDataTypeEnum::eFLOAT32, 4)
OMNI_PVD_ATTRIBUTE		(shape,		scale,						PxShape,	PxVec3,		OmniPvdDataTypeEnum::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(shape,		isExclusive,				PxShape,	bool,		OmniPvdDataTypeEnum::eUINT8, 1)
OMNI_PVD_ATTRIBUTE		(shape,		geom,						PxShape,	PxGeometry const *, OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1)
OMNI_PVD_ATTRIBUTE		(shape,		contactOffset,				PxShape,	PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(shape,		restOffset,					PxShape,	PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(shape,		densityForFluid,			PxShape,	PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(shape,		torsionalPatchRadius,		PxShape,	PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(shape,		minTorsionalPatchRadius,	PxShape,	PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE_FLAG	(shape,		shapeFlags,					PxShape,	PxShapeFlags, shapeflag)
OMNI_PVD_ATTRIBUTE		(shape,		simulationFilterData,		PxShape,	PxFilterData, OmniPvdDataTypeEnum::eUINT32, 4)
OMNI_PVD_ATTRIBUTE		(shape,		queryFilterData,			PxShape,	PxFilterData, OmniPvdDataTypeEnum::eUINT32, 4)
OMNI_PVD_ATTRIBUTE      (shape,     materials,                  PxShape, PxMaterial const *, OmniPvdDataTypeEnum::eOBJECT_HANDLE, 0)

////////////////////////////////////////////////////////////////////////////////
// GeomSphere
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_FAKE_CLASS     (geomsphere, PxGeometry, PxGeomSphere)
OMNI_PVD_ATTRIBUTE		(geomsphere, radius,	PxGeometry, PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)

////////////////////////////////////////////////////////////////////////////////
// GeomCapsule
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_FAKE_CLASS     (geomcapsule, PxGeometry, PxGeomCapsule)
OMNI_PVD_ATTRIBUTE		(geomcapsule, halfHeight,PxGeometry,PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(geomcapsule, radius,	PxGeometry,	PxReal,		OmniPvdDataTypeEnum::eFLOAT32, 1)

////////////////////////////////////////////////////////////////////////////////
// GeomBox
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_FAKE_CLASS		(geombox, PxGeometry, PxGeomBox)
OMNI_PVD_ATTRIBUTE		(geombox,	halfExtents,PxGeometry, PxVec3,		OmniPvdDataTypeEnum::eFLOAT32, 3)

////////////////////////////////////////////////////////////////////////////////
// GeomPlane
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_FAKE_CLASS     (geomplane, PxGeometry, PxGeomPlane)

////////////////////////////////////////////////////////////////////////////////
// GeomConvexMesh
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_FAKE_CLASS     (geomconvexmesh, PxGeometry, PxGeomConvexMesh)
OMNI_PVD_ATTRIBUTE		(geomconvexmesh, scale, PxGeometry, PxVec3,		OmniPvdDataTypeEnum::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(geomconvexmesh, convexMesh, PxGeometry, PxConvexMesh const *, OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1)

////////////////////////////////////////////////////////////////////////////////
// ConvexMesh
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(convexmesh,			PxConvexMesh)
OMNI_PVD_ATTRIBUTE		(convexmesh,	verts,	PxConvexMesh, PxReal,	OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(convexmesh,	tris,	PxConvexMesh, PxU32,	OmniPvdDataTypeEnum::eUINT32, 0)

////////////////////////////////////////////////////////////////////////////////
// GeomHeightfield
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_FAKE_CLASS     (geomheightfield, PxGeometry, PxGeomHeightField)
OMNI_PVD_ATTRIBUTE		(geomheightfield, scale,PxGeometry, PxVec3,		OmniPvdDataTypeEnum::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(geomheightfield, heightField, PxGeometry,PxHeightField const *, OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1)

////////////////////////////////////////////////////////////////////////////////
// Heightfield
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(heightfield,			PxHeightField)
OMNI_PVD_ATTRIBUTE		(heightfield, verts,	PxHeightField, PxReal,	OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(heightfield, tris,		PxHeightField, PxU32,	OmniPvdDataTypeEnum::eUINT32, 0)

////////////////////////////////////////////////////////////////////////////////
// GeomTriangleMesh
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_FAKE_CLASS		(geomtrianglemesh, PxGeometry, PxGeomTriangleMesh)
OMNI_PVD_ATTRIBUTE		(geomtrianglemesh, scale,PxGeometry, PxVec3,	OmniPvdDataTypeEnum::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(geomtrianglemesh, triangleMesh, PxGeometry, PxTriangleMesh const *, OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1)

////////////////////////////////////////////////////////////////////////////////
// TriangleMesh
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(trianglemesh,			PxTriangleMesh)
OMNI_PVD_ATTRIBUTE		(trianglemesh, verts,	PxTriangleMesh, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(trianglemesh, tris,	PxTriangleMesh, PxU32,	OmniPvdDataTypeEnum::eUINT32, 0)

////////////////////////////////////////////////////////////////////////////////
// GeomTetrahedronMesh
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_FAKE_CLASS		(geomtetrahedronmesh, PxGeometry, PxGeomTetrahedronMesh)
OMNI_PVD_ATTRIBUTE		(geomtetrahedronmesh, tetrahedronMesh, PxGeometry, PxTetrahedronMesh const *, OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1)

////////////////////////////////////////////////////////////////////////////////
// TetrahedronMesh
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(tetrahedronmesh,			PxTetrahedronMesh)
OMNI_PVD_ATTRIBUTE		(tetrahedronmesh, verts,	PxTetrahedronMesh, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(tetrahedronmesh, tets,		PxTetrahedronMesh, PxU32,	OmniPvdDataTypeEnum::eUINT32, 0)

////////////////////////////////////////////////////////////////////////////////
// SoftBodyMesh
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(softbodymesh,					PxSoftBodyMesh)
OMNI_PVD_ATTRIBUTE		(softbodymesh, collisionMesh,	PxSoftBodyMesh, PxTetrahedronMesh*, OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1)
OMNI_PVD_ATTRIBUTE		(softbodymesh, simulationMesh,	PxSoftBodyMesh, PxTetrahedronMesh*,	OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1)

////////////////////////////////////////////////////////////////////////////////
// BVH
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(bvh,					PxBVH)
