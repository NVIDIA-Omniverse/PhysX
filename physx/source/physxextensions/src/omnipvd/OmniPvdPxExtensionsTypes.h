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

// Declare OMNI_PVD Types and Attributes here!
// The last two attribute parameters could now be derived from the other data, so could be removed in a refactor, 
// though explicit control may be better.
// Note that HANDLE attributes have to use (Type const *) style, otherwise it won't compile!
// Note also that if we update the PVD USD reader code to not need different names than we use in the source code we don't need to pass both e.g. "scene" and "PxScene" and we can simplify

OMNI_PVD_ENUM			(constraintflag,		PxConstraintFlag)
OMNI_PVD_ENUM_VALUE		(constraintflag,		eBROKEN,									PxConstraintFlag::eBROKEN)
OMNI_PVD_ENUM_VALUE		(constraintflag,		ePROJECT_TO_ACTOR0,							PxConstraintFlag::ePROJECT_TO_ACTOR0)
OMNI_PVD_ENUM_VALUE		(constraintflag,		ePROJECT_TO_ACTOR1,							PxConstraintFlag::ePROJECT_TO_ACTOR1)
OMNI_PVD_ENUM_VALUE		(constraintflag,		ePROJECTION,								PxConstraintFlag::ePROJECTION)
OMNI_PVD_ENUM_VALUE		(constraintflag,		eCOLLISION_ENABLED,							PxConstraintFlag::eCOLLISION_ENABLED)
OMNI_PVD_ENUM_VALUE		(constraintflag,		eVISUALIZATION,								PxConstraintFlag::eVISUALIZATION)
OMNI_PVD_ENUM_VALUE		(constraintflag,		eDRIVE_LIMITS_ARE_FORCES,					PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES)
OMNI_PVD_ENUM_VALUE		(constraintflag,		eIMPROVED_SLERP,							PxConstraintFlag::eIMPROVED_SLERP)
OMNI_PVD_ENUM_VALUE		(constraintflag,		eDISABLE_PREPROCESSING,						PxConstraintFlag::eDISABLE_PREPROCESSING)
OMNI_PVD_ENUM_VALUE		(constraintflag,		eENABLE_EXTENDED_LIMITS,					PxConstraintFlag::eENABLE_EXTENDED_LIMITS)
OMNI_PVD_ENUM_VALUE		(constraintflag,		eGPU_COMPATIBLE,							PxConstraintFlag::eGPU_COMPATIBLE)
OMNI_PVD_ENUM_VALUE		(constraintflag,		eALWAYS_UPDATE,								PxConstraintFlag::eALWAYS_UPDATE)
OMNI_PVD_ENUM_VALUE		(constraintflag,		eDISABLE_CONSTRAINT,						PxConstraintFlag::eDISABLE_CONSTRAINT)

OMNI_PVD_ENUM			(revolutejointflag,		PxRevoluteJointFlag)
OMNI_PVD_ENUM_VALUE		(revolutejointflag,		eLIMIT_ENABLED,								PxRevoluteJointFlag::eLIMIT_ENABLED)
OMNI_PVD_ENUM_VALUE		(revolutejointflag,		eDRIVE_ENABLED,								PxRevoluteJointFlag::eDRIVE_ENABLED)
OMNI_PVD_ENUM_VALUE		(revolutejointflag,		eDRIVE_FREESPIN,							PxRevoluteJointFlag::eDRIVE_FREESPIN)

OMNI_PVD_ENUM			(prismaticjointflag,	PxPrismaticJointFlag)
OMNI_PVD_ENUM_VALUE		(prismaticjointflag,	eLIMIT_ENABLED,								PxPrismaticJointFlag::eLIMIT_ENABLED)

OMNI_PVD_ENUM			(distancejointflag,		PxDistanceJointFlag)
OMNI_PVD_ENUM_VALUE		(distancejointflag,		eMAX_DISTANCE_ENABLED,						PxDistanceJointFlag::eMAX_DISTANCE_ENABLED)
OMNI_PVD_ENUM_VALUE		(distancejointflag,		eMIN_DISTANCE_ENABLED,						PxDistanceJointFlag::eMIN_DISTANCE_ENABLED)
OMNI_PVD_ENUM_VALUE		(distancejointflag,		eSPRING_ENABLED,							PxDistanceJointFlag::eSPRING_ENABLED)

OMNI_PVD_ENUM			(sphericaljointflag,	PxSphericalJointFlag)
OMNI_PVD_ENUM_VALUE		(sphericaljointflag,	eLIMIT_ENABLED,								PxSphericalJointFlag::eLIMIT_ENABLED)

OMNI_PVD_ENUM			(d6jointdriveflag,		PxD6JointDriveFlag)
OMNI_PVD_ENUM_VALUE		(d6jointdriveflag,		eACCELERATION,								PxD6JointDriveFlag::eACCELERATION)

OMNI_PVD_ENUM			(jointconcretetype,		PxJointConcreteType)
OMNI_PVD_ENUM_VALUE		(jointconcretetype,		eSPHERICAL,						PxJointConcreteType::eSPHERICAL)
OMNI_PVD_ENUM_VALUE		(jointconcretetype,		eREVOLUTE,						PxJointConcreteType::eREVOLUTE)
OMNI_PVD_ENUM_VALUE		(jointconcretetype,		ePRISMATIC,						PxJointConcreteType::ePRISMATIC)
OMNI_PVD_ENUM_VALUE		(jointconcretetype,		eFIXED,							PxJointConcreteType::eFIXED)
OMNI_PVD_ENUM_VALUE		(jointconcretetype,		eDISTANCE,						PxJointConcreteType::eDISTANCE)
OMNI_PVD_ENUM_VALUE		(jointconcretetype,		eD6,							PxJointConcreteType::eD6)
OMNI_PVD_ENUM_VALUE		(jointconcretetype,		eCONTACT,						PxJointConcreteType::eCONTACT)
OMNI_PVD_ENUM_VALUE		(jointconcretetype,		eGEAR,							PxJointConcreteType::eGEAR)
OMNI_PVD_ENUM_VALUE		(jointconcretetype,		eRACK_AND_PINION,				PxJointConcreteType::eRACK_AND_PINION)

OMNI_PVD_ENUM			(d6motion,		PxD6Motion)
OMNI_PVD_ENUM_VALUE		(d6motion,		eLOCKED,				PxD6Motion::eLOCKED)
OMNI_PVD_ENUM_VALUE		(d6motion,		eLIMITED,				PxD6Motion::eLIMITED)
OMNI_PVD_ENUM_VALUE		(d6motion,		eFREE,					PxD6Motion::eFREE)

////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Joint
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(joint,		PxJoint)
// Common
OMNI_PVD_ATTRIBUTE_FLAG	(joint,		type,						PxJoint, PxJointConcreteType::Enum, jointconcretetype)
OMNI_PVD_ATTRIBUTE		(joint,		actor0,						PxJoint, PxRigidActor*, OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1)
OMNI_PVD_ATTRIBUTE		(joint,		actor1,						PxJoint, PxRigidActor*, OmniPvdDataTypeEnum::eOBJECT_HANDLE, 1)
OMNI_PVD_ATTRIBUTE		(joint,		actor0LocalPose,			PxJoint, PxTransform, OmniPvdDataTypeEnum::eFLOAT32, 7)
OMNI_PVD_ATTRIBUTE		(joint,		actor1LocalPose,			PxJoint, PxTransform, OmniPvdDataTypeEnum::eFLOAT32, 7)
OMNI_PVD_ATTRIBUTE		(joint,		breakForce,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		breakTorque,				PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE_FLAG	(joint,		constraintFlags,			PxJoint, PxConstraintFlags, constraintflag)
OMNI_PVD_ATTRIBUTE		(joint,		invMassScale0,				PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		invInertiaScale0,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		invMassScale1,				PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		invInertiaScale1,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		name,						PxJoint, char, OmniPvdDataTypeEnum::eSTRING, 1)
OMNI_PVD_ATTRIBUTE		(joint,		concreteTypeName,			PxJoint, char, OmniPvdDataTypeEnum::eSTRING, 1)
// Fixed
OMNI_PVD_ATTRIBUTE		(joint,		fixedProjectionLinearTolerance,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		fixedProjectionAngularTolerance,	PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
// Prismatic
OMNI_PVD_ATTRIBUTE		(joint,		prismaticPosition,						PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		prismaticVelocity,						PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		prismaticProjectionLinearTolerance,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		prismaticProjectionAngularTolerance,	PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		prismaticLimitLower,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		prismaticLimitUpper,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		prismaticLimitRestitution,				PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		prismaticLimitBounceThreshold,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		prismaticLimitStiffness,				PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		prismaticLimitDamping,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		prismaticLimitContactDistance,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE_FLAG	(joint,		prismaticJointFlags,					PxJoint, PxPrismaticJointFlags, prismaticjointflag)
// Revolute
OMNI_PVD_ATTRIBUTE		(joint,		revoluteAngle,							PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		revoluteVelocity,						PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		revoluteProjectionLinearTolerance,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		revoluteProjectionAngularTolerance,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		revoluteLimitLower,						PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		revoluteLimitUpper,						PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		revoluteLimitRestitution,				PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		revoluteLimitBounceThreshold,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		revoluteLimitStiffness,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		revoluteLimitDamping,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		revoluteLimitContactDistance,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		revoluteDriveVelocity,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		revoluteDriveForceLimit,				PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		revoluteDriveGearRatio,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE_FLAG	(joint,		revoluteJointFlags,						PxJoint, PxRevoluteJointFlags, revolutejointflag)
// Spherical
OMNI_PVD_ATTRIBUTE		(joint,		sphericalSwingYAngle,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		sphericalSwingZAngle,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		sphericalProjectionLinearTolerance,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		sphericalLimitYAngle,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		sphericalLimitZAngle,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		sphericalLimitRestitution,				PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		sphericalLimitBounceThreshold,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		sphericalLimitStiffness,				PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		sphericalLimitDamping,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		sphericalLimitContactDistance,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE_FLAG	(joint,		sphericalJointFlags,					PxJoint, PxSphericalJointFlags, sphericaljointflag)
// Distance
OMNI_PVD_ATTRIBUTE		(joint,		distanceDistance,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		distanceMinDistance,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		distanceMaxDistance,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		distanceTolerance,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		distanceStiffness,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		distanceDamping,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		distanceContactDistance,	PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE_FLAG	(joint,		distanceJointFlags,			PxJoint, PxDistanceJointFlags, distancejointflag)
// Contact
OMNI_PVD_ATTRIBUTE		(joint,		contactPoint,				PxJoint, PxVec3, OmniPvdDataTypeEnum::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(joint,		contactNormal,				PxJoint, PxVec3, OmniPvdDataTypeEnum::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(joint,		contactPenetration,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		contactRestitution,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		contactBounceThreshold,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
// Gear
OMNI_PVD_ATTRIBUTE		(joint,		gearRatio,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		gearHinges,			PxJoint, const PxBase*, OmniPvdDataTypeEnum::eOBJECT_HANDLE, 0)
// RackAndPinion
OMNI_PVD_ATTRIBUTE		(joint,		rackAndPinionRatio,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		rackAndPinionJoints,	PxJoint, const PxBase*, OmniPvdDataTypeEnum::eOBJECT_HANDLE, 0)
// D6
OMNI_PVD_ATTRIBUTE		(joint,		d6TwistAngle,						PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6SwingYAngle,						PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6SwingZAngle,						PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6ProjectionLinearTolerance,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6ProjectionAngularTolerance,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6Motions,							PxJoint, PxD6Motion::Enum, OmniPvdDataTypeEnum::eUINT32, 0)
OMNI_PVD_ATTRIBUTE		(joint,		d6DistanceLimitValue,				PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6DistanceLimitRestitution,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6DistanceLimitBounceThreshold,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6DistanceLimitStiffness,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6DistanceLimitDamping,				PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6DistanceLimitContactDistance,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6LinearLimitLower,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(joint,		d6LinearLimitUpper,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(joint,		d6LinearLimitRestitution,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(joint,		d6LinearLimitBounceThreshold,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(joint,		d6LinearLimitStiffness,				PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(joint,		d6LinearLimitDamping,				PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(joint,		d6LinearLimitContactDistance,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(joint,		d6TwistLimitLower,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6TwistLimitUpper,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6TwistLimitRestitution,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6TwistLimitBounceThreshold,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6TwistLimitStiffness,				PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6TwistLimitDamping,				PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6TwistLimitContactDistance,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6SwingLimitYAngle,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6SwingLimitZAngle,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6SwingLimitRestitution,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6SwingLimitBounceThreshold,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6SwingLimitStiffness,				PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6SwingLimitDamping,				PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6SwingLimitContactDistance,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6PyramidSwingLimitYAngleMin,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6PyramidSwingLimitYAngleMax,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6PyramidSwingLimitZAngleMin,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6PyramidSwingLimitZAngleMax,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6PyramidSwingLimitRestitution,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6PyramidSwingLimitBounceThreshold,	PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6PyramidSwingLimitStiffness,		PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6PyramidSwingLimitDamping,			PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6PyramidSwingLimitContactDistance,	PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(joint,		d6DriveForceLimit,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(joint,		d6DriveFlags,						PxJoint, PxD6JointDriveFlags, OmniPvdDataTypeEnum::eUINT32, 0)
OMNI_PVD_ATTRIBUTE		(joint,		d6DriveStiffness,					PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(joint,		d6DriveDamping,						PxJoint, PxReal, OmniPvdDataTypeEnum::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(joint,		d6DrivePosition,					PxJoint, PxTransform, OmniPvdDataTypeEnum::eFLOAT32, 7)
OMNI_PVD_ATTRIBUTE		(joint,		d6DriveLinVelocity,					PxJoint, PxVec3, OmniPvdDataTypeEnum::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(joint,		d6DriveAngVelocity,					PxJoint, PxVec3, OmniPvdDataTypeEnum::eFLOAT32, 3)
