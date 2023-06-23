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
// Enums
////////////////////////////////////////////////////////////////////////////////

OMNI_PVD_ENUM			(PxConstraintFlag)
OMNI_PVD_ENUM_VALUE		(PxConstraintFlag,			eBROKEN)
OMNI_PVD_ENUM_VALUE		(PxConstraintFlag,			eCOLLISION_ENABLED)
OMNI_PVD_ENUM_VALUE		(PxConstraintFlag,			eVISUALIZATION)
OMNI_PVD_ENUM_VALUE		(PxConstraintFlag,			eDRIVE_LIMITS_ARE_FORCES)
OMNI_PVD_ENUM_VALUE		(PxConstraintFlag,			eIMPROVED_SLERP)
OMNI_PVD_ENUM_VALUE		(PxConstraintFlag,			eDISABLE_PREPROCESSING)
OMNI_PVD_ENUM_VALUE		(PxConstraintFlag,			eENABLE_EXTENDED_LIMITS)
OMNI_PVD_ENUM_VALUE		(PxConstraintFlag,			eGPU_COMPATIBLE)
OMNI_PVD_ENUM_VALUE		(PxConstraintFlag,			eALWAYS_UPDATE)
OMNI_PVD_ENUM_VALUE		(PxConstraintFlag,			eDISABLE_CONSTRAINT)

OMNI_PVD_ENUM			(PxRevoluteJointFlag)
OMNI_PVD_ENUM_VALUE		(PxRevoluteJointFlag,		eLIMIT_ENABLED)
OMNI_PVD_ENUM_VALUE		(PxRevoluteJointFlag,		eDRIVE_ENABLED)
OMNI_PVD_ENUM_VALUE		(PxRevoluteJointFlag,		eDRIVE_FREESPIN)

OMNI_PVD_ENUM			(PxPrismaticJointFlag)
OMNI_PVD_ENUM_VALUE		(PxPrismaticJointFlag,		eLIMIT_ENABLED)

OMNI_PVD_ENUM			(PxDistanceJointFlag)
OMNI_PVD_ENUM_VALUE		(PxDistanceJointFlag,		eMAX_DISTANCE_ENABLED)
OMNI_PVD_ENUM_VALUE		(PxDistanceJointFlag,		eMIN_DISTANCE_ENABLED)
OMNI_PVD_ENUM_VALUE		(PxDistanceJointFlag,		eSPRING_ENABLED)

OMNI_PVD_ENUM			(PxSphericalJointFlag)
OMNI_PVD_ENUM_VALUE		(PxSphericalJointFlag,		eLIMIT_ENABLED)

OMNI_PVD_ENUM			(PxD6JointDriveFlag)
OMNI_PVD_ENUM_VALUE		(PxD6JointDriveFlag,		eACCELERATION)

OMNI_PVD_ENUM			(PxJointConcreteType)
OMNI_PVD_ENUM_VALUE		(PxJointConcreteType,		eSPHERICAL)
OMNI_PVD_ENUM_VALUE		(PxJointConcreteType,		eREVOLUTE)
OMNI_PVD_ENUM_VALUE		(PxJointConcreteType,		ePRISMATIC)
OMNI_PVD_ENUM_VALUE		(PxJointConcreteType,		eFIXED)
OMNI_PVD_ENUM_VALUE		(PxJointConcreteType,		eDISTANCE)
OMNI_PVD_ENUM_VALUE		(PxJointConcreteType,		eD6)
OMNI_PVD_ENUM_VALUE		(PxJointConcreteType,		eCONTACT)
OMNI_PVD_ENUM_VALUE		(PxJointConcreteType,		eGEAR)
OMNI_PVD_ENUM_VALUE		(PxJointConcreteType,		eRACK_AND_PINION)

OMNI_PVD_ENUM			(PxD6Motion)
OMNI_PVD_ENUM_VALUE		(PxD6Motion,				eLOCKED)
OMNI_PVD_ENUM_VALUE		(PxD6Motion,				eLIMITED)
OMNI_PVD_ENUM_VALUE		(PxD6Motion,				eFREE)

////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// PxJoint
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS			(PxJoint)
OMNI_PVD_ATTRIBUTE_FLAG	(PxJoint, type,						PxJointConcreteType::Enum, PxJointConcreteType)
OMNI_PVD_ATTRIBUTE		(PxJoint, actor0,					PxRigidActor*, OmniPvdDataType::eOBJECT_HANDLE, 1)
OMNI_PVD_ATTRIBUTE		(PxJoint, actor1,					PxRigidActor*, OmniPvdDataType::eOBJECT_HANDLE, 1)
OMNI_PVD_ATTRIBUTE		(PxJoint, actor0LocalPose,			PxTransform, OmniPvdDataType::eFLOAT32, 7)
OMNI_PVD_ATTRIBUTE		(PxJoint, actor1LocalPose,			PxTransform, OmniPvdDataType::eFLOAT32, 7)
OMNI_PVD_ATTRIBUTE		(PxJoint, breakForce,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxJoint, breakTorque,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE_FLAG	(PxJoint, constraintFlags,			PxConstraintFlags, PxConstraintFlag)
OMNI_PVD_ATTRIBUTE		(PxJoint, invMassScale0,			PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxJoint, invInertiaScale0,			PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxJoint, invMassScale1,			PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxJoint, invInertiaScale1,			PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxJoint, name,						char, OmniPvdDataType::eSTRING, 1)
OMNI_PVD_ATTRIBUTE		(PxJoint, concreteTypeName,			char, OmniPvdDataType::eSTRING, 1)

////////////////////////////////////////////////////////////////////////////////
// PxFixedJoint
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED  (PxFixedJoint, PxJoint)

////////////////////////////////////////////////////////////////////////////////
// PxPrismaticJoint
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED	(PxPrismaticJoint, PxJoint)
OMNI_PVD_ATTRIBUTE		(PxPrismaticJoint, position,					PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxPrismaticJoint, velocity,					PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxPrismaticJoint, limitLower,					PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxPrismaticJoint, limitUpper,					PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxPrismaticJoint, limitRestitution,			PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxPrismaticJoint, limitBounceThreshold,		PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxPrismaticJoint, limitStiffness,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxPrismaticJoint, limitDamping,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE_FLAG	(PxPrismaticJoint, jointFlags,					PxPrismaticJointFlags, PxPrismaticJointFlag)

////////////////////////////////////////////////////////////////////////////////
// PxRevoluteJoint
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED  (PxRevoluteJoint, PxJoint)
OMNI_PVD_ATTRIBUTE		(PxRevoluteJoint, angle,						PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxRevoluteJoint, velocity,						PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxRevoluteJoint, limitLower,					PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxRevoluteJoint, limitUpper,					PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxRevoluteJoint, limitRestitution,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxRevoluteJoint, limitBounceThreshold,			PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxRevoluteJoint, limitStiffness,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxRevoluteJoint, limitDamping,					PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxRevoluteJoint, driveVelocity,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxRevoluteJoint, driveForceLimit,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxRevoluteJoint, driveGearRatio,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE_FLAG	(PxRevoluteJoint, jointFlags,					PxRevoluteJointFlags, PxRevoluteJointFlag)

////////////////////////////////////////////////////////////////////////////////
// PxSphericalJoint
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED  (PxSphericalJoint, PxJoint)
OMNI_PVD_ATTRIBUTE		(PxSphericalJoint, swingYAngle,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxSphericalJoint, swingZAngle,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxSphericalJoint, limitYAngle,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxSphericalJoint, limitZAngle,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxSphericalJoint, limitRestitution,			PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxSphericalJoint, limitBounceThreshold,		PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxSphericalJoint, limitStiffness,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxSphericalJoint, limitDamping,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE_FLAG	(PxSphericalJoint, jointFlags,					PxSphericalJointFlags, PxSphericalJointFlag)

////////////////////////////////////////////////////////////////////////////////
// PxDistanceJoint
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED  (PxDistanceJoint, PxJoint)
OMNI_PVD_ATTRIBUTE		(PxDistanceJoint, distance,			PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxDistanceJoint, minDistance,		PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxDistanceJoint, maxDistance,		PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxDistanceJoint, tolerance,		PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxDistanceJoint, stiffness,		PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxDistanceJoint, damping,			PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE_FLAG	(PxDistanceJoint, jointFlags,		PxDistanceJointFlags, PxDistanceJointFlag)

////////////////////////////////////////////////////////////////////////////////
// PxContactJoint
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED  (PxContactJoint, PxJoint)
OMNI_PVD_ATTRIBUTE		(PxContactJoint, point,				PxVec3, OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(PxContactJoint, normal,				PxVec3, OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(PxContactJoint, penetration,		PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxContactJoint, restitution,		PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxContactJoint, bounceThreshold,	PxReal, OmniPvdDataType::eFLOAT32, 1)

////////////////////////////////////////////////////////////////////////////////
// PxGearJoint
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED  (PxGearJoint, PxJoint)
OMNI_PVD_ATTRIBUTE		(PxGearJoint, ratio,					PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxGearJoint, hinges,					const PxBase*, OmniPvdDataType::eOBJECT_HANDLE, 0)

////////////////////////////////////////////////////////////////////////////////
// PxRackAndPinionJoint
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED  (PxRackAndPinionJoint, PxJoint)
OMNI_PVD_ATTRIBUTE		(PxRackAndPinionJoint, ratio,	PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxRackAndPinionJoint, joints,	const PxBase*, OmniPvdDataType::eOBJECT_HANDLE, 0)

////////////////////////////////////////////////////////////////////////////////
// PxD6Joint
////////////////////////////////////////////////////////////////////////////////
OMNI_PVD_CLASS_DERIVED  (PxD6Joint, PxJoint)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	twistAngle,						PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	swingYAngle,						PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	swingZAngle,						PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	motions,							PxD6Motion::Enum, OmniPvdDataType::eUINT32, 0)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	distanceLimitValue,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	distanceLimitRestitution,			PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	distanceLimitBounceThreshold,		PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	distanceLimitStiffness,			PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	distanceLimitDamping,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	linearLimitLower,					PxReal, OmniPvdDataType::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	linearLimitUpper,					PxReal, OmniPvdDataType::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(PxD6Joint, linearLimitRestitution,			PxReal, OmniPvdDataType::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	linearLimitBounceThreshold,		PxReal, OmniPvdDataType::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(PxD6Joint, linearLimitStiffness,				PxReal, OmniPvdDataType::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	linearLimitDamping,				PxReal, OmniPvdDataType::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	twistLimitLower,					PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	twistLimitUpper,					PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	twistLimitRestitution,			PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	twistLimitBounceThreshold,		PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	twistLimitStiffness,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	twistLimitDamping,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	swingLimitYAngle,					PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	swingLimitZAngle,					PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	swingLimitRestitution,			PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	swingLimitBounceThreshold,		PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	swingLimitStiffness,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	swingLimitDamping,				PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	pyramidSwingLimitYAngleMin,		PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	pyramidSwingLimitYAngleMax,		PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	pyramidSwingLimitZAngleMin,		PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	pyramidSwingLimitZAngleMax,		PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	pyramidSwingLimitRestitution,		PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	pyramidSwingLimitBounceThreshold,	PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	pyramidSwingLimitStiffness,		PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	pyramidSwingLimitDamping,			PxReal, OmniPvdDataType::eFLOAT32, 1)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	driveForceLimit,					PxReal, OmniPvdDataType::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	driveFlags,						PxD6JointDriveFlags, OmniPvdDataType::eUINT32, 0)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	driveStiffness,					PxReal, OmniPvdDataType::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	driveDamping,						PxReal, OmniPvdDataType::eFLOAT32, 0)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	drivePosition,					PxTransform, OmniPvdDataType::eFLOAT32, 7)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	driveLinVelocity,					PxVec3, OmniPvdDataType::eFLOAT32, 3)
OMNI_PVD_ATTRIBUTE		(PxD6Joint,	driveAngVelocity,					PxVec3, OmniPvdDataType::eFLOAT32, 3)
