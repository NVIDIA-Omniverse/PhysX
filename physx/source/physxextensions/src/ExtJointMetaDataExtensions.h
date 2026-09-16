// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_JOINT_META_DATA_EXTENSIONS_H
#define EXT_JOINT_META_DATA_EXTENSIONS_H
#include "PvdMetaDataExtensions.h"

namespace physx
{

namespace pvdsdk
{


struct PxExtensionPvdOnlyProperties
{
	enum Enum
	{
		FirstProp = PxExtensionsPropertyInfoName::LastPxPropertyInfoName,
		DEFINE_ENUM_RANGE( PxJoint_Actors, 2 ),
		DEFINE_ENUM_RANGE( PxJoint_BreakForce, 2 ),
		DEFINE_ENUM_RANGE( PxD6Joint_DriveVelocity, 2 ),
		DEFINE_ENUM_RANGE( PxD6Joint_Motion, PxD6Axis::eCOUNT ),
		DEFINE_ENUM_RANGE( PxD6Joint_Drive, PxD6Drive::eCOUNT * ( PxExtensionsPropertyInfoName::PxD6JointDrive_PropertiesStop - PxExtensionsPropertyInfoName::PxD6JointDrive_PropertiesStart ) ),
		DEFINE_ENUM_RANGE( PxD6Joint_LinearLimit, 100 ),
		DEFINE_ENUM_RANGE( PxD6Joint_SwingLimit, 100 ),
		DEFINE_ENUM_RANGE( PxD6Joint_TwistLimit, 100 ),
		DEFINE_ENUM_RANGE( PxPrismaticJoint_Limit, 100 ),
		DEFINE_ENUM_RANGE( PxRevoluteJoint_Limit, 100 ),
		DEFINE_ENUM_RANGE( PxSphericalJoint_LimitCone, 100 ),
		DEFINE_ENUM_RANGE( PxJoint_LocalPose, 2 )
	};
};

}

}

#endif
