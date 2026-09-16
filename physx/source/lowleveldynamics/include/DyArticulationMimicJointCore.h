// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#ifndef PXD_ARTICULATION_MIMIC_JOINT_CORE_H
#define PXD_ARTICULATION_MIMIC_JOINT_CORE_H


#include "foundation/PxSimpleTypes.h"

namespace physx
{
namespace Dy
{

struct ArticulationMimicJointCore
{
	PxU32 linkA;	
	PxU32 axisA; //PxArticulationAxis::Enum			
	PxU32 linkB;
	PxU32 axisB; //PxArticulationAxis::Enum
	PxReal gearRatio;
	PxReal offset;
	PxReal naturalFrequency;
	PxReal dampingRatio;
};
PX_COMPILE_TIME_ASSERT(32 == sizeof(ArticulationMimicJointCore));

}//namespace Dy
}//namespace physx

#endif //PXD_ARTICULATION_MIMIC_JOINT_CORE_H
