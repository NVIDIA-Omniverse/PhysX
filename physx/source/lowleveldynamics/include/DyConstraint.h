// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_CONSTRAINT_H
#define DY_CONSTRAINT_H

#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "PxPhysXConfig.h"
#include "PxvDynamics.h"
#include "PxConstraint.h"
#include "DyConstraintWriteBack.h"

namespace physx
{

class PxsRigidBody;

namespace Dy
{

#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif
PX_ALIGN_PREFIX(16)
struct Constraint
{
public:

	PxReal					linBreakForce;
	PxReal					angBreakForce;
	PxU16					constantBlockSize;
	PxU16					flags;

	PxConstraintSolverPrep	solverPrep;
	void*					constantBlock;

	PxsRigidBody*			body0;
	PxsRigidBody*			body1;

	PxsBodyCore*			bodyCore0;
	PxsBodyCore*			bodyCore1;
	PxU32					index;
	PxReal					minResponseThreshold;
}
PX_ALIGN_SUFFIX(16);
#if PX_VC 
     #pragma warning(pop) 
#endif

#if !PX_P64_FAMILY
PX_COMPILE_TIME_ASSERT(48==sizeof(Constraint));
#endif

}

}

#endif
