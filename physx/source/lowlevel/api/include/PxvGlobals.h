// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXV_GLOBALS_H
#define PXV_GLOBALS_H

#include "PxPhysXConfig.h"
#include "foundation/PxBasicTemplates.h"
#include "PxContactModifyCallback.h"

namespace physx
{
class PxShape;
class PxRigidActor;
struct PxsShapeCore;
struct PxsRigidCore;

struct PxvOffsetTable
{
	PX_FORCE_INLINE	void fillPairPointers(	PxContactModifyPair& p,
											const PxsShapeCore* PX_RESTRICT shapeCore0, const PxsShapeCore* PX_RESTRICT shapeCore1,
											const PxsRigidCore* PX_RESTRICT rigidCore0, const PxsRigidCore* PX_RESTRICT rigidCore1,
											bool isDynamic0, bool isDynamic1)
	{
		p.shape[0] = PxPointerOffset<const PxShape*>(shapeCore0, pxsShapeCore2PxShape);
		p.shape[1] = PxPointerOffset<const PxShape*>(shapeCore1, pxsShapeCore2PxShape);

		p.actor[0] = PxPointerOffset<const PxRigidActor*>(rigidCore0, isDynamic0 ? pxsRigidCore2PxRigidBody : pxsRigidCore2PxRigidStatic);
		p.actor[1] = PxPointerOffset<const PxRigidActor*>(rigidCore1, isDynamic1 ? pxsRigidCore2PxRigidBody : pxsRigidCore2PxRigidStatic);
	}

	ptrdiff_t	pxsShapeCore2PxShape;
	ptrdiff_t	pxsRigidCore2PxRigidBody;
	ptrdiff_t	pxsRigidCore2PxRigidStatic;
};
extern PxvOffsetTable gPxvOffsetTable;

/*!
Initialize low-level implementation.
*/
void PxvInit(const PxvOffsetTable& offsetTable);

/*!
Shut down low-level implementation.
*/
void PxvTerm();

#if PX_SUPPORT_GPU_PHYSX
class PxPhysXGpu* PxvGetPhysXGpu(bool createIfNeeded);
void PxvReleasePhysXGpu(PxPhysXGpu*);
#endif

}

#endif
