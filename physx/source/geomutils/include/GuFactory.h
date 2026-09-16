// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_FACTORY_H
#define GU_FACTORY_H

#include "foundation/PxSimpleTypes.h"
#include "common/PxPhysXCommonConfig.h"
#include "GuPrunerTypedef.h"

namespace physx
{
namespace Gu
{
	class Pruner;

	PX_C_EXPORT	PX_PHYSX_COMMON_API	Gu::Pruner*	createBucketPruner(PxU64 contextID);
	PX_C_EXPORT	PX_PHYSX_COMMON_API	Gu::Pruner*	createAABBPruner(PxU64 contextID, bool dynamic, Gu::CompanionPrunerType type, Gu::BVHBuildStrategy buildStrategy, PxU32 nbObjectsPerNode);
	PX_C_EXPORT	PX_PHYSX_COMMON_API	Gu::Pruner*	createIncrementalPruner(PxU64 contextID);
}
}

#endif
