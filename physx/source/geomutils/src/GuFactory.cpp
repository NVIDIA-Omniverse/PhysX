// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuFactory.h"
#include "GuAABBPruner.h"
#include "GuBucketPruner.h"
#include "GuIncrementalAABBPruner.h"

using namespace physx;
using namespace Gu;

Pruner* physx::Gu::createBucketPruner(PxU64 contextID)
{
	return PX_NEW(BucketPruner)(contextID);
}

Pruner* physx::Gu::createAABBPruner(PxU64 contextID, bool dynamic, CompanionPrunerType cpType, BVHBuildStrategy buildStrategy, PxU32 nbObjectsPerNode)
{
	return PX_NEW(AABBPruner)(dynamic, contextID, cpType, buildStrategy, nbObjectsPerNode);
}

Pruner* physx::Gu::createIncrementalPruner(PxU64 contextID)
{
	return PX_NEW(IncrementalAABBPruner)(32, contextID);
}

