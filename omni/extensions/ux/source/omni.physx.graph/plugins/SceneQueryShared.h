// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "GraphShared.h"
#include <carb/Types.h>

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSceneQuery.h>
#include <private/omni/physx/IPhysxGraph.h>

#include <PxPhysicsAPI.h>

#include <common/foundation/TypeCast.h>
#include <omni/graph/core/OgnHelpers.h>
#include <omni/fabric/FabricUSD.h>

namespace omni
{
namespace physx
{
namespace graph
{

// Gets pointer to stored IPhysxSceneQuery object.
omni::physx::IPhysxSceneQuery* getPhysXSceneQuery();

// Sets pointer to stored IPhysxSceneQuery object.
void setPhysXSceneQuery(omni::physx::IPhysxSceneQuery* pPhysXSceneQuery);

inline bool sortHitByDistance(SceneQueryHitLocation& hit1, SceneQueryHitLocation& hit2)
{
    return hit1.distance < hit2.distance;
}

} // namespace graph
} // namespace physx
} // namespace omni
