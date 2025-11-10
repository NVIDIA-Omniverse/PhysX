// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "SceneQueryShared.h"
#include <omni/fabric/FabricTypes.h>

namespace omni
{
namespace physx
{
namespace graph
{

static omni::physx::IPhysxSceneQuery* gPhysXSceneQuery;
   
omni::physx::IPhysxSceneQuery* getPhysXSceneQuery()
{
    return gPhysXSceneQuery;
}

void setPhysXSceneQuery(omni::physx::IPhysxSceneQuery* pPhysXSceneQuery)
{
    gPhysXSceneQuery = pPhysXSceneQuery;
}

} // namespace graph
} // namespace physx
} // namespace omni
