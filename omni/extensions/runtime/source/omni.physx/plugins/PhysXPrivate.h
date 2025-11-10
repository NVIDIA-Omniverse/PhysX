// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <private/omni/physx/IPhysxPrivate.h>

namespace physx
{
class PxScene;
class PxCudaContextManager;
} // namespace physx

namespace omni
{
namespace physx
{

::physx::PxScene* privGetPhysXScene();
void primGetRigidBodyInstancedData(usdparser::ObjectId* ids, uint32_t numIds, InstancedData* data);
::physx::PxCudaContextManager* privGetCudaContextManager();

} // namespace physx
} // namespace omni
