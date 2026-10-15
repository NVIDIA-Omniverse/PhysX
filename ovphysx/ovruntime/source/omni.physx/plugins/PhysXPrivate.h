// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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

::physx::PxScene* privGetPhysXScene(omni::physics::AttachHandle attach);
void primGetRigidBodyInstancedData(omni::physics::AttachHandle attach, usdparser::ObjectId* ids, uint32_t numIds, InstancedData* data);
::physx::PxCudaContextManager* privGetCudaContextManager(omni::physics::AttachHandle attach, uint64_t sceneKey);

} // namespace physx
} // namespace omni
