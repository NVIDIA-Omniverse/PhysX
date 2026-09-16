// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-15 AC-16
 *
 * @implements REQ-PUBLICAPI-002
 * @covers AC-10
 */

#pragma once

#include <carb/Types.h>

#include <omni/physics/parse/Handles.h>
#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxCookingPrivate.h>

namespace omni
{
namespace physx
{

// IPhysxCooking

uint32_t getNbConvexMeshData(omni::physics::parse::ObjectKey key);
void getConvexMeshData(omni::physics::parse::ObjectKey key, uint32_t convexIndex, ConvexMeshData& meshData);
bool createConvexMesh(omni::physics::parse::ObjectKey key, uint32_t vertexLimit, ConvexMeshData& meshData);

uint32_t getNumCollisionTasks(void);
uint32_t cancelCollisionTasks(void);
void releaseLocalMeshCache();
uint32_t getTotalFinishedCollisionTasks(void);

bool cookAutoDeformableBody(omni::physics::parse::ObjectKey deformableBodyKey);

void addPrimToCookingRefreshSetForAttach(omni::physics::parse::ObjectKey key, omni::physics::AttachHandle attachHandle);

//~IPhysxCooking

// IPhysxCookingPrivate
PhysxCookingStatistics getCookingStatistics();
// ~IPhysxCookingPrivate


} // namespace physx
} // namespace omni
