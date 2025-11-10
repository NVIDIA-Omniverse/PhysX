// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Types.h>

#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxCookingPrivate.h>

namespace omni
{
namespace physx
{

// IPhysxCooking deprecated
bool cookDeformableBodyMeshDeprecated(const pxr::SdfPath& deformableBodyPath);

// IPhysxCooking

uint32_t getNbConvexMeshData(const pxr::SdfPath& path);
void getConvexMeshData(const pxr::SdfPath& path, uint32_t convexIndex, ConvexMeshData& meshData);
bool createConvexMesh(const pxr::SdfPath& path, uint32_t vertexLimit, ConvexMeshData& meshData);

uint32_t getNumCollisionTasks(void);
uint32_t cancelCollisionTasks(void);
void releaseLocalMeshCache();
uint32_t getTotalFinishedCollisionTasks(void);

bool cookAutoDeformableBody(const pxr::SdfPath& deformableBodyPath);

void addPrimToCookingRefreshSet(const pxr::SdfPath& path);

//~IPhysxCooking

// IPhysxCookingPrivate
PhysxCookingStatistics getCookingStatistics();
// ~IPhysxCookingPrivate


} // namespace physx
} // namespace omni
