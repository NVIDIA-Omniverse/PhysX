// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once
#include "../MeshTypes.h"
#include <PxPhysicsAPI.h>
namespace omni
{
namespace physx
{
namespace graph
{
struct ImmediateShared;

struct ImmediateNarrowPhase
{
    static bool sameMatrices(const pxr::GfMatrix4d& mat0, const pxr::GfMatrix4d& mat1);
    static bool sameGeometryInSamePosition(const MeshGeometryData& g1, const MeshGeometryData& g2);
};

} // namespace graph
} // namespace physx
} // namespace omni
