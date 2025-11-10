// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

namespace omni
{
namespace physx
{

::physx::PxConvexMesh* createCylinderConvexMesh(const float width,
                                                const float radius,
                                                const uint32_t numCirclePoints,
                                                enum omni::physx::usdparser::Axis axis);

::physx::PxConvexMesh* createConeConvexMesh(const float width,
                                            const float radius,
                                            const uint32_t numCirclePoints,
                                            enum omni::physx::usdparser::Axis axis);

::physx::PxVec3 getConeOrCylinderScale(const ::physx::PxF32 halfWidth,
                                       const ::physx::PxF32 radius,
                                       enum omni::physx::usdparser::Axis axis);
void getConeOrCylinderSize(const ::physx::PxVec3& scale,
                           enum omni::physx::usdparser::Axis axis,
                           ::physx::PxF32& halfWidth,
                           ::physx::PxF32& radius);

} // namespace physx
} // namespace omni
