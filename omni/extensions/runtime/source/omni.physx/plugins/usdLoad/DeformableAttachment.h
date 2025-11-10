// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <private/omni/physx/PhysxUsd.h>

namespace omni
{
namespace physx
{
namespace usdparser
{

PhysxDeformableAttachmentDesc* parseDeformableAttachment(const pxr::UsdStageWeakPtr stage,
                                                         const omni::physics::schema::AttachmentDesc& inDesc);
PhysxDeformableCollisionFilterDesc* parseDeformableCollisionFilter(
    const pxr::UsdStageWeakPtr stage, const omni::physics::schema::ElementCollisionFilterDesc& inDesc);
} // namespace usdparser
} // namespace physx
} // namespace omni
