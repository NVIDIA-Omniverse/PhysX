// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physics/schema/IUsdPhysics.h>
#include <private/omni/physx/IPhysxSupportUi.h>

namespace omni
{
namespace physx
{

struct PrimDef
{
    pxr::UsdPrim mPrim;
    bool mForceCollCreation{ false };

    IPhysxSupportUi::ColliderType mColliderType{ IPhysxSupportUi::ColliderType::eAutodetect };
    IPhysxSupportUi::StaticColliderSimplificationType mStaticColliderSimplificationType{
        IPhysxSupportUi::StaticColliderSimplificationType::eNone
    };
    IPhysxSupportUi::DynamicColliderSimplificationType mDynamicColliderSimplificationType{
        IPhysxSupportUi::DynamicColliderSimplificationType::eConvexHull
    };
};

} // namespace physx
} // namespace omni
