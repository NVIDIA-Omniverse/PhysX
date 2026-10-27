// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <private/omni/physics/schema/IUsdPhysics.h>
#include <private/omni/physx/IPhysxSupportUi.h>

namespace omni
{
namespace physx
{

struct PrimDef
{
    PXR_NS::UsdPrim mPrim;
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
