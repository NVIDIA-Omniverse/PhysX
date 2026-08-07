// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include <omni/physics/parse/Handles.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;

typedef bool (*OnUpdateObjectFn)(AttachedStage& attachedStage,
                                 ObjectId objectId,
                                 const PXR_NS::TfToken&,
                                 const PXR_NS::UsdTimeCode&);
typedef bool (*OnPrimRequirementCheckFn)(AttachedStage& attachedStage,
                                         const PXR_NS::SdfPath&,
                                         const PXR_NS::TfToken&,
                                         const PXR_NS::UsdPrim* prim);
typedef bool (*OnPrimRequirementCheckExtFn)(AttachedStage& attachedStage,
                                            const PXR_NS::SdfPath&,
                                            const PXR_NS::TfToken&,
                                            const PXR_NS::UsdPrim* prim,
                                            PXR_NS::SdfPath& resyncPath);
typedef bool (*OnPrimRequirementKeyCheckFn)(AttachedStage& attachedStage,
                                            omni::physics::parse::ObjectKey,
                                            const PXR_NS::TfToken&);

struct ChangeParams
{
    std::string changeAttribute;
    OnPrimRequirementCheckFn onPrimCheck;
    OnPrimRequirementCheckExtFn onPrimCheckExt;
    OnUpdateObjectFn onUpdate;
    OnPrimRequirementKeyCheckFn onPrimCheckKey = nullptr;
};

} // namespace usdparser
} // namespace physx
} // namespace omni
