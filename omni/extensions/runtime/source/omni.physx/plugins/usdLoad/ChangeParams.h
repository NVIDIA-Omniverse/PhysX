// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
class AttachedStage;

typedef bool (*OnUpdateObjectFn)(AttachedStage& attachedStage,
                                 ObjectId objectId,
                                 const pxr::TfToken&,
                                 const pxr::UsdTimeCode&);
typedef bool (*OnPrimRequirementCheckFn)(AttachedStage& attachedStage,
                                         const pxr::SdfPath&,
                                         const pxr::TfToken&,
                                         const pxr::UsdPrim* prim);
typedef bool (*OnPrimRequirementCheckExtFn)(AttachedStage& attachedStage,
                                            const pxr::SdfPath&,
                                            const pxr::TfToken&,
                                            const pxr::UsdPrim* prim,
                                            pxr::SdfPath& resyncPath);

struct ChangeParams
{
    std::string changeAttribute;
    OnPrimRequirementCheckFn onPrimCheck;
    OnPrimRequirementCheckExtFn onPrimCheckExt;
    OnUpdateObjectFn onUpdate;
};

} // namespace usdparser
} // namespace physx
} // namespace omni
