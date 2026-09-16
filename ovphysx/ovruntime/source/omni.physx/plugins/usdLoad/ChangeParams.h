// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

// OnPrimRequirementKeyCheckFn/OnPrimRequirementExtKeyCheckFn are the ObjectKey/TokenId-native
// requirement-check function types. checkPrimChange(ObjectKey, TokenId) (PrimUpdate.cpp) is the
// sole dispatch path for every source, including a genuine USD attach, so no SdfPath-typed
// twin (OnPrimRequirementCheckFn/OnPrimRequirementCheckExtFn) exists any more.

#include <private/omni/physx/PhysxUsd.h>
#include <omni/physics/parse/Handles.h>
#include <omni/physics/parse/IPhysicsSource.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;

typedef bool (*OnUpdateObjectFn)(AttachedStage& attachedStage,
                                 ObjectId objectId,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime);
typedef bool (*OnPrimRequirementKeyCheckFn)(AttachedStage& attachedStage,
                                            omni::physics::parse::ObjectKey,
                                            omni::physics::parse::TokenId);

// resyncKey reports the resync target, mirroring isNonMovable's own
// ObjectKey&-out-param overload (PrimUpdate.cpp).
typedef bool (*OnPrimRequirementExtKeyCheckFn)(AttachedStage& attachedStage,
                                               omni::physics::parse::ObjectKey,
                                               omni::physics::parse::TokenId,
                                               omni::physics::parse::ObjectKey& resyncKey);

struct ChangeParams
{
    std::string changeAttribute;
    OnUpdateObjectFn onUpdate;
    OnPrimRequirementKeyCheckFn onPrimCheckKey = nullptr;
    OnPrimRequirementExtKeyCheckFn onPrimCheckExtKey = nullptr;
};

} // namespace usdparser
} // namespace physx
} // namespace omni
