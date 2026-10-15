// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CUSTOM-CONTROL-001
// @covers AC-1 AC-2 AC-3
#include <omni/physx/NativeJointControl.h>
#include <omni/physx/PhysXRuntime.h>
#include "RuntimeExecution.h"
#include "PhysXCustomJoint.h"
#include "usdLoad/LoadUsd.h"
#include "usdLoad/AttachedStage.h"
namespace omni::physx::native
{
JointResult controlJoint(uint64_t attachment, const char* path, void* expectedContext,
    JointAction action, uint32_t& flags, const ParameterSpec* specs, uint32_t count,
    RefreshJoint refresh, void* context) noexcept
try
{
    if (!runtime::tryGetPhysxInterface() || !path || path[0] != '/' || !expectedContext ||
        uint32_t(action) > uint32_t(JointAction::Refresh) ||
        (action == JointAction::SetFlags && (flags & ~1u)) ||
        (action == JointAction::Refresh && (!refresh || (count && !specs)))) return JointResult::Invalid;
    std::shared_ptr<omni::physx::usdparser::AttachedStage> owner = usdparser::UsdLoad::getUsdLoad()->acquireAttachment(attachment);
    if (!owner) return JointResult::Stale;
    RuntimeExecutionScope admission(attachment);
    if (!admission) return JointResult::Busy;
    const omni::physics::parse::IPhysicsSource * source = owner->getSource();
    if (!source) return JointResult::NotFound;
    const omni::physics::parse::ObjectKey key = source->findByPath(path);
    const std::unordered_map<omni::physics::parse::ObjectKey, omni::physx::CustomPhysXJoint *, omni::physics::parse::ObjectKey::Hash>::iterator found = owner->customJoints.find(key);
    if (found == owner->customJoints.end() || found->second->callbackContext() != expectedContext)
        return JointResult::NotFound;
    ::physx::PxConstraint * constraint = found->second->getConstraint();
    if (action == JointAction::SetFlags) {
        constraint->setFlag(::physx::PxConstraintFlag::eALWAYS_UPDATE, (flags & 1u) != 0);
    } else if (action == JointAction::GetFlags) {
        flags = constraint->getFlags().isSet(::physx::PxConstraintFlag::eALWAYS_UPDATE) ? 1u : 0u;
    } else if (action == JointAction::MarkDirty) {
        constraint->markDirty();
    } else {
        std::vector<ParameterValue> values(count);
        uint32_t invalid;
        RuntimeCallbackScope callback(attachment, false, owner.get());
        if (!copyParameters(attachment, key.handle, specs, count, values.data(), invalid)) return JointResult::Invalid;
        if (!refresh(attachment, key.handle, values.data(), count, context)) return JointResult::Rejected;
        constraint->markDirty();
    }
    return JointResult::Success;
}
catch (...) { return JointResult::Rejected; }
}
