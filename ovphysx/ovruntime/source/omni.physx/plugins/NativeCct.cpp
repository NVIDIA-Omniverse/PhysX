// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CCT-CONTROL-001
// @covers AC-1 AC-2 AC-3 AC-4
#include <omni/physx/NativeCct.h>
#include <omni/physx/PhysXRuntime.h>
#include "RuntimeExecution.h"
#include "usdLoad/LoadUsd.h"
#include "usdLoad/AttachedStage.h"
#include "usdInterface/UsdInterface.h"
#include "internal/InternalPhysXDatabase.h"
#include <PxPhysicsAPI.h>
#include <cmath>
#include <cstring>

namespace omni::physx::native
{
namespace
{
struct ControllerFilter : ::physx::PxControllerFilterCallback
{
    bool include;
    explicit ControllerFilter(bool value) : include(value) {}
    bool filter(const ::physx::PxController&, const ::physx::PxController&) override { return include; }
};
template<typename T> bool finite3(const T (&value)[3])
{
    return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}
}
CctResult controlCct(uint64_t attachment, const char* path, const CctCommand& command, CctState& output) noexcept
try
{
    using namespace ::physx;
    if (!path || path[0] != '/' || uint32_t(command.action) > uint32_t(CctAction::Remove) ||
        (command.action != CctAction::Describe && !command.generation))
        return CctResult::Invalid;
    if (command.action == CctAction::Move &&
        (!finite3(command.displacement) || !std::isfinite(command.dt) || command.dt <= 0 || command.dt > 1 ||
         !std::isfinite(command.minDistance) || command.minDistance < 0 || (command.filters & ~7u)))
        return CctResult::Invalid;
    if (command.action == CctAction::Teleport && !finite3(command.position))
        return CctResult::Invalid;
    if (command.action == CctAction::Resize &&
        (!std::isfinite(command.height) || !std::isfinite(command.radius) || command.height <= 0 || command.radius <= 0))
        return CctResult::Invalid;
    if (!runtime::tryGetPhysxInterface())
        return CctResult::Stale;
    std::shared_ptr<omni::physx::usdparser::AttachedStage> owner = usdparser::UsdLoad::getUsdLoad()->acquireAttachment(attachment);
    if (!owner)
        return CctResult::Stale;
    RuntimeExecutionScope admission(attachment);
    if (!admission)
        return CctResult::Busy;
    const omni::physics::parse::IPhysicsSource * source = owner->getSource();
    if (!source)
        return CctResult::NotFound;
    const omni::physics::parse::ObjectKey key = source->findByPath(path);
    omni::physx::usdparser::ObjectDb * objects = owner->getObjectDatabase();
    const uint64_t id = objects->findEntry(key, usdparser::eCct);
    omni::physx::internal::InternalPhysXDatabase & database = owner->getInternalPhysXDatabase();
    const omni::physx::internal::InternalDatabase::Record * record = database.getFullTypedRecord(ePTCct, id);
    if (!record || !record->mPtr || record->mKey != key)
        return command.generation ? CctResult::Stale : CctResult::NotFound;
    if (command.generation && command.generation != record->mGeneration)
        return CctResult::Stale;
    ::physx::PxController * controller = static_cast<PxController*>(record->mPtr);
    if (controller->getType() != PxControllerShapeType::eCAPSULE)
        return CctResult::Unavailable;
    ::physx::PxCapsuleController * capsule = static_cast<PxCapsuleController*>(controller);
    ::physx::PxScene * scene = capsule->getScene();
    if (!scene || scene->getFlags().isSet(PxSceneFlag::eENABLE_GPU_DYNAMICS) ||
        scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API))
        return CctResult::Unavailable;
    uint32_t collisions = 0;
    if (command.action == CctAction::Move)
    {
        ControllerFilter ccts((command.filters & 4u) != 0);
        PxControllerFilters filters(nullptr, nullptr, &ccts);
        filters.mFilterFlags = PxQueryFlags();
        if (command.filters & 1u) filters.mFilterFlags |= PxQueryFlag::eSTATIC;
        if (command.filters & 2u) filters.mFilterFlags |= PxQueryFlag::eDYNAMIC;
        RuntimeCallbackScope callbacks(attachment);
        collisions = uint32_t(capsule->move(PxVec3(command.displacement[0], command.displacement[1], command.displacement[2]),
                                           command.minDistance, command.dt, filters));
    }
    else if (command.action == CctAction::Teleport)
    {
        if (!capsule->setPosition(PxExtendedVec3(command.position[0], command.position[1], command.position[2])))
            return CctResult::Error;
    }
    else if (command.action == CctAction::Resize)
    {
        const ::physx::PxVec3T<double> foot = capsule->getFootPosition();
        if (!capsule->setRadius(command.radius))
            return CctResult::Error;
        capsule->resize(command.height);
        capsule->setFootPosition(foot);
    }
    else if (command.action == CctAction::Remove)
    {
        owner->getPhysXPhysicsInterface()->releaseObject(*owner, key, id);
        objects->removeEntry(key, usdparser::eCct, id);
        output = {};
        return CctResult::Success;
    }
    CctState result;
    result.key = key.handle;
    result.generation = record->mGeneration;
    const ::physx::PxVec3T<double> position = capsule->getPosition();
    result.position[0] = position.x; result.position[1] = position.y; result.position[2] = position.z;
    const ::physx::PxVec3T<float> up = capsule->getUpDirection(), gravity = scene->getGravity();
    result.up[0] = up.x; result.up[1] = up.y; result.up[2] = up.z;
    result.gravity[0] = gravity.x; result.gravity[1] = gravity.y; result.gravity[2] = gravity.z;
    result.height = capsule->getHeight(); result.radius = capsule->getRadius(); result.collisions = collisions;
    output = result;
    return CctResult::Success;
}
catch (...) { return CctResult::Error; }
}
