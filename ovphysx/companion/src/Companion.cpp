// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-BOUNDARY-001
// @covers AC-1 AC-2 AC-3 AC-4

// @implements REQ-COMP-ATTACH-COMPUTE-001
// @covers AC-1 AC-2 AC-3 AC-4

// @implements REQ-COMP-MASS-001
// @covers AC-1 AC-2 AC-3
// @implements REQ-COMP-API-001
// @covers AC-1 AC-2 AC-3 AC-4
// @implements REQ-COMP-OWNER-001
// @covers AC-1 AC-2 AC-3 AC-4
// @implements REQ-COMP-DEBUG-001
// @covers AC-2
// @implements REQ-CAPI-COMPANION-001
// @covers AC-1 AC-2 AC-3

#include "internal/sdk/CompanionRuntime.hpp"
#include "ovphysx_companion/companion.h"
#include "internal/sdk/ovphysxSDK.hpp"
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxStatistics.h>
#include <omni/physx/PhysXRuntime.h>
#include <limits>
#include <vector>
#include <omni/physx/AuthoringMass.h>
#include <omni/physx/AuthoringPoints.h>
#include <algorithm>

namespace ovphysx::companion::sessions
{
struct Session
{
    uint64_t owner;
    uint64_t attachment;
    std::shared_ptr<InstanceData> instance;

    Session(uint64_t owner_, const std::shared_ptr<InstanceData>& instance_)
        : owner(owner_), attachment(instance_->attachHandle), instance(instance_)
    {
        ++instance->companionLeases;
    }
    ~Session() { --instance->companionLeases; }
};

// Lock order: sessions, then g_instances_mutex. Never enter the runtime while
// holding either mutex. Each in-flight call retains its session lease until it
// returns, even when close removes its handle during a callback. The core external
// per-owner serialization contract
// remains in force (this registry mutex is not a runtime admission lock).

ovphysx_companion_status_t openSession(uint64_t owner, uint64_t* output) noexcept
{
    if (!output)
        return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try
    {
        std::lock_guard<std::mutex> sessionsLock(companionRuntime().sessions().mutex);
        std::unique_lock<std::shared_mutex> ownerLock(g_instances_mutex);
        const std::unordered_map<uint64_t, std::shared_ptr<InstanceData>>::iterator it = g_instances.find(owner);
        if (it == g_instances.end())
            return OVPHYSX_COMPANION_NOT_FOUND;
        std::shared_ptr<InstanceData> const & instance = it->second;
        if (!companionRuntime().admissionOpen() || instance->companionDestroying)
            return OVPHYSX_COMPANION_BUSY;
        if (!instance->ovstage_attached || !instance->attachHandle)
            return OVPHYSX_COMPANION_STALE;
        if (companionRuntime().sessions().nextHandle == std::numeric_limits<uint64_t>::max() ||
            instance->companionLeases == std::numeric_limits<uint32_t>::max())
            return OVPHYSX_COMPANION_ERROR;
        const uint64_t handle = companionRuntime().sessions().nextHandle;
        companionRuntime().sessions().entries.emplace(handle, std::make_shared<Session>(owner, instance));
        ++companionRuntime().sessions().nextHandle;
        *output = handle;
        return OVPHYSX_COMPANION_OK;
    }
    catch (...)
    {
        return OVPHYSX_COMPANION_ERROR;
    }
}

ovphysx_companion_status_t closeSession(uint64_t handle) noexcept
{
    try
    {
        std::lock_guard<std::mutex> sessionsLock(companionRuntime().sessions().mutex);
        std::map<uint64_t, std::shared_ptr<Session>>::iterator it = companionRuntime().sessions().entries.find(handle);
        if (it == companionRuntime().sessions().entries.end())
            return OVPHYSX_COMPANION_NOT_FOUND;
        companionRuntime().sessions().entries.erase(it);
        return OVPHYSX_COMPANION_OK;
    }
    catch (...)
    {
        return OVPHYSX_COMPANION_ERROR;
    }
}

std::shared_ptr<Session> findSession(uint64_t handle)
{
    std::lock_guard<std::mutex> lock(companionRuntime().sessions().mutex);
    const std::map<uint64_t, std::shared_ptr<Session>>::iterator it = companionRuntime().sessions().entries.find(handle);
    return it == companionRuntime().sessions().entries.end() ? nullptr : it->second;
}

ovphysx_companion_status_t validate(const Session& session)
{
    if (!session.instance->ovstage_attached || session.instance->attachHandle != session.attachment)
        return OVPHYSX_COMPANION_STALE;
    return OVPHYSX_COMPANION_OK;
}

ovphysx_companion_status_t getIdentity(uint64_t handle, ovphysx_companion_identity_t* output) noexcept
{
    if (!output)
        return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try
    {
        const std::shared_ptr<Session> session = findSession(handle);
        if (!session)
            return OVPHYSX_COMPANION_NOT_FOUND;
        const ovphysx_companion_status_t status = validate(*session);
        if (status != OVPHYSX_COMPANION_OK)
            return status;
        *output = {session->owner, session->attachment};
        return OVPHYSX_COMPANION_OK;
    }
    catch (...)
    {
        return OVPHYSX_COMPANION_ERROR;
    }
}

ovphysx_companion_status_t sceneStatistics(uint64_t handle, const char* path, uint32_t size,
                                         ovphysx_companion_scene_statistics_t* output) noexcept
{
    if (!output || !path || path[0] != '/' || size != sizeof(*output))
        return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try
    {
        CompanionOperationGuard admission(companionRuntime());
        if (!admission)
            return OVPHYSX_COMPANION_BUSY;
        const std::shared_ptr<Session> retained = findSession(handle);
        if (!retained)
            return OVPHYSX_COMPANION_NOT_FOUND;
        const Session& session = *retained;
        const ovphysx_companion_status_t status = validate(session);
        if (status != OVPHYSX_COMPANION_OK)
            return status;
        if (omni::physx::runtime::isRuntimeBusy(session.attachment))
            return OVPHYSX_COMPANION_BUSY;
        omni::physx::IPhysx * physics = omni::physx::runtime::tryGetPhysxInterface();
        omni::physx::IPhysxStatistics * statistics = omni::physx::runtime::tryGetPhysxStatisticsInterface();
        if (!physics || !statistics)
            return OVPHYSX_COMPANION_UNAVAILABLE;
        const omni::physics::parse::ObjectKey key = physics->resolveObjectKey(session.attachment, path);
        if (!key.valid())
            return OVPHYSX_COMPANION_NOT_FOUND;
        omni::physx::PhysicsSceneStats counters;
        if (!statistics->getPhysXSceneStatistics(session.attachment, key.handle, counters))
            return OVPHYSX_COMPANION_ERROR;
        *output = {sizeof(*output), 1, {session.owner, session.attachment},
                   counters.nbDynamicRigids, counters.nbActiveDynamicRigids, counters.nbStaticRigids,
                   counters.nbKinematicBodies, counters.nbArticulations, counters.nbActiveConstraints};
        return OVPHYSX_COMPANION_OK;
    }
    catch (...)
    {
        return OVPHYSX_COMPANION_ERROR;
    }
}
ovphysx_companion_status_t computeMass(const ovphysx_companion_mass_shape_t* shapes, uint32_t count,
    uint32_t size, ovphysx_companion_mass_result_t* output, uint32_t* invalidShape) noexcept
{
    if (!invalidShape) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    *invalidShape = UINT32_MAX;
    if (!output || !shapes || count == 0 || size != sizeof(*output))
        return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try
    {
        using namespace omni::physx::authoring;
        std::vector<MassShape> input;
        input.reserve(count);
        for (uint32_t i = 0; i < count; ++i)
        {
            const ovphysx_companion_mass_shape_t & s = shapes[i];
            MassShape item;
            item.kind = static_cast<PrimitiveKind>(s.kind);
            item.dimensions = ::physx::PxVec3(s.dimensions[0], s.dimensions[1], s.dimensions[2]);
            item.pose.p = ::physx::PxVec3(s.position[0], s.position[1], s.position[2]);
            item.pose.q = ::physx::PxQuat(s.rotation[0], s.rotation[1], s.rotation[2], s.rotation[3]);
            item.density = s.density;
            input.push_back(item);
        }
        MassResult result;
        if (!computePrimitiveMass(input.data(), count, result, *invalidShape))
            return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        ovphysx_companion_mass_result_t record{};
        record.size = sizeof(record);
        record.version = 1;
        record.mass = result.mass;
        record.volume = result.volume;
        for (uint32_t i = 0; i < 3; ++i)
        {
            record.center_of_mass[i] = result.centerOfMass[i];
            record.diagonal_inertia[i] = result.diagonalInertia[i];
            record.bounds_min[i] = result.bounds.minimum[i];
            record.bounds_max[i] = result.bounds.maximum[i];
        }
        record.principal_axes[0] = result.principalAxes.x;
        record.principal_axes[1] = result.principalAxes.y;
        record.principal_axes[2] = result.principalAxes.z;
        record.principal_axes[3] = result.principalAxes.w;
        *output = record;
        return OVPHYSX_COMPANION_OK;
    }
    catch (...) { return OVPHYSX_COMPANION_ERROR; }
}
ovphysx_companion_status_t matchPoints(const float* source, uint32_t sourceCount,
    const float* target, uint32_t targetCount, float distance, int32_t* output, uint32_t capacity) noexcept
{
    if ((!source && sourceCount) || (!target && targetCount) || (!output && sourceCount) || capacity != sourceCount)
        return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try
    {
        std::vector<::physx::PxVec3> a, b;
        a.reserve(sourceCount);
        b.reserve(targetCount);
        for (uint32_t i = 0; i < sourceCount; ++i)
            a.emplace_back(source[size_t(i)*3], source[size_t(i)*3+1], source[size_t(i)*3+2]);
        for (uint32_t i = 0; i < targetCount; ++i)
            b.emplace_back(target[size_t(i)*3], target[size_t(i)*3+1], target[size_t(i)*3+2]);
        std::vector<int32_t> matches;
        if (!omni::physx::authoring::matchPoints(a.data(), sourceCount, b.data(), targetCount, distance, matches))
            return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        if (sourceCount) std::copy(matches.begin(), matches.end(), output);
        return OVPHYSX_COMPANION_OK;
    }
    catch (...) { return OVPHYSX_COMPANION_ERROR; }
}
} // namespace

using namespace ovphysx::companion::sessions;
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_api_t* output)
{
    if (!output)
        return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if (version != OVPHYSX_COMPANION_ABI_VERSION || size != sizeof(*output))
        return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output = {sizeof(*output), OVPHYSX_COMPANION_ABI_VERSION,
               OVPHYSX_COMPANION_CAP_BORROWED_SESSION | OVPHYSX_COMPANION_CAP_SCENE_STATISTICS |
               OVPHYSX_COMPANION_CAP_PRIMITIVE_MASS | OVPHYSX_COMPANION_CAP_POINT_MATCHING,
               openSession, closeSession, getIdentity, sceneStatistics, computeMass, matchPoints};
    return OVPHYSX_COMPANION_OK;
}
