// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-NATIVE-ABI-001
// @covers AC-1 AC-5
// @implements REQ-COMP-JOINT-001
// @covers AC-5
// @implements REQ-COMP-GEOMETRY-001
// @covers AC-5
// @implements REQ-CAPI-COMPANION-001
// @covers AC-1 AC-4 AC-5
#include <ovphysx_companion/custom_interfaces.h>
#include "internal/sdk/ovphysxSDK.hpp"
#include "internal/sdk/CompanionRuntime.hpp"
#include <omni/physx/PhysXRuntime.h>
#include <limits>

namespace ovphysx::companion::custom_interfaces
{
struct Lease
{
    CompanionServiceLease serviceLease{companionRuntime()};
    std::shared_ptr<ovphysx::CarboniteLoader> services;
    explicit Lease(std::shared_ptr<ovphysx::CarboniteLoader> owner) : services(std::move(owner)) {}
};
}

namespace sdk = ovphysx::companion::native;
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_custom_interfaces_acquire(
    uint64_t source, const sdk::Compatibility* compatibility, uint32_t compatibilitySize,
    uint32_t outputSize, sdk::CustomInterfaces* output)
{
    if (!compatibility || !output)
        return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if (compatibilitySize != sizeof(*compatibility) || outputSize != sizeof(*output) ||
        !sdk::compatibleBuild(*compatibility))
        return OVPHYSX_COMPANION_ABI_MISMATCH;
    try
    {
        CompanionOperationGuard admission(companionRuntime());
        if (!admission)
            return OVPHYSX_COMPANION_BUSY;
        std::shared_ptr<ovphysx::CarboniteLoader> services;
        {
            std::shared_lock<std::shared_mutex> lock(g_instances_mutex);
            const std::unordered_map<uint64_t, std::shared_ptr<InstanceData>>::const_iterator found = g_instances.find(source);
            if (found == g_instances.end())
                return OVPHYSX_COMPANION_NOT_FOUND;
            if (!companionRuntime().admissionOpen() || found->second->companionDestroying)
                return OVPHYSX_COMPANION_BUSY;
            services = found->second->carbonite;
        }
        if (!services)
            return OVPHYSX_COMPANION_UNAVAILABLE;
        omni::physx::IPhysxCustomJoint* joints = omni::physx::runtime::tryGetPhysxCustomJointInterface();
        omni::physx::IPhysxCustomGeometry* geometry = omni::physx::runtime::tryGetPhysxCustomGeometryInterface();
        if (!joints || !geometry)
            return OVPHYSX_COMPANION_UNAVAILABLE;
        CompanionRegistry<ovphysx::companion::custom_interfaces::Lease>& registry = companionRuntime().customInterfaces();
        std::lock_guard<std::mutex> lock(registry.mutex);
        if (registry.nextHandle == std::numeric_limits<uint64_t>::max())
            return OVPHYSX_COMPANION_ERROR;
        const uint64_t handle = registry.nextHandle;
        registry.entries.emplace(handle, std::make_shared<ovphysx::companion::custom_interfaces::Lease>(std::move(services)));
        ++registry.nextHandle;
        *output = {sizeof(*output), 1, handle, joints, geometry};
        return OVPHYSX_COMPANION_OK;
    }
    catch (...) { return OVPHYSX_COMPANION_ERROR; }
}
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_custom_interfaces_release(uint64_t handle)
{
    try
    {
        CompanionOperationGuard admission(companionRuntime());
        if (!admission || omni::physx::runtime::isRuntimeBusy(0))
            return OVPHYSX_COMPANION_BUSY;
        CompanionRegistry<ovphysx::companion::custom_interfaces::Lease>& registry = companionRuntime().customInterfaces();
        std::shared_ptr<ovphysx::companion::custom_interfaces::Lease> retained;
        {
            std::lock_guard<std::mutex> lock(registry.mutex);
            const std::map<uint64_t, std::shared_ptr<ovphysx::companion::custom_interfaces::Lease>>::iterator found = registry.entries.find(handle);
            if (found == registry.entries.end())
                return OVPHYSX_COMPANION_NOT_FOUND;
            retained = std::move(found->second);
            registry.entries.erase(found);
        }
        return OVPHYSX_COMPANION_OK;
    }
    catch (...) { return OVPHYSX_COMPANION_ERROR; }
}
