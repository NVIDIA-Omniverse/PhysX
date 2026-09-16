// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OBJECTKEY-001
 * @covers AC-3 AC-4
 *
 * @implements REQ-CAPI-STRING-001
 * @covers AC-4
 */

// Sidecar object-change subscription bridge.
//
// Translates omni::physx's std::function-based IPhysicsObjectChangeCallback
// into the C-friendly function-pointer signatures declared in
// ovphysxInternalObjectChange.h. The IPhysx subscription registry copies the
// callback struct (including the std::functions) into its own storage when
// subscribeObjectChangeNotifications() is called, so the lambdas built here
// don't need to outlive the function.

#include "internal/sidecar/ovphysxInternalObjectChange.h"
#include "ovphysx/ovphysx.h"
#include "ovphysxInternalPhysXAccess.hpp"

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

#include <cstdint>
#include <exception>
#include <string>

OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_subscribe_object_changes(
    ovphysx_internal_object_created_fn on_created,
    ovphysx_internal_object_destroyed_fn on_destroyed,
    ovphysx_internal_all_objects_destroyed_fn on_all_destroyed,
    void* user_data)
{
    // Failure sentinel matches OVPHYSX_INVALID_SUBSCRIPTION_ID in the public
    // ABI (UINT64_MAX), not omni::physx::kInvalidSubscriptionId (0xFFffFFffFF).
    // The latter is only 40 bits and could be confused with a real IPhysx ID by
    // the ovphysx-side check.
    constexpr uint64_t kInvalid = UINT64_MAX;

    try
    {
        omni::physx::IPhysx* physx = ovphysx::internal::sidecar::tryGetInjectedPhysxInterface();
        if (!physx)
        {
            CARB_LOG_ERROR("Internal sidecar: ovphysx_internal_subscribe_object_changes failed to resolve IPhysx runtime interface");
            return kInvalid;
        }

        omni::physx::IPhysicsObjectChangeCallback callback;
        callback.userData = user_data;
        // ovphysx has no Kit-style "simulation stopped" mode. omni.physx gates
        // notifications behind mObjectChangeNotificationsEnabled unless this
        // flag is false, so the gate is opted out and subscribers receive events
        // for every clone()/release()/reset() regardless of omni.physx's internal
        // simulation state.
        callback.stopCallbackWhenSimStopped = false;

        if (on_created)
        {
            callback.objectCreationNotifyFn =
                [physx, on_created](omni::physics::parse::ObjectKey key, omni::physx::usdparser::ObjectId /*objectId*/,
                                     omni::physx::PhysXType type, void* userData) {
                    const std::string s(physx->objectKeyToPath(key));
                    on_created(s.data(), s.size(), static_cast<int>(type), userData);
                };
        }
        if (on_destroyed)
        {
            callback.objectDestructionNotifyFn =
                [physx, on_destroyed](omni::physics::parse::ObjectKey key, omni::physx::usdparser::ObjectId /*objectId*/,
                                       omni::physx::PhysXType type, void* userData) {
                    const std::string s(physx->objectKeyToPath(key));
                    on_destroyed(s.data(), s.size(), static_cast<int>(type), userData);
                };
        }
        if (on_all_destroyed)
        {
            callback.allObjectsDestructionNotifyFn =
                [on_all_destroyed](void* userData) { on_all_destroyed(userData); };
        }

        // omni::physx does not return kInvalidSubscriptionId from a successful
        // registry add, but passing that 40-bit value through would let the
        // ovphysx side mistake it for a live subscription ID. Map it to kInvalid
        // (UINT64_MAX) at the boundary.
        const omni::physx::SubscriptionId id = physx->subscribeObjectChangeNotifications(callback);
        if (id == omni::physx::kInvalidSubscriptionId)
            return kInvalid;
        return static_cast<uint64_t>(id);
    }
    catch (const std::exception& e)
    {
        CARB_LOG_ERROR("Internal sidecar: ovphysx_internal_subscribe_object_changes caught exception: %s", e.what());
        return kInvalid;
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: ovphysx_internal_subscribe_object_changes caught unknown exception");
        return kInvalid;
    }
}

OVPHYSX_INTERNAL_API int ovphysx_internal_unsubscribe_object_changes(
    uint64_t subscription_id)
{
    try
    {
        omni::physx::IPhysx* physx = ovphysx::internal::sidecar::tryGetInjectedPhysxInterface();
        if (!physx)
        {
            CARB_LOG_ERROR("Internal sidecar: ovphysx_internal_unsubscribe_object_changes failed to resolve IPhysx runtime interface");
            return 1;
        }

        physx->unsubscribeObjectChangeNotifications(static_cast<omni::physx::SubscriptionId>(subscription_id));
        return 0;
    }
    catch (const std::exception& e)
    {
        CARB_LOG_ERROR("Internal sidecar: ovphysx_internal_unsubscribe_object_changes caught exception: %s", e.what());
        return 1;
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: ovphysx_internal_unsubscribe_object_changes caught unknown exception");
        return 1;
    }
}
