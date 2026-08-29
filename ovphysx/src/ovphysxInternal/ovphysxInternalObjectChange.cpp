// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// Sidecar object-change subscription bridge.
//
// Translates omni::physx's std::function-based IPhysicsObjectChangeCallback
// into the C-friendly function-pointer signatures declared in
// ovphysxInternalObjectChange.h. The IPhysx subscription registry copies the
// callback struct (including the std::functions) into its own storage when
// subscribeObjectChangeNotifications() is called, so the lambdas built here
// don't need to outlive the function.

#include "internal/sidecar/ovphysxInternalObjectChange.h"
#include "internal/sidecar/ovphysxInternalUtil.hpp"
#include "ovphysx/ovphysx.h"
#include "ovphysxInternalPhysXAccess.hpp"

#include <pxr/usd/sdf/path.h>  // must precede IPhysx.h: defines PXR_NS used by callback fn signatures

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
    // ABI (UINT64_MAX). NOT omni::physx::kInvalidSubscriptionId (0xFFffFFffFF) --
    // that's only 40 bits and could be confused with a real IPhysx ID by the
    // ovphysx-side check.
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
        // ovphysx has no Kit-style "simulation stopped" mode. omni.physx
        // gates notifications behind mObjectChangeNotificationsEnabled
        // UNLESS this flag is false; opt out of the gate so subscribers
        // receive events for every clone()/release()/reset() regardless of
        // omni.physx's internal simulation-state machine.
        callback.stopCallbackWhenSimStopped = false;

        if (on_created)
        {
            callback.objectCreationNotifyFn =
                [on_created](const PXR_NS::SdfPath& sdfPath, omni::physx::usdparser::ObjectId /*objectId*/,
                             omni::physx::PhysXType type, void* userData) {
                    const std::string& s = sdfPath.GetString();
                    on_created(s.data(), s.size(), static_cast<int>(type), userData);
                };
        }
        if (on_destroyed)
        {
            callback.objectDestructionNotifyFn =
                [on_destroyed](const PXR_NS::SdfPath& sdfPath, omni::physx::usdparser::ObjectId /*objectId*/,
                               omni::physx::PhysXType type, void* userData) {
                    const std::string& s = sdfPath.GetString();
                    on_destroyed(s.data(), s.size(), static_cast<int>(type), userData);
                };
        }
        if (on_all_destroyed)
        {
            callback.allObjectsDestructionNotifyFn =
                [on_all_destroyed](void* userData) { on_all_destroyed(userData); };
        }

        // Defensively translate the upstream sentinel: omni::physx today
        // never returns kInvalidSubscriptionId from a successful registry add,
        // but if it ever did (or if a future implementation uses it as a
        // failure signal), passing that 40-bit value through would let the
        // ovphysx side mistake it for a live subscription ID. Map it to our
        // kInvalid (UINT64_MAX) at the boundary.
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
