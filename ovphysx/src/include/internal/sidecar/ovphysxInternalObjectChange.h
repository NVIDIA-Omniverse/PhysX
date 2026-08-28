// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "internal/sidecar/ovphysxInternalExport.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

// Sidecar object-change subscription bridge. Translates omni::physx's
// std::function-based IPhysicsObjectChangeCallback into the C-friendly
// function-pointer signatures below so the main library can forward user
// callbacks without depending on USD/omni.physx types.

extern "C" {

// C-compatible callback signatures matching ovphysx_object_change_callbacks_t.
// Defined here (rather than including ovphysx_types.h) so the sidecar stays
// self-contained and the C ABI surface remains thin.
//
// prim_path_ptr / prim_path_len: UTF-8 prim path (NOT null-terminated; use len).
//                                Storage owned by the sidecar, valid only for
//                                the callback duration.
// physx_type:                    omni::physx::PhysXType integer value.
// user_data:                     opaque caller pointer passed through unchanged.
typedef void (*ovphysx_internal_object_created_fn)(
    const char* prim_path_ptr, size_t prim_path_len, int physx_type, void* user_data);

typedef void (*ovphysx_internal_object_destroyed_fn)(
    const char* prim_path_ptr, size_t prim_path_len, int physx_type, void* user_data);

typedef void (*ovphysx_internal_all_objects_destroyed_fn)(void* user_data);

// Subscribe to omni::physx object change notifications.
//
// Any of the function-pointer parameters may be NULL; the sidecar skips NULL
// callbacks rather than invoking them. At least one of the three must be
// non-NULL (caller-side check; the sidecar itself does not validate).
//
// Returns the omni::physx SubscriptionId on success, or UINT64_MAX on failure
// (matching OVPHYSX_INVALID_SUBSCRIPTION_ID in the public ABI; intentionally
// not the omni::physx kInvalidSubscriptionId value, which is only 40 bits and
// would otherwise be a valid IPhysx ID on the ovphysx side). Pair with
// ovphysx_internal_unsubscribe_object_changes().
OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_subscribe_object_changes(
    ovphysx_internal_object_created_fn on_created,
    ovphysx_internal_object_destroyed_fn on_destroyed,
    ovphysx_internal_all_objects_destroyed_fn on_all_destroyed,
    void* user_data);

// Unsubscribe a previously-registered object-change subscription.
// Returns 0 on success, non-zero on failure (e.g. omni::physx unavailable).
OVPHYSX_INTERNAL_API int ovphysx_internal_unsubscribe_object_changes(
    uint64_t subscription_id);

// SDK-side function-pointer typedefs (mirror the above exports) for dlsym use.
typedef uint64_t (*OvphysxSidecarSubscribeObjectChangesFn)(
    ovphysx_internal_object_created_fn,
    ovphysx_internal_object_destroyed_fn,
    ovphysx_internal_all_objects_destroyed_fn,
    void* /* user_data */);
typedef int      (*OvphysxSidecarUnsubscribeObjectChangesFn)(uint64_t);

} // extern "C"

// Resolved sidecar function pointers (populated by loadInternalSidecar()).
extern std::atomic<OvphysxSidecarSubscribeObjectChangesFn>   g_sidecarSubscribeObjectChanges;
extern std::atomic<OvphysxSidecarUnsubscribeObjectChangesFn> g_sidecarUnsubscribeObjectChanges;
