// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-NATIVE-ABI-001
// @covers AC-1 AC-5
// @implements REQ-COMP-JOINT-001
// @covers AC-5
// @implements REQ-COMP-GEOMETRY-001
// @covers AC-5
#pragma once
// Optional C++17 interface access: requires the matching PhysX SDK headers.
// Neither the core nor ordinary companion headers include this file.
#include "companion.h"
#include "native_compatibility.h"
#include <omni/physx/IPhysxCustomJoint.h>
#include <omni/physx/IPhysxCustomGeometry.h>

namespace ovphysx::companion::native
{
// The original runtime tables, with unchanged types and callback signatures.
// The lease retains the already loaded runtime; it creates no scene/attachment.
// Registration is shared by that runtime. Every object callback receives its
// exact AttachHandle and ObjectKey. Schedule live controls at an idle owner.
struct CustomInterfaces
{
    uint32_t size;
    uint32_t version;
    uint64_t lease;
    omni::physx::IPhysxCustomJoint* joints;
    omni::physx::IPhysxCustomGeometry* geometry;
};
}

// Acquire before attaching scenes that use custom types. The bootstrap instance
// must exist, but need not have a stage. Failed acquisition preserves output.
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_custom_interfaces_acquire(
    uint64_t sourceInstance, const ovphysx::companion::native::Compatibility* compatibility,
    uint32_t compatibilitySize, uint32_t outputSize,
    ovphysx::companion::native::CustomInterfaces* output);

// First retire every registration with the original tryUnregister operations.
// Keep callback code/context alive until those return true. Do not retain table
// pointers after releasing this lease. BUSY retains the lease; complete pending
// work and retry. Releasing an unknown/already released lease returns NOT_FOUND.
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_custom_interfaces_release(uint64_t lease);
