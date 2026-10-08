// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2 AC-3
 *
 * @implements REQ-BUILD-BRIDGE-001
 * @covers AC-3
 */

#pragma once

namespace omni::physics::parse
{

// USD-free operation table for a single USD smart-pointer type stored in opaque byte
// storage (ADR-0027 seam #3). The USD library supplies a filled table over a concrete
// handle (UsdStageWeakPtr, SdfLayerRefPtr); omni.physx manipulates the storage only
// through these hooks and never names the handle type. `storage` always points at
// caller-owned bytes sized/aligned for the concrete handle (asserted in the USD TU).
struct OpaquePxrHandleOps
{
    // Placement-new an empty handle into `storage`.
    void  (*construct)(void* storage) noexcept;
    // Placement-new a copy of `src` into `dst`.
    void  (*copyConstruct)(void* dst, const void* src) noexcept;
    // Copy-assign `src` onto an already-constructed `dst`.
    void  (*copyAssign)(void* dst, const void* src) noexcept;
    // Run the handle destructor on `storage`.
    void  (*destroy)(void* storage) noexcept;
    // Truthiness of the handle (non-null / valid).
    bool  (*toBool)(const void* storage) noexcept;
    // Raw pointee pointer, or null. Optional (null for handles without one).
    void* (*rawPtr)(const void* storage) noexcept;
    // Rebind the handle to own/reference `rawObject`. Optional (null when the
    // handle cannot be built from a raw pointer).
    void  (*adoptRaw)(void* storage, void* rawObject) noexcept;
};

// Process-global slots for the two handle kinds omni.physx stores opaquely: the attached
// UsdStage (weak) and the simulation SdfLayer (ref). Same registry shape as ScanBackend.h.
//
// Lifetime contract: a table selects the storage's representation, so once installed it must
// stay installed, and unchanged, for the lifetime of every handle owner -- install before the
// first owner exists (before omni::physx::runtime::startup()) and never clear or swap while the
// process lives. The tables are stateless, so re-storing the same pointer is harmless. The
// functional seams (parse/scan backends, reparse, authors, stage lifecycle) toggle
// independently of these. Production never installs a table: every handle there is a zeroed
// POD for its whole life.
void setAttachedStageUsdHandleOps(const OpaquePxrHandleOps*) noexcept;
const OpaquePxrHandleOps* attachedStageUsdHandleOps() noexcept;
void setSimulationLayerHandleOps(const OpaquePxrHandleOps*) noexcept;
const OpaquePxrHandleOps* simulationLayerHandleOps() noexcept;

} // namespace omni::physics::parse
