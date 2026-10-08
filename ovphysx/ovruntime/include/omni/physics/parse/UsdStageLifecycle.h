// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

#pragma once

#include <cstdint>
#include <memory>

namespace omni::physics::parse
{

// pxr-free interface the loadable USD library installs so the USD-free omni.physx side can drive
// the USD-stage/layer *lifecycle* operations the runtime-entry bridge needs -- UsdStage::Open /
// CreateNew / stage-cache insert-erase and SdfLayer::Find -- without linking or naming any pxr
// type (ADR-0027 bridge collapse). These back the IPhysxBenchmarks / Kit test surfaces
// (bridgeLoadTargetStage / bridgeCreateEmptyStage) and the Kit/USD-authoring sim-layer bind
// (bridgeSetSimulationLayer). Installed once by the USD backend loader; null in production, where
// nothing opens or caches a USD stage and the bridge entries fail closed (0 / no-op).
class IUsdStageLifecycle
{
public:
    virtual ~IUsdStageLifecycle() = default;

    // Find the SdfLayer named by `identifier` and assign it into the SimulationLayerHandle's
    // opaque SdfLayerRefPtr storage at `layerHandleStorage` (pre-constructed empty by the
    // simulation-layer ops table). An unknown identifier leaves it empty. Doing the Find and the
    // assign in one call keeps the found layer's reference alive across the hand-off.
    virtual void bindSimulationLayer(void* layerHandleStorage, const char* identifier) = 0;

    // Open the stage at `path`, insert it into the process UsdUtilsStageCache and return its new
    // cache id. Mirrors the USD arm's UsdStage::Open + Insert + GetId. `path` is never null.
    virtual uint64_t loadTargetStage(const char* path) = 0;

    // Create and cache an empty "default.usd" stage, returning its cache id.
    virtual uint64_t createEmptyStage() = 0;

    // Erase the stage cached under `stageId` from the process UsdUtilsStageCache. No-op when it is
    // not resident.
    virtual void eraseStage(uint64_t stageId) = 0;
};

// Install `lifecycle` as the single active stage-lifecycle backend, replacing any previous one.
// null clears it. Must be called detached (ADR-0005/0027).
void setUsdStageLifecycle(std::unique_ptr<IUsdStageLifecycle> lifecycle);

// The active stage-lifecycle backend, or null when none is installed.
IUsdStageLifecycle* usdStageLifecycle();

} // namespace omni::physics::parse
