// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

// Concrete USD stage-lifecycle backend (ADR-0027): UsdStage::Open / CreateNew, the process
// UsdUtilsStageCache, and SdfLayer::Find. Lives inside the test-only USD library so it may name
// pxr types freely; the pxr-free RuntimeBridge.cpp (bridgeLoadTargetStage / bridgeCreateEmptyStage
// / bridgeSetSimulationLayer) reaches it only through parse::IUsdStageLifecycle.

#include <omni/physics/usd/UsdStageLifecycle.h>

#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/stageCache.h>
#include <pxr/usd/usdUtils/stageCache.h>

#include <memory>

namespace omni::physics::usd
{

namespace
{

class UsdStageLifecycle final : public parse::IUsdStageLifecycle
{
public:
    void bindSimulationLayer(void* layerHandleStorage, const char* identifier) override
    {
        // Find() misses (returns a null ref) on an unknown identifier; assign the result -- null
        // or not -- INTO the already-live SdfLayerRefPtr the layer ops table pre-constructed at
        // handleStorage, never a fresh placement-new (which would leak that one). Doing Find and
        // assign here keeps the found layer referenced across the hand-off.
        *reinterpret_cast<PXR_NS::SdfLayerRefPtr*>(layerHandleStorage) = PXR_NS::SdfLayer::Find(identifier);
    }

    uint64_t loadTargetStage(const char* path) override
    {
        // 0 is the "no stage" sentinel; never insert a null stage into the cache.
        PXR_NS::UsdStageRefPtr stage = path ? PXR_NS::UsdStage::Open(path) : PXR_NS::UsdStageRefPtr();
        if (!stage)
            return 0;
        return static_cast<uint64_t>(PXR_NS::UsdUtilsStageCache::Get().Insert(stage).ToLongInt());
    }

    uint64_t createEmptyStage() override
    {
        PXR_NS::UsdStageRefPtr stage = PXR_NS::UsdStage::CreateNew("default.usd");
        if (!stage)
            return 0;
        return static_cast<uint64_t>(PXR_NS::UsdUtilsStageCache::Get().Insert(stage).ToLongInt());
    }

    void eraseStage(uint64_t stageId) override
    {
        PXR_NS::UsdUtilsStageCache::Get().Erase(PXR_NS::UsdStageCache::Id::FromLongInt(static_cast<long>(stageId)));
    }
};

} // namespace

std::unique_ptr<omni::physics::parse::IUsdStageLifecycle> makeUsdStageLifecycle()
{
    return std::make_unique<UsdStageLifecycle>();
}

} // namespace omni::physics::usd
