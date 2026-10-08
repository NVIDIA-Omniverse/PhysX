// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

// Concrete USD backing-stage author (ADR-0027): the resident-backing-stage authoring fallbacks
// (default PhysicsScene create/remove, backing-stage data write). Lives inside the test-only USD
// library so it may name pxr types freely; the pxr-free AttachedStageBridge.cpp reaches it only
// through parse::IUsdBackingAuthor.

#include <omni/physics/usd/UsdBackingAuthor.h>

#include <omni/physics/usd/StageScan.h> // createDefaultPhysicsScene / removeDefaultPhysicsScene
#include <UsdPhysicsDataWrite.h>         // omni::physics::usd::UsdPhysicsDataWrite

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>

#include <memory>

namespace omni::physics::usd
{

namespace
{

// The synthetic default-scene placeholder's fixed literal identity, mirroring the note in
// omni.physx's OmniPhysX.h: the same literal LoadStage.cpp's stageless-default-scene fallback
// and PhysXStageUpdate.cpp's physXReset() inline. With only a backing stage in hand (no active
// source to resolve a scene ObjectKey), the author works this one canonical default-scene path.
constexpr const char* kDefaultPhysicsScenePath = "/PhysicsScene_16e12ee3daea";

const PXR_NS::UsdStageWeakPtr& stageAt(const void* handleStorage)
{
    return *reinterpret_cast<const PXR_NS::UsdStageWeakPtr*>(handleStorage);
}

class UsdBackingAuthor final : public parse::IUsdBackingAuthor
{
public:
    bool createDefaultScene(const void* handleStorage) override
    {
        const PXR_NS::UsdStageWeakPtr& stage = stageAt(handleStorage);
        if (!stage)
            return false;
        return !createDefaultPhysicsScene(stage, PXR_NS::SdfPath(kDefaultPhysicsScenePath)).IsEmpty();
    }

    bool removeDefaultScene(const void* handleStorage) override
    {
        const PXR_NS::UsdStageWeakPtr& stage = stageAt(handleStorage);
        if (!stage)
            return false;
        removeDefaultPhysicsScene(stage, PXR_NS::SdfPath(kDefaultPhysicsScenePath));
        return true;
    }

    std::unique_ptr<parse::IPhysicsDataWrite> makeBackingDataWrite(const void* handleStorage,
                                                                   const parse::IPhysicsSource* fallbackSource) override
    {
        const PXR_NS::UsdStageWeakPtr& stage = stageAt(handleStorage);
        if (!stage)
            return nullptr;
        // No UsdSource: the sink resolves its ObjectKeys/TokenIds through the active (non-USD)
        // source's string identity, threaded in as `fallbackSource` -- matching the _usd twin
        // makeBackingStageDataWrite (UsdPhysicsDataWrite(stage, nullptr, mSource.get())).
        return std::make_unique<UsdPhysicsDataWrite>(stage, /*source=*/nullptr, fallbackSource);
    }
};

} // namespace

std::unique_ptr<omni::physics::parse::IUsdBackingAuthor> makeUsdBackingAuthor()
{
    return std::make_unique<UsdBackingAuthor>();
}

} // namespace omni::physics::usd
