// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-LIFETIME-001
// @covers AC-1 AC-2

/**
 * @implements REQ-RUNTIME-ERROR-001
 * @covers AC-3
 *
 * @implements REQ-BUILD-UNIBUILD-001
 * @covers AC-3
 *
 * @implements REQ-BUILD-BRIDGE-001
 * @covers AC-1 AC-2 AC-3 AC-4
 */

// pxr-free runtime-entry bridge (ADR-0027): names no pxr type, includes no pxr header, calls
// no omni::physics::usd:: symbol. With no seam installed (production / USD-free) every entry
// fails closed; a USD-loaded process reaches USD data only through the installed seams -- the
// reparse backend (omni::physics::parse::usdReparse()), the stage-lifecycle backend
// (omni::physics::parse::usdStageLifecycle()), the registered parse backend, and the opaque
// pxr-handle op table for the SimulationLayerHandle
// (omni::physics::parse::simulationLayerHandleOps(), ADR-0027 seam #3) -- never through a
// direct USD call from here.

#include <omni/physx/RuntimeError.h>
#include "OmniPhysX.h"
#include "Setup.h"
#include "PhysXReplicator.h"
#include "usdBridge/RuntimeBridge.h"
#include "utils/PrimPathGrammar.h"
#include <omni/physics/parse/OpaquePxrHandleOps.h> // parse::simulationLayerHandleOps (seam #3)
#include "usdLoad/LoadUsd.h"
#include "usdLoad/AttachedStage.h"
#include "usdLoad/PrimUpdate.h"
#include "RuntimeExecution.h"
#include "usdInterface/UsdInterface.h"

#include <omni/physics/parse/IParseBackend.h>   // parseBackend / AttachTarget / SourceBundle
#include <omni/physics/parse/IPhysicsSource.h>  // IPhysicsSource / SourceUnits
#include <omni/physics/parse/UsdReparse.h>      // parse::usdReparse
#include <omni/physics/parse/UsdStageLifecycle.h> // parse::usdStageLifecycle
#include <omni/physx/IPhysxSettings.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>

namespace omni
{
namespace physx
{

// UsdLoad / AttachedStage / AttachedStageUsdHandle live in omni::physx::usdparser; mirror the
// _usd arm's using-directive so those names resolve unqualified here.
using namespace omni::physx::usdparser;

AttachHandle OmniPhysX::physXAttach(long int stageId, bool loadPhysics, const PhysxAttachOptions& options)
{
    // Resource acquisition belongs to the private candidate, just as on ovstage.
    // No active database, clock or SDK is prepared through a throwaway source.
    getPhysXUsdPhysicsInterface().setExposePrimNames(mISettings->getAsBool(kSettingExposePrimPathNames));
    const char* singleScene = mISettings->getStringBuffer(kSettingForceParseOnlySingleScene);
    getPhysXUsdPhysicsInterface().setForceParseOnlySingleScene(singleScene ? singleScene : std::string());
    return UsdLoad::getUsdLoad()->attach(loadPhysics, stageId, &getPhysXUsdPhysicsInterface(), options);
}

long bridgeLoadTargetStage(const char* path, AttachHandle& attach)
{
    RuntimeExecutionScope configuration(kNoAttach);
    if (!configuration)
        return 0;
    std::unique_ptr<RuntimeExecutionScope> ownerOperation;
    if (attach != kNoAttach)
    {
        ownerOperation = std::make_unique<RuntimeExecutionScope>(attach, true);
        if (!*ownerOperation)
            return 0;
    }
    omni::physics::parse::IUsdStageLifecycle* lifecycle = omni::physics::parse::usdStageLifecycle();
    if (!lifecycle)
        return 0;
    if (AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach))
    {
        const uint64_t previousStageId = owner->getStageId();
        if (!OmniPhysX::getInstance().physXDetach(attach))
            return 0;
        lifecycle->eraseStage(previousStageId);
    }
    attach = kNoAttach;
    if (!path)
        return 0;
    const long stageId = static_cast<long>(lifecycle->loadTargetStage(path));
    if (!stageId)
        return 0;
    try
    {
        attach = OmniPhysX::getInstance().physXAttach(stageId, true);
    }
    catch (...)
    {
        lifecycle->eraseStage(stageId);
        return 0;
    }
    if (attach == kNoAttach)
    {
        lifecycle->eraseStage(stageId);
        return 0;
    }
    return stageId;
}

long bridgeCreateEmptyStage()
{
    omni::physics::parse::IUsdStageLifecycle* lifecycle = omni::physics::parse::usdStageLifecycle();
    return lifecycle ? static_cast<long>(lifecycle->createEmptyStage()) : 0;
}

bool bridgeAttachTargetStageId(long stageId, AttachHandle& attach)
{
    RuntimeExecutionScope configuration(kNoAttach);
    if (!configuration)
        return false;
    std::unique_ptr<RuntimeExecutionScope> ownerOperation;
    if (attach != kNoAttach)
    {
        ownerOperation = std::make_unique<RuntimeExecutionScope>(attach, true);
        if (!*ownerOperation)
            return false;
    }
    if (attach != kNoAttach && UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach))
        OmniPhysX::getInstance().physXDetach(attach);
    attach = kNoAttach;
    if (!stageId)
        return true;
    // Residency check via the reparse seam's stage-cache lookup (the USD arm's Find), then the
    // pxr-free physXAttach. With no seam installed a nonzero id can only be a caller error, so
    // fail closed.
    omni::physics::parse::IUsdReparse* reparse = omni::physics::parse::usdReparse();
    if (!reparse)
        return false;

    usdparser::AttachedStageUsdHandle handle; // empty; ctor routes through the ops table
    if (!stageId || !reparse->resolveStageToHandle(static_cast<uint64_t>(stageId), handle.storage()))
        return false;

    attach = OmniPhysX::getInstance().physXAttach(stageId, true);
    return attach != kNoAttach;
}

RuntimeResult bridgeForceLoadPhysicsFromUSD(AttachHandle attach)
{
    AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach);
    if (!owner)
        return RuntimeResult::eInvalidAttachment;
    if (!omni::physics::parse::usdReparse() || owner->getOvstageInstance())
        return RuntimeResult::eInvalidArgument;
    if (!owner->getObjectDatabase()->empty())
        UsdLoad::getUsdLoad()->releasePhysicsObjects(attach);
    processUpdates(*owner, 0.0f);
    owner->getPhysXPhysicsInterface()->finishSetup(*owner);
    owner->getInternalPhysXDatabase().updateDirtyMassActors();
    return RuntimeResult::eSuccess;
}

bool bridgeResolveBackingStage(uint64_t candidateStageId, AttachOvstageBackingStageHandle& outHandle)
{
    // The USD arm resolved via UsdUtilsStageCache Find and stored a weak handle. The reparse seam
    // assigns the resolved UsdStageWeakPtr into outHandle's already-live opaque storage (the cache
    // owns the reference that keeps it live), and fails closed to false when no seam is installed --
    // where ovstage's Kit-hosted backing-stage co-attach can never resolve one anyway.
    omni::physics::parse::IUsdReparse* reparse = omni::physics::parse::usdReparse();
    return reparse ? reparse->resolveStageToHandle(candidateStageId, outHandle.storage()) : false;
}

bool bridgeIsValidClonePath(const char* str, const char* role)
{
    // pxr-free superset of SdfPath's non-root absolute prim path grammar (utils/PrimPathGrammar.h).
    if (!str || !str[0])
    {
        OVX_RUNTIME_ERROR("clone: %s is empty.", role);
        return false;
    }
    if (!omni::physx::looksLikeAbsolutePrimPath(std::string_view(str), /*allowRoot=*/false))
    {
        OVX_RUNTIME_ERROR("clone: %s '%s' must be a non-root absolute prim path (e.g. '/World/env0') "
                       "with identifier-shaped name segments (UTF-8 allowed).", role, str);
        return false;
    }
    return true;
}

void bridgeInstallStartupParseBackend()
{
    // No-op: production installs no backend at startup; test executables install the USD
    // backends via omniPhysicsUsdInstallBackends(), and an ovstage attach installs its own.
}

void bridgeSetSimulationLayer(const char* layerIdentifier)
{
    // Kit/USD-authoring-only. The pxr part -- SdfLayer::Find + binding it into the handle storage --
    // routes through the stage-lifecycle seam; getSimulationLayer/setSimulationLayer and the handle
    // itself are pxr-free. No seam installed (production / USD-free) => no-op.
    omni::physics::parse::IUsdStageLifecycle* lifecycle = omni::physics::parse::usdStageLifecycle();
    if (!lifecycle)
        return;

    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    if (layerIdentifier)
    {
        // If someone set already a layer then we dont override with the anonymous sim layer
        if (omniPhysX.getSimulationLayer() && strstr(layerIdentifier, "PhysicsSimulationLayer"))
            return;

        SimulationLayerHandle handle; // empty; ctor routes through the layer ops table
        lifecycle->bindSimulationLayer(handle.storage(), layerIdentifier);
        omniPhysX.setSimulationLayer(handle);
    }
    else
    {
        omniPhysX.setSimulationLayer(nullptr);
    }
}

// --- SimulationLayerHandle (OmniPhysX.h) --------------------------------------------------
// Every operation forwards to the installed simulation-layer op table when present (ADR-0027
// seam #3); with no table installed (production / USD-free) the handle is a permanently-empty
// pointer-sized POD (zeroed storage, trivial copy/destroy, false truthiness, null rawLayer).
// rawLayer/fromRawLayer drive the table's rawPtr/adoptRaw hooks.
using omni::physics::parse::OpaquePxrHandleOps;
using omni::physics::parse::simulationLayerHandleOps;

SimulationLayerHandle::SimulationLayerHandle() noexcept
{
    if (const OpaquePxrHandleOps* ops = simulationLayerHandleOps())
        ops->construct(mStorage);
    else
        std::memset(mStorage, 0, sizeof(mStorage));
}

SimulationLayerHandle::SimulationLayerHandle(std::nullptr_t) noexcept
{
    if (const OpaquePxrHandleOps* ops = simulationLayerHandleOps())
        ops->construct(mStorage);
    else
        std::memset(mStorage, 0, sizeof(mStorage));
}

SimulationLayerHandle::SimulationLayerHandle(const SimulationLayerHandle& other) noexcept
{
    if (const OpaquePxrHandleOps* ops = simulationLayerHandleOps())
        ops->copyConstruct(mStorage, other.mStorage);
    else
        std::memcpy(mStorage, other.mStorage, sizeof(mStorage));
}

SimulationLayerHandle& SimulationLayerHandle::operator=(const SimulationLayerHandle& other) noexcept
{
    if (this != &other)
    {
        if (const OpaquePxrHandleOps* ops = simulationLayerHandleOps())
            ops->copyAssign(mStorage, other.mStorage);
        else
            std::memcpy(mStorage, other.mStorage, sizeof(mStorage));
    }
    return *this;
}

SimulationLayerHandle::~SimulationLayerHandle()
{
    if (const OpaquePxrHandleOps* ops = simulationLayerHandleOps())
        ops->destroy(mStorage);
    // ops null: empty POD, nothing to destroy.
}

SimulationLayerHandle::operator bool() const noexcept
{
    if (const OpaquePxrHandleOps* ops = simulationLayerHandleOps())
        return ops->toBool(mStorage);
    return false;
}

void* SimulationLayerHandle::rawLayer() const noexcept
{
    if (const OpaquePxrHandleOps* ops = simulationLayerHandleOps())
        return ops->rawPtr(mStorage);
    return nullptr;
}

SimulationLayerHandle SimulationLayerHandle::fromRawLayer(void* layer) noexcept
{
    SimulationLayerHandle handle; // constructed empty (through the ops table or zeroed)
    if (const OpaquePxrHandleOps* ops = simulationLayerHandleOps())
        ops->adoptRaw(handle.mStorage, layer);
    return handle;
}

} // namespace physx
} // namespace omni
