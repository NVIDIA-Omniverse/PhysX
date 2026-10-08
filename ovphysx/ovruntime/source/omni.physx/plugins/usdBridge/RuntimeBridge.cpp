// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
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

#include "OmniPhysX.h"
#include "Setup.h"
#include "PhysXReplicator.h"
#include "usdBridge/RuntimeBridge.h"
#include "utils/PrimPathGrammar.h"
#include <omni/physics/parse/OpaquePxrHandleOps.h> // parse::simulationLayerHandleOps (seam #3)
#include "usdLoad/LoadUsd.h"
#include "usdLoad/AttachedStage.h"
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

void OmniPhysX::physXAttach(long int stageId, bool loadPhysics)
{
    // Resolve the id through the reparse seam's stage-cache lookup (the USD arm's
    // UsdUtilsStageCache Find). With no seam installed (production / USD-free) fail closed with the
    // same message: a USD-free process has no UsdUtilsStageCache at all.
    omni::physics::parse::IUsdReparse* reparse = omni::physics::parse::usdReparse();
    if (!reparse)
    {
        CARB_LOG_ERROR("PhysX could not find USD stage");
        return;
    }

    usdparser::AttachedStageUsdHandle handle; // empty; ctor routes through the ops table
    if (!stageId || !reparse->resolveStageToHandle(static_cast<uint64_t>(stageId), handle.storage()))
    {
        CARB_LOG_ERROR("PhysX could not find USD stage");
        return;
    }

    // Ensure the PxPhysics singleton, but ONLY when we can derive its tolerances scale. Read
    // metersPerUnit through the source abstraction: the AttachedStage is not registered with
    // UsdLoad until the attach() call below, so the active stage would be empty here on first
    // physics creation. Build a throwaway source over the resolved handle through the reparse
    // seam's PRIVATE USD backend (createSourceFromHandle), never through the process-active parse
    // backend: that one may be a data-plane backend (ovstage) that would read the handle storage
    // as its own payload. Same rule as AttachedStage::rebuildUsdSource.
    // A.B. TODO tolerances are per-stage and should move into individual scenes.
    if (!getPhysXSetup().hasBeenInitiallyCreated())
    {
        float metersPerUnit = omni::physics::parse::SourceUnits{}.metersPerUnit;
        omni::physics::parse::SourceBundle bundle = reparse->createSourceFromHandle(handle.storage());
        if (bundle.source)
            metersPerUnit = bundle.source->getSourceUnits().metersPerUnit;
        getPhysXSetup().createPhysics(getPhysXSetup().getDefaultTolerances(double(metersPerUnit)));
    }
    getPhysXSetup().getPhysics();

    physXAttachSession();

    // A replicator that wants to shape *this* attach must have registered before it existed, so
    // it can only have registered under kActiveAttach -- handles are minted by the attach itself.
    // Look that key up exactly rather than through getReplicator(): a registration carrying a
    // real handle names some other, already-live attach, and letting it apply here would be
    // precisely the stage-id hijack that keying on the attach removes (ADR-0016).
    ReplicatorMap::iterator pendingReplicator = mReplicatorMap.find(kActiveAttach);
    PhysXReplicator* replicator = pendingReplicator != mReplicatorMap.end() ? &pendingReplicator->second : nullptr;
    if (replicator && loadPhysics)
    {
        replicator->attach(stageId, &getPhysXUsdPhysicsInterface(), true);
    }
    else
    {
        UsdLoad::getUsdLoad()->attach(loadPhysics, stageId, &getPhysXUsdPhysicsInterface());
    }
}

long bridgeLoadTargetStage(const char* path)
{
    // USD-stage lifecycle (Open/Insert/GetId, or erase) routes through the stage-lifecycle seam;
    // the attach/detach orchestration stays here on the omni.physx side (physXAttach/physXDetach
    // are pxr-free). With no seam installed (production / USD-free) return 0.
    omni::physics::parse::IUsdStageLifecycle* lifecycle = omni::physics::parse::usdStageLifecycle();
    if (!lifecycle)
        return 0;

    if (path)
    {
        const long stageId = static_cast<long>(lifecycle->loadTargetStage(path));
        if (stageId)
            OmniPhysX::getInstance().physXAttach(stageId, true);
        return stageId;
    }

    // Detach + erase the currently active stage. Capture its id BEFORE physXDetach clears the
    // active attach, then erase it from the cache -- mirroring the _usd order (detach, then erase).
    const uint64_t activeStageId = UsdLoad::getUsdLoad()->getActiveStageId();
    if (activeStageId)
    {
        OmniPhysX::getInstance().physXDetach();
        lifecycle->eraseStage(activeStageId);
    }
    return 0;
}

long bridgeCreateEmptyStage()
{
    omni::physics::parse::IUsdStageLifecycle* lifecycle = omni::physics::parse::usdStageLifecycle();
    return lifecycle ? static_cast<long>(lifecycle->createEmptyStage()) : 0;
}

bool bridgeAttachTargetStageId(long stageId)
{
    // Residency check via the reparse seam's stage-cache lookup (the USD arm's Find), then the
    // pxr-free physXAttach. With no seam installed a nonzero id can only be a caller error, so
    // fail closed.
    omni::physics::parse::IUsdReparse* reparse = omni::physics::parse::usdReparse();
    if (!reparse)
        return false;

    usdparser::AttachedStageUsdHandle handle; // empty; ctor routes through the ops table
    if (!stageId || !reparse->resolveStageToHandle(static_cast<uint64_t>(stageId), handle.storage()))
        return false;

    OmniPhysX::getInstance().physXAttach(stageId, true);
    return true;
}

void bridgeForceLoadPhysicsFromUSD()
{
    // "Force load physics FROM USD" only means anything when USD is resident; the reparse seam's
    // presence is the pxr-free proxy for that. Null (production / USD-free) reports no attach,
    // matching a USD-loaded process's response to an ovstage-only attach. The body below
    // names no pxr type -- every call is an omni.physx runtime call.
    if (!omni::physics::parse::usdReparse())
    {
        CARB_LOG_ERROR("No USD stage attached.");
        return;
    }

    const uint64_t stageId = UsdLoad::getUsdLoad()->getActiveStageId();
    if (stageId)
    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();

        // If physics objects are already loaded, release them first and notify listeners
        AttachedStage* existingStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
        if (existingStage && !existingStage->getObjectDatabase()->empty())
        {
            UsdLoad::getUsdLoad()->releasePhysicsObjects(stageId);
        }

        omniPhysX.getPhysXSetup().getPhysics(); // make sure we have physics created
        {
            const char* forceSingleScene = omniPhysX.getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene);
            getPhysXUsdPhysicsInterface().setForceParseOnlySingleScene(forceSingleScene ? forceSingleScene : std::string());
        }

        UsdLoad::getUsdLoad()->update(0.f);
        AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
        if (attachedStage)
        {
            attachedStage->getPhysXPhysicsInterface()->finishSetup(*attachedStage);
            omniPhysX.getInternalPhysXDatabase().updateDirtyMassActors();
        }
    }
    else
    {
        CARB_LOG_ERROR("No USD stage attached.");
    }
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
        CARB_LOG_ERROR("clone: %s is empty.", role);
        return false;
    }
    if (!omni::physx::looksLikeAbsolutePrimPath(std::string_view(str), /*allowRoot=*/false))
    {
        CARB_LOG_ERROR("clone: %s '%s' must be a non-root absolute prim path (e.g. '/World/env0') "
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
