// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

#include <omni/physics/usd/UsdBackendInstall.h>

#include <omni/physics/parse/IParseBackend.h>
#include <omni/physics/parse/ScanBackend.h>
#include <omni/physics/parse/UsdReparse.h> // parse::setUsdReparse
#include <omni/physics/parse/UsdBackingAuthor.h> // parse::setUsdBackingAuthor
#include <omni/physics/parse/UsdAttachmentAuthor.h> // parse::setUsdAttachmentAuthor
#include <omni/physics/parse/UsdStageLifecycle.h> // parse::setUsdStageLifecycle
#include <omni/physics/parse/OpaquePxrHandleOps.h> // parse::set*HandleOps (ADR-0027 seam #3)
#include <OpaquePxrHandleOpsUsd.h> // attachedStageUsdHandleOps / simulationLayerHandleOps
#include <omni/physics/usd/UsdParseBackend.h>
#include <omni/physics/usd/UsdReparse.h> // makeUsdReparse
#include <omni/physics/usd/UsdBackingAuthor.h> // makeUsdBackingAuthor
#include <omni/physics/usd/UsdAttachmentAuthor.h> // makeUsdAttachmentAuthor
#include <omni/physics/usd/UsdStageLifecycle.h> // makeUsdStageLifecycle
#include <omni/physics/usd/UsdScanBackend.h>

extern "C"
{
void omniPhysicsUsdInstallHandleOps(void)
{
    // ADR-0027 seam #3: stateless, process-constant tables. Installed once before any handle
    // owner exists and never removed; storing the same pointers again is a no-op.
    omni::physics::parse::setAttachedStageUsdHandleOps(omni::physics::usd::attachedStageUsdHandleOps());
    omni::physics::parse::setSimulationLayerHandleOps(omni::physics::usd::simulationLayerHandleOps());
}

void omniPhysicsUsdInstallBackends(void)
{
    omniPhysicsUsdInstallHandleOps();

    omni::physics::parse::setParseBackend(omni::physics::usd::makeUsdParseBackend());
    omni::physics::parse::setScanBackend(omni::physics::usd::makeUsdScanBackend());

    // ADR-0027 seam #2: the USD reparse/authoring backends the USD-free omni.physx side reaches
    // USD through.
    omni::physics::parse::setUsdReparse(omni::physics::usd::makeUsdReparse());
    omni::physics::parse::setUsdBackingAuthor(omni::physics::usd::makeUsdBackingAuthor());
    omni::physics::parse::setUsdAttachmentAuthor(omni::physics::usd::makeUsdAttachmentAuthor());
    omni::physics::parse::setUsdStageLifecycle(omni::physics::usd::makeUsdStageLifecycle());
}

void omniPhysicsUsdRemoveBackends(void)
{
    // Functional seams only; the handle op tables stay installed for the process lifetime
    // (handle owners are alive across this call).
    omni::physics::parse::setUsdStageLifecycle(nullptr);
    omni::physics::parse::setUsdAttachmentAuthor(nullptr);
    omni::physics::parse::setUsdBackingAuthor(nullptr);
    omni::physics::parse::setUsdReparse(nullptr);

    omni::physics::parse::setParseBackend(nullptr);
    omni::physics::parse::setScanBackend(nullptr);
}
}
