// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @implements REQ-BUILD-BRIDGE-001
 * @covers AC-4
 *
 * @implements REQ-BUILD-UNIBUILD-001
 * @covers AC-3
 */

// pxr-free declarations for the runtime-entry USD bridge (ADR-0027), implemented by
// usdBridge/RuntimeBridge.cpp. USD behaviour is reached only through the installed seams.


#include "OmniPhysX.h" // AttachOvstageBackingStageHandle

#include <cstdint>

namespace omni
{
namespace physx
{

// IPhysxBenchmarks-only stage loaders. Producing or resolving a UsdUtilsStageCache id goes
// through the stage-lifecycle / reparse seams; with no seam installed (production) all three
// report "no stage" rather than silently attaching nothing.

// Opens `path`, inserts it into the process-wide stage cache and attaches it, returning the
// new cache id. With a null `path`, detaches and erases the currently active stage instead.
// Returns 0 with no seam.
long bridgeLoadTargetStage(const char* path);

// Creates and caches an empty "default.usd" stage, returning its cache id. 0 with no seam.
long bridgeCreateEmptyStage();

// Attaches the already-cached stage named by `stageId`. False when it is not resident, and
// always with no seam, where a nonzero id can only be a caller error.
bool bridgeAttachTargetStageId(long stageId);

// Re-parses the active USD attach in place (Kit "force load physics"). Requires a live USD
// stage; otherwise reports that none is attached (also the response to an ovstage-only attach).
void bridgeForceLoadPhysicsFromUSD();

// Resolves `candidateStageId` in this runtime's UsdUtilsStageCache through the reparse seam.
// On success sets `outHandle` to the resident stage and returns true; otherwise leaves it
// untouched and returns false (always, with no seam).
bool bridgeResolveBackingStage(uint64_t candidateStageId, AttachOvstageBackingStageHandle& outHandle);

// "Non-root absolute prim path" grammar gate for clone source/target paths, logging the
// rejection reason against `role`. Hand-rolled, deliberately narrower than SdfPath's grammar.
bool bridgeIsValidClonePath(const char* str, const char* role);

// Plugin-startup hook for the process parse backend (ADR-0005). Production installs nothing:
// the registry stays empty until an ovstage attach installs its own backends, or a test
// executable installs the USD backends via omniPhysicsUsdInstallBackends().
void bridgeInstallStartupParseBackend();

// Kit/USD-authoring-only: binds the anonymous sublayer used to scrub simulation-time
// overrides back out of the edited stage (see OmniPhysX::setSimulationLayer), through the
// stage-lifecycle seam. No-op with no seam installed.
void bridgeSetSimulationLayer(const char* layerIdentifier);

} // namespace physx
} // namespace omni
