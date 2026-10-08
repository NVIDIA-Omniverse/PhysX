// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

// omni::physx::queryPrim / PhysXPropertyQueryManager (ADR-0018): arbitrary live-stage-by-id
// property/mass query (e.g. "select a prim in the viewport, compute its mass without
// simulating"). Genuinely Kit/USD-authoring-only -- never reachable from ovphysx's own
// ovstage attach path.
//
// This header is pxr-free: the manager is a PIMPL facade whose state lives entirely inside
// usdBridge/PropertyQueryBridge.cpp and reaches USD only through the parse::usdReparse()
// seam; with no seam installed every query fails synchronously.

#include <omni/physx/IPhysxPropertyQuery.h>
#include <private/omni/physx/PhysxUsd.h> // pxr-free; PhysxRigidBodyDesc/PhysxShapeDesc

#include <carb/extras/Timer.h>


namespace omni
{
namespace kit
{
struct StageUpdateNode;
struct StageUpdateSettings;
} // namespace kit
namespace physx
{
class PhysXPropertyQueryManager;
struct PhysXPropertyQueryManagerImpl;

// PhysX.cpp's fillInterface(IPhysxPropertyQuery&) wires this in; with no reparse seam
// installed it always reports failure.
void queryPrim(uint64_t stageId,
               uint64_t primKey,
               PhysxPropertyQueryMode::Enum queryMode,
               const IPhysxPropertyQueryCallback& callbacks);

// OmniPhysX.cpp constructs/destroys/calls this unconditionally. With no reparse seam there is
// no live UsdStage to queue a request against, so updateQueuedRequests/cancelAllPendingRequests
// are no-ops and mImpl stays null.
class PhysXPropertyQueryManager
{
public:
    PhysXPropertyQueryManager();
    ~PhysXPropertyQueryManager();

    PhysXPropertyQueryManager(const PhysXPropertyQueryManager&) = delete;
    PhysXPropertyQueryManager& operator=(const PhysXPropertyQueryManager&) = delete;

    void queryPrim(uint64_t stageId,
                   uint64_t primPathId,
                   PhysxPropertyQueryMode::Enum queryMode,
                   const IPhysxPropertyQueryCallback& callbacks);

    void updateQueuedRequests();

    // Fail all queued rigid-body requests with an error and clear the queue.
    // Called on stage detach so that pending requests don't leak across tests
    // (and don't hold dangling UsdStageWeakPtrs that can crash later).
    void cancelAllPendingRequests(PhysxPropertyQueryResult::Enum errorCode);

private:
    // Lazily create mImpl once a USD reparse backend is installed (the loader installs it after
    // startup(), so it is not available when this manager is constructed). Returns true when an
    // impl is live. In production usdReparse() is null, so this stays a no-op returning false.
    bool ensureImpl();

    PhysXPropertyQueryManagerImpl* mImpl = nullptr;
};

} // namespace physx
} // namespace omni
