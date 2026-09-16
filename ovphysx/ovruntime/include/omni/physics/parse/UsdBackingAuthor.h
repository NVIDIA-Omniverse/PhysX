// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

#pragma once

#include <omni/physics/parse/IPhysicsDataWrite.h> // IPhysicsDataWrite (pxr-free interface)

#include <memory>

namespace omni::physics::parse
{

class IPhysicsSource;

// pxr-free interface the loadable USD library installs so the USD-free omni.physx side can
// author into a resident backing UsdStage without linking or naming any pxr type (ADR-0027
// bridge collapse). Every method operates on the opaque UsdStageWeakPtr storage the
// OpaquePxrHandleOps manage -- `handleStorage` points at those bytes. Installed once by the
// USD backend loader; null in production, where nothing authors a resident backing stage.
class IUsdBackingAuthor
{
public:
    virtual ~IUsdBackingAuthor() = default;

    // Author the synthetic default physics scene into the backing stage at `handleStorage`.
    // Returns true when a prim was authored. Reuses the USD backend's createDefaultPhysicsScene.
    virtual bool createDefaultScene(const void* handleStorage) = 0;

    // Remove a default physics scene previously authored by createDefaultScene. No-op when the
    // stage is empty or nothing exists at the scene path.
    virtual bool removeDefaultScene(const void* handleStorage) = 0;

    // Build a USD-backed write sink over the backing stage at `handleStorage`, for the
    // resident-backing-stage authoring fallback (an attach whose active source is not USD).
    // `fallbackSource` is the active (non-USD) source: the sink has no UsdSource, so it resolves
    // its ObjectKeys/TokenIds through that source's string identity (ADR-0004 addendum). Null is
    // tolerated but leaves the sink unable to resolve keys authored by a non-USD source.
    virtual std::unique_ptr<IPhysicsDataWrite> makeBackingDataWrite(const void* handleStorage,
                                                                    const IPhysicsSource* fallbackSource) = 0;
};

// Install `author` as the single active backing-stage author, replacing any previous one.
// null clears it. Must be called detached (ADR-0005/0027).
void setUsdBackingAuthor(std::unique_ptr<IUsdBackingAuthor> author);

// The active backing-stage author, or null when none is installed.
IUsdBackingAuthor* usdBackingAuthor();

} // namespace omni::physics::parse
