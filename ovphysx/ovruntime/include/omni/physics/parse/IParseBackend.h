// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-1 AC-2 AC-5 AC-12
 */
#pragma once

// Parse backend registry (ADR-0005). A backend is the single provider of the
// per-attachment source trio (read source + write sink + change feed). At most
// one backend is active per process; the registry starts empty. An ovstage attach
// installs its own backend, and test executables install the USD backend through
// omniPhysicsUsdInstallBackends() (ADR-0027). Switching is only valid while no
// stage is attached.
//
// This header is part of the USD-free core parse library: it names only
// parse-lib interfaces and plain handles, never USD types. `scanStage` is
// deliberately NOT on this interface — its result type (ScannedStage) is
// USD-coupled (omni::physics::usd), so routing it here would break the
// USD-free invariant. The walker dispatch stays in omni.physics.usd.

// The interface headers are pulled in (not forward-declared) so SourceBundle's
// unique_ptr members have complete types: any consumer that constructs or
// destroys a SourceBundle needs them. All three are USD-free parse interfaces.
#include <omni/physics/parse/IChangeFeed.h>
#include <omni/physics/parse/IPhysicsDataWrite.h>
#include <omni/physics/parse/IPhysicsSource.h>

#include <cstdint>
#include <memory>
#include <string_view>

namespace omni
{
namespace physics
{
namespace parse
{

// Backend-opaque handle to the scene being attached. Each backend interprets
// `nativeStage` for its own source type: the USD backend reads it as a pointer
// to the live USD stage handle (UsdStageWeakPtr); an ovstage backend reads it
// as a pointer to its backend-specific attach payload.
// `residentBackingStageId` is the optional locally resident USD stage-cache id
// an external backend may use for compatibility fallbacks; the USD backend
// leaves it at 0 and derives the id from `nativeStage` instead, so a target is
// never the authority on an id the source it produces already owns
// (`IPhysicsSource::residentUsdStageId`). `readOrdinal` is optional backend
// snapshot state; 0 means "use the backend payload default".
//
// There is deliberately no field naming the *kind* of source: consumers that
// need to branch on USD-vs-external ask their attach (AttachedStage::
// hasExternalSource), because a target that merely happens to carry no stage id
// is indistinguishable from a stage-backed one whose id is 0.
struct AttachTarget
{
    const void* nativeStage = nullptr;
    uint64_t readOrdinal = 0;
    uint64_t residentBackingStageId = 0;
    // The consumer's live source for this attach, if any. A scan backend may read
    // through it (its caches stay warm across drains) instead of building a fresh
    // one; the parse backend ignores it. Not owned; null for a standalone scan.
    IPhysicsSource* attachedSource = nullptr;
};

// The per-attachment objects a backend produces for one scene. Any of the
// three may be null when the backend does not support that capability (e.g. an
// early ovstage backend with no write sink yet); consumers must null-check.
struct SourceBundle
{
    std::unique_ptr<IPhysicsSource> source;       // ADR-0001
    std::unique_ptr<IPhysicsDataWrite> write;     // ADR-0004
    std::unique_ptr<IChangeFeed> changeFeed;      // ADR-0003
};

// One registered provider of the parse source.
class IParseBackend
{
public:
    virtual ~IParseBackend() = default;

    // Stable identifier, e.g. "usd" or "ovstage".
    virtual std::string_view id() const = 0;

    // Build the source trio for `target`. Returns an empty bundle when the
    // target is not one this backend understands.
    virtual SourceBundle createSource(const AttachTarget& target) = 0;
};

// Install `backend` as the single active backend, replacing any previous one.
// Must only be called while no stage is attached (the slot is process-global).
void setParseBackend(std::unique_ptr<IParseBackend> backend);

// The active backend, or null if none has been installed yet.
IParseBackend* parseBackend();

// Move the active parse backend out of the registry, transferring ownership to
// the caller and leaving the registry empty. An ovstage attach stashes the
// previous default this way so detach reinstalls the exact same instance instead
// of a freshly built one (the collapsed pxr-free bridge cannot rebuild the USD
// backend, so its bridgeMakeDefaultParseBackend() captures the live one).
std::unique_ptr<IParseBackend> takeParseBackend();

} // namespace parse
} // namespace physics
} // namespace omni
