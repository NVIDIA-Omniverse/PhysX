// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-1 AC-2 AC-5
 */
#pragma once

// Parse backend registry (ADR-0005). A backend is the single provider of the
// per-attachment source trio (read source + write sink + change feed). Exactly
// one backend is active per process; the USD backend is installed as the
// default by the runtime at startup, and may be replaced on demand (e.g. tests
// switching to an ovstage-backed source). Switching is only valid while no
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
// as a pointer to its backend-specific attach payload. `stageId` is the native
// USD target id; external targets keep it at 0 as their source-kind sentinel.
// `residentBackingStageId` is the optional locally resident USD stage-cache id
// an external backend may use for compatibility fallbacks. `readOrdinal` is
// optional backend snapshot state; 0 means "use the backend payload default".
struct AttachTarget
{
    const void* nativeStage = nullptr;
    long stageId = 0;
    uint64_t readOrdinal = 0;
    uint64_t residentBackingStageId = 0;
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

} // namespace parse
} // namespace physics
} // namespace omni
