// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CACHE-INITIAL-001
 * @covers AC-1 AC-4
 */

// @implements REQ-ATTACH-LIFETIME-001
// @covers AC-1 AC-2
// @implements REQ-ATTACH-SERVICES-001
// @covers AC-3
// @implements REQ-CACHE-ATTACH-001
// @covers AC-1 AC-6
#pragma once
#include <cstdint>

namespace omni::physx
{
enum class PhysxCompiledStageCacheMode : uint32_t
{
    eDisabled,
    eReadOnly,
    eReadWrite
};
/// eStored means the bundle was accepted for publication. A whole-stage attach
/// writes it before returning; a subtree insertion queues the write, and a later
/// write failure only loses reuse. A matched bundle that does not fit is rejected
/// before anything changes; the stage or unit then loads ordinarily and reports
/// as on a miss.
enum class PhysxCompiledStageCacheResult : uint32_t
{
    eDisabled,
    eBypassed,
    eMiss,
    eStored,
    eHit,
    /// A matched bundle failed while it was applied. A whole-stage attach discards
    /// its candidate and attaches a fresh one ordinarily; a subtree insertion rolls
    /// the unit back and loads it ordinarily. Only a bundle at fault is removed.
    eRestoreFailed,
    eStoreFailed,
    /// No usable datastore (see PhysxCompiledStageCacheOptions). The stage or unit
    /// loaded ordinarily.
    eUnavailable
};
/// Bundles live in the process's UJITSO datastore, the one collision cooking uses
/// (ovphysx points it at the cooked-collider cache directory). The cache uses that
/// datastore only while it is purely local: without carb.ujitso.default, or when
/// the datastore also reaches Hub or network stores, the result is eUnavailable.
/// Trust boundary: a stored bundle's native section is deserialized in place by
/// PhysX, which trusts its contents. Its checksum detects corruption only; it is
/// not authenticated. Anyone who can write the datastore's directory can therefore
/// corrupt memory of the attaching process, as with cooked colliders. Keep that
/// directory private to the user.
/// The identity is always computed by the attach: it captures and hashes the
/// selected physics inputs, whose retained rows then serve miss-time parsing.
/// Hits reuse the stored capability verdict; misses run the capability audit.
struct PhysxCompiledStageCacheOptions
{
    PhysxCompiledStageCacheMode mode = PhysxCompiledStageCacheMode::eDisabled;
    /// Optional result written before attach returns, also when attach fails. Not
    /// retained by the owner.
    PhysxCompiledStageCacheResult* result = nullptr;
};
/// Per-attachment simulation, parsing and output settings, copied during attach.
/// These values never reconfigure process SDK, device or dispatcher resources.
struct PhysxAttachmentSettings
{
    float viewportGizmoScale = 1.0f;
    int32_t visualizationDisplayParticles = 0;
    bool updateToUsd = true;
    bool updateToUsdUsingXformCommonAPI = false;
    bool updateVelocitiesToUsd = true;
    bool outputVelocitiesLocalSpace = false;
    bool updateParticlesToUsd = true;
    int32_t minFrameRate = 30;
    float jointBodyTransformCheckTolerance = 0.001f;
    bool simulateEmptyScene = false;
    bool disableContactProcessing = false;
    bool approximateCones = false;
    bool approximateCylinders = false;
    bool enableExtendedJointAngles = false;
    bool resetOnStop = true;
    bool disableResetOnStop = false;
    /// -1 uses authored scene values; 0 forces CPU/PGS; 1 forces GPU/TGS.
    /// GPU requests remain subject to the process hard CPU-only policy.
    int32_t gpuPipelineOverride = -1;
    int32_t solverTypeOverride = -1;
    bool suppressReadback = false;
    bool replicatorEnvIdsOnAttach = false;
};

/// Resource configuration captured by an attachment. Each owner acquires its own
/// dispatcher and CUDA managers; peers may use different values. The SDK tolerance
/// scale and recording provider remain process-wide compatibility constraints.
struct PhysxSharedSettings
{
    uint32_t threadCount = 8;
    bool usePhysxDispatcher = false;
    int32_t cudaDevice = -1;
    /// Borrow the caller's current CUDA context. It must remain valid until runtime
    /// shutdown/SDK destruction, because compatible managers are retained by the SDK holder.
    bool useActiveCudaContext = false;
    bool synchronousCudaLaunches = false;
    /// 0 disables multi-GPU; 1 uses all devices; 2 skips the first device.
    int32_t sceneMultiGpuMode = 0;
};

/// Inputs for constructing one private attachment. The runtime copies path strings
/// before loading; the caller only keeps this descriptor alive through attach.
struct PhysxAttachOptions
{
    /// Absolute source paths whose subtrees are excluded from automatic population.
    /// Filtering survives source rebuilds and reset; explicit replication may populate them.
    /// The selected top-level traversal root retains its existing exclusion exception.
    const char* const* excludedPaths = nullptr;
    uint32_t excludedPathCount = 0;
    /// Optional complete override. Null snapshots the current process defaults.
    /// The pointed-to settings need remain alive only until attach returns.
    const PhysxAttachmentSettings* settings = nullptr;
    /// Optional copied override for attachment resources. Null snapshots process defaults.
    const PhysxSharedSettings* sharedSettings = nullptr;
    /// Experimental ovstage attachment and incremental insertion cache. The mode is
    /// copied for later additions; result is attach-only. Unsupported
    /// inputs use normal parsing.
    const PhysxCompiledStageCacheOptions* compiledStageCache = nullptr;
};
}
