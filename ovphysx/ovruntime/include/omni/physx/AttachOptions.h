// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-LIFETIME-001
// @covers AC-1 AC-2
// @implements REQ-ATTACH-SERVICES-001
// @covers AC-3
#pragma once
#include <cstdint>

namespace omni::physx
{
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
    bool physxDefaultSimulator = true;
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
};
}
