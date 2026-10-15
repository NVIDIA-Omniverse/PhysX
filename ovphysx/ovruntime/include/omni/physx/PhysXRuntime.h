// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-4

#pragma once

#include <omni/physx/IPhysxSimulation.h>

#define OMNI_PHYSX_RUNTIME_API

namespace omni
{
namespace physics
{
namespace tensors
{
struct TensorApi;
} // namespace tensors
} // namespace physics

namespace physx
{
struct IOptionalCuda;
struct IPhysx;
struct IPhysxAttachmentPrivate;
struct IPhysxBenchmarks;
struct IPhysxCooking;
struct IPhysxCookingService;
struct IPhysxCookingServicePrivate;
struct IPhysxCustomGeometry;
struct IPhysxCustomJoint;
struct IPhysxFoundation;
struct IPhysxJoint;
struct IPhysxPropertyQuery;
struct IPhysxPrivate;
struct IPhysxReplicator;
struct IPhysxSceneQuery;
struct IPhysxStageUpdate;
struct IPhysxStatistics;
struct IPhysxUnitTests;
struct IPhysxVisualization;

namespace runtime
{

OMNI_PHYSX_RUNTIME_API void startup();
OMNI_PHYSX_RUNTIME_API void shutdown();

// Re-evaluates whether the PhysX SDK profile zones are collected, by re-reading
// /physics/exposeProfilerData and installing or clearing the SDK profiler callback
// to match. A host calls this after selecting a profiler backend.
//
// The PhysX SDK is created once per process, so a host whose first instance ran
// without profiling has an SDK created with no callback installed, and every later
// zone would be dropped before reaching the bound backend. Safe to call before
// startup().
//
// @implements REQ-SIM-PROFILER-001
// @covers AC-6
OMNI_PHYSX_RUNTIME_API void refreshProfilerCallback();

/// Copy defaults for constructing an explicit attachment descriptor. Returns false
/// before startup or while execution is busy, without changing either output.
/// @implements REQ-ATTACH-SERVICES-001
/// @covers AC-3
OMNI_PHYSX_RUNTIME_API bool snapshotAttachDefaults(PhysxAttachmentSettings& attachment,
                                                  PhysxSharedSettings& shared);

/// Attach through the same runtime path while distinguishing retryable
/// configuration admission from an invalid payload or construction failure.
/// startConfiguredRecording starts legacy startup OmniPVD output after a
/// successful attach; attachments without a public recording owner pass false.
/// @implements REQ-ATTACH-EXECUTION-001
/// @covers AC-4
OMNI_PHYSX_RUNTIME_API AttachHandle attachOvstageWithAdmission(const void* payload, uint64_t ordinal,
                                                               const PhysxAttachOptions& options,
                                                               bool startConfiguredRecording, bool& busy);

/// Copy the settings retained by one exact live attachment. These are the
/// requested shared values, not a claim that a GPU context was realized.
/// Invalid owners and busy execution return false with unchanged outputs.
/// @implements REQ-ATTACH-SERVICES-001
/// @covers AC-3
OMNI_PHYSX_RUNTIME_API bool snapshotAttachedSettings(AttachHandle attach,
                                                     PhysxAttachmentSettings& attachment,
                                                     PhysxSharedSettings& shared);

/// Change contact processing for one idle attachment without changing peer settings.
/// Returns false for an invalid owner or busy execution.
/// @implements REQ-ATTACH-SERVICES-001
/// @covers AC-7
OMNI_PHYSX_RUNTIME_API bool setAttachmentContactProcessingDisabled(AttachHandle attach, bool disabled);

// Requires startup() to have completed. Use tryGetPhysxSimulationInterface()
// when probing optional availability.
OMNI_PHYSX_RUNTIME_API IPhysx& getPhysxInterface();
OMNI_PHYSX_RUNTIME_API IPhysx* tryGetPhysxInterface();
OMNI_PHYSX_RUNTIME_API IPhysxPrivate& getPhysxPrivateInterface();
OMNI_PHYSX_RUNTIME_API IPhysxPrivate* tryGetPhysxPrivateInterface();
OMNI_PHYSX_RUNTIME_API IPhysxJoint& getPhysxJointInterface();
OMNI_PHYSX_RUNTIME_API IPhysxJoint* tryGetPhysxJointInterface();
OMNI_PHYSX_RUNTIME_API IPhysxReplicator& getPhysxReplicatorInterface();
OMNI_PHYSX_RUNTIME_API IPhysxReplicator* tryGetPhysxReplicatorInterface();
OMNI_PHYSX_RUNTIME_API IPhysxCustomJoint& getPhysxCustomJointInterface();
OMNI_PHYSX_RUNTIME_API IPhysxCustomJoint* tryGetPhysxCustomJointInterface();
OMNI_PHYSX_RUNTIME_API IPhysxCustomGeometry& getPhysxCustomGeometryInterface();
OMNI_PHYSX_RUNTIME_API IPhysxCustomGeometry* tryGetPhysxCustomGeometryInterface();
OMNI_PHYSX_RUNTIME_API IPhysxVisualization& getPhysxVisualizationInterface();
OMNI_PHYSX_RUNTIME_API IPhysxVisualization* tryGetPhysxVisualizationInterface();
OMNI_PHYSX_RUNTIME_API IPhysxSceneQuery& getPhysxSceneQueryInterface();
OMNI_PHYSX_RUNTIME_API IPhysxSceneQuery* tryGetPhysxSceneQueryInterface();
OMNI_PHYSX_RUNTIME_API IPhysxPropertyQuery& getPhysxPropertyQueryInterface();
OMNI_PHYSX_RUNTIME_API IPhysxPropertyQuery* tryGetPhysxPropertyQueryInterface();
OMNI_PHYSX_RUNTIME_API IPhysxAttachmentPrivate& getPhysxAttachmentPrivateInterface();
OMNI_PHYSX_RUNTIME_API IPhysxAttachmentPrivate* tryGetPhysxAttachmentPrivateInterface();
OMNI_PHYSX_RUNTIME_API IPhysxStageUpdate& getPhysxStageUpdateInterface();
OMNI_PHYSX_RUNTIME_API IPhysxStageUpdate* tryGetPhysxStageUpdateInterface();
OMNI_PHYSX_RUNTIME_API IPhysxStatistics& getPhysxStatisticsInterface();
OMNI_PHYSX_RUNTIME_API IPhysxStatistics* tryGetPhysxStatisticsInterface();
OMNI_PHYSX_RUNTIME_API IPhysxUnitTests& getPhysxUnitTestsInterface();
OMNI_PHYSX_RUNTIME_API IPhysxUnitTests* tryGetPhysxUnitTestsInterface();
OMNI_PHYSX_RUNTIME_API IPhysxBenchmarks& getPhysxBenchmarksInterface();
OMNI_PHYSX_RUNTIME_API IPhysxBenchmarks* tryGetPhysxBenchmarksInterface();
OMNI_PHYSX_RUNTIME_API IPhysxCooking& getPhysxCookingInterface();
OMNI_PHYSX_RUNTIME_API IPhysxCooking* tryGetPhysxCookingInterface();
OMNI_PHYSX_RUNTIME_API IPhysxCookingService& getPhysxCookingServiceInterface();
OMNI_PHYSX_RUNTIME_API IPhysxCookingService* tryGetPhysxCookingServiceInterface();
OMNI_PHYSX_RUNTIME_API IPhysxCookingServicePrivate& getPhysxCookingServicePrivateInterface();
OMNI_PHYSX_RUNTIME_API IPhysxCookingServicePrivate* tryGetPhysxCookingServicePrivateInterface();
OMNI_PHYSX_RUNTIME_API IPhysxFoundation& getPhysxFoundationInterface();
OMNI_PHYSX_RUNTIME_API IPhysxFoundation* tryGetPhysxFoundationInterface();
OMNI_PHYSX_RUNTIME_API IOptionalCuda& getOptionalCudaInterface();
OMNI_PHYSX_RUNTIME_API IOptionalCuda* tryGetOptionalCudaInterface();
// Non-blocking observation for frontend callback preflight, safe before runtime
// startup. It does not reserve execution or replace the externally serialized
// caller contract; native dispatch still checks pending attachment ownership.
OMNI_PHYSX_RUNTIME_API bool isRuntimeExecuting() noexcept;
// Admission observation before frontend cleanup/settings mutation. This is not
// a reservation: callers still obey external serialization, and native entries
// take their own execution scope. Retirement may drain its own pending interval.
OMNI_PHYSX_RUNTIME_API bool isRuntimeBusy(omni::physics::AttachHandle attach, bool allowPendingOwner = false) noexcept;
OMNI_PHYSX_RUNTIME_API IPhysxSimulation& getPhysxSimulationInterface();
OMNI_PHYSX_RUNTIME_API IPhysxSimulation* tryGetPhysxSimulationInterface();
OMNI_PHYSX_RUNTIME_API omni::physics::tensors::TensorApi& getTensorApiInterface();
OMNI_PHYSX_RUNTIME_API omni::physics::tensors::TensorApi* tryGetTensorApiInterface();

} // namespace runtime
} // namespace physx
} // namespace omni
