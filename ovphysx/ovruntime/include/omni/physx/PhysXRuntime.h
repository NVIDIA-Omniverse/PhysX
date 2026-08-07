// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

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
OMNI_PHYSX_RUNTIME_API IPhysxSimulation& getPhysxSimulationInterface();
OMNI_PHYSX_RUNTIME_API IPhysxSimulation* tryGetPhysxSimulationInterface();
OMNI_PHYSX_RUNTIME_API omni::physics::tensors::TensorApi& getTensorApiInterface();
OMNI_PHYSX_RUNTIME_API omni::physics::tensors::TensorApi* tryGetTensorApiInterface();

} // namespace runtime
} // namespace physx
} // namespace omni
