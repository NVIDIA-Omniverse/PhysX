// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */
#pragma once
#include "ParseApi.h"
#include "PreparedAttribute.h"

namespace omni::physics::parse
{
struct PreparedScene
{
    bool prepared = false;
    bool physxSceneAPI = false, quasistaticAPI = false;
    SceneInfo info;
    TokenId broadphaseTypeGPU;
    TokenId broadphaseTypeMBP;
    TokenId broadphaseTypeSAP;
    TokenId collisionSystemSAT;
    TokenId sceneUpdateAsynchronous;
    TokenId sceneUpdateDisabled;
    TokenId sceneUpdateSynchronous;
    TokenId solverTypePGS;
    TokenId solverTypeTGS;
    PreparedAttribute<float> physxSceneBounceThreshold;
    PreparedAttribute<float> physxSceneFrictionOffsetThreshold;
    PreparedAttribute<float> physxSceneFrictionCorrelationDistance;
    PreparedAttribute<float> physxSceneMaxBiasCoefficient;
    PreparedAttribute<uint32_t> physxSceneTimeStepsPerSecond;
    PreparedAttribute<uint32_t> physxSceneMinPositionIterationCount;
    PreparedAttribute<uint32_t> physxSceneMaxPositionIterationCount;
    PreparedAttribute<uint32_t> physxSceneMinVelocityIterationCount;
    PreparedAttribute<uint32_t> physxSceneMaxVelocityIterationCount;
    PreparedAttribute<uint32_t> physxSceneGpuMaxNumPartitions;
    PreparedAttribute<bool> physxSceneEnableCCD;
    PreparedAttribute<bool> physxSceneEnableStabilization;
    PreparedAttribute<bool> physxSceneEnableGPUDynamics;
    PreparedAttribute<bool> physxSceneEnableEnhancedDeterminism;
    PreparedAttribute<bool> physxSceneEnableExternalForcesEveryIteration;
    PreparedAttribute<bool> physxSceneInvertCollisionGroupFilter;
    PreparedAttribute<bool> physxSceneReportKinematicKinematicPairs;
    PreparedAttribute<bool> physxSceneReportKinematicStaticPairs;
    PreparedAttribute<bool> physxSceneEnableSceneQuerySupport;
    PreparedAttribute<bool> physxSceneSolveArticulationContactLast;
    PreparedAttribute<bool> physxSceneDisableSleeping;
    PreparedAttribute<int64_t> physxSceneGpuTempBufferCapacity;
    PreparedAttribute<int64_t> physxSceneGpuMaxRigidContactCount;
    PreparedAttribute<int64_t> physxSceneGpuMaxRigidPatchCount;
    PreparedAttribute<int64_t> physxSceneGpuHeapCapacity;
    PreparedAttribute<int64_t> physxSceneGpuFoundLostPairsCapacity;
    PreparedAttribute<int64_t> physxSceneGpuFoundLostAggregatePairsCapacity;
    PreparedAttribute<int64_t> physxSceneGpuTotalAggregatePairsCapacity;
    PreparedAttribute<int64_t> physxSceneGpuMaxDeformableVolumeContacts;
    PreparedAttribute<int64_t> physxSceneGpuMaxDeformableSurfaceContacts;
    PreparedAttribute<int64_t> physxSceneGpuMaxParticleContacts;
    PreparedAttribute<int64_t> physxSceneGpuCollisionStackSize;
    PreparedAttribute<TokenId> physxSceneUpdateType;
    PreparedAttribute<TokenId> physxSceneCollisionSystem;
    PreparedAttribute<TokenId> physxSceneSolverType;
    PreparedAttribute<TokenId> physxSceneBroadphaseType;
    PreparedAttribute<int64_t> newtonTimeStepsPerSecond;
    PreparedAttribute<bool> newtonGravityEnabled;
    PreparedAttribute<bool> physxSceneQuasistaticEnableQuasistatic;
    PreparedAttribute<int64_t> physxSceneEnvIdInBoundsBitCount;
};
PreparedScene prepareScene(ParseContext& context, ObjectKey key, const SceneInfo& info);
bool fillScene(const PreparedScene& input, PhysxSceneDesc& output) noexcept;
} // namespace omni::physics::parse
