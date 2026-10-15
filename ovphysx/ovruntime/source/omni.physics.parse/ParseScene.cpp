// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-SCENE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 *
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-PARSE-CORE-003
 * @covers AC-2
 */

// Scene extension parser. Reads PhysxSceneAPI / PhysxSceneQuasistaticAPI
// overrides plus Newton fallbacks. The four nested material descriptors
// and the gravity-from-stage default are supplied by the caller via
// SceneInfo + the consumer's material-binding parsers.

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedScene.h>
#include <omni/physics/parse/KnownTokens.h>

#include <cfloat>
#include <climits>
#include <cmath>

namespace omni::physics::parse
{

namespace
{

// PhysX rejects non-power-of-two values for gpuMaxNumPartitions; the
// parser silently substitutes 8.
bool isPowerOfTwo(uint32_t v)
{
    return v != 0 && (v & (v - 1)) == 0;
}

// Clamped attribute read with "authored AND in-range" semantics:
//   - Only mutates `val` when the attribute has an authored value.
//   - Inf positive -> maxVal; inf negative -> minVal (before range check).
//   - Out-of-range authored values still write a clamped value but the
//     return is `false`. Only in-range authored values count as
//     "authored" for downstream gating such as `physxTimeStepsAuthored`.
template <typename T>
bool fillClamped(const PreparedAttribute<T>& input, T& val, T minVal, T maxVal) noexcept
{
    if (!input.valid) return false;
    T attrVal = input.value;

    if constexpr (std::is_floating_point_v<T>)
    {
        if (std::isinf(static_cast<float>(attrVal)))
            attrVal = (static_cast<float>(attrVal) < 0.0f) ? minVal : maxVal;
    }

    if (attrVal >= minVal && attrVal <= maxVal)
    {
        val = attrVal;
        return true;
    }
    if (attrVal < minVal) val = minVal;
    else                  val = maxVal;
    return false;
}

} // namespace

void setToDefault(PhysxSceneDesc& desc, const SourceUnits& units)
{
    // Units-aware overlay only. All units-agnostic defaults
    // (timeStepsPerSecond, bounceThreshold, solver/broadphase enums, all
    // bools, gpu* capacities, iter-count clamps, envIdInBoundsBitCount)
    // are default-initialised in PhysxSceneDesc (Descriptors.h).
    const float metersPerUnit = units.metersPerUnit;
    const float tolerancesLength = 1.0f / metersPerUnit;

    // Default gravity for Z up-axis. The schema parser always overrides
    // this with the stage-derived value before parseScene runs, so a
    // fixed Z default suffices here. For non-USD callers, Y up is
    // covered explicitly.
    if (units.upAxis == UpAxis::eY)
        desc.gravityDirection = { 0.0f, -1.0f, 0.0f };
    else
        desc.gravityDirection = { 0.0f, 0.0f, -1.0f };
    desc.gravityMagnitude = 9.81f / metersPerUnit;

    desc.frictionOffsetThreshold     = 0.04f  * tolerancesLength;
    desc.frictionCorrelationDistance = 0.025f * tolerancesLength;
}

DescPtr<PhysxSceneDesc> makeDefaultSceneDesc(IDescriptorAllocator& allocator, const SourceUnits& units)
{
    DescPtr<PhysxSceneDesc> desc = allocateDesc<PhysxSceneDesc>(allocator);
    if (!desc)
        return {};
    setToDefault(*desc, units);
    desc->synthetic = true; // no backing prim — consumer skips existence/ownership gates
    return desc;
}

void parseScene(ParseContext& ctx, ObjectKey key, const SceneInfo& info, PhysxSceneDesc& desc)
{
    fillScene(prepareScene(ctx, key, info), desc);
    if (!info.quasistaticActors.empty())
    {
        desc.quasistaticActors.clear();
        desc.quasistaticActors.reserve(info.quasistaticActors.size());
        desc.quasistaticActors.insert(info.quasistaticActors.begin(), info.quasistaticActors.end());
    }
}

PreparedScene prepareScene(ParseContext& ctx, ObjectKey key, const SceneInfo& info)
{
    const IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();
    PreparedScene input;
    input.info = info;
    input.physxSceneAPI = src.hasSchema(key, tok.physxSceneAPI);
    input.quasistaticAPI = src.hasSchema(key, tok.physxSceneQuasistaticAPI);
    input.broadphaseTypeGPU = tok.broadphaseTypeGPU;
    input.broadphaseTypeMBP = tok.broadphaseTypeMBP;
    input.broadphaseTypeSAP = tok.broadphaseTypeSAP;
    input.collisionSystemSAT = tok.collisionSystemSAT;
    input.sceneUpdateAsynchronous = tok.sceneUpdateAsynchronous;
    input.sceneUpdateDisabled = tok.sceneUpdateDisabled;
    input.sceneUpdateSynchronous = tok.sceneUpdateSynchronous;
    input.solverTypePGS = tok.solverTypePGS;
    input.solverTypeTGS = tok.solverTypeTGS;
    if (input.physxSceneAPI)
        input.physxSceneBounceThreshold = prepareAttribute<float>(src, key, tok.physxSceneBounceThreshold, true);
    if (input.physxSceneAPI)
        input.physxSceneFrictionOffsetThreshold = prepareAttribute<float>(src, key, tok.physxSceneFrictionOffsetThreshold, true);
    if (input.physxSceneAPI)
        input.physxSceneFrictionCorrelationDistance = prepareAttribute<float>(src, key, tok.physxSceneFrictionCorrelationDistance, true);
    if (input.physxSceneAPI)
        input.physxSceneMaxBiasCoefficient = prepareAttribute<float>(src, key, tok.physxSceneMaxBiasCoefficient, true);
    if (input.physxSceneAPI)
        input.physxSceneTimeStepsPerSecond = prepareAttribute<uint32_t>(src, key, tok.physxSceneTimeStepsPerSecond, true);
    if (input.physxSceneAPI)
        input.physxSceneMinPositionIterationCount = prepareAttribute<uint32_t>(src, key, tok.physxSceneMinPositionIterationCount, true);
    if (input.physxSceneAPI)
        input.physxSceneMaxPositionIterationCount = prepareAttribute<uint32_t>(src, key, tok.physxSceneMaxPositionIterationCount, true);
    if (input.physxSceneAPI)
        input.physxSceneMinVelocityIterationCount = prepareAttribute<uint32_t>(src, key, tok.physxSceneMinVelocityIterationCount, true);
    if (input.physxSceneAPI)
        input.physxSceneMaxVelocityIterationCount = prepareAttribute<uint32_t>(src, key, tok.physxSceneMaxVelocityIterationCount, true);
    if (input.physxSceneAPI)
        input.physxSceneGpuMaxNumPartitions = prepareAttribute<uint32_t>(src, key, tok.physxSceneGpuMaxNumPartitions, true);
    if (input.physxSceneAPI)
        input.physxSceneEnableCCD = prepareAttribute<bool>(src, key, tok.physxSceneEnableCCD, true);
    if (input.physxSceneAPI)
        input.physxSceneEnableStabilization = prepareAttribute<bool>(src, key, tok.physxSceneEnableStabilization, true);
    if (input.physxSceneAPI)
        input.physxSceneEnableGPUDynamics = prepareAttribute<bool>(src, key, tok.physxSceneEnableGPUDynamics, true);
    if (input.physxSceneAPI)
        input.physxSceneEnableEnhancedDeterminism = prepareAttribute<bool>(src, key, tok.physxSceneEnableEnhancedDeterminism, true);
    if (input.physxSceneAPI)
        input.physxSceneEnableExternalForcesEveryIteration = prepareAttribute<bool>(src, key, tok.physxSceneEnableExternalForcesEveryIteration, true);
    if (input.physxSceneAPI)
        input.physxSceneInvertCollisionGroupFilter = prepareAttribute<bool>(src, key, tok.physxSceneInvertCollisionGroupFilter, true);
    if (input.physxSceneAPI)
        input.physxSceneReportKinematicKinematicPairs = prepareAttribute<bool>(src, key, tok.physxSceneReportKinematicKinematicPairs, true);
    if (input.physxSceneAPI)
        input.physxSceneReportKinematicStaticPairs = prepareAttribute<bool>(src, key, tok.physxSceneReportKinematicStaticPairs, true);
    if (input.physxSceneAPI)
        input.physxSceneEnableSceneQuerySupport = prepareAttribute<bool>(src, key, tok.physxSceneEnableSceneQuerySupport, true);
    if (input.physxSceneAPI)
        input.physxSceneSolveArticulationContactLast = prepareAttribute<bool>(src, key, tok.physxSceneSolveArticulationContactLast, true);
    if (input.physxSceneAPI)
        input.physxSceneDisableSleeping = prepareAttribute<bool>(src, key, tok.physxSceneDisableSleeping, true);
    if (input.physxSceneAPI)
        input.physxSceneGpuTempBufferCapacity = prepareAttribute<int64_t>(src, key, tok.physxSceneGpuTempBufferCapacity);
    if (input.physxSceneAPI)
        input.physxSceneGpuMaxRigidContactCount = prepareAttribute<int64_t>(src, key, tok.physxSceneGpuMaxRigidContactCount);
    if (input.physxSceneAPI)
        input.physxSceneGpuMaxRigidPatchCount = prepareAttribute<int64_t>(src, key, tok.physxSceneGpuMaxRigidPatchCount);
    if (input.physxSceneAPI)
        input.physxSceneGpuHeapCapacity = prepareAttribute<int64_t>(src, key, tok.physxSceneGpuHeapCapacity);
    if (input.physxSceneAPI)
        input.physxSceneGpuFoundLostPairsCapacity = prepareAttribute<int64_t>(src, key, tok.physxSceneGpuFoundLostPairsCapacity);
    if (input.physxSceneAPI)
        input.physxSceneGpuFoundLostAggregatePairsCapacity = prepareAttribute<int64_t>(src, key, tok.physxSceneGpuFoundLostAggregatePairsCapacity);
    if (input.physxSceneAPI)
        input.physxSceneGpuTotalAggregatePairsCapacity = prepareAttribute<int64_t>(src, key, tok.physxSceneGpuTotalAggregatePairsCapacity);
    if (input.physxSceneAPI)
        input.physxSceneGpuMaxDeformableVolumeContacts = prepareAttribute<int64_t>(src, key, tok.physxSceneGpuMaxDeformableVolumeContacts);
    if (input.physxSceneAPI)
        input.physxSceneGpuMaxDeformableSurfaceContacts = prepareAttribute<int64_t>(src, key, tok.physxSceneGpuMaxDeformableSurfaceContacts);
    if (input.physxSceneAPI)
        input.physxSceneGpuMaxParticleContacts = prepareAttribute<int64_t>(src, key, tok.physxSceneGpuMaxParticleContacts);
    if (input.physxSceneAPI)
        input.physxSceneGpuCollisionStackSize = prepareAttribute<int64_t>(src, key, tok.physxSceneGpuCollisionStackSize);
    if (input.physxSceneAPI)
        input.physxSceneUpdateType = prepareAttribute<TokenId>(src, key, tok.physxSceneUpdateType);
    if (input.physxSceneAPI)
        input.physxSceneCollisionSystem = prepareAttribute<TokenId>(src, key, tok.physxSceneCollisionSystem);
    if (input.physxSceneAPI)
        input.physxSceneSolverType = prepareAttribute<TokenId>(src, key, tok.physxSceneSolverType);
    if (input.physxSceneAPI)
        input.physxSceneBroadphaseType = prepareAttribute<TokenId>(src, key, tok.physxSceneBroadphaseType);
    input.newtonTimeStepsPerSecond = prepareAttribute<int64_t>(src, key, tok.newtonTimeStepsPerSecond, true);
    input.newtonGravityEnabled = prepareAttribute<bool>(src, key, tok.newtonGravityEnabled, true);
    if (input.quasistaticAPI)
        input.physxSceneQuasistaticEnableQuasistatic = prepareAttribute<bool>(src, key, tok.physxSceneQuasistaticEnableQuasistatic);
    input.physxSceneEnvIdInBoundsBitCount = prepareAttribute<int64_t>(src, key, tok.physxSceneEnvIdInBoundsBitCount, true);
    input.prepared = true;
    return input;
}

bool fillScene(const PreparedScene& input, PhysxSceneDesc& desc) noexcept
{
    if (!input.prepared) return false;
    const PreparedScene& tok = input;
    const SceneInfo& info = input.info;

    // Gravity from info (already up-axis-resolved by the schema parser).
    desc.gravityDirection = info.gravityDirection;
    desc.gravityMagnitude = info.gravityMagnitude;

    const bool hasPhysxSceneAPI = input.physxSceneAPI;

    bool physxTimeStepsAuthored = false;

    if (hasPhysxSceneAPI)
    {
        // updateType token enum
        TokenId updateType;
        if (input.physxSceneUpdateType.read(updateType))
        {
            if      (updateType == tok.sceneUpdateSynchronous)  desc.sceneUpdateType = eSynchronous;
            else if (updateType == tok.sceneUpdateAsynchronous) desc.sceneUpdateType = eAsynchronous;
            else if (updateType == tok.sceneUpdateDisabled)     desc.sceneUpdateType = eDisabled;
        }

        fillClamped(input.physxSceneBounceThreshold, desc.bounceThreshold, 1e-06f, FLT_MAX);
        fillClamped(input.physxSceneFrictionOffsetThreshold, desc.frictionOffsetThreshold, 0.0f, FLT_MAX);
        fillClamped(input.physxSceneFrictionCorrelationDistance, desc.frictionCorrelationDistance, 0.0f, FLT_MAX);
        fillClamped(input.physxSceneMaxBiasCoefficient, desc.maxBiasCoefficient, 0.0f, FLT_MAX);

        // `physxTimeStepsAuthored` is true ONLY when the value is
        // authored AND in range — it gates whether the Newton fallback
        // runs below.
        physxTimeStepsAuthored = fillClamped(input.physxSceneTimeStepsPerSecond, desc.timeStepsPerSecond, 1u, UINT_MAX);

        fillClamped(input.physxSceneMinPositionIterationCount, desc.minPosIterationCount, 0u, 255u);
        fillClamped(input.physxSceneMaxPositionIterationCount, desc.maxPosIterationCount, 0u, 255u);
        fillClamped(input.physxSceneMinVelocityIterationCount, desc.minVelIterationCount, 0u, 255u);
        fillClamped(input.physxSceneMaxVelocityIterationCount, desc.maxVelIterationCount, 0u, 255u);

        // Silently clamp min down when min > max — the schema parser
        // already logs an error on the same descriptor before this
        // overlay runs.
        if (desc.minPosIterationCount > desc.maxPosIterationCount)
            desc.minPosIterationCount = desc.maxPosIterationCount;
        if (desc.minVelIterationCount > desc.maxVelIterationCount)
            desc.minVelIterationCount = desc.maxVelIterationCount;

        input.physxSceneEnableCCD.read( desc.enableCCD);
        input.physxSceneEnableStabilization.read( desc.enableStabilization);
        input.physxSceneEnableGPUDynamics.read( desc.enableGPUDynamics);
        input.physxSceneEnableEnhancedDeterminism.read( desc.enableEnhancedDeterminism);
        input.physxSceneEnableExternalForcesEveryIteration.read( desc.enableExternalForcesEveryIteration);
        input.physxSceneInvertCollisionGroupFilter.read( desc.invertedFiltering);
        input.physxSceneReportKinematicKinematicPairs.read( desc.reportKineKine);
        input.physxSceneReportKinematicStaticPairs.read( desc.reportKineStatic);
        input.physxSceneEnableSceneQuerySupport.read( desc.supportSceneQueries);
        input.physxSceneSolveArticulationContactLast.read( desc.solveArticulationContactLast);
        input.physxSceneDisableSleeping.read( desc.disableSleeping);

        TokenId collisionSystem;
        if (input.physxSceneCollisionSystem.read(collisionSystem))
        {
            // Only test against SAT; PCM is the schema-default token
            // and does NOT trigger a write (the PCM enum is already the
            // default).
            if (collisionSystem == tok.collisionSystemSAT)
                desc.collisionSystem = eSAT;
        }

        TokenId solverType;
        if (input.physxSceneSolverType.read(solverType))
        {
            if      (solverType == tok.solverTypeTGS) desc.solverType = eTGS;
            else if (solverType == tok.solverTypePGS) desc.solverType = ePGS;
        }

        TokenId broadphaseType;
        if (input.physxSceneBroadphaseType.read(broadphaseType))
        {
            if      (broadphaseType == tok.broadphaseTypeSAP) desc.broadphaseType = eSAP;
            else if (broadphaseType == tok.broadphaseTypeMBP) desc.broadphaseType = eMBP;
            else if (broadphaseType == tok.broadphaseTypeGPU) desc.broadphaseType = eGPU;
        }

        // frictionType is deprecated; the schema parser emits a warning
        // upstream — parse-lib doesn't re-emit (no carb dependency).

        // GPU capacity ints — raw `attr.Get(&...)` with no clamp writes
        // the schema fallback default unconditionally when unauthored.
        desc.gpuTempBufferCapacity = static_cast<uint64_t>(input.physxSceneGpuTempBufferCapacity.valueOr(desc.gpuTempBufferCapacity));
        desc.gpuMaxRigidContactCount = static_cast<uint32_t>(input.physxSceneGpuMaxRigidContactCount.valueOr(desc.gpuMaxRigidContactCount));
        desc.gpuMaxRigidPatchCount = static_cast<uint32_t>(input.physxSceneGpuMaxRigidPatchCount.valueOr(desc.gpuMaxRigidPatchCount));
        desc.gpuHeapCapacity = static_cast<uint32_t>(input.physxSceneGpuHeapCapacity.valueOr(desc.gpuHeapCapacity));
        desc.gpuFoundLostPairsCapacity = static_cast<uint32_t>(input.physxSceneGpuFoundLostPairsCapacity.valueOr(desc.gpuFoundLostPairsCapacity));
        desc.gpuFoundLostAggregatePairsCapacity = static_cast<uint32_t>(input.physxSceneGpuFoundLostAggregatePairsCapacity.valueOr(desc.gpuFoundLostAggregatePairsCapacity));
        desc.gpuTotalAggregatePairsCapacity = static_cast<uint32_t>(input.physxSceneGpuTotalAggregatePairsCapacity.valueOr(desc.gpuTotalAggregatePairsCapacity));
        desc.gpuMaxDeformableVolumeContacts = static_cast<uint32_t>(input.physxSceneGpuMaxDeformableVolumeContacts.valueOr(desc.gpuMaxDeformableVolumeContacts));
        desc.gpuMaxDeformableSurfaceContacts = static_cast<uint32_t>(input.physxSceneGpuMaxDeformableSurfaceContacts.valueOr(desc.gpuMaxDeformableSurfaceContacts));
        desc.gpuMaxParticleContacts = static_cast<uint32_t>(input.physxSceneGpuMaxParticleContacts.valueOr(desc.gpuMaxParticleContacts));
        desc.gpuCollisionStackSize = static_cast<uint32_t>(input.physxSceneGpuCollisionStackSize.valueOr(desc.gpuCollisionStackSize));

        fillClamped(input.physxSceneGpuMaxNumPartitions, desc.gpuMaxNumPartitions, 1u, 32u);
        if (!isPowerOfTwo(desc.gpuMaxNumPartitions))
            desc.gpuMaxNumPartitions = 8;

        // PGS + externalForcesEveryIteration is unsupported — silently
        // clear the flag (the schema parser logs the warning upstream).
        if (desc.solverType == ePGS && desc.enableExternalForcesEveryIteration)
            desc.enableExternalForcesEveryIteration = false;
    }

    // Newton schema fallbacks (NewtonSceneAPI / NewtonXpbdSceneAPI /
    // NewtonKaminoSceneAPI):
    //
    //   newton:timeStepsPerSecond -> physxScene:timeStepsPerSecond
    //     Applies only when PhysX side is unauthored. Out-of-range
    //     values clamp to [1u, UINT_MAX] in either direction.
    //
    //   newton:gravityEnabled — when authored AND false, zero the
    //     gravityMagnitude. Unconditional (no PhysX equivalent): this is an
    //     on/off toggle rather than a fallback spelling of a PhysX attribute,
    //     so an authored physics:gravityMagnitude does NOT suppress it — the
    //     magnitude says how strong gravity is, this says whether it applies.
    //     The runtime-change handler (updateNewtonGravityEnabled) matches.
    //     Per-body PhysX disableGravity flags still work when this is true.
    if (!physxTimeStepsAuthored)
    {
        if (input.newtonTimeStepsPerSecond.authored)
        {
            int64_t iv;
            if (input.newtonTimeStepsPerSecond.read(iv))
            {
                if (iv < 1)
                    desc.timeStepsPerSecond = 1u;
                else if (iv > static_cast<int64_t>(UINT_MAX))
                    desc.timeStepsPerSecond = UINT_MAX;
                else
                    desc.timeStepsPerSecond = static_cast<uint32_t>(iv);
            }
        }
    }

    if (input.newtonGravityEnabled.authored)
    {
        bool gravityEnabled = true;
        if (input.newtonGravityEnabled.read(gravityEnabled) && !gravityEnabled)
            desc.gravityMagnitude = 0.0f;
    }

    // PhysxSceneQuasistaticAPI — multi-apply schema with one bool flag and a
    // UsdCollectionAPI listing the actors to treat as quasistatic. Mirrors
    // usdLoad/Scene.cpp:275-298. The collection walk
    // (UsdCollectionAPI::ComputeIncludedPaths) is USD-specific work the
    // routing site performs and hands us via SceneInfo::quasistaticActors;
    // an empty list means "no actors" (either API not applied or empty
    // includes rel).
    if (input.quasistaticAPI)
    {
        input.physxSceneQuasistaticEnableQuasistatic.read(desc.enableQuasistatic);
    }

    // envIdInBoundsBitCount is read regardless of PhysxSceneAPI, but only
    // when the broadphase is GPU. Mirrors the Get() at usdLoad/Scene.cpp:303
    // gated on broadphaseType == eGPU. The attribute is stored on the prim
    // directly via TfToken("physxScene:envIdInBoundsBitCount") so the
    // schema-applied gate doesn't matter.
    if (desc.broadphaseType == eGPU)
    {
        if (input.physxSceneEnvIdInBoundsBitCount.authored)
        {
            int64_t iv;
            if (input.physxSceneEnvIdInBoundsBitCount.read(iv))
                desc.envIdInBoundsBitCount = static_cast<int32_t>(iv);
        }
        else
        {
            desc.envIdInBoundsBitCount = -1;
        }
    }
    return true;
}

} // namespace omni::physics::parse
