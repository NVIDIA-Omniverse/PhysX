// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-SCENE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
 *
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1 AC-2
 */

// Scene extension parser. Reads PhysxSceneAPI / PhysxSceneQuasistaticAPI
// overrides plus Newton fallbacks. The four nested material descriptors
// and the gravity-from-stage default are supplied by the caller via
// SceneInfo + the consumer's material-binding parsers.

#include <omni/physics/parse/ParseApi.h>
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
bool readClampedIfAuthored(const IPhysicsSource& src, ObjectKey key, TokenId attr,
                            T& val, T minVal, T maxVal)
{
    if (!src.hasAuthoredAttribute(key, attr)) return false;

    T attrVal;
    if constexpr (std::is_same_v<T, uint32_t> || std::is_same_v<T, int32_t>)
    {
        int64_t tmp;
        if (!src.getAttribute(key, attr, tmp)) return false;
        attrVal = static_cast<T>(tmp);
    }
    else
    {
        if (!src.getAttribute(key, attr, attrVal)) return false;
    }

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

// Unclamped attribute read for GPU capacity ints. Calls Get() on the
// attribute; for USD this resolves the schema fallback default when
// unauthored, so the write is unconditional.
template <typename T>
void readScalarAlways(const IPhysicsSource& src, ObjectKey key, TokenId attr, T& val)
{
    int64_t tmp;
    if (src.getAttribute(key, attr, tmp))
        val = static_cast<T>(tmp);
}

bool readBoolIfAuthored(const IPhysicsSource& src, ObjectKey key, TokenId attr, bool& out)
{
    if (!src.hasAuthoredAttribute(key, attr)) return false;
    return src.getAttribute(key, attr, out);
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
    IPhysicsSource& src = ctx.source();
    KnownTokens tok;
    tok.intern(src);

    // Gravity from info (already up-axis-resolved by the schema parser).
    desc.gravityDirection = info.gravityDirection;
    desc.gravityMagnitude = info.gravityMagnitude;

    const bool hasPhysxSceneAPI = src.hasSchema(key, tok.physxSceneAPI);

    bool physxTimeStepsAuthored = false;

    if (hasPhysxSceneAPI)
    {
        // updateType token enum
        TokenId updateType;
        if (src.getAttribute(key, tok.physxSceneUpdateType, updateType))
        {
            if      (updateType == tok.sceneUpdateSynchronous)  desc.sceneUpdateType = eSynchronous;
            else if (updateType == tok.sceneUpdateAsynchronous) desc.sceneUpdateType = eAsynchronous;
            else if (updateType == tok.sceneUpdateDisabled)     desc.sceneUpdateType = eDisabled;
        }

        readClampedIfAuthored<float>(
            src, key, tok.physxSceneBounceThreshold, desc.bounceThreshold, 1e-06f, FLT_MAX);
        readClampedIfAuthored<float>(
            src, key, tok.physxSceneFrictionOffsetThreshold, desc.frictionOffsetThreshold, 0.0f, FLT_MAX);
        readClampedIfAuthored<float>(
            src, key, tok.physxSceneFrictionCorrelationDistance, desc.frictionCorrelationDistance, 0.0f, FLT_MAX);
        readClampedIfAuthored<float>(
            src, key, tok.physxSceneMaxBiasCoefficient, desc.maxBiasCoefficient, 0.0f, FLT_MAX);

        // `physxTimeStepsAuthored` is true ONLY when the value is
        // authored AND in range — it gates whether the Newton fallback
        // runs below.
        physxTimeStepsAuthored = readClampedIfAuthored<uint32_t>(
            src, key, tok.physxSceneTimeStepsPerSecond, desc.timeStepsPerSecond, 1u, UINT_MAX);

        readClampedIfAuthored<uint32_t>(
            src, key, tok.physxSceneMinPositionIterationCount, desc.minPosIterationCount, 0u, 255u);
        readClampedIfAuthored<uint32_t>(
            src, key, tok.physxSceneMaxPositionIterationCount, desc.maxPosIterationCount, 0u, 255u);
        readClampedIfAuthored<uint32_t>(
            src, key, tok.physxSceneMinVelocityIterationCount, desc.minVelIterationCount, 0u, 255u);
        readClampedIfAuthored<uint32_t>(
            src, key, tok.physxSceneMaxVelocityIterationCount, desc.maxVelIterationCount, 0u, 255u);

        // Silently clamp min down when min > max — the schema parser
        // already logs an error on the same descriptor before this
        // overlay runs.
        if (desc.minPosIterationCount > desc.maxPosIterationCount)
            desc.minPosIterationCount = desc.maxPosIterationCount;
        if (desc.minVelIterationCount > desc.maxVelIterationCount)
            desc.minVelIterationCount = desc.maxVelIterationCount;

        readBoolIfAuthored(src, key, tok.physxSceneEnableCCD, desc.enableCCD);
        readBoolIfAuthored(src, key, tok.physxSceneEnableStabilization, desc.enableStabilization);
        readBoolIfAuthored(src, key, tok.physxSceneEnableGPUDynamics, desc.enableGPUDynamics);
        readBoolIfAuthored(src, key, tok.physxSceneEnableEnhancedDeterminism, desc.enableEnhancedDeterminism);
        readBoolIfAuthored(src, key, tok.physxSceneEnableExternalForcesEveryIteration, desc.enableExternalForcesEveryIteration);
        readBoolIfAuthored(src, key, tok.physxSceneInvertCollisionGroupFilter, desc.invertedFiltering);
        readBoolIfAuthored(src, key, tok.physxSceneReportKinematicKinematicPairs, desc.reportKineKine);
        readBoolIfAuthored(src, key, tok.physxSceneReportKinematicStaticPairs, desc.reportKineStatic);
        readBoolIfAuthored(src, key, tok.physxSceneEnableSceneQuerySupport, desc.supportSceneQueries);
        readBoolIfAuthored(src, key, tok.physxSceneSolveArticulationContactLast, desc.solveArticulationContactLast);
        readBoolIfAuthored(src, key, tok.physxSceneDisableSleeping, desc.disableSleeping);

        TokenId collisionSystem;
        if (src.getAttribute(key, tok.physxSceneCollisionSystem, collisionSystem))
        {
            // Only test against SAT; PCM is the schema-default token
            // and does NOT trigger a write (the PCM enum is already the
            // default).
            if (collisionSystem == tok.collisionSystemSAT)
                desc.collisionSystem = eSAT;
        }

        TokenId solverType;
        if (src.getAttribute(key, tok.physxSceneSolverType, solverType))
        {
            if      (solverType == tok.solverTypeTGS) desc.solverType = eTGS;
            else if (solverType == tok.solverTypePGS) desc.solverType = ePGS;
        }

        TokenId broadphaseType;
        if (src.getAttribute(key, tok.physxSceneBroadphaseType, broadphaseType))
        {
            if      (broadphaseType == tok.broadphaseTypeSAP) desc.broadphaseType = eSAP;
            else if (broadphaseType == tok.broadphaseTypeMBP) desc.broadphaseType = eMBP;
            else if (broadphaseType == tok.broadphaseTypeGPU) desc.broadphaseType = eGPU;
        }

        // frictionType is deprecated; the schema parser emits a warning
        // upstream — parse-lib doesn't re-emit (no carb dependency).

        // GPU capacity ints — raw `attr.Get(&...)` with no clamp writes
        // the schema fallback default unconditionally when unauthored.
        readScalarAlways<uint64_t>(src, key, tok.physxSceneGpuTempBufferCapacity, desc.gpuTempBufferCapacity);
        readScalarAlways<uint32_t>(src, key, tok.physxSceneGpuMaxRigidContactCount, desc.gpuMaxRigidContactCount);
        readScalarAlways<uint32_t>(src, key, tok.physxSceneGpuMaxRigidPatchCount, desc.gpuMaxRigidPatchCount);
        readScalarAlways<uint32_t>(src, key, tok.physxSceneGpuHeapCapacity, desc.gpuHeapCapacity);
        readScalarAlways<uint32_t>(src, key, tok.physxSceneGpuFoundLostPairsCapacity, desc.gpuFoundLostPairsCapacity);
        readScalarAlways<uint32_t>(src, key, tok.physxSceneGpuFoundLostAggregatePairsCapacity, desc.gpuFoundLostAggregatePairsCapacity);
        readScalarAlways<uint32_t>(src, key, tok.physxSceneGpuTotalAggregatePairsCapacity, desc.gpuTotalAggregatePairsCapacity);
        readScalarAlways<uint32_t>(src, key, tok.physxSceneGpuMaxDeformableVolumeContacts, desc.gpuMaxDeformableVolumeContacts);
        readScalarAlways<uint32_t>(src, key, tok.physxSceneGpuMaxDeformableSurfaceContacts, desc.gpuMaxDeformableSurfaceContacts);
        readScalarAlways<uint32_t>(src, key, tok.physxSceneGpuMaxParticleContacts, desc.gpuMaxParticleContacts);
        readScalarAlways<uint32_t>(src, key, tok.physxSceneGpuCollisionStackSize, desc.gpuCollisionStackSize);

        readClampedIfAuthored<uint32_t>(
            src, key, tok.physxSceneGpuMaxNumPartitions, desc.gpuMaxNumPartitions, 1u, 32u);
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
    //     gravityMagnitude. Unconditional (no PhysX equivalent). Per-body
    //     PhysX disableGravity flags still work when this is true.
    if (!physxTimeStepsAuthored)
    {
        if (src.hasAuthoredAttribute(key, tok.newtonTimeStepsPerSecond))
        {
            int64_t iv;
            if (src.getAttribute(key, tok.newtonTimeStepsPerSecond, iv))
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

    if (src.hasAuthoredAttribute(key, tok.newtonGravityEnabled))
    {
        bool gravityEnabled = true;
        if (src.getAttribute(key, tok.newtonGravityEnabled, gravityEnabled) && !gravityEnabled)
            desc.gravityMagnitude = 0.0f;
    }

    // PhysxSceneQuasistaticAPI — multi-apply schema with one bool flag and a
    // UsdCollectionAPI listing the actors to treat as quasistatic. Mirrors
    // usdLoad/Scene.cpp:275-298. The collection walk
    // (UsdCollectionAPI::ComputeIncludedPaths) is USD-specific work the
    // routing site performs and hands us via SceneInfo::quasistaticActors;
    // an empty list means "no actors" (either API not applied or empty
    // includes rel).
    if (src.hasSchema(key, tok.physxSceneQuasistaticAPI))
    {
        src.getAttribute(key, tok.physxSceneQuasistaticEnableQuasistatic, desc.enableQuasistatic);
    }
    if (!info.quasistaticActors.empty())
    {
        desc.quasistaticActors.clear();
        desc.quasistaticActors.reserve(info.quasistaticActors.size());
        for (ObjectKey k : info.quasistaticActors)
            desc.quasistaticActors.insert(k);
    }

    // envIdInBoundsBitCount is read regardless of PhysxSceneAPI, but only
    // when the broadphase is GPU. Mirrors the Get() at usdLoad/Scene.cpp:303
    // gated on broadphaseType == eGPU. The attribute is stored on the prim
    // directly via TfToken("physxScene:envIdInBoundsBitCount") so the
    // schema-applied gate doesn't matter.
    if (desc.broadphaseType == eGPU)
    {
        if (src.hasAuthoredAttribute(key, tok.physxSceneEnvIdInBoundsBitCount))
        {
            int64_t iv;
            if (src.getAttribute(key, tok.physxSceneEnvIdInBoundsBitCount, iv))
                desc.envIdInBoundsBitCount = static_cast<int32_t>(iv);
        }
        else
        {
            desc.envIdInBoundsBitCount = -1;
        }
    }
}

} // namespace omni::physics::parse
