// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <private/omni/physics/schema/IUsdPhysics.h>
#include <omni/physx/IPhysxSettings.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/UsdMaterialParsing.h>

#include "LoadUsd.h"
#include "LoadTools.h"
#include "Material.h"
#include "AttributeHelpers.h"

using namespace pxr;
using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{

static const TfToken kSolveArticulationContactLastToken("physxScene:solveArticulationContactLast");

void setToDefault(UsdStageWeakPtr stage, PhysxSceneDesc& desc)
{
    const double metersPerUnit = pxr::UsdGeomGetStageMetersPerUnit(stage);
    const TfToken upAxis = pxr::UsdGeomGetStageUpAxis(stage);

    float tolerancesLength = float(1.0f / metersPerUnit);
    float tolerancesSpeed = float(10.0f / metersPerUnit);

    desc.timeStepsPerSecond = 60;

    if (upAxis == pxr::UsdGeomTokens.Get()->x)
        desc.gravityDirection = { -1.0f, 0.0f, 0.0f };
    else if (upAxis == pxr::UsdGeomTokens.Get()->y)
        desc.gravityDirection = { 0.0f, -1.0f, 0.0f };
    else
        desc.gravityDirection = { 0.0f, 0.0f, -1.0f };    
    desc.gravityMagnitude = 9.81f / metersPerUnit;

    desc.bounceThreshold = 1e-06f;
    desc.frictionOffsetThreshold = 0.04f * tolerancesLength;
    desc.frictionCorrelationDistance = 0.025f * tolerancesLength;
    desc.maxBiasCoefficient = FLT_MAX;
    desc.collisionSystem = ePCM;
    desc.solverType = eTGS;
    desc.broadphaseType = eGPU;
    desc.enableCCD = false;
    desc.enableStabilization = false;
    desc.enableGPUDynamics = true;
    desc.solveArticulationContactLast = false;
    desc.enableEnhancedDeterminism = false;
    desc.enableExternalForcesEveryIteration = false;
    desc.invertedFiltering = false;
    desc.reportKineKine = false;
    desc.reportKineStatic = false;
    desc.sceneUpdateType = Synchronous;
    desc.supportSceneQueries = true;
    desc.enableResidualReporting = false;
    desc.reportResiduals = false;

    desc.enableQuasistatic = false;

    desc.gpuTempBufferCapacity = (16 * 1024 * 1024);
    desc.gpuMaxRigidContactCount = (1024 * 512);
    desc.gpuMaxRigidPatchCount = (1024 * 80);
    desc.gpuHeapCapacity = (64 * 1024 * 1024);
    desc.gpuFoundLostPairsCapacity = (256 * 1024);
    desc.gpuFoundLostAggregatePairsCapacity = (1024);
    desc.gpuTotalAggregatePairsCapacity = (1024);
    desc.gpuMaxDeformableVolumeContacts = (1 * 1024 * 1024);
    desc.gpuMaxDeformableSurfaceContacts = (1 * 1024 * 1024);
    desc.gpuMaxParticleContacts = (1 * 1024 * 1024);
    desc.gpuCollisionStackSize = (64 * 1024 * 1024);

    desc.gpuMaxNumPartitions = 8;

    desc.minPosIterationCount = 0;
    desc.maxPosIterationCount = 255;
    desc.minVelIterationCount = 0;
    desc.maxVelIterationCount = 255;

    desc.envIdInBoundsBitCount = -1;

    setToDefault(desc.defaultMaterialDesc);
    setToDefault(stage, desc.defaultDeformableMaterialDesc);
    setToDefault(stage, desc.defaultSurfaceDeformableMaterialDesc);
    setToDefault(desc.defaultPBDMaterialDesc);
    setToDefaultDeprecated(desc.defaultSoftBodyMaterialDesc);
    setToDefaultDeprecated(desc.defaultFemClothMaterialDesc);
}

template <typename T>
static bool GetAuthoredValueOrDefault(T* value, const UsdAttribute& attribute, const T& defaultValue)
{
    if (attribute.HasAuthoredValue())
    {
        attribute.Get(value);
        return true;
    }
    *value = defaultValue;
    return false;
}

PhysxSceneDesc* parseSceneDesc(const UsdStageWeakPtr stage, const omni::physics::schema::SceneDesc& inDesc)
{
    PhysxSceneDesc* sceneDesc = ICE_PLACEMENT_NEW(PhysxSceneDesc)();
    setToDefault(stage, *sceneDesc);

    GfVec3ToFloat3(inDesc.gravityDirection, sceneDesc->gravityDirection);
    sceneDesc->gravityMagnitude = inDesc.gravityMagnitude;

    auto materialSDFPath = usdmaterialutils::getMaterialBinding(inDesc.usdPrim);
    parseMaterialForPrim(inDesc.usdPrim, materialSDFPath, sceneDesc->defaultMaterialDesc);
    parseDeformableMaterialForPrim(inDesc.usdPrim, materialSDFPath, sceneDesc->defaultDeformableMaterialDesc);
    parseSurfaceDeformableMaterialForPrim(inDesc.usdPrim, materialSDFPath, sceneDesc->defaultSurfaceDeformableMaterialDesc);
    parsePBDMaterialForPrim(inDesc.usdPrim, materialSDFPath, sceneDesc->defaultPBDMaterialDesc);
    parseDeformableBodyMaterialForPrimDeprecated(inDesc.usdPrim, materialSDFPath, sceneDesc->defaultSoftBodyMaterialDesc);
    parseDeformableSurfaceMaterialForPrimDeprecated(inDesc.usdPrim, materialSDFPath, sceneDesc->defaultFemClothMaterialDesc);

    const PhysxSchemaPhysxSceneAPI physxSceneAPI = PhysxSchemaPhysxSceneAPI::Get(stage, inDesc.usdPrim.GetPrimPath());
    if (physxSceneAPI)
    {
        if (physxSceneAPI.GetUpdateTypeAttr())
        {
            TfToken sceneUpdateType;
            physxSceneAPI.GetUpdateTypeAttr().Get(&sceneUpdateType);
            if (sceneUpdateType == PhysxSchemaTokens.Get()->Synchronous)
                sceneDesc->sceneUpdateType = Synchronous;
            else if (sceneUpdateType == PhysxSchemaTokens.Get()->Asynchronous)
                sceneDesc->sceneUpdateType = Asynchronous;
            else if (sceneUpdateType == PhysxSchemaTokens.Get()->Disabled)
                sceneDesc->sceneUpdateType = Disabled;
        }

        getAttribute(sceneDesc->bounceThreshold, physxSceneAPI.GetBounceThresholdAttr(), 1e-06f, FLT_MAX, nullptr);
        getAttribute(sceneDesc->frictionOffsetThreshold, physxSceneAPI.GetFrictionOffsetThresholdAttr(), 0.0f, FLT_MAX, nullptr);
        getAttribute(sceneDesc->frictionCorrelationDistance, physxSceneAPI.GetFrictionCorrelationDistanceAttr(), 0.0f, FLT_MAX, nullptr);
        getAttribute(sceneDesc->maxBiasCoefficient, physxSceneAPI.GetMaxBiasCoefficientAttr(), 0.0f, FLT_MAX, nullptr);

        getAttribute(sceneDesc->timeStepsPerSecond, physxSceneAPI.GetTimeStepsPerSecondAttr(), 1u, UINT_MAX, nullptr);

        getAttribute(sceneDesc->minPosIterationCount, physxSceneAPI.GetMinPositionIterationCountAttr(), 0u, 255u, nullptr);
        getAttribute(sceneDesc->maxPosIterationCount, physxSceneAPI.GetMaxPositionIterationCountAttr(), 0u, 255u, nullptr);
        getAttribute(sceneDesc->minVelIterationCount, physxSceneAPI.GetMinVelocityIterationCountAttr(), 0u, 255u, nullptr);
        getAttribute(sceneDesc->maxVelIterationCount, physxSceneAPI.GetMaxVelocityIterationCountAttr(), 0u, 255u, nullptr);

        if (sceneDesc->minPosIterationCount > sceneDesc->maxPosIterationCount)
        {
            CARB_LOG_ERROR("Physics scene min position iteration count cannot be greater than max iteration count.");
            sceneDesc->minPosIterationCount = sceneDesc->maxPosIterationCount;
        }
        if (sceneDesc->minVelIterationCount > sceneDesc->maxVelIterationCount)
        {
            CARB_LOG_ERROR("Physics scene min velocity iteration count cannot be greater than max iteration count.");
            sceneDesc->minVelIterationCount = sceneDesc->maxVelIterationCount;
        }

        getBoolAttribute(sceneDesc->enableCCD, physxSceneAPI.GetEnableCCDAttr(), nullptr);
        getBoolAttribute(sceneDesc->enableStabilization, physxSceneAPI.GetEnableStabilizationAttr(), nullptr);
        getBoolAttribute(sceneDesc->enableGPUDynamics, physxSceneAPI.GetEnableGPUDynamicsAttr(), nullptr);
        getBoolAttribute(sceneDesc->enableEnhancedDeterminism, physxSceneAPI.GetEnableEnhancedDeterminismAttr(), nullptr);
        getBoolAttribute(sceneDesc->enableExternalForcesEveryIteration, physxSceneAPI.GetEnableExternalForcesEveryIterationAttr(), nullptr);
        getBoolAttribute(sceneDesc->invertedFiltering, physxSceneAPI.GetInvertCollisionGroupFilterAttr(), nullptr);
        getBoolAttribute(sceneDesc->reportKineKine, physxSceneAPI.GetReportKinematicKinematicPairsAttr(), nullptr);
        getBoolAttribute(sceneDesc->reportKineStatic, physxSceneAPI.GetReportKinematicStaticPairsAttr(), nullptr);
        getBoolAttribute(sceneDesc->supportSceneQueries, physxSceneAPI.GetEnableSceneQuerySupportAttr(), nullptr);
        getBoolAttribute(sceneDesc->enableResidualReporting, physxSceneAPI.GetEnableResidualReportingAttr(), nullptr);

        //Query the custom attribute "physxScene:enableRunArticulationContactLast"
        //This will be promoted to PhysxSceneAPI at next major release 108
        getBoolAttribute(sceneDesc->solveArticulationContactLast, inDesc.usdPrim.GetAttribute(kSolveArticulationContactLastToken), nullptr);

        if (physxSceneAPI.GetCollisionSystemAttr())
        {
            TfToken collisionSystem;
            physxSceneAPI.GetCollisionSystemAttr().Get(&collisionSystem);
            if (collisionSystem == PhysxSchemaTokens.Get()->SAT)
                sceneDesc->collisionSystem = eSAT;
        }
        if (physxSceneAPI.GetSolverTypeAttr())
        {
            TfToken solverType;
            physxSceneAPI.GetSolverTypeAttr().Get(&solverType);
            if (solverType == PhysxSchemaTokens.Get()->TGS)
                sceneDesc->solverType = eTGS;
            else if (solverType == PhysxSchemaTokens.Get()->PGS)
                sceneDesc->solverType = ePGS;
        }
        if (physxSceneAPI.GetBroadphaseTypeAttr())
        {
            TfToken bpType;
            physxSceneAPI.GetBroadphaseTypeAttr().Get(&bpType);
            if (bpType == PhysxSchemaTokens.Get()->SAP)
                sceneDesc->broadphaseType = eSAP;
            if (bpType == PhysxSchemaTokens.Get()->MBP)
                sceneDesc->broadphaseType = eMBP;
            if (bpType == PhysxSchemaTokens.Get()->GPU)
                sceneDesc->broadphaseType = eGPU;
            
        }
        if (physxSceneAPI.GetFrictionTypeAttr())
        {
            if (physxSceneAPI.GetFrictionTypeAttr().HasAuthoredValue())
            {
                CARB_LOG_WARN("Usd Physics: scene prim \"%s\", attribute \"frictionType\" is deprecated "
                    "and will be removed in the future.\n",
                    physxSceneAPI.GetPath().GetText());
            }
        }

        if (physxSceneAPI.GetGpuTempBufferCapacityAttr())
            physxSceneAPI.GetGpuTempBufferCapacityAttr().Get(&sceneDesc->gpuTempBufferCapacity);
        if (physxSceneAPI.GetGpuMaxRigidContactCountAttr())
            physxSceneAPI.GetGpuMaxRigidContactCountAttr().Get(&sceneDesc->gpuMaxRigidContactCount);
        if (physxSceneAPI.GetGpuMaxRigidPatchCountAttr())
            physxSceneAPI.GetGpuMaxRigidPatchCountAttr().Get(&sceneDesc->gpuMaxRigidPatchCount);
        if (physxSceneAPI.GetGpuHeapCapacityAttr())
            physxSceneAPI.GetGpuHeapCapacityAttr().Get(&sceneDesc->gpuHeapCapacity);
        if (physxSceneAPI.GetGpuFoundLostPairsCapacityAttr())
            physxSceneAPI.GetGpuFoundLostPairsCapacityAttr().Get(&sceneDesc->gpuFoundLostPairsCapacity);
        if (physxSceneAPI.GetGpuFoundLostAggregatePairsCapacityAttr())
            physxSceneAPI.GetGpuFoundLostAggregatePairsCapacityAttr().Get(&sceneDesc->gpuFoundLostAggregatePairsCapacity);
        if (physxSceneAPI.GetGpuTotalAggregatePairsCapacityAttr())
            physxSceneAPI.GetGpuTotalAggregatePairsCapacityAttr().Get(&sceneDesc->gpuTotalAggregatePairsCapacity);
        if (physxSceneAPI.GetGpuMaxSoftBodyContactsAttr())
            physxSceneAPI.GetGpuMaxSoftBodyContactsAttr().Get(&sceneDesc->gpuMaxDeformableVolumeContacts);
        if (physxSceneAPI.GetGpuMaxDeformableSurfaceContactsAttr())
            physxSceneAPI.GetGpuMaxDeformableSurfaceContactsAttr().Get(&sceneDesc->gpuMaxDeformableSurfaceContacts);
        if (physxSceneAPI.GetGpuMaxParticleContactsAttr())
            physxSceneAPI.GetGpuMaxParticleContactsAttr().Get(&sceneDesc->gpuMaxParticleContacts);
        if (physxSceneAPI.GetGpuCollisionStackSizeAttr())
            physxSceneAPI.GetGpuCollisionStackSizeAttr().Get(&sceneDesc->gpuCollisionStackSize);

        getAttribute(sceneDesc->gpuMaxNumPartitions, physxSceneAPI.GetGpuMaxNumPartitionsAttr(), 1u, 32u, nullptr);
        if (!isPowerOfTwo(sceneDesc->gpuMaxNumPartitions))
            sceneDesc->gpuMaxNumPartitions = 8;

        if (sceneDesc->solverType == ePGS && sceneDesc->enableExternalForcesEveryIteration)
        {
            CARB_LOG_ERROR("Physics scene enable external forces every iteration is not supported by the PGS solver type.");
            sceneDesc->enableExternalForcesEveryIteration = false;
        }
    }

    const PhysxSchemaPhysxResidualReportingAPI physxReportResidualsAPI = PhysxSchemaPhysxResidualReportingAPI::Get(stage, inDesc.usdPrim.GetPrimPath());
    if (physxReportResidualsAPI)
    {
        sceneDesc->reportResiduals = true;
    }

    const PhysxSchemaPhysxSceneQuasistaticAPI physxSceneQuasistaticAPI = PhysxSchemaPhysxSceneQuasistaticAPI::Get(stage, inDesc.usdPrim.GetPrimPath());
    if (physxSceneQuasistaticAPI)
    {
        physxSceneQuasistaticAPI.GetEnableQuasistaticAttr().Get(&sceneDesc->enableQuasistatic);

        const UsdCollectionAPI collection = physxSceneQuasistaticAPI.GetQuasistaticActorsCollectionAPI();
        SdfPathVector targets;
        collection.GetIncludesRel().GetTargets(&targets);
        if (!targets.empty())
        {
            const UsdCollectionMembershipQuery query = collection.ComputeMembershipQuery();
            const SdfPathSet collectionPaths = UsdCollectionAPI::ComputeIncludedPaths(query,
                stage, UsdTraverseInstanceProxies());
            sceneDesc->quasistaticActors = collectionPaths;
        }
    }

    if(sceneDesc->broadphaseType == eGPU)
    {
        static TfToken envIdsBoundsToken("physxScene:envIdInBoundsBitCount");
        GetAuthoredValueOrDefault<int>(&sceneDesc->envIdInBoundsBitCount, inDesc.usdPrim.GetAttribute(envIdsBoundsToken), -1);
        //printf("envIdInBoundsBitCount: %d\n", sceneDesc->envIdInBoundsBitCount);
    }

    return sceneDesc;
}

} // namespace usdparser
} // namespace physx
} // namespace omni
