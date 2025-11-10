// SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXStats.h"

#include <carb/logging/Log.h>

#include <common/utilities/Utilities.h>

#include <omni/fabric/usd/PathConversion.h>

using namespace pxr;

namespace omni
{
namespace physx
{

PhysXStats::PhysXStats()
{
    // If this breaks, the size of omni::physx::PhysicsSceneStats struct has changed, please revisit PhysXStats!
    static_assert(sizeof(omni::physx::PhysicsSceneStats) == PhysicsSceneStatsSize);

    // Get the stats interface
    mStats = carb::stats::getStatsInterface();
    if (!mStats)
    {
        CARB_LOG_ERROR("Acquiring carb::stats::IStats interface failed.");
    }

    mPhysxStatistics = carb::getCachedInterface<omni::physx::IPhysxStatistics>();
    if (!mPhysxStatistics)
    {
        CARB_LOG_ERROR("Acquiring omni::physx::IPhysxStatistics interface failed.");
    }

    mLastUpdateTime.start();
}

PhysXStats::~PhysXStats()
{
    setStage(nullptr);
    mStage = nullptr;
    mUsdrtStage = nullptr;
}

void PhysXStats::setStage(const UsdStageWeakPtr& stage)
{
    mPhysXStatIdsPerScene.clear();

    // hide all scopes
    for(const auto& scope: mScopeMap)
    {
        mStats->setScopeVisibility(scope.second, false);
    }
    
    if (stage)
    {
        mStageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();
        mStage = stage;
        mUsdrtStage = usdrt::UsdStage::Attach(omni::fabric::UsdStageId(mStageId));
    }
    else
    {
        mStageId = 0u;
        mStage = nullptr;
        mUsdrtStage = nullptr;
    }
}

std::vector<PhysXStats::PhysicsSceneStats> PhysXStats::getStatsPerPhysicsScene()
{
    CARB_PROFILE_ZONE(0, "PhysXStats::getStatsPerPhysicsScene");

    if (!mStage || !mUsdrtStage || !mPhysxStatistics)
    {
        return std::vector<PhysXStats::PhysicsSceneStats>(); // return an empty vector
    }

    static const usdrt::TfToken tokenPhysicsScene("UsdPhysicsScene");

    const usdrt::SdfPathVector vecUsdrtPhysicsScenes = mUsdrtStage->GetPrimsWithTypeName(tokenPhysicsScene);

    // skip inactive scenes and instance proxies and convert paths from usdrt::SdfPath to pxr::SdfPath
    pxr::SdfPathVector vecPhysicsScenes;

    for (const auto& path : vecUsdrtPhysicsScenes)
    {
        omni::fabric::PathC fabricPath = omni::fabric::PathC(path); // convert usdrt::SdfPath to omni::fabric::PathC
        const pxr::SdfPath& sdfPath = omni::fabric::toSdfPath(fabricPath);
        pxr::UsdPrim prim = mStage->GetPrimAtPath(sdfPath);

        if (prim && !prim.IsInstanceProxy() && prim.IsActive())
        {
            vecPhysicsScenes.push_back(sdfPath);
        }
    }

    std::vector<PhysXStats::PhysicsSceneStats> output;
    output.reserve(vecPhysicsScenes.size());

    for (const auto& sdfPath : vecPhysicsScenes)
    {
        omni::physx::PhysicsSceneStats sceneStats;

        if (mPhysxStatistics->getPhysXSceneStatistics(mStageId, asInt(sdfPath), sceneStats))
        {
            output.push_back({ sdfPath.GetString(), sceneStats });
        }
    }

    return output;
}

std::string PhysXStats::getPhysicsSceneName(size_t index)
{
    return std::string(PhysicsSceneScopeName) + std::string(" #") + std::to_string(index);
}

void PhysXStats::createStats(const std::vector<PhysicsSceneStats>& statsPerPhysicsScene)
{
    CARB_PROFILE_ZONE(0, "PhysXStats::createStats");

    if (!mStats)
    {
        return;
    }

    mPhysXStatIdsPerScene.clear();
    size_t statsCount = statsPerPhysicsScene.size();

    if (statsCount == 0)
    {
        return;
    }

    mPhysXStatIdsPerScene.reserve(statsCount);
    mPhysXStatIdsPerScene.insert(mPhysXStatIdsPerScene.end(), statsCount, {});

    for (size_t physxSceneCounter = 0; physxSceneCounter < statsCount; physxSceneCounter++)
    {
        const PhysXStats::PhysicsSceneStats& stats = statsPerPhysicsScene[physxSceneCounter];

        std::string physxSceneStr = getPhysicsSceneName(physxSceneCounter);
        auto it = mScopeMap.find(physxSceneStr);

        carb::stats::ScopeId scopeId;

        // Create or update scope for the scene statistics
        if (it == mScopeMap.end())
        {
            carb::stats::ScopeDesc scopeDesc = {};
            scopeDesc.scopeName = physxSceneStr.c_str();
            scopeDesc.scopeDescription = stats.mPhysicsSceneName.c_str();
            scopeId = mStats->getOrCreateScope(scopeDesc);
            mScopeMap[physxSceneStr] = scopeId;
        }
        else
        {
            // update scope description, which is the physics scene path name
            scopeId = it->second;
            mStats->setScopeDescription(scopeId, stats.mPhysicsSceneName.c_str());
            mStats->setScopeVisibility(scopeId, true);
        }

        // clang-format off
        #define ADD_STAT(stat, desc) setStat(physxSceneCounter, PhysXStat::stat, { #stat, desc, scopeId, carb::stats::StatType::eUint64 })

        ADD_STAT(eNbDynamicRigids, "Number of dynamic rigid bodies");
        ADD_STAT(eNbActiveDynamicRigids, "Number of active dynamic rigid bodies");
        ADD_STAT(eNbStaticRigids, "Number of static rigid bodies");
        ADD_STAT(eNbKinematicBodies, "Number of kinematic rigid bodies");
        ADD_STAT(eNbActiveKinematicBodies, "Number of active kinematic rigid bodies");

        ADD_STAT(eNbArticulations, "Number of articulations");
        ADD_STAT(eNbAggregates, "Number of aggregates");

        // shapes
        ADD_STAT(eNbSphereShapes, "Number of sphere shapes");
        ADD_STAT(eNbBoxShapes, "Number of box shapes");
        ADD_STAT(eNbCapsuleShapes, "Number of capsule shapes");
        ADD_STAT(eNbCylinderShapes, "Number of cylinder shapes");
        ADD_STAT(eNbConvexShapes, "Number of convex shapes");
        ADD_STAT(eNbConeShapes, "Number of cone shapes");
        ADD_STAT(eNbTriMeshShapes, "Number of triangles mesh shapes");
        ADD_STAT(eNbPlaneShapes, "Number of plane shapes");

        // constraints
        ADD_STAT(eNbActiveConstraints, "Number of active constraints");

        // solver
        ADD_STAT(eNbAxisSolverConstraints, "Number of 1D axis constraints(joints+contact) present in current sim. step");
        ADD_STAT(eCompressedContactSize, "Size of the compressed contact stream in current sim. step (bytes)");
        ADD_STAT(eRequiredContactConstraintMemory, "Total required size of the contact constraints in current sim. step (bytes)");
        ADD_STAT(ePeakConstraintMemory, "Peak amount of memory allocated for constraints+joints in current sim. step (bytes)");
        ADD_STAT(eNbDiscreteContactPairsTotal, "Total number of (non CCD) pairs reaching narrow phase");
        ADD_STAT(eNbDiscreteContactPairsWithCacheHits, "Total number of (non CCD) pairs for which contacts are successfully cached");
        ADD_STAT(eNbDiscreteContactPairsWithContacts, "Total number of (non CCD) pairs for which at least 1 contact was generated");

        // broadphase
        ADD_STAT(eNbNewPairs, "Number of new pairs found by BP this frame");
        ADD_STAT(eNbLostPairs, "Number of lost pairs from BP this frame");
        ADD_STAT(eNbNewTouches, "Number of new touches found by NP this frame");
        ADD_STAT(eNbLostTouches, "Number of lost touches from NP this frame");
        ADD_STAT(eNbPartitions, "Number of partitions used by the solver this frame");

        // GPU stats
        ADD_STAT(eGpuMemParticles, "GPU memory in bytes allocated for particle state");
        ADD_STAT(eGpuMemDeformableVolumes, "GPU memory in bytes allocated for deformable volume state");
        ADD_STAT(eGpuMemDeformableSurfaces, "GPU memory in bytes allocated for deformable surface state");
        ADD_STAT(eGpuMemHeap, "GPU memory in bytes allocated for internal heap allocation");
        ADD_STAT(eGpuMemHeapBroadPhase,"GPU heap memory used for broad phase (bytes)");
        ADD_STAT(eGpuMemHeapNarrowPhase, "GPU heap memory used for narrow phase (bytes)");
        ADD_STAT(eGpuMemHeapSolver, "GPU heap memory used for solver (bytes)");
        ADD_STAT(eGpuMemHeapArticulation, "GPU heap memory used for articulations (bytes)");
        ADD_STAT(eGpuMemHeapSimulation, "GPU heap memory used for simulation pipeline (bytes)");
        ADD_STAT(eGpuMemHeapSimulationArticulation,"GPU heap memory used for articulations (bytes)");
        ADD_STAT(eGpuMemHeapSimulationParticles, "GPU heap memory used for particles (bytes)");
        ADD_STAT(eGpuMemHeapSimulationDeformableVolume, "GPU heap memory used for deformable volumes (bytes)");
        ADD_STAT(eGpuMemHeapSimulationDeformableSurface, "GPU heap memory used for deformable surfaces (bytes)");
        ADD_STAT(eGpuMemHeapParticles, "GPU heap memory used for shared buffers in the particles pipeline (bytes)");
        ADD_STAT(eGpuMemHeapDeformableVolumes, "GPU heap memory used for shared buffers in the deformable volume pipeline (bytes)");
        ADD_STAT(eGpuMemHeapDeformableSurfaces, "GPU heap memory used for shared buffers in the deformable surface pipeline (bytes)");
        ADD_STAT(eGpuMemHeapOther, "GPU heap memory not covered by other stats (bytes)");

        // GPU memory config statistics / actual use
        ADD_STAT(eGpuMemTempBufferCapacity, "Actual size needed for PxGpuDynamicsMemoryConfig::tempBufferCapacity (bytes)");
        ADD_STAT(eGpuMemRigidContactCount, "Actual number of rigid contacts needed");
        ADD_STAT(eGpuMemRigidPatchCount, "Actual number of rigid contact patches needed");
        ADD_STAT(eGpuMemFoundLostPairs, "Actual number of lost/found pairs needed");
        ADD_STAT(eGpuMemFoundLostAggregatePairs, "Actual number of lost/found aggregate pairs needed");
        ADD_STAT(eGpuMemTotalAggregatePairs, "Actual number of aggregate pairs needed");
        ADD_STAT(eGpuMemDeformableVolumeContacts, "Actual number of deformable volume contact needed");
        ADD_STAT(eGpuMemDeformableSurfaceContacts, "Actual number of deformable surface contacts needed");
        ADD_STAT(eGpuMemParticleContacts, "Actual number of particle contacts needed");
        ADD_STAT(eGpuMemCollisionStackSize, "Actual size (bytes) needed for the collision stack");

        #undef ADD_STAT
        // clang-format on
    }

    // iterate over remaining already existing physics scene entries and hide them
    size_t index = statsCount;
    while (true)
    {
        std::string physxSceneStr = getPhysicsSceneName(index);

        auto it = mScopeMap.find(physxSceneStr); 
        if (it == mScopeMap.end())
        {
            // item does not exist, exit the loop
            break;
        }

        // hide the scope
        mStats->setScopeVisibility(it->second, false);

        index++;
    }
}

void PhysXStats::updateStats()
{
    CARB_PROFILE_ZONE(0, "PhysXStats::updateStats");

    if (!mStats)
    {
        return;
    }

    std::vector<PhysicsSceneStats> statsPerPhysicsScene = getStatsPerPhysicsScene();

    if (statsPerPhysicsScene.size() != mPhysXStatIdsPerScene.size())
    {
        // clear and recreate
        mPhysXStatIdsPerScene.clear();
        createStats(statsPerPhysicsScene);
    }

    if (statsPerPhysicsScene.size() != mPhysXStatIdsPerScene.size())
    {
        // fatal error, should never happen.
        CARB_LOG_ERROR("PhysXStats::updateStats error: stats arrays mismatch. PhysX Stats won't update.");
        return;
    }

    for (size_t index = 0; index < mPhysXStatIdsPerScene.size(); index++)
    {
        // clang-format off
        #define UPDATE_STAT(stat, field) mStats->setStatUint64(getStat(index, PhysXStat::stat), statsPerPhysicsScene[index].mPhysicsSceneStats.field)

        UPDATE_STAT(eNbDynamicRigids, nbDynamicRigids);
        UPDATE_STAT(eNbActiveDynamicRigids, nbActiveDynamicRigids);
        UPDATE_STAT(eNbStaticRigids, nbStaticRigids);
        UPDATE_STAT(eNbKinematicBodies, nbKinematicBodies);
        UPDATE_STAT(eNbActiveKinematicBodies, nbActiveKinematicBodies);

        UPDATE_STAT(eNbArticulations, nbArticulations);
        UPDATE_STAT(eNbAggregates, nbAggregates);

        UPDATE_STAT(eNbSphereShapes, nbSphereShapes);
        UPDATE_STAT(eNbBoxShapes, nbBoxShapes);
        UPDATE_STAT(eNbCapsuleShapes, nbCapsuleShapes);
        UPDATE_STAT(eNbCylinderShapes, nbCylinderShapes);
        UPDATE_STAT(eNbConvexShapes, nbConvexShapes);
        UPDATE_STAT(eNbConeShapes, nbConeShapes);
        UPDATE_STAT(eNbTriMeshShapes, nbTriMeshShapes);
        UPDATE_STAT(eNbPlaneShapes, nbPlaneShapes);

        UPDATE_STAT(eNbActiveConstraints, nbActiveConstraints);

        UPDATE_STAT(eNbAxisSolverConstraints, nbAxisSolverConstraints);
        UPDATE_STAT(eCompressedContactSize, compressedContactSize);
        UPDATE_STAT(eRequiredContactConstraintMemory, requiredContactConstraintMemory);
        UPDATE_STAT(ePeakConstraintMemory, peakConstraintMemory);
        UPDATE_STAT(eNbDiscreteContactPairsTotal, nbDiscreteContactPairsTotal);
        UPDATE_STAT(eNbDiscreteContactPairsWithCacheHits, nbDiscreteContactPairsWithCacheHits);
        UPDATE_STAT(eNbDiscreteContactPairsWithContacts, nbDiscreteContactPairsWithContacts);

        UPDATE_STAT(eNbNewPairs, nbNewPairs);
        UPDATE_STAT(eNbLostPairs, nbLostPairs);
        UPDATE_STAT(eNbNewTouches, nbNewTouches);
        UPDATE_STAT(eNbLostTouches, nbLostTouches);
        UPDATE_STAT(eNbPartitions, nbPartitions);

        UPDATE_STAT(eGpuMemParticles, gpuMemParticles);
        UPDATE_STAT(eGpuMemDeformableVolumes, gpuMemDeformableVolumes);
        UPDATE_STAT(eGpuMemDeformableSurfaces, gpuMemDeformableSurfaces);
        UPDATE_STAT(eGpuMemHeap, gpuMemHeap);
        UPDATE_STAT(eGpuMemHeapBroadPhase, gpuMemHeapBroadPhase);
        UPDATE_STAT(eGpuMemHeapNarrowPhase, gpuMemHeapNarrowPhase);
        UPDATE_STAT(eGpuMemHeapSolver, gpuMemHeapSolver);
        UPDATE_STAT(eGpuMemHeapArticulation, gpuMemHeapArticulation);
        UPDATE_STAT(eGpuMemHeapSimulation, gpuMemHeapSimulation);
        UPDATE_STAT(eGpuMemHeapSimulationArticulation, gpuMemHeapSimulationArticulation);
        UPDATE_STAT(eGpuMemHeapSimulationParticles, gpuMemHeapSimulationParticles);
        UPDATE_STAT(eGpuMemHeapSimulationDeformableVolume, gpuMemHeapSimulationDeformableVolume);
        UPDATE_STAT(eGpuMemHeapSimulationDeformableSurface, gpuMemHeapSimulationDeformableSurface);
        UPDATE_STAT(eGpuMemHeapParticles, gpuMemHeapParticles);
        UPDATE_STAT(eGpuMemHeapDeformableVolumes, gpuMemHeapDeformableVolumes);
        UPDATE_STAT(eGpuMemHeapDeformableSurfaces, gpuMemHeapDeformableSurfaces);
        UPDATE_STAT(eGpuMemHeapOther, gpuMemHeapOther);

        UPDATE_STAT(eGpuMemTempBufferCapacity, gpuMemTempBufferCapacity);
        UPDATE_STAT(eGpuMemRigidContactCount, gpuMemRigidContactCount);
        UPDATE_STAT(eGpuMemRigidPatchCount, gpuMemRigidPatchCount);
        UPDATE_STAT(eGpuMemFoundLostPairs, gpuMemFoundLostPairs);
        UPDATE_STAT(eGpuMemFoundLostAggregatePairs, gpuMemFoundLostAggregatePairs);
        UPDATE_STAT(eGpuMemTotalAggregatePairs, gpuMemTotalAggregatePairs);
        UPDATE_STAT(eGpuMemDeformableVolumeContacts, gpuMemDeformableVolumeContacts);
        UPDATE_STAT(eGpuMemDeformableSurfaceContacts, gpuMemDeformableSurfaceContacts);
        UPDATE_STAT(eGpuMemParticleContacts, gpuMemParticleContacts);
        UPDATE_STAT(eGpuMemCollisionStackSize, gpuMemCollisionStackSize);

        #undef UPDATE_STAT

        // clang-format on
    }
    
}

void PhysXStats::update()
{
    float elapsedTime = mLastUpdateTime.getElapsedTime<float>(carb::extras::Timer::Scale::eSeconds);
    if (elapsedTime >= kUpdateRate)
    {
        updateStats();
        mLastUpdateTime.start();
    }
}

} // namespace physx
} // namespace omni
