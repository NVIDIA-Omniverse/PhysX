// SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Types.h>
#include <carb/extras/Timer.h>
#include <carb/stats/Stats.h>

#include <omni/physx/IPhysxStatistics.h>
#include <private/omni/physx/PhysxUsd.h>

#include <usdrt/scenegraph/usd/usd/stage.h>

namespace omni
{
namespace physx
{

class PhysXStats
{
public:
    PhysXStats();
    ~PhysXStats();

    void setStage(const pxr::UsdStageWeakPtr& stage);
    void update();

private:
    struct PhysicsSceneStats
    {
        std::string mPhysicsSceneName;
        omni::physx::PhysicsSceneStats mPhysicsSceneStats;
    };

    std::vector<PhysicsSceneStats> getStatsPerPhysicsScene();

    void createStats(const std::vector<PhysicsSceneStats>& statsPerPhysicsScene);
    void updateStats();

    carb::stats::StatId getStat(size_t sceneIndex, PhysXStat stat) const
    {
        return mPhysXStatIdsPerScene[sceneIndex][static_cast<size_t>(stat)];
    }

    void setStat(size_t sceneIndex, PhysXStat stat, const carb::stats::StatDesc& statDesc)
    {
        mPhysXStatIdsPerScene[sceneIndex][static_cast<size_t>(stat)] = mStats->getOrCreateStat(statDesc);
    }

    static std::string getPhysicsSceneName(size_t index);

private:
    // size of omni::physx::PhysicsSceneStats struct this code is compatible with
    static constexpr size_t PhysicsSceneStatsSize{ 296 };
    // Stats Scope name for UsdPhysicsScene
    static constexpr const char* PhysicsSceneScopeName{ "PhysX Scene" };

    static constexpr float kUpdateRate{ 1.0f }; // Update interval [in seconds]
    carb::extras::Timer mLastUpdateTime; // The amount of time elapsed since the last update

    uint64_t mStageId{ 0u }; // USD stage id
    pxr::UsdStageWeakPtr mStage; // stage for retrieving prims
    usdrt::UsdStageRefPtr mUsdrtStage; // USDRT stage for fast traversing prims

    // Dictionary that holds physics scene name (key) and scopeId (value)
    std::map<std::string, carb::stats::ScopeId> mScopeMap;

    carb::stats::IStats* mStats{ nullptr }; // IStats interface
    omni::physx::IPhysxStatistics* mPhysxStatistics{ nullptr }; // IPhysxStatistics interface instance

    /// The PhysX statistics array
    typedef std::array<carb::stats::StatId, static_cast<size_t>(PhysXStat::eCount)> PhysXStatIdsArray;
    std::vector<PhysXStatIdsArray> mPhysXStatIdsPerScene; ///< Vector of PhysX stat ids per UsdPhysicsScene
};

} // namespace physx
} // namespace omni
