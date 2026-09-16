// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-TENSOR-VIEW-001
 * @covers AC-7
 *
 * @implements REQ-READ-VEHICLE-001
 * @covers AC-4, AC-5
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-40
 */

#include "tensors/cpu/CpuSimulationView.h"

#include "tensors/cpu/CpuPointInstancerView.h"

#include "tensors/base/SupersetRigidEntries.h"

#include "internal/InternalScene.h" // vehicleView reads the scene's vehicle array

#include <carb/Assert.h>
#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>

#include <utility>

using namespace physx;
using namespace carb;

namespace omni
{
namespace physx
{
namespace tensors
{

CpuSimulationView::CpuSimulationView(usdparser::AttachedStage* attachedStage,
                                     PxScene* scene,
                                     CpuSimulationDataPtr cpuSimData,
                                     bool notifyWhenSimStopped)
    : BaseSimulationView(attachedStage, scene, notifyWhenSimStopped), mCpuSimData(cpuSimData)
{
}

CpuSimulationView::~CpuSimulationView()
{
}

CpuArticulationView* CpuSimulationView::createArticulationView(const char* pattern){
    return createArticulationView(std::vector<std::string>{ pattern });
}

CpuArticulationView* CpuSimulationView::createArticulationView(const std::vector<std::string>& patterns)
{
    std::vector<ArticulationEntry> entries;
    processArticulationEntries(patterns, entries);
    if (entries.empty())
    {
        if (isNoMatchLoggingQuiet())
        {
            CARB_LOG_INFO("Provided pattern list did not match any articulations\n");
        }
        else
        {
            CARB_LOG_ERROR("Provided pattern list did not match any articulations\n");
        }
        return nullptr;
    }
    CpuArticulationView* aview = new CpuArticulationView(this, entries);
    mArtiViews.push_back(aview);
    return aview;
}

CpuRigidBodyView* CpuSimulationView::createRigidBodyView(const char* pattern)
{
    return createRigidBodyView(std::vector<std::string>{ pattern });
}

CpuRigidBodyView* CpuSimulationView::createRigidBodyView(const std::vector<std::string>& patterns)
{
    std::vector<RigidBodyEntry> entries;
    processRigidBodyEntries(patterns, entries);
    if (entries.empty())
    {
        if (isNoMatchLoggingQuiet())
        {
            CARB_LOG_INFO("Provided pattern list did not match any rigid bodies\n");
        }
        else
        {
            CARB_LOG_ERROR("Provided pattern list did not match any rigid bodies\n");
        }
        return nullptr;
    }
    CpuRigidBodyView* rbview = new CpuRigidBodyView(this, entries);
    mRbViews.push_back(rbview);
    return rbview;
}

CpuRigidBodyView* CpuSimulationView::createRigidBodyViewFromEntries(const std::vector<RigidBodyEntry>& entries)
{
    if (entries.empty())
    {
        return nullptr;
    }
    CpuRigidBodyView* rbview = new CpuRigidBodyView(this, entries);
    mRbViews.push_back(rbview);
    return rbview;
}

CpuPointInstancerView* CpuSimulationView::createPointInstancerViewFromEntries(
    const std::vector<PointInstancerEntry>& entries)
{
    if (entries.empty())
        return nullptr;
    CpuPointInstancerView* view = new CpuPointInstancerView(this, entries);
    mPointInstancerViews.push_back(view);
    return view;
}

CpuRigidBodyView* CpuSimulationView::supersetRigidView(
    const std::unordered_map<const PxRigidBody*, PxU32>** outRowMap)
{
    if (outRowMap)
        *outRowMap = nullptr;

    if (!mSupersetRigidView)
    {
        std::vector<RigidBodyEntry> entries;
        collectSupersetRigidEntries(mScene, entries);
        if (entries.empty())
            return nullptr;

        CpuRigidBodyView* rbv = createRigidBodyViewFromEntries(entries);
        if (!rbv)
            return nullptr;

        // No hasUnresolvedEntries() check as on the GPU view: a CPU entry holds the actor pointer
        // directly, so nothing is resolved and nothing can come back wrong. A stale cache is caught
        // by the caller's row lookup (a body missing from the map below means the view predates it)
        // plus the backend's per-acquire entry validation.
        mSupersetRigidView = rbv;
        mSupersetRigidRowMap.clear();
        mSupersetRigidRowMap.reserve(entries.size());
        for (PxU32 row = 0; row < entries.size(); ++row)
            mSupersetRigidRowMap.emplace(entries[row].body, row);
    }

    if (outRowMap)
        *outRowMap = &mSupersetRigidRowMap;
    return mSupersetRigidView;
}

CpuArticulationView* CpuSimulationView::supersetArticulationView(
    const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>** outRowMap,
    const std::vector<ArticulationEntry>** outEntries)
{
    if (outRowMap)
        *outRowMap = nullptr;
    if (outEntries)
        *outEntries = nullptr;

    if (!mSupersetArtiView)
    {
        const PxU32 numArtis = mScene->getNbArticulations();
        if (numArtis == 0)
            return nullptr;
        std::vector<PxArticulationReducedCoordinate*> artis(numArtis);
        mScene->getArticulations(artis.data(), numArtis);

        std::vector<ArticulationEntry> entries;
        entries.reserve(numArtis);
        for (PxArticulationReducedCoordinate* arti : artis)
        {
            ArticulationEntry e;
            if (!arti || !buildArticulationEntry(arti, omni::physics::parse::ObjectKey{}, e))
                return nullptr;
            entries.push_back(std::move(e));
        }

        CpuArticulationView* av = createArticulationViewFromEntries(entries);
        if (!av)
            return nullptr;

        // Guarded like the superset rigid view: the caller's row lookup plus the backend's
        // per-acquire entry validation catch a stale cache.
        mSupersetArtiView = av;
        mSupersetArtiEntries = std::move(entries);
        mSupersetArtiRowMap.clear();
        mSupersetArtiRowMap.reserve(artis.size());
        for (PxU32 row = 0; row < artis.size(); ++row)
            mSupersetArtiRowMap.emplace(artis[row], row);
    }

    if (outRowMap)
        *outRowMap = &mSupersetArtiRowMap;
    if (outEntries)
        *outEntries = &mSupersetArtiEntries;
    return mSupersetArtiView;
}

BaseVehicleView* CpuSimulationView::vehicleView(omni::physx::internal::InternalScene& scene)
{
    // A mismatched InternalScene would build the view from another scene's vehicles while validating
    // the epoch against a counter that never moves for them.
    CARB_ASSERT(scene.getScene() == mScene);

    // The epoch changes when a vehicle is added, removed, moved between the enabled and disabled
    // halves of the array, or loses a wheel attachment. It starts at 1, so a never-built view (0)
    // also takes this branch.
    if (mVehicleView.getBuiltEpoch() != scene.mVehicleSetEpoch)
        mVehicleView.rebuild(scene);
    return mVehicleView.getCount() ? &mVehicleView : nullptr;
}

CpuArticulationView* CpuSimulationView::createArticulationViewFromEntries(const std::vector<ArticulationEntry>& entries)
{
    if (entries.empty())
    {
        return nullptr;
    }
    CpuArticulationView* aview = new CpuArticulationView(this, entries);
    mArtiViews.push_back(aview);
    return aview;
}

CpuVolumeDeformableBodyView* CpuSimulationView::createVolumeDeformableBodyView(const char* pattern)
{
    return createVolumeDeformableBodyView(std::vector<std::string>{ pattern });
}

CpuVolumeDeformableBodyView* CpuSimulationView::createVolumeDeformableBodyView(const std::vector<std::string>& patterns)
{
    std::vector<DeformableBodyEntry> entries;
    processVolumeDeformableBodyEntries(patterns, entries);
    if (entries.empty())
    {
        CARB_LOG_ERROR("Provided pattern list did not match any volume deformable bodies\n");
        return nullptr;
    }
    CpuVolumeDeformableBodyView* view = new CpuVolumeDeformableBodyView(this, entries);
    mVolumeDeformableBodyViews.push_back(view);
    return view;
}

CpuSurfaceDeformableBodyView* CpuSimulationView::createSurfaceDeformableBodyView(const char* pattern)
{
    return createSurfaceDeformableBodyView(std::vector<std::string>{ pattern });
}

CpuSurfaceDeformableBodyView* CpuSimulationView::createSurfaceDeformableBodyView(const std::vector<std::string>& patterns)
{
    std::vector<DeformableBodyEntry> entries;
    processSurfaceDeformableBodyEntries(patterns, entries);
    if (entries.empty())
    {
        CARB_LOG_ERROR("Provided pattern list did not match any surface deformable bodies\n");
        return nullptr;
    }
    CpuSurfaceDeformableBodyView* view = new CpuSurfaceDeformableBodyView(this, entries);
    mSurfaceDeformableBodyViews.push_back(view);
    return view;
}

CpuDeformableMaterialView* CpuSimulationView::createDeformableMaterialView(const char* pattern)
{
    return createDeformableMaterialView(std::vector<std::string>{ pattern });
}

CpuDeformableMaterialView* CpuSimulationView::createDeformableMaterialView(const std::vector<std::string>& patterns)
{
    std::vector<DeformableMaterialEntry> entries;
    processDeformableMaterialEntries(patterns, entries);
    if (entries.empty())
    {
        CARB_LOG_ERROR("Provided pattern list did not match any deformable materials\n");
        return nullptr;
    }
    CpuDeformableMaterialView* view = new CpuDeformableMaterialView(this, entries);
    mDeformableMaterialViews.push_back(view);
    return view;
}

//DEPRECATED
CpuRigidContactView* CpuSimulationView::createRigidContactView(const char* pattern,
                                                               const char** _filterPatterns,
                                                               uint32_t numFilterPatterns,
                                                               uint32_t maxContactDataCount)
{
    CARB_LOG_ERROR("use of createRigidContactView with this signature is deprecated. Please use the new API");
    return nullptr;
}

CpuRigidContactView* CpuSimulationView::createRigidContactView(const std::string pattern,
                                                               const std::vector<std::string>& filterPatterns,
                                                               uint32_t maxContactDataCount)
{
    return createRigidContactView(
        std::vector<std::string>{ pattern }, std::vector<std::vector<std::string>>{ filterPatterns }, maxContactDataCount);
}
CpuRigidContactView* CpuSimulationView::createRigidContactView(const std::vector<std::string>& patterns,
                                                               const std::vector<std::vector<std::string>>& filterPatterns,
                                                               uint32_t maxContactDataCount)
{
    std::vector<RigidContactSensorEntry> entries;
    uint32_t filterPatternSize = 0;
    processRigidContactViewEntries(patterns, filterPatterns, entries, filterPatternSize);
    if (entries.empty())
    {
        CARB_LOG_ERROR("Provided patterns for sensor and filters did not match any rigid contact entries\n");
        return nullptr;
    }
    CpuRigidContactView* rcview =
        new CpuRigidContactView(this, std::move(entries), filterPatternSize, maxContactDataCount);
    mRcViews.push_back(rcview);
    return rcview;
}

CpuSdfShapeView* CpuSimulationView::createSdfShapeView(const char* pattern, uint32_t numSamplePoints)
{
    CARB_LOG_ERROR("CpuSimulationView::createSdfShapeView is not implemented yet");
    return nullptr;
}

void CpuSimulationView::clearForces()
{
    CHECK_VALID_DATA_SIM_NO_RETURN(mCpuSimData, this);
    mCpuSimData->clearForces();
}

bool CpuSimulationView::flush()
{
    // nothing to do
    return true;
}

void CpuSimulationView::enableGpuUsageWarnings(bool enable)
{
    // nothing to do
    return;
}

}
}
}
