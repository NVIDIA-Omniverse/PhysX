// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-TENSOR-VIEW-001
 * @covers AC-7
 *
 * @implements REQ-READ-CORE-001
 * @covers AC-8
 *
 * @implements REQ-INPUT-CORE-001
 * @covers AC-10
 *
 * @implements REQ-TENSOR-CPU-ONLY-001
 * @covers AC-4 AC-6
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-40
 */

#include "tensors/gpu/GpuSimulationView.h"

#include "tensors/gpu/GpuPointInstancerView.h"

#include "tensors/base/SupersetRigidEntries.h"

#include "tensors/gpu/CudaKernels.h"

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>

#include <utility>

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

GpuSimulationView::GpuSimulationView(usdparser::AttachedStage* attachedStage,
                                     PxScene* scene,
                                     GpuSimulationDataPtr gpuSimData,
                                     bool notifyWhenSimStopped)
    : BaseSimulationView(attachedStage, scene, notifyWhenSimStopped), mGpuSimData(gpuSimData)
{
    if (mGpuSimData)
    {
        mDevice = mGpuSimData->mDevice;
    }
}

GpuSimulationView::~GpuSimulationView()
{
}


GpuArticulationView* GpuSimulationView::createArticulationView(const char* pattern){
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    return createArticulationView(std::vector<std::string>{ pattern });
}


GpuArticulationView* GpuSimulationView::createArticulationView(const std::vector<std::string>& patterns)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    if (!mGpuSimData)
    {
        CARB_LOG_ERROR("Failed to create articulation view: GPU data not initialized");
        return nullptr;
    }
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
    GpuArticulationView* aview = new GpuArticulationView(this, entries, mDevice);
    mArtiViews.push_back(aview);
    return aview;
}

GpuRigidBodyView* GpuSimulationView::createRigidBodyView(const char* pattern)
{
    return createRigidBodyView(std::vector<std::string>{ pattern });
}

GpuRigidBodyView* GpuSimulationView::createRigidBodyView(const std::vector<std::string>& patterns)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    if (!mGpuSimData)
    {
        CARB_LOG_ERROR("Failed to create rigid body view: GPU data not initialized");
        return nullptr;
    }
    std::vector<RigidBodyEntry> entries;
    processRigidBodyEntries(patterns, entries);

    // OMPE-103213 / REQ-TENSOR-CPU-ONLY-001 AC-6: DirectGPU has no row for
    // eDISABLE_SIMULATION rigid dynamics. Omit them at create so a wildcard
    // pattern that also matches disabled actors still yields a valid view over
    // the enabled set. Re-enable is out-of-band; recreate after enable to
    // include the body again. CPU createRigidBodyView does not filter.
    {
        std::vector<RigidBodyEntry> enabled;
        enabled.reserve(entries.size());
        for (const RigidBodyEntry& entry : entries)
        {
            if (entry.type == RigidBodyType::eRigidDynamic && entry.body &&
                entry.body->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION))
            {
                continue;
            }
            enabled.push_back(entry);
        }
        entries.swap(enabled);
    }

    if (entries.empty())
    {
        if (isNoMatchLoggingQuiet())
        {
            CARB_LOG_INFO("Provided pattern list did not match any enabled rigid bodies\n");
        }
        else
        {
            CARB_LOG_ERROR("Provided pattern list did not match any enabled rigid bodies\n");
        }
        return nullptr;
    }
    GpuRigidBodyView* rbview = new GpuRigidBodyView(this, entries, mDevice);
    mRbViews.push_back(rbview);
    return rbview;
}

GpuArticulationView* GpuSimulationView::createArticulationViewFromEntries(const std::vector<ArticulationEntry>& entries)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    if (!mGpuSimData)
    {
        CARB_LOG_ERROR("Failed to create articulation view from entries: GPU data not initialized");
        return nullptr;
    }
    if (entries.empty())
    {
        return nullptr;
    }
    GpuArticulationView* aview = new GpuArticulationView(this, entries, mDevice);
    mArtiViews.push_back(aview);
    return aview;
}

GpuRigidBodyView* GpuSimulationView::createRigidBodyViewFromEntries(const std::vector<RigidBodyEntry>& entries)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    if (!mGpuSimData)
    {
        CARB_LOG_ERROR("Failed to create rigid body view from entries: GPU data not initialized");
        return nullptr;
    }
    if (entries.empty())
    {
        return nullptr;
    }

    // Unlike a fixed-membership tensor view, the ovstage superset retains disabled bodies as
    // sentinel records. Its actor-to-row map stays stable across disable/re-enable; the ovstage
    // gather exposes only enabled rows, while query discovery still reports every matching prim.
    // This is what lets unrelated rows and a later disableSimulation=0 write keep working.
    GpuRigidBodyView* rbview = new GpuRigidBodyView(this, entries, mDevice, /*invalidateOnDisabledRd=*/false);
    mRbViews.push_back(rbview);
    return rbview;
}

GpuPointInstancerView* GpuSimulationView::createPointInstancerViewFromEntries(
    const std::vector<PointInstancerEntry>& entries)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    if (!mGpuSimData)
    {
        CARB_LOG_ERROR("Failed to create point instancer view from entries: GPU data not initialized");
        return nullptr;
    }
    if (entries.empty())
        return nullptr;

    GpuPointInstancerView* view = new GpuPointInstancerView(this, entries);
    // Unusable means this view no longer describes the scene (an instance with no superset row) or
    // never got its device memory; either way the caller must rebuild rather than read holes.
    // Checked in the factory so an unusable view never escapes it.
    if (!view->isUsable())
    {
        view->release();
        return nullptr;
    }
    mPointInstancerViews.push_back(view);
    return view;
}

GpuVolumeDeformableBodyView* GpuSimulationView::createVolumeDeformableBodyView(const char* pattern)
{
    return createVolumeDeformableBodyView(std::vector<std::string>{ pattern });
}

GpuVolumeDeformableBodyView* GpuSimulationView::createVolumeDeformableBodyView(const std::vector<std::string>& patterns)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    std::vector<DeformableBodyEntry> entries;
    processVolumeDeformableBodyEntries(patterns, entries);
    if (entries.empty())
    {
        CARB_LOG_ERROR("Provided pattern list did not match any volume deformable bodies\n");
        return nullptr;
    }
    GpuVolumeDeformableBodyView* view = new GpuVolumeDeformableBodyView(this, entries, mDevice);
    mVolumeDeformableBodyViews.push_back(view);
    return view;
}

GpuSurfaceDeformableBodyView* GpuSimulationView::createSurfaceDeformableBodyView(const char* pattern)
{
    return createSurfaceDeformableBodyView(std::vector<std::string>{ pattern });
}

GpuSurfaceDeformableBodyView* GpuSimulationView::createSurfaceDeformableBodyView(const std::vector<std::string>& patterns)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    std::vector<DeformableBodyEntry> entries;
    processSurfaceDeformableBodyEntries(patterns, entries);
    if (entries.empty())
    {
        CARB_LOG_ERROR("Provided pattern list did not match any surface deformable bodies\n");
        return nullptr;
    }
    GpuSurfaceDeformableBodyView* view = new GpuSurfaceDeformableBodyView(this, entries, mDevice);
    mSurfaceDeformableBodyViews.push_back(view);
    return view;
}

GpuDeformableMaterialView* GpuSimulationView::createDeformableMaterialView(const char* pattern)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    if (!pattern || !*pattern)
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return nullptr;
    }

    std::vector<DeformableMaterialEntry> entries;
    std::unordered_set<const ::physx::PxDeformableMaterial*> seenMaterials;
    findMatchingDeformableMaterials(pattern, entries, seenMaterials);

    if (entries.empty())
    {
        CARB_LOG_ERROR("Pattern '%s' did not match any volume deformable body material\n", pattern);
        return nullptr;
    }

    GpuDeformableMaterialView* view = new GpuDeformableMaterialView(this, entries);
    mDeformableMaterialViews.push_back(view);
    return view;
}

GpuDeformableMaterialView* GpuSimulationView::createDeformableMaterialView(const std::vector<std::string>& patterns)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    std::vector<DeformableMaterialEntry> entries;
    processDeformableMaterialEntries(patterns, entries);
    if (entries.empty())
    {
        CARB_LOG_ERROR("Provided pattern list did not match any deformable materials\n");
        return nullptr;
    }
    GpuDeformableMaterialView* view = new GpuDeformableMaterialView(this, entries);
    mDeformableMaterialViews.push_back(view);
    return view;
}

//DEPRECATED
GpuRigidContactView* GpuSimulationView::createRigidContactView(const char* pattern,
                                                               const char** _filterPatterns,
                                                               uint32_t numFilterPatterns,
                                                               uint32_t maxContactDataCount)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    CARB_LOG_ERROR("use of createRigidContactView with this signature is deprecated. Please use the new API");
    return nullptr;
}

GpuRigidContactView* GpuSimulationView::createRigidContactView(const std::string pattern,
                                                               const std::vector<std::string>& filterPatterns,
                                                               uint32_t maxContactDataCount)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    return createRigidContactView(
        std::vector<std::string>{ pattern }, std::vector<std::vector<std::string>>{ filterPatterns }, maxContactDataCount);
}
GpuRigidContactView* GpuSimulationView::createRigidContactView(const std::vector<std::string>& patterns,
                                                               const std::vector<std::vector<std::string>>& filterPatterns,
                                                               uint32_t maxContactDataCount)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    if (!mGpuSimData)
    {
        CARB_LOG_ERROR("Failed to create articulation view: GPU data not initialized");
        return nullptr;
    }
    std::vector<RigidContactSensorEntry> entries;
    uint32_t filterPatternSize = 0;
    processRigidContactViewEntries(patterns, filterPatterns, entries, filterPatternSize);
    if (entries.empty())
    {
        CARB_LOG_ERROR("Provided patterns for sensor and filters did not match any rigid contact entries\n");
        return nullptr;
    }
    GpuRigidContactView* rcview =
        new GpuRigidContactView(this, std::move(entries), filterPatternSize, maxContactDataCount, mDevice);
    mRcViews.push_back(rcview);
    return rcview;
}

GpuSdfShapeView* GpuSimulationView::createSdfShapeView(const char* pattern, uint32_t numSamplePoints)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    if (!pattern || !*pattern)
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return nullptr;
    }

    if (!mGpuSimData)
    {
        CARB_LOG_ERROR("Failed to create SDF view: GPU data not initialized");
        return nullptr;
    }

    std::vector<SdfShapeEntry> entries;
    findMatchingSDFShapes(pattern, entries, numSamplePoints);

    if (entries.empty())
    {
        CARB_LOG_ERROR("Pattern '%s' did not match any SDF\n", pattern);
        return nullptr;
    }

    GpuSdfShapeView* sdfView = new GpuSdfShapeView(this, entries, mDevice);
    mSDFViews.push_back(sdfView);
    return sdfView;
}

void GpuSimulationView::clearForces()
{
    CHECK_VALID_DATA_SIM_NO_RETURN(mGpuSimData, this);
    mGpuSimData->clearForces();
}

bool GpuSimulationView::flush()
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, false);
    return mGpuSimData->flush();
}


void GpuSimulationView::updateArticulationsKinematic()
{
    CHECK_VALID_DATA_SIM_NO_RETURN(mGpuSimData, this);
    PhysxCudaContextGuard ctxGuard(mGpuSimData->mCudaContextManager);
    mGpuSimData->mScene->getDirectGPUAPI().computeArticulationData(NULL, NULL, PxArticulationGPUAPIComputeType::eUPDATE_KINEMATIC, 0);
}

void GpuSimulationView::enableGpuUsageWarnings(bool enable)
{
    CHECK_VALID_DATA_SIM_NO_RETURN(mGpuSimData, this);
    mGpuSimData->enableGpuUsageWarnings(enable);
}


GpuRigidBodyView* GpuSimulationView::supersetRigidView(
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

        GpuRigidBodyView* rbv = createRigidBodyViewFromEntries(entries);
        // Actors the simulation data does not know: it does not describe this scene, so the caller
        // must have the backend rebuild rather than read sentinel rows. Released here because it is
        // this view's child and would otherwise outlive the failure.
        if (!rbv || rbv->hasUnresolvedEntries())
        {
            if (rbv)
                rbv->release();
            return nullptr;
        }

        // collectSupersetRigidEntries walked the whole scene, so this view is the one that may
        // retire the scene-wide disable hint when its refresh finds every body enabled again.
        rbv->markAsSceneSuperset();

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

GpuArticulationView* GpuSimulationView::supersetArticulationView(
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

        GpuArticulationView* av = createArticulationViewFromEntries(entries);
        if (!av)
            return nullptr;

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

}
}
}
