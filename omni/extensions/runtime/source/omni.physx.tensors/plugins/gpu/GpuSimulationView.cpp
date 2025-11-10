// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "GpuSimulationView.h"

#include "CudaKernels.h"

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>

using namespace pxr;
using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

GpuSimulationView::GpuSimulationView(UsdStageRefPtr stage, GpuSimulationDataPtr gpuSimData)
    : BaseSimulationView(stage), mGpuSimData(gpuSimData)
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
        CARB_LOG_ERROR("Provided pattern list did not match any articulations\n");
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
        CARB_LOG_ERROR("Failed to create articulation view: GPU data not initialized");
        return nullptr;
    }
    std::vector<RigidBodyEntry> entries;
    processRigidBodyEntries(patterns, entries);
    if (entries.empty())
    {
        CARB_LOG_ERROR("Provided pattern list did not match any rigid bodies\n");
        return nullptr;
    }
    GpuRigidBodyView* rbview = new GpuRigidBodyView(this, entries, mDevice);
    mRbViews.push_back(rbview);
    return rbview;
}

GpuSoftBodyView* GpuSimulationView::createSoftBodyView(const char* pattern)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    if (!pattern || !*pattern)
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return nullptr;
    }

    std::vector<SoftBodyEntry> entries;
    findMatchingSoftBodies(pattern, entries);

    if (entries.empty())
    {
        CARB_LOG_ERROR("Pattern '%s' did not match any soft bodies\n", pattern);
        return nullptr;
    }

    GpuSoftBodyView* sbview = new GpuSoftBodyView(this, entries, mDevice);
    mSbViews.push_back(sbview);
    return sbview;
}

GpuSoftBodyMaterialView* GpuSimulationView::createSoftBodyMaterialView(const char* pattern)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    if (!pattern || !*pattern)
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return nullptr;
    }

    if (!mGpuSimData)
    {
        CARB_LOG_ERROR("Failed to create SoftBody material view: GPU data not initialized");
        return nullptr;
    }

    std::vector<SoftBodyMaterialEntry> entries;
    findMatchingSoftBodyMaterials(pattern, entries);

    if (entries.empty())
    {
        CARB_LOG_ERROR("Pattern '%s' did not match any SoftBody materials\n", pattern);
        return nullptr;
    }

    GpuSoftBodyMaterialView* sbmView = new GpuSoftBodyMaterialView(this, entries);
    mSbMaterialViews.push_back(sbmView);
    return sbmView;
}

GpuVolumeDeformableBodyView* GpuSimulationView::createVolumeDeformableBodyView(const char* pattern)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    if (!pattern || !*pattern)
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return nullptr;
    }

    std::vector<DeformableBodyEntry> entries;
    findMatchingVolumeDeformableBodies(pattern, entries);

    if (entries.empty())
    {
        CARB_LOG_ERROR("Pattern '%s' did not match any volume deformable bodies\n", pattern);
        return nullptr;
    }

    GpuVolumeDeformableBodyView* view = new GpuVolumeDeformableBodyView(this, entries, mDevice);
    mVolumeDeformableBodyViews.push_back(view);
    return view;
}

GpuSurfaceDeformableBodyView* GpuSimulationView::createSurfaceDeformableBodyView(const char* pattern)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    if (!pattern || !*pattern)
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return nullptr;
    }

    std::vector<DeformableBodyEntry> entries;
    findMatchingSurfaceDeformableBodies(pattern, entries);

    if (entries.empty())
    {
        CARB_LOG_ERROR("Pattern '%s' did not match any surface deformable bodies\n", pattern);
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

    if (!mGpuSimData)
    {
        CARB_LOG_ERROR("Failed to create deformable material view: GPU data not initialized");
        return nullptr;
    }

    std::vector<DeformableMaterialEntry> entries;
    findMatchingDeformableMaterials(pattern, entries);

    if (entries.empty())
    {
        CARB_LOG_ERROR("Pattern '%s' did not match any deformable materials\n", pattern);
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
    GpuRigidContactView* rcview = new GpuRigidContactView(this, entries, filterPatternSize, maxContactDataCount, mDevice);
    mRcViews.push_back(rcview);
    return rcview;
}

GpuParticleSystemView* GpuSimulationView::createParticleSystemView(const char* pattern)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    if (!pattern || !*pattern)
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return nullptr;
    }

    if (!mGpuSimData)
    {
        CARB_LOG_ERROR("Failed to create particle cloth view: GPU data not initialized");
        return nullptr;
    }

    std::vector<ParticleSystemEntry> entries;
    findMatchingParticleSystems(pattern, entries);

    if (entries.empty())
    {
        CARB_LOG_ERROR("Pattern '%s' did not match any particle systems\n", pattern);
        return nullptr;
    }

    GpuParticleSystemView* psView = new GpuParticleSystemView(this, entries);
    mParticleSysViews.push_back(psView);
    return psView;
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

GpuParticleClothView* GpuSimulationView::createParticleClothView(const char* pattern)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    if (!pattern || !*pattern)
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return nullptr;
    }

    if (!mGpuSimData)
    {
        CARB_LOG_ERROR("Failed to create particle cloth view: GPU data not initialized");
        return nullptr;
    }

    std::vector<ParticleClothEntry> entries;
    findMatchingParticleCloths(pattern, entries);

    if (entries.empty())
    {
        CARB_LOG_ERROR("Pattern '%s' did not match any particle cloths\n", pattern);
        return nullptr;
    }

    GpuParticleClothView* pcView = new GpuParticleClothView(this, entries, mDevice);
    mParticleClothViews.push_back(pcView);
    return pcView;
}

GpuParticleMaterialView* GpuSimulationView::createParticleMaterialView(const char* pattern)
{
    CHECK_VALID_DATA_SIM_RETURN(mGpuSimData, this, nullptr);
    if (!pattern || !*pattern)
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return nullptr;
    }

    if (!mGpuSimData)
    {
        CARB_LOG_ERROR("Failed to create particle material view: GPU data not initialized");
        return nullptr;
    }

    std::vector<ParticleMaterialEntry> entries;
    findMatchingParticleMaterials(pattern, entries);

    if (entries.empty())
    {
        CARB_LOG_ERROR("Pattern '%s' did not match any particle materials\n", pattern);
        return nullptr;
    }

    GpuParticleMaterialView* pmView = new GpuParticleMaterialView(this, entries);
    mParticleMatViews.push_back(pmView);
    return pmView;
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
    mGpuSimData->mScene->getDirectGPUAPI().computeArticulationData(NULL, NULL, PxArticulationGPUAPIComputeType::eUPDATE_KINEMATIC, 0);
}

void GpuSimulationView::enableGpuUsageWarnings(bool enable)
{
    CHECK_VALID_DATA_SIM_NO_RETURN(mGpuSimData, this);
    mGpuSimData->enableGpuUsageWarnings(enable);
}

}
}
}
