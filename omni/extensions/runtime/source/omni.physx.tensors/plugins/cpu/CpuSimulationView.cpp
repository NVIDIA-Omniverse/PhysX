// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "CpuSimulationView.h"

#include <carb/logging/Log.h>

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>

#include <omni/fabric/IFabric.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/fabric/SimStageWithHistory.h>

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>

#include <common/foundation/TypeCast.h>

#include <usdrt/scenegraph/usd/usd/stage.h>
#include <omni/fabric/connectivity/Connectivity.h>


using namespace pxr;
using namespace physx;
using namespace omni::fabric;
using namespace carb;

namespace omni
{
namespace physx
{
namespace tensors
{

CpuSimulationView::CpuSimulationView(UsdStageRefPtr stage, CpuSimulationDataPtr cpuSimData)
    : BaseSimulationView(stage), mCpuSimData(cpuSimData)
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
        CARB_LOG_ERROR("Provided pattern list did not match any articulations\n");
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
        CARB_LOG_ERROR("Provided pattern list did not match any rigid bodies\n");
        return nullptr;
    }
    CpuRigidBodyView* rbview = new CpuRigidBodyView(this, entries);
    mRbViews.push_back(rbview);
    return rbview;
}

CpuSoftBodyView* CpuSimulationView::createSoftBodyView(const char* pattern)
{
    // CARB_LOG_ERROR("CpuSimulationView::createSoftBodyView is not implemented yet");
    // return nullptr;

    //TODO: soft body API is not supported on CpuSoftBodyView methods need to have an implementation because we already
    // use the soft body view with the numpy frontend

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

    CpuSoftBodyView* sbview = new CpuSoftBodyView(this, entries);
    mSbViews.push_back(sbview);
    return sbview;
}

CpuSoftBodyMaterialView* CpuSimulationView::createSoftBodyMaterialView(const char* pattern)
{
    if (!pattern || !*pattern)
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return nullptr;
    }

    std::vector<SoftBodyMaterialEntry> entries;
    findMatchingSoftBodyMaterials(pattern, entries);

    if (entries.empty())
    {
        CARB_LOG_ERROR("Pattern '%s' did not match any soft body material\n", pattern);
        return nullptr;
    }

    CpuSoftBodyMaterialView* pmView = new CpuSoftBodyMaterialView(this, entries);
    mSbMaterialViews.push_back(pmView);
    return pmView;
}

CpuVolumeDeformableBodyView* CpuSimulationView::createVolumeDeformableBodyView(const char* pattern)
{
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

    CpuVolumeDeformableBodyView* view = new CpuVolumeDeformableBodyView(this, entries);
    mVolumeDeformableBodyViews.push_back(view);
    return view;
}

CpuSurfaceDeformableBodyView* CpuSimulationView::createSurfaceDeformableBodyView(const char* pattern)
{
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

    CpuSurfaceDeformableBodyView* view = new CpuSurfaceDeformableBodyView(this, entries);
    mSurfaceDeformableBodyViews.push_back(view);
    return view;
}

CpuDeformableMaterialView* CpuSimulationView::createDeformableMaterialView(const char* pattern)
{
    if (!pattern || !*pattern)
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return nullptr;
    }

    std::vector<DeformableMaterialEntry> entries;
    findMatchingDeformableMaterials(pattern, entries);

    if (entries.empty())
    {
        CARB_LOG_ERROR("Pattern '%s' did not match any volume deformable body material\n", pattern);
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
        new CpuRigidContactView(this, entries, filterPatternSize, maxContactDataCount);
    mRcViews.push_back(rcview);
    return rcview;
}

CpuParticleSystemView* CpuSimulationView::createParticleSystemView(const char* pattern)
{
    if (!pattern || !*pattern)
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return nullptr;
    }

    std::vector<ParticleSystemEntry> entries;
    findMatchingParticleSystems(pattern, entries);

    if (entries.empty())
    {
        CARB_LOG_ERROR("Pattern '%s' did not match any particle systems\n", pattern);
        return nullptr;
    }

    CpuParticleSystemView* psView = new CpuParticleSystemView(this, entries);
    mParticleSysViews.push_back(psView);
    return psView;
}


omni::physics::tensors::IParticleClothView* CpuSimulationView::createParticleClothView(const char* pattern)
{
    CARB_LOG_ERROR("CpuSimulationView::createParticleClothView is not implemented yet");
    return nullptr;
}

CpuParticleMaterialView* CpuSimulationView::createParticleMaterialView(const char* pattern)
{
    if (!pattern || !*pattern)
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return nullptr;
    }

    std::vector<ParticleMaterialEntry> entries;
    findMatchingParticleMaterials(pattern, entries);

    if (entries.empty())
    {
        CARB_LOG_ERROR("Pattern '%s' did not match any particle materials\n", pattern);
        return nullptr;
    }

    CpuParticleMaterialView* pmView = new CpuParticleMaterialView(this, entries);
    mParticleMatViews.push_back(pmView);
    return pmView;
}

CpuSdfShapeView* CpuSimulationView::createSdfShapeView(const char* pattern, uint32_t numSamplePoints)
{
    CARB_LOG_ERROR("CpuSimulationView::createSdfShapeView is not implemented yet");
    return nullptr;
}

void CpuSimulationView::clearForces()
{
    // printf("~!~!~! clearing: CpuSimulationView::clearForces\n");
    if (mCpuSimData)
    {
        mCpuSimData->clearForces();
    }
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
