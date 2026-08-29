// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "tensors/cpu/CpuSimulationView.h"

#include <carb/logging/Log.h>

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>

#include <common/foundation/TypeCast.h>


using namespace PXR_NS;
using namespace physx;
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
        new CpuRigidContactView(this, entries, filterPatternSize, maxContactDataCount);
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
