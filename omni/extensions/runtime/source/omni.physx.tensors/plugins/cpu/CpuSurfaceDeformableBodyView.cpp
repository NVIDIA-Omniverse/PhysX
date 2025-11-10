// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "CpuSurfaceDeformableBodyView.h"
#include "CpuSimulationView.h"

#include "../GlobalsAreBad.h"
#include "../CommonTypes.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

CpuSurfaceDeformableBodyView::CpuSurfaceDeformableBodyView(CpuSimulationView* sim, const std::vector<DeformableBodyEntry>& entries)
    : BaseSurfaceDeformableBodyView(sim, entries)
{
    uint32_t numBodies = uint32_t(mEntries.size());

    // initialize default indices
    mAllIndices.resize(numBodies);
    for (PxU32 i = 0; i < numBodies; i++)
    {
        mAllIndices[i] = i;
    }

    mCpuSimData = sim->getCpuSimulationData();
}

CpuSurfaceDeformableBodyView::~CpuSurfaceDeformableBodyView()
{
}

bool CpuSurfaceDeformableBodyView::getSimulationElementIndices(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getSimulationElementIndices is not implemented yet");
    return false;
};

bool CpuSurfaceDeformableBodyView::getSimulationNodalPositions(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getSimulationNodalPositions is not implemented yet");
    return false;
};

bool CpuSurfaceDeformableBodyView::setSimulationNodalPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CARB_LOG_ERROR("CpuSimulationView::setSimulationNodalPositions is not implemented yet");
    return false;
};

bool CpuSurfaceDeformableBodyView::getSimulationNodalVelocities(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getSimulationNodalVelocities is not implemented yet");
    return false;
};

bool CpuSurfaceDeformableBodyView::setSimulationNodalVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CARB_LOG_ERROR("CpuSimulationView::setSimulationNodalVelocities is not implemented yet");
    return false;
};

bool CpuSurfaceDeformableBodyView::getSimulationNodalKinematicTargets(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getSimulationNodalKinematicTargets is not implemented yet");
    return false;
};

bool CpuSurfaceDeformableBodyView::setSimulationNodalKinematicTargets(const TensorDesc* srcTensor,
                                                                      const TensorDesc* indexTensor)
{
    CARB_LOG_ERROR("CpuSimulationView::setSimulationNodalKinematicTargets is not implemented yet");
    return false;
};

bool CpuSurfaceDeformableBodyView::getRestElementIndices(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getRestElementIndices is not implemented yet");
    return false;
};

bool CpuSurfaceDeformableBodyView::getRestNodalPositions(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getRestNodalPositions is not implemented yet");
    return false;
};

bool CpuSurfaceDeformableBodyView::getCollisionElementIndices(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getCollisionElementIndices is not implemented yet");
    return false;
};

bool CpuSurfaceDeformableBodyView::getCollisionNodalPositions(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getCollisionNodalPositions is not implemented yet");
    return false;
};

} // namespace tensors
} // namespace physx
} // namespace omni
