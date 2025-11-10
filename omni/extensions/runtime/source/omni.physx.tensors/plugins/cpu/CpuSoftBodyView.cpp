// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "CpuSoftBodyView.h"
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

CpuSoftBodyView::CpuSoftBodyView(CpuSimulationView* sim, const std::vector<SoftBodyEntry>& entries)
    : BaseSoftBodyView(sim, entries)
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

CpuSoftBodyView::~CpuSoftBodyView()
{
}

bool CpuSoftBodyView::getElementIndices(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getElementIndices is not implemented yet");
    return false;
};
bool CpuSoftBodyView::getSimElementIndices(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getSimElementIndices is not implemented yet");
    return false;
};

bool CpuSoftBodyView::getNodalPositions(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getNodalPositions is not implemented yet");
    return false;
};
bool CpuSoftBodyView::getSimNodalPositions(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getSimNodalPositions is not implemented yet");
    return false;
};

bool CpuSoftBodyView::getElementStresses(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getElementStresses is not implemented yet");
    return false;
};
bool CpuSoftBodyView::getSimElementStresses(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getSimElementStresses is not implemented yet");
    return false;
};

bool CpuSoftBodyView::getElementRestPoses(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getElementRestPoses is not implemented yet");
    return false;
};
bool CpuSoftBodyView::getSimElementRestPoses(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getSimElementRestPoses is not implemented yet");
    return false;
};

bool CpuSoftBodyView::getElementRotations(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getElementRotations is not implemented yet");
    return false;
};
bool CpuSoftBodyView::getSimElementRotations(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getSimElementRotations is not implemented yet");
    return false;
};

bool CpuSoftBodyView::getElementDeformationGradients(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getElementDeformationGradients is not implemented yet");
    return false;
};
bool CpuSoftBodyView::getSimElementDeformationGradients(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getSimElementDeformationGradients is not implemented yet");
    return false;
};

bool CpuSoftBodyView::getSimKinematicTargets(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getSimKinematicTargets is not implemented yet");
    return false;
};
bool CpuSoftBodyView::getSimNodalVelocities(const TensorDesc* dstTensor) const
{
    CARB_LOG_ERROR("CpuSimulationView::getSimNodalVelocities is not implemented yet");
    return false;
};

bool CpuSoftBodyView::setSimNodalPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CARB_LOG_ERROR("CpuSimulationView::setSimNodalPositions is not implemented yet");
    return false;
};
bool CpuSoftBodyView::setSimNodalVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CARB_LOG_ERROR("CpuSimulationView::setSimNodalVelocities is not implemented yet");
    return false;
};
bool CpuSoftBodyView::setSimKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CARB_LOG_ERROR("CpuSimulationView::setSimKinematicTargets is not implemented yet");
    return false;
};


} // namespace tensors
} // namespace physx
} // namespace omni
