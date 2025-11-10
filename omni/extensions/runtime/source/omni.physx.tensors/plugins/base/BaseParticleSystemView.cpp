// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "BaseParticleSystemView.h"
#include "BaseSimulationView.h"

#include "../GlobalsAreBad.h"
#include "../CommonTypes.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

#include <omni/physics/tensors/TensorUtils.h>

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

BaseParticleSystemView::BaseParticleSystemView(BaseSimulationView* sim, const std::vector<ParticleSystemEntry>& entries)
    : mSim(sim), mEntries(entries)
{
    if (mSim)
    {
        // acquire a shared data pointer so the stuff we need doesn't get deleted
        mSimData = mSim->getBaseSimulationData();

        // initialize default indices
        uint32_t numParticleSystems = uint32_t(mEntries.size());
        mAllIndices.resize(numParticleSystems);
        for (PxU32 i = 0; i < numParticleSystems; i++)
        {
            mAllIndices[i] = i;
        }
    }
}

BaseParticleSystemView::~BaseParticleSystemView()
{
    if (mSim)
    {
        mSim->_onChildRelease(this);
    }
}

uint32_t BaseParticleSystemView::getCount() const
{
    return uint32_t(mEntries.size());
}

bool BaseParticleSystemView::check() const
{
    bool result = true;

    if (!g_physx)
    {
        return false;
    }

    for (auto& entry : mEntries)
    {
        void* ptr = g_physx->getPhysXPtr(entry.path, omni::physx::PhysXType::ePTParticleSystem);
        if (ptr != entry.particleSystem)
        {
            result = false;
        }
    }

    return result;
}

void BaseParticleSystemView::release()
{
    delete this;
}

void BaseParticleSystemView::_onParentRelease()
{
    mSim = nullptr;
}

bool BaseParticleSystemView::getSolidRestOffset(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "solid_rest_offsets", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "solid_rest_offsets", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount(), "solid_rest_offsets", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        dst[i] = mEntries[i].particleSystem->getSolidRestOffset();
    }

    return true;
}


bool BaseParticleSystemView::setSolidRestOffset(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "solid_rest_offsets", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "solid_rest_offsets", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount(), "solid_rest_offsets", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    const float* src = static_cast<const float*>(srcTensor->data);
    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            mEntries[idx].particleSystem->setSolidRestOffset(src[idx]);
        }
    }

    return true;
}

bool BaseParticleSystemView::getFluidRestOffset(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "fluid_rest_offsets", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "fluid_rest_offsets", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount(), "fluid_rest_offsets", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        dst[i] = mEntries[i].particleSystem->getFluidRestOffset();
    }

    return true;
}

bool BaseParticleSystemView::setFluidRestOffset(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "fluid_rest_offsets", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "fluid_rest_offsets", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount(), "fluid_rest_offsets", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    const float* src = static_cast<const float*>(srcTensor->data);
    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            mEntries[idx].particleSystem->setFluidRestOffset(src[idx]);
        }
    }

    return true;
}

bool BaseParticleSystemView::getParticleContactOffset(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "particle_contact_offsets", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "particle_contact_offsets", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount(), "particle_contact_offsets", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        dst[i] = mEntries[i].particleSystem->getParticleContactOffset();
    }

    return true;
}

bool BaseParticleSystemView::setParticleContactOffset(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "particle_contact_offsets", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "particle_contact_offsets", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount(), "particle_contact_offsets", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    const float* src = static_cast<const float*>(srcTensor->data);
    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            mEntries[idx].particleSystem->setParticleContactOffset(src[idx]);
        }
    }

    return true;
}

bool BaseParticleSystemView::getWind(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "wind", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "wind", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 3u, "wind", __FUNCTION__))
    {
        return false;
    }

    PxVec3* dst = static_cast<PxVec3*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        dst[i] = mEntries[i].particleSystem->getWind();
    }

    return true;
}

bool BaseParticleSystemView::setWind(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "wind", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "wind", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * 3u, "wind", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    const PxVec3* src = static_cast<const PxVec3*>(srcTensor->data);
    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            mEntries[idx].particleSystem->setWind(src[idx]);
        }
    }

    return true;
}

}
}
}
