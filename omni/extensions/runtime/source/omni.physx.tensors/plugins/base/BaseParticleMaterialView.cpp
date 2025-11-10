// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "BaseParticleMaterialView.h"
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

BaseParticleMaterialView::BaseParticleMaterialView(BaseSimulationView* sim, const std::vector<ParticleMaterialEntry>& entries)
    : mSim(sim), mEntries(entries)
{
    if (mSim)
    {
        // acquire a shared data pointer so the stuff we need doesn't get deleted
        mSimData = mSim->getBaseSimulationData();

        // initialize default indices
        uint32_t numParticleMaterials = uint32_t(mEntries.size());
        mAllIndices.resize(numParticleMaterials);
        for (PxU32 i = 0; i < numParticleMaterials; i++)
        {
            mAllIndices[i] = i;
        }
    }
}

BaseParticleMaterialView::~BaseParticleMaterialView()
{
    if (mSim)
    {
        mSim->_onChildRelease(this);
    }
}

uint32_t BaseParticleMaterialView::getCount() const
{
    return uint32_t(mEntries.size());
}

bool BaseParticleMaterialView::check() const
{
    bool result = true;

    if (!g_physx)
    {
        return false;
    }

    for (auto& entry : mEntries)
    {
        void* ptr = g_physx->getPhysXPtr(entry.path, omni::physx::PhysXType::ePTPBDMaterial);
        if (ptr != entry.pbdMaterial)
        {
            result = false;
        }
    }

    return result;
}

void BaseParticleMaterialView::release()
{
    delete this;
}

void BaseParticleMaterialView::_onParentRelease()
{
    mSim = nullptr;
}

bool BaseParticleMaterialView::getFriction(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "friction", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "friction", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount(), "friction", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        dst[i] = mEntries[i].pbdMaterial->getFriction();
    }

    return true;
}

bool BaseParticleMaterialView::setFriction(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "friction", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "friction", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount(), "friction", __FUNCTION__))
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
            mEntries[idx].pbdMaterial->setFriction(src[idx]);
        }
    }

    return true;
}

bool BaseParticleMaterialView::getDamping(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "damping", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "damping", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount(), "damping", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        dst[i] = mEntries[i].pbdMaterial->getDamping();
    }

    return true;
}

bool BaseParticleMaterialView::setDamping(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "damping", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "damping", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount(), "damping", __FUNCTION__))
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
            mEntries[idx].pbdMaterial->setDamping(src[idx]);
        }
    }

    return true;
}

bool BaseParticleMaterialView::getGravityScale(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "gravity_scale", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "gravity_scale", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount(), "gravity_scale", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        dst[i] = mEntries[i].pbdMaterial->getGravityScale();
    }

    return true;
}

bool BaseParticleMaterialView::setGravityScale(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "gravity_scale", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "gravity_scale", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount(), "gravity_scale", __FUNCTION__))
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
            mEntries[idx].pbdMaterial->setGravityScale(src[idx]);
        }
    }

    return true;
}

bool BaseParticleMaterialView::getLift(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "lift", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "lift", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount(), "lift", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        dst[i] = mEntries[i].pbdMaterial->getLift();
    }

    return true;
}

bool BaseParticleMaterialView::setLift(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "lift", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "lift", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount(), "lift", __FUNCTION__))
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
            mEntries[idx].pbdMaterial->setLift(src[idx]);
        }
    }

    return true;
}

bool BaseParticleMaterialView::getDrag(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "drag", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "drag", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount(), "drag", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        dst[i] = mEntries[i].pbdMaterial->getDrag();
    }

    return true;
}

bool BaseParticleMaterialView::setDrag(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "drag", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "drag", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount(), "drag", __FUNCTION__))
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
            mEntries[idx].pbdMaterial->setDrag(src[idx]);
        }
    }

    return true;
}

}
}
}
