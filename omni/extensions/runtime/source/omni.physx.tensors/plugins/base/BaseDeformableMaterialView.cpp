// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "BaseDeformableMaterialView.h"
#include "BaseSimulationView.h"
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

BaseDeformableMaterialView::BaseDeformableMaterialView(BaseSimulationView* sim, const std::vector<DeformableMaterialEntry>& entries)
    : mSim(sim), mEntries(entries)
{
    if (mSim)
    {
        // acquire a shared data pointer so the stuff we need doesn't get deleted
        mSimData = mSim->getBaseSimulationData();

        // initialize default indices
        uint32_t numMaterials = uint32_t(mEntries.size());
        mAllIndices.resize(numMaterials);
        for (PxU32 i = 0; i < numMaterials; i++)
        {
            mAllIndices[i] = i;
        }
    }
}

BaseDeformableMaterialView::~BaseDeformableMaterialView()
{
    if (mSim)
    {
        mSim->_onChildRelease(this);
    }
}

uint32_t BaseDeformableMaterialView::getCount() const
{
    return uint32_t(mEntries.size());
}

bool BaseDeformableMaterialView::check() const
{
    bool result = true;

    if (!g_physx)
    {
        return false;
    }

    for (auto& entry : mEntries)
    {
        void* ptr = g_physx->getPhysXPtr(entry.path, omni::physx::PhysXType::ePTDeformableVolumeMaterial);
        if (ptr != entry.material)
        {
            result = false;
        }
    }

    return result;
}

void BaseDeformableMaterialView::release()
{
    mSim = nullptr;
    delete this;
}

void BaseDeformableMaterialView::_onParentRelease()
{
    // printf("~!~!~! Detaching volume deformable body material view from parent %p\n", mSim);
    mSim = nullptr;
}

bool BaseDeformableMaterialView::getProperty(const TensorDesc* dstTensor,
                                             const MaterialGetter materialGetter,
                                             const char* parameterName,
                                             const char* callerFunctionName) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, parameterName, callerFunctionName) ||
        !checkTensorFloat32(*dstTensor, parameterName, callerFunctionName) ||
        !checkTensorSizeExact(*dstTensor, getCount(), parameterName, callerFunctionName))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxDeformableMaterial* mat = mEntries[i].material;
        dst[i] = (mat->*materialGetter)();
    }

    return true;
}

bool BaseDeformableMaterialView::setProperty(const TensorDesc* srcTensor,
                                             const TensorDesc* indexTensor,
                                             const MaterialSetter materialSetter,
                                             const char* parameterName,
                                             const char* callerFunctionName)
{
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, parameterName, callerFunctionName) ||
        !checkTensorFloat32(*srcTensor, parameterName, callerFunctionName) ||
        !checkTensorSizeExact(*srcTensor, getCount(), parameterName, callerFunctionName))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", callerFunctionName) ||
            !checkTensorInt32(*indexTensor, "index", callerFunctionName))
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
            PxDeformableMaterial* mat = mEntries[idx].material;
            (mat->*materialSetter)(src[idx]);
        }
    }

    return true;
}

bool BaseDeformableMaterialView::getDynamicFriction(const TensorDesc* dstTensor) const
{
    return getProperty(dstTensor, &PxDeformableMaterial::getDynamicFriction, "DynamicFriction", __FUNCTION__);
}

bool BaseDeformableMaterialView::setDynamicFriction(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return setProperty(srcTensor, indexTensor, &PxDeformableMaterial::setDynamicFriction, "DynamicFriction", __FUNCTION__);
}

bool BaseDeformableMaterialView::getYoungsModulus(const TensorDesc* dstTensor) const
{
    return getProperty(dstTensor, &PxDeformableMaterial::getYoungsModulus, "YoungsModulus", __FUNCTION__);
}

bool BaseDeformableMaterialView::setYoungsModulus(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return setProperty(srcTensor, indexTensor, &PxDeformableMaterial::setYoungsModulus, "YoungsModulus", __FUNCTION__);
}

bool BaseDeformableMaterialView::getPoissonsRatio(const TensorDesc* dstTensor) const
{
    return getProperty(dstTensor, &PxDeformableMaterial::getPoissons, "PoissonsRatio", __FUNCTION__);
}

bool BaseDeformableMaterialView::setPoissonsRatio(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return setProperty(srcTensor, indexTensor, &PxDeformableMaterial::setPoissons, "PoissonRatio", __FUNCTION__);
}


} // namespace tensors
} // namespace physx
} // namespace omni
