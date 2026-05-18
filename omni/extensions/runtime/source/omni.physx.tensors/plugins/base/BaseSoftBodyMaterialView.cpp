// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "BaseSoftBodyMaterialView.h"
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

BaseSoftBodyMaterialView::BaseSoftBodyMaterialView(BaseSimulationView* sim,
                                                   const std::vector<SoftBodyMaterialEntry>& entries)
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

BaseSoftBodyMaterialView::~BaseSoftBodyMaterialView()
{
    if (mSim)
    {
        mSim->_onChildRelease(this);
    }
}

uint32_t BaseSoftBodyMaterialView::getCount() const
{
    return uint32_t(mEntries.size());
}

bool BaseSoftBodyMaterialView::check() const
{
    bool result = true;

    if (!g_physx)
    {
        return false;
    }

    for (auto& entry : mEntries)
    {
        void* ptr = g_physx->getPhysXPtr(entry.path, omni::physx::PhysXType::ePTSoftBodyMaterialDeprecated);
        if (ptr != entry.femSoftBodyMaterial)
        {
            result = false;
        }
    }

    return result;
}

void BaseSoftBodyMaterialView::release()
{
    delete this;
}

void BaseSoftBodyMaterialView::_onParentRelease()
{
    // printf("~!~!~! Detaching RB view from parent %p\n", mSim);
    mSim = nullptr;
}


bool BaseSoftBodyMaterialView::getYoungsModulus(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "YoungsModulus", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "YoungsModulus", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount(), "YoungsModulus", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        dst[i] = mEntries[i].femSoftBodyMaterial->getYoungsModulus();
    }

    return true;
}

bool BaseSoftBodyMaterialView::setYoungsModulus(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "YoungsModulus", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "YoungsModulus", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount(), "YoungsModulus", __FUNCTION__))
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
            mEntries[idx].femSoftBodyMaterial->setYoungsModulus(src[idx]);
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// Mask support
//
// NOTE: Material views always operate via CPU-side PxMaterial API calls, even when
// the simulation runs on GPU (see BaseDeformableMaterialView.cpp for full rationale).
// The device check is hardcoded to -1 (CPU), matching the existing indexed setters.
// ---------------------------------------------------------------------------

using omni::physics::tensors::MaskResult;
using omni::physics::tensors::resolveMaskToIndices;
using omni::physics::tensors::makeIndexTensorDesc;

bool BaseSoftBodyMaterialView::setYoungsModulusMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(maskTensor, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setYoungsModulus(srcTensor, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setYoungsModulus(srcTensor, &idx);
}

bool BaseSoftBodyMaterialView::getPoissonsRatio(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "YoungsModulus", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "YoungsModulus", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount(), "YoungsModulus", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        dst[i] = mEntries[i].femSoftBodyMaterial->getPoissons();
    }

    return true;
}

bool BaseSoftBodyMaterialView::setPoissonsRatio(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "Poissons ratio", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "Poissons ratio", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount(), "Poissons ratio", __FUNCTION__))
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
            mEntries[idx].femSoftBodyMaterial->setPoissons(src[idx]);
        }
    }

    return true;
}

bool BaseSoftBodyMaterialView::setPoissonsRatioMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(maskTensor, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setPoissonsRatio(srcTensor, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setPoissonsRatio(srcTensor, &idx);
}

bool BaseSoftBodyMaterialView::getDynamicFriction(const TensorDesc* dstTensor) const
{

    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "dynamic friction", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "dynamic friction", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount(), "dynamic friction", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        dst[i] = mEntries[i].femSoftBodyMaterial->getDynamicFriction();
    }

    return true;
}

bool BaseSoftBodyMaterialView::setDynamicFriction(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "dynamic friction", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "dynamic friction", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount(), "dynamic friction", __FUNCTION__))
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
            mEntries[idx].femSoftBodyMaterial->setDynamicFriction(src[idx]);
        }
    }

    return true;
}

bool BaseSoftBodyMaterialView::setDynamicFrictionMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(maskTensor, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setDynamicFriction(srcTensor, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setDynamicFriction(srcTensor, &idx);
}

bool BaseSoftBodyMaterialView::getDamping(const TensorDesc* dstTensor) const
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
        dst[i] = mEntries[i].femSoftBodyMaterial->getDamping();
    }

    return true;
}

bool BaseSoftBodyMaterialView::setDamping(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
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
            mEntries[idx].femSoftBodyMaterial->setDamping(src[idx]);
        }
    }

    return true;
}

bool BaseSoftBodyMaterialView::setDampingMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(maskTensor, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setDamping(srcTensor, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setDamping(srcTensor, &idx);
}


bool BaseSoftBodyMaterialView::getDampingScale(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "damping scale", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "damping scale", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount(), "damping scale", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        dst[i] = mEntries[i].femSoftBodyMaterial->getDampingScale();
    }

    return true;
}

bool BaseSoftBodyMaterialView::setDampingScale(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "damping scale", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "damping scale", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount(), "damping scale", __FUNCTION__))
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
            mEntries[idx].femSoftBodyMaterial->setDampingScale(src[idx]);
        }
    }

    return true;
}

bool BaseSoftBodyMaterialView::setDampingScaleMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(maskTensor, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setDampingScale(srcTensor, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setDampingScale(srcTensor, &idx);
}


} // namespace tensors
} // namespace physx
} // namespace omni
