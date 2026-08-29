// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "tensors/base/BaseDeformableMaterialView.h"
#include "tensors/base/BaseSimulationView.h"
#include <PxPhysicsAPI.h>
#include <PxDeformableSurfaceMaterial.h>
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
    delete this;
}

void BaseDeformableMaterialView::_onParentRelease()
{
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

// ---------------------------------------------------------------------------
// Mask support
//
// NOTE: Material views always operate via CPU-side PxMaterial API calls, even when
// the simulation runs on GPU. Both CpuDeformableMaterialView and GpuDeformableMaterialView
// inherit from this base class without overriding. The device check below is hardcoded
// to -1 (CPU), matching the existing indexed setters. This is a pre-existing pattern
// in the tensorAPI - material properties are not part of the DirectGPU pipeline.
//
// Uses shared resolveMaskToIndices() / makeIndexTensorDesc() from TensorUtils.h.
// ---------------------------------------------------------------------------

using omni::physics::tensors::MaskResult;
using omni::physics::tensors::resolveMaskToIndices;
using omni::physics::tensors::makeIndexTensorDesc;

bool BaseDeformableMaterialView::applyMasked(const TensorDesc* srcTensor,
                                             const TensorDesc* maskTensor,
                                             const IndexedSetter indexedSetter,
                                             const char* callerFunctionName)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(maskTensor, getCount(), -1, indices, callerFunctionName);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return (this->*indexedSetter)(srcTensor, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return (this->*indexedSetter)(srcTensor, &idx);
}

bool BaseDeformableMaterialView::setDynamicFrictionMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor)
{
    return applyMasked(srcTensor, maskTensor, &BaseDeformableMaterialView::setDynamicFriction, __FUNCTION__);
}

bool BaseDeformableMaterialView::setYoungsModulusMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor)
{
    return applyMasked(srcTensor, maskTensor, &BaseDeformableMaterialView::setYoungsModulus, __FUNCTION__);
}

bool BaseDeformableMaterialView::setPoissonsRatioMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor)
{
    return applyMasked(srcTensor, maskTensor, &BaseDeformableMaterialView::setPoissonsRatio, __FUNCTION__);
}

// ---------------------------------------------------------------------------
// Elasticity damping -- PxDeformableMaterial base (volume + surface)
// ---------------------------------------------------------------------------

bool BaseDeformableMaterialView::getElasticityDamping(const TensorDesc* dstTensor) const
{
    return getProperty(dstTensor, &PxDeformableMaterial::getElasticityDamping, "ElasticityDamping", __FUNCTION__);
}

bool BaseDeformableMaterialView::setElasticityDamping(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return setProperty(srcTensor, indexTensor, &PxDeformableMaterial::setElasticityDamping, "ElasticityDamping", __FUNCTION__);
}

bool BaseDeformableMaterialView::setElasticityDampingMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor)
{
    return applyMasked(srcTensor, maskTensor, &BaseDeformableMaterialView::setElasticityDamping, __FUNCTION__);
}

// ---------------------------------------------------------------------------
// Bending stiffness, thickness, bending damping -- PxDeformableSurfaceMaterial
// ---------------------------------------------------------------------------

bool BaseDeformableMaterialView::getBendingStiffness(const TensorDesc* dstTensor) const
{
    return getSurfaceProperty(dstTensor, &PxDeformableSurfaceMaterial::getBendingStiffness, "BendingStiffness", __FUNCTION__);
}

bool BaseDeformableMaterialView::setBendingStiffness(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return setSurfaceProperty(srcTensor, indexTensor, &PxDeformableSurfaceMaterial::setBendingStiffness, "BendingStiffness", __FUNCTION__);
}

bool BaseDeformableMaterialView::setBendingStiffnessMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor)
{
    return applyMasked(srcTensor, maskTensor, &BaseDeformableMaterialView::setBendingStiffness, __FUNCTION__);
}

bool BaseDeformableMaterialView::getThickness(const TensorDesc* dstTensor) const
{
    return getSurfaceProperty(dstTensor, &PxDeformableSurfaceMaterial::getThickness, "Thickness", __FUNCTION__);
}

bool BaseDeformableMaterialView::setThickness(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return setSurfaceProperty(srcTensor, indexTensor, &PxDeformableSurfaceMaterial::setThickness, "Thickness", __FUNCTION__);
}

bool BaseDeformableMaterialView::setThicknessMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor)
{
    return applyMasked(srcTensor, maskTensor, &BaseDeformableMaterialView::setThickness, __FUNCTION__);
}

bool BaseDeformableMaterialView::getBendingDamping(const TensorDesc* dstTensor) const
{
    return getSurfaceProperty(dstTensor, &PxDeformableSurfaceMaterial::getBendingDamping, "BendingDamping", __FUNCTION__);
}

bool BaseDeformableMaterialView::setBendingDamping(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return setSurfaceProperty(srcTensor, indexTensor, &PxDeformableSurfaceMaterial::setBendingDamping, "BendingDamping", __FUNCTION__);
}

bool BaseDeformableMaterialView::setBendingDampingMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor)
{
    return applyMasked(srcTensor, maskTensor, &BaseDeformableMaterialView::setBendingDamping, __FUNCTION__);
}

// ---------------------------------------------------------------------------
// Surface material helpers -- downcast to PxDeformableSurfaceMaterial*
// Volume material entries produce 0.0 on get and are skipped on set.
// ---------------------------------------------------------------------------

bool BaseDeformableMaterialView::getSurfaceProperty(const TensorDesc* dstTensor,
                                                     const SurfaceMaterialGetter getter,
                                                     const char* parameterName,
                                                     const char* callerFunctionName) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
        return false;

    if (!checkTensorDevice(*dstTensor, -1, parameterName, callerFunctionName) ||
        !checkTensorFloat32(*dstTensor, parameterName, callerFunctionName) ||
        !checkTensorSizeExact(*dstTensor, getCount(), parameterName, callerFunctionName))
        return false;

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        if (mEntries[i].isSurface)
        {
            PxDeformableSurfaceMaterial* mat = static_cast<PxDeformableSurfaceMaterial*>(mEntries[i].material);
            dst[i] = (mat->*getter)();
        }
        else
        {
            dst[i] = 0.0f;
        }
    }
    return true;
}

bool BaseDeformableMaterialView::setSurfaceProperty(const TensorDesc* srcTensor,
                                                     const TensorDesc* indexTensor,
                                                     const SurfaceMaterialSetter setter,
                                                     const char* parameterName,
                                                     const char* callerFunctionName)
{
    if (!srcTensor || !srcTensor->data)
        return false;

    if (!checkTensorDevice(*srcTensor, -1, parameterName, callerFunctionName) ||
        !checkTensorFloat32(*srcTensor, parameterName, callerFunctionName) ||
        !checkTensorSizeExact(*srcTensor, getCount(), parameterName, callerFunctionName))
        return false;

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", callerFunctionName) ||
            !checkTensorInt32(*indexTensor, "index", callerFunctionName))
            return false;
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
        if (idx < mEntries.size() && mEntries[idx].isSurface)
        {
            PxDeformableSurfaceMaterial* mat = static_cast<PxDeformableSurfaceMaterial*>(mEntries[idx].material);
            (mat->*setter)(src[idx]);
        }
    }
    return true;
}


} // namespace tensors
} // namespace physx
} // namespace omni
