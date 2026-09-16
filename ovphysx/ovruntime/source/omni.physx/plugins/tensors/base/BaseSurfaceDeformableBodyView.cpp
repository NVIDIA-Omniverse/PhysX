// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-TENSOR-ATTACH-001
 * @covers AC-1
 */

// clang-format off
// clang-format on

#include "tensors/base/BaseSurfaceDeformableBodyView.h"
#include "tensors/base/BaseSimulationView.h"
#include "usdLoad/AttachedStage.h"

#include "tensors/GlobalsAreBad.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

#include <omni/physics/tensors/TensorUtils.h>
using omni::physics::tensors::checkTensorDevice;
using omni::physics::tensors::checkTensorFloat32;
using omni::physics::tensors::checkTensorSizeExact;

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

BaseSurfaceDeformableBodyView::BaseSurfaceDeformableBodyView(BaseSimulationView* sim, const std::vector<DeformableBodyEntry>& entries)
    : mSim(sim)
    , mMaxSimElementsPerBody(0)
    , mMaxSimNodesPerBody(0)
    , mMaxRestNodesPerBody(0)
    , mEntries(entries)
{
    if (mSim)
    {
        // acquire a shared data pointer so the stuff we need doesn't get deleted
        mSimData = mSim->getBaseSimulationData();
        for (const auto& entry : mEntries)
        {
            if (entry.body)
                mSim->deformableBodies.insert(entry.body);
        }
    }
}

BaseSurfaceDeformableBodyView::~BaseSurfaceDeformableBodyView()
{
    if (mSim)
    {
        mSim->_onChildRelease(this);
    }
}

uint32_t BaseSurfaceDeformableBodyView::getCount() const
{
    return uint32_t(mEntries.size());
}

const char* BaseSurfaceDeformableBodyView::getUsdPrimPath(uint32_t dbIdx) const
{
    if (dbIdx < mEntries.size())
    {
        return mEntries[dbIdx].path.c_str();
    }
    return nullptr;
}

const char* BaseSurfaceDeformableBodyView::getUsdSimulationMeshPrimPath(uint32_t dbIdx) const
{
    if (dbIdx < mEntries.size())
    {
        return mEntries[dbIdx].simMeshPath.c_str();
    }
    return nullptr;
}

const char* BaseSurfaceDeformableBodyView::getUsdCollisionMeshPrimPath(uint32_t dbIdx) const
{
    if (dbIdx < mEntries.size())
    {
        return mEntries[dbIdx].collMeshPath.c_str();
    }
    return nullptr;
}

uint32_t BaseSurfaceDeformableBodyView::getMaxCollisionElementsPerBody() const
{
    return mMaxSimElementsPerBody;
}

uint32_t BaseSurfaceDeformableBodyView::getMaxSimulationElementsPerBody() const
{
    return mMaxSimElementsPerBody;
}

uint32_t BaseSurfaceDeformableBodyView::getMaxCollisionNodesPerBody() const
{
    return mMaxSimNodesPerBody;
}

uint32_t BaseSurfaceDeformableBodyView::getMaxSimulationNodesPerBody() const
{
    return mMaxSimNodesPerBody;
}

uint32_t BaseSurfaceDeformableBodyView::getMaxRestNodesPerBody() const
{
    return mMaxRestNodesPerBody;
}

bool BaseSurfaceDeformableBodyView::getTransforms(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mSimData, mSim, false);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "transform", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "transform", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 7u, "transform", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxBounds3 bounds = mEntries[i].body->getWorldBounds();
        PxVec3 center = bounds.getCenter();
        Subspace* subspace = mEntries[i].subspace;
        if (subspace)
        {
            center.x -= subspace->origin.x;
            center.y -= subspace->origin.y;
            center.z -= subspace->origin.z;
        }
        *dst++ = center.x;
        *dst++ = center.y;
        *dst++ = center.z;
        *dst++ = 0.0f;
        *dst++ = 0.0f;
        *dst++ = 0.0f;
        *dst++ = 1.0f;
    }

    return true;
}

bool BaseSurfaceDeformableBodyView::check() const
{
    bool result = true;

    if (!g_physx)
    {
        return false;
    }

    usdparser::AttachedStage* attachedStage = mSim ? mSim->getAttachedStage() : nullptr;
    for (auto& entry : mEntries)
    {
        const omni::physics::parse::ObjectKey key =
            attachedStage ? attachedStage->keyFor(entry.path) : omni::physics::parse::ObjectKey{};
        void* ptr = BaseSimulationView::resolvePhysXPtr(attachedStage, key, omni::physx::PhysXType::ePTDeformableSurface);
        if (ptr != entry.body)
        {
            result = false;
        }
    }

    return result;
}

void BaseSurfaceDeformableBodyView::release()
{
    delete this;
}

void BaseSurfaceDeformableBodyView::_onParentRelease()
{
    mSim = nullptr;
}

}
}
}
