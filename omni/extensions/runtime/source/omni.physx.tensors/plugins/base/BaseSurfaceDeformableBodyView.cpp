// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "BaseSurfaceDeformableBodyView.h"
#include "BaseSimulationView.h"

#include "../GlobalsAreBad.h"

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
        return mEntries[dbIdx].path.GetString().c_str();
    }
    return nullptr;
}

const char* BaseSurfaceDeformableBodyView::getUsdSimulationMeshPrimPath(uint32_t dbIdx) const
{
    if (dbIdx < mEntries.size())
    {
        return mEntries[dbIdx].simMeshPath.GetString().c_str();
    }
    return nullptr;
}

const char* BaseSurfaceDeformableBodyView::getUsdCollisionMeshPrimPath(uint32_t dbIdx) const
{
    if (dbIdx < mEntries.size())
    {
        return mEntries[dbIdx].collMeshPath.GetString().c_str();
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

    // printf("~!~!~ Checking surface deformable body view\n");

    if (!g_physx)
    {
        return false;
    }

    for (auto& entry : mEntries)
    {
        void* ptr = g_physx->getPhysXPtr(entry.path, omni::physx::PhysXType::ePTDeformableVolume);
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
    // printf("~!~!~! Detaching surface deformable body view from parent %p\n", mSim);
    mSim = nullptr;
}

}
}
}
