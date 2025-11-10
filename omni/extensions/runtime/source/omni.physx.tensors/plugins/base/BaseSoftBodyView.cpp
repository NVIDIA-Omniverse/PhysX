// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "BaseSoftBodyView.h"
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

BaseSoftBodyView::BaseSoftBodyView(BaseSimulationView* sim, const std::vector<SoftBodyEntry>& entries)
    : mSim(sim), mMaxElementsPerBody(0), mMaxVerticesPerBody(0), mEntries(entries)
{
    if (mSim)
    {
        // acquire a shared data pointer so the stuff we need doesn't get deleted
        mSimData = mSim->getBaseSimulationData();
    }
}

BaseSoftBodyView::~BaseSoftBodyView()
{
    if (mSim)
    {
        mSim->_onChildRelease(this);
    }
}

uint32_t BaseSoftBodyView::getCount() const
{
    return uint32_t(mEntries.size());
}

uint32_t BaseSoftBodyView::getMaxElementsPerBody() const
{
    return mMaxElementsPerBody;
}

uint32_t BaseSoftBodyView::getMaxSimElementsPerBody() const
{
    return mMaxSimElementsPerBody;
}

uint32_t BaseSoftBodyView::getMaxVerticesPerBody() const
{
    return mMaxVerticesPerBody;
}

uint32_t BaseSoftBodyView::getMaxSimVerticesPerBody() const
{
    return mMaxSimVerticesPerBody;
}

bool BaseSoftBodyView::getTransforms(const TensorDesc* dstTensor) const
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

bool BaseSoftBodyView::check() const
{
    bool result = true;

    // printf("~!~!~ Checking soft body view\n");

    if (!g_physx)
    {
        return false;
    }

    for (auto& entry : mEntries)
    {
        void* ptr = g_physx->getPhysXPtr(entry.path, omni::physx::PhysXType::ePTSoftBodyDeprecated);
        if (ptr != entry.body)
        {
            result = false;
        }
    }

    return result;
}

void BaseSoftBodyView::release()
{
    delete this;
}

void BaseSoftBodyView::_onParentRelease()
{
    // printf("~!~!~! Detaching RB view from parent %p\n", mSim);
    mSim = nullptr;
}

}
}
}
