// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-TENSOR-ATTACH-001
 * @covers AC-1
 */

// clang-format off
// clang-format on

#include "tensors/base/BaseSdfShapeView.h"
#include "tensors/base/BaseSimulationView.h"
#include "usdLoad/AttachedStage.h"

#include "tensors/GlobalsAreBad.h"
#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>


using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

BaseSdfShapeView::BaseSdfShapeView(BaseSimulationView* sim, const std::vector<SdfShapeEntry>& entries)
    : mSim(sim), mEntries(entries)
{
    if (mSim)
    {
        // acquire a shared data pointer so the stuff we need doesn't get deleted
        mSimData = mSim->getBaseSimulationData();

        uint32_t numSdfShapes = uint32_t(mEntries.size());
        mAllIndices.resize(numSdfShapes);
        for (PxU32 i = 0; i < numSdfShapes; i++)
        {
            mAllIndices[i] = i;
            if (mEntries[i].numSamplePoints > mMaxNumPoints)
            {
                mMaxNumPoints = mEntries[i].numSamplePoints;
            }
        }
    }
}

BaseSdfShapeView::~BaseSdfShapeView()
{
    if (mSim)
    {
        mSim->_onChildRelease(this);
    }
}

uint32_t BaseSdfShapeView::getCount() const
{
    return uint32_t(mEntries.size());
}

uint32_t BaseSdfShapeView::getMaxNumPoints() const
{
    return mMaxNumPoints;
}

bool BaseSdfShapeView::check() const
{
    if (!g_physx)
    {
        return false;
    }

    usdparser::AttachedStage* attachedStage = mSim ? mSim->getAttachedStage() : nullptr;
    for (auto& entry : mEntries)
    {
        const omni::physics::parse::ObjectKey key =
            attachedStage ? attachedStage->keyFor(entry.path) : omni::physics::parse::ObjectKey{};
        void* ptr = BaseSimulationView::resolvePhysXPtr(attachedStage, key, omni::physx::PhysXType::ePTShape);
        if (ptr != entry.shape)
        {
           return false;
        }
    }

    return true;
}

const char* BaseSdfShapeView::getUsdPrimPath(uint32_t sdfIdx) const
{
    if (sdfIdx < mEntries.size())
    {
        return mEntries[sdfIdx].path.c_str();
    }
    return nullptr;
}

void BaseSdfShapeView::release()
{
    delete this;
}

void BaseSdfShapeView::_onParentRelease()
{
    mSim = nullptr;
}


} // namespace tensors
}
}
