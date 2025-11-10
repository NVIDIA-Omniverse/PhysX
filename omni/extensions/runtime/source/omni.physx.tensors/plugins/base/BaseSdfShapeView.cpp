// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "BaseSdfShapeView.h"
#include "BaseSimulationView.h"

#include "../GlobalsAreBad.h"
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

        // initialize default indices
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

    for (auto& entry : mEntries)
    {
        void* ptr = g_physx->getPhysXPtr(entry.path, omni::physx::PhysXType::ePTShape);
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
        return mEntries[sdfIdx].path.GetString().c_str();
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
