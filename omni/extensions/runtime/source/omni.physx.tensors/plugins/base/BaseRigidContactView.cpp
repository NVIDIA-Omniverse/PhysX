// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "BaseRigidContactView.h"
#include "BaseSimulationView.h"

#include "../GlobalsAreBad.h"
#include "../CommonTypes.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

BaseRigidContactView::BaseRigidContactView(BaseSimulationView* sim,
                                           const std::vector<RigidContactSensorEntry>& entries,
                                           uint32_t numFilters,
                                           uint32_t maxContactDataCount)
    : mSim(sim),
      mEntries(entries),
      mNumFilters(numFilters),
      mMaxContactDataCount(maxContactDataCount)
{
    if (mSim)
    {
        // acquire a shared data pointer so the stuff we need doesn't get deleted
        mSimData = mSim->getBaseSimulationData();
    }
}

BaseRigidContactView::~BaseRigidContactView()
{
    if (mSim)
    {
        mSim->_onChildRelease(this);
    }
}

uint32_t BaseRigidContactView::getSensorCount() const
{
    return uint32_t(mEntries.size());
}

uint32_t BaseRigidContactView::getFilterCount() const
{
    return mNumFilters;
}

uint32_t BaseRigidContactView::getMaxContactDataCount() const
{
    return mMaxContactDataCount;
}

bool BaseRigidContactView::check() const
{
    bool result = true;

    // printf("~!~!~ Checking rigid contact view\n");

    // TODO!!!

    return result;
}


const char* BaseRigidContactView::getUsdPrimPath(uint32_t sensorIdx) const
{
    if (sensorIdx < mEntries.size())
    {
        return mEntries[sensorIdx].path.GetString().c_str();
    }
    return nullptr;
}

const char* BaseRigidContactView::getUsdPrimName(uint32_t sensorIdx) const
{
    if (sensorIdx < mEntries.size())
    {
        return mSimData->mUniqueRCIdx2Names[mEntries[sensorIdx].nameID].c_str();
    }
    return nullptr;
}

const char* BaseRigidContactView::getFilterUsdPrimPath(uint32_t sensorIdx, uint32_t filterIdx) const
{
    if (sensorIdx < mEntries.size())
    {
        if (filterIdx < mEntries[sensorIdx].filterPaths.size())
        {
            return mEntries[sensorIdx].filterPaths[filterIdx].GetString().c_str();
        }
    }
    return nullptr;
}

const char* BaseRigidContactView::getFilterUsdPrimName(uint32_t sensorIdx, uint32_t filterIdx) const
{
    if (sensorIdx < mEntries.size())
    {
        if (filterIdx < mEntries[sensorIdx].filterPaths.size())
        {
            return mEntries[sensorIdx].filterPaths[filterIdx].GetName().c_str();
        }
    }
    return nullptr;
}

void BaseRigidContactView::release()
{
    delete this;
}

void BaseRigidContactView::_onParentRelease()
{
    // printf("~!~!~! Detaching RC view from parent %p\n", mSim);
    mSim = nullptr;
}

}
}
}
