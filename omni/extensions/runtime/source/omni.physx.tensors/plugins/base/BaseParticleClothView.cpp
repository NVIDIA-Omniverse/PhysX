// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "BaseParticleClothView.h"
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

BaseParticleClothView::BaseParticleClothView(BaseSimulationView* sim, const std::vector<ParticleClothEntry>& entries)
    : mSim(sim), mEntries(entries), mMaxParticlesPerCloth(0), mMaxSpringsPerCloth(0)
{
    if (mSim)
    {
        // acquire a shared data pointer so the stuff we need doesn't get deleted
        mSimData = mSim->getBaseSimulationData();
    }
}

BaseParticleClothView::~BaseParticleClothView()
{
    if (mSim)
    {
        mSim->_onChildRelease(this);
    }
}

uint32_t BaseParticleClothView::getCount() const
{
    return uint32_t(mEntries.size());
}

uint32_t BaseParticleClothView::getMaxParticlesPerCloth() const
{
    return mMaxParticlesPerCloth;
}

uint32_t BaseParticleClothView::getMaxSpringsPerCloth() const
{
    return mMaxSpringsPerCloth;
}

bool BaseParticleClothView::check() const
{
    bool result = true;

    if (!g_physx)
    {
        return false;
    }

    for (auto& entry : mEntries)
    {
        void* ptr = g_physx->getPhysXPtr(entry.path, omni::physx::PhysXType::ePTParticleClothDeprecated);
        if (ptr != entry.cloth)
        {
            result = false;
        }
    }

    return result;
}

void BaseParticleClothView::release()
{
    delete this;
}

void BaseParticleClothView::_onParentRelease()
{
    mSim = nullptr;
}

}
}
}
