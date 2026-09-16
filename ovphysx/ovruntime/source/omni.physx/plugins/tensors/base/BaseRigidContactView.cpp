// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-TENSOR-CONTACT-001
 * @covers AC-5 AC-6 AC-7
 *
 * @implements REQ-TENSOR-PATH-001
 * @covers AC-7
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-40
 */

// clang-format off
// clang-format on

#include "tensors/base/BaseRigidContactView.h"
#include "tensors/base/BaseSimulationView.h"

#include "tensors/GlobalsAreBad.h"
#include "tensors/CommonTypes.h"

#include "usdLoad/AttachedStage.h"

#include <omni/physics/parse/IPhysicsSource.h>

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>
#include <omni/physics/tensors/TensorUtils.h>

#include <utility>

using namespace physx;
using omni::physics::tensors::checkTensorDevice;
using omni::physics::tensors::checkTensorInt64;
using omni::physics::tensors::getTensorTotalSize;

namespace omni
{
namespace physx
{
namespace tensors
{

BaseRigidContactView::BaseRigidContactView(BaseSimulationView* sim,
                                           std::vector<RigidContactSensorEntry>&& entries,
                                           uint32_t numFilters,
                                           uint32_t maxContactDataCount)
    : mSim(sim),
      mEntries(std::move(entries)),
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

    return result;
}


const char* BaseRigidContactView::getUsdPrimPath(uint32_t sensorIdx) const
{
    if (sensorIdx < mEntries.size())
    {
        return mEntries[sensorIdx].path.c_str();
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
            return mEntries[sensorIdx].filterPaths[filterIdx].c_str();
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
            // Keyed by (sensorIdx, filterIdx) rather than a single shared scratch
            // string: mEntries is fixed for the view's lifetime (set once in the
            // constructor), so this key stably identifies the entry, and a
            // previously-returned pointer stays valid across later calls on the
            // same view (see mFilterNameCache's declaration for why).
            const uint64_t key = (uint64_t(sensorIdx) << 32) | filterIdx;
            auto it = mFilterNameCache.find(key);
            if (it == mFilterNameCache.end())
            {
                const std::string& path = mEntries[sensorIdx].filterPaths[filterIdx];
                const size_t slash = path.find_last_of('/');
                it = mFilterNameCache.emplace(key, (slash == std::string::npos) ? path : path.substr(slash + 1)).first;
            }
            return it->second.c_str();
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
    mSim = nullptr;
}

void BaseRigidContactView::getOtherActorPathsFromIds(const TensorDesc* otherActorIdsTensor, std::vector<std::string>& outPaths) const
{
    outPaths.clear();

    if (!otherActorIdsTensor || !otherActorIdsTensor->data)
    {
        return;
    }

    // Path conversion is a CPU-only operation - expect CPU data
    if (!checkTensorDevice(*otherActorIdsTensor, -1, "other actor IDs tensor", __FUNCTION__) ||
        !checkTensorInt64(*otherActorIdsTensor, "other actor IDs tensor", __FUNCTION__))
    {
        return;
    }

    size_t count = getTensorTotalSize(*otherActorIdsTensor);
    if (count == 0)
    {
        return;
    }

    const uint64_t* actorIds = static_cast<const uint64_t*>(otherActorIdsTensor->data);
    outPaths.reserve(count);

    usdparser::AttachedStage* attachedStage = mSim ? mSim->getAttachedStage() : nullptr;

    // Liveness gate. Neither of the two things this function reads expires: textFor()
    // reads interned storage that outlives the prim, and mPathCache is never invalidated.
    // Without a gate, an id whose actor has since been removed keeps resolving to its old
    // path -- a stale answer indistinguishable from a live one, which is exactly the
    // silent mis-resolution REQ-TENSOR-CONTACT-001 AC-7 forbids.
    //
    // Liveness is AttachedStage::isKeyLive, shared with objectKeyToPath so the two public
    // ObjectKey->path resolvers agree. It is broader than IPhysicsSource::exists(): a
    // PhysX-replicator clone (the tiled-env RL case) is registered in the ObjectDb with no
    // authored source object, so exists() alone would report every clone as stale. isKeyLive
    // checks the in-memory ObjectDb first, so the common per-contact case is a hash lookup,
    // not a backend round trip -- which also bounds the cost of gating a large buffer. A
    // per-call cache keeps a repeated actor from being probed once per contact point.
    std::unordered_map<uint64_t, bool> liveCache;
    auto keyIsLive = [&](uint64_t id) -> bool
    {
        if (!attachedStage)
            return false;
        auto it = liveCache.find(id);
        if (it != liveCache.end())
            return it->second;
        const bool live = attachedStage->isKeyLive(keyFromLegacyId(id));
        liveCache.emplace(id, live);
        return live;
    };

    for (size_t i = 0; i < count; ++i)
    {
        uint64_t id = actorIds[i];
        if (id == 0)
        {
            outPaths.emplace_back();
            continue;
        }

        if (!keyIsLive(id))
        {
            // Explicitly unresolvable. Reported as empty and deliberately not cached in
            // mPathCache: a non-zero id that comes back empty is the caller's stale signal,
            // and memoizing the path would also outlive whatever made it stale.
            outPaths.emplace_back();
            continue;
        }

        auto it = mPathCache.find(id);
        if (it != mPathCache.end())
        {
            outPaths.push_back(it->second);
        }
        else
        {
            // id is the raw ObjectKey handle (see CommonTypes.h's asInt()/
            // keyFromLegacyId(), both configs); resolve the display string through
            // AttachedStage::textFor() (see GpuRigidContactView.cpp for the same
            // precedent).
            auto result =
                mPathCache.emplace(id, attachedStage ? attachedStage->textFor(keyFromLegacyId(id)) : "");
            outPaths.push_back(result.first->second);
        }
    }
}

}
}
}
