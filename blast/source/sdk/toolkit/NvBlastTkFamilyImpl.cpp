// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.


#include "NvBlastTkFrameworkImpl.h"
#include "NvBlastTkFamilyImpl.h"
#include "NvBlastTkGroupImpl.h"
#include "NvBlastTkAssetImpl.h"
#include "NvBlastTkActorImpl.h"
#include "NvBlastTkJointImpl.h"

#include "NvBlastIndexFns.h"
#include "NvBlastMemory.h"


namespace Nv
{
namespace Blast
{

//////// Static data ////////

NVBLASTTK_DEFINE_TYPE_IDENTIFIABLE(Family);


//////// Member functions ////////

TkFamilyImpl::TkFamilyImpl() : m_familyLL(nullptr), m_internalJointCount(0), m_asset(nullptr)
{
}


TkFamilyImpl::TkFamilyImpl(const NvBlastID& id) : TkFamilyType(id), m_familyLL(nullptr), m_internalJointCount(0), m_asset(nullptr)
{
}


TkFamilyImpl::~TkFamilyImpl()
{
    if (m_familyLL != nullptr)
    {
        uint32_t familyActorCount = NvBlastFamilyGetActorCount(m_familyLL, logLL);
        if (familyActorCount != 0)
        {
            NVBLAST_LOG_WARNING("TkFamilyImpl::~TkFamilyImpl(): family actor count is not 0.");
        }
        NVBLAST_FREE(m_familyLL);
    }
}


void TkFamilyImpl::release()
{
    for (TkActorImpl& actor : m_actors)
    {
        if (actor.isActive())
        {
            actor.release();
        }
    }

    m_actors.clear();

    NVBLAST_DELETE(this, TkFamilyImpl);
}


const NvBlastFamily* TkFamilyImpl::getFamilyLL() const
{
    return m_familyLL;
}


TkActorImpl* TkFamilyImpl::addActor(NvBlastActor* actorLL)
{
    TkActorImpl* actor = getActorByActorLL(actorLL);
    NVBLAST_ASSERT(actor);
    actor->m_actorLL = actorLL;
    actor->m_family = this;
    return actor;
}


void TkFamilyImpl::removeActor(TkActorImpl* actor)
{
    NVBLAST_ASSERT(actor != nullptr && actor->m_family == this);
    //actor->m_family = nullptr;
    actor->m_actorLL = nullptr;
}


uint32_t TkFamilyImpl::getActorCount() const
{
    return getActorCountInternal();
}


uint32_t TkFamilyImpl::getActors(TkActor** buffer, uint32_t bufferSize, uint32_t indexStart /*= 0*/) const
{
    uint32_t actorCount = getActorCount();
    if (actorCount <= indexStart)
    {
        NVBLAST_LOG_WARNING("TkFamilyImpl::getActors: indexStart beyond end of actor list.");
        return 0;
    }

    actorCount -= indexStart;
    if (actorCount > bufferSize)
    {
        actorCount = static_cast<uint32_t>(bufferSize);
    }

    uint32_t index = 0;
    for (const TkActorImpl& actor : m_actors)
    {
        if (actor.isActive())
        {
            if (index >= indexStart)
            {
                if ((index - indexStart) >= actorCount)
                {
                    break;
                }
                else
                {
                    *buffer++ = const_cast<TkActorImpl*>(&actor);
                }
            }
            index++;
        }
    }

    return actorCount;
}


NV_INLINE bool areLLActorsEqual(const NvBlastActor* actor0, const NvBlastActor* actor1, Array<uint32_t>::type& scratch)
{
    if (NvBlastActorGetGraphNodeCount(actor0, logLL) != NvBlastActorGetGraphNodeCount(actor1, logLL))
    {
        return false;
    }

    const uint32_t chunkCount = NvBlastActorGetVisibleChunkCount(actor0, logLL);
    if (chunkCount != NvBlastActorGetVisibleChunkCount(actor1, logLL))
    {
        return false;
    }

    scratch.resize(chunkCount * 2);
    NvBlastActorGetVisibleChunkIndices(scratch.begin(), chunkCount, actor0, logLL);
    NvBlastActorGetVisibleChunkIndices(scratch.begin() + chunkCount, chunkCount, actor1, logLL);
    return memcmp(scratch.begin(), scratch.begin() + chunkCount, chunkCount * sizeof(uint32_t)) == 0;
}


void TkFamilyImpl::reinitialize(const NvBlastFamily* newFamily, TkGroup* group)
{
    NVBLAST_ASSERT(newFamily);
#if NV_ENABLE_ASSERTS
    NvBlastID id0 = NvBlastFamilyGetAssetID(m_familyLL, logLL);
    NvBlastID id1 = NvBlastFamilyGetAssetID(newFamily, logLL);
    NVBLAST_ASSERT(TkGUIDsEqual(&id0, &id1));
#endif
    NVBLAST_ASSERT(NvBlastFamilyGetSize(m_familyLL, logLL) == NvBlastFamilyGetSize(newFamily, logLL));

    // alloc and init new family
    const uint32_t blockSize = NvBlastFamilyGetSize(newFamily, logLL);
    NvBlastFamily* newFamilyCopy = (NvBlastFamily*)NVBLAST_ALLOC_NAMED(blockSize, "TkFamilyImpl::reinitialize");
    memcpy(newFamilyCopy, newFamily, blockSize);
    NvBlastFamilySetAsset(newFamilyCopy, m_asset->getAssetLL(), logLL);

    // get actors from new family
    Array<NvBlastActor*>::type newLLActors(NvBlastFamilyGetActorCount(newFamilyCopy, logLL));
    uint32_t actorCount = NvBlastFamilyGetActors(newLLActors.begin(), newLLActors.size(), newFamilyCopy, logLL);

    // reset actor families to nullptr (we use it as a flag later)
    for (TkActorImpl& actor : m_actors)
    {
        if (actor.isActive())
        {
            actor.m_family = nullptr;
        }
    }

    // prepare split event with new actors
    auto newActorsSplitEvent = getQueue().allocData<TkSplitEvent>();
    Array<TkActor*>::type children(actorCount);
    children.resizeUninitialized(0);
    newActorsSplitEvent->children = children.begin();

    // scratch
    Array<uint32_t>::type scratch(m_asset->getChunkCount());

    for (uint32_t i = 0; i < actorCount; ++i)
    {
        NvBlastActor* newLLActor = newLLActors[i];
        uint32_t actorIndex = NvBlastActorGetIndex(newLLActor, logLL);
        TkActorImpl& tkActor = *getActorByIndex(actorIndex);

        tkActor.m_family = this;

        if (!tkActor.isActive() || !areLLActorsEqual(newLLActor, tkActor.m_actorLL, scratch))
        {
            if (tkActor.isActive())
            {
                auto removeSplitEvent = getQueue().allocData<TkSplitEvent>();
                removeSplitEvent->parentData.family = this;
                removeSplitEvent->numChildren = 0;
                removeSplitEvent->parentData.userData = tkActor.userData;
                removeSplitEvent->parentData.index = tkActor.getIndex();
                getQueue().addEvent(removeSplitEvent);
            }

            tkActor.m_actorLL = newLLActor;

            // switch groups
            TkGroupImpl* prevGroup = tkActor.m_group;
            if (prevGroup != group)
            {
                if (prevGroup)
                {
                    prevGroup->removeActor(tkActor);
                }
                if (group)
                {
                    group->addActor(tkActor);
                }
            }

            children.pushBack(&tkActor);
        }
        else
        {
            tkActor.m_actorLL = newLLActor;
        }
    }

    // if m_family is still nullptr for an active actor -> remove it. It doesn't exist in new family.
    for (TkActorImpl& tkActor : m_actors)
    {
        if (tkActor.isActive() && tkActor.m_family == nullptr)
        {
            tkActor.m_family = this;
            if (tkActor.m_group)
            {
                tkActor.m_group->removeActor(tkActor);
            }

            auto removeSplitEvent = getQueue().allocData<TkSplitEvent>();
            removeSplitEvent->parentData.family = this;
            removeSplitEvent->numChildren = 0;
            removeSplitEvent->parentData.userData = tkActor.userData;
            removeSplitEvent->parentData.index = tkActor.getIndex();
            getQueue().addEvent(removeSplitEvent);

            tkActor.m_actorLL = nullptr;
        }
    }

    // add split event with all new actors
    newActorsSplitEvent->parentData.family = this;
    newActorsSplitEvent->parentData.userData = 0;
    newActorsSplitEvent->parentData.index = invalidIndex<uint32_t>();
    newActorsSplitEvent->numChildren = children.size();
    if (newActorsSplitEvent->numChildren > 0)
    {
        getQueue().addEvent(newActorsSplitEvent);
    }

    // replace family
    NVBLAST_FREE(m_familyLL);
    m_familyLL = newFamilyCopy;

    // update joints
    for (TkActorImpl& tkActor : m_actors)
    {
        if (!tkActor.m_jointList.isEmpty())
        {
            updateJoints(&tkActor);
        }
    }

    getQueue().dispatch();
}


TkActorImpl* TkFamilyImpl::getActorByChunk(uint32_t chunk)
{
    if (chunk >= NvBlastAssetGetChunkCount(m_asset->getAssetLLInternal(), logLL))
    {
        NVBLAST_LOG_WARNING("TkFamilyImpl::getActorByChunk: invalid chunk index.  Returning NULL.");
        return nullptr;
    }

    NvBlastActor* actorLL = NvBlastFamilyGetChunkActor(m_familyLL, chunk, logLL);
    return actorLL ? getActorByActorLL(actorLL) : nullptr;
}


void TkFamilyImpl::applyFractureInternal(const NvBlastFractureBuffers* commands)
{
    NvBlastSupportGraph graph = getAsset()->getGraph();

    // apply bond fracture commands on relevant actors
    {
        TkActorImpl* currActor = nullptr;
        NvBlastBondFractureData* bondFractures = commands->bondFractures;
        uint32_t bondFracturesCount = 0;

        auto applyFracture = [&]()
        {
            if (bondFracturesCount > 0)
            {
                if (currActor != nullptr && currActor->isActive())
                {
                    NvBlastFractureBuffers newCommands;
                    newCommands.bondFractures = bondFractures;
                    newCommands.bondFractureCount = bondFracturesCount;
                    newCommands.chunkFractures = nullptr;
                    newCommands.chunkFractureCount = 0;
                    currActor->applyFracture(nullptr, &newCommands);
                }

                bondFractures += bondFracturesCount;
                bondFracturesCount = 0;
            }
        };

        for (uint32_t i = 0; i < commands->bondFractureCount; ++i, ++bondFracturesCount)
        {
            const NvBlastBondFractureData& command = commands->bondFractures[i];
            uint32_t chunk0 = graph.chunkIndices[command.nodeIndex0];
            uint32_t chunk1 = graph.chunkIndices[command.nodeIndex1];
            TkActorImpl* actor0 = getActorByChunk(chunk0);
            TkActorImpl* actor1 = getActorByChunk(chunk1);
            if (actor0 != actor1)
            {
                // skipping this event, bond already broken
                actor0 = nullptr;
            }
            if (actor0 != currActor)
            {
                applyFracture();
                currActor = actor0;
            }
        }

        if (bondFracturesCount > 0)
        {
            applyFracture();
        }
    }

    // apply chunk fracture commands on relevant actors
    {
        TkActorImpl* currActor = nullptr;
        NvBlastChunkFractureData* chunkFractures = commands->chunkFractures;
        uint32_t chunkFracturesCount = 0;

        auto applyFracture = [&]()
        {
            if (chunkFracturesCount > 0)
            {
                if (currActor != nullptr && currActor->isActive())
                {
                    NvBlastFractureBuffers newCommands;
                    newCommands.bondFractures = nullptr;
                    newCommands.bondFractureCount = 0;
                    newCommands.chunkFractures = chunkFractures;
                    newCommands.chunkFractureCount = chunkFracturesCount;
                    currActor->applyFracture(nullptr, &newCommands);
                }

                chunkFractures += chunkFracturesCount;
                chunkFracturesCount = 0;
            }
        };

        for (uint32_t i = 0; i < commands->chunkFractureCount; ++i, ++chunkFracturesCount)
        {
            const NvBlastChunkFractureData& command = commands->chunkFractures[i];
            TkActorImpl* actor = getActorByChunk(command.chunkIndex);
            if (actor != currActor)
            {
                applyFracture();
                currActor = actor;
            }
        }
        if (chunkFracturesCount > 0)
        {
            applyFracture();
        }
    }
}


void TkFamilyImpl::updateJoints(TkActorImpl* actor, TkEventQueue* alternateQueue)
{
    // Copy joint array for safety against implementation of joint->setActor
    TkJointImpl** joints = reinterpret_cast<TkJointImpl**>(NvBlastAlloca(sizeof(TkJointImpl*)*actor->getJointCountInternal()));
    TkJointImpl** stop = joints + actor->getJointCountInternal();
    TkJointImpl** jointHandle = joints;
    for (TkActorImpl::JointIt j(*actor); (bool)j; ++j)
    {
        *jointHandle++ = *j;
    }
    jointHandle = joints;
    while (jointHandle < stop)
    {
        TkJointImpl* joint = *jointHandle++;
        
        const TkJointData& data = joint->getDataInternal();

        TkActorImpl* actor0 = data.actors[0] != nullptr ?
            static_cast<TkActorImpl&>(*data.actors[0]).getFamilyImpl().getActorByChunk(data.chunkIndices[0]) : nullptr;

        TkActorImpl* actor1 = data.actors[1] != nullptr ?
            static_cast<TkActorImpl&>(*data.actors[1]).getFamilyImpl().getActorByChunk(data.chunkIndices[1]) : nullptr;

        joint->setActors(actor0, actor1, alternateQueue);
    }
}


const TkAsset* TkFamilyImpl::getAsset() const
{
    return m_asset;
}


//////// Static functions ////////

TkFamilyImpl* TkFamilyImpl::create(const TkAssetImpl* asset)
{
    TkFamilyImpl* family = NVBLAST_NEW(TkFamilyImpl);
    family->m_asset = asset;
    void* mem = NVBLAST_ALLOC_NAMED(NvBlastAssetGetFamilyMemorySize(asset->getAssetLL(), logLL), "TkFamilyImpl::create");
    family->m_familyLL = NvBlastAssetCreateFamily(mem, asset->getAssetLL(), logLL);
    //family->addListener(*TkFrameworkImpl::get());

    if (family->m_familyLL == nullptr)
    {
        NVBLAST_LOG_ERROR("TkFamilyImpl::create: low-level family could not be created.");
        family->release();
        return nullptr;
    }

    uint32_t maxActorCount = NvBlastFamilyGetMaxActorCount(family->m_familyLL, logLL);
    family->m_actors.resize(maxActorCount);

    family->m_internalJointBuffer.resize(asset->getJointDescCountInternal() * sizeof(TkJointImpl), 0);
    family->m_internalJointCount = asset->getJointDescCountInternal();

    return family;
}


TkJointImpl** TkFamilyImpl::createExternalJointHandle(const NvBlastID& otherFamilyID, uint32_t chunkIndex0, uint32_t chunkIndex1)
{
    JointSet* jointSet;
    const FamilyIDMap::Entry* jointSetIndexEntry = m_familyIDMap.find(otherFamilyID);
    uint32_t otherFamilyIndex;
    if (jointSetIndexEntry != nullptr)
    {
        otherFamilyIndex = jointSetIndexEntry->second;
        jointSet = m_jointSets[otherFamilyIndex];
    }
    else
    {
        jointSet = NVBLAST_NEW(JointSet);
        NVBLAST_CHECK_ERROR(jointSet != nullptr, "TkFamilyImpl::addExternalJoint: failed to create joint set for other family ID.", return nullptr);
        jointSet->m_familyID = otherFamilyID;
        otherFamilyIndex = m_jointSets.size();
        m_familyIDMap[otherFamilyID] = otherFamilyIndex;
        m_jointSets.pushBack(jointSet);
    }

    const ExternalJointKey key(chunkIndex0, chunkIndex1);
    const bool jointExists = jointSet->m_joints.find(key) != nullptr;
    NVBLAST_CHECK_WARNING(!jointExists, "TkFamilyImpl::addExternalJoint: joint already added.", return nullptr);

    return &jointSet->m_joints[key];
}


bool TkFamilyImpl::deleteExternalJointHandle(TkJointImpl*& joint, const NvBlastID& otherFamilyID, uint32_t chunkIndex0, uint32_t chunkIndex1)
{
    const FamilyIDMap::Entry* jointSetIndexEntry = m_familyIDMap.find(otherFamilyID);
    if (jointSetIndexEntry != nullptr)
    {
        const uint32_t jointSetIndex = jointSetIndexEntry->second;
        ExternalJointKey jointKey = ExternalJointKey(chunkIndex0, chunkIndex1);
        const HashMap<ExternalJointKey, TkJointImpl*>::type::Entry* e = m_jointSets[jointSetIndex]->m_joints.find(jointKey);
        if (e != nullptr)
        {
            joint = e->second;  // Return value that was stored
            m_jointSets[jointSetIndex]->m_joints.erase(jointKey);
            // Delete the joint set if it is empty
            if (m_jointSets[jointSetIndex]->m_joints.size() == 0)
            {
                NVBLAST_DELETE(m_jointSets[jointSetIndex], JointSet);
                m_jointSets.replaceWithLast(jointSetIndex);
                m_familyIDMap.erase(otherFamilyID);
                if (jointSetIndex < m_jointSets.size())
                {
                    m_familyIDMap[m_jointSets[jointSetIndex]->m_familyID] = jointSetIndex;
                }
            }
            return true;
        }
    }

    return false;
}


TkJointImpl* TkFamilyImpl::findExternalJoint(const TkFamilyImpl* otherFamily, ExternalJointKey key) const
{
    const FamilyIDMap::Entry* jointSetIndexEntry = m_familyIDMap.find(getFamilyID(otherFamily));
    if (jointSetIndexEntry != nullptr)
    {
        const HashMap<ExternalJointKey, TkJointImpl*>::type::Entry* e = m_jointSets[jointSetIndexEntry->second]->m_joints.find(key);
        if (e != nullptr)
        {
            return e->second;
        }
    }

    return nullptr;
}

} // namespace Blast
} // namespace Nv
