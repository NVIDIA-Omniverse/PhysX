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
#include "NvBlastTkJointImpl.h"
#include "NvBlastTkActorImpl.h"
#include "NvBlastTkAssetImpl.h"
#include "NvBlastTkFamilyImpl.h"


namespace Nv
{
namespace Blast
{

//////// Member functions ////////

TkJointImpl::TkJointImpl(const TkJointDesc& desc, TkFamilyImpl* owner) : m_owner(owner)
{
    userData = nullptr;

    // Do not fire off a creation event.  Creation events will only be fired when a family-internal joint is created.
    NVBLAST_ASSERT(desc.families[0] != nullptr || desc.families[1] != nullptr);
    NVBLAST_ASSERT(desc.families[0] == nullptr || desc.chunkIndices[0] < static_cast<TkFamilyImpl*>(desc.families[0])->getAssetImpl()->getChunkCount());
    NVBLAST_ASSERT(desc.attachPositions[0].isFinite());
    NVBLAST_ASSERT(desc.families[1] == nullptr || desc.chunkIndices[1] < static_cast<TkFamilyImpl*>(desc.families[1])->getAssetImpl()->getChunkCount());
    NVBLAST_ASSERT(desc.attachPositions[1].isFinite());

    for (int i = 0; i < 2; ++i)
    {
        m_data.actors[i] = desc.families[i] != nullptr ? static_cast<TkFamilyImpl*>(desc.families[i])->getActorByChunk(desc.chunkIndices[i]) : nullptr;
        m_data.chunkIndices[i] = desc.chunkIndices[i];
        m_data.attachPositions[i] = desc.attachPositions[i];
        m_links[i].m_joint = this;
    }

    if (owner == nullptr)
    {
        TkFrameworkImpl::get()->onCreate(*this);
    }
}


void TkJointImpl::release()
{
    removeReferencesInActors();

    if (m_owner != nullptr)
    {
        // Internal joint
        m_owner->releaseJoint(*this);
    }
    else
    {
        // External joint
        removeReferencesInFamilies();
        TkFrameworkImpl::get()->onDestroy(*this);
        NVBLAST_DELETE(this, TkJointImpl);
    }
}


void TkJointImpl::setActors(TkActorImpl* actor0, TkActorImpl* actor1, TkEventQueue* alternateQueue)
{
    NVBLAST_ASSERT(m_data.actors[0] != nullptr || m_data.actors[1] != nullptr);

    const bool unreferenced = (actor0 == nullptr && m_data.actors[0] != nullptr) || (actor1 == nullptr && m_data.actors[1] != nullptr);

    removeReferencesInActors();

    if (!unreferenced)
    {
        if (actor0 != nullptr)
        {
            actor0->addJoint(m_links[0]);
        }
        
        if (actor1 != nullptr && actor1 != actor0)  // If the actors are the same, we only need one joint reference
        {
            actor1->addJoint(m_links[1]);
        }
    }

    // We do _not_ return if m_data.m_actors[0] == actor0 && m_data.m_actors[1] == actor1 since
    // this leads to a bug.  This function will only be called when an actor is split.  It is
    // possible that the two TkActors in a joint are the same as before, but in this case one
    // of the actors will be the split actor.  Since will be represented by a different
    // physical actor, this case still needs to be reported in an event. Returning when neither
    // TkActor has changed will prevent that, and lead to unwanted joint disconnection.

    const uint32_t familyToUse = m_data.actors[0] != actor0 ? 0 : 1;

    TkEventQueue* q = alternateQueue == nullptr ?
        &static_cast<TkActorImpl*>(m_data.actors[familyToUse])->getFamilyImpl().getQueue()
        : alternateQueue;

    const bool jointWasInternal = m_data.actors[0] == m_data.actors[1];

    if (unreferenced)
    {
        removeReferencesInFamilies();
        actor0 = actor1 = nullptr;  // Make both new actors NULL
    }

    if (!jointWasInternal || actor0 != actor1)
    {
        // The original actors were different, or they are now, signal a joint update
        TkJointUpdateEvent* e = q->allocData<TkJointUpdateEvent>();
        e->joint = this;
        e->subtype = unreferenced ? TkJointUpdateEvent::Unreferenced : (jointWasInternal ? TkJointUpdateEvent::External : TkJointUpdateEvent::Changed);
        m_data.actors[0] = actor0;
        m_data.actors[1] = actor1;
        q->addEvent(e);
    }
    else
    if (jointWasInternal)
    {
        // The joint was originally created within the same actor and now it remains within the same actor.  
        m_data.actors[0] = m_data.actors[1] = actor0;
    }
}


const TkJointData TkJointImpl::getData() const
{
    return getDataInternal();
}


void TkJointImpl::removeReferencesInActors()
{
    TkActorImpl* actor0 = static_cast<TkActorImpl*>(m_data.actors[0]);
    TkActorImpl* actor1 = static_cast<TkActorImpl*>(m_data.actors[1]);

    if (actor0 != nullptr)
    {
        actor0->removeJoint(m_links[0]);
    }

    if (actor1 != nullptr && actor1 != actor0)  // If the actors are the same, we only had one joint reference
    {
        actor1->removeJoint(m_links[1]);
    }
}


void TkJointImpl::removeReferencesInFamilies()
{
    if (m_owner != nullptr)
    {
        return; // Only concerned with external joints
    }

    NVBLAST_ASSERT(m_data.actors[0] != m_data.actors[1] || m_data.actors[0] == nullptr);    // This is enforced by the initial assumption in TkFrameworkImpl::createJoint.

    for (int i = 0; i < 2; ++i)
    {
        if (m_data.actors[i] != nullptr)
        {
            TkFamilyImpl& family = static_cast<TkActorImpl*>(m_data.actors[i])->getFamilyImpl();
            TkJointImpl* joint = nullptr;
            const bool found = family.deleteExternalJointHandle(joint, getFamilyID(m_data.actors[i ^ 1]), m_data.chunkIndices[i], m_data.chunkIndices[i ^ 1]);
            NVBLAST_ASSERT((!found && m_data.actors[i ^ 1] == nullptr) || joint == this);   // Might not be found if the actors in a family are in the process of being deleted
            NV_UNUSED(found);
        }
    }
}

} // namespace Blast
} // namespace Nv
