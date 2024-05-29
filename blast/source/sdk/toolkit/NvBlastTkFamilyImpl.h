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


#ifndef NVBLASTTKFAMILYIMPL_H
#define NVBLASTTKFAMILYIMPL_H

#include "NvBlastTkCommon.h"

#include "NvBlastTkFamily.h"
#include "NvBlastTkTypeImpl.h"
#include "NvBlastTkActorImpl.h"

#include "NvBlastTkEventQueue.h"
#include "NvBlastHashSet.h"
#include "NvBlastHashMap.h"

#include "NvBlast.h"
#include "NvBlastAssert.h"
#include "NvBlastDLink.h"


// Forward declarations
struct NvBlastFamily;

namespace Nv
{
namespace Blast
{

// Forward declarations
class TkGroupImpl;
class TkAssetImpl;


NVBLASTTK_IMPL_DECLARE(Family)
{
public:
    TkFamilyImpl();
    TkFamilyImpl(const NvBlastID& id);
    ~TkFamilyImpl();

    NVBLASTTK_IMPL_DEFINE_IDENTIFIABLE('A', 'C', 'T', 'F');

    // Begin TkFamily
    virtual const NvBlastFamily*    getFamilyLL() const override;

    virtual uint32_t                getActorCount() const override;

    virtual uint32_t                getActors(TkActor** buffer, uint32_t bufferSize, uint32_t indexStart = 0) const override;

    virtual void                    addListener(TkEventListener& l) override { m_queue.addListener(l); }

    virtual void                    removeListener(TkEventListener& l) override { m_queue.removeListener(l); }

    virtual void                    applyFracture(const NvBlastFractureBuffers* commands) override { applyFractureInternal(commands); }

    virtual const TkAsset*          getAsset() const override;

    virtual void                    reinitialize(const NvBlastFamily* newFamily, TkGroup* group) override;
    // End TkFamily

    // Public methods
    static TkFamilyImpl*            create(const TkAssetImpl* asset);

    const TkAssetImpl*              getAssetImpl() const;

    NvBlastFamily*                  getFamilyLLInternal() const;

    uint32_t                        getActorCountInternal() const;

    TkActorImpl*                    addActor(NvBlastActor* actorLL);

    void                            applyFractureInternal(const NvBlastFractureBuffers* commands);

    void                            removeActor(TkActorImpl* actorLL);

    TkEventQueue&                   getQueue() { return m_queue; }

    TkActorImpl*                    getActorByActorLL(const NvBlastActor* actorLL);

    void                            updateJoints(TkActorImpl* actor, TkEventQueue* alternateQueue = nullptr);

    Array<TkActorImpl>::type&       getActorsInternal();

    uint32_t                        getInternalJointCount() const;

    TkJointImpl*                    getInternalJoints() const;

    TkJointImpl**                   createExternalJointHandle(const NvBlastID& otherFamilyID, uint32_t chunkIndex0, uint32_t chunkIndex1);

    bool                            deleteExternalJointHandle(TkJointImpl*& joint, const NvBlastID& otherFamilyID, uint32_t chunkIndex0, uint32_t chunkIndex1);

    void                            releaseJoint(TkJointImpl& joint);

    TkActorImpl*                    getActorByChunk(uint32_t chunkIndex);

    typedef nvidia::shdfnd::Pair<uint32_t, uint32_t> ExternalJointKey;   //!< The chunk indices within the TkFamily objects joined by the joint.  These chunks will be support chunks.

    TkJointImpl*                    findExternalJoint(const TkFamilyImpl* otherFamily, ExternalJointKey key) const;

private:
    TkActorImpl*                    getActorByIndex(uint32_t index);

    struct JointSet
    {
        NvBlastID                                       m_familyID;
        HashMap<ExternalJointKey, TkJointImpl*>::type   m_joints;
    };

    typedef HashMap<NvBlastID, uint32_t>::type  FamilyIDMap;

    NvBlastFamily*              m_familyLL;
    Array<TkActorImpl>::type    m_actors;
    uint32_t                    m_internalJointCount;
    Array<uint8_t>::type        m_internalJointBuffer;
    Array<JointSet*>::type      m_jointSets;
    FamilyIDMap                 m_familyIDMap;
    const TkAssetImpl*          m_asset;

    TkEventQueue                m_queue;
};


//////// TkFamilyImpl inline methods ////////

NV_INLINE const TkAssetImpl* TkFamilyImpl::getAssetImpl() const
{
    return m_asset;
}


NV_INLINE NvBlastFamily* TkFamilyImpl::getFamilyLLInternal() const
{ 
    return m_familyLL; 
}


NV_INLINE uint32_t TkFamilyImpl::getActorCountInternal() const
{
    NVBLAST_ASSERT(m_familyLL != nullptr);

    return NvBlastFamilyGetActorCount(m_familyLL, logLL);
}


NV_INLINE TkActorImpl* TkFamilyImpl::getActorByIndex(uint32_t index)
{
    NVBLAST_ASSERT(index < m_actors.size());
    return &m_actors[index];
}


NV_INLINE TkActorImpl* TkFamilyImpl::getActorByActorLL(const NvBlastActor* actorLL)
{
    uint32_t index = NvBlastActorGetIndex(actorLL, logLL);
    return getActorByIndex(index);
}


NV_INLINE Array<TkActorImpl>::type& TkFamilyImpl::getActorsInternal()
{
    return m_actors;
}


NV_INLINE uint32_t TkFamilyImpl::getInternalJointCount() const
{
    return m_internalJointCount;
}


NV_INLINE TkJointImpl* TkFamilyImpl::getInternalJoints() const
{
    return const_cast<TkJointImpl*>(reinterpret_cast<const TkJointImpl*>(m_internalJointBuffer.begin()));
}


NV_INLINE void TkFamilyImpl::releaseJoint(TkJointImpl& joint)
{
    NVBLAST_ASSERT(joint.m_owner == this);
    NVBLAST_ASSERT(&joint >= getInternalJoints() && &joint < getInternalJoints() + getInternalJointCount() * sizeof(TkJointImpl));

    joint.~TkJointImpl();
    joint.m_owner = nullptr;
}


//////// Inline global functions ////////

NV_INLINE const NvBlastID& getFamilyID(const TkActor* actor)
{
    return actor != nullptr ? static_cast<const TkActorImpl*>(actor)->getFamilyImpl().getIDInternal() : *reinterpret_cast<const NvBlastID*>("\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0");
}

NV_INLINE const NvBlastID& getFamilyID(const TkFamilyImpl* family)
{
    return family != nullptr ? family->getIDInternal() : *reinterpret_cast<const NvBlastID*>("\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0");
}

} // namespace Blast
} // namespace Nv


#endif // ifndef NVBLASTTKFAMILYIMPL_H
