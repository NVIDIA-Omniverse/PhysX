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


#ifndef NVBLASTTKGROUPIMPL_H
#define NVBLASTTKGROUPIMPL_H


#include "NvBlastTkTaskImpl.h"
#include "NvBlastTkGroup.h"
#include "NvBlastTkTypeImpl.h"


namespace Nv
{
namespace Blast
{

class TkActorImpl;
class TkFamilyImpl;

NVBLASTTK_IMPL_DECLARE(Group)
{
    ~TkGroupImpl();

public:
    TkGroupImpl();

    NVBLASTTK_IMPL_DEFINE_IDENTIFIABLE('G', 'R', 'P', '\0');

    static TkGroupImpl*     create(const TkGroupDesc& desc);

    // Begin TkGroup
    virtual bool            addActor(TkActor& actor) override;

    virtual uint32_t        getActorCount() const override;

    virtual uint32_t        getActors(TkActor** buffer, uint32_t bufferSize, uint32_t indexStart = 0) const override;

    virtual uint32_t        startProcess() override;
    virtual bool            endProcess() override;

    virtual void            getStats(TkGroupStats& stats) const override;

    virtual void            setWorkerCount(uint32_t workerCount) override;
    virtual uint32_t        getWorkerCount() const override;

    virtual TkGroupWorker*  acquireWorker() override;
    virtual void            returnWorker(TkGroupWorker*) override;
    // End TkGroup

    // TkGroupImpl API

    /**
    Remove the actor from this group if the actor actually belongs to it and the group is not processing.

    \param[in]  actor       The TkActor to remove.

    \return                 true if removing succeeded, false otherwise
    */
    bool                    removeActor(TkActor& actor);

    /**
    Add the actor to this group's job queue. 
    It is the caller's responsibility to add an actor only once. This condition is checked in debug builds.
    */
    void                    enqueue(TkActorImpl* tkActor);

    /**
    Atomically check if this group is processing actors. @see setProcessing()

    \return                 true between startProcess() and endProcess() calls, false otherwise
    */
    bool                    isProcessing() const;

private:
    /**
    Atomically set the processing state. This function checks for the current state
    before changing it. @see isProcessing()

    \param[in]  value       the value of the new state

    \return                 true if the new state could be set, false otherwise
    */
    bool                    setProcessing(bool value);

    /** 
    Get the group-family shared memory for the specified family. To be used when the memory is expected to already exist.
    */
    SharedMemory*           getSharedMemory(TkFamilyImpl* family);
    void                    releaseSharedMemory(TkFamilyImpl* fam, SharedMemory* mem);

    // functions to add/remove actors _without_ group-family memory management
    void                    addActorInternal(TkActorImpl& tkActor);
    void                    addActorsInternal(TkActorImpl** actors, uint32_t numActors);
    void                    removeActorInternal(TkActorImpl& tkActor);


    uint32_t                                        m_actorCount;           //!< number of actors in this group

    HashMap<TkFamilyImpl*, SharedMemory*>::type     m_sharedMemory;         //!< memory sharable by actors in the same family in this group

    // it is assumed no more than the asset's number of bond and chunks fracture commands are produced
    SharedBlock<NvBlastChunkFractureData>           m_chunkTempDataBlock;   //!< chunk data for damage/fracture
    SharedBlock<NvBlastBondFractureData>            m_bondTempDataBlock;    //!< bond data for damage/fracture
    SharedBlock<NvBlastChunkFractureData>           m_chunkEventDataBlock;  //!< initial memory block for event data
    SharedBlock<NvBlastBondFractureData>            m_bondEventDataBlock;   //!< initial memory block for event data
    SharedBlock<char>                               m_splitScratchBlock;    //!< split scratch memory 

    std::atomic<bool>                               m_isProcessing;         //!< true while workers are processing

    Array<TkWorker>::type                           m_workers;              //!< this group's workers

    Array<TkWorkerJob>::type                        m_jobs;                 //!< this group's process jobs

//#if NV_PROFILE
    TkGroupStats                                    m_stats;                //!< accumulated group's worker stats
//#endif

    std::mutex  m_workerMtx;

    friend class TkWorker;
};


NV_INLINE bool TkGroupImpl::isProcessing() const
{
    return m_isProcessing.load();
}


NV_INLINE void TkGroupImpl::getStats(TkGroupStats& stats) const
{
#if NV_PROFILE
    memcpy(&stats, &m_stats, sizeof(TkGroupStats));
#else
    NV_UNUSED(stats);
#endif
}


NV_INLINE uint32_t TkGroupImpl::getActorCount() const
{
    return m_actorCount;
}


NV_INLINE SharedMemory* TkGroupImpl::getSharedMemory(TkFamilyImpl* family)
{
    SharedMemory* mem = m_sharedMemory[family];
    NVBLAST_ASSERT(mem != nullptr);
    return mem;
}


NV_FORCE_INLINE void operator +=(NvBlastTimers& lhs, const NvBlastTimers& rhs)
{
    lhs.material += rhs.material;
    lhs.fracture += rhs.fracture;
    lhs.island += rhs.fracture;
    lhs.partition += rhs.partition;
    lhs.visibility += rhs.visibility;
}


} // namespace Blast
} // namespace Nv


#endif // ifndef NVBLASTTKGROUPIMPL_H
