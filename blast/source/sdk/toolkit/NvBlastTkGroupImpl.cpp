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


#include "NvPreprocessor.h"

#include "NvBlastAssert.h"
#include "NvBlast.h"

#include "NvBlastTkFrameworkImpl.h"
#include "NvBlastTkGroupImpl.h"
#include "NvBlastTkActorImpl.h"
#include "NvBlastTkFamilyImpl.h"
#include "NvBlastTkAssetImpl.h"
#include "NvBlastTkTaskImpl.h"

#undef max
#undef min
#include <algorithm>

using namespace nvidia;


namespace Nv
{
namespace Blast
{

//////// Static data ////////

NVBLASTTK_DEFINE_TYPE_IDENTIFIABLE(Group);


//////// Member functions ////////

TkGroupImpl::TkGroupImpl() : m_actorCount(0), m_isProcessing(false)
{
#if NV_PROFILE
    memset(&m_stats, 0, sizeof(TkGroupStats)); 
#endif
}


TkGroupImpl::~TkGroupImpl()
{
    NVBLAST_ASSERT(getActorCount() == 0);
    NVBLAST_ASSERT(m_sharedMemory.size() == 0);
}


void TkGroupImpl::release()
{
    if (isProcessing())
    {
        // abort all processing? 
        NVBLAST_LOG_ERROR("TkGroup::release: cannot release Group while processing.");
        NVBLAST_ALWAYS_ASSERT_MESSAGE("TkGroup::release: cannot release Group while processing.");
        return;
    }

    for (auto it = m_sharedMemory.getIterator(); !it.done(); ++it)
    {
        TkFamilyImpl* family = it->first;
        for (TkActorImpl& actor : family->getActorsInternal())
        {
            if (actor.m_group == this)
            {
                removeActorInternal(actor);
            }
        }
        SharedMemory* mem = it->second;
        mem->release();
        NVBLAST_DELETE(mem, SharedMemory);
    }
    m_sharedMemory.clear();

    m_bondTempDataBlock.release();
    m_chunkTempDataBlock.release();
    m_bondEventDataBlock.release();
    m_chunkEventDataBlock.release();
    m_splitScratchBlock.release();

    NVBLAST_DELETE(this, TkGroupImpl);
}


void TkGroupImpl::addActorsInternal(TkActorImpl** actors, uint32_t numActors)
{
    for (uint32_t i = 0; i < numActors; i++)
    {
        addActorInternal(*actors[i]);
    }
}


void TkGroupImpl::addActorInternal(TkActorImpl& tkActor)
{
    NVBLAST_ASSERT(tkActor.getGroup() == nullptr);
    tkActor.m_group = this;
    m_actorCount++;
}


bool TkGroupImpl::addActor(TkActor& actor)
{
    TkActorImpl& tkActor = static_cast<TkActorImpl&>(actor);
    if (tkActor.getGroup() != nullptr)
    {
        NVBLAST_LOG_ERROR("TkGroup::addActor: actor already belongs to a Group.  Remove from current group first.");
        return false;
    }

    if (isProcessing())
    {
        NVBLAST_LOG_ERROR("TkGroup::addActor: cannot alter Group while processing.");
        return false;
    }

    // mark the actor that it now belongs to this group
    addActorInternal(tkActor);

    // actors that were fractured already or have damage requested
    // must be enqueued to be processed
    if (tkActor.isPending())
    {
        enqueue(&tkActor);
    }

    TkFamilyImpl& family = tkActor.getFamilyImpl();
    SharedMemory* mem = m_sharedMemory[&family];
    if (mem == nullptr)
    {
        // the actor belongs to a family not involved in this group yet
        // shared memory must be allocated and temporary buffers adjusted accordingly

        BLAST_PROFILE_ZONE_BEGIN("family memory");
        mem = NVBLAST_NEW(SharedMemory);
        mem->allocate(family);
        m_sharedMemory[&family] = mem;
        BLAST_PROFILE_ZONE_END("family memory");

        BLAST_PROFILE_ZONE_BEGIN("group memory");
        
        const uint32_t workerCount = m_workers.size();

        NvBlastLog theLog = logLL;

        // this group's tasks will use one temporary buffer each, which is of max size of, for all families involved
        const size_t requiredScratch = NvBlastActorGetRequiredScratchForSplit(tkActor.getActorLL(), theLog);
        if (static_cast<size_t>(m_splitScratchBlock.numElementsPerBlock()) < requiredScratch)
        {
            m_splitScratchBlock.release();
            m_splitScratchBlock.allocate(static_cast<uint32_t>(requiredScratch), workerCount);
        }
        
        // generate and apply fracture may create an entry for each bond
        const uint32_t bondCount = NvBlastAssetGetBondCount(tkActor.getAsset()->getAssetLL(), theLog);
        if (m_bondTempDataBlock.numElementsPerBlock() < bondCount)
        {
            m_bondTempDataBlock.release();
            m_bondTempDataBlock.allocate(bondCount, workerCount);
            m_bondEventDataBlock.release();
            m_bondEventDataBlock.allocate(bondCount, workerCount);
        }

        // apply fracture may create an entry for each lower-support chunk
        const uint32_t graphNodeCount = NvBlastAssetGetSupportGraph(tkActor.getAsset()->getAssetLL(), theLog).nodeCount;
        const uint32_t subsupportChunkCount
            = NvBlastAssetGetChunkCount(tkActor.getAsset()->getAssetLL(), theLog)
            - NvBlastAssetGetFirstSubsupportChunkIndex(tkActor.getAsset()->getAssetLL(), theLog);
        const uint32_t chunkCount = graphNodeCount + subsupportChunkCount;
        if (m_chunkTempDataBlock.numElementsPerBlock() < chunkCount)
        {
            m_chunkTempDataBlock.release();
            m_chunkTempDataBlock.allocate(chunkCount, workerCount);
            m_chunkEventDataBlock.release();
            m_chunkEventDataBlock.allocate(chunkCount, workerCount);
        }
        BLAST_PROFILE_ZONE_END("group memory");
    }
    mem->addReference();

    return true;
}


uint32_t TkGroupImpl::getActors(TkActor** buffer, uint32_t bufferSize, uint32_t indexStart /* = 0 */) const
{
    BLAST_PROFILE_SCOPE_L("TkGroup::getActors");

    uint32_t actorCount = m_actorCount;
    if (actorCount <= indexStart)
    {
        NVBLAST_LOG_WARNING("TkGroup::getActors: indexStart beyond end of actor list.");
        return 0;
    }

    actorCount -= indexStart;
    if (actorCount > bufferSize)
    {
        actorCount = bufferSize;
    }

    uint32_t index = 0;
    bool done = false;
    for (auto it = const_cast<TkGroupImpl*>(this)->m_sharedMemory.getIterator(); !it.done();++it)
    {
        TkFamilyImpl* fam = it->first;
        for (TkActorImpl& actor : fam->getActorsInternal())
        {
            if (actor.m_group == this)
            {
                NVBLAST_ASSERT(actor.isActive());
                
                if (index >= indexStart)
                {
                    *buffer++ = &actor;
                }
            
                index++;
                done = (index - indexStart) >= actorCount;
            }
            if (done) break;
        }
        if (done) break;
    }

    return actorCount;
}


void TkGroupImpl::removeActorInternal(TkActorImpl& tkActor)
{
    NVBLAST_ASSERT(tkActor.m_group == this);
    tkActor.m_group = nullptr;
    m_actorCount--;
}


void TkGroupImpl::releaseSharedMemory(TkFamilyImpl* fam, SharedMemory* mem)
{
    NVBLAST_ASSERT(mem != nullptr && m_sharedMemory[fam] == mem);
    mem->release();
    m_sharedMemory.erase(fam);
    NVBLAST_DELETE(mem, SharedMemory);
}


bool TkGroupImpl::removeActor(TkActor& actor)
{
    TkActorImpl& tkActor = static_cast<TkActorImpl&>(actor);

    if (tkActor.getGroup() != this)
    {
        NVBLAST_LOG_ERROR("TkGroup::removeActor: actor does not belong to this Group.");
        return false;
    }

    if (isProcessing())
    {
        NVBLAST_LOG_ERROR("TkGroup::removeActor: cannot alter Group while processing.");
        return false;
    }

    removeActorInternal(tkActor);

    // pending actors must be removed from the job queue as well
    if(tkActor.isPending())
    {
        uint32_t index = tkActor.m_groupJobIndex;
        tkActor.m_groupJobIndex = invalidIndex<uint32_t>();
        if (index < m_jobs.size())
        {
            m_jobs.replaceWithLast(index);
            if (index < m_jobs.size())
            {
                NVBLAST_ASSERT(m_jobs[index].m_tkActor->m_groupJobIndex == m_jobs.size());
                NVBLAST_ASSERT(m_jobs[index].m_tkActor->isPending());
                m_jobs[index].m_tkActor->m_groupJobIndex = index;
            }
        }
    }

    // if the actor is the last of its family in this group
    // the group-family memory can be released
    TkFamilyImpl* family = &tkActor.getFamilyImpl();
    SharedMemory* mem = getSharedMemory(family);
    if (mem->removeReference())
    {
        releaseSharedMemory(family, mem);
    }

    return true;
}


TkGroupImpl* TkGroupImpl::create(const TkGroupDesc& desc)
{
    TkGroupImpl* group = NVBLAST_NEW(TkGroupImpl);

    group->setWorkerCount(desc.workerCount);

    return group;
}


void TkGroupImpl::setWorkerCount(uint32_t workerCount)
{
    if (isProcessing())
    {
        NVBLAST_LOG_WARNING("TkGroup::setWorkerCount: Group is still processing, call TkGroup::endProcess first.");
        return;
    }

    if (workerCount == 0)
    {
        NVBLAST_LOG_WARNING("TkGroup: attempting to create a Group with 0 workers. Forced to 1.");
        workerCount = 1;
    }

    if (workerCount != m_workers.size())
    {
        m_workers.resize(workerCount);

        uint32_t workerId = 0;
        for (auto& worker : m_workers)
        {
            worker.m_id = workerId++;
            worker.m_group = this;
        }

        const uint32_t bondCount = m_bondTempDataBlock.numElementsPerBlock();
        if (bondCount > 0)
        {
            m_bondTempDataBlock.release();
            m_bondTempDataBlock.allocate(bondCount, workerCount);
            m_bondEventDataBlock.release();
            m_bondEventDataBlock.allocate(bondCount, workerCount);
        }
        const uint32_t chunkCount = m_chunkTempDataBlock.numElementsPerBlock();
        if (chunkCount > 0)
        {
            m_chunkTempDataBlock.release();
            m_chunkTempDataBlock.allocate(chunkCount, workerCount);
            m_chunkEventDataBlock.release();
            m_chunkEventDataBlock.allocate(chunkCount, workerCount);
        }
        const uint32_t scratchSize = m_splitScratchBlock.numElementsPerBlock();
        if (scratchSize > 0)
        {
            m_splitScratchBlock.release();
            m_splitScratchBlock.allocate(scratchSize, workerCount);
        }
    }
}


NV_INLINE uint32_t TkGroupImpl::getWorkerCount() const
{
    return m_workers.size();
}


uint32_t TkGroupImpl::startProcess()
{
    BLAST_PROFILE_SCOPE_L("TkGroup::startProcess");

    if (!setProcessing(true))
    {
        NVBLAST_LOG_WARNING("TkGroup::process: Group is still processing, call TkGroup::endProcess first.");
        return 0;
    }

    if (m_jobs.size() > 0)
    {
        BLAST_PROFILE_ZONE_BEGIN("task setup");

        BLAST_PROFILE_ZONE_BEGIN("setup job queue");
        for (const auto& job : m_jobs)
        {
            const TkActorImpl* a = job.m_tkActor;
            SharedMemory* mem = getSharedMemory(&a->getFamilyImpl());

            const uint32_t damageCount = a->m_damageBuffer.size();

            // applyFracture'd actor do not necessarily have damage queued
            NVBLAST_ASSERT(damageCount > 0 || a->m_flags.isSet(TkActorFlag::DAMAGED));

            // no reason to be here without these
            NVBLAST_ASSERT(a->m_flags.isSet(TkActorFlag::PENDING));
            NVBLAST_ASSERT(a->m_group == this);

            // collect the amount of event payload memory to preallocate for TkWorkers
            mem->m_eventsMemory += damageCount * (sizeof(TkFractureCommands) + sizeof(TkFractureEvents)) + sizeof(TkSplitEvent);

            // collect the amount of event entries to preallocate for TkWorkers
            // (two TkFracture* events per damage plus one TkSplitEvent)
            mem->m_eventsCount += 2 * damageCount + 1;
        }
        BLAST_PROFILE_ZONE_END("setup job queue");

        BLAST_PROFILE_ZONE_BEGIN("memory protect");
        for (auto it = m_sharedMemory.getIterator(); !it.done(); ++it)
        {
            // preallocate the event memory for TkWorkers
            SharedMemory* mem = it->second;
            mem->m_events.reserveData(mem->m_eventsMemory);
            mem->m_events.reserveEvents(mem->m_eventsCount);

            // these counters are not used anymore
            // reset them immediately for next time
            mem->m_eventsCount = 0;
            mem->m_eventsMemory = 0;

            // switch to parallel mode
            mem->m_events.protect(true);
        }
        BLAST_PROFILE_ZONE_END("memory protect");

        BLAST_PROFILE_ZONE_END("task setup");


        for (auto&worker : m_workers)
        {
            worker.initialize();
        }

        return m_jobs.size();
    }
    else
    {
        bool success = setProcessing(false);
        NVBLAST_ASSERT(success);
        NV_UNUSED(success);
        return 0;
    }
}


bool TkGroupImpl::endProcess()
{
    if (isProcessing())
    {
        BLAST_PROFILE_SCOPE_L("TkGroupImpl::endProcess");

        if (m_jobs.size() > 0)
        {
#if NV_PROFILE
            BLAST_PROFILE_ZONE_BEGIN("accumulate timers");
            NvBlastTimers accumulated;
            NvBlastTimersReset(&accumulated);
            uint32_t jobCount = 0;
            int64_t workerTime = 0;
            for (TkWorker& worker : m_workers)
            {
                accumulated += worker.m_stats.timers;
                jobCount += worker.m_stats.processedActorsCount;
                workerTime += worker.m_stats.workerTime;
            }
            m_stats.timers = accumulated;
            m_stats.processedActorsCount = jobCount;
            m_stats.workerTime = workerTime;
            BLAST_PROFILE_ZONE_END("accumulate timers");
#endif

            BLAST_PROFILE_ZONE_BEGIN("job update");
            for (auto& j : m_jobs)
            {
                if (j.m_newActorsCount)
                {
                    TkFamilyImpl* fam = &j.m_tkActor->getFamilyImpl();
                    SharedMemory* mem = getSharedMemory(fam);

                    // as LL is implemented, where newActorsCount the parent is always deleted
                    removeActorInternal(*j.m_tkActor);
                    mem->removeReference();
                    addActorsInternal(j.m_newActors, j.m_newActorsCount);
                    mem->addReference(j.m_newActorsCount);
                    
                    // Update joints
                    mem->m_events.protect(false); // allow allocations again
                    BLAST_PROFILE_ZONE_BEGIN("updateJoints");
                    fam->updateJoints(j.m_tkActor, &mem->m_events);
                    BLAST_PROFILE_ZONE_END("updateJoints");
                }

                // virtually dequeue the actor
                // the queue itself is cleared right after this loop
                j.m_tkActor->m_flags.clear(TkActorFlag::PENDING);
                j.m_tkActor->m_groupJobIndex = invalidIndex<uint32_t>();
                BLAST_PROFILE_ZONE_BEGIN("damageBuffer.clear");
                j.m_tkActor->m_damageBuffer.clear();
                BLAST_PROFILE_ZONE_END("damageBuffer.clear");
            }
            m_jobs.clear();
            BLAST_PROFILE_ZONE_END("job update");

            BLAST_PROFILE_ZONE_BEGIN("event dispatch");
            for (auto it = m_sharedMemory.getIterator(); !it.done(); ++it)
            {
                BLAST_PROFILE_SCOPE_L("event dispatch");
                TkFamilyImpl* family = it->first;
                SharedMemory* mem = it->second;

                NVBLAST_ASSERT(family != nullptr);
                NVBLAST_ASSERT(mem != nullptr && mem->isUsed());

                // where no actor of a family has split, 
                // its group/family event queue has not been 
                // unprotected in the jobs loop above
                mem->m_events.protect(false);

                family->getQueue().dispatch(mem->m_events);

                mem->m_events.reset();
                mem->reset();
            }
            BLAST_PROFILE_ZONE_END("event dispatch");

            BLAST_PROFILE_ZONE_BEGIN("event memory release");
            for (auto& worker : m_workers)
            {
                worker.m_bondBuffer.clear();
                worker.m_chunkBuffer.clear();
            }
            BLAST_PROFILE_ZONE_END("event memory release");
        }

        bool success = setProcessing(false);
        NVBLAST_ASSERT(success);
        return success;
    }

    return false;
}


bool TkGroupImpl::setProcessing(bool value)
{
    bool expected = !value;
    return m_isProcessing.compare_exchange_strong(expected, value);
}


void TkGroupImpl::enqueue(TkActorImpl* tkActor)
{
    NVBLAST_ASSERT(tkActor->getGroupImpl() != nullptr);
    NVBLAST_ASSERT(tkActor->getGroupImpl() == this);
    NVBLAST_ASSERT(isInvalidIndex(tkActor->m_groupJobIndex));
    NVBLAST_ASSERT(isProcessing() == false);
#if NV_DEBUG
    for (TkWorkerJob& j : m_jobs)
    {
        NVBLAST_ASSERT(j.m_tkActor != tkActor);
    }
#endif

    tkActor->m_groupJobIndex = m_jobs.size();
    TkWorkerJob& j = m_jobs.insert();
    j.m_tkActor = tkActor;
}


TkGroupWorker* TkGroupImpl::acquireWorker()
{
    BLAST_PROFILE_SCOPE_L("TkGroupImpl::acquireWorker");
    std::unique_lock<std::mutex> lk(m_workerMtx);
    for (auto& worker:m_workers)
    {
        if (!worker.m_isBusy)
        {
            worker.m_isBusy = true;
            return &worker;
        }
    }
    return nullptr;
}


void TkGroupImpl::returnWorker(TkGroupWorker* worker)
{
    BLAST_PROFILE_SCOPE_L("TkGroupImpl::returnWorker");
    std::unique_lock<std::mutex> lk(m_workerMtx);
    auto w = static_cast<TkWorker*>(worker);
    NVBLAST_CHECK_WARNING(w->m_group == this, "TkGroup::returnWorker worker does not belong to this group.", return);
    w->m_isBusy = false;
}


} // namespace Blast
} // namespace Nv
