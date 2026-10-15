// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-EVENTS-001
// @covers AC-1 AC-2 AC-3 AC-4
// @implements REQ-CAPI-COMPANION-001
// @covers AC-1 AC-4
#include "ovphysx_companion/events.h"
#include "internal/sdk/CompanionRuntime.hpp"
#include "CarboniteLoader/CarboniteLoader.hpp"
#include <omni/physx/NativeEvents.h>
#include <map>
#include <memory>
#include <mutex>
#include <cstring>
#include <vector>
namespace ovphysx::companion::events
{
using namespace omni::physx::native;
using Status=ovphysx_companion_status_t;
struct Queue {
    CompanionServiceLease lease{companionRuntime()};
    std::shared_ptr<ovphysx::CarboniteLoader> runtime;
    std::shared_ptr<EventQueue> native;
    std::mutex mutex;
    bool closed=false;
    explicit Queue(std::shared_ptr<ovphysx::CarboniteLoader> r):runtime(std::move(r)){}
    ~Queue(){closeEventQueue(native);native.reset();runtime.reset();}
};

Status convert(EventResult status) {
    switch(status) {
    case EventResult::Success:return OVPHYSX_COMPANION_OK;
    case EventResult::Invalid:return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    case EventResult::Stale:return OVPHYSX_COMPANION_STALE;
    case EventResult::Busy:return OVPHYSX_COMPANION_BUSY;
    case EventResult::Unavailable:return OVPHYSX_COMPANION_UNAVAILABLE;
    case EventResult::Error:return OVPHYSX_COMPANION_ERROR;
    }
    return OVPHYSX_COMPANION_ERROR;
}
Status open(uint64_t attachment,uint32_t mask,uint32_t capacity,const char* filter,uint64_t* output) noexcept {
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try {
        CompanionOperationGuard admission(companionRuntime());
        if (!admission) return OVPHYSX_COMPANION_BUSY;
        std::shared_ptr<ovphysx::CarboniteLoader> runtime=companionRuntimeForAttachment(attachment);
        if (!runtime) return OVPHYSX_COMPANION_STALE;
        std::shared_ptr<Queue> queue=std::make_shared<Queue>(std::move(runtime));
        const Status status=convert(openEventQueue(attachment,mask,capacity,filter,queue->native));
        if (status!=OVPHYSX_COMPANION_OK) return status;
        std::lock_guard<std::mutex> lock(companionRuntime().eventQueues().mutex);
        if (companionRuntime().eventQueues().nextHandle==UINT64_MAX) return OVPHYSX_COMPANION_ERROR;
        const uint64_t handle=companionRuntime().eventQueues().nextHandle++;companionRuntime().eventQueues().entries.emplace(handle,std::move(queue));*output=handle;
        return OVPHYSX_COMPANION_OK;
    } catch (...) {return OVPHYSX_COMPANION_ERROR;}
}
template<class Fn> Status withQueue(uint64_t handle,Fn fn) noexcept {
    try {
        CompanionOperationGuard admission(companionRuntime());
        if (!admission) return OVPHYSX_COMPANION_BUSY;
        std::shared_ptr<Queue> queue;
        {std::lock_guard<std::mutex> lock(companionRuntime().eventQueues().mutex);
         std::map<uint64_t, std::shared_ptr<Queue>>::iterator found=companionRuntime().eventQueues().entries.find(handle);if(found==companionRuntime().eventQueues().entries.end())return OVPHYSX_COMPANION_NOT_FOUND;queue=found->second;}
        std::unique_lock<std::mutex> lock(queue->mutex,std::try_to_lock);
        if (!lock.owns_lock())return OVPHYSX_COMPANION_BUSY;
        if (queue->closed)return OVPHYSX_COMPANION_NOT_FOUND;
        return fn(*queue);
    }catch(...){return OVPHYSX_COMPANION_ERROR;}
}
Status poll(uint64_t handle,uint32_t eventSize,ovphysx_companion_event_t* output,uint32_t capacity,
            uint32_t batchSize,ovphysx_companion_event_batch_t* batch) noexcept {
    if (!output || !batch || !capacity || capacity>65536 || eventSize!=sizeof(*output) || batchSize!=sizeof(*batch))
        return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    return withQueue(handle,[&](Queue& queue)->Status {
        std::vector<QueuedEvent> events(capacity);EventBatch info;
        Status status=convert(pollEventQueue(queue.native,events.data(),capacity,info));
        if(status!=OVPHYSX_COMPANION_OK)return status;
        static_assert(sizeof(QueuedEvent)==sizeof(*output));
        static_assert(offsetof(QueuedEvent,message)==offsetof(ovphysx_companion_event_t,message));
        static_assert(sizeof(EventBatch)==sizeof(*batch));
        if(info.count)std::memcpy(output,events.data(),sizeof(*output)*info.count);
        std::memcpy(batch,&info,sizeof(*batch));
        if (info.overflow) return OVPHYSX_COMPANION_EVENT_HISTORY_LOST;
        return OVPHYSX_COMPANION_OK;
    });
}
Status close(uint64_t handle) noexcept {
    return withQueue(handle,[&](Queue& queue)->Status {
        const Status status=convert(closeEventQueue(queue.native));
        if(status!=OVPHYSX_COMPANION_OK)return status;
        queue.closed=true;
        std::lock_guard<std::mutex> lock(companionRuntime().eventQueues().mutex);companionRuntime().eventQueues().entries.erase(handle);
        return OVPHYSX_COMPANION_OK;
    });
}
}
using namespace ovphysx::companion::events;
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_events_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_events_api_t* output) {
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if (version!=1 || size!=sizeof(*output)) return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output={sizeof(*output),1,127,open,poll,close};return OVPHYSX_COMPANION_OK;
}
