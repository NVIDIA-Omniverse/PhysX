// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-EVENT-QUEUE-001
// @covers AC-1 AC-2 AC-3 AC-4
#include <omni/physx/NativeEvents.h>
#include <omni/physx/PhysXRuntime.h>
#include <carb/events/EventsUtils.h>
#include <carb/dictionary/IDictionary.h>
#include "RuntimeExecution.h"
#include "OmniPhysX.h"
#include "PhysXScene.h"
#include "Trigger.h"
#include "usdLoad/LoadUsd.h"
#include "usdLoad/AttachedStage.h"
#include "internal/InternalPhysXDatabase.h"
#include <algorithm>
#include <cstring>
#include <map>
#include <tuple>
#include <mutex>
#include <vector>
namespace omni::physx::native
{
class EventQueue
{
public:
    uint64_t attachment=0, filter=0;
    uint32_t mask=0;
    std::weak_ptr<usdparser::AttachedStage> owner;
    std::mutex mutex;
    std::vector<QueuedEvent> ring;
    uint32_t head=0, count=0;
    uint64_t sequence=0, dropped=0, substep=0, pairSequence=0, lastStayStep=UINT64_MAX;
    bool closed=false, overflow=false;
    struct TriggerMembership { QueuedEvent event; uint64_t count=0; };
    std::map<std::tuple<uint64_t,uint64_t,uint32_t,uint32_t>,TriggerMembership> triggers;
    SubscriptionId pre=kInvalidSubscriptionId, post=kInvalidSubscriptionId,
        status=kInvalidSubscriptionId, contact=kInvalidSubscriptionId,
        trigger=kInvalidSubscriptionId, objects=kInvalidSubscriptionId;
    carb::events::ISubscriptionPtr errors;

    QueuedEvent base(EventKind kind, uint32_t phase)
    {
        QueuedEvent event;
        event.attachment=attachment; event.kind=uint32_t(kind); event.phase=phase;
        event.substep=substep;
        if (std::shared_ptr<omni::physx::usdparser::AttachedStage> stage=owner.lock())
        {
            event.step=stage->simulationTimestamp();
            event.generation=stage->recordVersion.load(std::memory_order_relaxed);
        }
        return event;
    }
    // Caller holds mutex. Fail-stop on overflow: retained prefix is diagnostic
    // only; pair state is invalid until the host closes/recreates and resubscribes.
    void push(QueuedEvent event)
    {
        if (closed) return;
        event.sequence=++sequence;
        if (overflow || count==ring.size())
        {
            overflow=true; ++dropped; triggers.clear(); return;
        }
        ring[(head+count)%ring.size()]=event; ++count;
    }
    void onStep(float dt, bool before)
    {
        std::lock_guard<std::mutex> lock(mutex);
        omni::physx::native::QueuedEvent event=base(before ? EventKind::PreStep : EventKind::PostStep,before ? 0 : 2);
        event.dt=dt;
        if (before) ++event.step;
        if (mask & (before ? 1u : 2u)) push(event);
    }
    void onStatus(SimulationStatusEvent type)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (closed) return;
        if (type==eSimulationStarting) ++substep;
        const omni::physx::native::EventKind kind=type==eSimulationStarting ? EventKind::SubstepBegin :
            type==eSimulationEnded ? EventKind::SubstepEnd : EventKind::SimulationComplete;
        omni::physx::native::QueuedEvent event=base(kind,type==eSimulationComplete ? 2 : 1);
        if (type==eSimulationStarting) ++event.step;
        if (mask & 4u) push(event);
        // Stay is derived only for pairs whose enter this subscription observed.
        // It is emitted on complete, after native leave callbacks removed pairs.
        if (type==eSimulationComplete && !overflow && lastStayStep!=event.step)
        {
            lastStayStep=event.step;
            for (const std::pair<const std::tuple<uint64_t, uint64_t, unsigned int, unsigned int>, TriggerMembership> & item:triggers)
            {
                omni::physx::native::QueuedEvent stay=item.second.event;
                if (stay.step==event.step) continue; // enter already covers this completion
                stay.kind=uint32_t(EventKind::TriggerStay); stay.phase=2;
                stay.step=event.step; stay.substep=substep; stay.generation=event.generation;
                push(stay);
                if (overflow) break;
            }
        }
    }
    void onContact(const ContactEventHeader* headers,uint32_t n,const ContactData* points,uint32_t np)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (closed) return;
        for (uint32_t i=0;i<n;++i)
        {
            const omni::physx::ContactEventHeader & h=headers[i];
            if (h.attachHandle!=attachment || (filter && h.actor0!=filter && h.actor1!=filter && h.collider0!=filter && h.collider1!=filter)) continue;
            omni::physx::native::QueuedEvent event=base(EventKind::Contact,2);
            event.type=uint32_t(h.type);event.actor0=h.actor0;event.actor1=h.actor1;
            event.collider0=h.collider0;event.collider1=h.collider1;
            event.instance0=h.protoIndex0;event.instance1=h.protoIndex1;
            event.pointCount=h.numContactData;event.pairSequence=++pairSequence;
            if (h.contactDataOffset>np || h.numContactData>np-h.contactDataOffset)
            { overflow=true; ++dropped; triggers.clear(); continue; }
            if (!h.numContactData) push(event);
            for (uint32_t j=0;j<h.numContactData;++j)
            {
                const omni::physx::ContactData & p=points[h.contactDataOffset+j];
                event.pointIndex=j;event.separation=p.separation;
                event.position[0]=p.position.x;event.position[1]=p.position.y;event.position[2]=p.position.z;
                event.normal[0]=p.normal.x;event.normal[1]=p.normal.y;event.normal[2]=p.normal.z;
                event.impulse[0]=p.impulse.x;event.impulse[1]=p.impulse.y;event.impulse[2]=p.impulse.z;
                event.material0=p.material0;event.material1=p.material1;event.face0=p.faceIndex0;event.face1=p.faceIndex1;
                push(event);
            }
        }
    }
    void onTrigger(const TriggerEventData* data,uint32_t instance0,uint32_t instance1)
    {
        if (!data || data->attachHandle!=attachment) return;
        std::lock_guard<std::mutex> lock(mutex);
        if (closed || (filter && data->triggerColliderPrimKey.handle!=filter && data->otherColliderPrimKey.handle!=filter)) return;
        omni::physx::native::QueuedEvent event=base(EventKind::Trigger,2);
        event.type=uint32_t(data->eventType);
        event.actor0=data->triggerBodyPrimKey.handle;event.actor1=data->otherBodyPrimKey.handle;
        event.collider0=data->triggerColliderPrimKey.handle;event.collider1=data->otherColliderPrimKey.handle;
        event.instance0=instance0;event.instance1=instance1;
        const std::tuple<uint64_t, uint64_t, unsigned int, unsigned int> key=std::make_tuple(event.collider0,event.collider1,instance0,instance1);
        if (data->eventType==TriggerEventType::eTRIGGER_ON_LEAVE) {
            const std::map<std::tuple<uint64_t,uint64_t,uint32_t,uint32_t>,TriggerMembership>::iterator found=triggers.find(key);
            if (found!=triggers.end() && --found->second.count==0) triggers.erase(found);
        }
        else if (!overflow)
        {
            if (triggers.size()>=ring.size() && !triggers.count(key))
            { overflow=true; triggers.clear(); }
            else { TriggerMembership& membership=triggers[key]; membership.event=event; ++membership.count; }
        }
        push(event);
    }
    void onObject(const PhysicsObjectChangeEvent& data,bool create)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (closed) return;
        omni::physx::native::QueuedEvent event=base(create ? EventKind::ObjectCreated : EventKind::ObjectRemoved,3);
        event.type=uint32_t(data.type);event.collider0=data.key.handle;event.objectId=uint64_t(data.objectId);
        if (!create)
        {
            for (std::map<std::tuple<uint64_t, uint64_t, unsigned int, unsigned int>, TriggerMembership>::iterator it=triggers.begin();it!=triggers.end();)
            {
                if (std::get<0>(it->first)==data.key.handle || std::get<1>(it->first)==data.key.handle) it=triggers.erase(it);
                else ++it;
            }
        }
        if (mask&32u) push(event);
    }
    void onReset()
    {
        std::lock_guard<std::mutex> lock(mutex);
        triggers.clear();
        // Always publish reset for trigger consumers; it invalidates pair state.
        if (mask&(16u|32u)) push(base(EventKind::ObjectsReset,3));
    }
};

namespace
{
template<class Fn> void deliver(const std::weak_ptr<EventQueue>& weak, Fn fn) noexcept
{
    if (std::shared_ptr<omni::physx::native::EventQueue> queue=weak.lock())
    {
        try { fn(*queue); }
        catch (...)
        {
            // An allocation failure while tracking trigger state is event loss,
            // not a log-only callback exception that a consumer could miss.
            std::lock_guard<std::mutex> lock(queue->mutex);
            if (!queue->closed) {queue->overflow=true;++queue->dropped;queue->triggers.clear();}
        }
    }
}
}

EventResult closeEventQueue(const std::shared_ptr<EventQueue>& queue) noexcept
try
{
    if (!queue) return EventResult::Invalid;
    {
        std::lock_guard<std::mutex> lock(queue->mutex);
        if (queue->closed) return EventResult::Success;
        queue->closed=true;queue->count=0;queue->triggers.clear();
    }
    // Metadata cancellation supports in-flight dispatch. Registry snapshots own
    // std::function closures with weak state; an admitted callback cannot use a
    // released raw userdata pointer or enqueue after the closed barrier.
    if (std::shared_ptr<omni::physx::usdparser::AttachedStage> owner=queue->owner.lock())
    {
        owner->getSimulationCallbacks().getContactReportRegistry().removeEvent(queue->contact);
        owner->getPhysXPhysicsInterface()->unsubscribeToObjectChangeNotifications(queue->objects);
    }
    if (runtime::tryGetPhysxInterface())
    {
        omni::physx::OmniPhysX & physics=OmniPhysX::getInstance();
        physics.removeOnStepEventSubscription(queue->pre);physics.removeOnStepEventSubscription(queue->post);
        physics.removeStatusEventSubscription(queue->status);
        physics.getTriggerManager()->unregisterNativeCallback(queue->trigger);
    }
    queue->errors=nullptr;
    return EventResult::Success;
}
catch (...) { return EventResult::Error; }

EventResult openEventQueue(uint64_t attachment,uint32_t mask,uint32_t capacity,const char* path,
                          std::shared_ptr<EventQueue>& output) noexcept
try
{
    if (!mask || (mask&~127u) || !capacity || capacity>65536 || (path && path[0]!='/')) return EventResult::Invalid;
    if (!runtime::tryGetPhysxInterface()) return EventResult::Stale;
    std::shared_ptr<omni::physx::usdparser::AttachedStage> owner=usdparser::UsdLoad::getUsdLoad()->acquireAttachment(attachment);
    if (!owner) return EventResult::Stale;
    RuntimeExecutionScope admission(attachment);
    if (!admission) return EventResult::Busy;
    if (usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachment)!=owner.get()) return EventResult::Stale;
    if (mask&(8u|16u))
        for (const std::pair<const uint64_t, omni::physx::PhysXScene *> & item:owner->getPhysXScenes())
        {
            const ::physx::PxScene * scene=item.second->getScene();
            if (scene && (scene->getFlags().isSet(::physx::PxSceneFlag::eENABLE_GPU_DYNAMICS) ||
                          scene->getFlags().isSet(::physx::PxSceneFlag::eENABLE_DIRECT_GPU_API)))
                return EventResult::Unavailable;
        }
    std::shared_ptr<omni::physx::native::EventQueue> queue=std::make_shared<EventQueue>();
    queue->attachment=attachment;queue->owner=owner;queue->mask=mask;queue->ring.resize(capacity);
    if (path)
    {
        queue->filter=owner->keyFor(path).handle;
        const std::vector<omni::physx::internal::InternalDatabase::Record> & records=owner->getInternalPhysXDatabase().getRecords();
        const bool live=std::any_of(records.begin(),records.end(),[&](const internal::InternalDatabase::Record& record) {
            return record.mKey.handle==queue->filter &&
                (record.mType==ePTActor || record.mType==ePTLink || record.mType==ePTShape || record.mType==ePTCompoundShape);
        });
        if (!queue->filter || !live) return EventResult::Invalid;
    }
    const std::weak_ptr<EventQueue> weak=queue;
    omni::physx::OmniPhysX & physics=OmniPhysX::getInstance();
    try
    {
        if (mask&1u) queue->pre=physics.addOnStepEventSubscription(attachment,
            [weak](float dt,void*) { deliver(weak,[&](EventQueue& q){q.onStep(dt,true);}); },nullptr,true,0);
        if (mask&2u) queue->post=physics.addOnStepEventSubscription(attachment,
            [weak](float dt,void*) { deliver(weak,[&](EventQueue& q){q.onStep(dt,false);}); },nullptr,false,0);
        if (mask&(4u|16u)) queue->status=physics.addStatusEventSubscription(attachment,
            [weak](SimulationStatusEvent type,void*) { deliver(weak,[&](EventQueue& q){q.onStatus(type);}); },nullptr);
        if (mask&8u) queue->contact=owner->getSimulationCallbacks().getContactReportRegistry().addEvent({
            [weak](const ContactEventHeader* h,uint32_t n,const ContactData* p,uint32_t np,void*) {
                deliver(weak,[&](EventQueue& q){q.onContact(h,n,p,np);});
            },nullptr});
        if (mask&16u) queue->trigger=physics.getTriggerManager()->registerNativeCallback({attachment,{},
            {},nullptr,[weak](const TriggerEventData* data,uint32_t i0,uint32_t i1) {
                deliver(weak,[&](EventQueue& q){q.onTrigger(data,i0,i1);});}});
        if (mask&(16u|32u))
        {
            IPhysicsObjectChangeCallback cb;
            cb.stopCallbackWhenSimStopped=false;cb.notifyInitialPopulation=true;
            cb.objectCreationNotifyFn=[weak](const PhysicsObjectChangeEvent& data,void*) {deliver(weak,[&](EventQueue& q){q.onObject(data,true);});};
            cb.objectDestructionNotifyFn=[weak](const PhysicsObjectChangeEvent& data,void*) {deliver(weak,[&](EventQueue& q){q.onObject(data,false);});};
            cb.allObjectsDestructionNotifyFn=[weak](AttachHandle,void*) {deliver(weak,[&](EventQueue& q){q.onReset();});};
            queue->objects=owner->getPhysXPhysicsInterface()->subscribeToObjectChangeNotifications(cb,attachment);
        }
        if (mask&64u) queue->errors=carb::events::createSubscriptionToPush(physics.getErrorEventStream().get(),
            [weak](carb::events::IEvent* data) {
                if (data->type<eUsdLoadError || data->type>ePhysxTooManyErrors) return;
                if (std::shared_ptr<omni::physx::native::EventQueue> q=weak.lock())
                {
                    carb::dictionary::IDictionary * dictionary=carb::getCachedInterface<carb::dictionary::IDictionary>();
                    const carb::cpp::optional<carb::cpp::basic_zstring_view<char>> message=dictionary->getStringView(dictionary->getItem(data->payload,"errorString"));
                    std::lock_guard<std::mutex> lock(q->mutex);
                    omni::physx::native::QueuedEvent event=q->base(EventKind::RuntimeError,4);
                    // The legacy stream is process-scoped. Never attribute an
                    // unqualified SDK error to this queue's selected attachment.
                    event.attachment=0;event.generation=0;event.step=0;event.substep=0;event.type=uint32_t(data->type);
                    if (message)
                    {
                        const size_t n=std::min(message->size(),sizeof(event.message)-1);
                        std::memcpy(event.message,message->data(),n);event.flags=message->size()>n ? 1u : 0u;
                    }
                    q->push(event);
                }
            });
        if (((mask&1u) && queue->pre==kInvalidSubscriptionId) ||
            ((mask&2u) && queue->post==kInvalidSubscriptionId) ||
            ((mask&(4u|16u)) && queue->status==kInvalidSubscriptionId) ||
            ((mask&8u) && queue->contact==kInvalidSubscriptionId) ||
            ((mask&16u) && queue->trigger==kInvalidSubscriptionId) ||
            ((mask&(16u|32u)) && queue->objects==kInvalidSubscriptionId) ||
            ((mask&64u) && !queue->errors))
        { closeEventQueue(queue);return EventResult::Error; }
    }
    catch (...) {closeEventQueue(queue);throw;}
    output=std::move(queue);return EventResult::Success;
}
catch (...) {return EventResult::Error;}

EventResult pollEventQueue(const std::shared_ptr<EventQueue>& queue,QueuedEvent* events,uint32_t capacity,EventBatch& output) noexcept
try
{
    if (!queue || !events || !capacity || capacity>65536) return EventResult::Invalid;
    // Poll cannot invoke callbacks or cause task completion. An uncollected owner
    // remains Busy even after SDK worker completion, until the host fetches it.
    RuntimeExecutionScope admission(queue->attachment);
    if (!admission) return EventResult::Busy;
    const std::shared_ptr<omni::physx::usdparser::AttachedStage> owner=usdparser::UsdLoad::getUsdLoad()->acquireAttachment(queue->attachment);
    std::lock_guard<std::mutex> lock(queue->mutex);
    if (queue->closed) return EventResult::Stale;
    EventBatch result;
    result.attachment=queue->attachment;result.dropped=queue->dropped;
    result.ownerLive=owner ? 1u : 0u;result.completedStep=owner ? owner->simulationTimestamp() : 0;
    result.count=std::min(capacity,queue->count);result.overflow=queue->overflow ? 1 : 0;
    for (uint32_t i=0;i<result.count;++i) events[i]=queue->ring[(queue->head+i)%queue->ring.size()];
    queue->head=(queue->head+result.count)%queue->ring.size();queue->count-=result.count;
    output=result;return EventResult::Success;
}
catch (...) {return EventResult::Error;}
}
