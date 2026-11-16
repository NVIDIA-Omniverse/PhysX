// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
#include "Events.h"
#include "host.h"
#include <carb/events/IEvents.h>
#include <carb/Framework.h>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

static_assert(carb_events_IEvents == CARB_HEXVERSION(1, 4), "The static provider implements IEvents 1.4");

namespace
{
using namespace carb::events;

class Event final : public IEvent
{
public:
    Event(EventType eventType, SenderId eventSender)
    {
        type = eventType;
        sender = eventSender;
        m_dictionary = carb::getCachedInterface<carb::dictionary::IDictionary>();
        if (!m_dictionary)
            throw std::runtime_error("Event payloads require IDictionary");
        payload = m_dictionary->createItem(nullptr, "", carb::dictionary::ItemType::eDictionary);
        if (!payload)
            throw std::bad_alloc();
    }
    ~Event() override { m_dictionary->destroyItem(payload); }
    size_t addRef() override { return m_references.fetch_add(1) + 1; }
    size_t release() override
    {
        const size_t remaining = m_references.fetch_sub(1) - 1;
        if (!remaining)
            delete this;
        return remaining;
    }
    void consume() override { consumed.store(true); }
    void internalAttachObject(const char* name, carb::IObject* object) override
    {
        if (!name)
            throw std::invalid_argument("Attached event object requires a name");
        carb::ObjectPtr<carb::IObject> previous;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            previous.swap(m_objects[name]);
            if (object)
                m_objects[name] = carb::borrowObject(object);
            else
                m_objects.erase(name);
        }
    }
    void internalRetrieveObject(const char* name, carb::ObjectPtr<carb::IObject>& out) override
    {
        if (!name)
            throw std::invalid_argument("Attached event object requires a name");
        carb::ObjectPtr<carb::IObject> result;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            const auto found = m_objects.find(name);
            if (found != m_objects.end())
                result = found->second;
        }
        out = std::move(result);
    }
    void getObjectNames(omni::vector<carb::RString>& out) const override
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_objects.empty())
            throw std::runtime_error("Event object-name enumeration requires the RString service");
        (void)out;
    }
    std::atomic<bool> consumed{false};
private:
    std::atomic<size_t> m_references{1};
    carb::dictionary::IDictionary* m_dictionary = nullptr;
    mutable std::mutex m_mutex;
    std::unordered_map<std::string, carb::ObjectPtr<carb::IObject>> m_objects;
};

class Subscription;
struct SubscriptionState
{
    std::mutex mutex;
    std::condition_variable drained;
    bool active = true;
    bool enabled = true;
    bool push = false;
    bool filtered = false;
    EventType type = 0;
    Order order = 0;
    uint64_t sequence = 0;
    size_t calls = 0;
    std::string name;
    carb::ObjectPtr<IEventListener> listener;
    Subscription* holder = nullptr;
};
thread_local std::vector<const SubscriptionState*> currentCallbacks;

void unsubscribe(const std::shared_ptr<SubscriptionState>& state)
{
    carb::ObjectPtr<IEventListener> listener;
    std::unique_lock<std::mutex> lock(state->mutex);
    state->active = false;
    // Callback-side unsubscribe must not wait on another callback that can be waiting on this one.
    if (currentCallbacks.empty())
        state->drained.wait(lock, [&] { return state->calls == 0; });
    if (!state->calls)
        listener.swap(state->listener);
}

class Subscription final : public ISubscription
{
public:
    explicit Subscription(std::shared_ptr<SubscriptionState> state) : m_state(std::move(state))
    {
        m_state->holder = this;
    }
    size_t addRef() override { return m_references.fetch_add(1) + 1; }
    bool tryRetain()
    {
        size_t references = m_references.load();
        while (references && !m_references.compare_exchange_weak(references, references + 1)) {}
        return references != 0;
    }
    size_t release() override
    {
        const size_t remaining = m_references.fetch_sub(1) - 1;
        if (!remaining)
        {
            {
                std::lock_guard<std::mutex> lock(m_state->mutex);
                m_state->holder = nullptr;
            }
            unsubscribe();
            delete this;
        }
        return remaining;
    }
    void unsubscribe() override { ::unsubscribe(m_state); }
    const char* getName() const noexcept override { return m_state->name.c_str(); }
    carb::cpp::zstring_view getNameS() const noexcept override { return carb::cpp::zstring_view(m_state->name); }
    Order getOrder() const noexcept override
    {
        std::lock_guard<std::mutex> lock(m_state->mutex);
        return m_state->order;
    }
    void setOrder(Order order) noexcept override
    {
        std::lock_guard<std::mutex> lock(m_state->mutex);
        m_state->order = order;
    }
    bool isEnabled() const noexcept override
    {
        std::lock_guard<std::mutex> lock(m_state->mutex);
        return m_state->enabled;
    }
    void setEnabled(bool enabled) noexcept override
    {
        std::lock_guard<std::mutex> lock(m_state->mutex);
        m_state->enabled = enabled;
    }
private:
    std::atomic<size_t> m_references{1};
    std::shared_ptr<SubscriptionState> m_state;
};

struct QueuedEvent
{
    explicit QueuedEvent(IEvent* value) : event(carb::borrowObject(value)) {}
    carb::ObjectPtr<IEvent> event;
    std::mutex mutex;
    std::condition_variable delivered;
    bool complete = false;
};

class Stream final : public IEventStream
{
public:
    explicit Stream(const char* name) : m_name(name ? name : "ovphysx.wasm.events") {}
    ~Stream() override
    {
        for (const std::shared_ptr<SubscriptionState>& state : snapshot())
            ::unsubscribe(state);
    }
    size_t addRef() override { return m_references.fetch_add(1) + 1; }
    size_t release() override
    {
        const size_t remaining = m_references.fetch_sub(1) - 1;
        if (!remaining)
            delete this;
        return remaining;
    }
    IEvent* createEventPtr(EventType type, SenderId sender) override { return new Event(type, sender); }
    void dispatch(IEvent* event) override
    {
        carb::ObjectPtr<IEventStream> keep = carb::borrowObject<IEventStream>(this);
        deliver(event, false);
    }
    void push(IEvent* event) override
    {
        carb::ObjectPtr<IEventStream> keep = carb::borrowObject<IEventStream>(this);
        enqueue(event);
    }
    void pushBlocked(IEvent* event) override
    {
        carb::ObjectPtr<IEventStream> keep = carb::borrowObject<IEventStream>(this);
        const std::shared_ptr<QueuedEvent> queued = enqueue(event);
        std::unique_lock<std::mutex> lock(queued->mutex);
        queued->delivered.wait(lock, [&] { return queued->complete; });
    }
    size_t getCount() override
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.size();
    }
    IEvent* popPtr() override
    {
        carb::ObjectPtr<IEventStream> keep = carb::borrowObject<IEventStream>(this);
        return pop(true);
    }
    IEvent* tryPopPtr() override
    {
        carb::ObjectPtr<IEventStream> keep = carb::borrowObject<IEventStream>(this);
        return pop(false);
    }
    ISubscription* internalCreateSubscriptionToPopPtr(IEventListener* listener, Order order, const char* name) override
    { return subscribe(listener, order, name, false, false, 0); }
    ISubscription* internalCreateSubscriptionToPopByTypePtr(EventType type, IEventListener* listener, Order order,
                                                           const char* name) override
    { return subscribe(listener, order, name, false, true, type); }
    ISubscription* internalCreateSubscriptionToPushPtr(IEventListener* listener, Order order, const char* name) override
    { return subscribe(listener, order, name, true, false, 0); }
    ISubscription* internalCreateSubscriptionToPushByTypePtr(EventType type, IEventListener* listener, Order order,
                                                            const char* name) override
    { return subscribe(listener, order, name, true, true, type); }
    bool internalSetSubscriptionToPopOrder(const char* name, Order order) override
    { return namedOrder(name, &order, false, true); }
    bool internalSetSubscriptionToPushOrder(const char* name, Order order) override
    { return namedOrder(name, &order, true, true); }
    bool internalGetSubscriptionToPopOrder(const char* name, Order* order) override
    { return namedOrder(name, order, false, false); }
    bool internalGetSubscriptionToPushOrder(const char* name, Order* order) override
    { return namedOrder(name, order, true, false); }
    const char* getName() const noexcept override { return m_name.c_str(); }
    size_t getSubscriptionToPopCount() const override { return count(false); }
    size_t getSubscriptionToPushCount() const override { return count(true); }
    bool walkSubscriptionsToPop(SubscriptionCallback callback, void* data) const override
    { return walk(callback, data, false); }
    bool walkSubscriptionsToPush(SubscriptionCallback callback, void* data) const override
    { return walk(callback, data, true); }
private:
    std::vector<std::shared_ptr<SubscriptionState>> snapshot() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::vector<std::shared_ptr<SubscriptionState>> result;
        for (const std::weak_ptr<SubscriptionState>& weak : m_subscriptions)
            if (std::shared_ptr<SubscriptionState> state = weak.lock())
                result.push_back(std::move(state));
        return result;
    }
    ISubscription* subscribe(IEventListener* listener, Order order, const char* name, bool push, bool filtered, EventType type)
    {
        if (!listener)
            throw std::invalid_argument("Event subscription requires a listener");
        std::shared_ptr<SubscriptionState> state = std::make_shared<SubscriptionState>();
        state->listener = carb::borrowObject(listener);
        state->order = order;
        state->push = push;
        state->filtered = filtered;
        state->type = type;
        std::lock_guard<std::mutex> lock(m_mutex);
        state->sequence = ++m_nextSubscription;
        state->name = name ? name : "subscription-" + std::to_string(state->sequence);
        m_subscriptions.erase(std::remove_if(m_subscriptions.begin(), m_subscriptions.end(),
            [](const std::weak_ptr<SubscriptionState>& value) { return value.expired(); }), m_subscriptions.end());
        Subscription* holder = new Subscription(state);
        m_subscriptions.push_back(state);
        return holder;
    }
    void deliver(IEvent* raw, bool push)
    {
        Event* event = dynamic_cast<Event*>(raw);
        if (!event)
            throw std::invalid_argument("Static event provider requires its own event object");
        carb::ObjectPtr<IEvent> keep = carb::borrowObject(raw);
        if (!push)
            event->consumed.store(false);
        struct Candidate { Order order; uint64_t sequence; std::shared_ptr<SubscriptionState> state; };
        std::vector<Candidate> candidates;
        for (const std::shared_ptr<SubscriptionState>& state : snapshot())
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            if (state->active && state->enabled && state->push == push && (!state->filtered || state->type == event->type))
                candidates.push_back({state->order, state->sequence, state});
        }
        std::sort(candidates.begin(), candidates.end(), [](const Candidate& left, const Candidate& right) {
            return left.order < right.order || (left.order == right.order && left.sequence < right.sequence);
        });
        for (const Candidate& candidate : candidates)
        {
            const std::shared_ptr<SubscriptionState>& state = candidate.state;
            carb::ObjectPtr<IEventListener> listener;
            {
                std::lock_guard<std::mutex> lock(state->mutex);
                if (!state->active || !state->enabled)
                    continue;
                listener = state->listener;
                ++state->calls;
            }
            currentCallbacks.push_back(state.get());
            const auto complete = [&] {
                currentCallbacks.pop_back();
                carb::ObjectPtr<IEventListener> retired;
                std::lock_guard<std::mutex> lock(state->mutex);
                --state->calls;
                if (!state->active && !state->calls)
                    retired.swap(state->listener);
                state->drained.notify_all();
            };
            try { listener->onEvent(event); }
            catch (...) { complete(); throw; }
            complete();
            if (!push && event->consumed.load())
                break;
        }
    }
    std::shared_ptr<QueuedEvent> enqueue(IEvent* event)
    {
        if (!dynamic_cast<Event*>(event))
            throw std::invalid_argument("Static event provider requires its own event object");
        std::shared_ptr<QueuedEvent> queued = std::make_shared<QueuedEvent>(event);
        deliver(event, true);
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_queue.push_back(queued);
        }
        m_ready.notify_one();
        return queued;
    }
    IEvent* pop(bool wait)
    {
        std::shared_ptr<QueuedEvent> queued;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            if (wait)
                m_ready.wait(lock, [&] { return !m_queue.empty(); });
            if (m_queue.empty())
                return nullptr;
            queued = std::move(m_queue.front());
            m_queue.pop_front();
        }
        const auto complete = [&] {
            std::lock_guard<std::mutex> lock(queued->mutex);
            queued->complete = true;
            queued->delivered.notify_all();
        };
        try { deliver(queued->event.get(), false); }
        catch (...) { complete(); throw; }
        complete();
        IEvent* result = queued->event.get();
        result->addRef();
        return result;
    }
    bool namedOrder(const char* name, Order* order, bool push, bool write)
    {
        if (!name || !order)
            return false;
        bool found = false;
        for (const std::shared_ptr<SubscriptionState>& state : snapshot())
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            if (state->active && state->push == push && state->name == name)
            {
                if (write)
                    state->order = *order;
                else
                    *order = state->order;
                found = true;
                if (!write)
                    break;
            }
        }
        return found;
    }
    size_t count(bool push) const
    {
        size_t result = 0;
        for (const std::shared_ptr<SubscriptionState>& state : snapshot())
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            if (state->active && state->push == push)
                ++result;
        }
        return result;
    }
    bool walk(SubscriptionCallback callback, void* data, bool push) const
    {
        carb::ObjectPtr<IEventStream> keep = carb::borrowObject<IEventStream>(const_cast<Stream*>(this));
        if (!callback)
            return false;
        for (const std::shared_ptr<SubscriptionState>& state : snapshot())
        {
            carb::ObjectPtr<ISubscription> holder;
            {
                std::lock_guard<std::mutex> lock(state->mutex);
                if (state->active && state->push == push && state->holder && state->holder->tryRetain())
                    holder = carb::stealObject<ISubscription>(state->holder);
            }
            if (holder && !callback(holder.get(), data))
                return false;
        }
        return true;
    }
    std::atomic<size_t> m_references{1};
    std::string m_name;
    mutable std::mutex m_mutex;
    std::condition_variable m_ready;
    uint64_t m_nextSubscription = 0;
    std::vector<std::weak_ptr<SubscriptionState>> m_subscriptions;
    std::deque<std::shared_ptr<QueuedEvent>> m_queue;
};

class Events final : public IEvents
{
public:
    IEventStream* createEventStreamPtr() override { return new Stream(nullptr); }
    IEventStream* internalCreateEventStream(const char* name) noexcept override
    {
        try { return new Stream(name); }
        catch (...) { return nullptr; }
    }
    SenderId acquireUniqueSenderId() override
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        for (uint64_t attempts = 0; attempts < std::numeric_limits<SenderId>::max(); ++attempts)
        {
            if (++m_nextSender == kGlobalSenderId)
                ++m_nextSender;
            if (m_senders.insert(m_nextSender).second)
                return m_nextSender;
        }
        throw std::overflow_error("No event sender IDs remain");
    }
    void releaseUniqueSenderId(SenderId sender) override
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_senders.erase(sender);
    }
private:
    std::mutex m_mutex;
    SenderId m_nextSender = 0;
    std::unordered_set<SenderId> m_senders;
};
} // namespace

void ovphysxWasmRegisterEvents()
{
    static Events events;
    static std::mutex registrationMutex;
    std::lock_guard<std::mutex> lock(registrationMutex);
    carb::Framework* framework = carb::getFramework();
    if (!framework)
        throw std::runtime_error("Event registration requires an initialized static host");
    if (IEvents* existing = framework->tryAcquireExistingInterface<IEvents>())
    {
        framework->releaseInterface(existing);
        return;
    }
    wasmFabricHostRegisterInterface(IEvents::getInterfaceDesc(), &events, "ovphysx.wasm.events");
}
