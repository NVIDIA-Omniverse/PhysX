// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
#include "Events.h"
#include "host.h"
#include <carb/Framework.h>
#include <carb/settings/ISettings.h>
#include <carb/events/IEvents.h>
#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

CARB_FRAMEWORK_GLOBALS("ovphysx.wasm.events-test")
#define CHECK(value) do { if (!(value)) { std::fprintf(stderr, "check failed at %d: %s\n", __LINE__, #value); return 1; } } while (false)
using namespace carb::events;
class Listener final : public IEventListener
{
public:
    explicit Listener(std::function<void(IEvent*)> callback) : callback(std::move(callback)) {}
    size_t addRef() override { return references.fetch_add(1) + 1; }
    size_t release() override
    {
        const size_t left = references.fetch_sub(1) - 1;
        if (!left) delete this;
        return left;
    }
    void onEvent(IEvent* event) override { callback(event); }
    std::function<void(IEvent*)> callback;
private:
    std::atomic<size_t> references{1};
};
carb::ObjectPtr<Listener> listen(std::function<void(IEvent*)> callback)
{
    return carb::stealObject(new Listener(std::move(callback)));
}

int main()
{
    g_carbFramework = wasmFabricFramework();
    ovphysxWasmRegisterEvents();
    ovphysxWasmRegisterEvents();
    IEvents* events = carb::getCachedInterface<IEvents>();
    CHECK(events);
    carb::dictionary::IDictionary* dictionary = carb::getCachedInterface<carb::dictionary::IDictionary>();
    carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
    settings->setDefaultStringArray("/physics/test/empty", nullptr, 0);
    CHECK(settings->getItemType("/physics/test/empty") == carb::dictionary::ItemType::eDictionary);
    CHECK(settings->getArrayLength("/physics/test/empty") == 0);
    CHECK(!settings->isAccessibleAsArray("/physics/test/empty"));
    const char* arrayValues[] = {"first", "second"};
    settings->setStringArray("/physics/test/array", arrayValues, 2);
    settings->setDefaultStringArray("/physics/test/array", nullptr, 0);
    CHECK(settings->getArrayLength("/physics/test/array") == 2);
    CHECK(settings->isAccessibleAsArrayOf(carb::dictionary::ItemType::eString, "/physics/test/array"));
    CHECK(std::string(settings->getStringBufferAt("/physics/test/array", 1)) == "second");
    settings->setStringArray("/physics/test/array", arrayValues, 1);
    CHECK(settings->getArrayLength("/physics/test/array") == 1);
    CHECK(!settings->getStringBufferAt("/physics/test/array", 1));
    IEventStreamPtr stream = events->createEventStream("contract-test");
    CHECK(stream && stream->getCount() == 0 && !stream->tryPop());
    IEventPtr event = stream->createEvent(7, kGlobalSenderId);
    dictionary->makeInt64AtPath(event->payload, "attachHandle", 1234567890123456LL);
    dictionary->makeStringAtPath(event->payload, "error/message", "actual payload");
    CHECK(dictionary->getAsInt64(dictionary->getItem(event->payload, "attachHandle")) == 1234567890123456LL);
    CHECK(std::string(dictionary->getStringBuffer(dictionary->getItem(event->payload, "error/message"))) == "actual payload");
    CHECK(dictionary->getItemChildCount(event->payload) == 2);
    CHECK(dictionary->getItemParent(dictionary->getItem(event->payload, "error/message")) == dictionary->getItem(event->payload, "error"));
    dictionary->makeBoolAtPath(event->payload, "error", true);
    CHECK(!dictionary->getItem(event->payload, "error/message"));

    const char* attachedName = "listener";
    bool attachedCalled = false;
    carb::ObjectPtr<Listener> attached = listen([&](IEvent*) { attachedCalled = true; });
    event->internalAttachObject("listener", attached.get());
    attached.reset();
    carb::ObjectPtr<carb::IObject> retained = event->retrieveObject(attachedName);
    CHECK(retained);
    static_cast<IEventListener*>(retained.get())->onEvent(event.get());
    CHECK(attachedCalled);
    event->internalAttachObject("listener", nullptr);
    CHECK(!event->retrieveObject(attachedName));
    retained.reset();

    std::vector<int> seen;
    carb::ObjectPtr<Listener> early = listen([&](IEvent* value) { seen.push_back(1); value->consume(); });
    carb::ObjectPtr<Listener> late = listen([&](IEvent*) { seen.push_back(2); });
    ISubscriptionPtr pushEarly = stream->createSubscriptionToPush(early.get(), -1, "push-early");
    ISubscriptionPtr pushLate = stream->createSubscriptionToPush(late.get(), 1, "push-late");
    ISubscriptionPtr popLate = stream->createSubscriptionToPop(late.get(), 1, "pop-late");
    ISubscriptionPtr popEarly = stream->createSubscriptionToPopByType(7, early.get(), -1, "pop-early");
    stream->push(event.get());
    CHECK((seen == std::vector<int>{1, 2}) && stream->getCount() == 1);
    seen.clear();
    IEventPtr popped = stream->tryPop();
    CHECK(popped.get() == event.get() && (seen == std::vector<int>{1}));
    popEarly->setEnabled(false);
    seen.clear();
    stream->dispatch(event.get());
    CHECK((seen == std::vector<int>{2}));
    popEarly->setEnabled(true);
    seen.clear();
    IEventPtr different = stream->createEvent(8, 0);
    stream->dispatch(different.get());
    CHECK((seen == std::vector<int>{2}));
    CHECK(stream->getSubscriptionToPushCount() == 2 && stream->getSubscriptionToPopCount() == 2);
    popEarly->unsubscribe();
    popLate.reset();
    CHECK(stream->getSubscriptionToPopCount() == 0);
    pushEarly.reset(); pushLate.reset();

    int selfCalls = 0, removedCalls = 0, addedCalls = 0;
    ISubscriptionPtr self, removed, added;
    carb::ObjectPtr<Listener> newListener = listen([&](IEvent*) { ++addedCalls; });
    carb::ObjectPtr<Listener> removedListener = listen([&](IEvent*) { ++removedCalls; });
    carb::ObjectPtr<Listener> selfListener = listen([&](IEvent*) {
        ++selfCalls;
        self->unsubscribe();
        self.reset();
        removed->unsubscribe();
        added = stream->createSubscriptionToPop(newListener.get(), 3, "added");
    });
    self = stream->createSubscriptionToPop(selfListener.get(), 0, "self");
    removed = stream->createSubscriptionToPop(removedListener.get(), 1, "removed");
    stream->dispatch(event.get());
    CHECK(selfCalls == 1 && removedCalls == 0 && addedCalls == 0);
    stream->dispatch(event.get());
    CHECK(selfCalls == 1 && removedCalls == 0 && addedCalls == 1);
    added.reset(); removed.reset();

    std::mutex gate;
    std::condition_variable condition;
    bool entered = false, releaseCallback = false;
    std::atomic<bool> unsubscribeStarted{false}, unsubscribeDone{false}, callbackActive{false}, lifetimeSafe{false};
    carb::ObjectPtr<Listener> blocking = listen([&](IEvent*) {
        std::unique_lock<std::mutex> lock(gate);
        callbackActive = true;
        entered = true;
        condition.notify_all();
        condition.wait(lock, [&] { return releaseCallback; });
        callbackActive = false;
    });
    ISubscriptionPtr blockingSubscription = stream->createSubscriptionToPop(blocking.get(), 0, "blocking");
    std::thread dispatchThread([&] { stream->dispatch(event.get()); });
    {
        std::unique_lock<std::mutex> lock(gate);
        condition.wait(lock, [&] { return entered; });
    }
    std::thread removeThread([&] {
        unsubscribeStarted = true;
        blockingSubscription->unsubscribe();
        lifetimeSafe = !callbackActive;
        unsubscribeDone = true;
    });
    while (!unsubscribeStarted) std::this_thread::yield();
    CHECK(!unsubscribeDone);
    {
        std::lock_guard<std::mutex> lock(gate);
        releaseCallback = true;
    }
    condition.notify_all();
    dispatchThread.join(); removeThread.join();
    CHECK(unsubscribeDone && lifetimeSafe);
    blockingSubscription.reset();

    std::atomic<bool> delivered{false}, producerFinished{false};
    carb::ObjectPtr<Listener> delivery = listen([&](IEvent*) { delivered = true; });
    ISubscriptionPtr deliverySubscription = stream->createSubscriptionToPop(delivery.get(), 0, "delivery");
    std::thread producer([&] { stream->pushBlocked(event.get()); producerFinished = delivered.load(); });
    while (!stream->getCount()) std::this_thread::yield();
    CHECK(!producerFinished);
    popped = stream->pop();
    producer.join();
    CHECK(producerFinished && delivered);
    deliverySubscription.reset();

    std::mutex rendezvousMutex;
    std::condition_variable rendezvous;
    unsigned concurrentCalls = 0;
    ISubscriptionPtr concurrent;
    carb::ObjectPtr<Listener> concurrentListener = listen([&](IEvent*) {
        {
            std::unique_lock<std::mutex> lock(rendezvousMutex);
            ++concurrentCalls;
            rendezvous.notify_all();
            rendezvous.wait(lock, [&] { return concurrentCalls == 2; });
        }
        concurrent->unsubscribe();
    });
    concurrent = stream->createSubscriptionToPop(concurrentListener.get(), 0, "concurrent-self-unsubscribe");
    std::thread firstDispatch([&] { stream->dispatch(event.get()); });
    std::thread secondDispatch([&] { stream->dispatch(different.get()); });
    firstDispatch.join(); secondDispatch.join();
    CHECK(concurrentCalls == 2 && stream->getSubscriptionToPopCount() == 0);
    concurrent.reset();

    IEventStreamPtr transient = events->createEventStream("release-during-push");
    IEventPtr transientEvent = transient->createEvent(1, 0);
    bool releasedDuringPush = false;
    carb::ObjectPtr<Listener> releaseOwner = listen([&](IEvent*) {
        transient.reset();
        releasedDuringPush = true;
    });
    ISubscriptionPtr ownerSubscription = transient->createSubscriptionToPush(releaseOwner.get(), 0, "release-owner");
    transient->push(transientEvent.get());
    CHECK(!transient && releasedDuringPush);
    ownerSubscription.reset(); transientEvent.reset();

    stream->push(event.get()); stream->push(different.get());
    CHECK(stream->getCount() == 2);
    CHECK(stream->pop()->type == 7 && stream->pop()->type == 8);
    const SenderId first = events->acquireUniqueSenderId(), second = events->acquireUniqueSenderId();
    CHECK(first != 0 && second != 0 && first != second);
    events->releaseUniqueSenderId(first); events->releaseUniqueSenderId(second);
    popped.reset(); event.reset(); different.reset(); stream.reset();
    wasmFabricHostReset();
    std::puts("PASS event payloads, FIFO, ordered filters, consumption, subscription changes, unsubscribe drain, blocking delivery");
    return 0;
}
