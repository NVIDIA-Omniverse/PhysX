// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#pragma once


#include "SimulationRegistry.h"
#include <omni/physics/simulation/IPhysicsStageUpdate.h>
#include <carb/events/IEvents.h>
#include <carb/tasking/TaskingUtils.h>

#include <mutex>
#include <unordered_map>
#include <vector>

namespace omni
{
namespace physics
{

// Subscription entry: pair of simulation ID and its subscription ID
struct SimulationSubscriptionEntry
{
    SimulationId simulationId;
    SubscriptionId subscriptionId;
};

// OmniPhysics class
class OmniPhysics
{
public:
    // Extension startup
    void onStartup();

    // Extension shutdown
    void onShutdown();

    // Get the global instance
    static OmniPhysics& getInstance();

    static void createOmniPhysicsInstance();

    // Get the simulation registry
    SimulationRegistry& getSimulationRegistry()
    {
        return mSimulationRegistry;
    }
    const SimulationRegistry& getSimulationRegistry() const
    {
        return mSimulationRegistry;
    }

    // Get simulation event stream
    carb::events::IEventStreamPtr getSimulationEventStream() const
    {
        return mSimulationEventStream;
    }

    // Send simulation event
    void sendSimulationEvent(SimulationEvent type);

    // sets simulation started value and returns previous value
    bool setSimulationStarted(bool val)
    {
        bool wasSimulationStopped;
        {
            std::unique_lock<carb::tasking::MutexWrapper> lock(mSubscriptionMutex);
            wasSimulationStopped = !mHasSimulationStarted;
            mHasSimulationStarted = val;
        }
        return wasSimulationStopped;
    }

    bool hasSimulationStarted() const
    {
        bool retVal = false;
        {
            std::unique_lock<carb::tasking::MutexWrapper> lock(mSubscriptionMutex);
            retVal = mHasSimulationStarted;
        }
        return retVal;
    }

    // Benchmark subscription management
    SubscriptionId addBenchmarkSubscription(std::vector<SimulationSubscriptionEntry>&& subscriptions)
    {
        std::unique_lock<carb::tasking::MutexWrapper> lock(mSubscriptionMutex);
        SubscriptionId internalId = mNextBenchmarkSubscriptionId++;
        mBenchmarkSubscriptionMap[internalId] = std::move(subscriptions);
        return internalId;
    }

    std::vector<SimulationSubscriptionEntry> removeBenchmarkSubscription(SubscriptionId subscriptionId)
    {
        std::unique_lock<carb::tasking::MutexWrapper> lock(mSubscriptionMutex);
        auto it = mBenchmarkSubscriptionMap.find(subscriptionId);
        if (it == mBenchmarkSubscriptionMap.end())
        {
            return {};
        }
        std::vector<SimulationSubscriptionEntry> result = std::move(it->second);
        mBenchmarkSubscriptionMap.erase(it);
        return result;
    }

    // Contact report subscription management
    SubscriptionId addContactReportSubscription(std::vector<SimulationSubscriptionEntry>&& subscriptions)
    {
        std::unique_lock<carb::tasking::MutexWrapper> lock(mSubscriptionMutex);
        SubscriptionId internalId = mNextContactReportSubscriptionId++;
        mContactReportSubscriptionMap[internalId] = std::move(subscriptions);
        return internalId;
    }

    std::vector<SimulationSubscriptionEntry> removeContactReportSubscription(SubscriptionId subscriptionId)
    {
        std::unique_lock<carb::tasking::MutexWrapper> lock(mSubscriptionMutex);
        auto it = mContactReportSubscriptionMap.find(subscriptionId);
        if (it == mContactReportSubscriptionMap.end())
        {
            return {};
        }
        std::vector<SimulationSubscriptionEntry> result = std::move(it->second);
        mContactReportSubscriptionMap.erase(it);
        return result;
    }

    // OnStep subscription management
    SubscriptionId addOnStepSubscription(std::vector<SimulationSubscriptionEntry>&& subscriptions)
    {
        std::unique_lock<carb::tasking::MutexWrapper> lock(mSubscriptionMutex);
        SubscriptionId internalId = mNextOnStepSubscriptionId++;
        mOnStepSubscriptionMap[internalId] = std::move(subscriptions);
        return internalId;
    }

    std::vector<SimulationSubscriptionEntry> removeOnStepSubscription(SubscriptionId subscriptionId)
    {
        std::unique_lock<carb::tasking::MutexWrapper> lock(mSubscriptionMutex);
        auto it = mOnStepSubscriptionMap.find(subscriptionId);
        if (it == mOnStepSubscriptionMap.end())
        {
            return {};
        }
        std::vector<SimulationSubscriptionEntry> result = std::move(it->second);
        mOnStepSubscriptionMap.erase(it);
        return result;
    }

private:
    // Simulation registry
    SimulationRegistry mSimulationRegistry;

    // Stores if simulation has started
    bool mHasSimulationStarted{ false };

    // Event stream for simulation events
    carb::events::IEventStreamPtr mSimulationEventStream;

    // Subscription storage mutex (protects all subscription maps and mHasSimulationStarted)
    mutable carb::tasking::MutexWrapper mSubscriptionMutex;

    // Benchmark subscription storage
    SubscriptionId mNextBenchmarkSubscriptionId{ 1 };
    std::unordered_map<SubscriptionId, std::vector<SimulationSubscriptionEntry>> mBenchmarkSubscriptionMap;

    // Contact report subscription storage
    SubscriptionId mNextContactReportSubscriptionId{ 1 };
    std::unordered_map<SubscriptionId, std::vector<SimulationSubscriptionEntry>> mContactReportSubscriptionMap;

    // OnStep subscription storage
    SubscriptionId mNextOnStepSubscriptionId{ 1 };
    std::unordered_map<SubscriptionId, std::vector<SimulationSubscriptionEntry>> mOnStepSubscriptionMap;
};
} // namespace physics
} // namespace omni
