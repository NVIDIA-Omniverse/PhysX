// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <omni/physx/IPhysxSimulation.h>

namespace omni
{
namespace physx
{

struct GlobalSimulationFlag
{
    enum Enum
    {
        eSKIP_WRITE = 1 << 0,
        eNOTIFY_UPDATE = 1 << 1,
        eNOTIFY_IN_RADIANS = 1 << 2,

        eTRANSFORMATION = 1 << (16 + SimulationOutputType::eTRANSFORMATION),
        eVELOCITY = 1 << (16 + SimulationOutputType::eVELOCITY),
        ePOINTS = 1 << (16 + SimulationOutputType::ePOINTS),
    };
};

void physxSetSimulationCallback(const omni::physx::ISimulationCallback& cb);

void physxSetSimulationFlags(uint32_t outputType, uint32_t flags);
void physxAddSimulationFlags(uint32_t outputType, uint32_t flags);
void physxRemoveSimulationFlags(uint32_t outputType, uint32_t flags);

SubscriptionId physxSubscribePhysicsTriggerReportEvents(AttachHandle attachHandle,
                                                        uint64_t path,
                                                        OnTriggerEventReportEventFn onEvent,
                                                        void* userData);
void physxUnsubscribePhysicsTriggerReportEvents(SubscriptionId id);
SubscriptionId physxSubscribePhysicsContactReportEvents(OnContactReportEventFn onEvent, void* userData);
void physxUnsubscribePhysicsContactReportEvents(SubscriptionId id);
SubscriptionId physxSubscribePhysicsFullContactReportEvents(OnFullContactReportEventFn onEvent, void* userData);
void physxUnsubscribePhysicsFullContactReportEvents(SubscriptionId id);
uint32_t physxGetContactReport(const ContactEventHeader** contactEventBuffer,
                               const ContactData** contactDataBuffer,
                               uint32_t& numContactData);
uint32_t physxFullGetContactReport(const ContactEventHeader** contactEventBuffer,
                                   const ContactData** contactDataBuffer,
                                   uint32_t& numContactData,
                                   const FrictionAnchor** frictionAnchorDataBuffer,
                                   uint32_t& numFrictionAnchorData);
uint64_t physxGetSimulationTimestamp();
uint64_t physxGetSimulationStepCount();

class SimulationCallbacks
{
public:
    SimulationCallbacks();

    ~SimulationCallbacks();

    static SimulationCallbacks* getSimulationCallbacks();

    void init(const ISimulationCallback& cb);

    void reset();

    bool checkRequireActiveActors() const;

    void setGlobalSimulationFlags(uint32_t flags);

    uint32_t getGlobalSimulationFlags() const
    {
        return mGlobalSimulationFlags;
    }

    bool checkGlobalSimulationFlags(uint32_t flags) const
    {
        return (mGlobalSimulationFlags & flags) == flags;
    }

    TransformUpdateNotificationFn getTransformationWriteFn() const
    {
        return mTransformationWriteFn;
    }

    VelocityUpdateNotificationFn getVelocityWriteFn() const
    {
        return mVelocityWriteFn;
    }

    TransformUpdateFn getTransformUpdateFn() const
    {
        return mTransformUpdateFn;
    }

    void* getUserData() const
    {
        return mUserData;
    }

    const ContactReportEventSubscriptionRegistry& getContactReportRegistry() const
    {
        return mContactReportSubscriptions;
    }

    ContactReportEventSubscriptionRegistry& getContactReportRegistry()
    {
        return mContactReportSubscriptions;
    }

    const FullContactReportEventSubscriptionRegistry& getFullContactReportRegistry() const
    {
        return mFullContactReportSubscriptions;
    }

    FullContactReportEventSubscriptionRegistry& getFullContactReportRegistry()
    {
        return mFullContactReportSubscriptions;
    }

private:
    TransformUpdateNotificationFn mTransformationWriteFn;
    VelocityUpdateNotificationFn mVelocityWriteFn;
    TransformUpdateFn mTransformUpdateFn;
    void* mUserData;
    uint32_t mGlobalSimulationFlags;

    ContactReportEventSubscriptionRegistry mContactReportSubscriptions;
    FullContactReportEventSubscriptionRegistry mFullContactReportSubscriptions;
};

} // namespace physx
} // namespace omni
