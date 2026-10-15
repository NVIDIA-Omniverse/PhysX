// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-EVENT-QUEUE-001
// @covers AC-4

// @implements REQ-ATTACH-LIFETIME-001
// @covers AC-3 AC-4

/**
 * @implements REQ-ATTACH-EXECUTION-001
 * @covers AC-1 AC-2 AC-4
 */

#pragma once

#include <omni/physx/IPhysxSimulation.h>
#include "CallbackRegistry.h"

namespace omni
{
namespace physx
{

namespace usdparser { class AttachedStage; }

using OwnedContactRegistry = CallbackRegistry<std::pair<std::function<void(const ContactEventHeader*, uint32_t, const ContactData*, uint32_t, void*)>, void*>>;
using OwnedFullContactRegistry = CallbackRegistry<std::pair<OnFullContactReportEventFn, void*>>;

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

void physxSetSimulationCallback(AttachHandle attachHandle, const omni::physx::ISimulationCallback& cb);

void physxSetSimulationFlags(AttachHandle attachHandle, uint32_t outputType, uint32_t flags);
void physxAddSimulationFlags(AttachHandle attachHandle, uint32_t outputType, uint32_t flags);
void physxRemoveSimulationFlags(AttachHandle attachHandle, uint32_t outputType, uint32_t flags);

SubscriptionId physxSubscribePhysicsTriggerReportEvents(AttachHandle attachHandle,
                                                        uint64_t path,
                                                        OnTriggerEventReportEventFn onEvent,
                                                        void* userData);
void physxUnsubscribePhysicsTriggerReportEvents(SubscriptionId id);
SubscriptionId physxSubscribePhysicsContactReportEvents(AttachHandle attachHandle, OnContactReportEventFn onEvent, void* userData);
void physxUnsubscribePhysicsContactReportEvents(AttachHandle attachHandle, SubscriptionId id);
SubscriptionId physxSubscribePhysicsFullContactReportEvents(AttachHandle attachHandle, OnFullContactReportEventFn onEvent, void* userData);
void physxUnsubscribePhysicsFullContactReportEvents(AttachHandle attachHandle, SubscriptionId id);
uint32_t physxGetContactReport(AttachHandle attachHandle, const ContactEventHeader** contactEventBuffer,
                               const ContactData** contactDataBuffer,
                               uint32_t& numContactData);
uint32_t physxFullGetContactReport(AttachHandle attachHandle, const ContactEventHeader** contactEventBuffer,
                                   const ContactData** contactDataBuffer,
                                   uint32_t& numContactData,
                                   const FrictionAnchor** frictionAnchorDataBuffer,
                                   uint32_t& numFrictionAnchorData);
uint64_t physxGetSimulationTimestamp(AttachHandle attachHandle);
uint64_t physxGetSimulationStepCount(AttachHandle attachHandle);

class SimulationCallbacks
{
public:
    explicit SimulationCallbacks(const usdparser::AttachedStage& attachedStage);

    ~SimulationCallbacks();

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

    const OwnedContactRegistry& getContactReportRegistry() const
    {
        return mContactReportSubscriptions;
    }

    OwnedContactRegistry& getContactReportRegistry()
    {
        return mContactReportSubscriptions;
    }

    const OwnedFullContactRegistry& getFullContactReportRegistry() const
    {
        return mFullContactReportSubscriptions;
    }

    OwnedFullContactRegistry& getFullContactReportRegistry()
    {
        return mFullContactReportSubscriptions;
    }

private:
    const usdparser::AttachedStage& mAttachedStage;

    TransformUpdateNotificationFn mTransformationWriteFn;
    VelocityUpdateNotificationFn mVelocityWriteFn;
    TransformUpdateFn mTransformUpdateFn;
    void* mUserData;
    uint32_t mGlobalSimulationFlags;

    OwnedContactRegistry mContactReportSubscriptions;
    OwnedFullContactRegistry mFullContactReportSubscriptions;
};

} // namespace physx
} // namespace omni
