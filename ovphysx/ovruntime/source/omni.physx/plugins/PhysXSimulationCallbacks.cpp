// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-LIFETIME-001
// @covers AC-3 AC-4

/**
 * @implements REQ-PUBLICAPI-002
 * @covers AC-7
 */
/**
 * @implements REQ-ATTACH-EXECUTION-001
 * @covers AC-1 AC-2 AC-3 AC-4
 */

#include "PhysXSimulationCallbacks.h"
#include "PhysXTools.h"
#include "PhysXScene.h"
#include "internal/InternalScene.h"
#include "Setup.h"
#include "usdLoad/LoadUsd.h"
#include "ObjectDataQuery.h"
#include "ContactReport.h"
#include "AttachmentContactLookup.h"
#include "OmniPhysX.h"
#include "Trigger.h"

using namespace omni::physx::internal;

namespace omni
{
namespace physx
{

uint32_t getGlobalSimulationFlags(uint32_t type, uint32_t flag)
{
    return (1 << (16 + type)) | flag;
}

static usdparser::AttachedStage* resolveCallbackOwner(AttachHandle attachHandle)
{
    return usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
}


SubscriptionId physxSubscribePhysicsContactReportEvents(AttachHandle attachHandle, OnContactReportEventFn onEvent, void* userData)
{
    RuntimeExecutionScope execution(attachHandle, true);
    if (!execution || !onEvent)
        return kInvalidSubscriptionId;
    usdparser::AttachedStage* owner = resolveCallbackOwner(attachHandle);
    if (!owner)
        return kInvalidSubscriptionId;
    return owner->getSimulationCallbacks().getContactReportRegistry().addEvent(std::make_pair(onEvent, userData));
}

void physxUnsubscribePhysicsContactReportEvents(AttachHandle attachHandle, SubscriptionId id)
{
    if (usdparser::AttachedStage* owner = resolveCallbackOwner(attachHandle))
    {
        OwnedContactRegistry& registry = owner->getSimulationCallbacks().getContactReportRegistry();
        registry.removeEvent(id);
    }
}

SubscriptionId physxSubscribePhysicsFullContactReportEvents(AttachHandle attachHandle, OnFullContactReportEventFn onEvent, void* userData)
{
    RuntimeExecutionScope execution(attachHandle, true);
    if (!execution || !onEvent)
        return kInvalidSubscriptionId;
    usdparser::AttachedStage* owner = resolveCallbackOwner(attachHandle);
    if (!owner)
        return kInvalidSubscriptionId;
    return owner->getSimulationCallbacks().getFullContactReportRegistry().addEvent(std::make_pair(onEvent, userData));
}

void physxUnsubscribePhysicsFullContactReportEvents(AttachHandle attachHandle, SubscriptionId id)
{
    if (usdparser::AttachedStage* owner = resolveCallbackOwner(attachHandle))
    {
        OwnedFullContactRegistry& registry = owner->getSimulationCallbacks().getFullContactReportRegistry();
        registry.removeEvent(id);
    }
}

SubscriptionId physxSubscribePhysicsTriggerReportEvents(AttachHandle attachHandle, uint64_t path, OnTriggerEventReportEventFn onEvent, void* userData)
{
    RuntimeExecutionScope execution(attachHandle, true);
    if (!execution || !onEvent || !resolveCallbackOwner(attachHandle))
        return kInvalidSubscriptionId;
    // Zero selects every trigger within this attachment. Other values are local keys.
    const omni::physics::parse::ObjectKey triggerColliderKey{ path };

    TriggerSubscriptionEntry sub;
    sub.attachHandle = attachHandle;
    sub.triggerColliderPrimKey = triggerColliderKey;
    sub.reportFn = onEvent;
    sub.userData = userData;
    return OmniPhysX::getInstance().getTriggerManager()->registerNativeCallback(sub);
}

void physxUnsubscribePhysicsTriggerReportEvents(SubscriptionId id)
{
   OmniPhysX::getInstance().getTriggerManager()->unregisterNativeCallback(id);
}

uint32_t physxGetContactReport(AttachHandle attachHandle, const ContactEventHeader** contactEventBuffer,
    const ContactData** contactDataBuffer, uint32_t& numContactData)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution || !contactEventBuffer || !contactDataBuffer)
        return 0;
    const usdparser::AttachedStage* owner = resolveCallbackOwner(attachHandle);
    if (!owner)
        return 0;
    const FrictionAnchor* anchors = nullptr;
    uint32_t numAnchors = 0;
    return attachmentContact::full(*owner, contactEventBuffer, contactDataBuffer, numContactData,
        &anchors, numAnchors);
}

uint32_t physxFullGetContactReport(AttachHandle attachHandle, const ContactEventHeader** contactEventBuffer,
    const ContactData** contactDataBuffer, uint32_t& numContactData,
    const FrictionAnchor** frictionAnchorsDataBuffer, uint32_t& numFrictionAnchorsData)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution || !contactEventBuffer || !contactDataBuffer || !frictionAnchorsDataBuffer)
        return 0;
    const usdparser::AttachedStage* owner = resolveCallbackOwner(attachHandle);
    if (!owner)
        return 0;
    return attachmentContact::full(*owner, contactEventBuffer, contactDataBuffer, numContactData,
        frictionAnchorsDataBuffer, numFrictionAnchorsData);
}


uint64_t physxGetSimulationTimestamp(AttachHandle attachHandle)
{
    const usdparser::AttachedStage* owner = resolveCallbackOwner(attachHandle);
    return owner ? owner->simulationTimestamp() : 0;
}

uint64_t physxGetSimulationStepCount(AttachHandle attachHandle)
{
    const usdparser::AttachedStage* owner = resolveCallbackOwner(attachHandle);
    return owner ? owner->simulationStepCount() : 0;
}

void physxSetSimulationCallback(AttachHandle attachHandle, const omni::physx::ISimulationCallback& cb)
{
    RuntimeExecutionScope operation(attachHandle);
    if (!operation)
        return;

    if (usdparser::AttachedStage* owner = resolveCallbackOwner(attachHandle))
        owner->getSimulationCallbacks().init(cb);
}

void physxSetSimulationFlags(AttachHandle attachHandle, uint32_t outputType, uint32_t flags)
{
    RuntimeExecutionScope operation(attachHandle);
    if (!operation)
        return;

    const usdparser::AttachedStage* attachedStage = resolveCallbackOwner(attachHandle);
    if (!attachedStage)
        return;

    SimulationCallbacks* cb = &attachedStage->getSimulationCallbacks();
    const uint32_t globalSimFlags = getGlobalSimulationFlags(outputType, flags);
    cb->setGlobalSimulationFlags(globalSimFlags);
}

void physxAddSimulationFlags(AttachHandle attachHandle, uint32_t outputType, uint32_t flags)
{
    RuntimeExecutionScope operation(attachHandle);
    if (!operation)
        return;

    const usdparser::AttachedStage* attachedStage = resolveCallbackOwner(attachHandle);
    if (!attachedStage)
        return;

    SimulationCallbacks* cb = &attachedStage->getSimulationCallbacks();
    const uint32_t globalSimFlags = getGlobalSimulationFlags(outputType, flags);
    cb->setGlobalSimulationFlags(cb->getGlobalSimulationFlags() | globalSimFlags);
}

void physxRemoveSimulationFlags(AttachHandle attachHandle, uint32_t outputType, uint32_t flags)
{
    RuntimeExecutionScope operation(attachHandle);
    if (!operation)
        return;

    const usdparser::AttachedStage* attachedStage = resolveCallbackOwner(attachHandle);
    if (!attachedStage)
        return;

    SimulationCallbacks* cb = &attachedStage->getSimulationCallbacks();
    const uint32_t globalSimFlags = getGlobalSimulationFlags(outputType, flags);
    cb->setGlobalSimulationFlags(cb->getGlobalSimulationFlags() & ~globalSimFlags);
}

SimulationCallbacks::SimulationCallbacks(const usdparser::AttachedStage& attachedStage)

    : mAttachedStage(attachedStage), mTransformationWriteFn(nullptr), mVelocityWriteFn(nullptr), mTransformUpdateFn(nullptr),
    mUserData(nullptr), mGlobalSimulationFlags(0)
{
}

SimulationCallbacks::~SimulationCallbacks()
{
    reset();

}

void SimulationCallbacks::init(const ISimulationCallback& cb)
{
    mTransformationWriteFn = cb.transformationWriteFn;
    mVelocityWriteFn = cb.velocityWriteFn;
    mTransformUpdateFn = cb.transformationUpdateFn;
    mUserData = cb.userData;
}

void SimulationCallbacks::reset()
{
    mTransformationWriteFn = nullptr;
    mVelocityWriteFn = nullptr;
    mTransformUpdateFn = nullptr;
    mGlobalSimulationFlags = 0;
}

bool SimulationCallbacks::checkRequireActiveActors() const
{
    const bool skipWriteTransforms = checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eSKIP_WRITE);
    const bool skipWriteVelocities = checkGlobalSimulationFlags(GlobalSimulationFlag::eVELOCITY | GlobalSimulationFlag::eSKIP_WRITE);
    TransformUpdateNotificationFn transformFn = getTransformationWriteFn();
    VelocityUpdateNotificationFn velocityFn = getVelocityWriteFn();
    const bool notifyTransforms = transformFn && checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eNOTIFY_UPDATE);
    const bool notifyVelocities = velocityFn && checkGlobalSimulationFlags(GlobalSimulationFlag::eVELOCITY | GlobalSimulationFlag::eNOTIFY_UPDATE);

    // skip the update loop if we should skip write and dont have notification callback request as a global
    // setting
    if (!(skipWriteTransforms && skipWriteVelocities && !notifyTransforms && !notifyVelocities))
        return true;
    else
        return false;
}

void SimulationCallbacks::setGlobalSimulationFlags(uint32_t flags)
{
    mGlobalSimulationFlags = flags;
    const bool needActive = checkRequireActiveActors();
    for (const usdparser::AttachedStage::SceneMap::value_type& entry : mAttachedStage.getPhysXScenes())
    {
        PhysXScene* scene = entry.second;
        scene->getScene()->setFlag(::physx::PxSceneFlag::eENABLE_ACTIVE_ACTORS,
                                  needActive && !scene->isReadbackSuppressed());
    }
}

}
}
