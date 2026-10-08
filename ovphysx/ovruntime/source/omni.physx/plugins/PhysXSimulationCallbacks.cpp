// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-002
 * @covers AC-7
 */
#include "PhysXSimulationCallbacks.h"
#include "PhysXTools.h"
#include "PhysXScene.h"
#include "internal/InternalScene.h"
#include "Setup.h"
#include "usdLoad/LoadUsd.h"
#include "ObjectDataQuery.h"
#include "ContactReport.h"
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

static SimulationCallbacks* gSimulationCallbacks = nullptr;

SimulationCallbacks* SimulationCallbacks::getSimulationCallbacks()
{
    if (!gSimulationCallbacks)
        gSimulationCallbacks = new SimulationCallbacks();

    return gSimulationCallbacks;
}

SubscriptionId physxSubscribePhysicsContactReportEvents(OnContactReportEventFn onEvent, void* userData)
{
    return SimulationCallbacks::getSimulationCallbacks()->getContactReportRegistry().addEvent(std::make_pair(onEvent, userData));
}

void physxUnsubscribePhysicsContactReportEvents(SubscriptionId id)
{
    SimulationCallbacks::getSimulationCallbacks()->getContactReportRegistry().removeEvent(id);
}

SubscriptionId physxSubscribePhysicsFullContactReportEvents(OnFullContactReportEventFn onEvent, void* userData)
{
    return SimulationCallbacks::getSimulationCallbacks()->getFullContactReportRegistry().addEvent(std::make_pair(onEvent, userData));
}

void physxUnsubscribePhysicsFullContactReportEvents(SubscriptionId id)
{
    SimulationCallbacks::getSimulationCallbacks()->getFullContactReportRegistry().removeEvent(id);
}

SubscriptionId physxSubscribePhysicsTriggerReportEvents(AttachHandle attachHandle, uint64_t path, OnTriggerEventReportEventFn onEvent, void* userData)
{
    // kActiveAttach is stored as-is and bound late, at fire time. Resolving it here would be
    // wrong in the ordinary case: subscribing *before* attaching is a supported pattern, and
    // there is no attach to resolve against yet. TriggerManager::processNativeEvent therefore
    // treats a stored kActiveAttach as "whichever attach is live" rather than comparing it as a
    // literal (ADR-0016 Decision 6). The consequence is deliberate: such a subscription follows
    // the active attach across a detach/reattach, which is what "the active one" asks for.
    //
    // A concrete handle is different -- it names one attach, so it is validated now. Deferring
    // that would turn a typo into a subscription that silently never fires.
    if (attachHandle != kActiveAttach && !usdparser::UsdLoad::getUsdLoad()->resolveAttach(attachHandle))
    {
        CARB_LOG_ERROR("subscribePhysicsTriggerReportEvents could not resolve attach handle %llu: it is either "
                       "kNoAttach or stale. Pass kActiveAttach to follow the active attach.",
                       static_cast<unsigned long long>(attachHandle));
        return kInvalidSubscriptionId;
    }

    // path == 0 is the "report from every trigger" sentinel (ADR-0016 Decision 6's wildcard,
    // expressed via triggerColliderPrimKey's invalid ObjectKey{} rather than a wildcard path).
    //
    // a nonzero path is now `ObjectKey::handle` directly, not the legacy
    // asInt()-encoded-SdfPath-bits wire format -- same pattern as PhysXReplicator's `topKey`
    // construction. `ObjectKey{ path }` naturally reproduces the path == 0 sentinel too (its
    // `handle` defaults to 0, same as the invalid ObjectKey{}), so no attach lookup and no
    // separate zero-check are needed any more: minting used to require an already-live
    // AttachedStage to decode the bits through; a raw handle needs none.
    //
    // Generation-tag caveat (ADR-0021): a handle is only valid against the Source instance that
    // minted it. A caller that caches a `path` value across a detach/reattach (or a kActiveAttach
    // subscription that outlives one) will silently stop matching after the reattach, since the
    // new Source mints a fresh generation for every key -- a resolution failure, not decode
    // garbage. No consumer attaches twice today (ADR-0016's own multi-attach caveat), so this is
    // unreached in practice; a future multi-attach or resubscribe-after-reattach consumer would
    // need to refresh its handle after each reattach.
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

uint32_t physxGetContactReport(const ContactEventHeader** contactEventBuffer, const ContactData** contactDataBuffer, uint32_t& numContactData)
{
    PhysXScene* physxScene = OmniPhysX::getInstance().getPhysXSetup().getPhysXScene(0);
    if (!physxScene)
        return 0;

    const ContactReport* cr = physxScene->getContactReport();
    const ContactHeadersVector& contactHeaderVector = cr->getContactHeaderVector();
    if (contactHeaderVector.empty())
    {
        return 0;
    }

    const ContactDataVector& contactDataVector = cr->getContactDataVector();

    *contactEventBuffer = contactHeaderVector.data();
    numContactData = uint32_t(contactDataVector.size());
    if (numContactData)
    {
        *contactDataBuffer = contactDataVector.data();
    }

    return uint32_t(contactHeaderVector.size());
}

uint32_t physxFullGetContactReport(const ContactEventHeader** contactEventBuffer, const ContactData** contactDataBuffer, uint32_t& numContactData,
    const FrictionAnchor** frictionAnchorsDataBuffer, uint32_t& numFrictionAnchorsData)
{
    PhysXScene* physxScene = OmniPhysX::getInstance().getPhysXSetup().getPhysXScene(0);
    if (!physxScene)
        return 0;

    const ContactReport* cr = physxScene->getContactReport();
    const ContactHeadersVector& contactHeaderVector = cr->getContactHeaderVector();
    if (contactHeaderVector.empty())
    {
        return 0;
    }

    const ContactDataVector& contactDataVector = cr->getContactDataVector();

    *contactEventBuffer = contactHeaderVector.data();
    numContactData = uint32_t(contactDataVector.size());
    if (numContactData)
    {
        *contactDataBuffer = contactDataVector.data();
    }

    const FrictionAnchorsDataVector& frictionAnchorsDataVector = cr->getFrictionAnchorsDataVector();
    numFrictionAnchorsData = uint32_t(frictionAnchorsDataVector.size());
    if (numFrictionAnchorsData)
    {
        *frictionAnchorsDataBuffer = frictionAnchorsDataVector.data();
    }

    return uint32_t(contactHeaderVector.size());
}


uint64_t physxGetSimulationTimestamp()
{
    return OmniPhysX::getInstance().getSimulationTimestamp();
}

uint64_t physxGetSimulationStepCount()
{
    return OmniPhysX::getInstance().getSimulationStepCount();
}

void physxSetSimulationCallback(const omni::physx::ISimulationCallback& cb)
{
    SimulationCallbacks::getSimulationCallbacks()->init(cb);
}

void physxSetSimulationFlags(uint32_t outputType, uint32_t flags)
{
    const usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
    if (!attachedStage)
        return;

    SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
    const uint32_t globalSimFlags = getGlobalSimulationFlags(outputType, flags);
    cb->setGlobalSimulationFlags(globalSimFlags);
}

void physxAddSimulationFlags(uint32_t outputType, uint32_t flags)
{
    const usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
    if (!attachedStage)
        return;

    SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
    const uint32_t globalSimFlags = getGlobalSimulationFlags(outputType, flags);
    cb->setGlobalSimulationFlags(cb->getGlobalSimulationFlags() | globalSimFlags);
}

void physxRemoveSimulationFlags(uint32_t outputType, uint32_t flags)
{
    const usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
    if (!attachedStage)
        return;

    SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
    const uint32_t globalSimFlags = getGlobalSimulationFlags(outputType, flags);
    cb->setGlobalSimulationFlags(cb->getGlobalSimulationFlags() & ~globalSimFlags);
}

SimulationCallbacks::SimulationCallbacks()

    : mTransformationWriteFn(nullptr), mVelocityWriteFn(nullptr), mTransformUpdateFn(nullptr),
    mUserData(nullptr), mGlobalSimulationFlags(0)
{
}

SimulationCallbacks::~SimulationCallbacks()
{
    reset();
    // gSimulationCallbacks is allocated with `new` and never deleted, so this
    // destructor is unreachable in practice. Both subscription registries therefore
    // live until process exit. Client code is responsible for calling unsubscribe
    // before its userData storage goes away.
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
    if (checkRequireActiveActors())
    {
        OmniPhysX::getInstance().getPhysXSetup().enableActiveActors(true);
    }
    else
    {
        OmniPhysX::getInstance().getPhysXSetup().enableActiveActors(false);
    }    
}

}
}
