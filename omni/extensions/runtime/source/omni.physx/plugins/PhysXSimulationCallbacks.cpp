// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

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

SubscriptionId physxSubscribePhysicsTriggerReportEvents(uint64_t stageId, uint64_t path, OnTriggerEventReportEventFn onEvent, void* userData)
{
    TriggerSubscriptionEntry sub;
    sub.stageId = stageId;
    sub.triggerColliderPrimId = path;
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
    // A.B. TODO
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
    // A.B. TODO
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

void setSimulationFlag(uint32_t& flagsOut, const uint32_t flagIn, uint32_t setFlagIn, const uint32_t setFlagOut)
{
    if (flagIn & setFlagIn)
        flagsOut |= setFlagOut;
    else
        flagsOut &= ~setFlagOut;
}

void addSimulationFlag(uint32_t& flagsOut, const uint32_t flagIn, uint32_t setFlagIn, const uint32_t setFlagOut)
{
    if (flagIn & setFlagIn)
        flagsOut |= setFlagOut;
}

void removeSimulationFlag(uint32_t& flagsOut, const uint32_t flagIn, uint32_t setFlagIn, const uint32_t setFlagOut)
{
    if (flagIn & setFlagIn)
        flagsOut &= ~setFlagOut;
}

void physxSetSimulationFlags(uint32_t flags, const uint64_t* paths, uint32_t numPaths)
{
    physxSetSimulationFlags(SimulationOutputType::eTRANSFORMATION, flags, paths, numPaths);
}

void physxSetSimulationFlags(uint32_t outputType, uint32_t flags, const uint64_t* paths, uint32_t numPaths)
{
    const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const uint64_t stageId = OmniPhysX::getInstance().getStageId();
    const usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (!attachedStage)
        return;

    SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
    const uint32_t globalSimFlags = getGlobalSimulationFlags(outputType, flags);
    if (paths)
    {
        for (uint32_t i = 0; i < numPaths; i++)
        {
            const pxr::SdfPath& path = intToPath(paths[i]);
            cb->setSimulationFlags(path, globalSimFlags);
            // check for existing internal actors
            InternalActor* actor = reinterpret_cast<InternalActor*>(getObjectDataOrID<ObjectDataQueryType::eINTERNAL_PTR>(path, ePTActor, db, *attachedStage));
            if (actor)
            {
                if (outputType == SimulationOutputType::eTRANSFORMATION)
                {
                    setSimulationFlag(actor->mFlags, flags, SimulationOutputFlag::eSKIP_WRITE, InternalActorFlag::eSKIP_UPDATE_TRANSFORM);
                    setSimulationFlag(actor->mFlags, flags, SimulationOutputFlag::eNOTIFY_UPDATE, InternalActorFlag::eNOTIFY_TRANSFORM);
                }
                else if (outputType == SimulationOutputType::eVELOCITY)
                {
                    setSimulationFlag(actor->mFlags, flags, SimulationOutputFlag::eSKIP_WRITE, InternalActorFlag::eSKIP_UPDATE_VELOCITY);
                    setSimulationFlag(actor->mFlags, flags, SimulationOutputFlag::eNOTIFY_UPDATE, InternalActorFlag::eNOTIFY_VELOCITY);
                    setSimulationFlag(actor->mFlags, flags, SimulationOutputFlag::eNOTIFY_IN_RADIANS, InternalActorFlag::eNOTIFY_VELOCITY_RADIANS);
                }
            }
            // check for existing internal vehicles
            InternalVehicle* vehicle = reinterpret_cast<InternalVehicle*>(getObjectDataOrID<ObjectDataQueryType::eINTERNAL_PTR>(path, ePTVehicle, db, *attachedStage));
            if (vehicle)
            {
                if (outputType == SimulationOutputType::eTRANSFORMATION)
                {
                    setSimulationFlag(vehicle->mFlags, flags, SimulationOutputFlag::eSKIP_WRITE, InternalVehicleFlag::eSKIP_UPDATE_TRANSFORM);
                    setSimulationFlag(vehicle->mFlags, flags, SimulationOutputFlag::eNOTIFY_UPDATE, InternalVehicleFlag::eNOTIFY_TRANSFORM);
                }
            }
        }
    }
    else
    {
        cb->setGlobalSimulationFlags(globalSimFlags);
    }
}

void physxAddSimulationFlags(uint32_t flags, const uint64_t* paths, uint32_t numPaths)
{
    physxAddSimulationFlags(SimulationOutputType::eTRANSFORMATION, flags, paths, numPaths);
}

void physxAddSimulationFlags(uint32_t outputType, uint32_t flags, const uint64_t* paths, uint32_t numPaths)
{
    const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const uint64_t stageId = OmniPhysX::getInstance().getStageId();
    const usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (!attachedStage)
        return;

    SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
    const uint32_t globalSimFlags = getGlobalSimulationFlags(outputType, flags);
    if (paths)
    {
        for (uint32_t i = 0; i < numPaths; i++)
        {
            const pxr::SdfPath& path = intToPath(paths[i]);
            cb->setSimulationFlags(path, cb->getSimulationFlags(path) | globalSimFlags);

            InternalActor* actor = reinterpret_cast<InternalActor*>(getObjectDataOrID<ObjectDataQueryType::eINTERNAL_PTR>(path, ePTActor, db, *attachedStage));
            if (actor)
            {
                if (outputType == SimulationOutputType::eTRANSFORMATION)
                {
                    addSimulationFlag(actor->mFlags, flags, SimulationOutputFlag::eSKIP_WRITE, InternalActorFlag::eSKIP_UPDATE_TRANSFORM);
                    addSimulationFlag(actor->mFlags, flags, SimulationOutputFlag::eNOTIFY_UPDATE, InternalActorFlag::eNOTIFY_TRANSFORM);
                }
                else if (outputType == SimulationOutputType::eVELOCITY)
                {
                    addSimulationFlag(actor->mFlags, flags, SimulationOutputFlag::eSKIP_WRITE, InternalActorFlag::eSKIP_UPDATE_VELOCITY);
                    addSimulationFlag(actor->mFlags, flags, SimulationOutputFlag::eNOTIFY_UPDATE, InternalActorFlag::eNOTIFY_VELOCITY);
                    addSimulationFlag(actor->mFlags, flags, SimulationOutputFlag::eNOTIFY_IN_RADIANS, InternalActorFlag::eNOTIFY_VELOCITY_RADIANS);
                }
            }
            // check for existing internal vehicles
            InternalVehicle* vehicle = reinterpret_cast<InternalVehicle*>(getObjectDataOrID<ObjectDataQueryType::eINTERNAL_PTR>(path, ePTVehicle, db, *attachedStage));
            if (vehicle)
            {
                if (outputType == SimulationOutputType::eTRANSFORMATION)
                {
                    addSimulationFlag(vehicle->mFlags, flags, SimulationOutputFlag::eSKIP_WRITE, InternalVehicleFlag::eSKIP_UPDATE_TRANSFORM);
                    addSimulationFlag(vehicle->mFlags, flags, SimulationOutputFlag::eNOTIFY_UPDATE, InternalVehicleFlag::eNOTIFY_TRANSFORM);
                }
            }
        }
    }
    else
    {        
        cb->setGlobalSimulationFlags(cb->getGlobalSimulationFlags() | globalSimFlags);
    }
}

void physxRemoveSimulationFlags(uint32_t flags, const uint64_t* paths, uint32_t numPaths)
{
    physxRemoveSimulationFlags(SimulationOutputType::eTRANSFORMATION, flags, paths, numPaths);
}

void physxRemoveSimulationFlags(uint32_t outputType, uint32_t flags, const uint64_t* paths, uint32_t numPaths)
{
    const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const uint64_t stageId = OmniPhysX::getInstance().getStageId();
    const usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (!attachedStage)
        return;

    SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
    const uint32_t globalSimFlags = getGlobalSimulationFlags(outputType, flags);
    if (paths)
    {
        for (uint32_t i = 0; i < numPaths; i++)
        {
            const pxr::SdfPath& path = intToPath(paths[i]);
            cb->setSimulationFlags(path, cb->getSimulationFlags(path) & ~globalSimFlags);

            InternalActor* actor = reinterpret_cast<InternalActor*>(getObjectDataOrID<ObjectDataQueryType::eINTERNAL_PTR>(path, ePTActor, db, *attachedStage));
            if (actor)
            {
                if (outputType == SimulationOutputType::eTRANSFORMATION)
                {
                    removeSimulationFlag(actor->mFlags, flags, SimulationOutputFlag::eSKIP_WRITE, InternalActorFlag::eSKIP_UPDATE_TRANSFORM);
                    removeSimulationFlag(actor->mFlags, flags, SimulationOutputFlag::eNOTIFY_UPDATE, InternalActorFlag::eNOTIFY_TRANSFORM);
                }
                else if (outputType == SimulationOutputType::eVELOCITY)
                {
                    removeSimulationFlag(actor->mFlags, flags, SimulationOutputFlag::eSKIP_WRITE, InternalActorFlag::eSKIP_UPDATE_VELOCITY);
                    removeSimulationFlag(actor->mFlags, flags, SimulationOutputFlag::eNOTIFY_UPDATE, InternalActorFlag::eNOTIFY_VELOCITY);
                    removeSimulationFlag(actor->mFlags, flags, SimulationOutputFlag::eNOTIFY_IN_RADIANS, InternalActorFlag::eNOTIFY_VELOCITY_RADIANS);
                }
            }
            // check for existing internal vehicles
            InternalVehicle* vehicle = reinterpret_cast<InternalVehicle*>(getObjectDataOrID<ObjectDataQueryType::eINTERNAL_PTR>(path, ePTVehicle, db, *attachedStage));
            if (vehicle)
            {
                if (outputType == SimulationOutputType::eTRANSFORMATION)
                {
                    removeSimulationFlag(vehicle->mFlags, flags, SimulationOutputFlag::eSKIP_WRITE, InternalVehicleFlag::eSKIP_UPDATE_TRANSFORM);
                    removeSimulationFlag(vehicle->mFlags, flags, SimulationOutputFlag::eNOTIFY_UPDATE, InternalVehicleFlag::eNOTIFY_TRANSFORM);
                }
            }
        }
    }
    else
    {
        cb->setGlobalSimulationFlags(cb->getGlobalSimulationFlags() & ~globalSimFlags);
    }
}

SimulationCallbacks::SimulationCallbacks()

    : mTransformationWriteFn(nullptr), mVelocityWriteFn(nullptr), mResidualWriteFn(nullptr), mTransformUpdateFn(nullptr),
    mUserData(nullptr), mGlobalSimulationFlags(0)
{
}

SimulationCallbacks::~SimulationCallbacks()
{
    reset();
    mContactReportSubscriptions.clear();
}

void SimulationCallbacks::init(const ISimulationCallback& cb)
{
    mTransformationWriteFn = cb.transformationWriteFn;
    mVelocityWriteFn = cb.velocityWriteFn;
    mResidualWriteFn = cb.residualWriteFn;
    mTransformUpdateFn = cb.transformationUpdateFn;
    mUserData = cb.userData;
}

void SimulationCallbacks::reset()
{
    mTransformationWriteFn = nullptr;
    mVelocityWriteFn = nullptr;
    mResidualWriteFn = nullptr;
    mTransformUpdateFn = nullptr;
    mGlobalSimulationFlags = 0;
    mActorSimulationFlags = 0;

    mSimulationFlagsMap.clear();
}

bool SimulationCallbacks::checkRequireActiveActors() const
{
    const bool skipWriteTransforms = checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eSKIP_WRITE);
    const bool skipWriteVelocities = checkGlobalSimulationFlags(GlobalSimulationFlag::eVELOCITY | GlobalSimulationFlag::eSKIP_WRITE);
    TransformUpdateNotificationFn transformFn = getTransformationWriteFn();
    VelocityUpdateNotificationFn velocityFn = getVelocityWriteFn();    
    const bool notifyTransforms = transformFn && checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eNOTIFY_UPDATE);
    const bool notifyVelocities = velocityFn && checkGlobalSimulationFlags(GlobalSimulationFlag::eVELOCITY | GlobalSimulationFlag::eNOTIFY_UPDATE);

    const bool notifyActorTransforms = checkActorSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION |
        GlobalSimulationFlag::eNOTIFY_UPDATE);
    const bool notifyActorVelocities = checkActorSimulationFlags(GlobalSimulationFlag::eVELOCITY |
        GlobalSimulationFlag::eNOTIFY_UPDATE);

    // skip the update loop if we should skip write, dont have notification callback request as a global
    // setting and if its not set on any actor
    if (!(skipWriteTransforms && skipWriteVelocities && !notifyTransforms && !notifyVelocities &&
        !notifyActorTransforms && !notifyActorVelocities))
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
