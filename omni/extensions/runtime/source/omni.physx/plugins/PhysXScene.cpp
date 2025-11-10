// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXSettings.h"
#include "PhysXScene.h"
#include "PhysXDefines.h"
#include "Setup.h"
#include "OmniPhysX.h"
#include "ContactReport.h"
#include "Trigger.h"
#include "PhysXTools.h"
#include "usdLoad/LoadUsd.h"
#include "usdLoad/Material.h"
#include "usdLoad/AttachedStage.h"
#include "PhysXSimulationCallbacks.h"

#include "internal/InternalParticle.h"

#include <omni/physx/TriggerEvent.h>

#include <utils/SplinesCurve.h>

#include <PxPhysicsAPI.h>
#include "gpu/PxPhysicsGpu.h"

#include <carb/profiler/Profile.h>
#include <common/utilities/MemoryMacros.h>

#include <deformables/PhysXDeformablePost.h>

using namespace pxr;
using namespace carb;
using namespace ::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

OMNI_LOG_DECLARE_CHANNEL(kSceneMultiGPULogChannel)

namespace omni
{
namespace physx
{


static PxFilterFlags OmniFilterShader(PxFilterObjectAttributes attributes0,
    PxFilterData filterData0,
    PxFilterObjectAttributes attributes1,
    PxFilterData filterData1,
    PxPairFlags& pairFlags,
    const void* constantBlock,
    PxU32 constantBlockSize)
{
    pairFlags = PxPairFlag::eCONTACT_DEFAULT | PxPairFlag::eDETECT_CCD_CONTACT;

    // if (usesCCD())
    //    pairFlags |= PxPairFlag::eDETECT_CCD_CONTACT | PxPairFlag::eNOTIFY_TOUCH_CCD;

    // let triggers through
    if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
    {
        pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
        return PxFilterFlags();
    }

    const PhysXScene* scene = *(const PhysXScene**)constantBlock;
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    if (filterData0.word2 && filterData1.word2)
    {
        const Pair<uint32_t> groupPair(filterData0.word2, filterData1.word2);
        CollisionGroupsPairsSet::const_iterator it = physxSetup.getCollisionGroupFilteredPairs().find(groupPair);
        if (!scene->isInvertedCollisionGroupFilter())
        {
            if (it != physxSetup.getCollisionGroupFilteredPairs().end())
                return PxFilterFlag::eKILL;
        }
        else
        {
            if (it == physxSetup.getCollisionGroupFilteredPairs().end())
                return PxFilterFlag::eKILL;
        }
    }

    // Contact modify flags stored in word3
    if (filterData0.word3 & CONTACT_MODIFY_SURFACE_VELOCITY || filterData1.word3 & CONTACT_MODIFY_SURFACE_VELOCITY)
    {
        pairFlags = pairFlags | PxPairFlag::eMODIFY_CONTACTS;
    }

    // Contact solve flags stored in word3
    if (filterData0.word3 & CONTACT_SOLVE_DISABLE || filterData1.word3 & CONTACT_SOLVE_DISABLE)
    {
        return PxFilterFlag::eCALLBACK;
    }

    // filtered pairs stored in word1
    if (filterData0.word1 && filterData1.word1)
    {
        const Pair<uint32_t> filterPair(filterData0.word1, filterData1.word1);
        FilteredPairsSet::const_iterator it = physxSetup.getFilteredPairs().find(filterPair);
        if (it != physxSetup.getFilteredPairs().end())
            return PxFilterFlag::eKILL;
    }

    if (!scene->getContactReport()->empty())
    {
        return PxFilterFlag::eCALLBACK;
    }

    return PxFilterFlag::eDEFAULT;
}

PxFilterFlags OmniFilterCallback::pairFound(PxU32 pairID,
        PxFilterObjectAttributes attributes0, PxFilterData filterData0, const PxActor* a0, const PxShape* s0,
        PxFilterObjectAttributes attributes1, PxFilterData filterData1, const PxActor* a1, const PxShape* s1,
        PxPairFlags& pairFlags)
{
    return pairFound(PxU64(pairID), attributes0, filterData0, a0, s0, attributes1, filterData1, a1, s1, pairFlags);
}

PxFilterFlags OmniFilterCallback::pairFound(PxU64 pairID,
        PxFilterObjectAttributes attributes0, PxFilterData filterData0, const PxActor* a0, const PxShape* s0,
        PxFilterObjectAttributes attributes1, PxFilterData filterData1, const PxActor* a1, const PxShape* s1,
        PxPairFlags& pairFlags)
{
    const PhysXSetup& physxSetup = mOmniPhysX->getPhysXSetup();

    // Check for invalid PxActors
    auto type0 = a0->getConcreteType();
    auto type1 = a1->getConcreteType();
    if (type0 == 0xFFFF || type1 == 0xFFFF)
        return PxFilterFlags();

    // Handle special condition for particle system because it does not have a valid PxShape
    if (a0->getType() == PxActorType::ePBD_PARTICLESYSTEM || a1->getType() == PxActorType::ePBD_PARTICLESYSTEM)
        return PxFilterFlags();

    if (s0->getFlags() & PxShapeFlag::eTRIGGER_SHAPE || s1->getFlags() & PxShapeFlag::eTRIGGER_SHAPE)
        pairFlags = PxPairFlag::eTRIGGER_DEFAULT;

    if (filterData0.word3 & CONTACT_SOLVE_DISABLE || filterData1.word3 & CONTACT_SOLVE_DISABLE)
    {
        pairFlags &= ~PxPairFlag::eSOLVE_CONTACT;
    }

    if (mContactReport->checkPair(a0, a1))
    {
        const bool kinematic0 = PxFilterObjectIsKinematic(attributes0);
        const bool kinematic1 = PxFilterObjectIsKinematic(attributes1);
        const bool static0 = ::physx::PxGetFilterObjectType(attributes0) == ::physx::PxFilterObjectType::eRIGID_STATIC;
        const bool static1 = ::physx::PxGetFilterObjectType(attributes1) == ::physx::PxFilterObjectType::eRIGID_STATIC;

        pairFlags = pairFlags | PxPairFlag::eNOTIFY_TOUCH_LOST
            | PxPairFlag::eNOTIFY_TOUCH_FOUND
            | PxPairFlag::eNOTIFY_TOUCH_PERSISTS
            | PxPairFlag::eNOTIFY_CONTACT_POINTS;

        if ((kinematic0 || static0) && (kinematic1 || static1))
        {
            pairFlags &= ~PxPairFlag::eSOLVE_CONTACT;
            pairFlags |= PxPairFlag::eDETECT_DISCRETE_CONTACT;
        }
        return PxFilterFlags();
    }

    return PxFilterFlags();
}



void OmniContactReportCallback::onConstraintBreak(PxConstraintInfo* constraints, PxU32 count)
{
    for (PxU32 i = 0; i < count; i++)
    {
        PxJoint* joint = reinterpret_cast<PxJoint*>(constraints[i].externalReference);
        if (joint)
        {
            mContactReport->reportJointBreak(joint);
        }
    }

}

void OmniContactReportCallback::onWake(PxActor** actors, PxU32 count)
{
}

void OmniContactReportCallback::onSleep(PxActor** actors, PxU32 count)
{
}

bool OmniContactReportCallback::isFilteredEvent(const ::physx::PxShape* s0, const ::physx::PxShape* s1)
{
    const PhysXSetup& physxSetup = mOmniPhysX->getPhysXSetup();

    const PxFilterData filterData0 = s0->getSimulationFilterData();
    const PxFilterData filterData1 = s1->getSimulationFilterData();

    if (filterData0.word2 && filterData1.word2)
    {
        const Pair<uint32_t> groupPair(filterData0.word2, filterData1.word2);
        CollisionGroupsPairsSet::const_iterator it = physxSetup.getCollisionGroupFilteredPairs().find(groupPair);
        if (!mPhysXScene->isInvertedCollisionGroupFilter())
        {
            if (it != physxSetup.getCollisionGroupFilteredPairs().end())
                return true;
        }
        else
        {
            if (it == physxSetup.getCollisionGroupFilteredPairs().end())
                return true;
        }
    }

    if (filterData0.word1 && filterData1.word1)
    {
        const Pair<uint32_t> filterPair(filterData0.word1, filterData1.word1);
        FilteredPairsSet::const_iterator it = physxSetup.getFilteredPairs().find(filterPair);
        if (it != physxSetup.getFilteredPairs().end())
            return true;
    }

    return false;
}

void OmniContactReportCallback::onTrigger(PxTriggerPair* pairs, PxU32 count)
{
    while (count--)
    {
        const PxTriggerPair& current = *pairs++;
        if (current.flags & PxTriggerPairFlag::eREMOVED_SHAPE_TRIGGER)
            continue;
        if (current.flags & PxTriggerPairFlag::eREMOVED_SHAPE_OTHER)
        {
            mOmniPhysX->getTriggerManager()->bufferTriggerEvent(current.triggerShape, current.otherShape, TriggerEventType::eTRIGGER_ON_LEAVE);
            continue;
        }
        if (!current.triggerShape || !current.otherShape)
        {
            CARB_ASSERT(!current.triggerShape || !current.otherShape);
            continue;
        }

        if (current.status & PxPairFlag::eNOTIFY_TOUCH_FOUND && !isFilteredEvent(current.triggerShape, current.otherShape))
            mOmniPhysX->getTriggerManager()->bufferTriggerEvent(current.triggerShape, current.otherShape, TriggerEventType::eTRIGGER_ON_ENTER);
        if (current.status & PxPairFlag::eNOTIFY_TOUCH_LOST && !isFilteredEvent(current.triggerShape, current.otherShape))
            mOmniPhysX->getTriggerManager()->bufferTriggerEvent(current.triggerShape, current.otherShape, TriggerEventType::eTRIGGER_ON_LEAVE);
    }
}

void OmniContactReportCallback::onAdvance(const PxRigidBody* const* bodyBuffer, const PxTransform* poseBuffer, const PxU32 count)
{
}

void OmniContactReportCallback::onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairsIn, PxU32 countIn)
{
    if (mOmniPhysX->getCachedSettings().disableContactProcessing)
        return;

    PxU32 count = countIn;
    const PxContactPair* pairs = pairsIn;
    float forceSum = 0;
    PxU32 contactCountSum = 0;

    while (count--)
    {
        const PxContactPair& current = *pairs++;

        if (current.flags & PxContactPairFlag::eREMOVED_SHAPE_0 || current.flags & PxContactPairFlag::eREMOVED_SHAPE_1)
        {
            mContactReport->reportContactShapeRemoved(pairHeader.actors[0], current.shapes[0], pairHeader.actors[1], current.shapes[1], current.flags);
            continue;
        }

        PxU32 contactCount = current.contactCount;
        contactCountSum += contactCount;

        if (contactCount && current.contactImpulses)
        {
            for (PxU32 j = 0; j < contactCount; j++)
            {
                forceSum += current.contactImpulses[j];
            }
        }
    }
    forceSum /= mStepSize;

    if (pairHeader.flags & PxContactPairHeaderFlag::eREMOVED_ACTOR_0 || pairHeader.flags & PxContactPairHeaderFlag::eREMOVED_ACTOR_1)
        return;

    const bool kinematic0 = (pairHeader.actors[0]->getType() == PxActorType::eRIGID_DYNAMIC) && ((PxRigidDynamic*)pairHeader.actors[0])->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC;
    const bool kinematic1 = (pairHeader.actors[1]->getType() == PxActorType::eRIGID_DYNAMIC) && ((PxRigidDynamic*)pairHeader.actors[1])->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC;

    const bool static0 = pairHeader.actors[0]->getType() == PxActorType::eRIGID_STATIC;
    const bool static1 = pairHeader.actors[1]->getType() == PxActorType::eRIGID_STATIC;

    bool kinematicReport = false;
    if ((kinematic0 || static0) && (kinematic1 || static1))
    {
        kinematicReport = true;
    }

    if (!kinematicReport && contactCountSum > 0)
    {
        if (!mContactReport->checkThreshold(pairHeader.actors[0], pairHeader.actors[1], forceSum))
            return;
    }

    count = countIn;
    pairs = pairsIn;
    while (count--)
    {
        const PxContactPair& current = *pairs++;

        if (current.flags & PxContactPairFlag::eREMOVED_SHAPE_0 || current.flags & PxContactPairFlag::eREMOVED_SHAPE_1)
        {
            continue;
        }

        const PxU32 contactCount = current.contactCount;

        // friction anchors
        std::vector<PxContactPairFrictionAnchor> frictionAnchors;
        if (current.patchCount && current.contactPatches && current.frictionPatches)
        {
            const PxU32 maxNumAnchors = current.patchCount * PxFrictionPatch::MAX_ANCHOR_COUNT;            
            frictionAnchors.resize(maxNumAnchors);
            const PxU32 actualNumAnchors = current.extractFrictionAnchors(&frictionAnchors[0], maxNumAnchors);
            if (actualNumAnchors)
                frictionAnchors.resize(actualNumAnchors);
            else
                frictionAnchors.clear();
        }

        if (current.events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
            mContactReport->reportContact(pairHeader.actors[0], current.shapes[0], pairHeader.actors[1], current.shapes[1], eContactFound, contactCount, frictionAnchors);
        else if (current.events & PxPairFlag::eNOTIFY_TOUCH_LOST)
            mContactReport->reportContact(pairHeader.actors[0], current.shapes[0], pairHeader.actors[1], current.shapes[1], eContactLost, contactCount, frictionAnchors);
        else if (current.events & PxPairFlag::eNOTIFY_TOUCH_PERSISTS)
            mContactReport->reportContact(pairHeader.actors[0], current.shapes[0], pairHeader.actors[1], current.shapes[1], eContactPersists, contactCount, frictionAnchors);

        if (contactCount && current.contactPatches && current.contactPoints)
        {
            // contacts
            std::vector<PxContactPairPoint> contactPoints;
            contactPoints.resize(contactCount);
            current.extractContacts(&contactPoints[0], contactCount);

            internal::FaceIndexResolve indexResolve0(current.shapes[0]);
            internal::FaceIndexResolve indexResolve1(current.shapes[1]);

            for (PxU32 j = 0; j < contactCount; j++)
            {
                const uint32_t faceIndex0 = indexResolve0.resolveFaceIndex(contactPoints[j].internalFaceIndex0);
                const uint32_t faceIndex1 = indexResolve1.resolveFaceIndex(contactPoints[j].internalFaceIndex1);

                PxBaseMaterial* baseMaterial0 = current.shapes[0]->getMaterialFromInternalFaceIndex(contactPoints[j].internalFaceIndex0);
                PX_ASSERT(!baseMaterial0 || baseMaterial0->getConcreteType() == PxConcreteType::eMATERIAL);
                const PxMaterial* material0 = (baseMaterial0 != nullptr) ? baseMaterial0->is<PxMaterial>() : nullptr;

                PxBaseMaterial* baseMaterial1 = current.shapes[1]->getMaterialFromInternalFaceIndex(contactPoints[j].internalFaceIndex1);
                PX_ASSERT(!baseMaterial1 || baseMaterial1->getConcreteType() == PxConcreteType::eMATERIAL);
                const PxMaterial* material1 = (baseMaterial1 != nullptr) ? baseMaterial1->is<PxMaterial>() : nullptr;

                mContactReport->reportContactPoint(mPhysXScene, contactPoints[j].position, contactPoints[j].normal, contactPoints[j].impulse, contactPoints[j].separation, faceIndex0, faceIndex1, material0, material1);
            }
        }
    }
}

bool getSurfaceVelocities(internal::InternalPhysXDatabase& db,
                          const PxRigidActor& actor,
                          const PxContactSet& contacts,
                          PxVec3& linearVelocity,
                          PxVec3& angularVelocity,
                          PxTransform& angularVelocityPivot)
{
    const size_t index = size_t(actor.userData);
    if (index < db.getRecords().size())
    {
        const internal::InternalActor* intActor = (const internal::InternalActor*)db.getRecords()[index].mInternalPtr;
        if (intActor->mSplinesCurve)
        {
            CARB_PROFILE_ZONE(0, "ContactModify::splineEvaluation");
            // First transform the point to the spline local space
            const PxTransform splineWorldPose = actor.getGlobalPose() * intActor->mSplineLocalSpace;
            // Computation is expensive, we take just the first point
            const PxVec3 localPoint = splineWorldPose.transformInv(contacts.getPoint(0));

            GfVec3f pointOnCurve;
            GfVec3f tangent;
            const bool retVal = intActor->mSplinesCurve->getClosestPoint(toVec3f(localPoint), pointOnCurve, tangent);
            
            if (retVal)
            {
                linearVelocity = splineWorldPose.transform(toPhysX(tangent));
                linearVelocity.normalize();
                linearVelocity *= intActor->mSplinesSurfaceVelocityMagnitude;
            }
            else
            {
                linearVelocity = PxVec3(0.0f, 0.0f, 0.0f);
            }
            angularVelocity = PxVec3(0.0f, 0.0f, 0.0f);
            angularVelocityPivot = PxTransform(PxIdentity);
        }
        else
        {
            if (intActor->mSurfaceVelocityLocalSpace)
            {
                PxQuat q = actor.getGlobalPose().q;
                angularVelocity = q.rotate(intActor->mSurfaceAngularVelocity);
                linearVelocity = q.rotate(intActor->mSurfaceVelocity);
            }
            else
            {
                angularVelocity = intActor->mSurfaceAngularVelocity;
                linearVelocity = intActor->mSurfaceVelocity;
            }
            angularVelocityPivot = intActor->mSurfaceAngularVelocityPivot;
        }
        return true;
    }
    else
    {
        return false;
    }
}

void applyTargetVelocity(PxContactSet& contacts, const PxVec3& linearVelocity, const PxVec3& surfaceAngularVelocity, const PxTransform& surfaceAngularVelocityPivot)
{
    for (PxU32 i = 0; i < contacts.size(); ++i)
    {
        const PxVec3 distanceFromPivot = contacts.getPoint(i) - surfaceAngularVelocityPivot.p;
        const PxVec3 pivot = surfaceAngularVelocityPivot.p;
        const PxVec3 radius = contacts.getPoint(i) - pivot;
        const PxVec3 tangentVelocity = surfaceAngularVelocity.cross(radius);
        const PxVec3 compoundVelocity = linearVelocity + tangentVelocity;
        contacts.setTargetVelocity(i, compoundVelocity);
    }
}

void computeContactsSurfaceVelocities(internal::InternalPhysXDatabase& db,
                                      const PxRigidActor& actor,
                                      PxContactSet& contacts)
{
    PxVec3 linearVelocity;
    PxVec3 surfaceAngularVelocity;
    PxTransform surfaceAngularVelocityPivot;
    if (getSurfaceVelocities(db, actor, contacts, linearVelocity, surfaceAngularVelocity, surfaceAngularVelocityPivot))
    {
        applyTargetVelocity(contacts, linearVelocity, surfaceAngularVelocity, surfaceAngularVelocityPivot);
    }
}

void computeContactsSurfaceVelocities(internal::InternalPhysXDatabase& db,
                                      const PxRigidActor& actor0,
                                      const PxRigidActor& actor1,
                                      PxContactSet& contacts)
{
    PxVec3 linearVelocity0;
    PxVec3 surfaceAngularVelocity0;
    PxTransform surfaceAngularVelocityPivot0;
    const bool velocity0Valid = getSurfaceVelocities(
        db, actor0, contacts, linearVelocity0, surfaceAngularVelocity0, surfaceAngularVelocityPivot0);

    PxVec3 linearVelocity1;
    PxVec3 surfaceAngularVelocity1;
    PxTransform surfaceAngularVelocityPivot1;
    const bool velocity1Valid = getSurfaceVelocities(
        db, actor1, contacts, linearVelocity1, surfaceAngularVelocity1, surfaceAngularVelocityPivot1);
    if (velocity0Valid && velocity1Valid)
    {
        for (PxU32 i = 0; i < contacts.size(); ++i)
        {
            // ang vel0
            const PxVec3 distanceFromPivot0 = contacts.getPoint(i) - surfaceAngularVelocityPivot0.p;
            const PxVec3 pivot0 = surfaceAngularVelocityPivot0.p;
            const PxVec3 radius0 = contacts.getPoint(i) - pivot0;
            const PxVec3 tangentVelocity0 = surfaceAngularVelocity0.cross(radius0);

            // ang vel1
            const PxVec3 distanceFromPivot1 = contacts.getPoint(i) - surfaceAngularVelocityPivot1.p;
            const PxVec3 pivot1 = surfaceAngularVelocityPivot1.p;
            const PxVec3 radius1 = contacts.getPoint(i) - pivot1;
            const PxVec3 tangentVelocity1 = surfaceAngularVelocity1.cross(radius1);

            const PxVec3 compoundVelocity = linearVelocity0 + linearVelocity1 + tangentVelocity0 + tangentVelocity1;
            contacts.setTargetVelocity(i, compoundVelocity);
        }
    }
    else if (velocity0Valid)
    {
        applyTargetVelocity(contacts, linearVelocity0, surfaceAngularVelocity0, surfaceAngularVelocityPivot0);
    }
    else if (velocity1Valid)
    {
        applyTargetVelocity(contacts, linearVelocity1, surfaceAngularVelocity1, surfaceAngularVelocityPivot1);
    }
}


void OmniContactReportCallback::onContactModify(PxContactModifyPair* pairs, PxU32 count)
{
    for (PxU32 i = 0; i < count; i++)
    {
        PxContactModifyPair& pair = pairs[i];
        if (pair.shape[0]->getSimulationFilterData().word3 & CONTACT_MODIFY_SURFACE_VELOCITY && pair.shape[1]->getSimulationFilterData().word3 & CONTACT_MODIFY_SURFACE_VELOCITY)
        {
            computeContactsSurfaceVelocities(mOmniPhysX->getInternalPhysXDatabase(), * pair.actor[0], * pair.actor[1], pair.contacts);
        }
        else if (pair.shape[0]->getSimulationFilterData().word3 & CONTACT_MODIFY_SURFACE_VELOCITY)
        {
            computeContactsSurfaceVelocities(mOmniPhysX->getInternalPhysXDatabase(), * pair.actor[0], pair.contacts);
        }
        else if (pair.shape[1]->getSimulationFilterData().word3 & CONTACT_MODIFY_SURFACE_VELOCITY)
        {
            computeContactsSurfaceVelocities(mOmniPhysX->getInternalPhysXDatabase(), * pair.actor[1], pair.contacts);
        }
    }
}

class VehicleUpdateTask : public ::physx::PxLightCpuTask
{
public:
    VehicleUpdateTask()
    {
    }

    virtual ~VehicleUpdateTask()
    {
    }

    void init(std::atomic<int32_t>& vehicleProcessingIndex, InternalScene& internalScene,
        const float& stepSize)
    {
        mVehicleProcessingIndex = &vehicleProcessingIndex;
        mInternalScene = &internalScene;
        mStepSize = &stepSize;
    }

    virtual void run()
    {
        CARB_PROFILE_ZONE(0, "PhysXVehicleUpdateTask");

        const uint32_t enabledVehicleCount = mInternalScene->mEnabledVehicleCount;
        CARB_ASSERT(enabledVehicleCount > 0);  // else the task should not run

        uint32_t vehicleIndex = (*mVehicleProcessingIndex)++;
        if (vehicleIndex < enabledVehicleCount)
        {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();
            InternalVehicle** vehicles = mInternalScene->mVehicles.data();
            const InternalVehicleContext& internalContext = mInternalScene->getVehicleContext();
            ::physx::vehicle2::PxVehiclePhysXSimulationContext simContext = internalContext.getContext();

            do
            {
                InternalVehicle* vehicle = vehicles[vehicleIndex];
                PhysXActorVehicleBase* physxVehicle = vehicle->mPhysXVehicle;

                PhysXStepper::customizeVehicleSimulationContext(simContext,
                    *physxVehicle, physxSetup);

                physxVehicle->simulate(*mStepSize, simContext);

                vehicleIndex = (*mVehicleProcessingIndex)++;
            }
            while (vehicleIndex < enabledVehicleCount);
        }
    }

    virtual const char* getName() const
    {
        return "PhysXVehicleUpdate";
    }

private:
    std::atomic<int32_t>* mVehicleProcessingIndex;
    InternalScene* mInternalScene;
    const float* mStepSize;
};

PhysXStepper::PhysXStepper() : mComplete(true), mNbSubsteps(0), mStepSize(0.f), mPhysXScene(nullptr), mUpdateVelocities(true),
    mForceAsync(false), mSendStepping(false), mVehicleUpdateSyncTask(mVehicleSync), mSkipSimulation(false)
{
}

PhysXStepper::~PhysXStepper()
{
    for (size_t i = 0; i < mVehicleUpdateTasks.size(); i++)
    {
        mVehicleUpdateTasks[i]->~VehicleUpdateTask();
        ICE_FREE_BASIC(mVehicleUpdateTasks[i]);
    }
    mVehicleUpdateTasks.clear();
};

void PhysXStepper::customizeVehicleSimulationContext(::physx::vehicle2::PxVehiclePhysXSimulationContext& context,
    const PhysXActorVehicleBase& vehicle, PhysXSetup& physxSetup)
{
    if (vehicle.getRoadGeometryQueryType() == ::physx::vehicle2::PxVehiclePhysXRoadGeometryQueryType::eSWEEP)
    {
        // to ensure the code further below stays valid
        static_assert(::physx::vehicle2::PxVehicleAxes::ePosX == 0, "");
        static_assert(::physx::vehicle2::PxVehicleAxes::eNegX == 1, "");
        static_assert(::physx::vehicle2::PxVehicleAxes::ePosY == 2, "");
        static_assert(::physx::vehicle2::PxVehicleAxes::eNegY == 3, "");
        static_assert(::physx::vehicle2::PxVehicleAxes::ePosZ == 4, "");
        static_assert(::physx::vehicle2::PxVehicleAxes::eNegZ == 5, "");
        static_assert(usdparser::Axis::eX == 0, "");
        static_assert(usdparser::Axis::eY == 1, "");
        static_assert(usdparser::Axis::eZ == 2, "");
        usdparser::Axis axisEnum = static_cast<usdparser::Axis>(context.frame.latAxis >> 1);
        ::physx::PxConvexMesh* wheelMesh = physxSetup.getVehicleWheelCylinderConvexMesh(axisEnum);
        CARB_ASSERT(wheelMesh);

        context.physxUnitCylinderSweepMesh = wheelMesh;
    }
    else
        context.physxUnitCylinderSweepMesh = nullptr;

    context.tireSlipParams = vehicle.getTireSlipParams();

    const ::physx::vehicle2::PxVehicleTireStickyParams& tireStickyParams = vehicle.getTireStickyParams();
    context.tireStickyParams.stickyParams[::physx::vehicle2::PxVehicleTireDirectionModes::eLONGITUDINAL] =
        tireStickyParams.stickyParams[::physx::vehicle2::PxVehicleTireDirectionModes::eLONGITUDINAL];
    context.tireStickyParams.stickyParams[::physx::vehicle2::PxVehicleTireDirectionModes::eLATERAL] =
        tireStickyParams.stickyParams[::physx::vehicle2::PxVehicleTireDirectionModes::eLATERAL];
}

void PhysXStepper::createVehicleUpdateTasks()
{
    if (mVehicleUpdateTasks.size())
        return;
    else
    {
        InternalScene& internalScene = *mPhysXScene->getInternalScene();
        PxScene* pxScene = mPhysXScene->getScene();
        PxTaskManager* taskManager = pxScene->getTaskManager();
        const PxU32 workerCount = taskManager->getCpuDispatcher()->getWorkerCount();
        const PxU32 minTaskCount = PxMax(1u, workerCount);

        for (uint32_t i = 0; i < minTaskCount; i++)
        {
            VehicleUpdateTask* task = ICE_PLACEMENT_NEW(VehicleUpdateTask)();
            if (task)
            {
                task->init(mVehicleProcessingIndex, internalScene, mStepSize);

                mVehicleUpdateTasks.push_back(task);
            }
            else
            {
                CARB_LOG_ERROR("PhysX Vehicle: allocation of vehicle update task failed\n");
                return;
            }
        }
    }
}

void PhysXStepper::launch(int nbSubsteps, float stepSize, PhysXScene* scene, PxTaskManager* taskManager, bool substeppingEnabled, bool forceAsync, bool lastStep)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    mPhysXScene = scene;
    mNbSubsteps = nbSubsteps;
    mStepSize = stepSize;
    mLastStep = lastStep;
    mSubsteppingEnabled = substeppingEnabled;
    mForceAsync = forceAsync;
    mSendStepping = false;
    mPhysXScene->getContactReportCallback().mStepSize = stepSize;
    mComplete = false;
    mUpdateVelocities = omniPhysX.getCachedSettings().updateVelocitiesToUsd;
    mSceneQueryUpdateMode = scene->getSceneQueryUpdateMode();

    // skip for empty scenes
    mSkipSimulation = !omniPhysX.getCachedSettings().simulateEmptyScene && omniPhysX.getInternalPhysXDatabase().getRecords().size() <= 2;
    if (taskManager->getCpuDispatcher()->getWorkerCount() > 0)
    {
        if (!omniPhysX.getPhysXSetup().isPhysXCpuDispatcher())
        {
            omniPhysX.getITasking()->addTask(carb::tasking::Priority::eHigh, {}, [this] { run(); });
        }
        else
        {
            setContinuation(*taskManager, NULL);
            removeReference();
        }
    }
    else
    {
        run();
    }
}

void clearVelocities(PxActor& pxActor)
{
    if (pxActor.getType() == PxActorType::eRIGID_DYNAMIC)
    {
        PxRigidDynamic& rbo = (PxRigidDynamic&)pxActor;
        rbo.setLinearVelocity(PxVec3(PxZero));
        rbo.setAngularVelocity(PxVec3(PxZero));
    }
    else if (pxActor.getType() == PxActorType::eARTICULATION_LINK)
    {
        PxArticulationLink& link = (PxArticulationLink&)pxActor;
        PxArticulationJointReducedCoordinate* joint = link.getInboundJoint();
        if (joint)
        {
            for (PxU32 i = 0; i < PxArticulationAxis::eCOUNT; i++)
            {
                if (joint->getMotion((PxArticulationAxis::Enum)i) != PxArticulationMotion::eLOCKED)
                {
                    joint->setJointVelocity((PxArticulationAxis::Enum)i, 0.0f);
                }
            }
        }
    }
}

void PhysXStepper::updateQuasistaticActors(const PxU32 nbActors, PxActor** activeActors)
{
    if (mPhysXScene->getInternalScene()->getSceneDesc().quasistaticActors.empty())
    {
        for (PxU32 i = 0; i < nbActors; i++)
        {
            PxActor* pxActor = activeActors[i];
            clearVelocities(*pxActor);
        }
    }
    else
    {
        InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
        for (PxU32 i = 0; i < nbActors; i++)
        {
            PxActor* pxActor = activeActors[i];
            const size_t recordsIndex = (size_t)pxActor->userData;
            if (!pxActor || recordsIndex >= db.getRecords().size())
                continue;

            const InternalDatabase::Record& record = db.getRecords()[recordsIndex];
            const bool isLink = record.mType == ePTLink;
            const bool isActor = record.mType == ePTActor;
            if ((isActor || isLink) &&
                mPhysXScene->getInternalScene()->getSceneDesc().quasistaticActors.find(record.mPath) !=
                    mPhysXScene->getInternalScene()->getSceneDesc().quasistaticActors.end())
            {
                clearVelocities(*pxActor);
            }
        }
    }
}

void PhysXStepper::updateQuasistatic()
{
    CARB_PROFILE_ZONE(0, "PhysXUpdateQuasistatic");
    if (mPhysXScene->isReadbackSuppressed() || !mPhysXScene->getInternalScene()->getSceneDesc().enableQuasistatic)
    {
        return;
    }

    PxScene* scene = mPhysXScene->getScene();
    if (scene)
    {
        if (scene->getFlags() & PxSceneFlag::eENABLE_ACTIVE_ACTORS)
        {
            PxU32 nbActors = 0;
            PxActor** activeActors = scene->getActiveActors(nbActors);
            updateQuasistaticActors(nbActors, activeActors);
        }
        else
        {
            std::vector<PxActor*> dynamicActorsVector;
            std::vector<PxArticulationLink*> linksVector;

            // process all dynamic actors
            {
                const PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
                if (nbActors)
                {
                    dynamicActorsVector.resize(nbActors);
                    scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &dynamicActorsVector[0], nbActors);
                    updateQuasistaticActors(nbActors, &dynamicActorsVector[0]);
                }
            }

            // process all articulations
            {
                const PxU32 nbArticulations = scene->getNbArticulations();
                PxArticulationReducedCoordinate* art = nullptr;
                for (PxU32 i = 0; i < nbArticulations; i++)
                {
                    scene->getArticulations(&art, 1, i);
                    const PxU32 numLinks = art->getNbLinks();
                    if (numLinks)
                    {
                        linksVector.resize(numLinks);
                        art->getLinks(&linksVector[0], numLinks);
                        updateQuasistaticActors(numLinks, (PxActor**)(&linksVector[0]));
                    }
                }
            }
        }

    }
}

void PhysXStepper::updateForces()
{
    CARB_PROFILE_ZONE(0, "PhysXForcesUpdate");

    if (mPhysXScene->isReadbackSuppressed() && (mPhysXScene->getScene()->getTimestamp() > 1))
        return;

    const std::vector<InternalForce*>& forces = mPhysXScene->getInternalScene()->mForces;
    for (size_t i = forces.size(); i--;)
    {
        const InternalForce& force = *forces[i];
        if (force.mEnabled)
        {
            PxForceMode::Enum mode = force.mAccelerationMode ? PxForceMode::eACCELERATION : PxForceMode::eFORCE;
            PxRigidBody* rbo = nullptr;
            if (force.mRigidActor)
            {
                const PxActorType::Enum actorType = force.mRigidActor->getType();
                if (actorType == PxActorType::eRIGID_DYNAMIC || actorType == PxActorType::eARTICULATION_LINK)
                {
                    rbo = static_cast<PxRigidBody *>(force.mRigidActor);
                    if (rbo->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)
                    {
                        rbo = nullptr;
                    }
                }
            }
            if (force.mForceEnabled && rbo)
            {
                PxVec3 appliedForce = force.mWorldFrame ? force.getForce() : (rbo->getGlobalPose().q * force.mLocalRot).rotate(force.getForce());
                if (force.mCoMApplied)
                {
                    rbo->addForce(appliedForce, mode);
                }
                else
                {
                    const PxVec3 p = rbo->getGlobalPose().transform(force.mLocalPos);
                    if (force.mAccelerationMode)
                    {
                        appliedForce = appliedForce * rbo->getMass();
                    }
                    PxRigidBodyExt::addForceAtPos(*rbo, appliedForce, p, PxForceMode::eFORCE);
                }
            }
            if (force.mTorqueEnabled && rbo)
            {
                const PxVec3 appliedTorque = force.mWorldFrame ? force.getTorque() : (rbo->getGlobalPose().q * force.mLocalRot).rotate(force.getTorque());
                rbo->addTorque(appliedTorque, mode);
            }
        }
    }
}

void PhysXStepper::updateVehicles()
{
    CARB_PROFILE_ZONE(0, "PhysXVehicleSimUpdate");

    InternalScene& internalScene = *mPhysXScene->getInternalScene();

    const uint32_t enabledVehicleCount = internalScene.mEnabledVehicleCount;
    if (enabledVehicleCount > 0)
    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();
        InternalVehicle** vehicles = internalScene.mVehicles.data();
        InternalVehicleContext& internalContext = internalScene.getVehicleContext();
        ::physx::vehicle2::PxVehiclePhysXSimulationContext& simContext = internalContext.getContext();
        PxScene* pxScene = mPhysXScene->getScene();
        simContext.gravity = pxScene->getGravity();  // in case gravity is modified during simulation

        // creating a copy that can get modified per vehicle instance
        ::physx::vehicle2::PxVehiclePhysXSimulationContext simContextMod = simContext;

        {
            CARB_PROFILE_ZONE(0, "PhysXVehicleSimBeginUpdate");

            // run initial component in serial as PhysX does not support parallel writes to parameters

            for (uint32_t i = 0; i < enabledVehicleCount; i++)
            {
                PhysXActorVehicleBase* physxVehicle = vehicles[i]->mPhysXVehicle;

                PhysXStepper::customizeVehicleSimulationContext(simContextMod,
                    *physxVehicle, physxSetup);

                physxVehicle->simulateBegin(mStepSize, simContextMod);
            }
        }

        mVehicleProcessingIndex = 0;

        createVehicleUpdateTasks();

        const uint32_t maxTaskCount = static_cast<uint32_t>(mVehicleUpdateTasks.size());

        if (maxTaskCount > 1)
        {
            // for workerCount=1, this update call is already running on the worker and thus no need
            // to schedule the tasks

            PxTaskManager* taskManager = pxScene->getTaskManager();

            constexpr uint32_t minVehicleCountToLaunchTask = 4;
            const uint32_t tasksToRun = PxMin(maxTaskCount, (enabledVehicleCount / minVehicleCountToLaunchTask) + 1);

            // supporting both ITasking and PhysX dispatcher for now as Ales encountered cases
            // where ITasking was slow

            if (!omniPhysX.getPhysXSetup().isPhysXCpuDispatcher())
            {
                carb::tasking::ITasking* iTasking = omniPhysX.getITasking();
                iTasking->applyRangeBatch(tasksToRun, 1, [&](size_t begin, size_t end) {
                    for (; begin != end; ++begin)
                        mVehicleUpdateTasks[begin]->run();
                });
            }
            else
            {
                mVehicleSync.reset();

                mVehicleUpdateSyncTask.setContinuation(*taskManager, NULL);

                for (uint32_t i = 1; i < tasksToRun; i++)
                {
                    mVehicleUpdateTasks[i]->setContinuation(&mVehicleUpdateSyncTask);
                    mVehicleUpdateTasks[i]->removeReference();
                }
                mVehicleUpdateSyncTask.removeReference();

                // this update call is already running on a worker, so instead of just waiting
                // for all the tasks to finish, process a task directly.
                mVehicleUpdateTasks[0]->run();

                mVehicleSync.wait();
            }
        }
        else if (maxTaskCount > 0)
        {
            mVehicleUpdateTasks[0]->run();
        }

        {
            CARB_PROFILE_ZONE(0, "PhysXVehicleSimEndUpdate");

            // run final component in serial as PhysX does not support parallel writes to parameters

            for (uint32_t i = 0; i < enabledVehicleCount; i++)
            {
                PhysXActorVehicleBase* physxVehicle = vehicles[i]->mPhysXVehicle;

                PhysXStepper::customizeVehicleSimulationContext(simContextMod,
                    *physxVehicle, physxSetup);

                physxVehicle->simulateEnd(mStepSize, simContextMod);
            }
        }
    }
}

void PhysXStepper::updateParticles(int step, int numSteps)
{
    CARB_PROFILE_ZONE(0, "PhysXParticlesSimUpdate");
    InternalScene& internalScene = *mPhysXScene->getInternalScene();

    bool isLastSubstep = step == numSteps - 1;
    if (!isLastSubstep)
    {
        return;
    }

    // set flag to DMA back particle positions for rendering
    for (InternalPbdParticleSystem* internalPS : internalScene.mParticleSystems)
    {
        if (!internalPS->mEnabled)
            continue;

        internalPS->mParticleDataAvailable = true;
        internalPS->mAsyncSim = (mForceAsync || mPhysXScene->getUpdateType() == Asynchronous);
    }
}

void PhysXStepper::updateDeformables(int step, int numSteps)
{
    CARB_PROFILE_ZONE(0, "PhysXDeformableSimUpdate");
    bool isLastSubstep = step == numSteps - 1;
    if (!isLastSubstep)
        return;

    const bool updateUSD = OmniPhysX::getInstance().getCachedSettings().updateToUsd;
    if (!updateUSD)
        return; // AD is this the right thing to do? we might want the DtoH copy even if we don't write to USD..?

    SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
    const bool skipWriteTransforms = cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eSKIP_WRITE);
    if (skipWriteTransforms)
        return;

    PxCudaContextManager* cudaContextManager = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
    if (!cudaContextManager)
        return;

    PxScopedCudaLock _lock(*cudaContextManager);
    PxCudaContext* cudaCtx = cudaContextManager->getCudaContext();

    InternalScene& internalScene = *mPhysXScene->getInternalScene();
    CUstream copyStream = internalScene.getDeformableCopyStream();

    const bool updateVelocitiesToUsd = OmniPhysX::getInstance().getCachedSettings().updateVelocitiesToUsd;

    for (size_t i = 0; i < internalScene.mDeformableBodiesDeprecated.size(); ++i)
    {
        InternalDeformableBodyDeprecated* deformableBody = internalScene.mDeformableBodiesDeprecated[i];

        PxSoftBody* softBody = deformableBody->mSoftBody;

        if (!softBody->isSleeping() && !deformableBody->mSkipUpdateTransform)
        {
            cudaCtx->memcpyDtoHAsync(deformableBody->mCollPositionInvMassH, reinterpret_cast<CUdeviceptr>(softBody->getPositionInvMassBufferD()), deformableBody->mNumCollSkinMeshVertices * sizeof(PxVec4), copyStream);
            cudaCtx->memcpyDtoHAsync(deformableBody->mSimPositionInvMassH, reinterpret_cast<CUdeviceptr>(softBody->getSimPositionInvMassBufferD()), deformableBody->mNumSimMeshVertices * sizeof(PxVec4), copyStream);

            if (updateVelocitiesToUsd)
                cudaCtx->memcpyDtoHAsync(deformableBody->mSimVelocityH, reinterpret_cast<CUdeviceptr>(softBody->getSimVelocityBufferD()), deformableBody->mNumSimMeshVertices * sizeof(PxVec4), copyStream);
        }
    }

    for (size_t i = 0; i < internalScene.mDeformableSurfacesDeprecated.size(); ++i)
    {
        InternalDeformableSurfaceDeprecated* internalDeformableSurface = internalScene.mDeformableSurfacesDeprecated[i];
        PxDeformableSurface* deformableSurface = internalDeformableSurface->mDeformableSurface;

        if (!deformableSurface->isSleeping() && !internalDeformableSurface->mSkipUpdateTransform)
        {
            cudaCtx->memcpyDtoHAsync(internalDeformableSurface->mPositionInvMassH,
                reinterpret_cast<CUdeviceptr>(deformableSurface->getPositionInvMassBufferD()),
                internalDeformableSurface->mNumCollMeshVerticesWelded * sizeof(PxVec4), copyStream);

            if (updateVelocitiesToUsd)
                cudaCtx->memcpyDtoHAsync(internalDeformableSurface->mVelocityH,
                    reinterpret_cast<CUdeviceptr>(deformableSurface->getVelocityBufferD()),
                    internalDeformableSurface->mNumCollMeshVerticesWelded * sizeof(PxVec4), copyStream);
        }
    }
    
    for (size_t i = 0; i < internalScene.mVolumeDeformableBodies.size(); ++i)
    {
        InternalVolumeDeformableBody* deformableBody = internalScene.mVolumeDeformableBodies[i];

        PxDeformableVolume* deformableVolume = deformableBody->mDeformableVolume;

        if (deformableVolume && !deformableVolume->isSleeping())
        {
            cudaCtx->memcpyDtoHAsync(deformableBody->mSimMeshPositionInvMassH,
                reinterpret_cast<CUdeviceptr>(deformableVolume->getSimPositionInvMassBufferD()),
                deformableBody->mNumSimMeshVertices * sizeof(PxVec4), copyStream);

            cudaCtx->memcpyDtoHAsync(deformableBody->mCollMeshPositionInvMassH,
                                     reinterpret_cast<CUdeviceptr>(deformableVolume->getPositionInvMassBufferD()),
                                     deformableBody->mNumCollMeshVertices * sizeof(PxVec4), copyStream);

            if (updateVelocitiesToUsd)
            {
                cudaCtx->memcpyDtoHAsync(deformableBody->mSimMeshVelocityH,
                                         reinterpret_cast<CUdeviceptr>(deformableVolume->getSimVelocityBufferD()),
                                         deformableBody->mNumSimMeshVertices * sizeof(PxVec4), copyStream);
            }

            if (internalScene.mVolumeDeformablePostSolveCallback)
            {
                internalScene.mVolumeDeformablePostSolveCallback->copySkinnedVerticesDtoHAsync(i, deformableBody->mAllSkinnedVerticesH);
            }
        }
    }

    for (size_t i = 0; i < internalScene.mSurfaceDeformableBodies.size(); ++i)
    {
        InternalSurfaceDeformableBody* deformableBody = internalScene.mSurfaceDeformableBodies[i];
        PxDeformableSurface* deformableSurface = deformableBody->mDeformableSurface;
        if (deformableSurface && !deformableSurface->isSleeping())
        {
            cudaCtx->memcpyDtoHAsync(deformableBody->mSimMeshPositionInvMassH,
                reinterpret_cast<CUdeviceptr>(deformableSurface->getPositionInvMassBufferD()),
                deformableBody->mNumSimMeshVertices * sizeof(PxVec4), copyStream);

            if (updateVelocitiesToUsd)
            {
                cudaCtx->memcpyDtoHAsync(deformableBody->mSimMeshVelocityH,
                    reinterpret_cast<CUdeviceptr>(deformableSurface->getVelocityBufferD()),
                    deformableBody->mNumSimMeshVertices * sizeof(PxVec4), copyStream);
            }

            if (internalScene.mSurfaceDeformablePostSolveCallback)
            {
                internalScene.mSurfaceDeformablePostSolveCallback->copySkinnedVerticesDtoHAsync(i, deformableBody->mAllSkinnedVerticesH);
            }
        }
    }

    // Todo AD record event?
}

void PhysXStepper::run()
{
    // AD if this is starting to become an issue, consider adding a "deformables dirty" flag that only syncs
    // this stream if we know we actually wrote something to it.
    PxCudaContextManager* cudaContextManager = mPhysXScene->getScene()->getCudaContextManager();
    if (cudaContextManager)
    {
        PxScopedCudaLock _lock(*cudaContextManager);
        mPhysXScene->getInternalScene()->syncDeformableCopyStream(cudaContextManager);
    }

    OmniPhysX& omniPhysX = OmniPhysX::getInstance();

    for (int i = 0; i < mNbSubsteps; ++i)
    {
        if (mSubsteppingEnabled)
        {
            bool updateSceneQueries = true;
            if (mNbSubsteps > 1 && i != mNbSubsteps - 1)
            {
                updateSceneQueries = false;
            }
            else if (!mLastStep)
            {
                updateSceneQueries = false;
            }

            if (updateSceneQueries)
            {
                mPhysXScene->getScene()->setSceneQueryUpdateMode(mSceneQueryUpdateMode);
            }
            else
            {
                mPhysXScene->getScene()->setSceneQueryUpdateMode(::physx::PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED);
            }
        }

        updateForces();
        updateVehicles();
        updateParticles(i, mNbSubsteps);
        mPhysXScene->getContactReport()->clearBatchedData();

        omniPhysX.fireStatusEventSubscriptions(eSimulationStarting);
        {
            CARB_PROFILE_ZONE(0, "PhysX simulate");
            // artificially skip simulation if there is nothing to simulate
            if (!mSkipSimulation)
            {
                if (mPhysXScene->getUpdateType() == Asynchronous)
                {
                    CARB_PROFILE_ZONE(0, "pre-step update subscription update");
                    omniPhysX.fireOnStepEventSubscriptions(mStepSize, true);
                }

                mPhysXScene->getScene()->simulate(mStepSize);
                mPhysXScene->getScene()->fetchResults(true);
            }
            // A.B. This might not be actually correct, we might need this per scene in the end
            OmniPhysX::getInstance().increateSimulationTimestamp();
        }
        updateQuasistatic();
        omniPhysX.fireStatusEventSubscriptions(eSimulationEnded);

        // requests the DMA to CPU if lastSubStep, UpdateToUsd + no skipWriteTransforms.
        updateDeformables(i, mNbSubsteps);

        mPhysXScene->getContactReport()->flushContactReports();

        if (mForceAsync || mPhysXScene->getUpdateType() == Asynchronous)
        {
            if (mLastStep)
            {
                mPhysXScene->getScene()->fetchResultsParticleSystem();

                // Only sync here for async, otherwise this is done in updateDeformableTransforms.
                if (cudaContextManager)
                {
                    PxScopedCudaLock _lock(*cudaContextManager);
                    mPhysXScene->getInternalScene()->syncDeformableCopyStream(cudaContextManager);
                }
            }

            if (!mForceAsync)
            {
                {
                    CARB_PROFILE_ZONE(0, "post-step update subscription update");
                    omniPhysX.fireOnStepEventSubscriptions(mStepSize, false);
                }
            }
            else
            {
                mSendStepping = true;
            }
        }
    }

    // stop buffering USD changes and process them
    UsdLoad::getUsdLoad()->setAsyncUSDUpdate(false);
    UsdLoad::getUsdLoad()->processChanges();

    {
        std::lock_guard<carb::tasking::MutexWrapper> completeLock(mCompleteLock);        
        mCompleteCV.notify_all();
        mComplete = true;
    }
}

void PhysXStepper::waitForCompletion(bool sendPostEvents)
{
    std::unique_lock<carb::tasking::MutexWrapper> uniqueLock(mCompleteLock);
    while (!mComplete)
    {
        mCompleteCV.wait(mCompleteLock);
    }
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    if (sendPostEvents)
    {
        // A.B. this code should not run in parallel
        // send the events from the waiting thread to avoid deadlocking in python that is waiting for results.
        if (mSendStepping)
        {
            {
                CARB_PROFILE_ZONE(0, "post-step update subscription update");
                OmniPhysX::getInstance().fireOnStepEventSubscriptions(mStepSize, false);
            }
            mSendStepping = false;
        }
        if (mPhysXScene)
        {
            CARB_PROFILE_ZONE(0, "FireTriggerEvents");
            OmniPhysX::getInstance().getTriggerManager()->fireTriggerEvents(mPhysXScene->getAttachedStage());
        }
        {
            CARB_PROFILE_ZONE(0, "SimulationEventStreamPump");
            omniPhysX.getSimulationEventStreamV2()->pump();
        }
        if (mPhysXScene && mPhysXScene->getContactReport())
        {
            CARB_PROFILE_ZONE(0, "FlushBatchedContactReport");
            mPhysXScene->getContactReport()->flushBatchedContactReports();
        }
        {
            CARB_PROFILE_ZONE(0, "ErrorEventStreamPump");
            omniPhysX.getErrorEventStream()->pump();
        }
        // during asynchronous operation of the simulation, methods that want to safely read or write data
        // to the simulation can safely do so during an eSimulationComplete event.
        // For synchronous simulation this has to be called outside of this scope
        if (mPhysXScene && (mForceAsync || mPhysXScene->getUpdateType() == Asynchronous))
        {
            CARB_PROFILE_ZONE(0, "SimulationComplete");
            omniPhysX.fireStatusEventSubscriptions(eSimulationComplete);
        }
    }
    else
    {
        {
            CARB_PROFILE_ZONE(0, "SimulationEventStreamPump");
            omniPhysX.getSimulationEventStreamV2()->pump();
        }
        {
            CARB_PROFILE_ZONE(0, "ErrorEventStreamPump");
            omniPhysX.getErrorEventStream()->pump();
        }
    }

    mPhysXScene = nullptr;
}

void PhysXScene::launch(bool forceAsync)
{
    mPhysXStepper.launch(mCurrentStep, mCurrentTimeStep, this, getScene()->getTaskManager(), mSubsteppingEnabled, forceAsync, true);
    mCurrentStep = 0;
}

void PhysXScene::step()
{
    mPhysXStepper.launch(1, mCurrentTimeStep, this, getScene()->getTaskManager(), mSubsteppingEnabled, false, mCurrentStep == 1 ? true : false);
}

#if USE_PHYSX_GPU
static uint32_t roundUpToNextPowerOfTwo(uint32_t value)
{
    PxU32 pow2 = 1;
    while (pow2 < value)
        pow2 = pow2 << 1;
    return pow2;
}
#endif

static PxScene* createPhysicsScene(PhysXSetup& physxSetup, double metersPerUnit, const PhysxSceneDesc& physxSceneDesc,
    OmniFilterCallback* filterCallback, OmniContactReportCallback* reportCallback)
{
    PxScene* physxScene = nullptr;
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    carb::settings::ISettings* iSettings = omniPhysX.getISettings();
    ::physx::PxTolerancesScale tolerances = physxSetup.getDefaultTolerances(metersPerUnit);

    PxSceneDesc sceneDesc(tolerances);

    sceneDesc.flags |= PxSceneFlag::eENABLE_ACTIVE_ACTORS;
    sceneDesc.flags |= PxSceneFlag::eENABLE_BODY_ACCELERATIONS;

    if (physxSceneDesc.enableCCD)
        sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
    else
        sceneDesc.flags &= ~PxSceneFlag::eENABLE_CCD;

#if ENABLE_STABILIZATION_AND_CCD
    sceneDesc.flags |= PxSceneFlag::eENABLE_STABILIZATION | PxSceneFlag::eENABLE_CCD;
#endif

#if USE_PHYSX_GPU
    carb::Framework* framework = carb::getFramework();
    carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
    const bool suppressReadback = settings->getAsBool(kSettingSuppressReadback);
    const int multiGPUMode = settings->getAsInt(kSettingSceneMultiGPUMode);

    sceneDesc.cudaContextManager = multiGPUMode ? physxSetup.getNextCudaContextManager()  : physxSetup.getCudaContextManager();
    if (multiGPUMode)
    {
        CARB_ASSERT(sceneDesc.cudaContextManager);
        OMNI_LOG_WARN(kSceneMultiGPULogChannel, "Next PhysicsScene is assigned to %s", sceneDesc.cudaContextManager->getDeviceName());
    }

    if (omniPhysX.getGpuPipelineOverride() == -1)
    {
        if (physxSceneDesc.enableGPUDynamics)
            sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
    }
    else if (omniPhysX.getGpuPipelineOverride() > 0)
    {
        sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
    }

    if(physxSceneDesc.solveArticulationContactLast)
    {
        sceneDesc.flags |= PxSceneFlag::eSOLVE_ARTICULATION_CONTACT_LAST;
    }
    else
    {
        sceneDesc.flags &= ~PxSceneFlag::eSOLVE_ARTICULATION_CONTACT_LAST;
    }

    if (physxSceneDesc.enableResidualReporting)
        sceneDesc.flags |= PxSceneFlag::eENABLE_SOLVER_RESIDUAL_REPORTING;

    // suppress readback
    if (sceneDesc.flags.isSet(PxSceneFlag::eENABLE_GPU_DYNAMICS) && suppressReadback)
    {
        sceneDesc.flags |= PxSceneFlag::eENABLE_DIRECT_GPU_API;

        // Disable active actors feature if suppress readback is enabled
        sceneDesc.flags &= ~PxSceneFlag::eENABLE_ACTIVE_ACTORS;

        if (sceneDesc.flags & PxSceneFlag::eENABLE_CCD)
        {
            CARB_LOG_WARN("Physics suppress readback and physicsScene enabled CCD does not work together, CCD will be disabled.");
            sceneDesc.flags &= ~PxSceneFlag::eENABLE_CCD;
        }
    }

    sceneDesc.gpuDynamicsConfig.tempBufferCapacity = physxSceneDesc.gpuTempBufferCapacity;
    sceneDesc.gpuDynamicsConfig.maxRigidContactCount = physxSceneDesc.gpuMaxRigidContactCount;
    sceneDesc.gpuDynamicsConfig.maxRigidPatchCount = physxSceneDesc.gpuMaxRigidPatchCount;
    sceneDesc.gpuDynamicsConfig.heapCapacity = roundUpToNextPowerOfTwo(physxSceneDesc.gpuHeapCapacity);
    sceneDesc.gpuDynamicsConfig.foundLostPairsCapacity = physxSceneDesc.gpuFoundLostPairsCapacity;
    sceneDesc.gpuDynamicsConfig.foundLostAggregatePairsCapacity = physxSceneDesc.gpuFoundLostAggregatePairsCapacity;
    sceneDesc.gpuDynamicsConfig.totalAggregatePairsCapacity = physxSceneDesc.gpuTotalAggregatePairsCapacity;
    sceneDesc.gpuDynamicsConfig.maxDeformableVolumeContacts = physxSceneDesc.gpuMaxDeformableVolumeContacts;
    sceneDesc.gpuDynamicsConfig.maxDeformableSurfaceContacts = physxSceneDesc.gpuMaxDeformableSurfaceContacts;
    sceneDesc.gpuDynamicsConfig.maxParticleContacts = physxSceneDesc.gpuMaxParticleContacts;
    sceneDesc.gpuDynamicsConfig.collisionStackSize = physxSceneDesc.gpuCollisionStackSize;
    sceneDesc.gpuMaxNumPartitions = physxSceneDesc.gpuMaxNumPartitions;
#endif

    sceneDesc.bounceThresholdVelocity = physxSceneDesc.bounceThreshold;
    sceneDesc.frictionOffsetThreshold = physxSceneDesc.frictionOffsetThreshold;
    sceneDesc.frictionCorrelationDistance = physxSceneDesc.frictionCorrelationDistance;
    sceneDesc.maxBiasCoefficient = physxSceneDesc.maxBiasCoefficient;
    if (physxSceneDesc.enableEnhancedDeterminism)
        sceneDesc.flags |= PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
    else
        sceneDesc.flags &= ~PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
    if (physxSceneDesc.enableExternalForcesEveryIteration)
        sceneDesc.flags |= PxSceneFlag::eENABLE_EXTERNAL_FORCES_EVERY_ITERATION_TGS;
    else
        sceneDesc.flags &= ~PxSceneFlag::eENABLE_EXTERNAL_FORCES_EVERY_ITERATION_TGS;
    if (physxSceneDesc.enableStabilization)
        sceneDesc.flags |= PxSceneFlag::eENABLE_STABILIZATION;
    else
        sceneDesc.flags &= ~PxSceneFlag::eENABLE_STABILIZATION;
    if (physxSceneDesc.collisionSystem == ePCM)
        sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;
    else
        sceneDesc.flags &= ~PxSceneFlag::eENABLE_PCM;

    if (omniPhysX.getSolverTypeOverride() == -1)
    {
        if (physxSceneDesc.solverType == eTGS)
            sceneDesc.solverType = PxSolverType::eTGS;
        else
            sceneDesc.solverType = PxSolverType::ePGS;
    }
    else
    {
        if (omniPhysX.getSolverTypeOverride() > 0)
            sceneDesc.solverType = PxSolverType::eTGS;
        else
            sceneDesc.solverType = PxSolverType::ePGS;
    }

    if (omniPhysX.getGpuPipelineOverride() == -1)
    {
        if (physxSceneDesc.broadphaseType == eMBP)
            sceneDesc.broadPhaseType = PxBroadPhaseType::ePABP;
        else if (physxSceneDesc.broadphaseType == eGPU)
#if USE_PHYSX_GPU
            sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
#else
            sceneDesc.broadPhaseType = PxBroadPhaseType::ePABP;
#endif
        else if (physxSceneDesc.broadphaseType == eSAP)
            sceneDesc.broadPhaseType = PxBroadPhaseType::eSAP;
    }
    else if (omniPhysX.getGpuPipelineOverride() > 0)
    {
        sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
    }

    if (physxSceneDesc.reportKineKine)
    {
        sceneDesc.kineKineFilteringMode = PxPairFilteringMode::eKEEP;
    }
    if (physxSceneDesc.reportKineStatic)
    {
        sceneDesc.staticKineFilteringMode = PxPairFilteringMode::eKEEP;
    }

    if (!physxSceneDesc.supportSceneQueries)
    {
        sceneDesc.sceneQueryUpdateMode = PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED;
    }

    const int envIdInBoundsBitCount = physxSceneDesc.envIdInBoundsBitCount;
    //printf("envIdInBoundsBitCount: %d\n", envIdInBoundsBitCount);
	PxGpuBroadPhaseDesc gpuBPDesc;
	if(envIdInBoundsBitCount>=0 && sceneDesc.broadPhaseType == PxBroadPhaseType::eGPU)
	{
        // For simplicity we use the same number of bits everywhere for now.
		gpuBPDesc.gpuBroadPhaseNbBitsShiftX	= envIdInBoundsBitCount;
		gpuBPDesc.gpuBroadPhaseNbBitsShiftY	= envIdInBoundsBitCount;
		gpuBPDesc.gpuBroadPhaseNbBitsShiftZ	= envIdInBoundsBitCount;
		gpuBPDesc.gpuBroadPhaseNbBitsEnvIDX	= envIdInBoundsBitCount;
		gpuBPDesc.gpuBroadPhaseNbBitsEnvIDY	= envIdInBoundsBitCount;
		gpuBPDesc.gpuBroadPhaseNbBitsEnvIDZ	= envIdInBoundsBitCount;
			
		sceneDesc.gpuBroadPhaseDesc = &gpuBPDesc;
	}

    sceneDesc.cpuDispatcher = physxSetup.getCpuDispatcher();
    // sceneDesc.filterShader = PxDefaultSimulationFilterShader;
    sceneDesc.filterShader = OmniFilterShader;
    sceneDesc.filterCallback = filterCallback;
    sceneDesc.simulationEventCallback = reportCallback;
    sceneDesc.contactModifyCallback = reportCallback;

    if (!sceneDesc.isValid())
    {
        // A.B. this is an edge case we should guard for the paramters in the parsing
        CARB_LOG_ERROR("PhysicsScene does have invalid parameters, falling back to default parameters.");
        sceneDesc = PxSceneDesc(tolerances);
        sceneDesc.cpuDispatcher = physxSetup.getCpuDispatcher();
        sceneDesc.filterShader = OmniFilterShader;
        sceneDesc.filterCallback = filterCallback;
    }

    physxScene = physxSetup.getPhysics()->createScene(sceneDesc);

    if (!physxScene)
    {
#if USE_PHYSX_GPU
        if (physxSetup.getCudaContextManager() && physxSetup.getCudaContextManager()->getCudaContext()->isInAbortMode())
        {
            CARB_LOG_WARN("Not enough GPU memory available to create a PhysicsScene, falling back to CPU simulation.");
            sceneDesc.flags.clear(PxSceneFlag::eENABLE_GPU_DYNAMICS);
            sceneDesc.broadPhaseType = PxBroadPhaseType::ePABP;

            // AD: we don't reset the cudaContextManager skip state here - CPU sim does not need it and it will be
            // cleared on the next reset. After all, we're most likely still in a state where we cannot allocate memory.
            physxScene = physxSetup.getPhysics()->createScene(sceneDesc);
            CARB_ASSERT(physxScene);

            // reset the error counter.
            physxSetup.resetPhysXErrorCounter();
        }
#endif

        // AD: check again, maybe the fallback worked.
        if (!physxScene)
        {
            CARB_LOG_ERROR("unable to create a PhysicsScene.");
            return nullptr;
        }
    }

    if (physxSetup.getPvd() && physxScene)
    {
        PxPvdSceneClient* pvdClient = physxScene->getScenePvdClient();
        if (pvdClient)
        {
            pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
            pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
            pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
        }
    }

    if (omniPhysX.isDebugVisualizationEnabled())
    {
        physxScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, omniPhysX.getVisualizationScale());
        const uint32_t visMask = omniPhysX.getVisualizationBitMask();
        // A.B. store the size somewhere!
        for (uint32_t i = 0; i < 32; i++)
        {
            if (visMask & (1 << i))
            {
                physxScene->setVisualizationParameter(PxVisualizationParameter::Enum(i), 1.0f);
            }
        }
    }
    return physxScene;
}

PhysXScene::PhysXScene(const usdparser::AttachedStage& attachedStage)
    : mAttachedStage(attachedStage), mScene(nullptr), mControllerManager(nullptr), mInternalScene(nullptr), mTimeStepsPerSeconds(60),
      mSceneUpdateType(Synchronous), mMaterial(nullptr), mSoftBodyMaterialDeprecated(nullptr), mDeformableSurfaceMaterialDeprecated(nullptr),
      mVolumeDeformableMaterial(nullptr), mSurfaceDeformableMaterial(nullptr),
      mPBDMaterial(nullptr), mSceneQueryUpdateMode(PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_ENABLED), mRemaining(0.0f),
      mSubsteppingEnabled(false), mCurrentStep(0), mCurrentTimeStep(1.0f/60.0f), mNewScene(true), mContactReport(nullptr)
{
}

PhysXScene::~PhysXScene()
{
    SAFE_RELEASE(mInternalScene);

    SAFE_RELEASE(mMaterial)
    mMaterialPath = SdfPath();
    SAFE_RELEASE(mSoftBodyMaterialDeprecated);
    mSoftBodyMaterialPathDeprecated = SdfPath();
    SAFE_RELEASE(mDeformableSurfaceMaterialDeprecated);
    mDeformableSurfaceMaterialPathDeprecated = SdfPath();
    SAFE_RELEASE(mVolumeDeformableMaterial);
    mVolumeDeformableMaterialPath = SdfPath();
    SAFE_RELEASE(mSurfaceDeformableMaterial);
    mSurfaceDeformableMaterialPath = SdfPath();
    SAFE_RELEASE(mPBDMaterial);
    mPBDMaterialPath = SdfPath();

    SAFE_RELEASE(mControllerManager);
    SAFE_RELEASE(mScene);

    SAFE_RELEASE(mContactReport);
}

void PhysXScene::updateMirroredBodies()
{
    for (const InternalActor* actor : mInternalScene->mMirorredActors)
    {
        const PxTransform actorPose = actor->mActor->getGlobalPose();
        for (const MirrorActor& mirror : actor->mMirrors)
        {
            PxRigidDynamic* rboDyn = mirror.actor->is<PxRigidDynamic>();
            if (rboDyn)
                rboDyn->setKinematicTarget(actorPose);
        }
    }
}

void PhysXScene::computeSubstepping(float elapsedSecs, float fixedTimeStep, float timestepsPerSecond)
{
    uint32_t steps = (uint32_t)((elapsedSecs + mRemaining) * (1.f / fixedTimeStep));

    const uint32_t minFrameRate = OmniPhysX::getInstance().getCachedSettings().minFrameRate;
    if (minFrameRate)
    {
        const uint32_t floorSteps = (uint32_t)timestepsPerSecond / minFrameRate;
        const uint32_t maxIters = floorSteps > 0 ? floorSteps : 1;

        if (steps > maxIters)
            steps = maxIters;

        mRemaining = fminf((elapsedSecs + mRemaining) - (fixedTimeStep * steps), (maxIters)*fixedTimeStep);
    }
    else
    {
        mRemaining = (elapsedSecs + mRemaining) - (fixedTimeStep * steps);
    }

    mCurrentStep = steps;
    if (steps > 1)
    {
        mSubsteppingEnabled = true;
    }
    else
    {
        mSubsteppingEnabled = false;
    }
    mCurrentTimeStep = fixedTimeStep;

    mNewScene = false;
}


PhysXScene* PhysXScene::createPhysXScene(const usdparser::AttachedStage& attachedStage, size_t sceneId, PhysXSetup& physxSetup, double metersPerUnit, double kilogramsPerUnit,
    const usdparser::PhysxSceneDesc& sceneDesc)
{
    const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    if (sceneId >= db.getRecords().size())
        return nullptr;

    if (db.getRecords()[sceneId].mType != ePTScene)
        return nullptr;

    PhysXScene* scene = new PhysXScene(attachedStage);

    scene->mContactReport = new ContactReport();
    scene->mSceneSdfPath = db.getRecords()[sceneId].mPath;
    scene->mSupportSceneQuery = sceneDesc.supportSceneQueries;

    // setup callbacks
    scene->mFilterCallback.mInvertedCollisionFilter = sceneDesc.invertedFiltering;
    scene->mFilterCallback.mOmniPhysX = &OmniPhysX::getInstance();
    scene->mFilterCallback.mContactReport = scene->mContactReport;

    scene->mContactReportCallback.mOmniPhysX = &OmniPhysX::getInstance();
    scene->mContactReportCallback.mPhysXScene = scene;
    scene->mContactReportCallback.mContactReport = scene->mContactReport;

    PxScene* physXScene = createPhysicsScene(physxSetup, metersPerUnit, sceneDesc, &scene->mFilterCallback, &scene->mContactReportCallback);

    if (!physXScene)
    {
        SAFE_DELETE_SINGLE(scene->mContactReport);
        SAFE_DELETE_SINGLE(scene);
        return nullptr;
    }
    // set filterShader data
    physXScene->setFilterShaderData((const void*)(&scene), sizeof(const void*));
    scene->mScene = physXScene;
    internal::InternalScene* internalScene = ICE_NEW(internal::InternalScene)(sceneDesc, physXScene);
    scene->mInternalScene = internalScene;

    scene->mSceneQueryUpdateMode = physXScene->getSceneQueryUpdateMode();

    scene->mControllerManager = PxCreateControllerManager(*physXScene);
    const float mpu = float(metersPerUnit);
    const float kpu = float(kilogramsPerUnit);

    // default scene material
    {
        scene->mMaterial = physxSetup.getPhysics()->createMaterial(sceneDesc.defaultMaterialDesc.staticFriction, sceneDesc.defaultMaterialDesc.dynamicFriction,
            sceneDesc.defaultMaterialDesc.restitution);

        if (!scene->mMaterial)
        {
            CARB_LOG_ERROR("Failed to create default material: %s\n", sceneDesc.defaultMaterialDesc.materialPath.GetText());
            scene->mMaterial = physxSetup.getPhysics()->createMaterial(0.5f, 0.5f, 0.5f);
        }

        if (scene->mMaterial)
        {
            scene->mMaterial->setFrictionCombineMode(
                (PxCombineMode::Enum)sceneDesc.defaultMaterialDesc.frictionCombineMode);
            scene->mMaterial->setRestitutionCombineMode(
                (PxCombineMode::Enum)sceneDesc.defaultMaterialDesc.restitutionCombineMode);
            scene->mMaterial->setDampingCombineMode((PxCombineMode::Enum)sceneDesc.defaultMaterialDesc.dampingCombineMode);
        }
        scene->mMaterialPath = sceneDesc.defaultMaterialDesc.materialPath;
    }

#if USE_PHYSX_GPU
    // DEPRECATED
    {
        scene->mSoftBodyMaterialDeprecated = physxSetup.getPhysics()->createFEMSoftBodyMaterial(
            sceneDesc.defaultSoftBodyMaterialDesc.youngs * mpu / kpu, sceneDesc.defaultSoftBodyMaterialDesc.poissons,
            sceneDesc.defaultSoftBodyMaterialDesc.dynamicFriction);

        if (scene->mSoftBodyMaterialDeprecated)
        {
            scene->mSoftBodyMaterialDeprecated->setDamping(sceneDesc.defaultSoftBodyMaterialDesc.damping);
            scene->mSoftBodyMaterialDeprecated->setDampingScale(sceneDesc.defaultSoftBodyMaterialDesc.dampingScale);
        }
        scene->mSoftBodyMaterialPathDeprecated = sceneDesc.defaultSoftBodyMaterialDesc.materialPath;
    }
    {
        // no conversion here, we expect the descriptor's parameter to be in the correct units
        const PhysxDeformableMaterialDesc& defaultMat = sceneDesc.defaultDeformableMaterialDesc;
        scene->mVolumeDeformableMaterial = physxSetup.getPhysics()->createDeformableVolumeMaterial(
            defaultMat.youngsModulus, defaultMat.poissonsRatio, defaultMat.dynamicFriction, defaultMat.elasticityDamping);

        scene->mVolumeDeformableMaterialPath = defaultMat.materialPath;
    }
    // DEPRECATED
    {
        scene->mDeformableSurfaceMaterialDeprecated = physxSetup.getPhysics()->createDeformableSurfaceMaterial(
            sceneDesc.defaultFemClothMaterialDesc.youngs * mpu / kpu, sceneDesc.defaultFemClothMaterialDesc.poissons,
            sceneDesc.defaultFemClothMaterialDesc.dynamicFriction, sceneDesc.defaultFemClothMaterialDesc.thickness / mpu,

            //TODO: Not sure if the physical unit scaling of bendStiffness, elasticityDamping and defaultFemClothMaterialDesc is correct
            sceneDesc.defaultFemClothMaterialDesc.bendStiffness * mpu / kpu, sceneDesc.defaultFemClothMaterialDesc.elasticityDamping, sceneDesc.defaultFemClothMaterialDesc.bendDamping);

        scene->mDeformableSurfaceMaterialPathDeprecated = sceneDesc.defaultFemClothMaterialDesc.materialPath;
    }
    {
        // no conversion here, we expect the descriptor's parameter to be in the correct units
        const PhysxSurfaceDeformableMaterialDesc& defaultMat = sceneDesc.defaultSurfaceDeformableMaterialDesc;
        scene->mSurfaceDeformableMaterial = physxSetup.getPhysics()->createDeformableSurfaceMaterial(
            defaultMat.youngsModulus, defaultMat.poissonsRatio, defaultMat.dynamicFriction, defaultMat.surfaceThickness,
            defaultMat.surfaceBendStiffness, defaultMat.elasticityDamping, defaultMat.bendDamping);

        scene->mSurfaceDeformableMaterialPath = defaultMat.materialPath;
    }
    {
        scene->mPBDMaterial = physxSetup.getPhysics()->createPBDMaterial(sceneDesc.defaultPBDMaterialDesc.friction,
            sceneDesc.defaultPBDMaterialDesc.damping, sceneDesc.defaultPBDMaterialDesc.adhesion, sceneDesc.defaultPBDMaterialDesc.viscosity,
            sceneDesc.defaultPBDMaterialDesc.vorticityConfinement, sceneDesc.defaultPBDMaterialDesc.surfaceTension,
            sceneDesc.defaultPBDMaterialDesc.cohesion, sceneDesc.defaultPBDMaterialDesc.lift,
            sceneDesc.defaultPBDMaterialDesc.drag, sceneDesc.defaultPBDMaterialDesc.cflCoefficient, sceneDesc.defaultPBDMaterialDesc.gravityScale);

        if (scene->mPBDMaterial)
        {
            scene->mPBDMaterial->setParticleFrictionScale(sceneDesc.defaultPBDMaterialDesc.particleFrictionScale);
            scene->mPBDMaterial->setParticleAdhesionScale(sceneDesc.defaultPBDMaterialDesc.particleAdhesionScale);
            scene->mPBDMaterial->setAdhesionRadiusScale(sceneDesc.defaultPBDMaterialDesc.adhesionOffsetScale);
        }

        scene->mPBDMaterialPath = sceneDesc.defaultPBDMaterialDesc.materialPath;
    }
#endif // USE_PHYSX_GPU

    scene->mTimeStepsPerSeconds = sceneDesc.timeStepsPerSecond;
    scene->mSceneUpdateType = sceneDesc.sceneUpdateType;
    scene->mInvertedCollisionGroupFilter = sceneDesc.invertedFiltering;
    return scene;
}

void PhysXScene::resetDefaultMaterial()
{
    usdparser::PhysxMaterialDesc defaultMaterialDesc;
    usdparser::setToDefault(defaultMaterialDesc);    
    mMaterial = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->createMaterial(
        defaultMaterialDesc.staticFriction, defaultMaterialDesc.dynamicFriction,
        defaultMaterialDesc.restitution);
    mMaterialPath = SdfPath();
}

bool isRigidBodyDynamic(omni::physx::usdparser::ObjectId id)
{
    PhysXType internalType = ePTRemoved;
    const internal::InternalDatabase::Record* objectRecord = OmniPhysX::getInstance().getInternalPhysXDatabase().getFullRecord(internalType, id);
    if (!objectRecord)
        return false;

    if (internalType == ePTLink)
    {
        return true;
    }
    else if (internalType == ePTActor)
    {
        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor && actor->is<PxRigidDynamic>() && !(actor->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
            return true;
    }
    return false;
}

} // namespace physx
} // namespace omni
