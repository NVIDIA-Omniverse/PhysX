// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "RigidBodyManipulatorHelper.h"

#include <common/utilities/Utilities.h>
#include <common/foundation/TypeCast.h>

#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>

#include <physxSchema/physxSceneAPI.h>

#include <private/omni/physx/IPhysxSupportUi.h>

#include "UsdUtils.h"

OMNI_LOG_DECLARE_CHANNEL(kSupportUiLogChannel)

using namespace pxr;

namespace omni
{
namespace physx
{

void RigidBodyManipulatorHelper::setStage(const UsdStageWeakPtr& stage)
{
    const UsdStage* stagePtr = stage ? &(*stage) : nullptr;
    OMNI_LOG_INFO(kSupportUiLogChannel, "RigidBodyManipulatorHelper::setStage(): stage \"%p\"", stagePtr);

    mStage = stage;
    setPxScene(nullptr);
}

void RigidBodyManipulatorHelper::setPxScene(::physx::PxScene* physxScene)
{
    OMNI_LOG_INFO(kSupportUiLogChannel, "RigidBodyManipulatorHelper::setPxScene(): PxScene \"%p\"", physxScene);

    mPhysxScene = physxScene;
}


RigidBodyManipulatorHelper::RigidBodyManipulatorHelper()
{
    mSettings = carb::getCachedInterface<carb::settings::ISettings>();
    CARB_ASSERT(mSettings != nullptr, "RigidBodyManipulatorHelper: Failed to obtain settings interface!");

    mPhysX = carb::getCachedInterface<omni::physx::IPhysx>();
    CARB_ASSERT(mPhysX != nullptr, "RigidBodyManipulatorHelper: Failed to obtain IPhysX interface!");
}

RigidBodyManipulatorHelper::~RigidBodyManipulatorHelper()
{
    restorePxSceneBackup();
    mActorManipulators.clear();
}

void RigidBodyManipulatorHelper::lockRotation(::physx::PxActor* pa, bool stateX, bool stateY, bool stateZ)
{
    ::physx::PxRigidDynamic* rd = pa->is<::physx::PxRigidDynamic>();
    if (rd)
    {
        rd->setRigidDynamicLockFlag(::physx::PxRigidDynamicLockFlag::eLOCK_ANGULAR_X, stateX);
        rd->setRigidDynamicLockFlag(::physx::PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y, stateY);
        rd->setRigidDynamicLockFlag(::physx::PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z, stateZ);
    }
}
void RigidBodyManipulatorHelper::lockTranslation(::physx::PxActor* pa, bool stateX, bool stateY, bool stateZ)
{
    ::physx::PxRigidDynamic* rd = pa->is<::physx::PxRigidDynamic>();
    if (rd)
    {
        rd->setRigidDynamicLockFlag(::physx::PxRigidDynamicLockFlag::eLOCK_LINEAR_X, stateX);
        rd->setRigidDynamicLockFlag(::physx::PxRigidDynamicLockFlag::eLOCK_LINEAR_Y, stateY);
        rd->setRigidDynamicLockFlag(::physx::PxRigidDynamicLockFlag::eLOCK_LINEAR_Z, stateZ);
    }
}

void RigidBodyManipulatorHelper::savePxSceneBackup()
{
    if (mPhysxScene != nullptr)
    {
        mSceneBackup.ccdEnabled = mPhysxScene->getFlags() & ::physx::PxSceneFlag::eENABLE_CCD;
    }
}

void RigidBodyManipulatorHelper::restorePxSceneBackup()
{
    if (mPhysxScene != nullptr)
    {
        bool ccdEnabled = mPhysxScene->getFlags() & ::physx::PxSceneFlag::eENABLE_CCD;
        if (mSceneBackup.ccdEnabled != ccdEnabled)
        {
            mPhysxScene->setFlag(::physx::PxSceneFlag::eENABLE_CCD, mSceneBackup.ccdEnabled);
        }
    }
}

void RigidBodyManipulatorHelper::onManipulationBegan(const pxr::SdfPath& path)
{
    OMNI_LOG_INFO(kSupportUiLogChannel,
                  "RigidBodyManipulatorHelper::onManipulationBegan() for: \"%s\"",
                  path.GetText());

    if (mPhysxScene != nullptr && mActorManipulators.size() == 0)
    {
        savePxSceneBackup();
    }

    mActorManipulators[path] = std::make_unique<ActorManipulator>(this, path);
}

void RigidBodyManipulatorHelper::onManipulationEnded(const pxr::SdfPath& path)
{
    ::physx::PxActor* pa = getPxActor(path);
    if (!pa)
    {
        OMNI_LOG_WARN(kSupportUiLogChannel,
                      "RigidBodyManipulatorHelper::onManipulationEnded(): Could not find PxActor at path: \"%s\"",
                      path.GetText());
        return;
    }

    OMNI_LOG_INFO(kSupportUiLogChannel,
                  "RigidBodyManipulatorHelper::onManipulationEnded() for: \"%s\"",
                  path.GetText());

    if (pa)
    {
        ::physx::PxRigidBody* rb = pa->is<::physx::PxRigidBody>();
        if (rb)
        {
            rb->clearForce(::physx::PxForceMode::eVELOCITY_CHANGE);
            rb->clearTorque(::physx::PxForceMode::eVELOCITY_CHANGE);
            // Effectively nullifies current velocities.
            const ::physx::PxVec3 linearVelocity = rb->getLinearVelocity();
            if (!linearVelocity.isFinite())
            {
                OMNI_LOG_ERROR("RigidBodyManipulatorHelper::onManipulationEnded(): Invalid linear velocity for PhysX Actor at path \"%s\"",
                              path.GetText());
            }
            else
            {
                rb->addForce(-linearVelocity, ::physx::PxForceMode::eVELOCITY_CHANGE);
            }

            const ::physx::PxVec3 angular_velocity = rb->getAngularVelocity();
            if (!angular_velocity.isFinite())
            {
                OMNI_LOG_ERROR("RigidBodyManipulatorHelper::onManipulationEnded(): Invalid angular velocity for PhysX Actor at path \"%s\"",
                              path.GetText());
            }
            else
            {
                rb->addTorque(-angular_velocity, ::physx::PxForceMode::eVELOCITY_CHANGE);
            }
        }
    }

    TActorManipulatorMap::const_iterator data = mActorManipulators.find(path);
    if (data != mActorManipulators.end())
    {
        mActorManipulators.erase(data);
    }

    if(mActorManipulators.size() == 0)
    {
        restorePxSceneBackup();
    }
}

void RigidBodyManipulatorHelper::update(float dt)
{
    if (dt == 0.0f)
    {
        return;
    }

    for (auto iter = mActorManipulators.begin(); iter != mActorManipulators.end();)
    {
        const pxr::SdfPath& path = iter->first;
        if(path.IsEmpty())
        {
            OMNI_LOG_WARN(kSupportUiLogChannel,
                          "RigidBodyManipulatorHelper::update() invalid path in actor list.");
            iter = mActorManipulators.erase(iter);
            continue;
        }
        ::physx::PxActor* pa = getPxActor(path); 
        if(!pa)
        {
            OMNI_LOG_WARN(kSupportUiLogChannel,
                "RigidBodyManipulatorHelper::update() failed to locate actor at path %s.", path.GetText());
            iter = mActorManipulators.erase(iter);
            continue;
        }

        ::physx::PxRigidBody* rb = pa->is<::physx::PxRigidBody>();
        if (!rb)
        {
            OMNI_LOG_WARN(kSupportUiLogChannel,
                          "RigidBodyManipulatorHelper::update() actor at path %s was not a rigid body.", path.GetText());
            iter = mActorManipulators.erase(iter);
            continue;
        }

        if (rb->getRigidBodyFlags() & ::physx::PxRigidBodyFlag::eKINEMATIC)
        {
            OMNI_LOG_WARN(kSupportUiLogChannel, "RigidBodyManipulatorHelper::update(): Kinematic actors are unsupported: \"%s\"",
                          path.GetText());
            iter = mActorManipulators.erase(iter);
            continue;
        }
        
        const ActorManipulator* data = iter->second.get();

        iter++;

        rb->clearForce(::physx::PxForceMode::eVELOCITY_CHANGE);
        rb->clearForce(::physx::PxForceMode::eACCELERATION);
        rb->clearTorque(::physx::PxForceMode::eVELOCITY_CHANGE);
        rb->clearTorque(::physx::PxForceMode::eACCELERATION);


        const ::physx::PxTransform rbWorldTransform = rb->getGlobalPose();
        if (!rbWorldTransform.isValid())
        {
            OMNI_LOG_ERROR("RigidBodyManipulatorHelper::update(): Invalid global pose for PhysX Actor at path \"%s\"",
                          path.GetText());
            continue;
        }

        const ::physx::PxVec3 linearVelocity = rb->getLinearVelocity();
        if (!linearVelocity.isFinite())
        {
            OMNI_LOG_ERROR("RigidBodyManipulatorHelper::update(): Invalid linear velocity for PhysX Actor at path \"%s\"",
                          path.GetText());
            continue;
        }

        const ::physx::PxVec3 angularVelocity = rb->getAngularVelocity();
        if (!angularVelocity.isFinite())
        {
            OMNI_LOG_ERROR("RigidBodyManipulatorHelper::update(): Invalid angular velocity for PhysX Actor at path \"%s\"",
                          path.GetText());
            continue;
        }

        ::physx::PxVec3 linearDelta = data->mTargetTranslation - rbWorldTransform.p;
        
        ::physx::PxQuat rotationDelta = data->mTargetRotation * rbWorldTransform.q.getConjugate();

        ::physx::PxF32 rotateAngleDelta;
        ::physx::PxVec3 rotateAxis;
        rotationDelta.toRadiansAndUnitAxis(rotateAngleDelta, rotateAxis);

        if (rotateAngleDelta >= ::physx::PxPi)
        {
            // If more than pi, rotate in opposite direction for the shortest angular distance.
            rotateAngleDelta -= 2.0f * ::physx::PxPi;
        }

        ::physx::PxVec3 angularDelta = rotateAxis * rotateAngleDelta;

        ::physx::PxRigidDynamic* rd = pa->is<::physx::PxRigidDynamic>();
        if (rd)
        {
            ::physx::PxRigidDynamicLockFlags rbDynLockFlags;
            if (data->bLockTranslation)
            {
                if(data->eType == RigidBodyManipulationType::eRotate)
                {
                    lockTranslation(pa, true, true, true);
                }
                else
                {
                    lockTranslation(pa, fabsf(linearDelta.x) <= kDistanceEpsilon, fabsf(linearDelta.y) <= kDistanceEpsilon, fabsf(linearDelta.z) <= kDistanceEpsilon);
                }
            }
            if (data->bLockRotation)
            {
                if(data->eType == RigidBodyManipulationType::eMove)
                {
                    lockRotation(pa, true, true, true);
                }
                else
                {
                    lockRotation(pa, fabsf(angularDelta.x) < kAngleEpsilon, fabsf(angularDelta.y ) < kAngleEpsilon, fabsf(angularDelta.z ) < kAngleEpsilon);
                }
            }
        }

        // The aim is to apply force that will bring us to desired position next kit update (unless we are simulating at a slower rate).
        float fTimestepFactor = std::min(60.0f, 1.0f / dt);
        ::physx::PxVec3 action = (data->eType == RigidBodyManipulationType::eMove ? linearDelta * fTimestepFactor * kMoveSmoothingFactor : ::physx::PxVec3(::physx::PxZero));
        ::physx::PxVec3 torque = (data->eType == RigidBodyManipulationType::eRotate ? angularDelta * fTimestepFactor * kRotateSmoothingFactor : ::physx::PxVec3(::physx::PxZero));

        // Cancel out existing velocities.
        action -= linearVelocity;
        torque -= angularVelocity;


        if(!action.isZero())
        {
            OMNI_LOG_INFO(kSupportUiLogChannel,
                          "RigidBodyManipulatorHelper::update(): Applying force to \"%s\": %f, %f, %f",
                          path.GetText(), action.x, action.y, action.z);
            rb->addForce(action, ::physx::PxForceMode::eVELOCITY_CHANGE);
        }

        if(!torque.isZero())
        {
            OMNI_LOG_INFO(kSupportUiLogChannel,
                          "RigidBodyManipulatorHelper::update(): Applying torque to \"%s\": %f, %f, %f",
                          path.GetText(), torque.x, torque.y, torque.z);

            rb->addTorque(torque, ::physx::PxForceMode::eVELOCITY_CHANGE);
        }
    }
}

bool RigidBodyManipulatorHelper::move(const pxr::SdfPath& path,
                                      const carb::Float3& deltaTranslation,
                                      bool lockRot,
                                      bool lockTrans)
{
    ::physx::PxActor* pa = getPxActor(path);

    if (!pa)
    {
        OMNI_LOG_ERROR(kSupportUiLogChannel,
                      "RigidBodyManipulatorHelper::move(): Could not find any PhysX Actor at path \"%s\"",
                      path.GetText());
        return false;
    }

    ::physx::PxRigidBody* rb = pa->is<::physx::PxRigidBody>();
    if (!rb)
    {
        OMNI_LOG_ERROR(kSupportUiLogChannel,
                      "RigidBodyManipulatorHelper::move(): Did not find a rigid body associated with PhysX Actor \"%s\"",
                      path.GetText());
        return false;
    }

    TActorManipulatorMap::iterator iter = mActorManipulators.find(path);
    if (iter == mActorManipulators.end())
    {
        OMNI_LOG_ERROR(kSupportUiLogChannel, "RigidBodyManipulatorHelper::move(): Could not find manipulation entry for PhysX Actor \"%s\". Did you remember to call manipulation_began for the path?",
                        path.GetText());
        return false;
    }

    ActorManipulator* data = iter->second.get();
    data->eType = RigidBodyManipulationType::eMove;

    const ::physx::PxTransform rbWorldTransform = rb->getGlobalPose();
    if (!rbWorldTransform.isValid())
    {
        OMNI_LOG_ERROR("RigidBodyManipulatorHelper::move(): Invalid global pose for PhysX Actor at path \"%s\"",
                      path.GetText());
        return false;
    }

    data->mTargetTranslation = rbWorldTransform.p + asPhysX(deltaTranslation);
    data->bLockRotation = lockRot;
    data->bLockTranslation = lockTrans;

    rb->setLinearDamping(0.0f);
    rb->setAngularDamping(0.0f);
    rb->setMaxAngularVelocity(kMoveMaxAngularVelocity);
    rb->setMaxLinearVelocity(kMoveMaxLinearVelocity);
    return true;
}

bool RigidBodyManipulatorHelper::rotate(const pxr::SdfPath& path,
                                        const carb::Float3& pivotWorldPos,
                                        const carb::Float4& deltaRotation,
                                        bool lockRot,
                                        bool lockTrans)
{

    ::physx::PxActor* pa = getPxActor(path);

    if (!pa)
    {
        OMNI_LOG_ERROR(kSupportUiLogChannel,
                      "RigidBodyManipulatorHelper::rotate(): Could not find any PhysX Actor at path \"%s\"",
                      path.GetText());
        return false;
    }

    ::physx::PxRigidBody* rb = pa->is<::physx::PxRigidBody>();
    if (!rb)
    {
        OMNI_LOG_ERROR(kSupportUiLogChannel,
                      "RigidBodyManipulatorHelper::rotate(): Did not find a rigid body associated with PhysX Actor \"%s\"",
                      path.GetText());
        return false;
    }

    TActorManipulatorMap::iterator iter = mActorManipulators.find(path);
    if (iter == mActorManipulators.end())
    {
        OMNI_LOG_ERROR(kSupportUiLogChannel, "RigidBodyManipulatorHelper::rotate(): Could not find manipulation entry for PhysX Actor \"%s\". Did you remember to call manipulation_began for the path?",
                        path.GetText());
        return false;
    }

    ActorManipulator* data = iter->second.get();

    data->eType = RigidBodyManipulationType::eRotate;

    ::physx::PxQuat angularDelta = toPhysXQuat(deltaRotation);

    const ::physx::PxTransform rbWorldTransform = rb->getGlobalPose();
    if (!rbWorldTransform.isValid())
    {
        OMNI_LOG_ERROR("RigidBodyManipulatorHelper::rotate(): Invalid global pose for PhysX Actor at path \"%s\"",
                      path.GetText());
        return false;
    }

    data->mTargetRotation = angularDelta * rbWorldTransform.q;

    data->mRotationPivot = asPhysX(pivotWorldPos);
    data->bLockRotation = lockRot;
    data->bLockTranslation = lockTrans;

    if (rb->getConcreteType() != ::physx::PxConcreteType::eARTICULATION_LINK)
    {
        // Reposition our center of mass to the pivot position. This allows us to rotate about the pivot.
        const ::physx::PxTransform rbWorldTransform = rb->getGlobalPose();
        ::physx::PxTransform pivot = ::physx::PxTransform(::physx::PxIdentity);
        pivot.p = data->mRotationPivot - rbWorldTransform.p;
        pivot.p = rbWorldTransform.q.rotateInv(pivot.p);
        rb->setCMassLocalPose(pivot);
    }
    rb->setLinearDamping(0.0f);
    rb->setAngularDamping(0.0f);
    rb->setMaxAngularVelocity(kRotateMaxAngularVelocity);
    rb->setMaxLinearVelocity(kRotateMaxLinearVelocity);

    return true;
}

::physx::PxActor* RigidBodyManipulatorHelper::getPxActor(const pxr::SdfPath& path) const
{
    ::physx::PxActor* actor = nullptr;

    actor = reinterpret_cast<::physx::PxActor*>(mPhysX->getPhysXPtr(path, omni::physx::PhysXType::ePTActor));

    if (!actor)
    {
        actor = reinterpret_cast<::physx::PxActor*>(mPhysX->getPhysXPtr(path, omni::physx::PhysXType::ePTLink));
    }

    if (actor)
    {
        return actor;
    }

    OMNI_LOG_INFO(kSupportUiLogChannel,
                  "RigidBodyManipulatorHelper::getPxActor(): Did not find a PhysX Actor associated with: \"%s\"",
                  path.GetText());

    return nullptr;
}

RigidBodyManipulatorHelper::ActorManipulator::ActorManipulator(RigidBodyManipulatorHelper* helper, const pxr::SdfPath& path) :
                        helper(helper),
                        path(path),
                        mTargetTranslation(::physx::PxVec3(::physx::PxZero)),
                        mTargetRotation(::physx::PxQuat(::physx::PxIdentity)),
                        mRotationPivot(::physx::PxVec3(::physx::PxZero)),
                        bLockTranslation(false),
                        bLockRotation(false),
                        eType(RigidBodyManipulationType::eNone)
{
    ::physx::PxActor* actor = helper->getPxActor(path);
    if(!actor)
    {
        OMNI_LOG_ERROR(kSupportUiLogChannel,
                      "RigidBodyManipulatorHelper::ActorManipulator creation failed - could not find any PhysX Actor at path \"%s\"",
                      path.GetText());
        return;
    }

    OMNI_LOG_INFO(kSupportUiLogChannel,
                    "RigidBodyManipulatorHelper::ActorManipulator created for path \"%s\"",
                    path.GetText());

    omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(helper->mStage, helper->mStage->GetSessionLayer());

    backUp.mPxActorFlags = actor->getActorFlags();
    actor->setActorFlag(::physx::PxActorFlag::eDISABLE_GRAVITY, true);

    ::physx::PxRigidBody* rb = actor->is<::physx::PxRigidBody>();
    if (rb)
    {
        backUp.mPxRigidBodyFlags = rb->getRigidBodyFlags();
        backUp.mAngularDamping = rb->getAngularDamping();
        backUp.mMaxAngularVelocity = rb->getMaxAngularVelocity();
        backUp.mLinearDamping = rb->getLinearDamping();
        backUp.mMaxLinearVelocity = rb->getMaxLinearVelocity();
        backUp.mPxCenterOfMass = rb->getCMassLocalPose();

        const ::physx::PxRigidDynamic* rd = actor->is<::physx::PxRigidDynamic>();
        if (rd)
        {
            backUp.mPxRigidDynamicLockFlags = rd->getRigidDynamicLockFlags();
        }

        auto rbFlags = rb->getRigidBodyFlags();
        bool isKinematic = rbFlags & ::physx::PxRigidBodyFlag::eKINEMATIC ? true : false;
        if (!isKinematic)
        {
            rb->setRigidBodyFlag(::physx::PxRigidBodyFlag::eENABLE_CCD, true);
            rb->setRigidBodyFlag(::physx::PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, true);

        }
    }
}

RigidBodyManipulatorHelper::ActorManipulator::~ActorManipulator()
{

    ::physx::PxActor* actor = helper->getPxActor(path);

    if(!actor)
    {
        OMNI_LOG_WARN(kSupportUiLogChannel,
                      "RigidBodyManipulatorHelper::ActorManipulator removal failed - could not find any PhysX Actor at path \"%s\"",
                      path.GetText());
        return;
    }

    OMNI_LOG_INFO(kSupportUiLogChannel,
                    "RigidBodyManipulatorHelper::ActorManipulator removed for path \"%s\"",
                    path.GetText());

    omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(helper->mStage, helper->mStage->GetSessionLayer());

    actor->setActorFlags(backUp.mPxActorFlags);

    ::physx::PxRigidBody* rb = actor->is<::physx::PxRigidBody>();
    if (rb)
    {
        rb->setRigidBodyFlags(backUp.mPxRigidBodyFlags);
        rb->setAngularDamping(backUp.mAngularDamping);
        rb->setMaxAngularVelocity(backUp.mMaxAngularVelocity);
        rb->setLinearDamping(backUp.mLinearDamping);
        rb->setMaxLinearVelocity(backUp.mMaxLinearVelocity);
        rb->setCMassLocalPose(backUp.mPxCenterOfMass);

        ::physx::PxRigidDynamic* rd = actor->is<::physx::PxRigidDynamic>();
        if (rd)
        {
            rd->setRigidDynamicLockFlags(backUp.mPxRigidDynamicLockFlags);
        }
    }
}

} // namespace physx
} // namespace omni
