// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "CameraFollowLook.h"

#include <omni/physx/IPhysx.h>
#include <common/foundation/Allocator.h>


using namespace physx;
using namespace pxr;


namespace omni
{
namespace physx
{


CameraFollowLook::CameraFollowLook(const pxr::UsdPrim& prim, const pxr::UsdPrim& subjectPrim)
    : CameraFollow(prim, subjectPrim)
{
    mInitialTransform = ::physx::PxTransform(PxIdentity);
}

CameraFollowLook::~CameraFollowLook()
{
}

CameraFollowLook* CameraFollowLook::create(const pxr::UsdPrim& prim, const pxr::UsdPrim& subjectPrim)
{
    CameraFollowLook* camera = ICE_PLACEMENT_NEW(CameraFollowLook)(prim, subjectPrim);

    return camera;
}

void CameraFollowLook::updateAxes()
{
    CameraFollow::updateAxes();

    PxRigidBody* rigidBody = nullptr;
    ::physx::PxRigidDynamic* dynamicActor = reinterpret_cast<::physx::PxRigidDynamic*>(gPhysXInterface->getPhysXPtr(mSubjectPath, ePTActor));

    if (dynamicActor && dynamicActor->is<PxRigidBody>())
    {
        rigidBody = static_cast<PxRigidBody*>(dynamicActor);
    }

    // Update vehicle states.
    if (rigidBody)
    {
        mInitialTransform = rigidBody->getGlobalPose();
        mInitialTransform.p = PxVec3(0.0f);
    }
    else
    {
        UsdGeomXform xform = UsdGeomXform(mSubjectPrim);
        GfMatrix4d subjectTransform = xform.ComputeLocalToWorldTransform(UsdTimeCode::Default());

        GfQuatd quaternion = subjectTransform.ExtractRotationQuat();

        // Convert to a PxTransform.
        PxVec3 p = PxVec3(0.0f);
        PxQuat q = PxQuat((float)quaternion.GetImaginary()[0], (float)quaternion.GetImaginary()[1],
            (float)quaternion.GetImaginary()[2], (float)quaternion.GetReal());

        mInitialTransform = ::physx::PxTransform(p, q);
    }

    mInitialTransform = mInitialTransform.getInverse();
}

void CameraFollowLook::readUsdSettings(const pxr::UsdTimeCode& timeCode)
{
    CameraFollow::readUsdSettings(timeCode);

    if (mCameraPrim.HasAPI<PhysxSchemaPhysxCameraFollowLookAPI>())
    {
        PhysxSchemaPhysxCameraFollowLookAPI followLookCamera(mCameraPrim);

        followLookCamera.GetDownHillGroundAngleAttr().Get(&mDownHillGroundAngle, timeCode);
        followLookCamera.GetDownHillGroundPitchAttr().Get(&mDownHillGroundPitch, timeCode);
        followLookCamera.GetUpHillGroundAngleAttr().Get(&mUpHillGroundAngle, timeCode);
        followLookCamera.GetUpHillGroundPitchAttr().Get(&mUpHillGroundPitch, timeCode);
        followLookCamera.GetVelocityBlendTimeConstantAttr().Get(&mVelocityBlendTimeConstant, timeCode);
        followLookCamera.GetFollowReverseSpeedAttr().Get(&mFollowReverseSpeed, timeCode);
        followLookCamera.GetFollowReverseDistanceAttr().Get(&mFollowReverseDistance, timeCode);
    }
}

::physx::PxTransform CameraFollowLook::updateTransform(float timeStep)
{
    PxRigidBody* rigidBody = nullptr;
    ::physx::PxRigidDynamic* dynamicActor = reinterpret_cast<::physx::PxRigidDynamic*>(gPhysXInterface->getPhysXPtr(mSubjectPath, ePTActor));

    if (dynamicActor && dynamicActor->is<PxRigidBody>())
    {
        rigidBody = static_cast<PxRigidBody*>(dynamicActor);
    }

    // Update vehicle states.
    if (rigidBody)
    {
        mSubjectTransform = rigidBody->getGlobalPose() * mInitialTransform;
        mSubjectVelocity = rigidBody->getLinearVelocity();
        mSubjectAngularVelocity = rigidBody->getAngularVelocity();
    }
    else if (timeStep > 0.0f)
    {
        UsdGeomXform xform = UsdGeomXform(mSubjectPrim);
        GfMatrix4d subjectTransform = xform.ComputeLocalToWorldTransform(UsdTimeCode::Default());

        GfQuatd quaternion = subjectTransform.ExtractRotationQuat();
        GfVec3d position = subjectTransform.ExtractTranslation();

        // Convert to a PxTransform.
        PxVec3 p = PxVec3((float)position[0], (float)position[1], (float)position[2]);
        PxQuat q = PxQuat((float)quaternion.GetImaginary()[0], (float)quaternion.GetImaginary()[1],
                          (float)quaternion.GetImaginary()[2], (float)quaternion.GetReal());

        if (mResetRates)
        {
            mSubjectAngularVelocity = { 0.0f, 0.0f, 0.0f };
            mSubjectVelocity = { 0.0f, 0.0f, 0.0f };

            mResetRates = false;
        }
        else
        {
            PxQuat difference = q * mSubjectTransform.q.getConjugate();
            PxVec3 angularDifference = difference.getImaginaryPart() * difference.getAngle();
            mSubjectAngularVelocity = angularDifference / timeStep;
            mSubjectVelocity = (p - mSubjectTransform.p) / timeStep;
        }

        mSubjectTransform = ::physx::PxTransform(p, q) * mInitialTransform;;
    }

    mSubjectLook = mSubjectTransform.rotate(mForward);
    mSubjectUp = mSubjectTransform.rotate(mUp);

    float cosPitchAngle = mSubjectUp.dot(mUp);
    mSubjectPitchAngle = std::acos(CARB_CLAMP(cosPitchAngle, -1.0f, 1.0f));

    mSubjectSpeed = mSubjectVelocity.magnitude();

    ::physx::PxVec3 bodyAngularVelocity = mSubjectTransform.rotateInv(mSubjectAngularVelocity);
    mSubjectYawRate = bodyAngularVelocity.dot(mUp);

    // Reset filter time constants in the update so they can be tuned in real-time.
    setupFilters();

    // look point
    updateActorPosition();
    updateLookPosition();

    // camera position
    updateFollowVectorAngles();
    updateFollowVector(timeStep);

    // filter and update
    updateFilters(timeStep);

    return computeTransform();
}

void CameraFollowLook::updateActorPosition()
{
    ::physx::PxVec3 positionOffset = mSubjectTransform.rotate(mPositionOffset);

    mSubjectReferencePosition = mSubjectTransform.p + positionOffset;
}

void CameraFollowLook::updateLookPosition()
{
    // Look further ahead as a function of subject mSubjectSpeed.
    float mSubjectSpeedInLookDirection = mSubjectVelocity.dot(mSubjectLook);
    float lookAheadDistance = Lerp(mLookAheadMinSpeed, mLookAheadMinDistance, mLookAheadMaxSpeed, mLookAheadMaxDistance,
                                   mSubjectSpeedInLookDirection);

    // Look into turns.
    float lookYawAngle = mLookAheadTurnRateGain * mYawRateFilter.getValue();

    PxQuat qYaw = ::physx::PxQuat(lookYawAngle, mUp);
    PxVec3 lookVector = qYaw.rotate(mSubjectLook);

    mLookPosition = mSubjectReferencePosition + lookAheadDistance * lookVector;

    // Adjust the look position height.
    mLookPosition += mLookPositionHeight * mSubjectUp;
}

void CameraFollowLook::updateFollowVectorAngles()
{
    // Swing the follow vector as a function of yaw rate.
    float swingYawAngle = mFollowTurnRateGain * mYawRateFilter.getValue();

    // Rotate the follow vector.
    mFollowVectorYaw = (float)GfDegreesToRadians(mYawAngle) + swingYawAngle;

    // Wrap yaw and pitch angles.

    // If the subject is moving slowly, lower the pitch angle for a better view.
    // At slow mSubjectSpeeds, the subject's lookahead point moves in very close. This has a better effect than
    // setting a minimum value for the lookahead point (as well as avoiding camera rotation problems).
    float pitchAngleScale = Lerp(0.0f, mSlowSpeedPitchAngleScale, mSlowPitchAngleSpeed, 1.0f, mSubjectSpeed);

    float hillPitchAngle = 0.0f;
    float subjectPitchAngle = mSubjectPitchFilter.getValue();

    if (mSubjectPitchAngle > 0.0f)
    {
        // Desensitize the pitch up and pitch down by using the square of the pitch angle.
        float pitchParameter = Lerp(0.0f, 0.0f, (float)GfDegreesToRadians(mUpHillGroundAngle), 1.0f, subjectPitchAngle);
        hillPitchAngle =
            Lerp(0.0f, 0.0f, 1.0f, (float)GfDegreesToRadians(mUpHillGroundPitch), pitchParameter * pitchParameter);
    }
    else
    {
        // Desensitize the pitch up and pitch down by using the square of the pitch angle.
        float pitchParameter = Lerp(0.0f, 0.0f, (float)GfDegreesToRadians(mDownHillGroundAngle), 1.0f, subjectPitchAngle);
        hillPitchAngle =
            Lerp(0.0f, 0.0f, 1.0f, (float)GfDegreesToRadians(mDownHillGroundPitch), pitchParameter * pitchParameter);
    }

    mFollowVectorPitch = pitchAngleScale * (float)GfDegreesToRadians(mPitchAngle) + hillPitchAngle;
}

void CameraFollowLook::updateFollowVector(float timeStep)
{
    // TODO: Need a method to determine when the subject is on the ground.
    bool onGround = true;
    CARB_UNUSED(onGround);

    // Blend between the mSubjectVelocity vector when airborne and the mSubjectLook vector on the ground.
    mLookVectorParameter = 0.0f;

    if (mSubjectSpeed > mVelocityNormalMinSpeed)
    {
        mLookVectorParameter = 1.0f;
    }

    PxVec3 lookVector = Lerp(mSubjectLook, mSubjectVelocity.getNormalized(), mLookVectorFilter.getValue());
    lookVector.normalize();

    // Rotate the follow vector.
    PxQuat qPitch = ::physx::PxQuat(mFollowVectorPitch, mLeft);
    PxQuat qYaw = ::physx::PxQuat(mFollowVectorYaw, mUp);

    PxVec3 pitchedLook = qPitch.rotate(mForward);
    PxVec3 localFollowVector = qYaw.rotate(pitchedLook);
    mFollowVector = mSubjectTransform.rotate(localFollowVector);

    // Move closer to the subject at higher mSubjectSpeeds as the filter causes the camera to follow further away.
    ::physx::PxVec3 bodyVelocity = mSubjectTransform.rotateInv(mSubjectVelocity);

    if (bodyVelocity.dot(mForward) < -mFollowMinSpeed)
    {
        mFollowDistance =
            Lerp(mFollowMinSpeed, mFollowMinDistance, mFollowReverseSpeed, mFollowReverseDistance, mSubjectSpeed);
    }
    else
    {
        mFollowDistance = Lerp(mFollowMinSpeed, mFollowMinDistance, mFollowMaxSpeed, mFollowMaxDistance, mSubjectSpeed);
    }

    mCameraPosition = mSubjectReferencePosition - mFollowDistance * mFollowVector;
}

} // namespace physx
} // namespace omni
