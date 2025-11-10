// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "CameraFollowVelocity.h"

#include <omni/physx/IPhysx.h>
#include <common/foundation/Allocator.h>


using namespace physx;
using namespace pxr;


namespace omni
{
namespace physx
{


CameraFollowVelocity::CameraFollowVelocity(const pxr::UsdPrim& prim, const pxr::UsdPrim& subjectPrim)
    : CameraFollow(prim, subjectPrim)
{
}

CameraFollowVelocity::~CameraFollowVelocity()
{
}

CameraFollowVelocity* CameraFollowVelocity::create(const pxr::UsdPrim& prim, const pxr::UsdPrim& subjectPrim)
{
    CameraFollowVelocity* camera = ICE_PLACEMENT_NEW(CameraFollowVelocity)(prim, subjectPrim);

    return camera;
}

::physx::PxTransform CameraFollowVelocity::updateTransform(float timeStep)
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
        mSubjectTransform = rigidBody->getGlobalPose();
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

        mSubjectTransform = ::physx::PxTransform(p, q);
    }

    mSubjectSpeed = mSubjectVelocity.magnitude();

    if (mSubjectSpeed > mVelocityNormalMinSpeed)
    {
        mSubjectLook = mSubjectVelocity;

        // Remove the vertical component of the look vector.
        mSubjectLook -= mSubjectLook.dot(mUp) * mUp;
        mSubjectLook.normalize();

        ::physx::PxVec3 left = mUp.cross(mSubjectLook);
        mSubjectUp = mSubjectLook.cross(left);
    }

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

void CameraFollowVelocity::updateActorPosition()
{
    mSubjectReferencePosition = mSubjectTransform.p + mPositionOffset;
}

void CameraFollowVelocity::updateLookPosition()
{
    // Look further ahead as a function of subject mSubjectSpeed.
    float mSubjectSpeedInLookDirection = mSubjectVelocity.dot(mSubjectLook);
    float lookAheadDistance = Lerp(mLookAheadMinSpeed, mLookAheadMinDistance, mLookAheadMaxSpeed, mLookAheadMaxDistance,
                                   mSubjectSpeedInLookDirection);

    // Look into turns.
    float lookYawAngle = 0.0f; //  mLookAheadTurnRateGain * mYawRateFilter.getValue();

    PxQuat qYaw = ::physx::PxQuat(lookYawAngle, mUp);
    PxVec3 lookVector = qYaw.rotate(mSubjectLook);

    mLookPosition = mSubjectReferencePosition + lookAheadDistance * lookVector;

    // Adjust the look position height.
    mLookPosition += mLookPositionHeight * mSubjectUp;
}

void CameraFollowVelocity::updateFollowVectorAngles()
{
    // Swing the follow vector as a function of yaw rate.
    float swingYawAngle = mFollowTurnRateGain * mYawRateFilter.getValue();

    // Rotate the follow vector.
    mFollowVectorYaw = (float)GfDegreesToRadians(mYawAngle) + swingYawAngle;
    mFollowVectorPitch = (float)GfDegreesToRadians(mPitchAngle);
}

void CameraFollowVelocity::updateFollowVector(float timeStep)
{
    // TODO: Need a method to determine when the subject is on the ground.
    bool onGround = true;
    CARB_UNUSED(onGround);

    // Rotate the follow vector.
    ::physx::PxVec3 left = mSubjectUp.cross(mSubjectLook);
    PxQuat qPitch = ::physx::PxQuat(mFollowVectorPitch, left);
    PxQuat qYaw = ::physx::PxQuat(mFollowVectorYaw, mSubjectUp);

    PxVec3 pitchedLook = qPitch.rotate(mSubjectLook);
    mFollowVector = qYaw.rotate(pitchedLook);

    // Move closer to the subject at higher mSubjectSpeeds as the filter causes the camera to follow further away.
    mFollowDistance = Lerp(mFollowMinSpeed, mFollowMinDistance, mFollowMaxSpeed, mFollowMaxDistance, mSubjectSpeed);

    mCameraPosition = mSubjectReferencePosition - mFollowDistance * mFollowVector;
}

} // namespace physx
} // namespace omni
