// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "CameraDrone.h"

#include <omni/physx/IPhysx.h>
#include <common/foundation/Allocator.h>


using namespace physx;
using namespace pxr;


namespace omni
{
namespace physx
{


CameraDrone::CameraDrone(const pxr::UsdPrim& prim, const pxr::UsdPrim& subjectPrim) : CameraController(prim, subjectPrim)
{
    // Subject states
    mSubjectVelocity = { 0.0f, 0.0f, 0.0f };

    mSubjectTransform = ::physx::PxTransform(PxIdentity);

    mCameraPosition = { 0.0f, 0.0f, 0.0f };
    mCameraVelocity = { 0.0f, 0.0f, 0.0f };

    mResetRates = true;

    // Camera
    mLookPosition = { 0.0f, 0.0f, 0.0f };

    mCameraRotation = { 0.0f, 0.0f, 0.0f, 0.0f };

    mResetFilters = true;
}

CameraDrone::~CameraDrone()
{
}

CameraDrone* CameraDrone::create(const pxr::UsdPrim& prim, const pxr::UsdPrim& subjectPrim)
{
    CameraDrone* camera = ICE_PLACEMENT_NEW(CameraDrone)(prim, subjectPrim);

    return camera;
}

void CameraDrone::readUsdSettings(const pxr::UsdTimeCode& timeCode)
{
    CameraController::readUsdSettings(timeCode);

    if (mCameraPrim.HasAPI<PhysxSchemaPhysxCameraDroneAPI>())
    {
        PhysxSchemaPhysxCameraDroneAPI droneCamera(mCameraPrim);
        PhysxSchemaPhysxCameraAPI cameraApi(droneCamera);

        GfVec3f vehiclePositionOffset;
        droneCamera.GetPositionOffsetAttr().Get(&vehiclePositionOffset, timeCode);

        droneCamera.GetFollowHeightAttr().Get(&mFollowHeight, timeCode);
        droneCamera.GetFollowDistanceAttr().Get(&mFollowDistance, timeCode);
        droneCamera.GetMaxDistanceAttr().Get(&mMaxDistance, timeCode);
        droneCamera.GetMaxSpeedAttr().Get(&mMaxSpeed, timeCode);
        droneCamera.GetHorizontalVelocityGainAttr().Get(&mHorizontalVelocityGain, timeCode);
        droneCamera.GetVerticalVelocityGainAttr().Get(&mVerticalVelocityGain, timeCode);
        droneCamera.GetFeedForwardVelocityGainAttr().Get(&mFeedForwardVelocityGain, timeCode);
        droneCamera.GetVelocityFilterTimeConstantAttr().Get(&mVelocityFilterTimeConstant, timeCode);   
        droneCamera.GetRotationFilterTimeConstantAttr().Get(&mRotationFilterTimeConstant, timeCode);

        mSubjectPositionOffset = ::physx::PxVec3(vehiclePositionOffset[0], vehiclePositionOffset[1], vehiclePositionOffset[2]);
    }
}

::physx::PxTransform CameraDrone::updateTransform(float timeStep)
{
    if (timeStep > 0.0f)
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
        }
        else
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
                mSubjectVelocity = { 0.0f, 0.0f, 0.0f };

                mResetRates = false;
            }
            else
            {
                mSubjectVelocity = (p - mSubjectTransform.p) / timeStep;
            }

            mSubjectTransform = ::physx::PxTransform(p, q);
        }

        updateCameraPosition(timeStep);
        ::physx::PxTransform transform = computeTransform(timeStep);

        mResetFilters = false;

        return transform;
    }
    else
    {
        return ::physx::PxTransform(PxIDENTITY::PxIdentity);
    }
}

::physx::PxVec3 CameraDrone::computeTargetPosition()
{
    ::physx::PxVec3 positionOffset = mSubjectTransform.rotate(mSubjectPositionOffset);
    mLookPosition = mSubjectTransform.p + positionOffset;

    // Initialize the camera position.
    if (mCameraPosition.magnitudeSquared() == 0.0f)
    {
        mCameraPosition = mLookPosition - mFollowDistance * mForward + mFollowHeight * mUp;
    }

    ::physx::PxVec3 vectorToSubject = mLookPosition - mCameraPosition;

    ::physx::PxVec3 flatVectorToSubject = vectorToSubject;
    flatVectorToSubject -= flatVectorToSubject.dot(mUp) * mUp;
    flatVectorToSubject.normalize();

    return mLookPosition - mFollowDistance * flatVectorToSubject + mFollowHeight * mUp;
}

void CameraDrone::updateCameraPosition(float timeStep)
{
    ::physx::PxVec3 targetPosition = computeTargetPosition();
    ::physx::PxVec3 vectorToTarget = targetPosition - mCameraPosition;

    // Compute the velocity to the target position.
    ::physx::PxVec3 velocityCommand = mHorizontalVelocityGain * vectorToTarget;
    velocityCommand += mFeedForwardVelocityGain * mSubjectVelocity;

    // Altitude control.
    float targetHeight = targetPosition.dot(mUp);

    /*
    float groundClearanceHeight = GroundHeight + mGroundClearanceHeight;

    float verticalVelocityGain = mVerticalVelocityGain;

    if (groundClearanceHeight > targetHeight)
    {
        targetHeight = groundClearanceHeight;
        verticalVelocityGain = mVerticalCollisionVelocityGain;
    }
    */

    float cameraHeight = mCameraPosition.dot(mUp);
    float heightDifference = targetHeight - cameraHeight;
    float climbCommand = mVerticalVelocityGain * heightDifference;

    velocityCommand -= velocityCommand.dot(mUp) * mUp;
    velocityCommand += climbCommand * mUp;

    if (velocityCommand.magnitude() > mMaxSpeed)
    {
        velocityCommand.normalize();
        velocityCommand *= mMaxSpeed;
    }

    if (mResetFilters)
    {
        mVelocityFilter.reset(velocityCommand);
    }

    // Filter the camera velocity.
    mVelocityFilter.setTimeConstant(::physx::PxVec3(mVelocityFilterTimeConstant));
    mCameraVelocity = mVelocityFilter.filter(velocityCommand, timeStep);

    // Integrate the velocity.
    mCameraPosition += mCameraVelocity * timeStep;

    // Clamp the maximum horizontal distance the camera can be from the vehicle.
    ::physx::PxVec3 vectorToSubject = mLookPosition - mCameraPosition;

    if (vectorToSubject.magnitude() > mMaxDistance)
    {
        vectorToSubject.normalize();
        mCameraPosition = mLookPosition - mMaxDistance * vectorToSubject;
    }
}

::physx::PxTransform CameraDrone::computeTransform(float timeStep)
{
    ::physx::PxTransform transform;

    transform.p = mCameraPosition;

    // Compute an orientation that looks at the look position.
    // The camera points along the negative z axis, so reverse the look vector.
    ::physx::PxVec3 lookVector = -(mLookPosition - mCameraPosition);
    lookVector.normalize();

    ::physx::PxVec3 leftVector = mUp.cross(lookVector);
    leftVector.normalize();

    ::physx::PxVec3 upVector = lookVector.cross(leftVector);
    upVector.normalize();

    ::physx::PxMat33 cameraMat = ::physx::PxMat33(leftVector, upVector, lookVector);
    transform.q = ::physx::PxQuat(cameraMat);

    ::physx::PxQuat targetRotation = ::physx::PxQuat(cameraMat);

    if (mResetFilters)
    {
        mCameraRotation = targetRotation;
    }

    if(timeStep < mRotationFilterTimeConstant)
    {
        transform.q = Slerp(mCameraRotation, targetRotation, timeStep / mRotationFilterTimeConstant);
    }
    else
    {
        transform.q = targetRotation;
    }
    mCameraRotation = transform.q;

    return transform;
}

}
}
