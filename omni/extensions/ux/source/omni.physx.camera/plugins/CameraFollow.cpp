// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "CameraFollow.h"

#include <omni/physx/IPhysx.h>
#include <common/foundation/Allocator.h>


using namespace physx;
using namespace pxr;


namespace omni
{
namespace physx
{


CameraFollow::CameraFollow(const pxr::UsdPrim& prim, const pxr::UsdPrim& subjectPrim)
    : CameraController(prim, subjectPrim)
{
    // Subject states
    mSubjectLook = { 0.0f, 0.0f, 0.0f };
    mSubjectUp = { 0.0f, 0.0f, 0.0f };

    mSubjectPitchAngle = 0.0f;

    mSubjectVelocity = { 0.0f, 0.0f, 0.0f };
    mSubjectAngularVelocity = { 0.0f, 0.0f, 0.0f };
    mSubjectSpeed = 0.0f;

    mSubjectBodyAngularVelocity = { 0.0f, 0.0f, 0.0f };
    mSubjectYawRate = 0.0f;

    mSubjectTransform = ::physx::PxTransform(PxIdentity);

    mResetRates = true;
    mResetRates = true;

    // Camera
    mSubjectReferencePosition = { 0.0f, 0.0f, 0.0f };
    mFollowVector = { 0.0f, 0.0f, 0.0f };
    mLookPosition = { 0.0f, 0.0f, 0.0f };

    mFollowDistance = 0.0f;
    mFollowDistanceOffset = 0.0f;

    mFollowVectorPitch = 0.0f;
    mFollowVectorYaw = 0.0f;

    mLookVectorParameter = 0.0f;
    mResetFilters = true;
}

CameraFollow::~CameraFollow()
{
}

void CameraFollow::updateAxes()
{
    CameraController::updateAxes();

    // Subject states
    mSubjectLook = mForward;
    mSubjectUp = mUp;
}

void CameraFollow::readUsdSettings(const pxr::UsdTimeCode& timeCode)
{
    CameraController::readUsdSettings(timeCode);

    if (mCameraPrim.HasAPI<PhysxSchemaPhysxCameraFollowAPI>())
    {
        PhysxSchemaPhysxCameraFollowAPI followCamera(mCameraPrim);
        PhysxSchemaPhysxCameraAPI cameraApi(followCamera);

        GfVec3f cameraPositionTimeConstant;
        GfVec3f subjectPositionOffset;
        GfVec3f lookPositionTimeConstant;

        followCamera.GetYawAngleAttr().Get(&mYawAngle, timeCode);
        followCamera.GetPitchAngleAttr().Get(&mPitchAngle, timeCode);
        followCamera.GetPitchAngleTimeConstantAttr().Get(&mPitchAngleTimeConstant, timeCode);
        followCamera.GetSlowSpeedPitchAngleScaleAttr().Get(&mSlowSpeedPitchAngleScale, timeCode);
        followCamera.GetSlowPitchAngleSpeedAttr().Get(&mSlowPitchAngleSpeed, timeCode);
        followCamera.GetVelocityNormalMinSpeedAttr().Get(&mVelocityNormalMinSpeed, timeCode);
        followCamera.GetFollowMinSpeedAttr().Get(&mFollowMinSpeed, timeCode);
        followCamera.GetFollowMinDistanceAttr().Get(&mFollowMinDistance, timeCode);
        followCamera.GetFollowMaxSpeedAttr().Get(&mFollowMaxSpeed, timeCode);
        followCamera.GetFollowMaxDistanceAttr().Get(&mFollowMaxDistance, timeCode);
        followCamera.GetYawRateTimeConstantAttr().Get(&mYawRateTimeConstant, timeCode);
        followCamera.GetFollowTurnRateGainAttr().Get(&mFollowTurnRateGain, timeCode);
        followCamera.GetCameraPositionTimeConstantAttr().Get(&cameraPositionTimeConstant, timeCode);
        followCamera.GetPositionOffsetAttr().Get(&subjectPositionOffset, timeCode);
        followCamera.GetLookAheadMinSpeedAttr().Get(&mLookAheadMinSpeed, timeCode);
        followCamera.GetLookAheadMinDistanceAttr().Get(&mLookAheadMinDistance, timeCode);
        followCamera.GetLookAheadMaxSpeedAttr().Get(&mLookAheadMaxSpeed, timeCode);
        followCamera.GetLookAheadMaxDistanceAttr().Get(&mLookAheadMaxDistance, timeCode);
        followCamera.GetLookAheadTurnRateGainAttr().Get(&mLookAheadTurnRateGain, timeCode);
        followCamera.GetLookPositionHeightAttr().Get(&mLookPositionHeight, timeCode);
        followCamera.GetLookPositionTimeConstantAttr().Get(&lookPositionTimeConstant, timeCode);

        mCameraPositionTimeConstant =
            ::physx::PxVec3(cameraPositionTimeConstant[0], cameraPositionTimeConstant[1], cameraPositionTimeConstant[2]);
        mPositionOffset =
            ::physx::PxVec3(subjectPositionOffset[0], subjectPositionOffset[1], subjectPositionOffset[2]);
        mLookPositionTimeConstant =
            ::physx::PxVec3(lookPositionTimeConstant[0], lookPositionTimeConstant[1], lookPositionTimeConstant[2]);
    }
}

::physx::PxTransform CameraFollow::updateTransform(float timeStep)
{
    updateSubjectStates(timeStep);

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

void CameraFollow::updateSubjectStates(float timeStep)
{
}

void CameraFollow::updateActorPosition()
{
    ::physx::PxVec3 positionOffset = mSubjectTransform.rotate(mPositionOffset);

    mSubjectReferencePosition = mSubjectTransform.p + positionOffset;
}

void CameraFollow::updateFollowVectorAngles()
{
}

void CameraFollow::updateFollowVector(float timeStep)
{
}

void CameraFollow::updateLookPosition()
{
}

void CameraFollow::setupFilters()
{
    mLookVectorFilter.setTimeConstant(mVelocityBlendTimeConstant);

    mYawRateFilter.setTimeConstant(mYawRateTimeConstant);
    mSubjectPitchFilter.setTimeConstant(mPitchAngleTimeConstant);

    mLookPositionFilter.setTimeConstant(mLookPositionTimeConstant);
    mCameraPositionFilter.setTimeConstant(mCameraPositionTimeConstant);
}

void CameraFollow::updateFilters(float timeStep)
{
    if (mResetFilters)
    {
        mLookVectorFilter.reset(0.0f);
        mYawRateFilter.reset(0.0f);
        mSubjectPitchFilter.reset(0.0f);
        mLookPositionFilter.reset(mLookPosition);
        mCameraPositionFilter.reset(mCameraPosition);

        mResetFilters = false;
    }
    else
    {
        mLookVectorFilter.filter(mLookVectorParameter, timeStep);
        mYawRateFilter.filter(mSubjectYawRate, timeStep);
        mSubjectPitchFilter.filter(mSubjectPitchAngle, timeStep);

        mLookPosition = mLookPositionFilter.filter(mLookPosition, timeStep);
        mCameraPosition = mCameraPositionFilter.filter(mCameraPosition, timeStep);
    }
}

::physx::PxTransform CameraFollow::computeTransform()
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

    return transform;
}

} // namespace physx
} // namespace omni
