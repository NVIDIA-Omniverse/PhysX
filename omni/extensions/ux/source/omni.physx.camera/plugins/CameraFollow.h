// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "CameraController.h"

#include <carb/Types.h>

#include <PxPhysicsAPI.h>


namespace omni
{
namespace physx
{

class CameraFollow : public CameraController
{
protected:
    CameraFollow(const pxr::UsdPrim& prim, const pxr::UsdPrim& subjectPath);
    virtual ~CameraFollow();

public:
    virtual void initialize() override
    {
        resetFilters();
    }

    virtual void updateAxes() override;

protected:
    // Subject states.
    ::physx::PxVec3 mSubjectLook;
    ::physx::PxVec3 mSubjectUp;

    float mSubjectPitchAngle;

    ::physx::PxVec3 mSubjectVelocity;
    ::physx::PxVec3 mSubjectAngularVelocity;
    float mSubjectSpeed;

    ::physx::PxVec3 mSubjectBodyAngularVelocity;
    float mSubjectYawRate;

    ::physx::PxTransform mSubjectTransform;

    bool mResetRates;

    // Camera
    ::physx::PxVec3 mSubjectReferencePosition;
    ::physx::PxVec3 mFollowVector;

    ::physx::PxVec3 mCameraPosition;
    ::physx::PxVec3 mLookPosition;

    float mLookVectorParameter;

    float mFollowDistance;
    float mFollowDistanceOffset;

    float mFollowVectorPitch;
    float mFollowVectorYaw;

    // Filters
    bool mResetFilters;

    LowPassFilter mLookVectorFilter;
    LowPassFilter mYawRateFilter;
    LowPassFilter mSubjectPitchFilter;
    VectorLowPassFilter mLookPositionFilter;
    VectorLowPassFilter mCameraPositionFilter;

    // Camera settings.
    float mYawAngle;
    float mPitchAngle;
    float mPitchAngleTimeConstant;
    float mSlowSpeedPitchAngleScale;
    float mSlowPitchAngleSpeed;
    float mVelocityNormalMinSpeed;
    float mVelocityBlendTimeConstant;
    float mFollowMinSpeed;
    float mFollowMinDistance;
    float mFollowMaxSpeed;
    float mFollowMaxDistance;
    float mYawRateTimeConstant;
    float mFollowTurnRateGain;
    ::physx::PxVec3 mCameraPositionTimeConstant;
    ::physx::PxVec3 mPositionOffset;
    float mLookAheadMinSpeed;
    float mLookAheadMinDistance;
    float mLookAheadMaxSpeed;
    float mLookAheadMaxDistance;
    float mLookAheadTurnRateGain;
    float mLookPositionHeight;
    ::physx::PxVec3 mLookPositionTimeConstant;


    void resetFilters()
    {
        mResetFilters = true;
    }

    void setupFilters();
    void updateFilters(float timeStep);

    virtual void readUsdSettings(const pxr::UsdTimeCode&);

    virtual ::physx::PxTransform updateTransform(float timeStep) override;

    virtual void updateSubjectStates(float timeStep);
    virtual void updateActorPosition();
    virtual void updateFollowVectorAngles();
    virtual void updateFollowVector(float timeStep);
    virtual void updateLookPosition();

    ::physx::PxTransform computeTransform();
};

} // namespace physx
} // namespace omni
