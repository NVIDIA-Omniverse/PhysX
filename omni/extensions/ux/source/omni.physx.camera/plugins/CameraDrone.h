// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

class CameraDrone : public CameraController
{
private:
    CameraDrone(const pxr::UsdPrim& prim, const pxr::UsdPrim& subjectPrim);
    virtual ~CameraDrone();

public:
    static CameraDrone* create(const pxr::UsdPrim& prim, const pxr::UsdPrim& subjectPrim);

    virtual void initialize() override
    {
        resetFilters();
    }

protected:
    // Subject states.
    ::physx::PxVec3 mSubjectVelocity;

    ::physx::PxTransform mSubjectTransform;

    // Camera
    ::physx::PxVec3 mCameraPosition;
    ::physx::PxVec3 mCameraVelocity;
    ::physx::PxVec3 mLookPosition;

    ::physx::PxQuat mCameraRotation;

    bool mResetRates;

    // Filters
    bool mResetFilters;

    VectorLowPassFilter mVelocityFilter;
    LowPassFilter mAngleFilter;

    // Camera settings.
    float mFollowHeight;
    float mFollowDistance;
    float mMaxDistance;
    float mMaxSpeed;
    float mHorizontalVelocityGain;
    float mVerticalVelocityGain;
    float mFeedForwardVelocityGain;
    float mVelocityFilterTimeConstant;
    float mRotationFilterTimeConstant;

    ::physx::PxVec3 mSubjectPositionOffset;


    void resetFilters()
    {
        mResetFilters = true;
    }

    virtual void readUsdSettings(const pxr::UsdTimeCode&);

    virtual ::physx::PxTransform updateTransform(float timeStep) override;

    ::physx::PxVec3 computeTargetPosition();
    void updateCameraPosition(float timeStep);
    ::physx::PxTransform computeTransform(float timeStep);
};

} // namespace physx
} // namespace omni
