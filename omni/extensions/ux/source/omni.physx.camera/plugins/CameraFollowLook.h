// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "CameraFollow.h"

#include <carb/Types.h>

#include <PxPhysicsAPI.h>


namespace omni
{
namespace physx
{

class CameraFollowLook : public CameraFollow
{
private:
    CameraFollowLook(const pxr::UsdPrim& prim, const pxr::UsdPrim& subjectPath);
    virtual ~CameraFollowLook();

public:
    static CameraFollowLook* create(const pxr::UsdPrim& prim, const pxr::UsdPrim& subjectPrim);

    virtual void updateAxes();

protected:
    // Unique Look Camera settings.
    float mDownHillGroundAngle;
    float mDownHillGroundPitch;
    float mUpHillGroundAngle;
    float mUpHillGroundPitch;
    float mFollowReverseSpeed;
    float mFollowReverseDistance;

    // Local variables.
    ::physx::PxTransform mInitialTransform;


    virtual void readUsdSettings(const pxr::UsdTimeCode&);

    virtual ::physx::PxTransform updateTransform(float timeStep) override;

    virtual void updateActorPosition();
    virtual void updateFollowVectorAngles();
    virtual void updateFollowVector(float timeStep);
    virtual void updateLookPosition();
};

} // namespace physx
} // namespace omni
