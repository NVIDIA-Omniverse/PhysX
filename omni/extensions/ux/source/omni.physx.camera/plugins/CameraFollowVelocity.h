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

class CameraFollowVelocity : public CameraFollow
{
private:
    CameraFollowVelocity(const pxr::UsdPrim& prim, const pxr::UsdPrim& subjectPath);
    virtual ~CameraFollowVelocity();

public:
    static CameraFollowVelocity* create(const pxr::UsdPrim& prim, const pxr::UsdPrim& subjectPrim);

protected:
    virtual ::physx::PxTransform updateTransform(float timeStep) override;

    virtual void updateActorPosition();
    virtual void updateFollowVectorAngles();
    virtual void updateFollowVector(float timeStep);
    virtual void updateLookPosition();
};

} // namespace physx
} // namespace omni
