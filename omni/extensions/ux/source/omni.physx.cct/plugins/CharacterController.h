// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include <carb/Defines.h>
#include <carb/Types.h>

#include <omni/kit/ExtensionWindowTypes.h>

#include <PxPhysicsAPI.h>

namespace omni
{
namespace physx
{
class CharacterController
{
    enum class GravityMode
    {
        DISABLED = 0,
        APPLY_BASE,
        APPLY_CUSTOM,
    };

public:
    CharacterController(const pxr::SdfPath& path);
    ~CharacterController();

    void update(float timeStep, const unsigned char stageUpAxisIndex);
    void setPosition(const ::physx::PxExtendedVec3& position);

    void enableGravity();
    void enableCustomGravity(::physx::PxVec3 gravity);
    void disableGravity();
    bool hasGravityEnabled() const;

    float getHeight();
    void setHeight(float height);

    void enableFirstPerson(const pxr::SdfPath& cameraPath)
    {
        mCameraPath = cameraPath;
    }

    void disableFirstPerson()
    {
        mCameraPath = pxr::SdfPath();
    }

    bool isFirstPerson()
    {
        return !mCameraPath.IsEmpty();
    }

    pxr::SdfPath& getCameraPath()
    {
        return mCameraPath;
    }

    void setDirty()
    {
        mDirty = true;
    }

    const pxr::UsdPrim& getUsdPrim() const
    {
        return mUsdPrim;
    }

    void cacheMoveTarget();
    void resetMoveTarget();
    void setMove(const pxr::GfVec3f& displ);

    void switchPurposeToGuide();
    void resetPurpose();
    void enableWorldSpaceMove(bool enable);

    void onResume();
    void onStop();
    void onTimelinePlay();

private:
    void readUsdControls();
    void sendCollisionFlagEvents(const ::physx::PxControllerCollisionFlags flags);
    void getLocalMoveFrame(::physx::PxVec3& fwd, ::physx::PxVec3& right, const unsigned char stageUpAxisIndex);

private:
    pxr::SdfPath mUsdPath;
    pxr::UsdPrim mUsdPrim;
    pxr::SdfPath mCameraPath;
    GravityMode mGravityMode;
    ::physx::PxVec3 mCustomGravity;
    bool mDirty;
    bool mWorldSpaceMove;
    pxr::GfVec3f mMove;
    pxr::GfVec3f mInitMove;
    pxr::TfToken mInitPurpose;
    float mTimeSinceFalling;
    ::physx::PxControllerCollisionFlags mCurrentCollisionFlags;
};
} // namespace physx
} // namespace omni
;
