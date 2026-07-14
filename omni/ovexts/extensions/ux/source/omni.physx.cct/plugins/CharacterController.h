// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
    CharacterController(const PXR_NS::SdfPath& path);
    ~CharacterController();

    void update(float timeStep, const unsigned char stageUpAxisIndex);
    void setPosition(const ::physx::PxExtendedVec3& position);

    void enableGravity();
    void enableCustomGravity(::physx::PxVec3 gravity);
    void disableGravity();
    bool hasGravityEnabled() const;

    float getHeight();
    void setHeight(float height);

    void enableFirstPerson(const PXR_NS::SdfPath& cameraPath)
    {
        mCameraPath = cameraPath;
    }

    void disableFirstPerson()
    {
        mCameraPath = PXR_NS::SdfPath();
    }

    bool isFirstPerson()
    {
        return !mCameraPath.IsEmpty();
    }

    PXR_NS::SdfPath& getCameraPath()
    {
        return mCameraPath;
    }

    void setDirty()
    {
        mDirty = true;
    }

    const PXR_NS::UsdPrim& getUsdPrim() const
    {
        return mUsdPrim;
    }

    void cacheMoveTarget();
    void resetMoveTarget();
    void setMove(const PXR_NS::GfVec3f& displ);

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
    PXR_NS::SdfPath mUsdPath;
    PXR_NS::UsdPrim mUsdPrim;
    PXR_NS::SdfPath mCameraPath;
    GravityMode mGravityMode;
    ::physx::PxVec3 mCustomGravity;
    bool mDirty;
    bool mWorldSpaceMove;
    PXR_NS::GfVec3f mMove;
    PXR_NS::GfVec3f mInitMove;
    PXR_NS::TfToken mInitPurpose;
    float mTimeSinceFalling;
    ::physx::PxControllerCollisionFlags mCurrentCollisionFlags;
};
} // namespace physx
} // namespace omni
;
