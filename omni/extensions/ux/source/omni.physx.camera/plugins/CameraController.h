// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"
#include "CameraTools.h"

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/input/IInput.h>
#include <omni/physx/IPhysx.h>


extern omni::physx::IPhysx* gPhysXInterface;


namespace omni
{
namespace physx
{

class CameraController
{
protected:
    CameraController(const pxr::UsdPrim& prim, const pxr::UsdPrim& subjectPrim);
    virtual ~CameraController();

public:
    struct DirtyFlag
    {
        enum Enum
        {
            eUSD = (1 << 0), // USD changes need to be processed
            eKEYBOARDINPUT = (1 << 1), // user keyboard input needs to be processed
            eGAMEPADINPUT = (1 << 2) // user gamepad/joystick input needs to be processed
        };
    };

    static void release(CameraController&);

    // Setting path to get vehicle inputs
    static const char* kGamepadCameraControl;

    void bindInputs();
    void unbindInputs();

    virtual void initialize()
    {
    }

    virtual void updateAxes();

    bool isRigidBody()
    {
        return (mBodyId != usdparser::kInvalidObjectId);
    }

    virtual void preUpdate();
    virtual void stepUpdate(float timeStep);
    virtual void postUpdate();

    virtual ::physx::PxTransform updateTransform(float timeStep) = 0;

    inline const pxr::UsdPrim& getPrim()
    {
        return mCameraPrim;
    }

    inline const pxr::SdfPath& getSubjectPath()
    {
        return mSubjectPath;
    }

    void setDirty(DirtyFlag::Enum flag)
    {
        mDirtyFlags |= flag;
    }

    void setAlwaysUpdate(bool alwaysUpdate)
    {
        mAlwaysUpdate = alwaysUpdate;
    }

    bool getAlwaysUpdate()
    {
        return mAlwaysUpdate;
    }

protected:
    void subscribOnKeyboardAction(const char* action, carb::input::KeyboardInput keyboardInput);
    void bindKeyboardActions();
    void unbindKeyboardActions();

    void onGamepadConnectionEvent(const carb::input::GamepadConnectionEvent& padEvent);
    void subscribOnGamepadAction(carb::input::Gamepad* gamepad,
                                 const char* action,
                                 const carb::input::GamepadInput gamepadInput);
    void bindGamepadActions(carb::input::Gamepad* gamepad);
    void unbindGamepadActions(carb::input::Gamepad* gamepad);

    virtual void readUsdControls();
    virtual void writeUsdControls();
    virtual void readUsdSettings(const pxr::UsdTimeCode& timeCode);

protected:
    pxr::SdfPath mCameraPath;
    pxr::UsdPrim mCameraPrim;
    pxr::SdfPath mSubjectPath;
    pxr::UsdPrim mSubjectPrim;

    usdparser::ObjectId mBodyId;

    uint32_t mDirtyFlags;

    bool mCameraEnabled;
    bool mAlwaysUpdate;

    ::physx::PxVec3 mLeft;
    ::physx::PxVec3 mUp;
    ::physx::PxVec3 mForward;

    pxr::GfMatrix4d mInitialCameraMatrix;
    pxr::GfMatrix4d mCameraMatrix;

    // Inputs for manual camera control
    carb::input::ActionMappingSet* mCameraActionMappingSet;

    carb::input::SubscriptionId mKeyboardEventId;
    carb::input::Keyboard* mKeyboard;
    std::unordered_map<const char*, carb::input::SubscriptionId> mKeyboardSubscriptionMap;
    std::unordered_map<std::string, float> mKeyboardActionMap;

    carb::input::SubscriptionId mGamepadConnectionEventId;
    std::unordered_map<std::string, float> mGamepadActionMap;

    using ActionSubscriptionMap = std::unordered_map<const char*, carb::input::SubscriptionId>;
    std::unordered_map<carb::input::Gamepad*, ActionSubscriptionMap> mGamepadSubscriptionMappings;
};

} // namespace physx
} // namespace omni
