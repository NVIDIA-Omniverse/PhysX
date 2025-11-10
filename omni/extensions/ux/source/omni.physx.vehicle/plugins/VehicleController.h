// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"
#include "VehicleTools.h"

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/input/IInput.h>
#include <omni/physx/IPhysx.h>

#include <PxPhysicsAPI.h>


namespace omni
{
namespace physx
{


class VehicleManager;


struct DriveType
{
    enum Enum
    {
        eNONE = 0, // user wants to control wheels directly
        eBASIC, // simple steering support and max brake/drive torque
        eSTANDARD // full engine, gears etc. setup
    };
};


struct VehicleControllerSettings
{
    VehicleControllerSettings()
    {
        mInputEnabled = false;
        mMouseEnabled = false;
        mAutoReverseEnabled = true;

        mSteeringSensitivity = 2.0f;
        mSteeringFilterTime = 0.1f;
    }

    bool mInputEnabled;
    bool mMouseEnabled;
    bool mAutoReverseEnabled;

    float mSteeringSensitivity;
    float mSteeringFilterTime;
};


class VehicleController
{
private:
    struct Cache
    {
        Cache()
        {
        }

        usdparser::ObjectId vehicleControllerId;
        usdparser::ObjectId bodyId;
        DriveType::Enum driveType;
        bool enabled; // whether simulation is enabled for the vehicle or not
        bool hasAutobox; // if the vehicle has an autobox (for DriveType::eSTANDARD)
    };

    struct ControllerParams
    {
        ControllerParams() : accelerator(0.0f), brake0(0.0f), brake1(0.0f), steer(0.0f), targetGear(0)
        {
        }

        float accelerator;
        float brake0;
        float brake1;
        float steer;
        int targetGear;
    };

public:
    struct DirtyFlag
    {
        enum Enum
        {
            eKEYBOARDINPUT = (1 << 0), // user keyboard input needs to be processed
            eMOUSEINPUT = (1 << 1), // user mouse input needs to be processed
            eGAMEPADINPUT = (1 << 2), // user gamepad/joystick input needs to be processed
            eDEVICE_INPUT_MASK = eKEYBOARDINPUT | eMOUSEINPUT | eGAMEPADINPUT,

            eACCELERATOR = (1 << 3),
            eBRAKE0 = (1 << 4),
            eBRAKE1 = (1 << 5),
            eSTEER = (1 << 6),
            eTARGET_GEAR = (1 << 7),
            eCONTROLLER_PARAM_MASK = eACCELERATOR | eBRAKE0 | eBRAKE1 | eSTEER | eTARGET_GEAR
        };
    };

    // Setting path to get vehicle inputs
    static const char* kGamepadVehicleControl;


private:
    VehicleController(const pxr::UsdPrim&, const VehicleControllerSettings& vehicleControllerSettings);
    ~VehicleController();

public:
    static VehicleController* create(const pxr::UsdPrim&,
                                     const VehicleControllerSettings&,
                                     VehicleManager&,
                                     usdparser::ObjectId);
    static void release(VehicleController&);

    void bindInputs();
    void unbindInputs();

    void update(float timeStep, const pxr::UsdTimeCode&);

    void setDirty(DirtyFlag::Enum flag)
    {
        mDirtyFlags |= flag;
    }

    DriveType::Enum getDriveType();

    bool getEnabled() const;
    void setEnabled(const bool enabled);

    void onSteeringFilterTimeChanged()
    {
        mSteerFilter.setTimeConstant(mVehicleControllerSettings.mSteeringFilterTime);
    }

    void disableFiltering()
    {
        mFilterSteeringDeviceInput = false;
    }

private:
    float getAccelerator() const;
    float getBrake0() const;
    float getBrake1() const;
    float getSteer() const;
    int getTargetGear() const;

    inline void setAcceleratorAndBrake0(const Cache&,
                                        const float accelerator,
                                        const float brake,
                                        const bool checkForAutoReverse);
    inline void setBrake1(const float brake);
    inline void setSteer(const float steer);
    inline void setTargetGear(const int targetGear);

    void onMouseMove(const carb::Float2 mousePos);

    void subscribeOnKeyboardAction(const char* action, carb::input::KeyboardInput keyboardInput);
    void bindKeyboardActions();
    void unbindKeyboardActions();

    void onGamepadConnectionEvent(const carb::input::GamepadConnectionEvent& padEvent);
    void subscribeOnGamepadAction(carb::input::Gamepad* gamepad,
                                  const char* action,
                                  const carb::input::GamepadInput gamepadInput);
    void bindGamepadActions(carb::input::Gamepad* gamepad);
    void unbindGamepadActions(carb::input::Gamepad* gamepad);

    void writeUsdControls(const Cache&);

    void updateReverse(const Cache&, float timeStep);

    bool fillCache(VehicleManager& vehicleManager, usdparser::ObjectId);
    const Cache& getCache() const;
    Cache& getCache();


private:
    pxr::SdfPath mUsdPath;
    pxr::UsdPrim mUsdPrim;

    const VehicleControllerSettings& mVehicleControllerSettings;

    Cache mCache; // do not access directly. Use getCache().

    uint32_t mDirtyFlags;

    ControllerParams mControllerParams;

    // Inputs for manual vehicle control
    bool mInReverse; // only used for DriveType::eBASIC
    int8_t mAutoReverseInProgress; // only used for DriveType::eSTANDARD
                                   // 0: no pending auto-reverse, 1: switch to forward, -1: switch to reverse

    bool mFilterSteeringDeviceInput;
    float mBrakeTimer;

    // The input device filter runs every update using the last received raw steer input from the device.
    float mLastRawSteerInput;

    carb::input::ActionMappingSet* mVehicleActionMappingSet;

    carb::input::IInput* mInput;

    carb::Float2 mMouseInput;
    carb::Float2 mPreviousCoords;
    carb::input::Mouse* mMouse;

    carb::input::SubscriptionId mKeyboardEventId;
    carb::input::Keyboard* mKeyboard;
    std::unordered_map<const char*, carb::input::SubscriptionId> mKeyboardSubscriptionMap;
    std::unordered_map<std::string, float> mKeyboardActionMap;

    carb::input::SubscriptionId mGamepadConnectionEventId;
    std::unordered_map<std::string, float> mGamepadActionMap;

    using ActionSubscriptionMap = std::unordered_map<const char*, carb::input::SubscriptionId>;
    std::unordered_map<carb::input::Gamepad*, ActionSubscriptionMap> mGamepadSubscriptionMappings;

    LowPassFilter mSteerFilter;
};

} // namespace physx
} // namespace omni
