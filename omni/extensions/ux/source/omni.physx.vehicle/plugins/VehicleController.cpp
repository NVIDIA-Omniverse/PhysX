// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "VehicleController.h"
#include "VehicleManager.h"

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>

#include <omni/kit/IAppWindow.h>
#include <omni/kit/SettingsUtils.h>

#include <common/foundation/Allocator.h>


// Keyboard controls
// Is there a way to stringify the keys without having to explicitely duplicate them?
#define STEER_LEFT_KEY eLeft
#define STEER_RIGHT_KEY eRight
#define ACCELERATOR_KEY eUp
#define BRAKE0_KEY eDown
#define BRAKE1_KEY ePageDown

#define STEER_LEFT_STRING "eLeft"
#define STEER_RIGHT_STRING "eRight"
#define ACCELERATOR_STRING "eUp"
#define BRAKE0_STRING "eDown"
#define BRAKE1_STRING "ePageDown"


extern omni::physx::IPhysx* gPhysXInterface;
extern pxr::UsdStageRefPtr gStage;


using namespace ::physx;
using namespace pxr;
using namespace carb;
using namespace carb::input;


namespace omni
{
namespace physx
{

const char* VehicleController::kGamepadVehicleControl = PERSISTENT_SETTINGS_PREFIX "/app/omniverse/gamepadVehicleControl";

VehicleController::VehicleController(const pxr::UsdPrim& prim, const VehicleControllerSettings& vehicleControllerSettings)
    : mVehicleControllerSettings(vehicleControllerSettings)
{
    mUsdPath = prim.GetPath();

    mDirtyFlags = 0;

    mUsdPrim = prim;
    CARB_ASSERT(prim.HasAPI<PhysxSchemaPhysxVehicleControllerAPI>());  // else this should not get called

    // Inputs
    mFilterSteeringDeviceInput  = false;

    mInReverse = false;
    mAutoReverseInProgress = 0;

    mBrakeTimer = 0.0f;

    mInput = nullptr;

    mMouseInput.x = 0.0;
    mMouseInput.y = 0.0;

    mPreviousCoords.x = 0.0;
    mPreviousCoords.y = 0.0;

    mMouse = nullptr;

    mKeyboard = nullptr;

    mLastRawSteerInput = 0.0f;
    mSteerFilter.setTimeConstant(mVehicleControllerSettings.mSteeringFilterTime);

    mGamepadConnectionEventId = 0;
    mVehicleActionMappingSet = nullptr;
    mGamepadActionMap.clear();
    mGamepadSubscriptionMappings.clear();

    if (vehicleControllerSettings.mInputEnabled)
        bindInputs();
}

VehicleController::~VehicleController()
{
    unbindInputs();
}

VehicleController* VehicleController::create(const pxr::UsdPrim& prim, const VehicleControllerSettings& vehicleControllerSettings,
    VehicleManager& vehicleManager, usdparser::ObjectId objectId)
{
    VehicleController* vehicleController = ICE_PLACEMENT_NEW(VehicleController)(prim, vehicleControllerSettings);
    if (vehicleController->fillCache(vehicleManager, objectId))
        return vehicleController;
    else
    {
        ICE_PLACEMENT_DELETE(vehicleController, VehicleController);
        return nullptr;
    }
}

void VehicleController::release(VehicleController& vehicleController)
{
    ICE_PLACEMENT_DELETE_UNCHECKED(&vehicleController, VehicleController);
}

void VehicleController::bindInputs()
{
    mInput = carb::getCachedInterface<carb::input::IInput>();

    if (mInput)
    {
        // Create a vehicle controller action mapping set if one does not exist.
        mVehicleActionMappingSet = mInput->getActionMappingSetByPath(kGamepadVehicleControl);

        if (mVehicleActionMappingSet == nullptr)
        {
            mVehicleActionMappingSet = mInput->createActionMappingSet(kGamepadVehicleControl);
        }

        // Subscribe to keyboard input.
        omni::kit::IAppWindow* appWindow = omni::kit::getDefaultAppWindow();
        if (appWindow)
        {
            mKeyboard = appWindow->getKeyboard();
            if (mKeyboard)
            {
                bindKeyboardActions();
            }

            mMouse = appWindow->getMouse();
        }

        // Subscribe to gamepad input.
        mGamepadConnectionEventId = mInput->subscribeToGamepadConnectionEvents(
            [](const carb::input::GamepadConnectionEvent& gamegamepadEvent, void* userData) {
                VehicleController* vehicleController = reinterpret_cast<VehicleController*>(userData);
                vehicleController->onGamepadConnectionEvent(gamegamepadEvent);
            },
            this);
    }
}

void VehicleController::unbindInputs()
{
    if (mInput)
    {
        // Keyboard
        for (auto& mapping : mKeyboardSubscriptionMap)
        {
            mInput->unsubscribeToActionEvents(mapping.second);
        }

        mKeyboardSubscriptionMap.clear();
        unbindKeyboardActions();

        // Gamepads
        mInput->unsubscribeToGamepadConnectionEvents(mGamepadConnectionEventId);

        // when enabling, start from clean input
        mDirtyFlags &= ~(DirtyFlag::eDEVICE_INPUT_MASK | DirtyFlag::eCONTROLLER_PARAM_MASK); 
        mGamepadConnectionEventId = 0;
        mVehicleActionMappingSet = nullptr;
        mMouse = nullptr;
        mInput = nullptr;
    }
}

void VehicleController::onMouseMove(const carb::Float2 mousePos)
{
    // note: the input coordinates are in [0, 1]. Want to be able to go from
    //       max left to max right steer by moving the mouse only through a
    //       fraction of the window size, thus multiplying the deltas with a scale.
    static const float sMouseScale = 16.0f;

    if (mPreviousCoords.x == 0.0 && mPreviousCoords.y == 0.0)
    {
        mPreviousCoords = mousePos;
    }

    float deltaX = mousePos.x - mPreviousCoords.x;

    if (abs(deltaX) > 0.0f)
    {
        mMouseInput.x += sMouseScale * deltaX;
        mMouseInput.x = CARB_CLAMP(mMouseInput.x, -1.0f, 1.0f);

        setDirty(DirtyFlag::eMOUSEINPUT);
    }

    mPreviousCoords = mousePos;
}

void VehicleController::bindKeyboardActions()
{
    // Subscribe to keyboard control inputs.
    subscribeOnKeyboardAction(STEER_LEFT_STRING, carb::input::KeyboardInput::STEER_LEFT_KEY);
    subscribeOnKeyboardAction(STEER_RIGHT_STRING, carb::input::KeyboardInput::STEER_RIGHT_KEY);
    subscribeOnKeyboardAction(ACCELERATOR_STRING, carb::input::KeyboardInput::ACCELERATOR_KEY);
    subscribeOnKeyboardAction(BRAKE0_STRING, carb::input::KeyboardInput::BRAKE0_KEY);
    subscribeOnKeyboardAction(BRAKE1_STRING, carb::input::KeyboardInput::BRAKE1_KEY);
}

void VehicleController::unbindKeyboardActions()
{
    CARB_ASSERT(mInput);

    if (mVehicleActionMappingSet != nullptr)
    {
        mInput->clearActionMappings(mVehicleActionMappingSet, STEER_LEFT_STRING);
        mInput->clearActionMappings(mVehicleActionMappingSet, STEER_RIGHT_STRING);
        mInput->clearActionMappings(mVehicleActionMappingSet, ACCELERATOR_STRING);
        mInput->clearActionMappings(mVehicleActionMappingSet, BRAKE0_STRING);
        mInput->clearActionMappings(mVehicleActionMappingSet, BRAKE1_STRING);
    }
}

void VehicleController::subscribeOnKeyboardAction(const char* key, carb::input::KeyboardInput keyboardInput)
{
    CARB_ASSERT(mInput);

    if (mVehicleActionMappingSet)
    {
        if (mKeyboardSubscriptionMap.find(key) == mKeyboardSubscriptionMap.end())
        {
            mKeyboardSubscriptionMap[key] = mInput->subscribeToActionEvents(
                mVehicleActionMappingSet, key,
                [](const carb::input::ActionEvent& actionEvent, void* userData) {
                    VehicleController* vehicleController = reinterpret_cast<VehicleController*>(userData);
                    vehicleController->setDirty(DirtyFlag::eKEYBOARDINPUT);
                    vehicleController->mKeyboardActionMap[actionEvent.action] = actionEvent.value;
                    return false;
                },
                this);

            carb::input::ActionMappingDesc actionKeyDesc;
            actionKeyDesc.deviceType = carb::input::DeviceType::eKeyboard;
            actionKeyDesc.keyboard = mKeyboard;
            actionKeyDesc.keyboardInput = keyboardInput;
            actionKeyDesc.modifiers = 0;
            mInput->addActionMapping(mVehicleActionMappingSet, key, actionKeyDesc);
        }
    }
}

void VehicleController::onGamepadConnectionEvent(const carb::input::GamepadConnectionEvent& gamepadEvent)
{
    CARB_ASSERT(mInput);

    // Setup gamepad control

    if (gamepadEvent.type == carb::input::GamepadConnectionEventType::eConnected)
    {
        mGamepadSubscriptionMappings[gamepadEvent.gamepad].clear();
        bindGamepadActions(gamepadEvent.gamepad);
    }
    else if (gamepadEvent.type == carb::input::GamepadConnectionEventType::eDisconnected)
    {
        for (auto& mapping : mGamepadSubscriptionMappings[gamepadEvent.gamepad])
        {
            mInput->unsubscribeToActionEvents(mapping.second);
        }

        mGamepadSubscriptionMappings[gamepadEvent.gamepad].clear();
        unbindGamepadActions(gamepadEvent.gamepad);
    }
}

void VehicleController::subscribeOnGamepadAction(carb::input::Gamepad* gamepad,
                                                const char* action,
                                                const carb::input::GamepadInput gamepadInput)
{
    CARB_ASSERT(mInput);

    if (mVehicleActionMappingSet)
    {
        if (mGamepadSubscriptionMappings[gamepad].find(action) == mGamepadSubscriptionMappings[gamepad].end())
        {
            mGamepadSubscriptionMappings[gamepad][action] = mInput->subscribeToActionEvents(
                mVehicleActionMappingSet, action,
                [](const carb::input::ActionEvent& actionEvent, void* userData) {
                    VehicleController* vehicleController = reinterpret_cast<VehicleController*>(userData);
                    vehicleController->setDirty(DirtyFlag::eGAMEPADINPUT);
                    vehicleController->mGamepadActionMap[actionEvent.action] = actionEvent.value;
                    return false;
                },
                this);

            carb::input::ActionMappingDesc actionPadDesc;
            actionPadDesc.deviceType = carb::input::DeviceType::eGamepad;
            actionPadDesc.gamepad = gamepad;
            actionPadDesc.gamepadInput = gamepadInput;
            actionPadDesc.modifiers = 0;

            mInput->addActionMapping(mVehicleActionMappingSet, action, actionPadDesc);
        }
    }
}

void VehicleController::bindGamepadActions(carb::input::Gamepad* gamepad)
{
    subscribeOnGamepadAction(gamepad, "eLeftStickUp", carb::input::GamepadInput::eLeftStickUp);
    subscribeOnGamepadAction(gamepad, "eLeftStickDown", carb::input::GamepadInput::eLeftStickDown);
    subscribeOnGamepadAction(gamepad, "eLeftStickLeft", carb::input::GamepadInput::eLeftStickLeft);
    subscribeOnGamepadAction(gamepad, "eLeftStickRight", carb::input::GamepadInput::eLeftStickRight);
    subscribeOnGamepadAction(gamepad, "eRightStickUp", carb::input::GamepadInput::eRightStickUp);
    subscribeOnGamepadAction(gamepad, "eRightStickDown", carb::input::GamepadInput::eRightStickDown);
    subscribeOnGamepadAction(gamepad, "eRightStickLeft", carb::input::GamepadInput::eRightStickLeft);
    subscribeOnGamepadAction(gamepad, "eRightStickRight", carb::input::GamepadInput::eRightStickRight);
    subscribeOnGamepadAction(gamepad, "eRightTrigger", carb::input::GamepadInput::eRightTrigger);
    subscribeOnGamepadAction(gamepad, "eLeftTrigger", carb::input::GamepadInput::eLeftTrigger);
}

void VehicleController::unbindGamepadActions(carb::input::Gamepad* gamepad)
{
    CARB_ASSERT(mInput);

    if (mVehicleActionMappingSet != nullptr)
    {
        mInput->clearActionMappings(mVehicleActionMappingSet, "eLeftStickUp");
        mInput->clearActionMappings(mVehicleActionMappingSet, "eLeftStickDown");
        mInput->clearActionMappings(mVehicleActionMappingSet, "eLeftStickLeft");
        mInput->clearActionMappings(mVehicleActionMappingSet, "eLeftStickRight");
        mInput->clearActionMappings(mVehicleActionMappingSet, "eRightStickUp");
        mInput->clearActionMappings(mVehicleActionMappingSet, "eRightStickDown");
        mInput->clearActionMappings(mVehicleActionMappingSet, "eRightStickLeft");
        mInput->clearActionMappings(mVehicleActionMappingSet, "eRightStickRight");
        mInput->clearActionMappings(mVehicleActionMappingSet, "eRightTrigger");
        mInput->clearActionMappings(mVehicleActionMappingSet, "eLeftTrigger");
    }
}

void VehicleController::writeUsdControls(const Cache& cache)
{
    CARB_ASSERT(cache.vehicleControllerId != usdparser::kInvalidObjectId); // else this method should not get called

    if (mDirtyFlags & DirtyFlag::eCONTROLLER_PARAM_MASK)
    {
        CARB_ASSERT((cache.driveType == DriveType::eBASIC) || (cache.driveType == DriveType::eSTANDARD));
        CARB_ASSERT(mUsdPrim.HasAPI<PhysxSchemaPhysxVehicleControllerAPI>());

        PhysxSchemaPhysxVehicleControllerAPI vehicleControllerAPI = PhysxSchemaPhysxVehicleControllerAPI(mUsdPrim);

        if (mDirtyFlags & DirtyFlag::eACCELERATOR)
        {
            vehicleControllerAPI.GetAcceleratorAttr().Set(getAccelerator());
        }

        if (mDirtyFlags & DirtyFlag::eBRAKE0)
        {
            vehicleControllerAPI.GetBrake0Attr().Set(getBrake0());
        }

        if (mDirtyFlags & DirtyFlag::eBRAKE1)
        {
            vehicleControllerAPI.GetBrake1Attr().Set(getBrake1());
        }

        if (mDirtyFlags & DirtyFlag::eSTEER)
        {
            vehicleControllerAPI.GetSteerAttr().Set(getSteer());
        }

        if (mDirtyFlags & DirtyFlag::eTARGET_GEAR)
        {
            vehicleControllerAPI.GetTargetGearAttr().Set(getTargetGear());
        }

        mDirtyFlags &= ~DirtyFlag::eCONTROLLER_PARAM_MASK;
    }
}

static bool checkForAutoBox(const PhysxSchemaPhysxVehicleDriveStandardAPI& driveAPI, const UsdStageWeakPtr& stage)
{
    UsdRelationship autoboxRel = driveAPI.GetAutoGearBoxRel();
    if (autoboxRel.HasAuthoredTargets())
    {
        SdfPathVector paths;
        autoboxRel.GetTargets(&paths);
        if (paths.size() > 0)
        {
            SdfPath autoboxPath = paths[0];
            UsdPrim autoboxPrim = stage->GetPrimAtPath(autoboxPath);
            if (autoboxPrim)
            {
                if (autoboxPrim.HasAPI<PhysxSchemaPhysxVehicleAutoGearBoxAPI>())
                {
                    return true;
                }
                else
                {
                    CARB_LOG_ERROR("PhysX Vehicle: \"%s\" has relationship to auto gear box prim \"%s\" which does "
                        "not have PhysxVehicleAutoGearBoxAPI applied.\n",
                        driveAPI.GetPath().GetText(), autoboxPath.GetText());
                    return false;
                }
            }
            else
            {
                CARB_LOG_ERROR("PhysX Vehicle: \"%s\" has relationship to nonexistent auto gear box prim \"%s\".\n",
                    driveAPI.GetPath().GetText(), autoboxPath.GetText());
                return false;
            }
        }
        else
        {
            return driveAPI.GetPrim().HasAPI<PhysxSchemaPhysxVehicleAutoGearBoxAPI>();
        }
    }
    else
    {
        return driveAPI.GetPrim().HasAPI<PhysxSchemaPhysxVehicleAutoGearBoxAPI>();
    }

    return false;
}

bool VehicleController::fillCache(VehicleManager& vehicleManager, usdparser::ObjectId objectId)
{
    // caching vehicle controller ID
    if (objectId == usdparser::kInvalidObjectId)
    {
        usdparser::ObjectId vehicleControllerId = gPhysXInterface->getObjectId(mUsdPath, ePTVehicleController);
        if (vehicleControllerId != usdparser::kInvalidObjectId)
            mCache.vehicleControllerId = vehicleControllerId;
        else
        {
            CARB_LOG_ERROR("PhysX Vehicle: internal vehicle controller object for vehicle \"%s\" could not be found.\n",
                mUsdPath.GetText());
            return false;
        }
    }
    else
        mCache.vehicleControllerId = objectId;

	// caching body ID
    usdparser::ObjectId bodyId = gPhysXInterface->getObjectId(mUsdPath, ePTActor);
    if (bodyId != usdparser::kInvalidObjectId)
        mCache.bodyId = bodyId;
    else
    {
        CARB_LOG_ERROR("PhysX Vehicle: internal rigid body object for vehicle \"%s\" could not be found.\n",
            mUsdPath.GetText());
        return false;
    }

    // detecting drive type
    UsdStageWeakPtr stage = mUsdPrim.GetStage();
    CARB_ASSERT(mUsdPrim.HasAPI<PhysxSchemaPhysxVehicleAPI>());
    PhysxSchemaPhysxVehicleAPI vehicleAPI(mUsdPrim);
    vehicleAPI.GetVehicleEnabledAttr().Get(&mCache.enabled);
    bool hasNoDriveRelationship = false;
    mCache.hasAutobox = false;
    UsdRelationship driveRel = vehicleAPI.GetDriveRel();
    if (driveRel.HasAuthoredTargets())
    {
        SdfPathVector paths;
        driveRel.GetTargets(&paths);
        if (paths.size() == 1)
        {
            SdfPath drivePath = paths[0];
            UsdPrim drivePrim = stage->GetPrimAtPath(drivePath);
            if (drivePrim)
            {
                if (drivePrim.HasAPI<PhysxSchemaPhysxVehicleDriveStandardAPI>())
                {
                    mCache.driveType = DriveType::eSTANDARD;
                    mCache.hasAutobox = checkForAutoBox(PhysxSchemaPhysxVehicleDriveStandardAPI(drivePrim), stage);
                }
                else if (drivePrim.HasAPI<PhysxSchemaPhysxVehicleDriveBasicAPI>())
                {
                    mCache.driveType = DriveType::eBASIC;
                }
                else
                {
                    CARB_LOG_ERROR("PhysX Vehicle: \"%s\" has relationship to drive prim \"%s\" but it has neither "
                        "PhysxVehicleDriveStandardAPI nor PhysxVehicleDriveBasicAPI applied.\n",
                        mUsdPrim.GetPath().GetText(), drivePath.GetText());
                    return false;
                }
            }
            else
            {
                CARB_LOG_ERROR("PhysX Vehicle: \"%s\" has relationship to nonexistent drive prim \"%s\".\n",
                    mUsdPrim.GetPath().GetText(), drivePath.GetText());
                return false;
            }
        }
        else if (paths.size() == 0)
        {
            hasNoDriveRelationship = true;
        }
        else
        {
            CARB_LOG_ERROR("PhysX Vehicle: \"%s\" has drive relationship with multiple paths defined.\n",
                mUsdPrim.GetPath().GetText());
            return false;
        }
    }
    else
        hasNoDriveRelationship = true;

    if (hasNoDriveRelationship)
    {
        if (mUsdPrim.HasAPI<PhysxSchemaPhysxVehicleDriveStandardAPI>())
        {
            mCache.driveType = DriveType::eSTANDARD;
            mCache.hasAutobox = checkForAutoBox(PhysxSchemaPhysxVehicleDriveStandardAPI(mUsdPrim), stage);
        }
        else if (mUsdPrim.HasAPI<PhysxSchemaPhysxVehicleDriveBasicAPI>())
        {
            mCache.driveType = DriveType::eBASIC;
        }
        else
        {
            mCache.driveType = DriveType::eNONE;
        }
    }

    return true;
}

const VehicleController::Cache& VehicleController::getCache() const
{
    return mCache;
}

VehicleController::Cache& VehicleController::getCache()
{
    return mCache;
}

DriveType::Enum VehicleController::getDriveType()
{
    const Cache& cache = getCache();
    return cache.driveType;
}

float VehicleController::getAccelerator() const
{
    return mControllerParams.accelerator;
}

float VehicleController::getBrake0() const
{
    return mControllerParams.brake0;
}

float VehicleController::getBrake1() const
{
    return mControllerParams.brake1;
}

float VehicleController::getSteer() const
{
    return mControllerParams.steer;
}

int VehicleController::getTargetGear() const
{
    return mControllerParams.targetGear;
}

static constexpr int sReverseGear = -1;
static constexpr int sFirstGear = 1;

void VehicleController::setAcceleratorAndBrake0(const Cache& cache,
    const float accelerator, const float brake, const bool checkForAutoReverse)
{
    if (!mVehicleControllerSettings.mAutoReverseEnabled || !checkForAutoReverse)
    {
        mControllerParams.accelerator = accelerator;
        mControllerParams.brake0 = brake;
    }
    else
    {
        if (cache.driveType == DriveType::eSTANDARD)
        {
            VehicleDriveState driveState;
            if (gPhysXInterface->getVehicleDriveState(cache.vehicleControllerId, driveState))
            {
                // reading current gear from PhysX vehicle is important since it might take
                // a few sim steps for the vehicle to switch gears for autoreverse. The deciding
                // factor for flipping accelerator and brake is thus the current gear (also,
                // the vehicle might start in reverse gear).
                // In addition, acceleration gets set to 0 while auto-reverse is in progress.
                // This avoids the scenario where you can get acceleration for one frame in the
                // wrong direction because this controller will see the gear change with a one
                // frame delay. Example:
                // frame 0:  driving forward, brake key
                // frame 10: reverse in progress
                // frame 11: accelerator key -> accelerate (note: PhysX ignores acc during gear shift)
                // frame 20: PhysX switches to reverse and accelerates (but should not)
                // frame 21: accelerator key -> brake (only from here on the accelerator key is
                //                                     assigned to braking)

                if (driveState.currentGear == sReverseGear)
                {
                    if (mAutoReverseInProgress == 0)
                        mControllerParams.accelerator = brake;
                    else
                        mControllerParams.accelerator = 0.0f;
                    mControllerParams.brake0 = accelerator;
                }
                else
                {
                    if (mAutoReverseInProgress == 0)
                        mControllerParams.accelerator = accelerator;
                    else
                        mControllerParams.accelerator = 0.0f;
                    mControllerParams.brake0 = brake;
                }
            }
            else
            {
                CARB_LOG_ERROR("PhysX Vehicle: setAcceleratorAndBrake: fetching drive state for vehicle at path \"%s\" failed.\n",
                    mUsdPrim.GetPath().GetText());
            }
        }
        else
        {
            CARB_ASSERT(cache.driveType == DriveType::eBASIC);

            if (mInReverse)
            {
                mControllerParams.accelerator = brake;
                mControllerParams.brake0 = accelerator;
            }
            else
            {
                mControllerParams.accelerator = accelerator;
                mControllerParams.brake0 = brake;
            }
        }
    }

    mDirtyFlags |= (DirtyFlag::eACCELERATOR | DirtyFlag::eBRAKE0);
}

void VehicleController::setBrake1(const float brake)
{
    mControllerParams.brake1 = brake;
    setDirty(DirtyFlag::eBRAKE1);
}

void VehicleController::setSteer(const float steer)
{
    mControllerParams.steer = steer;
    setDirty(DirtyFlag::eSTEER);
}

void VehicleController::setTargetGear(const int targetGear)
{
    mControllerParams.targetGear = targetGear;
    setDirty(DirtyFlag::eTARGET_GEAR);
}

bool VehicleController::getEnabled() const
{
    const Cache& cache = getCache();

    return cache.enabled;
}

void VehicleController::setEnabled(const bool enabled)
{
    Cache& cache = getCache();
#if CARB_DEBUG
    bool usdEnabled;
    PhysxSchemaPhysxVehicleAPI(mUsdPrim).GetVehicleEnabledAttr().Get(&usdEnabled);
    CARB_ASSERT(enabled == usdEnabled);
    // for now: ensure this only changes due to a change on the USD side
#endif

    CARB_ASSERT(cache.enabled != enabled);  // only call this if it needs to change
    if (!cache.enabled)
    {
        // when enabling, start from clean input
        mDirtyFlags &= ~(DirtyFlag::eDEVICE_INPUT_MASK | DirtyFlag::eCONTROLLER_PARAM_MASK);
    }
    cache.enabled = enabled;
}

void VehicleController::update(float timeStep, const pxr::UsdTimeCode& timeCode)
{
    const Cache& cache = getCache();

    CARB_ASSERT(cache.vehicleControllerId != usdparser::kInvalidObjectId);
    CARB_ASSERT(mUsdPrim.HasAPI<PhysxSchemaPhysxVehicleControllerAPI>());

    const bool checkForAutoReverse = cache.hasAutobox || (cache.driveType == DriveType::eBASIC);

    // note: polling the mouse position here. Subscribing to mouse events did not work. Only a tiny
    //       amount of events (if at all) were sent. Looks like someone is consuming the events without
    //       passing them on.
    if (mVehicleControllerSettings.mMouseEnabled && mMouse)
    {
        CARB_ASSERT(mInput);
        carb::Float2 mousePos = mInput->getMouseCoordsPixel(mMouse);
        onMouseMove(mousePos);
    }

    if (mDirtyFlags)
    {
        if ((mDirtyFlags & DirtyFlag::eDEVICE_INPUT_MASK) && mVehicleControllerSettings.mInputEnabled)
        {
            if (!mFilterSteeringDeviceInput)
            {
                // Initialize the steering filter.
                float steer;
                PhysxSchemaPhysxVehicleControllerAPI vehicleControllerAPI = PhysxSchemaPhysxVehicleControllerAPI(mUsdPrim);
                vehicleControllerAPI.GetSteerAttr().Get(&steer);
                mSteerFilter.reset(steer);

                // This turns on device input filtering.
                mFilterSteeringDeviceInput = true;
            }

            if (mDirtyFlags & DirtyFlag::eGAMEPADINPUT)
            {
                setAcceleratorAndBrake0(cache, mGamepadActionMap["eRightTrigger"], mGamepadActionMap["eLeftTrigger"],
                    checkForAutoReverse);

                // Map the steering inputs into an under-curve to adjust the sensitivity.
                const float steerRight = powf(mGamepadActionMap["eLeftStickRight"], mVehicleControllerSettings.mSteeringSensitivity);
                const float steerLeft = powf(mGamepadActionMap["eLeftStickLeft"], mVehicleControllerSettings.mSteeringSensitivity);
                mLastRawSteerInput = steerLeft - steerRight;
            }
            else
            {
                if (mDirtyFlags & DirtyFlag::eKEYBOARDINPUT)
                {
                    setAcceleratorAndBrake0(cache, mKeyboardActionMap[ACCELERATOR_STRING], mKeyboardActionMap[BRAKE0_STRING],
                        checkForAutoReverse);
                    setBrake1(mKeyboardActionMap[BRAKE1_STRING]);

                    const float steerRight = mKeyboardActionMap[STEER_RIGHT_STRING];
                    const float steerLeft = mKeyboardActionMap[STEER_LEFT_STRING];
                    mLastRawSteerInput = steerLeft - steerRight;
                }

                // order is important! Mouse steering should overwrite keyboard steering
                if (mDirtyFlags & DirtyFlag::eMOUSEINPUT)
                {
                    mLastRawSteerInput = CARB_CLAMP(mMouseInput.x, -1.0f, 1.0f);
                }
            }
        }

        mDirtyFlags &= ~(DirtyFlag::eDEVICE_INPUT_MASK);
    }

    if (mFilterSteeringDeviceInput && mVehicleControllerSettings.mInputEnabled)
    {
        setSteer(mSteerFilter.filter(mLastRawSteerInput, timeStep));
    }

    if (checkForAutoReverse && mVehicleControllerSettings.mAutoReverseEnabled)
    {
        // Determine when to switch into reverse gear.
        updateReverse(cache, timeStep);
    }

    writeUsdControls(cache);
}

void VehicleController::updateReverse(const Cache& cache, float timeStep)
{
    static const float sReverseTime = 0.5f;
    static const float sReverseSpeed = 1.0f;

    CARB_ASSERT(mVehicleControllerSettings.mAutoReverseEnabled);

    if (mAutoReverseInProgress != 0)
    {
        // ckeck if the pending auto-reverse has taken place and if so, assign brake
        // to acceleration since the inputs are now considered flipped

        CARB_ASSERT(cache.driveType == DriveType::eSTANDARD);

        VehicleDriveState driveState;
        if (gPhysXInterface->getVehicleDriveState(cache.vehicleControllerId, driveState))
        {
            if (((driveState.currentGear == sReverseGear) && (mAutoReverseInProgress == -1)) ||
                ((driveState.currentGear >= sFirstGear) && (mAutoReverseInProgress == 1)))
            {
                mControllerParams.accelerator = mControllerParams.brake0;
                mControllerParams.brake0 = 0.0f;
                mDirtyFlags |= (DirtyFlag::eACCELERATOR | DirtyFlag::eBRAKE0);
                mAutoReverseInProgress = 0;
            }
        }
        else
        {
            CARB_LOG_ERROR("PhysX Vehicle: updateReverse: fetching drive state for vehicle at path \"%s\" failed.\n",
                mUsdPrim.GetPath().GetText());
        }
    }

    ::physx::PxRigidDynamic* dynamicActor = reinterpret_cast<::physx::PxRigidDynamic*>(gPhysXInterface->getPhysXPtrFast(cache.bodyId));

    if (dynamicActor)
    {
        CARB_ASSERT(dynamicActor->getType() == PxActorType::eRIGID_DYNAMIC);

        // Go into reverse by holding the brake, while stopped, for a period of time.
        float speed = dynamicActor->getLinearVelocity().magnitude() * (float)pxr::UsdGeomGetStageMetersPerUnit(gStage);

        // note: using getBrake() is fine as long as this method gets called after accelerator and brake
        //       input have been buffered (using the current reverse state).
        //
        // note: mAutoReverseInProgress makes sure the timer gets increased too such that the actor
        //       sleep state can be checked regularly further below.
        //
        if ((getBrake0() > 0.2f && speed < sReverseSpeed) || (mAutoReverseInProgress != 0))
        {
            mBrakeTimer += timeStep;
        }
        else
        {
            mBrakeTimer = 0.0f;
        }

        if (mBrakeTimer > sReverseTime)
        {
            mBrakeTimer = 0.0f;

            if (cache.driveType == DriveType::eSTANDARD)
            {
                // a pending switch to or from reverse should finish before the next
                // switch can take place. Furthermore, we can get here even if no switch
                // should happen (but to check the sleep state of the actor)

                if (mAutoReverseInProgress == 0)
                {
                    VehicleDriveState driveState;
                    if (gPhysXInterface->getVehicleDriveState(cache.vehicleControllerId, driveState))
                    {
                        if (driveState.currentGear == sReverseGear)
                        {
                            setTargetGear(::physx::vehicle2::PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR);
                            mAutoReverseInProgress = 1;
                        }
                        else
                        {
                            setTargetGear(sReverseGear);
                            mAutoReverseInProgress = -1;
                        }

                        mControllerParams.accelerator = 0.0f;  // see comment in setAcceleratorAndBrake()
                        mDirtyFlags |= (DirtyFlag::eACCELERATOR | DirtyFlag::eBRAKE0);
                    }
                    else
                    {
                        CARB_LOG_ERROR("PhysX Vehicle: updateReverse: fetching drive state for vehicle at path \"%s\" failed.\n",
                            mUsdPrim.GetPath().GetText());
                    }
                }

                // if the vehicle falls asleep, the gear change will not happen and as a consequence,
                // accelerator and brake won't flip and the vehicle will stay asleep
                dynamicActor->wakeUp();
            }
            else
            {
                CARB_ASSERT(cache.driveType == DriveType::eBASIC);

                if (mInReverse)
                {
                    setTargetGear(sFirstGear);
                }
                else
                {
                    setTargetGear(sReverseGear);
                }

                mInReverse = !mInReverse;

                // since drive basic will switch "gear" immediately, accelerator and brake need to get flipped
                // immediately too
                const float accelerator = mControllerParams.accelerator;
                mControllerParams.accelerator = mControllerParams.brake0;
                mControllerParams.brake0 = accelerator;
                mDirtyFlags |= (DirtyFlag::eACCELERATOR | DirtyFlag::eBRAKE0);
            }
        }
    }
    else
    {
        CARB_LOG_ERROR("PhysX Vehicle: updateReverse: could not find rigid body for prim \"%s\".\n",
            mUsdPrim.GetPath().GetText());
    }
}

} // namespace physx
} // namespace omni
