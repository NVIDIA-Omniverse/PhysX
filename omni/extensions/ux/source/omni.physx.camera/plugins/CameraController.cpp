// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>

#include <omni/kit/IAppWindow.h>

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/PhysxUsd.h>

#include <common/foundation/Allocator.h>

#include "CameraController.h"


using namespace ::physx;
using namespace pxr;
using namespace carb::input;


namespace omni
{
namespace physx
{

const char* CameraController::kGamepadCameraControl = PERSISTENT_SETTINGS_PREFIX "/app/omniverse/gamepadCameraControl";


CameraController::CameraController(const pxr::UsdPrim& prim, const pxr::UsdPrim& subjectPrim)
{
    mCameraPrim = prim;
    mCameraPath = prim.GetPath();
    mSubjectPrim = subjectPrim;
    mSubjectPath = subjectPrim.GetPath();

    if (gPhysXInterface)
    {
        mBodyId = gPhysXInterface->getObjectId(mSubjectPath, ePTActor);
    }
    else
    {
        mBodyId = usdparser::kInvalidObjectId;
    }

    mInitialCameraMatrix = ::omni::usd::UsdUtils::getLocalTransformMatrix(mCameraPrim);

    mDirtyFlags = 0;

    mAlwaysUpdate = false;

    mLeft = { 1.0f, 0.0f, 0.0f };
    mUp = { 0.0f, 1.0f, 0.0f };
    mForward = { 0.0f, 0.0f, 1.0f };

    mKeyboard = nullptr;

    mGamepadConnectionEventId = 0;
    mCameraActionMappingSet = nullptr;
    mGamepadActionMap.clear();
    mGamepadSubscriptionMappings.clear();


    // Camera inputs might be added in the future.
    // bindInputs();
}

CameraController::~CameraController()
{
    // Camera inputs might be added in the future.
    // unbindInputs();
}

void CameraController::release(CameraController& camera)
{
    if (camera.mCameraPrim.IsValid())
    {
        // Reset the camera to its initial transform.
        ::omni::usd::UsdUtils::setLocalTransformFromWorldTransformMatrix(camera.mCameraPrim, camera.mInitialCameraMatrix);
    }

    CameraController* pcamera = &camera;
    ICE_PLACEMENT_DELETE(pcamera, CameraController);
}

void CameraController::bindInputs()
{
    carb::input::IInput* input = carb::getCachedInterface<carb::input::IInput>();

    // Create a vehicle controller action mapping set if one does not exist.
    mCameraActionMappingSet = input->getActionMappingSetByPath(kGamepadCameraControl);

    if (mCameraActionMappingSet == nullptr)
    {
        mCameraActionMappingSet = input->createActionMappingSet(kGamepadCameraControl);
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
    }

    // Subscribe to gamepad input.
    mGamepadConnectionEventId = input->subscribeToGamepadConnectionEvents(
        [](const carb::input::GamepadConnectionEvent& gamegamepadEvent, void* userData) {
            CameraController* cameraController = reinterpret_cast<CameraController*>(userData);
            cameraController->onGamepadConnectionEvent(gamegamepadEvent);
        },
        this);
}

void CameraController::unbindInputs()
{
    carb::input::IInput* input = carb::getCachedInterface<carb::input::IInput>();

    // Keyboard
    for (auto& mapping : mKeyboardSubscriptionMap)
    {
        input->unsubscribeToActionEvents(mapping.second);
    }

    mKeyboardSubscriptionMap.clear();
    unbindKeyboardActions();

    // Gamepads
    input->unsubscribeToGamepadConnectionEvents(mGamepadConnectionEventId);
}

void CameraController::bindKeyboardActions()
{
}

void CameraController::unbindKeyboardActions()
{
}

void CameraController::subscribOnKeyboardAction(const char* key, carb::input::KeyboardInput keyboardInput)
{
    carb::input::IInput* input = carb::getCachedInterface<carb::input::IInput>();

    if (mCameraActionMappingSet)
    {
        if (mKeyboardSubscriptionMap.find(key) == mKeyboardSubscriptionMap.end())
        {
            mKeyboardSubscriptionMap[key] = input->subscribeToActionEvents(
                mCameraActionMappingSet, key,
                [](const carb::input::ActionEvent& actionEvent, void* userData) {
                    CameraController* cameraController = reinterpret_cast<CameraController*>(userData);
                    cameraController->setDirty(DirtyFlag::eKEYBOARDINPUT);
                    cameraController->mKeyboardActionMap[actionEvent.action] = actionEvent.value;
                    return false;
                },
                this);

            carb::input::ActionMappingDesc actionKeyDesc;
            actionKeyDesc.deviceType = carb::input::DeviceType::eKeyboard;
            actionKeyDesc.keyboard = mKeyboard;
            actionKeyDesc.keyboardInput = keyboardInput;
            actionKeyDesc.modifiers = 0;
            input->addActionMapping(mCameraActionMappingSet, key, actionKeyDesc);
        }
    }
}

void CameraController::onGamepadConnectionEvent(const carb::input::GamepadConnectionEvent& gamepadEvent)
{
    // Setup gamepad control
    carb::input::IInput* input = carb::getCachedInterface<carb::input::IInput>();

    if (gamepadEvent.type == carb::input::GamepadConnectionEventType::eConnected)
    {
        mGamepadSubscriptionMappings[gamepadEvent.gamepad].clear();
        bindGamepadActions(gamepadEvent.gamepad);
    }
    else if (gamepadEvent.type == carb::input::GamepadConnectionEventType::eDisconnected)
    {
        for (auto& mapping : mGamepadSubscriptionMappings[gamepadEvent.gamepad])
        {
            input->unsubscribeToActionEvents(mapping.second);
        }

        mGamepadSubscriptionMappings[gamepadEvent.gamepad].clear();
        unbindGamepadActions(gamepadEvent.gamepad);
    }
}

void CameraController::subscribOnGamepadAction(carb::input::Gamepad* gamepad,
                                               const char* action,
                                               const carb::input::GamepadInput gamepadInput)
{
    carb::input::IInput* input = carb::getCachedInterface<carb::input::IInput>();

    if (mCameraActionMappingSet)
    {
        if (mGamepadSubscriptionMappings[gamepad].find(action) == mGamepadSubscriptionMappings[gamepad].end())
        {
            mGamepadSubscriptionMappings[gamepad][action] = input->subscribeToActionEvents(
                mCameraActionMappingSet, action,
                [](const carb::input::ActionEvent& actionEvent, void* userData) {
                    CameraController* cameraController = reinterpret_cast<CameraController*>(userData);
                    cameraController->setDirty(DirtyFlag::eGAMEPADINPUT);
                    cameraController->mGamepadActionMap[actionEvent.action] = actionEvent.value;
                    return false;
                },
                this);

            carb::input::ActionMappingDesc actionPadDesc;
            actionPadDesc.deviceType = carb::input::DeviceType::eGamepad;
            actionPadDesc.gamepad = gamepad;
            actionPadDesc.gamepadInput = gamepadInput;
            actionPadDesc.modifiers = 0;

            input->addActionMapping(mCameraActionMappingSet, action, actionPadDesc);
        }
    }
}

void CameraController::bindGamepadActions(carb::input::Gamepad* gamepad)
{
    subscribOnGamepadAction(gamepad, "eLeftStickUp", carb::input::GamepadInput::eLeftStickUp);
    subscribOnGamepadAction(gamepad, "eLeftStickDown", carb::input::GamepadInput::eLeftStickDown);
    subscribOnGamepadAction(gamepad, "eLeftStickLeft", carb::input::GamepadInput::eLeftStickLeft);
    subscribOnGamepadAction(gamepad, "eLeftStickRight", carb::input::GamepadInput::eLeftStickRight);
    subscribOnGamepadAction(gamepad, "eRightStickUp", carb::input::GamepadInput::eRightStickUp);
    subscribOnGamepadAction(gamepad, "eRightStickDown", carb::input::GamepadInput::eRightStickDown);
    subscribOnGamepadAction(gamepad, "eRightStickLeft", carb::input::GamepadInput::eRightStickLeft);
    subscribOnGamepadAction(gamepad, "eRightStickRight", carb::input::GamepadInput::eRightStickRight);
    subscribOnGamepadAction(gamepad, "eRightTrigger", carb::input::GamepadInput::eRightTrigger);
    subscribOnGamepadAction(gamepad, "eLeftTrigger", carb::input::GamepadInput::eLeftTrigger);
}

void CameraController::unbindGamepadActions(carb::input::Gamepad* gamepad)
{
    if (mCameraActionMappingSet != nullptr)
    {
        carb::input::IInput* input = carb::getCachedInterface<carb::input::IInput>();

        input->clearActionMappings(mCameraActionMappingSet, "eLeftStickUp");
        input->clearActionMappings(mCameraActionMappingSet, "eLeftStickDown");
        input->clearActionMappings(mCameraActionMappingSet, "eLeftStickLeft");
        input->clearActionMappings(mCameraActionMappingSet, "eLeftStickRight");
        input->clearActionMappings(mCameraActionMappingSet, "eRightStickUp");
        input->clearActionMappings(mCameraActionMappingSet, "eRightStickDown");
        input->clearActionMappings(mCameraActionMappingSet, "eRightStickLeft");
        input->clearActionMappings(mCameraActionMappingSet, "eRightStickRight");
        input->clearActionMappings(mCameraActionMappingSet, "eRightTrigger");
        input->clearActionMappings(mCameraActionMappingSet, "eLeftTrigger");
    }
}

void CameraController::updateAxes()
{
    pxr::UsdStageRefPtr stage = omni::usd::UsdContext::getContext()->getStage();
    TfToken usdUpAxis = pxr::UsdGeomGetStageUpAxis(stage);

    if (usdUpAxis == TfToken("X"))
    {
        mUp = PxVec3(1.0f, 0.0f, 0.0f);
        mForward = PxVec3(0.0f, 1.0f, 0.0f);
    }
    else if (usdUpAxis == TfToken("Y"))
    {
        mUp = PxVec3(0.0f, 1.0f, 0.0f);
        mForward = PxVec3(0.0f, 0.0f, 1.0f);
    }
    else if (usdUpAxis == TfToken("Z"))
    {
        mUp = PxVec3(0.0f, 0.0f, 1.0f);
        mForward = PxVec3(1.0f, 0.0f, 0.0f);
    }

    mLeft = mUp.cross(mForward);
}

void CameraController::readUsdControls()
{
}

void CameraController::writeUsdControls()
{
}

void CameraController::readUsdSettings(const pxr::UsdTimeCode& timeCode)
{
    if (mCameraPrim.HasAPI<PhysxSchemaPhysxCameraAPI>())
    {
        PhysxSchemaPhysxCameraAPI camera(mCameraPrim);

        camera.GetAlwaysUpdateEnabledAttr().Get(&mAlwaysUpdate, timeCode);
    }
}

void CameraController::preUpdate()
{
    // Read the controls from USD.
    readUsdControls();

    if (mDirtyFlags)
    {
        if (mDirtyFlags & DirtyFlag::eUSD)
        {
            const pxr::UsdTimeCode timeCode = pxr::UsdTimeCode::Default();
            readUsdSettings(timeCode);
        }
        else
        {
            if (mDirtyFlags & DirtyFlag::eGAMEPADINPUT)
            {
            }
            else if (mDirtyFlags & DirtyFlag::eKEYBOARDINPUT)
            {
            }

            // Local inputs overwrite externally provided inputs in USD.
            writeUsdControls();
        }

        mDirtyFlags = 0;
    }
}

void CameraController::stepUpdate(float timeStep)
{
    ::physx::PxTransform cameraTransform = updateTransform(timeStep);

    // Is there an easier way to copy the PhysX transform into the camera? This is nuts.
    double real = cameraTransform.q.w;
    GfVec3d imaginary(cameraTransform.q.x, cameraTransform.q.y, cameraTransform.q.z);
    GfQuaternion quaternion(real, imaginary);
    GfRotation rotation(quaternion);
    GfVec3d translation(cameraTransform.p.x, cameraTransform.p.y, cameraTransform.p.z);

    mCameraMatrix = GfMatrix4d(rotation, translation);
}

void CameraController::postUpdate()
{
    // The implementation of this method looks slow but at the moment we do not make
    // any assumptions about the type of xform operations used (transform matrix,
    // translate & rotate, ...). At least the transform matrix code path seems to
    // avoid some of the more heavy-weight operations.
    ::omni::usd::UsdUtils::setLocalTransformFromWorldTransformMatrix(mCameraPrim, mCameraMatrix);
}

} // namespace physx
} // namespace omni
