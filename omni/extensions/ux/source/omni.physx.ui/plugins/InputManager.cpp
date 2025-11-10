// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "InputManager.h"

#include <carb/logging/Log.h>
#include <carb/windowing/IWindowing.h>
#include <omni/kit/IAppWindow.h>
#include <carb/input/IInput.h>

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>

extern carb::input::IInput* gInput;
extern pxr::UsdStageRefPtr gStage;

using namespace pxr;
using namespace carb::input;

namespace omni
{
namespace physx
{

namespace ui
{

InputManager::InputManager()
    :mDirty(false)
{
}

InputManager::~InputManager()
{
    clear();
}

void InputManager::addRegisteredActions()
{
    ActionMappingSet* actionMappingSet = GetActionMappingSet();
    if (!actionMappingSet)
        return;

    for (auto& actionMapping : mKbActions)
    {
        gInput->addActionMapping(actionMappingSet, actionMapping.first.c_str(), actionMapping.second);
    }

    for (auto& actionMapping : mPadActions)
    {
        gInput->addActionMapping(actionMappingSet, actionMapping.first.c_str(), actionMapping.second);
    }
}

void InputManager::clearRegisteredActions()
{
    ActionMappingSet* actionMappingSet = GetActionMappingSet();

    if (!actionMappingSet)
        return;

    for (auto& actionMapping : mKbActions)
    {
        gInput->clearActionMappings(actionMappingSet, actionMapping.first.c_str());
    }

    for (auto& actionMapping : mPadActions)
    {
        gInput->clearActionMappings(actionMappingSet, actionMapping.first.c_str());
    }
}

void InputManager::disableActions(const std::vector<const char*>& actionsToDisable)
{
    ActionMappingSet* actionMappingSet = GetActionMappingSet();
    if (!actionMappingSet)
        return;

    mDisabledActions.clear();

    for (auto& actionName : actionsToDisable)
    {
        size_t mappingCount = gInput->getActionMappingCount(actionMappingSet, actionName);
        const ActionMappingDesc* mappings = gInput->getActionMappings(actionMappingSet, actionName);
        for (size_t i = 0; i < mappingCount; ++i)
        {
            mDisabledActions.insert({ actionName, { i, mappings[i] } });
        }

        gInput->clearActionMappings(actionMappingSet, actionName);
    }
}

void InputManager::restoreDisabledActions()
{
    ActionMappingSet* actionMappingSet = GetActionMappingSet();
    if (!actionMappingSet)
        return;

    for (auto& actionMapping : mDisabledActions)
    {
        gInput->addActionMapping(actionMappingSet, actionMapping.first.c_str(), actionMapping.second.second);
    }

    mDisabledActions.clear();
}

void InputManager::registerAction(const char* actionName, KeyboardInput input, KeyboardModifierFlags modifiers /*= 0*/)
{
    omni::kit::IAppWindow* appWindow = omni::kit::getDefaultAppWindow();

    ActionMappingDesc actionDesc = {};
    actionDesc.deviceType = carb::input::DeviceType::eKeyboard;
    actionDesc.keyboard = appWindow->getKeyboard();
    actionDesc.keyboardInput = input;
    actionDesc.modifiers = modifiers;
    mKbActions.insert({ actionName, actionDesc });

    mDirty = true;
}

void InputManager::unregisterAction(const char* actionName)
{
    mKbActions.erase(actionName);
    mPadActions.erase(actionName);

    mDirty = true;
}

void InputManager::clear()
{
    clearRegisteredActions();

    mKbActions.clear();
    mPadActions.clear();

    restoreDisabledActions();

    mDirty = true;
}

void InputManager::update(bool isPlaying)
{
    if (mDirty && isPlaying)
    {
        onStop();
        onResume();
        mDirty = false;
    }
}

void InputManager::onResume()
{
    useActions(true);
    mDirty = false;
}

void InputManager::onStop()
{
    useActions(false);
}

void InputManager::onPause()
{
    useActions(false);
}

ActionMappingSet* InputManager::GetActionMappingSet()
{
    omni::kit::IAppWindow* appWindow = omni::kit::getDefaultAppWindow();
    return appWindow ? gInput->getActionMappingSetByPath(appWindow->getActionMappingSetPath()) : nullptr;
}

void InputManager::registerAction(const char* actionName, GamepadInput input, size_t gamepad_index)
{
    omni::kit::IAppWindow* appWindow = omni::kit::getDefaultAppWindow();

    ActionMappingDesc actionDesc = {};
    actionDesc.deviceType = carb::input::DeviceType::eGamepad;
    actionDesc.gamepad = appWindow->getGamepad(gamepad_index);
    actionDesc.gamepadInput = input;
    actionDesc.modifiers = 0;
    mPadActions.insert({ actionName, actionDesc });
}

bool InputManager::getConflictingMappings(const char* actionName, std::vector<size_t>& conflicts)
{
    ActionMappingSet* actionMappingSet = GetActionMappingSet();
    if (!actionMappingSet)
        return false;

    conflicts.clear();

    size_t mappingCount = gInput->getActionMappingCount(actionMappingSet, actionName);
    const ActionMappingDesc* mappings = gInput->getActionMappings(actionMappingSet, actionName);
    for (size_t i = 0; i < mappingCount; ++i)
    {
        const carb::input::ActionMappingDesc& mapping = mappings[i];
        switch (mapping.deviceType)
        {
        case DeviceType::eKeyboard:
            for (auto& activeKbAction : mKbActions)
            {
                const carb::input::ActionMappingDesc& kbDesc = activeKbAction.second;
                if (kbDesc.keyboardInput == mapping.keyboardInput && kbDesc.modifiers == mapping.modifiers)
                {
                    conflicts.push_back(i);
                }
            }
            break;
        case DeviceType::eGamepad:
            for (auto& activePadAction : mPadActions)
            {
                if (activePadAction.second.gamepadInput == mapping.gamepadInput)
                {
                    conflicts.push_back(i);
                }
            }
            break;
        default: break;
        }
    }

    return !conflicts.empty();
}

void InputManager::useActions(bool enable)
{
    if (enable)
    {
        disableConflictingActions();
        addRegisteredActions();
    }
    else
    {
        clearRegisteredActions();
        restoreDisabledActions();
    }
}

void InputManager::disableConflictingActions()
{
    ActionMappingSet* actionMappingSet = GetActionMappingSet();
    if (!actionMappingSet)
        return;

    size_t actionCount = gInput->getActionCount(actionMappingSet);
    const char* const* actionNames = gInput->getActions(actionMappingSet);

    std::unordered_map<const char*, std::vector<size_t>> mappingsToDisable;
    std::vector<size_t> conflicts;

    for (size_t i = 0; i < actionCount; ++i)
    {
        const char* actionName = actionNames[i];
        if (getConflictingMappings(actionName, conflicts))
        {
            mappingsToDisable.insert({ actionName, conflicts });
        }
    }

    mDisabledActions.clear();

    for (auto& mappingToDisable : mappingsToDisable)
    {
        const char* actionName = mappingToDisable.first;
        std::vector<size_t>& conflicts = mappingToDisable.second;
        size_t mappingCount = gInput->getActionMappingCount(actionMappingSet, actionName);
        const ActionMappingDesc* mappings = gInput->getActionMappings(actionMappingSet, actionName);

        for (auto iter = conflicts.rbegin(); iter != conflicts.rend(); ++iter)
        {
            mDisabledActions.insert({ actionName, { *iter, mappings[*iter] } });
            gInput->removeActionMapping(actionMappingSet, actionName, *iter);
        }
    }
}

} // namespace ui
} // namespace physx
} // namespace omni
