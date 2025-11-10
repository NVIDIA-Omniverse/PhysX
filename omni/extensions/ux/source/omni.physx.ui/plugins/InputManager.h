// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/input/InputTypes.h>

#include <map>
#include <string>

namespace carb
{
namespace input
{
class ActionMappingSet;
}
} // namespace carb

namespace omni
{
namespace physx
{
namespace ui
{
class InputManager
{
public:
    virtual void registerAction(const char* actionName,
                                carb::input::KeyboardInput input,
                                carb::input::KeyboardModifierFlags modifiers = 0);
    virtual void registerAction(const char* actionName, carb::input::GamepadInput input, size_t gamepad_index = 0);
    virtual void unregisterAction(const char* actionName);
    virtual void clear();

public:
    InputManager();
    virtual ~InputManager();

    void update(bool isPlaying);
    void onResume();
    void onStop();
    void onPause();

private:
    void addRegisteredActions();
    void clearRegisteredActions();

    void disableConflictingActions();
    void disableActions(const std::vector<const char*>& actionsToDisable);
    void restoreDisabledActions();

    carb::input::ActionMappingSet* GetActionMappingSet();
    bool getConflictingMappings(const char* actionName, std::vector<size_t>& conflicts);
    void useActions(bool enable);

private:
    using TDisabledMap = std::unordered_map<std::string, std::pair<size_t, carb::input::ActionMappingDesc>>;
    using TActionDescMap = std::unordered_map<std::string, carb::input::ActionMappingDesc>;

    TDisabledMap mDisabledActions;
    TActionDescMap mKbActions;
    TActionDescMap mPadActions;
    bool mDirty;
};
} // namespace ui
} // namespace physx
} // namespace omni
