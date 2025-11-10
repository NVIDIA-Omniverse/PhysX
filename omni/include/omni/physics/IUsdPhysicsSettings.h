// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <omni/kit/SettingsUtils.h>

#define PHYSICS_SETTINGS_PREFIX "/physics"
#define DEFAULT_SETTING_PREFIX "/defaults"

#define DEFINE_PHYSICS_SETTING(name, path)                                                                             \
    static constexpr char name[] = PHYSICS_SETTINGS_PREFIX path;                                                       \
    static constexpr char name##Default[] = DEFAULT_SETTING_PREFIX PHYSICS_SETTINGS_PREFIX path;

#define DEFINE_PERSISTENT_PHYSICS_SETTING(name, path)                                                                  \
    static constexpr char name[] = PERSISTENT_SETTINGS_PREFIX PHYSICS_SETTINGS_PREFIX path;                            \
    static constexpr char name##Default[] =                                                                            \
        DEFAULT_SETTING_PREFIX PERSISTENT_SETTINGS_PREFIX PHYSICS_SETTINGS_PREFIX path;

#define DEFINE_EXT_SETTING(name, path)                                                                                 \
    static constexpr char name[] = path;                                                                               \
    static constexpr char name##Default[] = DEFAULT_SETTING_PREFIX path;


namespace omni
{

namespace physics
{

/** \addtogroup Settings
 *  @{
 */

/// (bool) Enables display of viewport icons and visual authoring for joints.
DEFINE_PERSISTENT_PHYSICS_SETTING(kSettingDisplayJoints, "/visualizationDisplayJoints");

/// \ingroup private
DEFINE_EXT_SETTING(kSettingJointBodyTransformCheckTolerance, "/simulation/jointBodyTransformCheckTolerance")

/** @}*/

} // namespace physics
} // namespace omni
