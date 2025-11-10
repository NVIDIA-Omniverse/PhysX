// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <omni/physx/IPhysxSettings.h>

#include <unordered_map>

namespace omni
{
namespace physx
{

class PhysXSettings
{
    using TBoolDefaults = std::unordered_map<const char*, bool>;
    using TIntDefaults = std::unordered_map<const char*, int32_t>;
    using TInt64Defaults = std::unordered_map<const char*, int64_t>;
    using TFloatDefaults = std::unordered_map<const char*, float>;
    using TStringDefaults = std::unordered_map<const char*, const char*>;
    using TStringArrayDefaults = std::unordered_map<const char*, std::pair<const char* const*, size_t>>;

public:
    PhysXSettings(TBoolDefaults boolDefaults,
                  TIntDefaults intDefaults,
                  TInt64Defaults int64Defaults,
                  TFloatDefaults floatDefaults,
                  TStringDefaults stringDefaults,
                  TStringArrayDefaults stringArrayDefaults);

    static PhysXSettings& getInstance();

    void setDefaults();
    void resetAllSettings();
    void resetSettingsInPreferences();
    void resetSettingsInStage();
    void resetSettingAtPath(const char* path);

private:
    void resetSettingsSection(const char* path);

private:
    TBoolDefaults mBoolDefaults;
    TIntDefaults mIntDefaults;
    TInt64Defaults mInt64Defaults;
    TFloatDefaults mFloatDefaults;
    TStringDefaults mStringDefaults;
    TStringArrayDefaults mStringArrayDefaults;
};

} // namespace physx
} // namespace omni
