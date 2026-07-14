// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "Interaction.h"
#include <cstring>

namespace omni
{
namespace physics
{

// Inline implementation of AddDebugDataItem
inline carb::dictionary::Item* AddDebugDataItemBase(carb::dictionary::IDictionary* iDictionary,
                             carb::dictionary::Item* dictionary,
                             const char* name,
                             const char* docString,
                             DebugDataItemType type)
{
    carb::dictionary::Item* item = iDictionary->makeDictionaryAtPath(dictionary, name);
    iDictionary->makeAtPath(item, "type", static_cast<int32_t>(type));
    iDictionary->makeAtPath(item, "doc", docString);
    return item;
}

inline void AddDebugDataItemDouble(carb::dictionary::IDictionary* iDictionary,
    carb::dictionary::Item* dictionary,
    const char* name,
    const char* docString,
    double value)
{
    carb::dictionary::Item* item = AddDebugDataItemBase(iDictionary, dictionary, name, docString, DebugDataItemType::eFloat);
    iDictionary->makeAtPath(item, "value", value);
}

inline void AddDebugDataItemFloat(carb::dictionary::IDictionary* iDictionary,
                                  carb::dictionary::Item* dictionary,
                                  const char* name,
                                  const char* docString,
                                  float value)
{
    carb::dictionary::Item* item = AddDebugDataItemBase(iDictionary, dictionary, name, docString, DebugDataItemType::eFloat);
    iDictionary->makeAtPath(item, "value", value);
}

inline void AddDebugDataItemVector(carb::dictionary::IDictionary* iDictionary,
                                   carb::dictionary::Item* dictionary,
                                   const char* name,
                                   const char* docString,
                                   carb::Double3 value)
{
    carb::dictionary::Item* item = AddDebugDataItemBase(iDictionary, dictionary, name, docString, DebugDataItemType::eVector);
    iDictionary->makeAtPath(item, "value", value);
}

inline void AddDebugDataItemVector(carb::dictionary::IDictionary* iDictionary,
    carb::dictionary::Item* dictionary,
    const char* name,
    const char* docString,
    carb::Float3 value)
{
    carb::dictionary::Item* item = AddDebugDataItemBase(iDictionary, dictionary, name, docString, DebugDataItemType::eVector);
    iDictionary->makeAtPath(item, "value", value);
}

inline void AddDebugDataItemPoint(carb::dictionary::IDictionary* iDictionary,
                                  carb::dictionary::Item* dictionary,
                                  const char* name,
                                  const char* docString,
                                  carb::Double3 value)
{
    carb::dictionary::Item* item = AddDebugDataItemBase(iDictionary, dictionary, name, docString, DebugDataItemType::ePoint);
    iDictionary->makeAtPath(item, "value", value);
}

inline void AddDebugDataItemPoint(carb::dictionary::IDictionary* iDictionary,
    carb::dictionary::Item* dictionary,
    const char* name,
    const char* docString,
    carb::Float3 value)
{
    carb::dictionary::Item* item = AddDebugDataItemBase(iDictionary, dictionary, name, docString, DebugDataItemType::ePoint);
    iDictionary->makeAtPath(item, "value", value);
}


inline void AddDebugDataItemQuaternion(carb::dictionary::IDictionary* iDictionary,
                                       carb::dictionary::Item* dictionary,
                                       const char* name,
                                       const char* docString,
                                       carb::Double4 value)
{
    carb::dictionary::Item* item = AddDebugDataItemBase(iDictionary, dictionary, name, docString, DebugDataItemType::eQuaternion);
    iDictionary->makeAtPath(item, "value", value);
}

inline void AddDebugDataItemQuaternion(carb::dictionary::IDictionary* iDictionary,
    carb::dictionary::Item* dictionary,
    const char* name,
    const char* docString,
    carb::Float4 value)
{
    carb::dictionary::Item* item = AddDebugDataItemBase(iDictionary, dictionary, name, docString, DebugDataItemType::eQuaternion);
    iDictionary->makeAtPath(item, "value", value);
}

inline void AddDebugDataItemString(carb::dictionary::IDictionary* iDictionary,
                                   carb::dictionary::Item* dictionary,
                                   const char* name,
                                   const char* docString,
                                   const char* value)
{
    carb::dictionary::Item* item = AddDebugDataItemBase(iDictionary, dictionary, name, docString, DebugDataItemType::eString);
    iDictionary->makeAtPath(item, "value", value);
}

inline void AddDebugDataItemBool(carb::dictionary::IDictionary* iDictionary,
                                 carb::dictionary::Item* dictionary,
                                 const char* name,
                                 const char* docString,
                                 bool value)
{
    carb::dictionary::Item* item = AddDebugDataItemBase(iDictionary, dictionary, name, docString, DebugDataItemType::eBool);
    iDictionary->makeAtPath(item, "value", value);
}

inline void AddDebugDataItemInt(carb::dictionary::IDictionary* iDictionary,
                                carb::dictionary::Item* dictionary,
                                const char* name,
                                const char* docString,
                                int32_t value)
{
    carb::dictionary::Item* item = AddDebugDataItemBase(iDictionary, dictionary, name, docString, DebugDataItemType::eInt);
    iDictionary->makeAtPath(item, "value", value);
}
} // namespace physics
} // namespace omni
