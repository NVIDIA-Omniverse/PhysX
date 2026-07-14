// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/dictionary/IDictionary.h>

#include <omni/Function.h>

namespace omni
{
namespace physics
{

/// Debug data item type enumeration
///
/// Each type specifies the expected format of the associated value.
enum class DebugDataItemType : int
{
    /// Single floating-point value
    /// C++: Store as double using setFloat64()
    /// Python: float
    eFloat,

    /// 3D vector (typically for directions, velocities)
    /// C++: Store as array of 3 doubles using setFloat64Array()
    /// Python: tuple of 3 floats
    eVector,

    /// 3D point (typically for positions in space)
    /// C++: Store as array of 3 doubles using setFloat64Array()
    /// Python: tuple of 3 floats
    ePoint,

    /// Quaternion rotation (x, y, z, w)
    /// C++: Store as array of 4 doubles using setFloat64Array()
    /// Python: tuple of 4 floats
    eQuaternion,

    /// String value
    /// C++: Store as const char* using setString()
    /// Python: str
    eString,

    /// Boolean value
    /// C++: Store as bool using setBool()
    /// Python: bool
    eBool,

    /// Integer value
    /// C++: Store as int32_t using setInt()
    /// Python: int
    eInt,

    /// Undefined or unsupported type
    /// Use this as a default value when the type is unknown
    eUndefined,
};

// Disable the reset on stop
//\param[in] disable Disable/enable the reset on stop
using DisableResetOnStopFn = std::function<void(bool disable)>;

// Get the current state of the reset on stop
//\return Disable/enable for the reset on stop
using IsDisabledResetOnStopFn = std::function<bool()>;

// Handle the raycast request from the interaction system
//\param[in] orig Start position of the raycast
//\param[in] dir Direction of the raycast
//\param[in] input Whether the input control is set or reset (e.g. mouse down)
using HandleRaycastFn = std::function<void(const float* orig, const float* dir, bool input)>;

/// Get simulation debug data for a prim
///
/// \param[in] primPath The prim path as a string
/// \return Carbonite dictionary of simulation debug data for the prim.
///
/// The returned dictionary should contain debug data items as key-value pairs where each key
/// is a string (e.g., "Position", "Velocity") and each value is a dictionary containing:
/// - **"type"** (int): The type of the debug data item (one of DebugDataItemType).
/// - **"doc"** (string): Documentation string describing what this debug item represents
/// - **"value"**: The actual debug value. The expected format depends on the type (see DebugDataItemType enum
/// documentation for the expected value format)
using GetPrimDebugDataFn = std::function<carb::dictionary::Item*(const char* primPath)>;

struct InteractionFns
{
    InteractionFns()
        : disableResetOnStop(nullptr), isDisabledResetOnStop(nullptr), handleRaycast(nullptr), getPrimDebugData(nullptr)
    {
    }

    DisableResetOnStopFn disableResetOnStop;
    IsDisabledResetOnStopFn isDisabledResetOnStop;
    HandleRaycastFn handleRaycast;
    GetPrimDebugDataFn getPrimDebugData;
};

} // namespace physics
} // namespace omni
