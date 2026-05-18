// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/logging/Log.h>

// If not zero, append file and line to log output
#define LOG_FILE_AND_LINE 0

// Condition test, logs expression on fail and returns a value (if _fail_ret_val is defined)
// Optional end parameter is the logging level, default is carb::logging::kLevelError
#define CHECK_RETURN_VALUE_ON_FAIL(_expr, _fail_ret_val, ...)                                                          \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(_expr))                                                                                                  \
        {                                                                                                              \
            if (LOG_FILE_AND_LINE)                                                                                     \
                CARB_LOG(loggingLevel(__VA_ARGS__), #_expr " is false. (%s: %d)", __FILE__, __LINE__);                 \
            else                                                                                                       \
                CARB_LOG(loggingLevel(__VA_ARGS__), #_expr " is false.");                                              \
            return _fail_ret_val;                                                                                      \
        }                                                                                                              \
    } while (false)

// Various wrappers for CHECK_RETURN_VALUE_ON_FAIL that return different values (or nothing)
// Optional end parameter is the logging level, default is carb::logging::kLevelError
#define CHECK_RETURN_ON_FAIL(_expr, ...) CHECK_RETURN_VALUE_ON_FAIL(_expr, , __VA_ARGS__)
#define CHECK_RETURN_NULL_ON_FAIL(_expr, ...) CHECK_RETURN_VALUE_ON_FAIL(_expr, nullptr, __VA_ARGS__)
#define CHECK_RETURN_FALSE_ON_FAIL(_expr, ...) CHECK_RETURN_VALUE_ON_FAIL(_expr, false, __VA_ARGS__)


namespace omni
{
namespace physx
{

// Utility to change the logging level in the macros above
static constexpr int32_t loggingLevel(int32_t level = carb::logging::kLevelError)
{
    return level;
}

} // namespace physx
} // namespace omni
